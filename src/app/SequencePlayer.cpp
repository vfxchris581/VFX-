// Sequence Player: plays rendered image sequences inside the app, like Houdini's MPlay.
// Frames are decoded ahead of the playhead on background threads (core/ImageSequence) and shown
// through one texture; playback runs at the chosen frame rate and waits for frames still loading.
#include "App.h"
#include "render/GL.h"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>

namespace fs = std::filesystem;

namespace hd {

namespace {
constexpr size_t kCacheBudget = size_t(1536) << 20; // 1.5 GB of decoded frames

const char* kChannels[] = {"RGB", "Red", "Green", "Blue", "Alpha", "Luminance"};
const char* kLoopModes[] = {"Loop", "Play Once", "Ping-Pong"};
} // namespace

bool App::openSequence(const std::string& anyFrame, bool focus) {
    SequenceInfo seq;
    std::string err;
    if (!findSequence(anyFrame, seq, err)) {
        player.error = err;
        status("Sequence Player: " + err, true);
        return false;
    }
    if (!seqCache_) seqCache_ = std::make_unique<SequenceCache>();
    seqCache_->open(seq, kCacheBudget, player.exposure);
    player.index = 0;
    player.dir = 1;
    player.accum = 0;
    player.playing = focus; // opened by the user: start playing straight away
    player.buffering = false;
    player.zoom = 0;
    player.panX = player.panY = 0;
    player.fps = graph.fps > 0 ? graph.fps : 24;
    player.texFrame = nullptr;
    player.error.clear();
    player.realFps = 0;
    player.fpsFrames = 0;
    player.fpsClock = ImGui::GetTime();
    showSequencePlayer = true;
    if (focus) player.focusNext = true;
    status("Sequence Player: " + seq.label + " (" + std::to_string(seq.size()) + " frames)");
    return true;
}

void App::startMovieExport(const std::string& path) {
    if (movieJob_ || !seqCache_ || seqCache_->sequence().size() == 0) return;
    auto job = std::make_unique<MovieJob>();
    job->path = path;
    job->total = int(seqCache_->sequence().size());
    MovieOptions mo;
    mo.fps = player.fps;
    mo.qp = movieQpForQuality(movieQuality);
    SequenceInfo seq = seqCache_->sequence(); // the thread's own copy
    MovieJob* j = job.get();
    job->thread = std::thread([j, seq, mo] {
        std::string err;
        j->ok = writeMovie(seq, j->path, mo, [j](int done, int) { j->done = done; return !j->cancel; }, err);
        j->error = err;
        j->finished = true;
    });
    movieJob_ = std::move(job);
    status("Writing " + path + "...");
}

void App::pollMovieJob() {
    if (!movieJob_ || !movieJob_->finished) return;
    movieJob_->thread.join();
    std::error_code ec;
    if (movieJob_->ok) {
        double mb = double(fs::file_size(movieJob_->path, ec)) / (1024.0 * 1024.0);
        char size[32];
        std::snprintf(size, sizeof size, "%.1f MB", ec ? 0.0 : mb);
        status("Saved movie " + movieJob_->path + " (" + std::to_string(int(movieJob_->total)) + " frames, " + size + ")");
    } else status(movieJob_->error == "Cancelled" ? "Movie cancelled" : "Movie failed: " + movieJob_->error, movieJob_->error != "Cancelled");
    movieJob_.reset();
}

// Manual stepping always wraps around.
void App::playerStep(int frames) {
    if (!seqCache_ || seqCache_->sequence().size() == 0) return;
    const int n = int(seqCache_->sequence().size());
    player.index = ((player.index + frames) % n + n) % n;
}

void App::drawSequencePlayer() {
    // a new window goes in with the Render View the first time (layouts saved before it existed)
    if (ImGuiWindow* rv = ImGui::FindWindowByName("Render View"); rv && rv->DockId)
        ImGui::SetNextWindowDockID(rv->DockId, ImGuiCond_FirstUseEver);
    if (player.focusNext) { ImGui::SetNextWindowFocus(); player.focusNext = false; }
    if (!ImGui::Begin("Sequence Player", &showSequencePlayer, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
        ImGui::End();
        return;
    }
    ImGuiIO& io = ImGui::GetIO();
    const SequenceInfo* seq = seqCache_ ? &seqCache_->sequence() : nullptr;
    const int n = seq ? int(seq->size()) : 0;

    // ---- toolbar
    if (ImGui::Button("Open...")) openFileDialog(FileDialogMode::OpenSequence);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Pick any frame of a numbered sequence (name.0001.png, .exr or .hdr).\nYou can also drop an image file onto the window.");
    ImGui::SameLine();
    ImGui::BeginDisabled(player.lastRendered.empty());
    if (ImGui::Button("Last Render")) openSequence(player.lastRendered);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip(player.lastRendered.empty() ? "Render a sequence first (Render View > Render Sequence...)" : "%s", player.lastRendered.c_str());
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (movieJob_) {
        // export running: progress and cancel
        const int done = movieJob_->done, total = std::max(1, int(movieJob_->total));
        char buf[64];
        std::snprintf(buf, sizeof buf, "Movie %d / %d", done, total);
        ImGui::ProgressBar(float(done) / float(total), ImVec2(150, 0), buf);
        ImGui::SameLine();
        if (ImGui::Button("Cancel##movie")) movieJob_->cancel = true;
    } else {
        ImGui::BeginDisabled(n == 0);
        if (ImGui::Button("Save Movie...")) openFileDialog(FileDialogMode::SaveMovie);
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Write the sequence as an H.264 movie (.mp4 or .mov) at the player's frame rate (%.0f fps)", double(player.fps));
        ImGui::EndDisabled();
    }
    if (n > 0) {
        ImGui::SameLine();
        ImGui::TextUnformatted(seq->label.c_str());
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", fs::path(seq->files[0]).parent_path().string().c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("frames %d-%d (%d)", seq->frames.front(), seq->frames.back(), n);
        if (player.texW > 0) { ImGui::SameLine(); ImGui::TextDisabled("%d x %d", player.texW, player.texH); }
    }

    if (n == 0) {
        ImVec2 avail = ImGui::GetContentRegionAvail();
        ImGui::Dummy(ImVec2(0, avail.y * 0.4f));
        const char* hint = player.error.empty() ? "Open a rendered image sequence to play it here (or render one: Render View > Render Sequence...)."
                                                : player.error.c_str();
        ImGui::SetCursorPosX(std::max(8.f, (avail.x - ImGui::CalcTextSize(hint).x) * 0.5f));
        ImGui::TextDisabled("%s", hint);
        ImGui::End();
        return;
    }
    player.index = std::clamp(player.index, 0, n - 1);

    // ---- playback
    if (player.playing) {
        player.accum = std::min(player.accum + double(io.DeltaTime) * double(player.fps), 2.0); // no bursts after a stall
        player.buffering = false;
        while (player.accum >= 1) {
            int next = player.index + player.dir;
            if (next < 0 || next >= n) {
                if (player.loop == 0) next = (next + n) % n;
                else if (player.loop == 2 && n > 1) { player.dir = -player.dir; next = player.index + player.dir; }
                else { player.playing = false; player.accum = 0; break; }
            }
            if (!seqCache_->loaded(next)) { player.buffering = true; player.accum = std::min(player.accum, 1.0); break; }
            player.index = next;
            player.accum -= 1;
            ++player.fpsFrames;
        }
        double now = ImGui::GetTime();
        if (now - player.fpsClock >= 0.5) {
            player.realFps = float(player.fpsFrames / (now - player.fpsClock));
            player.fpsFrames = 0;
            player.fpsClock = now;
        }
    } else {
        player.accum = 0;
        player.buffering = false;
        player.realFps = 0;
        player.fpsFrames = 0;
        player.fpsClock = ImGui::GetTime();
    }
    seqCache_->setPlayhead(player.index, player.playing ? player.dir : 1);
    seqCache_->setExposure(player.exposure);

    // ---- texture for the current frame (the previous one stays up while this one loads)
    auto fr = seqCache_->frame(player.index);
    if (fr && fr->w > 0 && (fr.get() != player.texFrame || player.channel != player.texChannel)) {
        std::vector<uint8_t> tmp;
        const uint8_t* data = fr->rgba.data();
        if (player.channel != 0) {
            tmp.resize(fr->rgba.size());
            for (size_t i = 0; i < fr->rgba.size(); i += 4) {
                const uint8_t* s = &fr->rgba[i];
                uint8_t v = player.channel == 5 ? uint8_t((s[0] * 54 + s[1] * 183 + s[2] * 19) >> 8) : s[std::min(player.channel - 1, 3)];
                tmp[i] = tmp[i + 1] = tmp[i + 2] = v;
                tmp[i + 3] = 255;
            }
            data = tmp.data();
        }
        if (!player.tex) glGenTextures(1, &player.tex);
        glBindTexture(GL_TEXTURE_2D, player.tex);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, fr->w, fr->h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glBindTexture(GL_TEXTURE_2D, 0);
        player.texW = fr->w;
        player.texH = fr->h;
        player.texFrame = fr.get();
        player.texChannel = player.channel;
    }

    // ---- image area
    const float controlsH = ImGui::GetFrameHeightWithSpacing() * 2 + 10;
    ImVec2 avail = ImGui::GetContentRegionAvail();
    ImVec2 area(avail.x, std::max(40.f, avail.y - controlsH));
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##seqimage", area, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonMiddle);
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + area.x, p0.y + area.y), IM_COL32(24, 24, 26, 255));
    float scale = 1;
    ImVec2 imgMin(0, 0);
    if (player.tex && player.texW > 0) {
        const float fit = std::min(area.x / float(player.texW), area.y / float(player.texH));
        scale = player.zoom > 0 ? player.zoom : fit;
        // zoom about the mouse with the wheel; drag to pan; double-click to fit
        if (hovered && io.MouseWheel != 0) {
            float ns = std::clamp(scale * std::pow(1.15f, io.MouseWheel), 0.05f, 32.f);
            ImVec2 c(p0.x + area.x * 0.5f + player.panX, p0.y + area.y * 0.5f + player.panY);
            ImVec2 m = io.MousePos;
            player.panX += (m.x - c.x) * (1 - ns / scale);
            player.panY += (m.y - c.y) * (1 - ns / scale);
            scale = player.zoom = ns;
        }
        if (ImGui::IsItemActive() && (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 1) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 1))) {
            player.panX += io.MouseDelta.x;
            player.panY += io.MouseDelta.y;
        }
        if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) { player.zoom = 0; player.panX = player.panY = 0; scale = fit; }
        ImVec2 sz(float(player.texW) * scale, float(player.texH) * scale);
        imgMin = ImVec2(std::floor(p0.x + (area.x - sz.x) * 0.5f + player.panX), std::floor(p0.y + (area.y - sz.y) * 0.5f + player.panY));
        // smooth when fitted; zoomed in past 3x, show the pixels
        glBindTexture(GL_TEXTURE_2D, player.tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, scale >= 3 ? GL_NEAREST : GL_LINEAR);
        glBindTexture(GL_TEXTURE_2D, 0);
        dl->PushClipRect(p0, ImVec2(p0.x + area.x, p0.y + area.y), true);
        dl->AddImage((ImTextureID)(intptr_t)player.tex, imgMin, ImVec2(imgMin.x + sz.x, imgMin.y + sz.y));
        dl->PopClipRect();
        // pixel under the mouse
        if (hovered && fr && fr->w == player.texW) {
            int px = int(std::floor((io.MousePos.x - imgMin.x) / scale)), py = int(std::floor((io.MousePos.y - imgMin.y) / scale));
            if (px >= 0 && py >= 0 && px < fr->w && py < fr->h) {
                const uint8_t* c = &fr->rgba[(size_t(py) * size_t(fr->w) + size_t(px)) * 4];
                char info[96];
                std::snprintf(info, sizeof info, "x %d  y %d   R %.3f  G %.3f  B %.3f  A %.3f", px, py, c[0] / 255.f, c[1] / 255.f, c[2] / 255.f, c[3] / 255.f);
                ImVec2 ts = ImGui::CalcTextSize(info);
                ImVec2 at(p0.x + 6, p0.y + area.y - ts.y - 6);
                dl->AddRectFilled(ImVec2(at.x - 4, at.y - 2), ImVec2(at.x + ts.x + 4, at.y + ts.y + 2), IM_COL32(0, 0, 0, 170), 3);
                dl->AddText(at, IM_COL32(230, 230, 230, 255), info);
            }
        }
    }
    {
        // frame number and status in the corner
        char tag[96];
        std::snprintf(tag, sizeof tag, "%d%s", seq->frames[size_t(player.index)],
                      !fr ? "   loading..." : player.buffering ? "   buffering" : (fr->w == 0 ? "   (unreadable)" : ""));
        ImVec2 ts = ImGui::CalcTextSize(tag);
        ImVec2 at(p0.x + area.x - ts.x - 8, p0.y + 6);
        dl->AddRectFilled(ImVec2(at.x - 4, at.y - 2), ImVec2(at.x + ts.x + 4, at.y + ts.y + 2), IM_COL32(0, 0, 0, 170), 3);
        dl->AddText(at, fr && fr->w == 0 ? IM_COL32(240, 120, 110, 255) : IM_COL32(240, 240, 240, 255), tag);
        if (fr && fr->w == 0 && hovered) ImGui::SetTooltip("%s", fr->error.c_str());
    }

    // ---- scrub bar with the cached frames marked
    ImGui::Dummy(ImVec2(0, 2));
    {
        const float h = 14;
        ImVec2 b0 = ImGui::GetCursorScreenPos();
        float w = ImGui::GetContentRegionAvail().x;
        ImGui::InvisibleButton("##seqscrub", ImVec2(w, h));
        if (ImGui::IsItemActive()) {
            float t = std::clamp((io.MousePos.x - b0.x) / std::max(1.f, w), 0.f, 1.f);
            player.index = std::clamp(int(t * float(n)), 0, n - 1);
            player.playing = false;
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Drag to scrub. The light band shows frames loaded in memory.");
        dl->AddRectFilled(b0, ImVec2(b0.x + w, b0.y + h), IM_COL32(40, 40, 44, 255), 2);
        std::vector<char> mask = seqCache_->loadedMask();
        const float cw = w / float(n);
        for (int i = 0; i < n;) {
            if (!mask[size_t(i)]) { ++i; continue; }
            int j = i;
            while (j < n && mask[size_t(j)]) ++j;
            dl->AddRectFilled(ImVec2(b0.x + cw * float(i), b0.y + h - 4), ImVec2(b0.x + cw * float(j), b0.y + h), IM_COL32(90, 170, 110, 255));
            i = j;
        }
        float x = b0.x + cw * (float(player.index) + 0.5f);
        dl->AddRectFilled(ImVec2(x - std::max(1.f, cw * 0.5f), b0.y), ImVec2(x + std::max(1.f, cw * 0.5f), b0.y + h - 4), IM_COL32(80, 150, 240, 255));
    }

    // ---- controls
    if (ImGui::Button("|<")) { player.index = 0; player.playing = false; }
    ImGui::SameLine();
    if (ImGui::Button("<")) { playerStep(-1); player.playing = false; }
    ImGui::SameLine();
    if (ImGui::Button(player.playing && player.dir < 0 ? "Pause##b" : "<<##playback")) {
        if (player.playing && player.dir < 0) player.playing = false;
        else { player.playing = true; player.dir = -1; }
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Play backward");
    ImGui::SameLine();
    if (ImGui::Button(player.playing && player.dir > 0 ? "Pause##f" : "Play")) {
        if (player.playing && player.dir > 0) player.playing = false;
        else {
            if (player.loop == 1 && player.index == n - 1) player.index = 0; // play once from the end: start over
            player.playing = true;
            player.dir = 1;
        }
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Play / pause (Space)");
    ImGui::SameLine();
    if (ImGui::Button(">")) { playerStep(1); player.playing = false; }
    ImGui::SameLine();
    if (ImGui::Button(">|")) { player.index = n - 1; player.playing = false; }
    ImGui::SameLine();
    int fnum = seq->frames[size_t(player.index)];
    ImGui::SetNextItemWidth(70);
    if (ImGui::InputInt("##seqframe", &fnum, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue)) {
        auto it = std::lower_bound(seq->frames.begin(), seq->frames.end(), fnum);
        player.index = std::clamp(int(it - seq->frames.begin()), 0, n - 1);
        player.playing = false;
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Go to frame");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80);
    ImGui::DragFloat("##seqfps", &player.fps, 0.2f, 1, 120, "%.0f fps");
    player.fps = std::clamp(player.fps, 1.f, 240.f);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Playback speed (starts at the scene's frame rate)");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::Combo("##seqloop", &player.loop, kLoopModes, 3);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(95);
    ImGui::Combo("##seqchan", &player.channel, kChannels, 6);
    if (seq->linear) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110);
        ImGui::DragFloat("##seqexp", &player.exposure, 0.05f, -10, 10, "exposure %+.1f");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Exposure in stops for EXR / HDR frames (double-click the image to fit)");
    }
    ImGui::SameLine();
    if (ImGui::Button("Fit")) { player.zoom = 0; player.panX = player.panY = 0; }
    ImGui::SameLine();
    if (ImGui::Button("1:1")) { player.zoom = 1; player.panX = player.panY = 0; }
    ImGui::SameLine();
    {
        std::vector<char> mask = seqCache_->loadedMask();
        int loaded = int(std::count(mask.begin(), mask.end(), 1));
        if (player.playing) ImGui::TextDisabled("%.1f fps  ", player.realFps);
        else ImGui::TextDisabled(" ");
        ImGui::SameLine(0, 0);
        ImGui::TextDisabled("cache %d/%d  %.0f MB", loaded, n, double(seqCache_->bytes()) / (1024.0 * 1024.0));
    }

    // ---- keys while the player has the focus
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) && !io.WantTextInput) {
        if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) {
            if (player.playing) player.playing = false;
            else { if (player.loop == 1 && player.index == n - 1) player.index = 0; player.playing = true; player.dir = 1; }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) { playerStep(1); player.playing = false; }
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) { playerStep(-1); player.playing = false; }
        if (ImGui::IsKeyPressed(ImGuiKey_Home, false)) { player.index = 0; player.playing = false; }
        if (ImGui::IsKeyPressed(ImGuiKey_End, false)) { player.index = n - 1; player.playing = false; }
        if (ImGui::IsKeyPressed(ImGuiKey_F, false)) { player.zoom = 0; player.panX = player.panY = 0; }
    }
    ImGui::End();
}

} // namespace hd

#include "App.h"
#include "render/GL.h"
#include <imgui.h>
#include <algorithm>
#include <cstdint>
#include <cstdio>

namespace hd {

void App::startRender() {
    Graph& vn = viewNet();
    Node* d = vn.displayNode();
    if (!d) { status("Nothing to render: no display node", true); return; }
    vn.cook(d);
    if (!d->error().empty()) { status("Cannot render: " + d->name() + " has an error", true); return; }
    syncCameraToScene();
    RenderSettings rs = graph.render;
    rs.fps = graph.fps;
    rs.frame = graph.frame;
    for (auto& cam : collectCameras(graph, graph.frame)) {
        if (cam.name != rs.cameraName) continue; // rendering through a camera node: its position, lens and resolution
        rs.useEye = true; rs.camEye = cam.eye; rs.camTarget = cam.eye + cam.fwd;
        rs.width = cam.resx; rs.height = cam.resy;
        rs.camFov = cam.fovYDegrees();
        rs.sensorHeight = cam.aperture * float(cam.resy) / float(cam.resx);
        if (cam.dof) { rs.dof = true; rs.fstop = cam.fstop; rs.focusDist = cam.focus; }
    }
    Geometry scene;
    if (sceneGeometry(graph, &vn == &graph ? d : nullptr, true, scene) > 0) rs.ground = false; // the Ground Plane replaces the built-in ground
    if (&vn != &graph) { scene.merge(d->geometry()); } // display inside a subnet: sceneGeometry got only the planes
    tracer_.start(scene, rs, collectLights(graph, graph.frame));
    std::string warn = tracer_.warnings();
    if (!warn.empty()) status("Render: " + warn.substr(0, warn.find('\n')), true);
}

void App::stepSequence() {
    // ---- sequence rendering: one frame at a time
    if (seqActive_ && !tracer_.running()) {
        if (seqWaiting_) {
            uint64_t v = 0;
            std::vector<uint8_t> img; int w = 0, h = 0;
            tracer_.fetch(img, w, h, v);
            std::string path = seqBase_;
            char num[16];
            std::snprintf(num, sizeof num, ".%04d", seqFrame_);
            size_t dot = path.rfind('.');
            path.insert(dot == std::string::npos ? path.size() : dot, num);
            std::string err;
            bool exr = path.size() > 4 && path.compare(path.size() - 4, 4, ".exr") == 0;
            PathTracer::Buffers buf;
            bool ok = exr ? (tracer_.fetchBuffers(buf) && writeExrPasses(path, buf, true, &err)) : writePng(path, img, w, h, &err);
            if (!ok) { status("Sequence stopped: " + err, true); seqActive_ = false; }
            else { ++seqCount_; status("Saved " + path); }
            seqWaiting_ = false;
            ++seqFrame_;
        }
        if (seqActive_) {
            if (seqFrame_ > seqEnd_) {
                seqActive_ = false;
                status("Sequence finished: " + std::to_string(seqCount_) + " frames");
            } else {
                graph.frame = float(seqFrame_);
                startRender();
                seqWaiting_ = tracer_.running();
                if (!seqWaiting_) seqActive_ = false;
            }
        }
    }

}

void App::drawRenderSettings() {
    if (!showRenderSettings) return;
    RenderSettings& rs = graph.render;
    ImGui::SetNextWindowSize(ImVec2(380, 0), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Render Settings", &showRenderSettings, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize)) {
        bool ch = false;
        ImGui::SeparatorText("Output");
        ch |= ImGui::InputInt("Width", &rs.width, 16, 160);
        ch |= ImGui::InputInt("Height", &rs.height, 16, 160);
        ch |= ImGui::InputInt("Samples", &rs.samples, 8, 64);
        rs.width = std::clamp(rs.width, 16, 4096); rs.height = std::clamp(rs.height, 16, 4096); rs.samples = std::clamp(rs.samples, 1, 100000);
        ImGui::SeparatorText("Light");
        ch |= ImGui::SliderFloat("Sun Direction", &rs.sunAzimuth, 0, 360, "%.0f deg");
        ch |= ImGui::SliderFloat("Sun Height", &rs.sunElevation, 2, 90, "%.0f deg");
        ch |= ImGui::SliderFloat("Sun Brightness", &rs.sunIntensity, 0, 10);
        ch |= ImGui::SliderFloat("Sky Brightness", &rs.skyIntensity, 0, 4);
        ch |= ImGui::SliderFloat("Exposure", &rs.exposure, 0.1f, 5);
        ImGui::SeparatorText("Scene");
        ch |= ImGui::Checkbox("Ground Plane", &rs.ground);
        ch |= ImGui::DragFloat("Ground Height", &rs.groundY, 0.01f);
        ch |= ImGui::DragFloat("Curve Thickness", &rs.curveWidth, 0.001f, 0.001f, 1.f, "%.3f");
        ch |= ImGui::SliderInt("Light Bounces", &rs.bounces, 0, 12);
        ImGui::SeparatorText("Camera");
        ch |= ImGui::Checkbox("Depth of Field", &rs.dof);
        ImGui::BeginDisabled(!rs.dof);
        ch |= ImGui::SliderFloat("F-Stop", &rs.fstop, 0.5f, 32.f, "f/%.1f", ImGuiSliderFlags_Logarithmic);
        ch |= ImGui::DragFloat("Focus Distance", &rs.focusDist, 0.02f, 0.01f, 1000.f, "%.2f");
        ImGui::EndDisabled();
        ch |= ImGui::Checkbox("Motion Blur", &rs.motionBlur);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Blurs moving geometry using its v (velocity) attribute");
        ImGui::BeginDisabled(!rs.motionBlur);
        ch |= ImGui::SliderFloat("Shutter (frames)", &rs.shutter, 0.f, 1.f);
        ImGui::EndDisabled();
        ImGui::SeparatorText("Image");
        ch |= ImGui::Checkbox("Denoise Final Image", &rs.denoise);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Smooths the remaining noise when the render finishes (uses the albedo, normal and depth passes)");
        ImGui::TextDisabled("Camera nodes with Depth of Field on use their own\nf-stop and focus. Save as .exr to keep every pass.\nChanges apply to the next render.");
        if (ch) markDirty();
    }
    ImGui::End();
}

void App::drawRenderView() {
    if (!ImGui::Begin("Render View", &showRenderView, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) { ImGui::End(); return; }
    RenderSettings& rs = graph.render;

    // ---- toolbar
    bool busy = tracer_.running();
    if (!busy) {
        if (ImGui::Button("Render")) startRender();
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Path-trace the displayed node from the Scene View camera");
    } else if (ImGui::Button(seqActive_ ? "Stop Sequence" : "Stop")) {
        tracer_.stop();
        if (seqActive_) { seqActive_ = false; status("Sequence stopped after " + std::to_string(seqCount_) + " frames"); }
    }
    ImGui::SameLine();
    static const int sizes[][2] = {{640, 360}, {960, 540}, {1280, 720}, {1920, 1080}, {1080, 1080}, {1080, 1920}};
    char cur[48];
    std::snprintf(cur, sizeof cur, "%d x %d", rs.width, rs.height);
    bool fromCamera = false;
    for (auto& cam : collectCameras(graph, graph.frame))
        if (cam.name == rs.cameraName) { std::snprintf(cur, sizeof cur, "%d x %d (%s)", cam.resx, cam.resy, cam.name.c_str()); fromCamera = true; }
    ImGui::SetNextItemWidth(fromCamera ? 190.f : 120.f);
    ImGui::BeginDisabled(busy);
    if (ImGui::BeginCombo("##size", cur)) {
        if (fromCamera) ImGui::TextDisabled("Size comes from the camera node.\nThese apply when no camera is active:");
        for (auto& sz : sizes) {
            char label[32];
            std::snprintf(label, sizeof label, "%d x %d", sz[0], sz[1]);
            if (ImGui::Selectable(label, rs.width == sz[0] && rs.height == sz[1])) { rs.width = sz[0]; rs.height = sz[1]; markDirty(); }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110);
    if (ImGui::DragInt("##samples", &rs.samples, 1, 1, 4096, "%d samples")) { rs.samples = std::max(1, rs.samples); markDirty(); }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("More samples = less noise, longer render");
    ImGui::SameLine();
    if (ImGui::Button("Settings")) showRenderSettings = true;
    ImGui::SameLine();
    if (ImGui::Button("Save Image...") && !renderImg_.empty()) openFileDialog(FileDialogMode::SaveImage);
    ImGui::SameLine();
    if (ImGui::Button("Render Sequence...")) openFileDialog(FileDialogMode::SaveSequence);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Render frames %d-%d (the timeline range) to numbered PNG or EXR files", graph.frameStart, graph.frameEnd);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (tracer_.samplesTarget() > 0) {
        if (seqActive_) ImGui::Text("Frame %d / %d   %d / %d samples", seqFrame_, seqEnd_, tracer_.samplesDone(), tracer_.samplesTarget());
        else ImGui::TextDisabled("%d / %d samples   %.1fs%s", tracer_.samplesDone(), tracer_.samplesTarget(), tracer_.secondsElapsed(), busy ? "" : "   done");
    }

    // ---- image
    std::vector<uint8_t> img;
    int w = 0, h = 0;
    if (tracer_.fetch(img, w, h, renderVersion_)) {
        if (!renderTex_) glGenTextures(1, &renderTex_);
        glBindTexture(GL_TEXTURE_2D, renderTex_);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glBindTexture(GL_TEXTURE_2D, 0);
        renderImg_.swap(img);
        renderW_ = w; renderH_ = h;
    }
    ImVec2 avail = ImGui::GetContentRegionAvail();
    if (renderTex_ && renderW_ > 0 && avail.x > 8 && avail.y > 8) {
        float s = std::min(avail.x / float(renderW_), avail.y / float(renderH_));
        ImVec2 sz(float(renderW_) * s, float(renderH_) * s);
        ImVec2 pos = ImGui::GetCursorPos();
        ImGui::SetCursorPos(ImVec2(pos.x + (avail.x - sz.x) * 0.5f, pos.y + (avail.y - sz.y) * 0.5f));
        ImGui::Image((ImTextureID)(intptr_t)renderTex_, sz);
    } else {
        ImGui::Dummy(ImVec2(0, avail.y * 0.4f));
        const char* hint = "Press Render to path-trace the displayed node from the Scene View camera.";
        ImGui::SetCursorPosX(std::max(8.f, (avail.x - ImGui::CalcTextSize(hint).x) * 0.5f));
        ImGui::TextDisabled("%s", hint);
    }
    ImGui::End();
}

} // namespace hd

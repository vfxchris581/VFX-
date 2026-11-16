#include "ImageSequence.h"
#include "Image.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <map>

namespace fs = std::filesystem;

namespace hd {

namespace {

// Splits a file name into prefix, frame digits and suffix ("shot.0012.png" -> "shot.", "0012", ".png").
bool splitNumber(const std::string& name, std::string& prefix, std::string& digits, std::string& suffix) {
    size_t dot = name.rfind('.');
    size_t end = dot == std::string::npos || dot == 0 ? name.size() : dot;
    size_t e = end;
    while (e > 0 && !std::isdigit((unsigned char)name[e - 1])) --e; // digits may be followed by e.g. "_beauty"
    if (e == 0) return false;
    size_t b = e;
    while (b > 0 && std::isdigit((unsigned char)name[b - 1])) --b;
    prefix = name.substr(0, b);
    digits = name.substr(b, e - b);
    suffix = name.substr(e);
    return true;
}

std::string lower(std::string s) {
    for (auto& c : s) c = char(std::tolower((unsigned char)c));
    return s;
}

} // namespace

bool findSequence(const std::string& anyFrame, SequenceInfo& out, std::string& error) {
    out = SequenceInfo();
    std::error_code ec;
    fs::path p(anyFrame);
    if (!fs::is_regular_file(p, ec)) { error = "Not a file: " + anyFrame; return false; }
    const std::string ext = lower(p.extension().string());
    if (ext != ".png" && ext != ".exr" && ext != ".hdr") { error = "Not an image (.png, .exr or .hdr): " + anyFrame; return false; }
    out.linear = ext != ".png";
    std::string prefix, digits, suffix;
    const std::string name = p.filename().string();
    if (!splitNumber(name, prefix, digits, suffix)) {
        out.label = name;
        out.frames = {1};
        out.files = {p.string()};
        return true;
    }
    std::map<int, std::string> found;
    for (auto& e : fs::directory_iterator(p.parent_path().empty() ? fs::path(".") : p.parent_path(), ec)) {
        if (!e.is_regular_file(ec)) continue;
        std::string n = e.path().filename().string(), pr, dg, sf;
        if (!splitNumber(n, pr, dg, sf) || pr != prefix || sf != suffix) continue;
        if (dg.size() > 9) continue;
        int f = std::stoi(dg);
        // with several paddings of one number (shot.1 and shot.0001), keep the one like the chosen file
        auto it = found.find(f);
        if (it == found.end() || dg.size() == digits.size()) found[f] = e.path().string();
    }
    if (found.empty()) { error = "No frames found for " + anyFrame; return false; }
    out.label = prefix + std::string(digits.size(), '#') + suffix;
    for (auto& [f, path] : found) { out.frames.push_back(f); out.files.push_back(path); }
    return true;
}

// ------------------------------------------------------------------ cache

SequenceCache::SequenceCache(int threads) {
    if (threads <= 0) {
        unsigned hc = std::thread::hardware_concurrency();
        threads = int(std::clamp(hc / 2, 1u, 4u));
    }
    for (int i = 0; i < threads; ++i) threads_.emplace_back([this] { worker(); });
}

SequenceCache::~SequenceCache() {
    {
        std::lock_guard<std::mutex> lk(m_);
        quit_ = true;
    }
    cv_.notify_all();
    for (auto& t : threads_) t.join();
}

void SequenceCache::open(const SequenceInfo& seq, size_t budgetBytes, float exposure) {
    {
        std::lock_guard<std::mutex> lk(m_);
        ++gen_;
        seq_ = seq;
        frames_.assign(seq.size(), nullptr);
        inFlight_.assign(seq.size(), 0);
        bytes_ = 0;
        frameBytes_ = 0;
        budget_ = std::max<size_t>(budgetBytes, 1);
        playhead_ = 0;
        dir_ = 1;
        exposure_ = exposure;
    }
    cv_.notify_all();
}

void SequenceCache::close() { open(SequenceInfo(), 1, exposure_); }

void SequenceCache::setPlayhead(int index, int direction) {
    {
        std::lock_guard<std::mutex> lk(m_);
        if (index == playhead_ && (direction < 0 ? -1 : 1) == dir_) return;
        playhead_ = index;
        dir_ = direction < 0 ? -1 : 1;
    }
    cv_.notify_all();
}

void SequenceCache::setExposure(float stops) {
    std::lock_guard<std::mutex> lk(m_);
    if (stops == exposure_) return;
    exposure_ = stops;
    if (!seq_.linear) return; // 8-bit images don't use it
    ++gen_;
    frames_.assign(seq_.size(), nullptr);
    inFlight_.assign(seq_.size(), 0);
    bytes_ = 0;
    cv_.notify_all();
}

std::shared_ptr<const SequenceCache::Frame> SequenceCache::frame(int index) const {
    std::lock_guard<std::mutex> lk(m_);
    if (index < 0 || index >= int(frames_.size())) return nullptr;
    return frames_[size_t(index)];
}

bool SequenceCache::loaded(int index) const { return frame(index) != nullptr; }

std::vector<char> SequenceCache::loadedMask() const {
    std::lock_guard<std::mutex> lk(m_);
    std::vector<char> m(frames_.size());
    for (size_t i = 0; i < frames_.size(); ++i) m[i] = frames_[i] != nullptr;
    return m;
}

size_t SequenceCache::bytes() const {
    std::lock_guard<std::mutex> lk(m_);
    return bytes_;
}

bool SequenceCache::waitFor(int index, int timeoutMs) {
    std::unique_lock<std::mutex> lk(m_);
    if (index < 0 || index >= int(frames_.size())) return false;
    return loadedCv_.wait_for(lk, std::chrono::milliseconds(timeoutMs), [&] { return frames_[size_t(index)] != nullptr; });
}

// Distance ahead of the playhead in the playback direction (0 = the playhead's frame).
int SequenceCache::rankLocked(int index) const {
    const int n = int(frames_.size());
    int d = dir_ > 0 ? index - playhead_ : playhead_ - index;
    return ((d % n) + n) % n;
}

int SequenceCache::pickLocked() const {
    const int n = int(frames_.size());
    if (n == 0) return -1;
    // how many frames fit: everything ahead within that window is wanted
    const size_t per = frameBytes_ ? frameBytes_ : 1;
    const int fit = frameBytes_ ? int(std::max<size_t>(1, budget_ / per)) : n;
    const int window = std::min(n, fit);
    for (int k = 0; k < window; ++k) {
        int i = ((dir_ > 0 ? playhead_ + k : playhead_ - k) % n + n) % n;
        if (!frames_[size_t(i)] && !inFlight_[size_t(i)]) return i;
    }
    return -1;
}

void SequenceCache::toDisplay(const Image& img, bool linear, float exposure, Frame& out) {
    out.w = img.w;
    out.h = img.h;
    out.rgba.assign(size_t(img.w) * size_t(img.h) * 4, 255);
    const float gain = std::exp2(exposure);
    auto enc = [&](float v) {
        if (linear) {
            v *= gain;
            v = v <= 0.0031308f ? v * 12.92f : 1.055f * std::pow(std::max(v, 0.f), 1 / 2.4f) - 0.055f;
        }
        return uint8_t(std::clamp(v, 0.f, 1.f) * 255.f + 0.5f);
    };
    const size_t np = size_t(img.w) * size_t(img.h);
    for (size_t i = 0; i < np; ++i) {
        const float* s = &img.px[i * size_t(img.c)];
        uint8_t* d = &out.rgba[i * 4];
        if (img.c >= 3) { d[0] = enc(s[0]); d[1] = enc(s[1]); d[2] = enc(s[2]); }
        else d[0] = d[1] = d[2] = enc(s[0]);
        if (img.c == 4) d[3] = uint8_t(std::clamp(s[3], 0.f, 1.f) * 255.f + 0.5f);
        else if (img.c == 2) d[3] = uint8_t(std::clamp(s[1], 0.f, 1.f) * 255.f + 0.5f);
    }
}

void SequenceCache::worker() {
    std::unique_lock<std::mutex> lk(m_);
    for (;;) {
        int idx = -1;
        cv_.wait(lk, [&] { return quit_ || (idx = pickLocked()) >= 0; });
        if (quit_) return;
        inFlight_[size_t(idx)] = 1;
        const uint64_t gen = gen_;
        const std::string path = seq_.files[size_t(idx)];
        const bool linear = seq_.linear;
        const float exposure = exposure_;
        lk.unlock();

        auto fr = std::make_shared<Frame>();
        Image img;
        std::string err;
        if (readImage(path, img, false, &err)) toDisplay(img, linear, exposure, *fr);
        else fr->error = err.empty() ? "Could not read " + path : err;

        lk.lock();
        if (gen != gen_) continue; // the sequence changed meanwhile
        inFlight_[size_t(idx)] = 0;
        const size_t sz = fr->rgba.size() + 64;
        frameBytes_ = std::max(frameBytes_, sz);
        // make room: drop the frames farthest ahead (that is, just behind the playhead)
        while (bytes_ + sz > budget_) {
            int worst = -1, worstRank = -1;
            for (int i = 0; i < int(frames_.size()); ++i)
                if (frames_[size_t(i)] && rankLocked(i) > worstRank) { worstRank = rankLocked(i); worst = i; }
            if (worst < 0 || worstRank <= rankLocked(idx)) break;
            bytes_ -= frames_[size_t(worst)]->rgba.size() + 64;
            frames_[size_t(worst)] = nullptr;
        }
        if (bytes_ + sz > budget_ && bytes_ > 0) { cv_.notify_all(); continue; } // not worth keeping now
        frames_[size_t(idx)] = std::move(fr);
        bytes_ += sz;
        loadedCv_.notify_all();
        cv_.notify_all();
    }
}

} // namespace hd

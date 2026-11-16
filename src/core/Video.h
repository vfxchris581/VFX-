// Movie files: H.264 video in MP4 or QuickTime MOV, with no external libraries.
//
// The encoder writes Constrained Baseline H.264 (CAVLC): keyframes use 16x16 intra prediction, the
// frames between them use motion-compensated prediction from the previous frame (full-pixel motion
// search) or skip unchanged macroblocks. Each frame is cut into horizontal slices that are encoded
// in parallel. Colours are converted to BT.709 YUV 4:2:0 (video range) and tagged as such.
#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace hd {

struct SequenceInfo;

struct MovieOptions {
    float fps = 24;
    int qp = 23;      // quantiser 0-51: lower = better quality, bigger file (18 high, 23 medium, 28 small)
    int keyint = 0;   // frames between keyframes (0 = two seconds)
    int slices = 0;   // per frame, encoded in parallel (0 = by core count)
    bool deblock = true; // H.264 loop filter (smooths block edges)
};

// Quality presets used by the UI and hdc.
int movieQpForQuality(int preset); // 0 high, 1 medium, 2 small

class MovieWriter {
public:
    MovieWriter();
    ~MovieWriter();
    // The container follows the extension: .mp4, .mov, or .h264 / .264 (raw Annex B stream).
    // Odd sizes are cut down to even ones (4:2:0 needs even sizes).
    bool open(const std::string& path, int width, int height, const MovieOptions& opt, std::string& error);
    // One frame of 8-bit RGBA, top row first, `width` x `height` as given to open().
    bool addFrame(const uint8_t* rgba, std::string& error);
    bool finish(std::string& error);
    int frames() const;
    int width() const;
    int height() const;
    // The decoder's picture for the last frame (Y, then U, then V; cropped to the output size).
    // A conforming H.264 decoder reproduces it exactly.
    const std::vector<uint8_t>& lastReconstruction() const;
    // Raw size in bytes of the coded video so far.
    size_t bytes() const;

private:
    struct Impl;
    std::unique_ptr<Impl> d_;
};

// Encodes an image sequence (any frame size) into a movie. progress(done, total) returns false to cancel.
bool writeMovie(const SequenceInfo& seq, const std::string& path, const MovieOptions& opt,
                const std::function<bool(int, int)>& progress, std::string& error);

} // namespace hd

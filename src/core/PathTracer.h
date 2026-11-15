// Progressive CPU path tracer used by the Render View and `hdc render`.
#pragma once
#include "Geometry.h"
#include "Lights.h"
#include "RenderSettings.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hd {

class PathTracer {
public:
    PathTracer();
    ~PathTracer();
    PathTracer(const PathTracer&) = delete;
    PathTracer& operator=(const PathTracer&) = delete;

    // Starts rendering a snapshot of geo in the background (cancels any render in progress).
    // The camera comes from settings (eye(), camTarget, camFov).
    // With no lights given, the sun and sky from the settings are used.
    void start(const Geometry& geo, const RenderSettings& settings, const std::vector<Light>& lights = {});
    void stop();
    void wait(); // block until the current render finishes
    bool running() const;
    int samplesDone() const;
    int samplesTarget() const;
    double secondsElapsed() const;
    // Copies the latest tonemapped image (RGBA8, top row first) if it is newer than `version`.
    bool fetch(std::vector<uint8_t>& rgba, int& w, int& h, uint64_t& version);

    // Linear render passes (averaged over the samples so far). Exposure is applied to beauty and denoised.
    struct Buffers {
        int w = 0, h = 0, samples = 0;
        std::vector<float> beauty;   // rgb
        std::vector<float> alpha;    // coverage: 1 where a surface or volume was seen
        std::vector<float> albedo;   // rgb, first surface
        std::vector<float> normal;   // xyz, first surface (world space)
        std::vector<float> depth;    // distance from the camera to the first surface (1e10 for none)
        std::vector<float> denoised; // rgb; filled when denoising is on and the render has finished
    };
    bool fetchBuffers(Buffers& out) const;
    // Problems found while building the scene (missing textures, unreadable environment maps...).
    std::string warnings() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Writes an 8-bit RGB PNG from RGBA data (alpha ignored).
bool writePng(const std::string& path, const std::vector<uint8_t>& rgba, int w, int h, std::string* error = nullptr);

// Writes the passes as one OpenEXR: R G B A (denoised colour when present, plus beauty.R/G/B then),
// albedo.R/G/B, N.X/Y/Z and Z. half = 16-bit floats.
bool writeExrPasses(const std::string& path, const PathTracer::Buffers& b, bool half, std::string* error = nullptr);

} // namespace hd

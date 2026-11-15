// Edge-avoiding a-trous wavelet denoiser (Dammertz et al. 2010, with SVGF-style variance guidance).
// Illumination is demodulated by albedo so textures stay sharp, then filtered over five widening
// passes whose weights respect normals, depth and the per-pixel noise level.
#pragma once
#include <vector>

namespace hd {

struct DenoiseInput {
    int w = 0, h = 0;
    const float* color = nullptr;    // rgb, linear
    const float* albedo = nullptr;   // rgb (may be null)
    const float* normal = nullptr;   // xyz (may be null; zero = background)
    const float* depth = nullptr;    // 1 value (may be null)
    const float* variance = nullptr; // luminance variance of the pixel mean (may be null)
};

// Returns the filtered rgb image (w*h*3).
std::vector<float> denoiseImage(const DenoiseInput& in, int passes = 5, float strength = 1.f);

} // namespace hd

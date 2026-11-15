#include "Denoise.h"
#include "Volume.h"
#include <algorithm>
#include <cmath>

namespace hd {

std::vector<float> denoiseImage(const DenoiseInput& in, int passes, float strength) {
    const int W = in.w, H = in.h;
    const size_t n = size_t(W) * size_t(H);
    std::vector<float> out(n * 3, 0.f);
    if (!in.color || n == 0) return out;
    const float eps = 0.01f;
    auto lum = [](const float* c) { return 0.2126f * c[0] + 0.7152f * c[1] + 0.0722f * c[2]; };
    // demodulate
    std::vector<float> illum(n * 3), alb(n * 3, 1.f), var(n, 0.f);
    for (size_t i = 0; i < n; ++i) {
        for (int k = 0; k < 3; ++k) {
            float a = in.albedo ? std::max(in.albedo[i * 3 + size_t(k)], eps) : 1.f;
            alb[i * 3 + size_t(k)] = a;
            illum[i * 3 + size_t(k)] = in.color[i * 3 + size_t(k)] / a;
        }
        float la = in.albedo ? std::max(lum(&alb[i * 3]), eps) : 1.f;
        var[i] = in.variance ? in.variance[i] / (la * la) : 0.f;
    }
    if (!in.variance) {
        // estimate noise from the local spread of luminance (3x3)
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) {
                float s = 0, s2 = 0; int c = 0;
                for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
                    int xx = std::clamp(x + dx, 0, W - 1), yy = std::clamp(y + dy, 0, H - 1);
                    float l = lum(&illum[(size_t(yy) * size_t(W) + size_t(xx)) * 3]);
                    s += l; s2 += l * l; ++c;
                }
                float m = s / float(c);
                var[size_t(y) * size_t(W) + size_t(x)] = std::max(0.f, s2 / float(c) - m * m);
            }
    }
    static const float k[5] = {1.f / 16, 1.f / 4, 3.f / 8, 1.f / 4, 1.f / 16};
    std::vector<float> cur = illum, next(n * 3), nvar(n);
    for (int pass = 0; pass < passes; ++pass) {
        const int step = 1 << pass;
        // blur the variance a little before using it (SVGF)
        std::vector<float> vblur(n);
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) {
                float s = 0, ws = 0;
                for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
                    int xx = x + dx, yy = y + dy;
                    if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
                    float wgt = (dx == 0 ? 0.5f : 0.25f) * (dy == 0 ? 0.5f : 0.25f);
                    s += var[size_t(yy) * size_t(W) + size_t(xx)] * wgt; ws += wgt;
                }
                vblur[size_t(y) * size_t(W) + size_t(x)] = s / ws;
            }
        parallelFor(H, [&](int y0, int y1) {
            for (int y = y0; y < y1; ++y)
                for (int x = 0; x < W; ++x) {
                    const size_t p = size_t(y) * size_t(W) + size_t(x);
                    const float lp = lum(&cur[p * 3]);
                    const float sigL = 4.f * std::sqrt(std::max(vblur[p], 1e-10f)) / std::max(strength, 1e-3f) + 1e-6f;
                    const float* np = in.normal ? &in.normal[p * 3] : nullptr;
                    const bool bgp = np && (np[0] * np[0] + np[1] * np[1] + np[2] * np[2]) < 0.25f;
                    const float zp = in.depth ? in.depth[p] : 0.f;
                    float acc[3] = {0, 0, 0}, wsum = 0, vsum = 0;
                    for (int dy = -2; dy <= 2; ++dy)
                        for (int dx = -2; dx <= 2; ++dx) {
                            int xx = x + dx * step, yy = y + dy * step;
                            if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
                            const size_t q = size_t(yy) * size_t(W) + size_t(xx);
                            float wgt = k[dx + 2] * k[dy + 2];
                            if (q != p) {
                                wgt *= std::exp(-std::fabs(lp - lum(&cur[q * 3])) / sigL);
                                if (np) {
                                    const float* nq = &in.normal[q * 3];
                                    bool bgq = (nq[0] * nq[0] + nq[1] * nq[1] + nq[2] * nq[2]) < 0.25f;
                                    if (bgp != bgq) wgt = 0;
                                    else if (!bgp) wgt *= std::pow(std::max(0.f, np[0] * nq[0] + np[1] * nq[1] + np[2] * nq[2]), 64.f);
                                }
                                if (in.depth && wgt > 0) {
                                    float zq = in.depth[q];
                                    wgt *= std::exp(-std::fabs(zp - zq) / (0.02f * std::fabs(zp) * float(step) + 1e-4f));
                                }
                            }
                            acc[0] += cur[q * 3] * wgt; acc[1] += cur[q * 3 + 1] * wgt; acc[2] += cur[q * 3 + 2] * wgt;
                            vsum += var[q] * wgt * wgt;
                            wsum += wgt;
                        }
                    for (int c = 0; c < 3; ++c) next[p * 3 + size_t(c)] = acc[c] / wsum;
                    nvar[p] = vsum / (wsum * wsum);
                }
        });
        cur.swap(next);
        var.swap(nvar);
    }
    for (size_t i = 0; i < n * 3; ++i) out[i] = cur[i] * alb[i];
    return out;
}

} // namespace hd

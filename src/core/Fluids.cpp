// Fluid nodes: FLIP Solver, Particle Fluid Surface and Ocean.
//
// FLIP Solver
//   Particles carry velocity; every substep they are splatted onto a MAC (staggered) grid, forces
//   and viscosity are applied, the grid is made divergence-free with a pressure solve (MIC(0)
//   preconditioned conjugate gradient, air cells as p = 0, solids as walls), and the grid velocity
//   change is blended back onto the particles (FLIP / PIC blend). Particles are advected through the
//   grid with RK2 and kept out of the container walls and the collider. Substeps follow a CFL limit.
//
// Particle Fluid Surface
//   A signed distance field built from the particles (union of spheres, then smoothed) is turned into
//   a closed triangle mesh with marching tetrahedra (watertight, no lookup tables).
//
// Ocean
//   A directional wave spectrum (Phillips, scaled to Pierson-Moskowitz wave height for the wind speed)
//   is sampled into Gerstner waves: horizontal choppiness, analytic normals and a Jacobian foam mask.
#include "Fluids.h"
#include "Node.h"
#include "Volume.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <unordered_map>

namespace hd {
namespace {

ParmDef F(const char* n, const char* l, float d, float mn = 0, float mx = 1, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Float; p.def[0] = d; p.min = mn; p.max = mx; p.help = help; return p;
}
ParmDef I(const char* n, const char* l, int d, int mn = 0, int mx = 10, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Int; p.def[0] = float(d); p.min = float(mn); p.max = float(mx); p.help = help; return p;
}
ParmDef T(const char* n, const char* l, bool d, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Toggle; p.def[0] = d ? 1.f : 0.f; p.help = help; return p;
}
ParmDef V(const char* n, const char* l, Vec3 d, float mn = -1, float mx = 1, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Vec3; p.def[0] = d.x; p.def[1] = d.y; p.def[2] = d.z; p.min = mn; p.max = mx; p.help = help; return p;
}
ParmDef M(const char* n, const char* l, std::vector<std::string> items, int d = 0) {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Menu; p.menu = std::move(items); p.def[0] = float(d); return p;
}

// ================================================================== FLIP

enum : unsigned char { AIR = 0, FLUID = 1, SOLID = 2 };

struct MacGrid {
    int nx = 0, ny = 0, nz = 0;
    Vec3 origin;
    float dx = 0.1f;
    std::vector<float> u, v, w;          // face velocities
    std::vector<float> uw, vw, ww;       // splat weights (validity)
    std::vector<unsigned char> cell;     // AIR / FLUID / SOLID
    std::vector<unsigned char> solidMask; // static solids (collider)

    void init(int x, int y, int z, Vec3 o, float h) {
        nx = x; ny = y; nz = z; origin = o; dx = h;
        u.assign(size_t(nx + 1) * ny * nz, 0); uw = u;
        v.assign(size_t(nx) * (ny + 1) * nz, 0); vw = v;
        w.assign(size_t(nx) * ny * (nz + 1), 0); ww = w;
        cell.assign(size_t(nx) * ny * nz, AIR);
        solidMask.assign(cell.size(), 0);
    }
    size_t ci(int i, int j, int k) const { return size_t(i) + size_t(nx) * (size_t(j) + size_t(ny) * size_t(k)); }
    size_t ui(int i, int j, int k) const { return size_t(i) + size_t(nx + 1) * (size_t(j) + size_t(ny) * size_t(k)); }
    size_t vi(int i, int j, int k) const { return size_t(i) + size_t(nx) * (size_t(j) + size_t(ny + 1) * size_t(k)); }
    size_t wi(int i, int j, int k) const { return size_t(i) + size_t(nx) * (size_t(j) + size_t(ny) * size_t(k)); }
    unsigned char type(int i, int j, int k) const {
        if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz) return SOLID;
        return cell[ci(i, j, k)];
    }
};

// Trilinear sample of a staggered component whose samples sit at origin + (i + ox, j + oy, k + oz) * dx.
float sampleComp(const std::vector<float>& f, int sx, int sy, int sz, const Vec3& origin, float dx, Vec3 off, const Vec3& p) {
    float fx = (p.x - origin.x) / dx - off.x, fy = (p.y - origin.y) / dx - off.y, fz = (p.z - origin.z) / dx - off.z;
    fx = std::clamp(fx, 0.f, float(sx - 1) - 1e-4f);
    fy = std::clamp(fy, 0.f, float(sy - 1) - 1e-4f);
    fz = std::clamp(fz, 0.f, float(sz - 1) - 1e-4f);
    int i = int(fx), j = int(fy), k = int(fz);
    float tx = fx - float(i), ty = fy - float(j), tz = fz - float(k);
    int i1 = std::min(i + 1, sx - 1), j1 = std::min(j + 1, sy - 1), k1 = std::min(k + 1, sz - 1);
    auto at = [&](int a, int b, int c) { return f[size_t(a) + size_t(sx) * (size_t(b) + size_t(sy) * size_t(c))]; };
    float c00 = at(i, j, k) * (1 - tx) + at(i1, j, k) * tx;
    float c10 = at(i, j1, k) * (1 - tx) + at(i1, j1, k) * tx;
    float c01 = at(i, j, k1) * (1 - tx) + at(i1, j, k1) * tx;
    float c11 = at(i, j1, k1) * (1 - tx) + at(i1, j1, k1) * tx;
    return (c00 * (1 - ty) + c10 * ty) * (1 - tz) + (c01 * (1 - ty) + c11 * ty) * tz;
}

Vec3 sampleVel(const MacGrid& g, const std::vector<float>& u, const std::vector<float>& v, const std::vector<float>& w, const Vec3& p) {
    return {sampleComp(u, g.nx + 1, g.ny, g.nz, g.origin, g.dx, Vec3(0, 0.5f, 0.5f), p),
            sampleComp(v, g.nx, g.ny + 1, g.nz, g.origin, g.dx, Vec3(0.5f, 0, 0.5f), p),
            sampleComp(w, g.nx, g.ny, g.nz + 1, g.origin, g.dx, Vec3(0.5f, 0.5f, 0), p)};
}

void splatComp(std::vector<float>& f, std::vector<float>& wsum, int sx, int sy, int sz, const Vec3& origin, float dx, Vec3 off,
               const Vec3& p, float val) {
    float fx = (p.x - origin.x) / dx - off.x, fy = (p.y - origin.y) / dx - off.y, fz = (p.z - origin.z) / dx - off.z;
    int i = int(std::floor(fx)), j = int(std::floor(fy)), k = int(std::floor(fz));
    float tx = fx - float(i), ty = fy - float(j), tz = fz - float(k);
    for (int c = 0; c < 8; ++c) {
        int a = i + (c & 1), b = j + ((c >> 1) & 1), d = k + ((c >> 2) & 1);
        if (a < 0 || b < 0 || d < 0 || a >= sx || b >= sy || d >= sz) continue;
        float wt = ((c & 1) ? tx : 1 - tx) * (((c >> 1) & 1) ? ty : 1 - ty) * (((c >> 2) & 1) ? tz : 1 - tz);
        size_t id = size_t(a) + size_t(sx) * (size_t(b) + size_t(sy) * size_t(d));
        f[id] += wt * val;
        wsum[id] += wt;
    }
}

// Solves A q = b on fluid cells with MIC(0)-preconditioned CG. Returns the final residual (max norm).
float pressureSolve(const MacGrid& g, const std::vector<float>& b, std::vector<float>& q, int maxIter, float tol) {
    const size_t n = g.cell.size();
    q.assign(n, 0.f);
    std::vector<float> Adiag(n, 0), Ai(n, 0), Aj(n, 0), Ak(n, 0);
    for (int k = 0; k < g.nz; ++k)
        for (int j = 0; j < g.ny; ++j)
            for (int i = 0; i < g.nx; ++i) {
                size_t id = g.ci(i, j, k);
                if (g.cell[id] != FLUID) continue;
                float d = 0;
                const int nb[6][3] = {{i - 1, j, k}, {i + 1, j, k}, {i, j - 1, k}, {i, j + 1, k}, {i, j, k - 1}, {i, j, k + 1}};
                for (auto& o : nb) if (g.type(o[0], o[1], o[2]) != SOLID) d += 1;
                Adiag[id] = d;
                if (g.type(i + 1, j, k) == FLUID) Ai[id] = -1;
                if (g.type(i, j + 1, k) == FLUID) Aj[id] = -1;
                if (g.type(i, j, k + 1) == FLUID) Ak[id] = -1;
            }
    // MIC(0) preconditioner
    std::vector<float> prec(n, 0);
    const float tau = 0.97f, sigma = 0.25f;
    for (int k = 0; k < g.nz; ++k)
        for (int j = 0; j < g.ny; ++j)
            for (int i = 0; i < g.nx; ++i) {
                size_t id = g.ci(i, j, k);
                if (g.cell[id] != FLUID) continue;
                float e = Adiag[id];
                if (i > 0) { size_t m = g.ci(i - 1, j, k); if (g.cell[m] == FLUID) { float a = Ai[m] * prec[m]; e -= a * a + tau * Ai[m] * (Aj[m] + Ak[m]) * prec[m] * prec[m]; } }
                if (j > 0) { size_t m = g.ci(i, j - 1, k); if (g.cell[m] == FLUID) { float a = Aj[m] * prec[m]; e -= a * a + tau * Aj[m] * (Ai[m] + Ak[m]) * prec[m] * prec[m]; } }
                if (k > 0) { size_t m = g.ci(i, j, k - 1); if (g.cell[m] == FLUID) { float a = Ak[m] * prec[m]; e -= a * a + tau * Ak[m] * (Ai[m] + Aj[m]) * prec[m] * prec[m]; } }
                if (e < sigma * Adiag[id]) e = Adiag[id];
                prec[id] = e > 1e-12f ? 1.f / std::sqrt(e) : 0.f;
            }
    auto applyPrec = [&](const std::vector<float>& r, std::vector<float>& z) {
        std::vector<float> t(n, 0);
        for (int k = 0; k < g.nz; ++k)
            for (int j = 0; j < g.ny; ++j)
                for (int i = 0; i < g.nx; ++i) {
                    size_t id = g.ci(i, j, k);
                    if (g.cell[id] != FLUID) continue;
                    float s = r[id];
                    if (i > 0) { size_t m = g.ci(i - 1, j, k); if (g.cell[m] == FLUID) s -= Ai[m] * prec[m] * t[m]; }
                    if (j > 0) { size_t m = g.ci(i, j - 1, k); if (g.cell[m] == FLUID) s -= Aj[m] * prec[m] * t[m]; }
                    if (k > 0) { size_t m = g.ci(i, j, k - 1); if (g.cell[m] == FLUID) s -= Ak[m] * prec[m] * t[m]; }
                    t[id] = s * prec[id];
                }
        z.assign(n, 0);
        for (int k = g.nz - 1; k >= 0; --k)
            for (int j = g.ny - 1; j >= 0; --j)
                for (int i = g.nx - 1; i >= 0; --i) {
                    size_t id = g.ci(i, j, k);
                    if (g.cell[id] != FLUID) continue;
                    float s = t[id];
                    if (i + 1 < g.nx) { size_t p = g.ci(i + 1, j, k); if (g.cell[p] == FLUID) s -= Ai[id] * prec[id] * z[p]; }
                    if (j + 1 < g.ny) { size_t p = g.ci(i, j + 1, k); if (g.cell[p] == FLUID) s -= Aj[id] * prec[id] * z[p]; }
                    if (k + 1 < g.nz) { size_t p = g.ci(i, j, k + 1); if (g.cell[p] == FLUID) s -= Ak[id] * prec[id] * z[p]; }
                    z[id] = s * prec[id];
                }
    };
    auto applyA = [&](const std::vector<float>& s, std::vector<float>& out) {
        out.assign(n, 0);
        parallelFor(g.nz, [&](int k0, int k1) {
            for (int k = k0; k < k1; ++k)
                for (int j = 0; j < g.ny; ++j)
                    for (int i = 0; i < g.nx; ++i) {
                        size_t id = g.ci(i, j, k);
                        if (g.cell[id] != FLUID) continue;
                        float r = Adiag[id] * s[id];
                        if (i + 1 < g.nx) r += Ai[id] * s[g.ci(i + 1, j, k)];
                        if (j + 1 < g.ny) r += Aj[id] * s[g.ci(i, j + 1, k)];
                        if (k + 1 < g.nz) r += Ak[id] * s[g.ci(i, j, k + 1)];
                        if (i > 0) r += Ai[g.ci(i - 1, j, k)] * s[g.ci(i - 1, j, k)];
                        if (j > 0) r += Aj[g.ci(i, j - 1, k)] * s[g.ci(i, j - 1, k)];
                        if (k > 0) r += Ak[g.ci(i, j, k - 1)] * s[g.ci(i, j, k - 1)];
                        out[id] = r;
                    }
        });
    };
    auto dotv = [&](const std::vector<float>& a, const std::vector<float>& c) { double s = 0; for (size_t i = 0; i < n; ++i) s += double(a[i]) * c[i]; return s; };
    auto maxAbs = [&](const std::vector<float>& a) { float m = 0; for (float x : a) m = std::max(m, std::fabs(x)); return m; };
    std::vector<float> r = b, z, s, As;
    float r0 = maxAbs(r);
    if (r0 < tol) return r0;
    applyPrec(r, z);
    s = z;
    double sigmaNew = dotv(z, r);
    for (int it = 0; it < maxIter; ++it) {
        applyA(s, As);
        double denom = dotv(s, As);
        if (std::fabs(denom) < 1e-30) break;
        double alpha = sigmaNew / denom;
        for (size_t i = 0; i < n; ++i) { q[i] += float(alpha) * s[i]; r[i] -= float(alpha) * As[i]; }
        float rm = maxAbs(r);
        if (rm < tol * std::max(1.f, r0)) return rm;
        applyPrec(r, z);
        double sigmaNext = dotv(z, r);
        double beta = sigmaNext / sigmaNew;
        for (size_t i = 0; i < n; ++i) s[i] = z[i] + float(beta) * s[i];
        sigmaNew = sigmaNext;
    }
    return maxAbs(r);
}

// Spreads valid face values outward by `layers` cells (averaging valid neighbours).
void extrapolate(std::vector<float>& f, std::vector<char>& valid, int sx, int sy, int sz, int layers) {
    auto id = [&](int i, int j, int k) { return size_t(i) + size_t(sx) * (size_t(j) + size_t(sy) * size_t(k)); };
    for (int l = 0; l < layers; ++l) {
        std::vector<char> nv = valid;
        std::vector<float> nf = f;
        for (int k = 0; k < sz; ++k)
            for (int j = 0; j < sy; ++j)
                for (int i = 0; i < sx; ++i) {
                    size_t c = id(i, j, k);
                    if (valid[c]) continue;
                    float s = 0; int cnt = 0;
                    const int nb[6][3] = {{i - 1, j, k}, {i + 1, j, k}, {i, j - 1, k}, {i, j + 1, k}, {i, j, k - 1}, {i, j, k + 1}};
                    for (auto& o : nb) {
                        if (o[0] < 0 || o[1] < 0 || o[2] < 0 || o[0] >= sx || o[1] >= sy || o[2] >= sz) continue;
                        size_t q = id(o[0], o[1], o[2]);
                        if (valid[q]) { s += f[q]; ++cnt; }
                    }
                    if (cnt) { nf[c] = s / float(cnt); nv[c] = 1; }
                }
        f.swap(nf);
        valid.swap(nv);
    }
}

struct FlipParams {
    Vec3 gravity;
    float flip;          // 0 = PIC, 1 = FLIP
    float viscosity;     // m^2/s
    float cfl;
    int maxSubsteps;
    bool openTop;
    float volumeCorrection; // 0..1: how hard over-packed cells push apart
};

struct FlipStats { int substeps = 0; float residual = 0; float maxDiv = 0; };

// One frame of simulation for particles P, Vp in grid g (whose solidMask is set). Returns stats.
FlipStats flipFrame(MacGrid& g, std::vector<Vec3>& P, std::vector<Vec3>& Vp, float frameDt, const FlipParams& fp) {
    FlipStats stats;
    float t = 0;
    const float dx = g.dx;
    const Vec3 lo = g.origin + Vec3(dx * 0.501f), hi = g.origin + Vec3(float(g.nx), float(g.ny), float(g.nz)) * dx - Vec3(dx * 0.501f);
    while (t < frameDt - 1e-7f && stats.substeps < fp.maxSubsteps) {
        float maxV = 1e-3f;
        for (auto& v : Vp) maxV = std::max(maxV, std::max(std::fabs(v.x), std::max(std::fabs(v.y), std::fabs(v.z))));
        maxV += std::sqrt(dx * length(fp.gravity)); // account for acceleration within the step
        float dt = std::min(frameDt - t, fp.cfl * dx / maxV);
        if (stats.substeps == fp.maxSubsteps - 1) dt = frameDt - t;
        ++stats.substeps;

        // ---- particles to grid
        std::fill(g.u.begin(), g.u.end(), 0.f); std::fill(g.uw.begin(), g.uw.end(), 0.f);
        std::fill(g.v.begin(), g.v.end(), 0.f); std::fill(g.vw.begin(), g.vw.end(), 0.f);
        std::fill(g.w.begin(), g.w.end(), 0.f); std::fill(g.ww.begin(), g.ww.end(), 0.f);
        for (size_t i = 0; i < g.cell.size(); ++i) g.cell[i] = g.solidMask[i] ? SOLID : AIR;
        parallelFor(4, [&](int c0, int c1) {
            for (int comp = c0; comp < c1; ++comp) {
                if (comp == 0) for (size_t p = 0; p < P.size(); ++p) splatComp(g.u, g.uw, g.nx + 1, g.ny, g.nz, g.origin, dx, Vec3(0, 0.5f, 0.5f), P[p], Vp[p].x);
                else if (comp == 1) for (size_t p = 0; p < P.size(); ++p) splatComp(g.v, g.vw, g.nx, g.ny + 1, g.nz, g.origin, dx, Vec3(0.5f, 0, 0.5f), P[p], Vp[p].y);
                else if (comp == 2) for (size_t p = 0; p < P.size(); ++p) splatComp(g.w, g.ww, g.nx, g.ny, g.nz + 1, g.origin, dx, Vec3(0.5f, 0.5f, 0), P[p], Vp[p].z);
                else for (size_t p = 0; p < P.size(); ++p) {
                    int i = int(std::floor((P[p].x - g.origin.x) / dx)), j = int(std::floor((P[p].y - g.origin.y) / dx)), k = int(std::floor((P[p].z - g.origin.z) / dx));
                    if (i >= 0 && j >= 0 && k >= 0 && i < g.nx && j < g.ny && k < g.nz && g.cell[g.ci(i, j, k)] != SOLID) g.cell[g.ci(i, j, k)] = FLUID;
                }
            }
        });
        auto normalizeComp = [](std::vector<float>& f, const std::vector<float>& wsum) {
            for (size_t i = 0; i < f.size(); ++i) f[i] = wsum[i] > 1e-6f ? f[i] / wsum[i] : 0.f;
        };
        normalizeComp(g.u, g.uw); normalizeComp(g.v, g.vw); normalizeComp(g.w, g.ww);
        std::vector<float> u0 = g.u, v0 = g.v, w0 = g.w;

        // ---- forces
        for (size_t i = 0; i < g.u.size(); ++i) g.u[i] += fp.gravity.x * dt;
        for (size_t i = 0; i < g.v.size(); ++i) g.v[i] += fp.gravity.y * dt;
        for (size_t i = 0; i < g.w.size(); ++i) g.w[i] += fp.gravity.z * dt;

        // faces between two non-fluid cells carry no meaning; faces touching a solid are walls
        auto enforceSolids = [&]() {
            for (int k = 0; k < g.nz; ++k)
                for (int j = 0; j < g.ny; ++j)
                    for (int i = 0; i <= g.nx; ++i)
                        if (g.type(i - 1, j, k) == SOLID || g.type(i, j, k) == SOLID) g.u[g.ui(i, j, k)] = 0;
            for (int k = 0; k < g.nz; ++k)
                for (int j = 0; j <= g.ny; ++j)
                    for (int i = 0; i < g.nx; ++i) {
                        bool top = j == g.ny && fp.openTop;
                        if (!top && (g.type(i, j - 1, k) == SOLID || g.type(i, j, k) == SOLID)) g.v[g.vi(i, j, k)] = 0;
                    }
            for (int k = 0; k <= g.nz; ++k)
                for (int j = 0; j < g.ny; ++j)
                    for (int i = 0; i < g.nx; ++i)
                        if (g.type(i, j, k - 1) == SOLID || g.type(i, j, k) == SOLID) g.w[g.wi(i, j, k)] = 0;
        };

        // ---- viscosity (implicit, Jacobi) on faces next to fluid
        if (fp.viscosity > 0) {
            float a = fp.viscosity * dt / (dx * dx);
            auto diffuse = [&](std::vector<float>& f, int sx, int sy, int sz, int axis) {
                std::vector<float> f0 = f, cur = f;
                auto id = [&](int i, int j, int k) { return size_t(i) + size_t(sx) * (size_t(j) + size_t(sy) * size_t(k)); };
                auto nearFluid = [&](int i, int j, int k) {
                    int di = axis == 0, dj = axis == 1, dk = axis == 2;
                    return g.type(i, j, k) == FLUID || g.type(i - di, j - dj, k - dk) == FLUID;
                };
                for (int it = 0; it < 30; ++it) {
                    std::vector<float> nx = cur;
                    for (int k = 0; k < sz; ++k)
                        for (int j = 0; j < sy; ++j)
                            for (int i = 0; i < sx; ++i) {
                                if (!nearFluid(i, j, k)) continue;
                                float s = 0; int cnt = 0;
                                const int nb[6][3] = {{i - 1, j, k}, {i + 1, j, k}, {i, j - 1, k}, {i, j + 1, k}, {i, j, k - 1}, {i, j, k + 1}};
                                for (auto& o : nb) {
                                    if (o[0] < 0 || o[1] < 0 || o[2] < 0 || o[0] >= sx || o[1] >= sy || o[2] >= sz) continue;
                                    if (!nearFluid(o[0], o[1], o[2])) continue;
                                    s += cur[id(o[0], o[1], o[2])]; ++cnt;
                                }
                                nx[id(i, j, k)] = (f0[id(i, j, k)] + a * s) / (1 + a * float(cnt));
                            }
                    cur.swap(nx);
                }
                f.swap(cur);
            };
            diffuse(g.u, g.nx + 1, g.ny, g.nz, 0);
            diffuse(g.v, g.nx, g.ny + 1, g.nz, 1);
            diffuse(g.w, g.nx, g.ny, g.nz + 1, 2);
        }
        enforceSolids();

        // ---- pressure projection
        std::vector<float> b(g.cell.size(), 0.f), q;
        for (int k = 0; k < g.nz; ++k)
            for (int j = 0; j < g.ny; ++j)
                for (int i = 0; i < g.nx; ++i) {
                    size_t id = g.ci(i, j, k);
                    if (g.cell[id] != FLUID) continue;
                    float div = g.u[g.ui(i + 1, j, k)] - g.u[g.ui(i, j, k)] + g.v[g.vi(i, j + 1, k)] - g.v[g.vi(i, j, k)] + g.w[g.wi(i, j, k + 1)] - g.w[g.wi(i, j, k)];
                    b[id] = -div;
                }
        stats.residual = pressureSolve(g, b, q, 400, 1e-5f);
        auto qa = [&](int i, int j, int k) { return g.type(i, j, k) == FLUID ? q[g.ci(i, j, k)] : 0.f; };
        for (int k = 0; k < g.nz; ++k)
            for (int j = 0; j < g.ny; ++j)
                for (int i = 0; i <= g.nx; ++i) {
                    unsigned char a = g.type(i - 1, j, k), c = g.type(i, j, k);
                    if (a == SOLID || c == SOLID) continue;
                    if (a == FLUID || c == FLUID) g.u[g.ui(i, j, k)] -= qa(i, j, k) - qa(i - 1, j, k);
                }
        for (int k = 0; k < g.nz; ++k)
            for (int j = 0; j <= g.ny; ++j)
                for (int i = 0; i < g.nx; ++i) {
                    unsigned char a = g.type(i, j - 1, k), c = (j == g.ny && fp.openTop) ? (unsigned char)AIR : g.type(i, j, k);
                    if (a == SOLID || c == SOLID) continue;
                    if (a == FLUID || c == FLUID) g.v[g.vi(i, j, k)] -= (j < g.ny ? qa(i, j, k) : 0.f) - qa(i, j - 1, k);
                }
        for (int k = 0; k <= g.nz; ++k)
            for (int j = 0; j < g.ny; ++j)
                for (int i = 0; i < g.nx; ++i) {
                    unsigned char a = g.type(i, j, k - 1), c = g.type(i, j, k);
                    if (a == SOLID || c == SOLID) continue;
                    if (a == FLUID || c == FLUID) g.w[g.wi(i, j, k)] -= qa(i, j, k) - qa(i, j, k - 1);
                }
        // measure divergence left in fluid cells
        float maxDiv = 0;
        for (int k = 0; k < g.nz; ++k)
            for (int j = 0; j < g.ny; ++j)
                for (int i = 0; i < g.nx; ++i)
                    if (g.cell[g.ci(i, j, k)] == FLUID) {
                        float div = g.u[g.ui(i + 1, j, k)] - g.u[g.ui(i, j, k)] + g.v[g.vi(i, j + 1, k)] - g.v[g.vi(i, j, k)] + g.w[g.wi(i, j, k + 1)] - g.w[g.wi(i, j, k)];
                        maxDiv = std::max(maxDiv, std::fabs(div) / dx);
                    }
        stats.maxDiv = std::max(stats.maxDiv, maxDiv);

        // ---- extrapolate into the air so particles near the surface sample sensible velocities
        auto validFaces = [&](int axis, int sx, int sy, int sz) {
            std::vector<char> valid(size_t(sx) * sy * sz, 0);
            for (int k = 0; k < sz; ++k)
                for (int j = 0; j < sy; ++j)
                    for (int i = 0; i < sx; ++i) {
                        int di = axis == 0, dj = axis == 1, dk = axis == 2;
                        unsigned char a = g.type(i - di, j - dj, k - dk), c = g.type(i, j, k);
                        if (a == FLUID || c == FLUID) valid[size_t(i) + size_t(sx) * (size_t(j) + size_t(sy) * size_t(k))] = 1;
                    }
            return valid;
        };
        std::vector<float> du(g.u.size()), dv(g.v.size()), dw(g.w.size());
        {
            auto vu = validFaces(0, g.nx + 1, g.ny, g.nz), vv = validFaces(1, g.nx, g.ny + 1, g.nz), vw = validFaces(2, g.nx, g.ny, g.nz + 1);
            for (size_t i = 0; i < du.size(); ++i) du[i] = vu[i] ? g.u[i] - u0[i] : 0.f;
            for (size_t i = 0; i < dv.size(); ++i) dv[i] = vv[i] ? g.v[i] - v0[i] : 0.f;
            for (size_t i = 0; i < dw.size(); ++i) dw[i] = vw[i] ? g.w[i] - w0[i] : 0.f;
            extrapolate(g.u, vu, g.nx + 1, g.ny, g.nz, 4);
            extrapolate(g.v, vv, g.nx, g.ny + 1, g.nz, 4);
            extrapolate(g.w, vw, g.nx, g.ny, g.nz + 1, 4);
            enforceSolids();
        }

        // ---- grid to particles, advect, collide
        const float flip = std::clamp(fp.flip, 0.f, 1.f);
        parallelFor(int(P.size()), [&](int b0, int b1) {
            for (int p = b0; p < b1; ++p) {
                Vec3 x = P[size_t(p)];
                Vec3 vpic = sampleVel(g, g.u, g.v, g.w, x);
                Vec3 vflip = Vp[size_t(p)] + sampleVel(g, du, dv, dw, x);
                Vec3 vel = vflip * flip + vpic * (1 - flip);
                // RK2 through the grid velocity field
                Vec3 mid = x + vpic * (0.5f * dt);
                Vec3 xn = x + sampleVel(g, g.u, g.v, g.w, mid) * dt;
                Vec3 hiC = hi;
                if (fp.openTop) hiC.y = 1e30f; // particles may leave through the top
                xn = vmax(lo, vmin(hiC, xn));
                int i = int(std::floor((xn.x - g.origin.x) / dx)), j = int(std::floor((xn.y - g.origin.y) / dx)), k = int(std::floor((xn.z - g.origin.z) / dx));
                if (i >= 0 && j >= 0 && k >= 0 && i < g.nx && j < g.ny && k < g.nz && g.solidMask[g.ci(i, j, k)]) {
                    xn = x; // never step into the collider
                    vel = vpic;
                }
                // velocities into the container walls are removed
                if (xn.x <= lo.x && vel.x < 0) vel.x = 0;
                if (xn.x >= hi.x && vel.x > 0) vel.x = 0;
                if (xn.y <= lo.y && vel.y < 0) vel.y = 0;
                if (!fp.openTop && xn.y >= hi.y && vel.y > 0) vel.y = 0;
                if (xn.z <= lo.z && vel.z < 0) vel.z = 0;
                if (xn.z >= hi.z && vel.z > 0) vel.z = 0;
                P[size_t(p)] = xn;
                Vp[size_t(p)] = vel;
            }
        });
        // ---- separation: push apart particles closer than their rest spacing (positions only, so no energy
        // is added). This is what keeps FLIP from slowly losing volume as particles bunch up.
        if (fp.volumeCorrection > 0 && !P.empty()) {
            // uniform grid with cells of the rest spacing, filled by counting sort (no hashing)
            const float r = dx * 0.5f; // rest spacing for 8 particles per voxel
            const float inv = 1.f / r;
            const int gx = g.nx * 2 + 2, gy = g.ny * 2 + 2 + (fp.openTop ? 64 : 0), gz = g.nz * 2 + 2;
            auto cellOf = [&](const Vec3& x, int& cx, int& cy, int& cz) {
                cx = std::clamp(int(std::floor((x.x - g.origin.x) * inv)) + 1, 0, gx - 1);
                cy = std::clamp(int(std::floor((x.y - g.origin.y) * inv)) + 1, 0, gy - 1);
                cz = std::clamp(int(std::floor((x.z - g.origin.z) * inv)) + 1, 0, gz - 1);
            };
            const size_t ncell = size_t(gx) * size_t(gy) * size_t(gz);
            std::vector<int> cellStart(ncell + 1, 0), cellIdx(P.size()), sorted(P.size());
            for (size_t p = 0; p < P.size(); ++p) {
                int cx, cy, cz;
                cellOf(P[p], cx, cy, cz);
                int c = cx + gx * (cy + gy * cz);
                cellIdx[p] = c;
                cellStart[size_t(c) + 1]++;
            }
            for (size_t c = 0; c < ncell; ++c) cellStart[c + 1] += cellStart[c];
            {
                std::vector<int> fill(cellStart.begin(), cellStart.end() - 1);
                for (size_t p = 0; p < P.size(); ++p) sorted[size_t(fill[size_t(cellIdx[p])]++)] = int(p);
            }
            std::vector<Vec3> push(P.size(), Vec3(0));
            parallelFor(int(P.size()), [&](int b0, int b1) {
                for (int p = b0; p < b1; ++p) {
                    const Vec3 x = P[size_t(p)];
                    int cx, cy, cz;
                    cellOf(x, cx, cy, cz);
                    Vec3 acc(0);
                    for (int dz = -1; dz <= 1; ++dz) {
                        int zz = cz + dz;
                        if (zz < 0 || zz >= gz) continue;
                        for (int dy = -1; dy <= 1; ++dy) {
                            int yy = cy + dy;
                            if (yy < 0 || yy >= gy) continue;
                            for (int dxx = -1; dxx <= 1; ++dxx) {
                                int xx = cx + dxx;
                                if (xx < 0 || xx >= gx) continue;
                                size_t c = size_t(xx) + size_t(gx) * (size_t(yy) + size_t(gy) * size_t(zz));
                                for (int k = cellStart[c]; k < cellStart[c + 1]; ++k) {
                                    int q = sorted[size_t(k)];
                                    if (q == p) continue;
                                    Vec3 d = x - P[size_t(q)];
                                    float l2 = dot(d, d);
                                    if (l2 >= r * r) continue;
                                    float l = std::sqrt(l2);
                                    if (l < 1e-7f) { d = Vec3(hashRandom(uint32_t(p) * 3u) - 0.5f, hashRandom(uint32_t(p) * 3u + 1u) - 0.5f, hashRandom(uint32_t(p) * 3u + 2u) - 0.5f); l = length(d) + 1e-7f; }
                                    acc += d / l * (0.5f * (r - l));
                                }
                            }
                        }
                    }
                    push[size_t(p)] = acc * fp.volumeCorrection;
                }
            });
            Vec3 hiC = hi;
            if (fp.openTop) hiC.y = 1e30f;
            for (size_t p = 0; p < P.size(); ++p) {
                Vec3 xn = vmax(lo, vmin(hiC, P[p] + push[p]));
                int i = int(std::floor((xn.x - g.origin.x) / dx)), j = int(std::floor((xn.y - g.origin.y) / dx)), k = int(std::floor((xn.z - g.origin.z) / dx));
                if (i >= 0 && j >= 0 && k >= 0 && i < g.nx && j < g.ny && k < g.nz && g.solidMask[g.ci(i, j, k)]) continue;
                P[p] = xn;
            }
        }
        if (fp.openTop) {
            // particles that left the container are removed
            size_t wpos = 0;
            for (size_t p = 0; p < P.size(); ++p)
                if (P[p].y < hi.y + dx) { P[wpos] = P[p]; Vp[wpos] = Vp[p]; ++wpos; }
            P.resize(wpos); Vp.resize(wpos);
        }
        t += dt;
    }
    return stats;
}

struct FlipFrame { std::vector<Vec3> P, V; };
struct FlipState {
    uint64_t sig = 0;
    std::map<int, FlipFrame> frames;
    size_t total = 0;
    FlipStats last;
};

// Jittered particles (2x2x2 per voxel) in every voxel of the mask not already crowded.
void seedParticles(const MacGrid& g, const std::vector<unsigned char>& mask, const std::vector<int>* counts, int maxPerCell,
                   uint32_t seed, const Vec3& vel, std::vector<Vec3>& P, std::vector<Vec3>& V) {
    for (int k = 0; k < g.nz; ++k)
        for (int j = 0; j < g.ny; ++j)
            for (int i = 0; i < g.nx; ++i) {
                size_t id = g.ci(i, j, k);
                if (!mask[id] || g.solidMask[id]) continue;
                if (counts && (*counts)[id] >= maxPerCell) continue;
                for (int s = 0; s < 8; ++s) {
                    uint32_t h = uint32_t(id) * 8u + uint32_t(s) + seed;
                    Vec3 o((float(s & 1) + 0.2f + 0.6f * hashRandom(h * 3u)) * 0.5f, (float((s >> 1) & 1) + 0.2f + 0.6f * hashRandom(h * 3u + 1u)) * 0.5f,
                           (float((s >> 2) & 1) + 0.2f + 0.6f * hashRandom(h * 3u + 2u)) * 0.5f);
                    P.push_back(g.origin + (Vec3(float(i), float(j), float(k)) + o) * g.dx);
                    V.push_back(vel);
                }
            }
}

void cookFlip(CookContext& c) {
    FlipState& st = c.state<FlipState>();
    const uint64_t sig = c.setupSignature();
    if (st.sig != sig) { st = FlipState(); st.sig = sig; }
    const int start = c.i("startframe");
    const int frame = int(std::floor(c.frame() + 0.5f));
    Node& node = c.node();
    node.simCacheStart = 0; node.simCacheEnd = -1;

    const float dx = std::max(0.005f, c.f("voxelsize"));
    const Vec3 size = vmax(c.v("domainsize"), Vec3(dx * 4));
    const Vec3 center = c.v("domaincenter");
    const int nx = std::max(4, int(std::ceil(size.x / dx))), ny = std::max(4, int(std::ceil(size.y / dx))), nz = std::max(4, int(std::ceil(size.z / dx)));
    if (double(nx) * ny * nz > 8e6) { c.error("The container has too many voxels (over 8 million). Raise Voxel Size."); return; }
    MacGrid g;
    g.init(nx, ny, nz, center - Vec3(float(nx), float(ny), float(nz)) * (dx * 0.5f), dx);

    auto output = [&](const FlipFrame& f) {
        Geometry& o = c.out;
        o.P = f.P;
        Attrib& v = o.addAttrib(AttribClass::Point, "v", 3);
        v.data.resize(f.V.size() * 3);
        for (size_t i = 0; i < f.V.size(); ++i) v.setV(i, f.V[i]);
        Attrib& ps = o.addAttrib(AttribClass::Point, "pscale", 1);
        ps.data.assign(f.P.size(), dx * 0.35f);
        if (c.b("colorspeed")) {
            Attrib& cd = o.addAttrib(AttribClass::Point, "Cd", 3);
            cd.data.resize(f.P.size() * 3);
            const Vec3 deep = c.v("color"), fast(0.85f, 0.93f, 1.f);
            const float top = std::max(0.01f, c.f("speedmax"));
            for (size_t i = 0; i < f.P.size(); ++i) cd.setV(i, lerp(deep, fast, std::clamp(length(f.V[i]) / top, 0.f, 1.f)));
        }
        o.syncAttribSizes();
    };
    if (frame < start) { output(FlipFrame()); return; }
    if (frame - start > 100000) { c.error("Frame is too far from the start frame."); return; }

    // collider voxels (static, sampled at the start frame)
    if (c.hasInput(1)) {
        Geometry col;
        if (c.inputAtFrame(1, float(start), col)) voxelizeGeometry(col, nx, ny, nz, g.origin, dx, dx, g.solidMask);
    }
    const bool animatedSource = c.inputIsAnimated(0);
    const int mode = c.i("emission");

    int have = start - 1;
    { auto it = st.frames.upper_bound(frame); if (it != st.frames.begin()) { --it; have = it->first; } }
    FlipFrame cur;
    if (have >= start) cur = st.frames[have];
    for (int k = have + 1; k <= frame; ++k) {
        c.setEvalFrame(float(k));
        // ---- emission
        if (k == start || mode == 1) {
            Geometry src;
            bool ok = (k == frame || !animatedSource) ? (src = c.input(0), true) : c.inputAtFrame(0, float(k), src);
            if (ok) {
                std::vector<unsigned char> mask(g.cell.size(), 0);
                voxelizeGeometry(src, nx, ny, nz, g.origin, dx, dx * 0.5f, mask);
                std::vector<int> counts(g.cell.size(), 0);
                for (auto& p : cur.P) {
                    int i = int(std::floor((p.x - g.origin.x) / dx)), j = int(std::floor((p.y - g.origin.y) / dx)), kk = int(std::floor((p.z - g.origin.z) / dx));
                    if (i >= 0 && j >= 0 && kk >= 0 && i < nx && j < ny && kk < nz) counts[g.ci(i, j, kk)]++;
                }
                seedParticles(g, mask, &counts, 3, uint32_t(k) * 7919u + uint32_t(c.i("seed")) * 104729u, c.v("vel"), cur.P, cur.V);
                const size_t cap = size_t(std::max(1000, c.i("maxpts")));
                if (cur.P.size() > cap) { cur.P.resize(cap); cur.V.resize(cap); }
            }
        }
        // ---- simulate this frame (the start frame is the initial state)
        if (k > start) {
            FlipParams fp;
            fp.gravity = c.v("gravity");
            fp.flip = c.f("flipratio");
            fp.viscosity = std::max(0.f, c.f("viscosity"));
            fp.cfl = std::clamp(c.f("cfl"), 0.2f, 10.f);
            fp.maxSubsteps = std::clamp(c.i("maxsubsteps"), 1, 50);
            fp.openTop = c.b("opentop");
            fp.volumeCorrection = std::clamp(c.f("volcorrect"), 0.f, 1.f);
            st.last = flipFrame(g, cur.P, cur.V, 1.f / c.fps(), fp);
        }
        auto old = st.frames.find(k);
        if (old != st.frames.end()) st.total -= old->second.P.size();
        st.frames[k] = cur;
        st.total += cur.P.size();
    }
    c.clearEvalFrame();
    const size_t cap = 30u * 1000u * 1000u;
    while (st.total > cap && st.frames.size() > 1 && st.frames.begin()->first != frame) {
        st.total -= st.frames.begin()->second.P.size();
        st.frames.erase(st.frames.begin());
    }
    output(st.frames[frame]);
    int lo = frame, hi = frame;
    while (st.frames.count(lo - 1)) --lo;
    while (st.frames.count(hi + 1)) ++hi;
    node.simCacheStart = lo;
    node.simCacheEnd = hi;
    if (st.frames[frame].P.empty() && frame >= start) c.warning("No fluid: the source has no closed surface inside the container.");
}

// ================================================================== surfacing

// Marching tetrahedra over a node-sampled scalar field (negative inside). Appends a closed mesh to out.
void marchingTets(const std::vector<float>& phi, int nx, int ny, int nz, const Vec3& origin, float dx, Geometry& out) {
    auto nid = [&](int i, int j, int k) { return size_t(i) + size_t(nx) * (size_t(j) + size_t(ny) * size_t(k)); };
    std::unordered_map<uint64_t, int> edgePt;
    auto edgePoint = [&](size_t a, size_t b, const Vec3& pa, const Vec3& pb, float fa, float fb) {
        if (a > b) { std::swap(a, b); }
        uint64_t key = uint64_t(a) * uint64_t(nx) * uint64_t(ny) * uint64_t(nz) + uint64_t(b);
        auto it = edgePt.find(key);
        if (it != edgePt.end()) return it->second;
        float t = fa / (fa - fb);
        int id = out.addPoint(pa + (pb - pa) * t);
        edgePt[key] = id;
        return id;
    };
    // the six tetrahedra of a cube sharing the main diagonal 0-7 (corner bits: x=1, y=2, z=4)
    static const int tets[6][4] = {{0, 1, 3, 7}, {0, 1, 5, 7}, {0, 2, 3, 7}, {0, 2, 6, 7}, {0, 4, 5, 7}, {0, 4, 6, 7}};
    for (int k = 0; k + 1 < nz; ++k)
        for (int j = 0; j + 1 < ny; ++j)
            for (int i = 0; i + 1 < nx; ++i) {
                size_t ids[8];
                Vec3 pos[8];
                float val[8];
                bool anyIn = false, anyOut = false;
                for (int c = 0; c < 8; ++c) {
                    int a = i + (c & 1), b = j + ((c >> 1) & 1), d = k + ((c >> 2) & 1);
                    ids[c] = nid(a, b, d);
                    pos[c] = origin + Vec3(float(a), float(b), float(d)) * dx;
                    val[c] = phi[ids[c]];
                    if (val[c] < 0) anyIn = true; else anyOut = true;
                }
                if (!anyIn || !anyOut) continue;
                for (auto& t : tets) {
                    int in[4], outv[4], ni = 0, no = 0;
                    for (int q = 0; q < 4; ++q) (val[t[q]] < 0 ? in[ni++] : outv[no++]) = t[q];
                    if (ni == 0 || ni == 4) continue;
                    auto E = [&](int a, int b) { return edgePoint(ids[a], ids[b], pos[a], pos[b], val[a], val[b]); };
                    Vec3 cin(0), cout(0);
                    for (int q = 0; q < ni; ++q) cin += pos[in[q]];
                    for (int q = 0; q < no; ++q) cout += pos[outv[q]];
                    Vec3 dir = cout / float(no) - cin / float(ni); // outward
                    auto tri = [&](int a, int b, int cc) {
                        Vec3 n = cross(out.P[size_t(b)] - out.P[size_t(a)], out.P[size_t(cc)] - out.P[size_t(a)]);
                        if (dot(n, dir) < 0) std::swap(b, cc);
                        out.addPrim({a, b, cc});
                    };
                    if (ni == 1) tri(E(in[0], outv[0]), E(in[0], outv[1]), E(in[0], outv[2]));
                    else if (ni == 3) tri(E(outv[0], in[0]), E(outv[0], in[1]), E(outv[0], in[2]));
                    else {
                        int p0 = E(in[0], outv[0]), p1 = E(in[0], outv[1]), p2 = E(in[1], outv[1]), p3 = E(in[1], outv[0]);
                        tri(p0, p1, p2);
                        tri(p0, p2, p3);
                    }
                }
            }
}

} // namespace

bool particleFluidSurface(const Geometry& in, float sep, float radiusScale, float voxelScale, int smoothIters, Geometry& out, std::string* err) {
    out.clear();
    if (in.numPoints() == 0) return true;
    sep = std::max(sep, 1e-4f);
    const float r = sep * std::max(0.1f, radiusScale);
    const float dx = sep * std::clamp(voxelScale, 0.1f, 4.f);
    Vec3 mn, mx;
    in.bounds(mn, mx);
    const float pad = r + 3 * dx;
    mn -= Vec3(pad); mx += Vec3(pad);
    int nx = int(std::ceil((mx.x - mn.x) / dx)) + 1, ny = int(std::ceil((mx.y - mn.y) / dx)) + 1, nz = int(std::ceil((mx.z - mn.z) / dx)) + 1;
    if (double(nx) * ny * nz > 3e7) { if (err) *err = "Surface grid too large: raise Voxel Scale or Particle Separation."; return false; }
    const size_t n = size_t(nx) * ny * nz;
    std::vector<float> phi(n, r + 3 * dx);
    auto nid = [&](int i, int j, int k) { return size_t(i) + size_t(nx) * (size_t(j) + size_t(ny) * size_t(k)); };
    const int reach = int(std::ceil((r + 2 * dx) / dx));
    for (const Vec3& p : in.P) {
        int ci = int((p.x - mn.x) / dx), cj = int((p.y - mn.y) / dx), ck = int((p.z - mn.z) / dx);
        for (int k = std::max(0, ck - reach); k <= std::min(nz - 1, ck + reach); ++k)
            for (int j = std::max(0, cj - reach); j <= std::min(ny - 1, cj + reach); ++j)
                for (int i = std::max(0, ci - reach); i <= std::min(nx - 1, ci + reach); ++i) {
                    Vec3 x = mn + Vec3(float(i), float(j), float(k)) * dx;
                    float d = length(x - p) - r;
                    float& f = phi[nid(i, j, k)];
                    if (d < f) f = d;
                }
    }
    // smoothing: average with neighbours, then erode by the amount the blur grew the surface
    const int smooth = std::clamp(smoothIters, 0, 10);
    for (int s = 0; s < smooth; ++s) {
        std::vector<float> nphi = phi;
        parallelFor(nz, [&](int k0, int k1) {
            for (int k = std::max(1, k0); k < std::min(nz - 1, k1); ++k)
                for (int j = 1; j < ny - 1; ++j)
                    for (int i = 1; i < nx - 1; ++i) {
                        float sum = phi[nid(i - 1, j, k)] + phi[nid(i + 1, j, k)] + phi[nid(i, j - 1, k)] + phi[nid(i, j + 1, k)] + phi[nid(i, j, k - 1)] + phi[nid(i, j, k + 1)];
                        nphi[nid(i, j, k)] = 0.5f * phi[nid(i, j, k)] + sum / 12.f;
                    }
        });
        phi.swap(nphi);
    }
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                float& f = phi[nid(i, j, k)];
                if (i == 0 || j == 0 || k == 0 || i == nx - 1 || j == ny - 1 || k == nz - 1) f = std::max(f, dx);
                if (f == 0.f) f = 1e-7f;
            }
    marchingTets(phi, nx, ny, nz, mn, dx, out);
    out.computePointNormals();
    return true;
}

namespace {

void cookSurface(CookContext& c) {
    const Geometry& in = c.input(0);
    if (in.numPoints() == 0) return;
    float sep = c.f("separation");
    if (sep <= 0) {
        const Attrib* ps = in.findAttrib(AttribClass::Point, "pscale");
        sep = ps ? ps->getF(0) / 0.35f * 0.5f : 0.05f;
    }
    std::string err;
    if (!particleFluidSurface(in, sep, c.f("radiusscale"), c.f("voxelscale"), c.i("smooth"), c.out, &err)) { c.error(err); return; }
    const float r = std::max(sep, 1e-4f) * std::max(0.1f, c.f("radiusscale"));
    // carry velocity onto the surface (nearest particle) when the particles have it
    const Attrib* pv = in.findAttrib(AttribClass::Point, "v");
    if (pv && pv->tuple == 3 && c.b("transferv")) {
        std::unordered_map<int64_t, std::vector<int>> hash;
        const float cs = r * 2;
        auto key = [&](int a, int b, int d) { return (int64_t(a) * 73856093) ^ (int64_t(b) * 19349663) ^ (int64_t(d) * 83492791); };
        for (size_t i = 0; i < in.numPoints(); ++i) {
            const Vec3& p = in.P[i];
            hash[key(int(std::floor(p.x / cs)), int(std::floor(p.y / cs)), int(std::floor(p.z / cs)))].push_back(int(i));
        }
        Attrib& ov = c.out.addAttrib(AttribClass::Point, "v", 3);
        for (size_t i = 0; i < c.out.numPoints(); ++i) {
            const Vec3& x = c.out.P[i];
            int a = int(std::floor(x.x / cs)), b = int(std::floor(x.y / cs)), d = int(std::floor(x.z / cs));
            float best = 1e30f; int bi = -1;
            for (int dz = -1; dz <= 1; ++dz) for (int dy = -1; dy <= 1; ++dy) for (int dxx = -1; dxx <= 1; ++dxx) {
                auto it = hash.find(key(a + dxx, b + dy, d + dz));
                if (it == hash.end()) continue;
                for (int pi : it->second) { float dd = dot(in.P[size_t(pi)] - x, in.P[size_t(pi)] - x); if (dd < best) { best = dd; bi = pi; } }
            }
            if (bi >= 0) ov.setV(i, pv->getV(size_t(bi)));
        }
    }
}

// ================================================================== ocean

struct Wave { float kx, kz, k, amp, omega, phase, q; };

std::vector<Wave> oceanWaves(float windSpeed, float windDirDeg, float amplitude, float chop, int count, float minWavelength, int seed, float alignment) {
    const float g = 9.81f;
    const float L = std::max(0.01f, windSpeed * windSpeed / g); // largest wave from the wind
    const float lmax = std::max(minWavelength * 2, 6.28f * L * 1.5f);
    const float wd = radians(windDirDeg);
    const Vec3 wdir(std::cos(wd), 0, std::sin(wd));
    std::vector<Wave> waves;
    const int nl = std::max(1, int(std::sqrt(float(count))));
    const int na = std::max(1, count / nl);
    double var = 0;
    for (int li = 0; li < nl; ++li) {
        // log-spaced wavelengths
        float t0 = float(li) / float(nl), t1 = float(li + 1) / float(nl);
        float l0 = lmax * std::pow(minWavelength / lmax, t0), l1 = lmax * std::pow(minWavelength / lmax, t1);
        float k0 = 2 * kPi / l0, k1 = 2 * kPi / l1;
        for (int ai = 0; ai < na; ++ai) {
            uint32_t h = uint32_t(seed) * 7919u + uint32_t(li * 131 + ai) * 31u;
            float k = k0 + (k1 - k0) * hashRandom(h);
            float ang = wd + (float(ai) + hashRandom(h + 1)) / float(na) * 2 * kPi;
            float kx = std::cos(ang), kz = std::sin(ang);
            float align = std::fabs(kx * wdir.x + kz * wdir.z);
            if (kx * wdir.x + kz * wdir.z < 0) align *= 0.15f; // little energy travelling against the wind
            float ph = std::exp(-1.f / (k * k * L * L)) / (k * k * k * k) * std::pow(align, std::max(0.f, alignment));
            float dk = k1 - k0, dth = 2 * kPi / float(na);
            float a = std::sqrt(std::max(0.f, 2 * ph * k * dk * dth));
            Wave w{kx * k, kz * k, k, a, std::sqrt(g * k), hashRandom(h + 2) * 2 * kPi, 0};
            var += 0.5 * double(a) * a;
            waves.push_back(w);
        }
    }
    // scale to the Pierson-Moskowitz significant wave height Hs = 0.21 U^2 / g  (Hs = 4 sigma)
    float hs = 0.21f * windSpeed * windSpeed / g;
    float scale = var > 0 ? float(hs / 4.0 / std::sqrt(var)) * amplitude : 0.f;
    float sumKA = 0;
    for (auto& w : waves) { w.amp *= scale; sumKA += w.k * w.amp; }
    for (auto& w : waves) w.q = sumKA > 0 ? std::clamp(chop, 0.f, 1.f) / sumKA : 0.f; // keeps sum(q k a) <= chop <= 1
    return waves;
}

void cookOcean(CookContext& c) {
    Geometry& o = c.out;
    if (c.hasInput(0)) o = c.input(0);
    else {
        const float size = std::max(0.1f, c.f("gridsize"));
        const int res = std::clamp(c.i("gridres"), 2, 1024);
        for (int j = 0; j <= res; ++j)
            for (int i = 0; i <= res; ++i) o.addPoint(Vec3((float(i) / float(res) - 0.5f) * size, 0, (float(j) / float(res) - 0.5f) * size));
        for (int j = 0; j < res; ++j)
            for (int i = 0; i < res; ++i) {
                int a = j * (res + 1) + i;
                o.addPrim({a, a + res + 1, a + res + 2, a + 1});
            }
    }
    auto waves = oceanWaves(std::max(0.f, c.f("windspeed")), c.f("winddir"), std::max(0.f, c.f("amplitude")), c.f("chop"),
                            std::clamp(c.i("waves"), 1, 4096), std::max(0.01f, c.f("minwavelength")), c.i("seed"), c.f("alignment"));
    const float t = c.time() + c.f("timeoffset");
    Attrib& N = o.addAttrib(AttribClass::Point, "N", 3);
    Attrib& foam = o.addAttrib(AttribClass::Point, "foam", 1);
    Attrib& v = o.addAttrib(AttribClass::Point, "v", 3);
    o.syncAttribSizes();
    const float foamStart = c.f("foamthreshold");
    const bool color = c.b("setcolor");
    Attrib* Cd = color ? &o.addAttrib(AttribClass::Point, "Cd", 3) : nullptr;
    o.syncAttribSizes();
    const Vec3 water = c.v("watercolor");
    parallelFor(int(o.numPoints()), [&](int b0, int b1) {
        for (int i = b0; i < b1; ++i) {
            Vec3 p0 = o.P[size_t(i)];
            Vec3 d(0), vel(0);
            float hx = 0, hz = 0; // -dy/dx0, -dy/dz0
            float jxx = 1, jzz = 1, jxz = 0;
            for (const Wave& w : waves) {
                float th = w.kx * p0.x + w.kz * p0.z - w.omega * t + w.phase;
                float s = std::sin(th), co = std::cos(th);
                float dx = w.kx / w.k, dz = w.kz / w.k;
                float qa = w.q * w.amp;
                d.x += qa * dx * co;
                d.z += qa * dz * co;
                d.y += w.amp * co;
                // derivatives (velocity is d/dt)
                vel.x += qa * dx * s * w.omega;
                vel.z += qa * dz * s * w.omega;
                vel.y += w.amp * s * w.omega;
                float ka = w.k * w.amp;
                hx += dx * ka * s;
                hz += dz * ka * s;
                jxx -= w.q * ka * dx * dx * s;
                jzz -= w.q * ka * dz * dz * s;
                jxz -= w.q * ka * dx * dz * s;
            }
            // exact normal: cross of the displaced surface's tangents d/dz0 and d/dx0
            Vec3 Tx(jxx, -hx, jxz), Tz(jxz, -hz, jzz);
            o.P[size_t(i)] = p0 + d;
            N.setV(size_t(i), normalize(cross(Tz, Tx)));
            v.setV(size_t(i), vel);
            float J = jxx * jzz - jxz * jxz;
            float f = std::clamp((foamStart - J) / std::max(1e-3f, foamStart), 0.f, 1.f);
            foam.setF(size_t(i), f);
            if (Cd) Cd->setV(size_t(i), lerp(water, Vec3(0.9f, 0.93f, 0.95f), f));
        }
    });
}

} // namespace

float flipTestStep(std::vector<Vec3>& P, std::vector<Vec3>& V, Vec3 origin, int n, float dx, float dt, Vec3 gravity, int* substeps) {
    MacGrid g;
    g.init(n, n, n, origin, dx);
    FlipParams fp{gravity, 0.95f, 0, 1.5f, 20, false, 0.f};
    FlipStats s = flipFrame(g, P, V, dt, fp);
    if (substeps) *substeps = s.substeps;
    return s.maxDiv;
}

void registerFluidNodes(std::vector<NodeType>& types) {
    {
        NodeType t; t.name = "flipsolver"; t.label = "FLIP Solver"; t.category = "Fluids";
        t.description = "Liquid simulation: fills the source with particles and moves them as an incompressible fluid inside a container.";
        t.minInputs = 1; t.maxInputs = 2;
        t.inputLabels = {"Fluid source (closed mesh)", "Collider (closed mesh, static)"};
        t.timeDependent = true;
        t.parms = {
            I("startframe", "Start Frame", 1, 1, 240),
            F("voxelsize", "Voxel Size", 0.08f, 0.01f, 0.5f, "Grid spacing; 8 particles per voxel"),
            V("domaincenter", "Container Center", Vec3(0, 1, 0), -10, 10),
            V("domainsize", "Container Size", Vec3(4, 4, 4), 0.1f, 20),
            T("opentop", "Open Top", false, "Particles that leave through the top are removed"),
            M("emission", "Source", {"Fill Once at Start", "Keep Emitting"}),
            V("vel", "Source Velocity", Vec3(0), -10, 10),
            I("maxpts", "Max Particles", 2000000, 1000, 10000000),
            V("gravity", "Gravity", Vec3(0, -9.81f, 0), -20, 20),
            F("flipratio", "FLIP Ratio", 0.95f, 0, 1, "1 = lively FLIP, 0 = smooth, damped PIC"),
            F("viscosity", "Viscosity (m^2/s)", 0, 0, 1),
            F("volcorrect", "Particle Separation", 0.5f, 0, 1, "Pushes apart bunched-up particles so the liquid keeps its volume"),
            F("cfl", "CFL Condition", 2, 0.5f, 6, "Voxels a particle may travel per substep"),
            I("maxsubsteps", "Max Substeps", 10, 1, 50),
            I("seed", "Seed", 0, 0, 100),
            T("colorspeed", "Color by Speed", true),
            V("color", "Water Color", Vec3(0.1f, 0.35f, 0.8f), 0, 1),
            F("speedmax", "Speed for White", 4, 0.1f, 20),
        };
        for (auto& p : t.parms) if (p.name == "color") p.help = "color";
        t.cook = cookFlip;
        types.push_back(t);
    }
    {
        NodeType t; t.name = "particlefluidsurface"; t.label = "Particle Fluid Surface"; t.category = "Fluids";
        t.description = "Builds a closed, smooth mesh around fluid particles.";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {
            F("separation", "Particle Separation", 0, 0, 0.5f, "0 = from the particles' pscale (FLIP output)"),
            F("radiusscale", "Radius Scale", 1.2f, 0.5f, 4, "1 = surface at the outer particles' edge; more fills gaps"),
            F("voxelscale", "Voxel Scale", 0.6f, 0.2f, 2, "Voxel size relative to the separation"),
            I("smooth", "Smoothing", 2, 0, 10),
            T("transferv", "Transfer Velocity", true),
        };
        t.cook = cookSurface;
        types.push_back(t);
    }
    {
        NodeType t; t.name = "ocean"; t.label = "Ocean"; t.category = "Fluids";
        t.description = "Animated ocean surface from a wind-driven wave spectrum (Gerstner waves with choppiness and foam).";
        t.minInputs = 0; t.maxInputs = 1;
        t.inputLabels = {"Surface to displace (optional, flat in XZ)"};
        t.timeDependent = true;
        t.parms = {
            F("gridsize", "Grid Size", 40, 1, 200),
            I("gridres", "Grid Resolution", 200, 2, 1000),
            F("windspeed", "Wind Speed (m/s)", 8, 0, 30),
            F("winddir", "Wind Direction (deg)", 0, -180, 180),
            F("alignment", "Wind Alignment", 2, 0, 8),
            F("amplitude", "Amplitude Scale", 1, 0, 3),
            F("chop", "Choppiness", 0.7f, 0, 1),
            I("waves", "Wave Count", 256, 1, 2048),
            F("minwavelength", "Shortest Wavelength", 0.4f, 0.05f, 10),
            F("timeoffset", "Time Offset", 0, -100, 100),
            I("seed", "Seed", 0, 0, 100),
            F("foamthreshold", "Foam Threshold", 0.4f, 0, 1, "Foam appears where the surface folds (Jacobian below this)"),
            T("setcolor", "Set Color", true),
            V("watercolor", "Water Color", Vec3(0.02f, 0.12f, 0.2f), 0, 1),
        };
        for (auto& p : t.parms) if (p.name == "watercolor") p.help = "color";
        t.cook = cookOcean;
        types.push_back(t);
    }
}

} // namespace hd

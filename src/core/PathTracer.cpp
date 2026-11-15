// Progressive CPU path tracer.
//
// Surfaces use a principled material: Lambert diffuse under a GGX specular layer, GGX metal, and a rough
// GGX dielectric for glass and water (exact Fresnel, refraction, Walter et al. 2007 transmission), sampled
// with visible-normal importance sampling (Heitz 2018). Subsurface scattering is a random walk inside closed
// meshes. Base colour, roughness, metallic and normal maps are read through the point "uv" attribute
// (UDIM tiles supported). Lights are sampled directly every bounce; environment maps are importance sampled
// and combined with BSDF sampling by multiple importance sampling. The camera supports thin-lens depth of
// field and velocity motion blur, and the renderer writes albedo, normal, depth and alpha passes plus an
// optional denoised image.
#include "PathTracer.h"
#include "Denoise.h"
#include "Image.h"
#include "Volume.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace hd {
namespace {

struct Ray { Vec3 o, d; float time = 0; }; // time in [-0.5, 0.5] of the shutter interval

struct MatInst {
    float rough = 0.5f, metal = 0, emit = 0, spec = 0.5f, transmit = 0, ior = 1.5f, sss = 0;
    Vec3 sssRadius{0.1f, 0.05f, 0.02f};
    int texBase = 0, texRough = 0, texMetal = 0, texNormal = 0;
};

struct Tri {
    Vec3 p0, e1, e2;
    Vec3 n0, n1, n2;
    Vec3 c0, c1, c2;
    Vec3 uv0, uv1, uv2;
    Vec3 tangent; float bsign = 1;
    Vec3 m0, dm1, dm2; // motion over the shutter: vertex 0 displacement and the edge changes
    int mat = 0;
    bool moving = false;
};
struct Sphere {
    Vec3 c; float r;
    Vec3 col; Vec3 motion;
    int mat = 0;
    bool moving = false;
};
struct BvhNode {
    Vec3 mn, mx;
    int left = -1, right = -1, first = 0, count = 0;
};
struct Hit {
    float t = 1e30f;
    int prim = -1;
    float u = 0, v = 0;
};

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed * 747796405u + 2891336453u) { next(); }
    float next() {
        s = s * 747796405u + 2891336453u;
        uint32_t w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
        w = (w >> 22u) ^ w;
        return float(w >> 8) * (1.f / 16777216.f);
    }
};

inline Vec3 toLinear(const Vec3& c) {
    return {std::pow(std::max(c.x, 0.f), 2.2f), std::pow(std::max(c.y, 0.f), 2.2f), std::pow(std::max(c.z, 0.f), 2.2f)};
}
inline float maxc(const Vec3& v) { return std::max(v.x, std::max(v.y, v.z)); }
inline float lum(const Vec3& c) { return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z; }

void basis(const Vec3& n, Vec3& t, Vec3& b) {
    t = std::fabs(n.x) > 0.5f ? normalize(cross(n, Vec3(0, 1, 0))) : normalize(cross(n, Vec3(1, 0, 0)));
    b = cross(n, t);
}

// ================================================================== microfacet BSDF (local frame, z = normal)

inline float sqr(float x) { return x * x; }
inline Vec3 reflectZ(const Vec3& w) { return Vec3(-w.x, -w.y, w.z); }

float frDielectric(float cosi, float eta) {
    cosi = std::clamp(cosi, -1.f, 1.f);
    if (cosi < 0) { eta = 1.f / eta; cosi = -cosi; }
    float sin2i = 1 - cosi * cosi, sin2t = sin2i / (eta * eta);
    if (sin2t >= 1) return 1.f;
    float cost = std::sqrt(std::max(0.f, 1 - sin2t));
    float rpar = (eta * cosi - cost) / (eta * cosi + cost), rperp = (cosi - eta * cost) / (cosi + eta * cost);
    return (rpar * rpar + rperp * rperp) * 0.5f;
}

inline Vec3 schlick(const Vec3& F0, float c) {
    float m = std::pow(std::clamp(1.f - c, 0.f, 1.f), 5.f);
    return F0 + (Vec3(1) - F0) * m;
}

// Refracts w (pointing away from the surface) about normal n (same side as w) with eta = n_t / n_i.
bool refractDir(const Vec3& wi, Vec3 n, float eta, Vec3& wt) {
    float cosi = dot(n, wi);
    if (cosi < 0) { eta = 1.f / eta; cosi = -cosi; n = -n; }
    float sin2i = std::max(0.f, 1 - cosi * cosi), sin2t = sin2i / (eta * eta);
    if (sin2t >= 1) return false;
    float cost = std::sqrt(1 - sin2t);
    wt = -wi / eta + n * (cosi / eta - cost);
    return true;
}

struct Ggx {
    float a; // alpha = roughness^2
    bool smooth() const { return a < 1e-3f; }
    float D(const Vec3& wm) const {
        float c2 = wm.z * wm.z;
        float d = c2 * (a * a - 1) + 1;
        return a * a / (kPi * d * d);
    }
    float lambda(const Vec3& w) const {
        float c2 = w.z * w.z;
        if (c2 <= 0) return 0;
        float t2 = std::max(0.f, 1 - c2) / c2;
        return (std::sqrt(1 + a * a * t2) - 1) * 0.5f;
    }
    float G1(const Vec3& w) const { return 1.f / (1 + lambda(w)); }
    float G(const Vec3& wo, const Vec3& wi) const { return 1.f / (1 + lambda(wo) + lambda(wi)); }
    float Dvis(const Vec3& wo, const Vec3& wm) const {
        return G1(wo) / std::max(std::fabs(wo.z), 1e-7f) * D(wm) * std::fabs(dot(wo, wm));
    }
    // visible normal sampling (Heitz 2018)
    Vec3 sampleWm(const Vec3& wo, float u1, float u2) const {
        Vec3 wh = normalize(Vec3(a * wo.x, a * wo.y, wo.z));
        if (wh.z < 0) wh = -wh;
        Vec3 T1 = wh.z < 0.99999f ? normalize(cross(Vec3(0, 0, 1), wh)) : Vec3(1, 0, 0);
        Vec3 T2 = cross(wh, T1);
        float r = std::sqrt(u1), phi = 2 * kPi * u2;
        float px = r * std::cos(phi), py = r * std::sin(phi);
        float h = std::sqrt(std::max(0.f, 1 - px * px));
        float s = (1 + wh.z) * 0.5f;
        py = (1 - s) * h + s * py;
        float pz = std::sqrt(std::max(0.f, 1 - px * px - py * py));
        Vec3 nh = T1 * px + T2 * py + wh * pz;
        return normalize(Vec3(a * nh.x, a * nh.y, std::max(1e-6f, nh.z)));
    }
};

// One shading point's material, in the local frame. All directions point away from the surface; wo.z > 0.
struct Bsdf {
    Vec3 base{0.5f};
    float metal = 0, transmit = 0, sss = 0, eta = 1.5f; // eta = n(other side) / n(this side)
    Ggx g{0.25f};
    Vec3 F0d{0.04f};
    // lobe weights and sampling probabilities (depend on wo only)
    float wMetal = 0, wDiel = 0, wOpaq = 0;
    float pDiff = 0, pSpec = 0, pMetal = 0, pDiel = 0, pSss = 0;

    void setup(const Vec3& wo) {
        wMetal = metal;
        wDiel = (1 - metal) * transmit;
        wOpaq = (1 - metal) * (1 - transmit);
        float Fs = lum(schlick(F0d, wo.z));
        pDiff = wOpaq * (1 - sss) * std::max(lum(base), 0.02f) * (1 - Fs);
        pSss = wOpaq * sss * std::max(lum(base), 0.02f);
        pSpec = wOpaq * Fs;
        pMetal = wMetal;
        pDiel = wDiel;
        float sum = pDiff + pSpec + pMetal + pDiel + pSss;
        if (sum <= 0) { pDiff = 1; sum = 1; }
        pDiff /= sum; pSpec /= sum; pMetal /= sum; pDiel /= sum; pSss /= sum;
    }

    Vec3 fDiffuse(const Vec3& wo, const Vec3& wi) const {
        if (wi.z <= 0) return Vec3(0);
        float a = 1 - lum(schlick(F0d, wo.z)), b = 1 - lum(schlick(F0d, wi.z));
        return base * (wOpaq * (1 - sss) * a * b / kPi);
    }
    // GGX reflection (non-smooth only); F from Schlick
    Vec3 fReflect(const Vec3& wo, const Vec3& wi, const Vec3& F0) const {
        if (wi.z <= 0 || g.smooth()) return Vec3(0);
        Vec3 wm = normalize(wo + wi);
        return schlick(F0, std::fabs(dot(wo, wm))) * (g.D(wm) * g.G(wo, wi) / (4 * wo.z * wi.z));
    }
    float pdfReflect(const Vec3& wo, const Vec3& wi) const {
        if (wi.z <= 0 || g.smooth()) return 0;
        Vec3 wm = normalize(wo + wi);
        return g.Dvis(wo, wm) / (4 * std::fabs(dot(wo, wm)));
    }
    // rough dielectric (pbrt-v4 style), radiance transport
    void dielectric(const Vec3& wo, const Vec3& wi, Vec3& f, float& pdf) const {
        f = Vec3(0); pdf = 0;
        if (g.smooth()) return;
        float coso = wo.z, cosi = wi.z;
        bool reflect = cosi * coso > 0;
        float etap = 1;
        if (!reflect) etap = coso > 0 ? eta : 1.f / eta;
        Vec3 wm = wi * etap + wo;
        if (cosi == 0 || coso == 0 || dot(wm, wm) == 0) return;
        wm = normalize(wm);
        if (wm.z < 0) wm = -wm;
        if (dot(wm, wi) * cosi < 0 || dot(wm, wo) * coso < 0) return;
        float R = frDielectric(dot(wo, wm), eta), T = 1 - R;
        if (reflect) {
            f = Vec3(g.D(wm) * g.G(wo, wi) * R / std::fabs(4 * cosi * coso));
            pdf = g.Dvis(wo, wm) / (4 * std::fabs(dot(wo, wm))) * R;
        } else {
            float denom = sqr(dot(wi, wm) + dot(wo, wm) / etap);
            float dwm = std::fabs(dot(wi, wm)) / denom;
            float ft = T * g.D(wm) * g.G(wo, wi) * std::fabs(dot(wi, wm) * dot(wo, wm) / (cosi * coso * denom));
            ft /= etap * etap;
            f = base * ft;
            pdf = g.Dvis(wo, wm) * dwm * T;
        }
    }

    // f (without cosine) and the mixture pdf over the non-delta lobes.
    void eval(const Vec3& wo, const Vec3& wi, Vec3& f, float& pdf) const {
        f = Vec3(0); pdf = 0;
        if (pDiff > 0 && wi.z > 0) { f += fDiffuse(wo, wi); pdf += pDiff * wi.z / kPi; }
        if (pSpec > 0) { f += fReflect(wo, wi, F0d) * wOpaq; pdf += pSpec * pdfReflect(wo, wi); }
        if (pMetal > 0) { f += fReflect(wo, wi, base) * wMetal; pdf += pMetal * pdfReflect(wo, wi); }
        if (pDiel > 0) { Vec3 fd; float pd; dielectric(wo, wi, fd, pd); f += fd * wDiel; pdf += pDiel * pd; }
    }

    enum Kind { None, Scatter, Delta, Subsurface };
    // Samples a direction. weight = f * |cos| / pdf (for Scatter/Delta). pdf is the mixture pdf (0 for delta).
    Kind sample(const Vec3& wo, Rng& rng, Vec3& wi, Vec3& weight, float& pdf) const {
        float u = rng.next();
        float u1 = rng.next(), u2 = rng.next();
        if (u < pSss) { weight = Vec3(wOpaq * sss / pSss); pdf = 0; return Subsurface; }
        u -= pSss;
        if (u < pDiff) {
            float r = std::sqrt(u1), ph = 2 * kPi * u2;
            wi = Vec3(r * std::cos(ph), r * std::sin(ph), std::sqrt(std::max(0.f, 1 - u1)));
        } else if (u < pDiff + pSpec || u < pDiff + pSpec + pMetal) {
            bool metalLobe = u >= pDiff + pSpec;
            if (g.smooth()) {
                wi = reflectZ(wo);
                Vec3 F = schlick(metalLobe ? base : F0d, wo.z);
                weight = F * ((metalLobe ? wMetal : wOpaq) / (metalLobe ? pMetal : pSpec));
                pdf = 0;
                return Delta;
            }
            Vec3 wm = g.sampleWm(wo, u1, u2);
            wi = wm * (2 * dot(wo, wm)) - wo;
            if (wi.z <= 0) return None;
        } else {
            // dielectric
            if (g.smooth()) {
                float R = frDielectric(wo.z, eta), T = 1 - R;
                if (rng.next() < R / (R + T)) {
                    wi = reflectZ(wo);
                    weight = Vec3(wDiel / pDiel);
                } else {
                    Vec3 wt;
                    if (!refractDir(wo, Vec3(0, 0, 1), eta, wt)) return None;
                    wi = wt;
                    float etap = wo.z > 0 ? eta : 1.f / eta;
                    weight = base * (wDiel / pDiel / (etap * etap));
                }
                pdf = 0;
                return Delta;
            }
            Vec3 wm = g.sampleWm(wo, u1, u2);
            float R = frDielectric(dot(wo, wm), eta), T = 1 - R;
            if (rng.next() < R / (R + T)) {
                wi = wm * (2 * dot(wo, wm)) - wo;
                if (wi.z * wo.z <= 0) return None;
            } else {
                Vec3 wt;
                if (!refractDir(wo, wm, eta, wt) || wt.z * wo.z >= 0) return None;
                wi = wt;
            }
        }
        Vec3 f;
        eval(wo, wi, f, pdf);
        if (pdf <= 0) return None;
        weight = f * (std::fabs(wi.z) / pdf);
        return Scatter;
    }
};

// ================================================================== environment map

struct EnvMap {
    Image img;
    std::vector<float> marginal, conditional; // CDFs: rows, then each row's columns
    float rotate = 0;
    Vec3 tint{1};
    float total = 0;
    bool valid() const { return img.w > 0; }

    void build() {
        const int W = img.w, H = img.h;
        conditional.assign(size_t(W + 1) * size_t(H), 0.f);
        marginal.assign(size_t(H + 1), 0.f);
        for (int y = 0; y < H; ++y) {
            float sinT = std::sin(kPi * (float(y) + 0.5f) / float(H));
            float* row = &conditional[size_t(y) * size_t(W + 1)];
            for (int x = 0; x < W; ++x) row[x + 1] = row[x] + std::max(1e-6f, lumAt(x, y)) * sinT;
            marginal[size_t(y) + 1] = marginal[size_t(y)] + row[W];
        }
        total = marginal[size_t(H)];
    }
    float lumAt(int x, int y) const {
        const float* p = &img.px[(size_t(y) * size_t(img.w) + size_t(x)) * size_t(img.c)];
        return img.c >= 3 ? 0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2] : p[0];
    }
    Vec3 texel(int x, int y) const {
        const float* p = &img.px[(size_t(y) * size_t(img.w) + size_t(x)) * size_t(img.c)];
        return img.c >= 3 ? Vec3(p[0], p[1], p[2]) : Vec3(p[0]);
    }
    // direction <-> (u, v): u = azimuth (rotated), v = 0 at +Y
    Vec3 dirFrom(float u, float v) const {
        float phi = (u - 0.5f) * 2 * kPi + radians(rotate), th = v * kPi;
        return Vec3(std::sin(th) * std::sin(phi), std::cos(th), std::sin(th) * std::cos(phi));
    }
    void uvFrom(const Vec3& d, float& u, float& v) const {
        float phi = std::atan2(d.x, d.z) - radians(rotate);
        u = phi / (2 * kPi) + 0.5f;
        u -= std::floor(u);
        v = std::acos(std::clamp(d.y, -1.f, 1.f)) / kPi;
    }
    Vec3 eval(const Vec3& d) const {
        float u, v;
        uvFrom(d, u, v);
        int x = std::min(img.w - 1, int(u * float(img.w))), y = std::min(img.h - 1, int(v * float(img.h)));
        return texel(x, y) * tint;
    }
    float pdf(const Vec3& d) const {
        float u, v;
        uvFrom(d, u, v);
        int x = std::min(img.w - 1, int(u * float(img.w))), y = std::min(img.h - 1, int(v * float(img.h)));
        float sinT = std::sin(kPi * (float(y) + 0.5f) / float(img.h));
        if (sinT <= 0 || total <= 0) return 0;
        float w = std::max(1e-6f, lumAt(x, y)) * sinT;
        float puv = w / total * float(img.w) * float(img.h);
        return puv / (2 * kPi * kPi * sinT);
    }
    Vec3 sample(float u1, float u2, float& pdfOut, Vec3& L) const {
        const int W = img.w, H = img.h;
        float t = u1 * total;
        int y = int(std::upper_bound(marginal.begin(), marginal.end(), t) - marginal.begin()) - 1;
        y = std::clamp(y, 0, H - 1);
        const float* row = &conditional[size_t(y) * size_t(W + 1)];
        float tr = u2 * row[W];
        int x = int(std::upper_bound(row, row + W + 1, tr) - row) - 1;
        x = std::clamp(x, 0, W - 1);
        float u = (float(x) + 0.5f) / float(W), v = (float(y) + 0.5f) / float(H);
        Vec3 d = dirFrom(u, v);
        float sinT = std::sin(kPi * v);
        float w = std::max(1e-6f, lumAt(x, y)) * sinT;
        pdfOut = sinT > 0 ? w / total * float(W) * float(H) / (2 * kPi * kPi * sinT) : 0.f;
        L = texel(x, y) * tint;
        return d;
    }
};

struct Scene {
    std::vector<Tri> tris;
    std::vector<Sphere> spheres;
    std::vector<MatInst> mats;
    std::vector<int> order; // prim ids: < tris.size() is a triangle, otherwise a sphere
    std::vector<BvhNode> nodes;
    std::vector<Volume> vols;
    RenderSettings set;
    std::vector<Light> lights;
    const Light* env = nullptr;
    EnvMap envMap;
    Vec3 envAmbient{0, 0, 0};
    std::map<int, std::unique_ptr<Texture>> textures; // by registered id (colour maps use id, data maps -id)
    std::string warnings;
    float shutterScale = 0; // velocity -> displacement over the shutter

    const Texture* tex(int id, bool color) const {
        auto it = textures.find(color ? id : -id);
        return it != textures.end() && it->second && it->second->valid() ? it->second.get() : nullptr;
    }
    void loadTexture(int id, bool color) {
        if (id <= 0) return;
        int key = color ? id : -id;
        if (textures.count(key)) return;
        auto t = std::make_unique<Texture>();
        std::string path = texturePath(id), err;
        if (!t->load(path, color, &err)) {
            warnings += (path.empty() ? std::string("Unknown texture") : err) + "\n";
            t.reset();
        }
        textures[key] = std::move(t);
    }

    // ------------------------------------------------------------ build
    void addTube(const Vec3& a, const Vec3& b, float r, const Vec3& ca, const Vec3& cb, int mat) {
        Vec3 axis = b - a;
        if (length(axis) < 1e-9f) return;
        Vec3 t, bt;
        basis(normalize(axis), t, bt);
        Vec3 off[3];
        for (int i = 0; i < 3; ++i) {
            float ang = 2 * kPi * float(i) / 3.f;
            off[i] = t * std::cos(ang) + bt * std::sin(ang);
        }
        for (int i = 0; i < 3; ++i) {
            int j = (i + 1) % 3;
            Vec3 a0 = a + off[i] * r, a1 = a + off[j] * r, b0 = b + off[i] * r, b1 = b + off[j] * r;
            Tri t1; t1.p0 = a0; t1.e1 = a1 - a0; t1.e2 = b1 - a0; t1.n0 = off[i]; t1.n1 = off[j]; t1.n2 = off[j]; t1.c0 = ca; t1.c1 = ca; t1.c2 = cb; t1.mat = mat;
            Tri t2; t2.p0 = a0; t2.e1 = b1 - a0; t2.e2 = b0 - a0; t2.n0 = off[i]; t2.n1 = off[j]; t2.n2 = off[i]; t2.c0 = ca; t2.c1 = cb; t2.c2 = cb; t2.mat = mat;
            tris.push_back(t1);
            tris.push_back(t2);
        }
    }

    void build(const Geometry& g, const RenderSettings& s, const std::vector<Light>& userLights) {
        set = s;
        lights = userLights;
        shutterScale = s.motionBlur ? s.shutter / std::max(1e-3f, s.fps) : 0.f;
        if (lights.empty()) { // no light nodes in the scene: default sun and sky
            float az = radians(s.sunAzimuth), el = radians(s.sunElevation);
            Light sun;
            sun.type = Light::Distant;
            sun.dir = -normalize(Vec3(std::cos(el) * std::sin(az), std::sin(el), std::cos(el) * std::cos(az)));
            sun.color = Vec3(1.f, 0.96f, 0.9f);
            sun.intensity = s.sunIntensity;
            sun.angle = 1.15f;
            Light sky;
            sky.type = Light::Env;
            sky.intensity = s.skyIntensity;
            if (sun.intensity > 0) lights.push_back(sun);
            if (sky.intensity > 0) lights.push_back(sky);
        }
        for (auto& l : lights)
            if (l.type == Light::Env) {
                env = &l;
                envAmbient = (l.sky ? Vec3(0.55f, 0.65f, 0.85f) : Vec3(1)) * l.color * (0.5f * l.intensity);
                if (!l.map.empty()) {
                    std::string err;
                    if (readImage(resolvePath(l.map), envMap.img, true, &err) && envMap.img.w > 1 && envMap.img.h > 1) {
                        envMap.rotate = l.rotate;
                        envMap.tint = l.color * l.intensity;
                        envMap.build();
                        Vec3 avg(0);
                        for (int y = 0; y < envMap.img.h; y += std::max(1, envMap.img.h / 32))
                            for (int x = 0; x < envMap.img.w; x += std::max(1, envMap.img.w / 64)) avg += envMap.texel(x, y);
                        envAmbient = avg / float(((envMap.img.h + std::max(1, envMap.img.h / 32) - 1) / std::max(1, envMap.img.h / 32)) *
                                                 ((envMap.img.w + std::max(1, envMap.img.w / 64) - 1) / std::max(1, envMap.img.w / 64))) * envMap.tint;
                    } else {
                        warnings += "Environment map: " + err + "\n";
                        envMap.img = Image();
                    }
                }
                break;
            }
        for (auto& v : g.volumes)
            if (v.density && v.density->size() == v.voxels() && v.voxels() > 0) vols.push_back(v);

        const Attrib* pN = g.findAttrib(AttribClass::Point, "N");
        const Attrib* qN = g.findAttrib(AttribClass::Prim, "N");
        const Attrib* pCd = g.findAttrib(AttribClass::Point, "Cd");
        const Attrib* qCd = g.findAttrib(AttribClass::Prim, "Cd");
        const Attrib* ps = g.findAttrib(AttribClass::Point, "pscale");
        const Attrib* pUv = g.findAttrib(AttribClass::Point, "uv");
        const Attrib* pV = shutterScale > 0 ? g.findAttrib(AttribClass::Point, "v") : nullptr;
        if (pN && pN->tuple != 3) pN = nullptr;
        if (qN && qN->tuple != 3) qN = nullptr;
        if (pV && pV->tuple != 3) pV = nullptr;
        struct MatAttr { const Attrib* prim; const Attrib* point; float def; };
        auto ma = [&](const char* name, float def) { return MatAttr{g.findAttrib(AttribClass::Prim, name), g.findAttrib(AttribClass::Point, name), def}; };
        MatAttr aRough = ma("rough", 0.65f), aMetal = ma("metal", 0.f), aEmit = ma("emit", 0.f), aSpec = ma("spec", 0.5f), aTrans = ma("transmit", 0.f),
                aIor = ma("ior", 1.5f), aSss = ma("sss", 0.f), aBase = ma("basemap", 0.f), aRmap = ma("roughmap", 0.f), aMmap = ma("metalmap", 0.f),
                aNmap = ma("normalmap", 0.f);
        const Attrib* qSssR = g.findAttrib(AttribClass::Prim, "sssradius");
        const Attrib* pSssR = g.findAttrib(AttribClass::Point, "sssradius");
        auto val = [](const MatAttr& a, int prim, int pt) {
            if (a.prim && prim >= 0) return a.prim->getF(size_t(prim));
            if (a.point && pt >= 0) return a.point->getF(size_t(pt));
            return a.def;
        };
        const Vec3 defCol = toLinear(Vec3(0.72f, 0.74f, 0.78f));
        auto colorOf = [&](int prim, int pt) {
            if (pCd) return toLinear(pCd->getV(size_t(pt)));
            if (qCd && prim >= 0) return toLinear(qCd->getV(size_t(prim)));
            return defCol;
        };
        std::unordered_map<std::string, int> matIndex;
        auto matFor = [&](int prim, int pt, bool hasBaseColorAttr) {
            MatInst m;
            m.rough = std::clamp(val(aRough, prim, pt), 0.02f, 1.f);
            m.metal = std::clamp(val(aMetal, prim, pt), 0.f, 1.f);
            m.emit = std::max(0.f, val(aEmit, prim, pt));
            m.spec = std::clamp(val(aSpec, prim, pt), 0.f, 1.f);
            m.transmit = std::clamp(val(aTrans, prim, pt), 0.f, 1.f);
            m.ior = std::clamp(val(aIor, prim, pt), 1.0001f, 4.f);
            m.sss = std::clamp(val(aSss, prim, pt), 0.f, 1.f);
            if (qSssR && prim >= 0 && qSssR->tuple == 3) m.sssRadius = qSssR->getV(size_t(prim));
            else if (pSssR && pt >= 0 && pSssR->tuple == 3) m.sssRadius = pSssR->getV(size_t(pt));
            m.sssRadius = vmax(m.sssRadius, Vec3(1e-4f));
            m.texBase = int(val(aBase, prim, pt) + 0.5f);
            m.texRough = int(val(aRmap, prim, pt) + 0.5f);
            m.texMetal = int(val(aMmap, prim, pt) + 0.5f);
            m.texNormal = int(val(aNmap, prim, pt) + 0.5f);
            (void)hasBaseColorAttr;
            std::string key(reinterpret_cast<const char*>(&m), sizeof(MatInst));
            auto it = matIndex.find(key);
            if (it != matIndex.end()) return it->second;
            loadTexture(m.texBase, true);
            loadTexture(m.texRough, false);
            loadTexture(m.texMetal, false);
            loadTexture(m.texNormal, false);
            mats.push_back(m);
            matIndex[key] = int(mats.size()) - 1;
            return int(mats.size()) - 1;
        };
        const bool anyColor = pCd || qCd;
        auto uvOf = [&](int pt) { return pUv ? pUv->getV(size_t(pt)) : Vec3(0); };
        auto motionOf = [&](int pt) { return pV ? pV->getV(size_t(pt)) * shutterScale : Vec3(0); };
        std::vector<char> used(g.numPoints(), 0);
        for (size_t pi = 0; pi < g.numPrims(); ++pi) {
            const Prim& pr = g.prims[pi];
            for (int k = 0; k < pr.count; ++k) used[size_t(g.vertices[size_t(pr.start + k)])] = 1;
            if (pr.count < 2) continue;
            int first = g.vertices[size_t(pr.start)];
            int mat = matFor(int(pi), first, anyColor);
            const bool texColor = mats[size_t(mat)].texBase > 0;
            auto baseCol = [&](int prim, int pt) { return texColor && !anyColor ? Vec3(1) : colorOf(prim, pt); };
            if (pr.closed && pr.count >= 3) {
                Vec3 fn = qN ? qN->getV(pi) : g.primNormal(int(pi));
                auto nrm = [&](int pt) {
                    if (!pN) return fn;
                    Vec3 n = pN->getV(size_t(pt));
                    return dot(n, n) < 1e-12f ? fn : normalize(n);
                };
                for (int k = 1; k + 1 < pr.count; ++k) {
                    int a = first, b = g.vertices[size_t(pr.start + k)], c = g.vertices[size_t(pr.start + k + 1)];
                    Tri t;
                    t.p0 = g.P[size_t(a)]; t.e1 = g.P[size_t(b)] - g.P[size_t(a)]; t.e2 = g.P[size_t(c)] - g.P[size_t(a)];
                    t.n0 = nrm(a); t.n1 = nrm(b); t.n2 = nrm(c);
                    t.c0 = baseCol(int(pi), a); t.c1 = baseCol(int(pi), b); t.c2 = baseCol(int(pi), c);
                    t.uv0 = uvOf(a); t.uv1 = uvOf(b); t.uv2 = uvOf(c);
                    t.mat = mat;
                    if (pV) {
                        Vec3 ma0 = motionOf(a), mb = motionOf(b), mc = motionOf(c);
                        t.m0 = ma0; t.dm1 = mb - ma0; t.dm2 = mc - ma0;
                        t.moving = dot(ma0, ma0) + dot(mb, mb) + dot(mc, mc) > 1e-14f;
                    }
                    // tangent frame from the uv layout (for normal maps)
                    Vec3 duv1 = t.uv1 - t.uv0, duv2 = t.uv2 - t.uv0;
                    float det = duv1.x * duv2.y - duv2.x * duv1.y;
                    if (std::fabs(det) > 1e-12f) {
                        float r = 1.f / det;
                        Vec3 T = (t.e1 * duv2.y - t.e2 * duv1.y) * r;
                        Vec3 B = (t.e2 * duv1.x - t.e1 * duv2.x) * r;
                        t.tangent = normalize(T);
                        t.bsign = dot(cross(normalize(cross(t.e1, t.e2)), t.tangent), B) < 0 ? -1.f : 1.f;
                    }
                    if (length(cross(t.e1, t.e2)) > 1e-14f) tris.push_back(t);
                }
            } else {
                int ne = pr.closed ? pr.count : pr.count - 1;
                for (int k = 0; k < ne; ++k) {
                    int a = g.vertices[size_t(pr.start + k)], b = g.vertices[size_t(pr.start + (k + 1) % pr.count)];
                    Vec3 ca = anyColor ? colorOf(int(pi), a) : toLinear(Vec3(0.9f, 0.9f, 0.6f));
                    Vec3 cb = anyColor ? colorOf(int(pi), b) : ca;
                    addTube(g.P[size_t(a)], g.P[size_t(b)], std::max(1e-5f, s.curveWidth * 0.5f), ca, cb, mat);
                }
            }
        }
        for (size_t i = 0; i < g.numPoints(); ++i) {
            if (used[i]) continue;
            Sphere sp;
            sp.c = g.P[i];
            sp.r = std::max(1e-5f, ps ? ps->getF(i) : 0.02f);
            sp.col = pCd ? toLinear(pCd->getV(i)) : toLinear(Vec3(0.95f, 0.85f, 0.35f));
            sp.mat = matFor(-1, int(i), anyColor);
            if (pV) { sp.motion = motionOf(int(i)); sp.moving = dot(sp.motion, sp.motion) > 1e-14f; }
            spheres.push_back(sp);
        }
        if (mats.empty()) mats.push_back(MatInst());
        buildBvh();
    }

    void primBounds(int id, Vec3& mn, Vec3& mx) const {
        if (id < int(tris.size())) {
            const Tri& t = tris[size_t(id)];
            mn = Vec3(1e30f); mx = Vec3(-1e30f);
            for (float tau : {-0.5f, 0.5f}) {
                if (!t.moving && tau > 0) break;
                Vec3 off = t.moving ? t.m0 * tau : Vec3(0);
                Vec3 a = t.p0 + off;
                Vec3 b = a + t.e1 + (t.moving ? t.dm1 * tau : Vec3(0));
                Vec3 c = a + t.e2 + (t.moving ? t.dm2 * tau : Vec3(0));
                if (!t.moving) { a = t.p0; b = t.p0 + t.e1; c = t.p0 + t.e2; }
                mn = vmin(mn, vmin(a, vmin(b, c))); mx = vmax(mx, vmax(a, vmax(b, c)));
            }
        } else {
            const Sphere& s = spheres[size_t(id) - tris.size()];
            Vec3 c0 = s.c - s.motion * 0.5f, c1 = s.c + s.motion * 0.5f;
            mn = vmin(c0, c1) - Vec3(s.r); mx = vmax(c0, c1) + Vec3(s.r);
        }
    }

    void buildBvh() {
        size_t n = tris.size() + spheres.size();
        order.resize(n);
        std::vector<Vec3> cen(n), bmn(n), bmx(n);
        for (size_t i = 0; i < n; ++i) {
            order[i] = int(i);
            primBounds(int(i), bmn[i], bmx[i]);
            cen[i] = (bmn[i] + bmx[i]) * 0.5f;
        }
        nodes.clear();
        if (n == 0) return;
        nodes.reserve(n * 2);
        struct Job { int node, lo, hi; };
        std::vector<Job> stack;
        nodes.push_back(BvhNode());
        stack.push_back({0, 0, int(n)});
        while (!stack.empty()) {
            Job j = stack.back(); stack.pop_back();
            Vec3 mn(1e30f), mx(-1e30f), cmn(1e30f), cmx(-1e30f);
            for (int i = j.lo; i < j.hi; ++i) {
                int id = order[size_t(i)];
                mn = vmin(mn, bmn[size_t(id)]); mx = vmax(mx, bmx[size_t(id)]);
                cmn = vmin(cmn, cen[size_t(id)]); cmx = vmax(cmx, cen[size_t(id)]);
            }
            nodes[size_t(j.node)].mn = mn; nodes[size_t(j.node)].mx = mx;
            Vec3 ext = cmx - cmn;
            int axis = ext.x > ext.y ? (ext.x > ext.z ? 0 : 2) : (ext.y > ext.z ? 1 : 2);
            if (j.hi - j.lo <= 4 || ext[axis] < 1e-12f) {
                nodes[size_t(j.node)].first = j.lo;
                nodes[size_t(j.node)].count = j.hi - j.lo;
                continue;
            }
            int mid = (j.lo + j.hi) / 2;
            std::nth_element(order.begin() + j.lo, order.begin() + mid, order.begin() + j.hi,
                             [&](int a, int b) { return cen[size_t(a)][axis] < cen[size_t(b)][axis]; });
            int l = int(nodes.size());
            nodes.push_back(BvhNode());
            nodes.push_back(BvhNode());
            nodes[size_t(j.node)].left = l;
            nodes[size_t(j.node)].right = l + 1;
            stack.push_back({l, j.lo, mid});
            stack.push_back({l + 1, mid, j.hi});
        }
    }

    // ------------------------------------------------------------ intersection
    static bool slab(const Vec3& mn, const Vec3& mx, const Ray& r, const Vec3& inv, float tmax) {
        float t0 = 0, t1 = tmax;
        for (int a = 0; a < 3; ++a) {
            float ta = (mn[a] - r.o[a]) * inv[a], tb = (mx[a] - r.o[a]) * inv[a];
            if (ta > tb) std::swap(ta, tb);
            t0 = std::max(t0, ta); t1 = std::min(t1, tb);
            if (t0 > t1) return false;
        }
        return true;
    }

    // triangle geometry at the ray's time
    void triAt(const Tri& t, float time, Vec3& p0, Vec3& e1, Vec3& e2) const {
        if (!t.moving) { p0 = t.p0; e1 = t.e1; e2 = t.e2; return; }
        p0 = t.p0 + t.m0 * time; e1 = t.e1 + t.dm1 * time; e2 = t.e2 + t.dm2 * time;
    }

    bool hitPrim(int id, const Ray& r, Hit& h) const {
        if (id < int(tris.size())) {
            const Tri& t = tris[size_t(id)];
            Vec3 p0, e1, e2;
            triAt(t, r.time, p0, e1, e2);
            Vec3 pv = cross(r.d, e2);
            float det = dot(e1, pv);
            if (std::fabs(det) < 1e-12f) return false;
            float inv = 1.f / det;
            Vec3 tv = r.o - p0;
            float u = dot(tv, pv) * inv;
            if (u < 0 || u > 1) return false;
            Vec3 qv = cross(tv, e1);
            float v = dot(r.d, qv) * inv;
            if (v < 0 || u + v > 1) return false;
            float tt = dot(e2, qv) * inv;
            if (tt < 1e-4f || tt >= h.t) return false;
            h.t = tt; h.prim = id; h.u = u; h.v = v;
            return true;
        }
        const Sphere& s = spheres[size_t(id) - tris.size()];
        Vec3 c = s.moving ? s.c + s.motion * r.time : s.c;
        Vec3 oc = r.o - c;
        float b = dot(oc, r.d), cc = dot(oc, oc) - s.r * s.r;
        float disc = b * b - cc;
        if (disc < 0) return false;
        float sq = std::sqrt(disc);
        float tt = -b - sq;
        if (tt < 1e-4f) tt = -b + sq;
        if (tt < 1e-4f || tt >= h.t) return false;
        h.t = tt; h.prim = id;
        return true;
    }

    bool intersect(const Ray& r, Hit& h, bool any) const {
        if (nodes.empty()) return false;
        Vec3 inv(1.f / r.d.x, 1.f / r.d.y, 1.f / r.d.z);
        int stack[64];
        int sp = 0;
        stack[sp++] = 0;
        bool found = false;
        while (sp) {
            const BvhNode& n = nodes[size_t(stack[--sp])];
            if (!slab(n.mn, n.mx, r, inv, h.t)) continue;
            if (n.count) {
                for (int i = 0; i < n.count; ++i)
                    if (hitPrim(order[size_t(n.first + i)], r, h)) {
                        found = true;
                        if (any) return true;
                    }
            } else if (sp < 62) {
                stack[sp++] = n.left;
                stack[sp++] = n.right;
            }
        }
        return found;
    }

    // ------------------------------------------------------------ volumes
    static float sampleField(const Volume& v, const std::vector<float>& f, const Vec3& p) {
        float x = (p.x - v.origin.x) / v.dx - 0.5f, y = (p.y - v.origin.y) / v.dx - 0.5f, z = (p.z - v.origin.z) / v.dx - 0.5f;
        if (x < -0.5f || y < -0.5f || z < -0.5f || x > v.nx - 0.5f || y > v.ny - 0.5f || z > v.nz - 0.5f) return 0;
        x = std::clamp(x, 0.f, float(v.nx - 1)); y = std::clamp(y, 0.f, float(v.ny - 1)); z = std::clamp(z, 0.f, float(v.nz - 1));
        int i = std::min(int(x), v.nx - 2), j = std::min(int(y), v.ny - 2), k = std::min(int(z), v.nz - 2);
        if (i < 0 || j < 0 || k < 0) return 0;
        float fx = x - float(i), fy = y - float(j), fz = z - float(k);
        const float* q = f.data() + size_t(i) + size_t(v.nx) * (size_t(j) + size_t(v.ny) * size_t(k));
        size_t sx = size_t(v.nx), sxy = sx * size_t(v.ny);
        float c00 = q[0] + (q[1] - q[0]) * fx, c10 = q[sx] + (q[sx + 1] - q[sx]) * fx;
        float c01 = q[sxy] + (q[sxy + 1] - q[sxy]) * fx, c11 = q[sxy + sx] + (q[sxy + sx + 1] - q[sxy + sx]) * fx;
        float c0 = c00 + (c10 - c00) * fy, c1 = c01 + (c11 - c01) * fy;
        return c0 + (c1 - c0) * fz;
    }
    static bool boxRange(const Volume& v, const Ray& r, float tmax, float& t0, float& t1) {
        Vec3 mn = v.bmin(), mx = v.bmax();
        t0 = 0; t1 = tmax;
        for (int a = 0; a < 3; ++a) {
            float inv = 1.f / r.d[a];
            float ta = (mn[a] - r.o[a]) * inv, tb = (mx[a] - r.o[a]) * inv;
            if (ta > tb) std::swap(ta, tb);
            t0 = std::max(t0, ta); t1 = std::min(t1, tb);
            if (t0 >= t1) return false;
        }
        return true;
    }
    static constexpr float kExt = 6.f; // extinction per unit density, matches the viewport

    // Transmittance of all volumes along a ray segment (for shadow rays).
    float transmittance(const Ray& r, float tmax, Rng& rng) const {
        float T = 1;
        for (auto& v : vols) {
            float t0, t1;
            if (!boxRange(v, r, tmax, t0, t1)) continue;
            int n = std::clamp(int((t1 - t0) / (v.dx * 1.5f)) + 1, 1, 48);
            float step = (t1 - t0) / float(n), sum = 0, t = t0 + step * rng.next();
            for (int i = 0; i < n; ++i, t += step) sum += sampleField(v, *v.density, r.o + r.d * t);
            T *= std::exp(-sum * step * kExt * v.densityScale);
            if (T < 0.003f) return 0;
        }
        return T;
    }

    // Single scattering + emission along a camera/bounce ray segment.
    void march(const Ray& r, float tmax, Rng& rng, Vec3& L, float& T) const {
        L = Vec3(0); T = 1;
        const Vec3 amb = envAmbient;
        for (auto& v : vols) {
            float t0, t1;
            if (!boxRange(v, r, tmax, t0, t1)) continue;
            int n = std::clamp(int((t1 - t0) / v.dx) + 1, 1, 200);
            float step = (t1 - t0) / float(n), t = t0 + step * rng.next();
            const Vec3 albedo = toLinear(v.color);
            const bool hasHeat = v.fire > 0 && v.heat && v.heat->size() == v.density->size();
            for (int i = 0; i < n && T > 0.005f; ++i, t += step) {
                Vec3 p = r.o + r.d * t;
                float d = sampleField(v, *v.density, p);
                if (d > 0.003f) {
                    float a = 1.f - std::exp(-d * v.densityScale * step * kExt);
                    // light reaching this point through the volume, from every light
                    Vec3 lit = amb;
                    const float ls = v.dx * 1.5f;
                    for (auto& l : lights) {
                        Vec3 wi, E; float dist;
                        if (!sampleLight(l, p, rng, wi, dist, E)) continue;
                        float sum = 0;
                        for (int k = 1; k <= 8 && ls * float(k) < dist; ++k) sum += sampleField(v, *v.density, p + wi * (ls * float(k)));
                        lit += E * (std::exp(-sum * ls * kExt * v.densityScale * 0.7f) * 0.32f);
                    }
                    L += albedo * lit * (T * a);
                    T *= 1.f - a;
                }
                if (hasHeat) {
                    float h = sampleField(v, *v.heat, p);
                    if (h > 0.02f) L += Vec3(h * 3.2f, h * h * 1.7f, h * h * h * 0.7f) * (T * v.fire * step * 9.f);
                }
            }
        }
    }

    // ------------------------------------------------------------ lights
    // Picks a point on the light as seen from P. Returns false if the light cannot reach P.
    // wi: direction to the light, dist: distance to it, E: light arriving (before the surface's response).
    bool sampleLight(const Light& l, const Vec3& P, Rng& rng, Vec3& wi, float& dist, Vec3& E) const {
        switch (l.type) {
        case Light::Env: return false; // reached by bounced rays instead
        case Light::Distant: {
            Vec3 t, b, toL = -l.dir;
            basis(toL, t, b);
            float a = 2 * kPi * rng.next(), rr = radians(l.angle) * std::sqrt(rng.next());
            wi = normalize(toL + t * (std::cos(a) * rr) + b * (std::sin(a) * rr));
            dist = 1e30f;
            E = l.color * l.intensity;
            return true;
        }
        case Light::Point:
        case Light::Spot: {
            Vec3 lp = l.pos;
            if (l.radius > 0) {
                float z = rng.next() * 2 - 1, a = rng.next() * 2 * kPi, r = std::sqrt(std::max(0.f, 1 - z * z));
                lp += Vec3(r * std::cos(a), z, r * std::sin(a)) * (l.radius * std::cbrt(rng.next()));
            }
            Vec3 d = lp - P;
            dist = length(d);
            if (dist < 1e-5f) return false;
            wi = d / dist;
            E = l.color * (l.intensity / std::max(dist * dist, 1e-4f));
            if (l.type == Light::Spot) {
                float c = dot(-wi, l.dir);
                float inner = std::cos(radians(l.coneAngle * 0.5f)), outer = std::cos(radians(std::min(179.f, l.coneAngle * 0.5f + l.coneDelta)));
                float f = inner > outer ? std::clamp((c - outer) / (inner - outer), 0.f, 1.f) : (c >= inner ? 1.f : 0.f);
                f = f * f * (3 - 2 * f);
                if (f <= 0) return false;
                E *= f;
            }
            return true;
        }
        case Light::Area: {
            Vec3 lp = l.pos + l.ux * ((rng.next() - 0.5f) * l.sizeX) + l.uy * ((rng.next() - 0.5f) * l.sizeY);
            Vec3 d = lp - P;
            dist = length(d);
            if (dist < 1e-5f) return false;
            wi = d / dist;
            float cosL = dot(-wi, l.dir);
            if (cosL <= 0) return false; // behind the panel
            E = l.color * (l.intensity * cosL / std::max(dist * dist, 1e-4f));
            return true;
        }
        }
        return false;
    }


    // ------------------------------------------------------------ shading
    Vec3 sky(const Vec3& d) const {
        if (!env) return Vec3(0);
        if (envMap.valid()) return envMap.eval(d);
        if (!env->sky) return env->color * env->intensity;
        const Vec3 horizon(0.85f, 0.88f, 0.93f), zenith(0.36f, 0.50f, 0.80f), below(0.22f, 0.21f, 0.20f);
        Vec3 c = d.y >= 0 ? lerp(horizon, zenith, std::sqrt(std::min(1.f, d.y))) : lerp(horizon, below, std::min(1.f, -d.y * 5.f));
        return c * env->color * env->intensity;
    }

    struct SurfacePt {
        Vec3 pos, n, ng; // n: shading normal on the incoming side; ng: geometric normal on that side
        Vec3 base{0.5f};
        float rough = 0.5f, metal = 0;
        const MatInst* m = nullptr;
        bool front = true;
        bool closed = true;
    };

    void shade(const Ray& ray, const Hit& h, bool groundHit, SurfacePt& s) const {
        s.pos = ray.o + ray.d * h.t;
        static const MatInst groundMat = [] { MatInst m; m.rough = 0.9f; return m; }();
        if (groundHit) {
            s.n = s.ng = Vec3(0, 1, 0);
            s.base = Vec3(0.18f); s.rough = 0.9f; s.metal = 0; s.m = &groundMat; s.front = true;
            return;
        }
        if (h.prim < int(tris.size())) {
            const Tri& t = tris[size_t(h.prim)];
            const MatInst& m = mats[size_t(t.mat)];
            s.m = &m;
            Vec3 p0, e1, e2;
            triAt(t, ray.time, p0, e1, e2);
            float w = 1 - h.u - h.v;
            Vec3 gn = normalize(cross(e1, e2));
            s.front = dot(gn, ray.d) < 0;
            if (!s.front) gn = -gn;
            Vec3 n = normalize(t.n0 * w + t.n1 * h.u + t.n2 * h.v);
            if (dot(n, gn) < 0) n = -n;
            Vec3 uv = t.uv0 * w + t.uv1 * h.u + t.uv2 * h.v;
            s.base = t.c0 * w + t.c1 * h.u + t.c2 * h.v;
            s.rough = m.rough; s.metal = m.metal;
            float tx[4];
            if (const Texture* tb = m.texBase ? tex(m.texBase, true) : nullptr) { tb->sample(uv.x, uv.y, tx); s.base = s.base * Vec3(tx[0], tx[1], tx[2]); }
            if (const Texture* tr = m.texRough ? tex(m.texRough, false) : nullptr) { tr->sample(uv.x, uv.y, tx); s.rough = std::clamp(tx[0], 0.02f, 1.f); }
            if (const Texture* tm = m.texMetal ? tex(m.texMetal, false) : nullptr) { tm->sample(uv.x, uv.y, tx); s.metal = std::clamp(tx[0], 0.f, 1.f); }
            if (const Texture* tn = m.texNormal ? tex(m.texNormal, false) : nullptr) {
                if (dot(t.tangent, t.tangent) > 0.5f) {
                    tn->sample(uv.x, uv.y, tx);
                    Vec3 T = normalize(t.tangent - n * dot(n, t.tangent));
                    Vec3 B = cross(n, T) * t.bsign * (s.front ? 1.f : -1.f);
                    Vec3 mapped = normalize(T * (tx[0] * 2 - 1) + B * (tx[1] * 2 - 1) + n * std::max(0.05f, tx[2] * 2 - 1));
                    if (dot(mapped, mapped) > 0.5f) n = mapped;
                }
            }
            if (dot(n, ray.d) > 0) n = gn; // shading normal faces away: fall back to the geometric one
            s.n = n; s.ng = gn;
        } else {
            const Sphere& sp = spheres[size_t(h.prim) - tris.size()];
            const MatInst& m = mats[size_t(sp.mat)];
            s.m = &m;
            Vec3 c = sp.moving ? sp.c + sp.motion * ray.time : sp.c;
            Vec3 n = normalize(s.pos - c);
            s.front = dot(n, ray.d) < 0;
            if (!s.front) n = -n;
            s.n = s.ng = n;
            s.base = sp.col; s.rough = m.rough; s.metal = m.metal;
        }
    }

    Bsdf makeBsdf(const SurfacePt& s) const {
        Bsdf b;
        b.base = s.base;
        b.metal = s.metal;
        b.transmit = s.m->transmit;
        b.sss = s.m->sss;
        b.eta = s.front ? s.m->ior : 1.f / s.m->ior;
        b.g.a = std::max(1e-4f, s.rough * s.rough);
        b.F0d = Vec3(0.08f * s.m->spec);
        return b;
    }

    bool anyTransmissive() const {
        for (auto& m : mats) if (m.transmit > 0 && m.metal < 1) return true;
        return false;
    }
    bool hasGlass = false;

    // Light reaching along a shadow ray, through glass-like surfaces (tinted, refraction ignored) and volumes.
    Vec3 shadowTransmittance(Vec3 o, const Vec3& d, float dist, float time, Rng& rng) const {
        Vec3 T(1);
        if (!hasGlass) {
            Ray sr{o, d, time};
            Hit sh;
            sh.t = dist < 1e29f ? dist * 0.999f : 1e30f;
            if (intersect(sr, sh, true)) return Vec3(0);
        } else {
            float left = dist < 1e29f ? dist * 0.999f : 1e30f;
            for (int i = 0; i < 16; ++i) {
                Ray sr{o, d, time};
                Hit sh;
                sh.t = left;
                if (!intersect(sr, sh, false)) break;
                int mat = sh.prim < int(tris.size()) ? tris[size_t(sh.prim)].mat : spheres[size_t(sh.prim) - tris.size()].mat;
                const MatInst& m = mats[size_t(mat)];
                float tr = m.transmit * (1 - m.metal);
                if (tr <= 0) return Vec3(0);
                Vec3 tint(1);
                if (sh.prim < int(tris.size())) {
                    const Tri& t = tris[size_t(sh.prim)];
                    float w = 1 - sh.u - sh.v;
                    tint = t.c0 * w + t.c1 * sh.u + t.c2 * sh.v;
                } else tint = spheres[size_t(sh.prim) - tris.size()].col;
                T = T * tint * tr;
                if (maxc(T) < 1e-3f) return Vec3(0);
                o = o + d * (sh.t + 1e-4f);
                if (left < 1e29f) left -= sh.t + 1e-4f;
                if (left <= 0) break;
            }
        }
        if (!vols.empty()) T *= transmittance(Ray{o, d, time}, dist, rng);
        return T;
    }

    // Random walk inside a closed mesh from an entry point (subsurface scattering).
    bool subsurfaceWalk(const SurfacePt& s, const Ray& inRay, Rng& rng, Vec3& thr, Vec3& exitP, Vec3& exitN) const {
        Vec3 alpha, sigt;
        for (int c = 0; c < 3; ++c) {
            float A = std::clamp(s.base[c], 0.f, 0.999f);
            float q = 4.09712f + 4.20863f * A - std::sqrt(9.59217f + 41.6808f * A + 17.7126f * A * A);
            alpha[c] = std::clamp(1.f - q * q, 0.f, 0.9999f);
            sigt[c] = 1.f / std::max(1e-4f, s.m->sssRadius[c]);
        }
        Vec3 t, b;
        basis(-s.ng, t, b);
        float r1 = rng.next(), r2 = 2 * kPi * rng.next(), sr = std::sqrt(r1);
        Vec3 dir = normalize(t * (sr * std::cos(r2)) + b * (sr * std::sin(r2)) - s.ng * std::sqrt(1 - r1));
        Vec3 p = s.pos - s.ng * 1e-4f;
        thr = Vec3(1);
        for (int step = 0; step < 256; ++step) {
            int c = std::min(2, int(rng.next() * 3));
            float dist = -std::log(std::max(1e-12f, 1 - rng.next())) / sigt[c];
            Ray r{p, dir, inRay.time};
            Hit h;
            h.t = dist;
            if (intersect(r, h, false)) {
                Vec3 Tk(std::exp(-sigt.x * h.t), std::exp(-sigt.y * h.t), std::exp(-sigt.z * h.t));
                float pdf = (Tk.x + Tk.y + Tk.z) / 3.f;
                if (pdf <= 0) return false;
                thr = thr * Tk / pdf;
                exitP = p + dir * h.t;
                Vec3 ng;
                if (h.prim < int(tris.size())) {
                    Vec3 p0, e1, e2;
                    triAt(tris[size_t(h.prim)], inRay.time, p0, e1, e2);
                    ng = normalize(cross(e1, e2));
                } else {
                    const Sphere& sp = spheres[size_t(h.prim) - tris.size()];
                    ng = normalize(exitP - (sp.moving ? sp.c + sp.motion * inRay.time : sp.c));
                }
                exitN = dot(ng, dir) > 0 ? ng : -ng; // outward
                return true;
            }
            p = p + dir * dist;
            Vec3 Tk(std::exp(-sigt.x * dist), std::exp(-sigt.y * dist), std::exp(-sigt.z * dist));
            Vec3 f = alpha * sigt * Tk;
            float pdf = (sigt.x * Tk.x + sigt.y * Tk.y + sigt.z * Tk.z) / 3.f;
            if (pdf <= 0) return false;
            thr = thr * f / pdf;
            // isotropic phase function
            float z = rng.next() * 2 - 1, a = rng.next() * 2 * kPi, rr = std::sqrt(std::max(0.f, 1 - z * z));
            dir = Vec3(rr * std::cos(a), z, rr * std::sin(a));
            if (step > 6) {
                float q = std::clamp(maxc(thr), 0.05f, 0.95f);
                if (rng.next() > q) return false;
                thr = thr / q;
            }
        }
        return false;
    }

    struct Aov { Vec3 albedo{0}, normal{0}; float depth = 1e10f, alpha = 0; };

    // Direct light at a point for a given BSDF evaluator.
    template <class EvalFn>
    Vec3 directLight(const Vec3& pos, const Vec3& ngSide, float time, float tScale, Rng& rng, const EvalFn& evalFn) const {
        Vec3 L(0);
        for (auto& l : lights) {
            Vec3 wi, E; float dist;
            if (!sampleLight(l, pos, rng, wi, dist, E)) continue;
            Vec3 f; float pdfB, cosv;
            evalFn(wi, f, pdfB, cosv);
            if (maxc(f) <= 0) continue;
            Vec3 o = pos + ngSide * ((dot(wi, ngSide) >= 0 ? 1e-4f : -1e-4f) * tScale);
            Vec3 T = shadowTransmittance(o, wi, dist, time, rng);
            if (maxc(T) <= 0) continue;
            L += f * E * T * cosv;
        }
        if (envMap.valid()) {
            float pe; Vec3 Le;
            Vec3 wi = envMap.sample(rng.next(), rng.next(), pe, Le);
            if (pe > 0) {
                Vec3 f; float pdfB, cosv;
                evalFn(wi, f, pdfB, cosv);
                if (maxc(f) > 0) {
                    Vec3 o = pos + ngSide * ((dot(wi, ngSide) >= 0 ? 1e-4f : -1e-4f) * tScale);
                    Vec3 T = shadowTransmittance(o, wi, 1e30f, time, rng);
                    float w = pe * pe / (pe * pe + pdfB * pdfB);
                    L += f * Le * T * (cosv * w / pe);
                }
            }
        }
        return L;
    }

    // screenV: 0 at the bottom of the image, 1 at the top (for the backdrop gradient)
    Vec3 radiance(Ray ray, Rng& rng, float screenV, Aov* aov) const {
        Vec3 L(0), beta(1);
        bool specular = true;
        float prevPdf = 0;
        for (int bounce = 0; bounce <= set.bounces; ++bounce) {
            Hit h;
            bool hit = intersect(ray, h, false);
            bool groundHit = false;
            if (set.ground && ray.d.y < -1e-6f) {
                float tg = (set.groundY - ray.o.y) / ray.d.y;
                if (tg > 1e-4f && tg < h.t) { h.t = tg; groundHit = true; hit = true; }
            }
            if (!vols.empty()) {
                Vec3 Lv; float Tv;
                march(ray, hit ? h.t : 1e30f, rng, Lv, Tv);
                L += beta * Lv;
                beta *= Tv;
                if (aov && bounce == 0) aov->alpha = std::max(aov->alpha, 1.f - Tv);
                if (maxc(beta) < 1e-3f) break;
            }
            if (!hit) {
                if (envMap.valid()) {
                    Vec3 Le = envMap.eval(ray.d);
                    if (bounce == 0 || specular) L += beta * Le;
                    else {
                        float pe = envMap.pdf(ray.d);
                        L += beta * Le * (prevPdf * prevPdf / (prevPdf * prevPdf + pe * pe));
                    }
                } else if (bounce == 0) {
                    // camera sees a neutral studio backdrop (same gradient as the viewport); lighting still comes from the sky
                    float t = screenV * screenV * (3 - 2 * screenV);
                    L += beta * toLinear(lerp(Vec3(0.13f, 0.135f, 0.15f), Vec3(0.40f, 0.43f, 0.48f), t)) * (0.8f / std::max(set.exposure, 1e-3f));
                } else L += beta * sky(ray.d);
                break;
            }

            SurfacePt s;
            shade(ray, h, groundHit, s);
            if (aov && bounce == 0) { aov->albedo = s.base; aov->normal = s.n; aov->depth = h.t; aov->alpha = 1; }
            if (s.m->emit > 0 && !groundHit) L += beta * s.base * s.m->emit;

            const float tScale = std::max(1.f, h.t);
            Vec3 tt, bt;
            basis(s.n, tt, bt);
            auto toLocal = [&](const Vec3& v) { return Vec3(dot(v, tt), dot(v, bt), dot(v, s.n)); };
            auto toWorld = [&](const Vec3& v) { return tt * v.x + bt * v.y + s.n * v.z; };
            const Vec3 wo = toLocal(-ray.d);
            Bsdf bsdf = makeBsdf(s);
            bsdf.setup(wo);

            // direct light (shadow rays to each light; environment map with MIS)
            L += beta * directLight(s.pos, s.ng, ray.time, tScale, rng, [&](const Vec3& wi, Vec3& f, float& pdfB, float& cosv) {
                Vec3 wl = toLocal(wi);
                bsdf.eval(wo, wl, f, pdfB);
                cosv = std::fabs(wl.z);
                // geometric consistency: reflection must stay on the visible side, transmission must cross
                if ((dot(wi, s.ng) > 0) != (wl.z > 0)) f = Vec3(0);
            });

            Vec3 wiL, weight;
            float pdf = 0;
            Bsdf::Kind kind = bsdf.sample(wo, rng, wiL, weight, pdf);
            if (kind == Bsdf::None) break;
            if (kind == Bsdf::Subsurface) {
                Vec3 thr, ep, en;
                if (!subsurfaceWalk(s, ray, rng, thr, ep, en)) break;
                beta = beta * weight * thr;
                // light leaving at the exit point (diffuse transmission out of the surface)
                L += beta * directLight(ep, en, ray.time, tScale, rng, [&](const Vec3& wi, Vec3& f, float& pdfB, float& cosv) {
                    float c = dot(wi, en);
                    f = c > 0 ? Vec3(1.f / kPi) : Vec3(0);
                    pdfB = c > 0 ? c / kPi : 0.f;
                    cosv = std::max(0.f, c);
                });
                Vec3 t2, b2;
                basis(en, t2, b2);
                float r1 = rng.next(), r2 = 2 * kPi * rng.next(), sr = std::sqrt(r1);
                Vec3 dir = normalize(t2 * (sr * std::cos(r2)) + b2 * (sr * std::sin(r2)) + en * std::sqrt(1 - r1));
                prevPdf = std::sqrt(1 - r1) / kPi;
                specular = false;
                ray = Ray{ep + en * (1e-4f * tScale), dir, ray.time};
            } else {
                Vec3 wi = toWorld(wiL);
                beta = beta * weight;
                specular = kind == Bsdf::Delta;
                prevPdf = pdf;
                float side = dot(wi, s.ng) >= 0 ? 1.f : -1.f;
                ray = Ray{s.pos + s.ng * (side * 1e-4f * tScale), wi, ray.time};
            }
            if (bounce >= 2) {
                float q = std::clamp(maxc(beta), 0.05f, 0.95f);
                if (rng.next() > q) break;
                beta = beta / q;
            }
        }
        float m = lum(L);
        if (!(m == m)) return Vec3(0);
        if (m > 50.f) L = L * (50.f / m); // tame fireflies
        return L;
    }
};

inline unsigned char tone(float c, float exposure) {
    c *= exposure;
    c = (c * (2.51f * c + 0.03f)) / (c * (2.43f * c + 0.59f) + 0.14f); // ACES fit
    c = std::pow(std::clamp(c, 0.f, 1.f), 1.f / 2.2f);
    return (unsigned char)(c * 255.f + 0.5f);
}

} // namespace

// ---------------------------------------------------------------- PathTracer
struct PathTracer::Impl {
    std::thread thread;
    std::atomic<bool> cancel{false}, running{false};
    std::atomic<int> done{0};
    int target = 0;
    mutable std::mutex m;
    std::vector<uint8_t> display;
    int w = 0, h = 0;
    uint64_t version = 0;
    Buffers buffers;
    std::string warnings;
    std::chrono::steady_clock::time_point t0;
    std::atomic<double> elapsed{0};

    void run(std::shared_ptr<Scene> scene) {
        const RenderSettings& s = scene->set;
        const int W = s.width, H = s.height;
        const size_t N = size_t(W) * size_t(H);
        std::vector<float> accum(N * 3, 0.f), accum2(N, 0.f), alpha(N, 0.f), albedo(N * 3, 0.f), normal(N * 3, 0.f), depth(N, 0.f), hits(N, 0.f);
        Vec3 eye = s.eye();
        Vec3 fwd = normalize(s.camTarget - eye);
        if (length(fwd) < 0.5f) fwd = Vec3(0, 0, -1);
        Vec3 right = normalize(cross(fwd, Vec3(0, 1, 0)));
        if (length(right) < 0.5f) right = Vec3(1, 0, 0);
        Vec3 up = cross(right, fwd);
        const float th = std::tan(radians(s.camFov) * 0.5f), aspect = float(W) / float(H);
        // thin lens: focal length from the sensor height and field of view
        const float focalM = (s.sensorHeight * 0.5f / std::max(th, 1e-4f)) / 1000.f;
        const float lensR = s.dof ? focalM / (2.f * std::max(0.1f, s.fstop)) : 0.f;
        const float focus = std::max(1e-3f, s.focusDist);
        std::vector<uint8_t> img(N * 4, 255);
        for (int sidx = 0; sidx < s.samples && !cancel; ++sidx) {
            parallelFor(H, [&](int y0, int y1) {
                for (int y = y0; y < y1 && !cancel; ++y)
                    for (int x = 0; x < W; ++x) {
                        Rng rng(uint32_t(x + y * W) * 9781u + uint32_t(sidx) * 6271u + 17u);
                        float px = (float(x) + rng.next()) / float(W) * 2 - 1, py = 1 - (float(y) + rng.next()) / float(H) * 2;
                        Vec3 d = normalize(fwd + right * (px * th * aspect) + up * (py * th));
                        Ray r{eye, d, 0.f};
                        if (lensR > 0) {
                            Vec3 pf = eye + d * (focus / dot(d, fwd));
                            float rr = lensR * std::sqrt(rng.next()), ph = 2 * kPi * rng.next();
                            r.o = eye + right * (rr * std::cos(ph)) + up * (rr * std::sin(ph));
                            r.d = normalize(pf - r.o);
                        }
                        if (scene->shutterScale > 0) r.time = rng.next() - 0.5f;
                        Scene::Aov aov;
                        Vec3 c = scene->radiance(r, rng, py * 0.5f + 0.5f, &aov);
                        size_t i = size_t(y) * size_t(W) + size_t(x);
                        accum[i * 3] += c.x; accum[i * 3 + 1] += c.y; accum[i * 3 + 2] += c.z;
                        float l = lum(c);
                        accum2[i] += l * l;
                        alpha[i] += aov.alpha;
                        if (aov.alpha >= 1.f) {
                            albedo[i * 3] += aov.albedo.x; albedo[i * 3 + 1] += aov.albedo.y; albedo[i * 3 + 2] += aov.albedo.z;
                            normal[i * 3] += aov.normal.x; normal[i * 3 + 1] += aov.normal.y; normal[i * 3 + 2] += aov.normal.z;
                            depth[i] += aov.depth;
                            hits[i] += 1;
                        }
                    }
            });
            if (cancel) break;
            const int ns = sidx + 1;
            const bool last = ns == s.samples;
            Buffers b;
            b.w = W; b.h = H; b.samples = ns;
            b.beauty.resize(N * 3); b.alpha.resize(N); b.albedo.assign(N * 3, 0.f); b.normal.assign(N * 3, 0.f); b.depth.assign(N, 1e10f);
            std::vector<float> variance(N);
            const float inv = 1.f / float(ns);
            for (size_t i = 0; i < N; ++i) {
                for (int k = 0; k < 3; ++k) b.beauty[i * 3 + size_t(k)] = accum[i * 3 + size_t(k)] * inv * s.exposure;
                b.alpha[i] = alpha[i] * inv;
                float meanL = lum(Vec3(accum[i * 3], accum[i * 3 + 1], accum[i * 3 + 2])) * inv;
                variance[i] = std::max(0.f, accum2[i] * inv - meanL * meanL) / float(ns) * s.exposure * s.exposure;
                if (hits[i] > 0) {
                    float hi = 1.f / hits[i];
                    for (int k = 0; k < 3; ++k) b.albedo[i * 3 + size_t(k)] = albedo[i * 3 + size_t(k)] * hi;
                    Vec3 nn = normalize(Vec3(normal[i * 3], normal[i * 3 + 1], normal[i * 3 + 2]));
                    b.normal[i * 3] = nn.x; b.normal[i * 3 + 1] = nn.y; b.normal[i * 3 + 2] = nn.z;
                    b.depth[i] = depth[i] * hi;
                }
            }
            if (last && s.denoise) {
                DenoiseInput di;
                di.w = W; di.h = H;
                di.color = b.beauty.data(); di.albedo = b.albedo.data(); di.normal = b.normal.data(); di.depth = b.depth.data();
                di.variance = variance.data();
                b.denoised = denoiseImage(di);
            }
            const std::vector<float>& shown = b.denoised.empty() ? b.beauty : b.denoised;
            for (size_t i = 0; i < N; ++i) {
                img[i * 4] = tone(shown[i * 3], 1.f);
                img[i * 4 + 1] = tone(shown[i * 3 + 1], 1.f);
                img[i * 4 + 2] = tone(shown[i * 3 + 2], 1.f);
            }
            {
                std::lock_guard<std::mutex> lk(m);
                display = img;
                w = W; h = H;
                buffers = std::move(b);
                ++version;
            }
            done = ns;
            elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        }
        running = false;
    }
};

PathTracer::PathTracer() : impl_(new Impl()) {}
PathTracer::~PathTracer() { stop(); }

void PathTracer::start(const Geometry& geo, const RenderSettings& settings, const std::vector<Light>& lights) {
    stop();
    RenderSettings s = settings;
    s.width = std::clamp(s.width, 16, 4096);
    s.height = std::clamp(s.height, 16, 4096);
    s.samples = std::clamp(s.samples, 1, 100000);
    s.bounces = std::clamp(s.bounces, 0, 16);
    auto scene = std::make_shared<Scene>();
    scene->build(geo, s, lights);
    scene->hasGlass = scene->anyTransmissive();
    {
        std::lock_guard<std::mutex> lk(impl_->m);
        impl_->warnings = scene->warnings;
    }
    impl_->cancel = false;
    impl_->done = 0;
    impl_->target = s.samples;
    impl_->running = true;
    impl_->elapsed = 0;
    impl_->t0 = std::chrono::steady_clock::now();
    impl_->thread = std::thread([this, scene] { impl_->run(scene); });
}

void PathTracer::stop() {
    impl_->cancel = true;
    if (impl_->thread.joinable()) impl_->thread.join();
    impl_->running = false;
}
void PathTracer::wait() {
    if (impl_->thread.joinable()) impl_->thread.join();
}
bool PathTracer::running() const { return impl_->running; }
int PathTracer::samplesDone() const { return impl_->done; }
int PathTracer::samplesTarget() const { return impl_->target; }
double PathTracer::secondsElapsed() const { return impl_->elapsed; }

bool PathTracer::fetch(std::vector<uint8_t>& rgba, int& w, int& h, uint64_t& version) {
    std::lock_guard<std::mutex> lk(impl_->m);
    if (impl_->version == version || impl_->display.empty()) return false;
    rgba = impl_->display;
    w = impl_->w; h = impl_->h;
    version = impl_->version;
    return true;
}

bool PathTracer::fetchBuffers(Buffers& out) const {
    std::lock_guard<std::mutex> lk(impl_->m);
    if (impl_->buffers.w == 0) return false;
    out = impl_->buffers;
    return true;
}

std::string PathTracer::warnings() const {
    std::lock_guard<std::mutex> lk(impl_->m);
    return impl_->warnings;
}

bool writePng(const std::string& path, const std::vector<uint8_t>& rgba, int w, int h, std::string* error) {
    if (w <= 0 || h <= 0 || rgba.size() < size_t(w) * size_t(h) * 4) { if (error) *error = "Nothing to save"; return false; }
    std::vector<uint8_t> rgb(size_t(w) * size_t(h) * 3);
    for (size_t i = 0, n = size_t(w) * size_t(h); i < n; ++i) { rgb[i * 3] = rgba[i * 4]; rgb[i * 3 + 1] = rgba[i * 4 + 1]; rgb[i * 3 + 2] = rgba[i * 4 + 2]; }
    return writePng8(path, rgb, w, h, 3, error);
}

bool writeExrPasses(const std::string& path, const PathTracer::Buffers& b, bool half, std::string* error) {
    if (b.w <= 0) { if (error) *error = "Nothing to save"; return false; }
    const size_t n = size_t(b.w) * size_t(b.h);
    auto plane = [&](const std::vector<float>& src, int stride, int k) {
        std::vector<float> p(n);
        for (size_t i = 0; i < n; ++i) p[i] = src[i * size_t(stride) + size_t(k)];
        return p;
    };
    const std::vector<float>& col = b.denoised.empty() ? b.beauty : b.denoised;
    std::vector<std::vector<float>> store;
    std::vector<ExrChannel> ch;
    auto add = [&](const std::string& name, std::vector<float> p) { store.push_back(std::move(p)); ch.push_back({name, nullptr}); };
    add("R", plane(col, 3, 0)); add("G", plane(col, 3, 1)); add("B", plane(col, 3, 2)); add("A", b.alpha);
    if (!b.denoised.empty()) { add("beauty.R", plane(b.beauty, 3, 0)); add("beauty.G", plane(b.beauty, 3, 1)); add("beauty.B", plane(b.beauty, 3, 2)); }
    add("albedo.R", plane(b.albedo, 3, 0)); add("albedo.G", plane(b.albedo, 3, 1)); add("albedo.B", plane(b.albedo, 3, 2));
    add("N.X", plane(b.normal, 3, 0)); add("N.Y", plane(b.normal, 3, 1)); add("N.Z", plane(b.normal, 3, 2));
    add("Z", b.depth);
    for (size_t i = 0; i < ch.size(); ++i) ch[i].data = store[i].data();
    return writeExr(path, b.w, b.h, ch, half, error);
}

} // namespace hd

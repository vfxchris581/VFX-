// Compositing nodes (COPs). Images travel through the graph as Geometry::images (linear, premultiplied RGBA
// plus optional depth), so image nodes wire into the same networks as geometry: render a SOP network,
// grade it, key a plate, merge them by depth and write the result.
#include "Compositing.h"
#include "Image.h"
#include "Lights.h"
#include "Node.h"
#include "PathTracer.h"
#include "Volume.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>

namespace hd {

std::string expandFrame(const std::string& path, float frame) {
    std::string out = path;
    size_t pos;
    while ((pos = out.find("$F")) != std::string::npos) {
        int pad = 1;
        size_t len = 2;
        if (pos + 2 < out.size() && out[pos + 2] >= '1' && out[pos + 2] <= '9') { pad = out[pos + 2] - '0'; len = 3; }
        char num[32];
        std::snprintf(num, sizeof num, "%0*d", pad, int(std::lround(frame)));
        out.replace(pos, len, num);
    }
    return out;
}

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
ParmDef FileP(const char* n, const char* l, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::File; p.help = help; return p;
}
ParmDef ColorP(const char* n, const char* l, Vec3 d) { ParmDef p = V(n, l, d, 0, 1); p.help = "color"; return p; }

// Working image: premultiplied RGBA floats + optional depth
struct Img {
    int w = 0, h = 0;
    std::vector<float> px;
    std::vector<float> z;
    float* at(int x, int y) { return &px[(size_t(y) * size_t(w) + size_t(x)) * 4]; }
    const float* at(int x, int y) const { return &px[(size_t(y) * size_t(w) + size_t(x)) * 4]; }
    void alloc(int W, int H) { w = W; h = H; px.assign(size_t(W) * size_t(H) * 4, 0.f); }
    void sample(float x, float y, float out[4]) const { // bilinear, transparent outside
        out[0] = out[1] = out[2] = out[3] = 0;
        float fx = x - 0.5f, fy = y - 0.5f;
        int x0 = int(std::floor(fx)), y0 = int(std::floor(fy));
        float tx = fx - float(x0), ty = fy - float(y0);
        for (int k = 0; k < 4; ++k) {
            int xi = x0 + (k & 1), yi = y0 + (k >> 1);
            if (xi < 0 || yi < 0 || xi >= w || yi >= h) continue;
            float wgt = ((k & 1) ? tx : 1 - tx) * ((k >> 1) ? ty : 1 - ty);
            const float* p = at(xi, yi);
            for (int c = 0; c < 4; ++c) out[c] += p[c] * wgt;
        }
    }
};

bool getImg(CookContext& c, int input, Img& img, const char* what = "image") {
    if (!c.hasInput(input)) { c.error(std::string("Connect an ") + what + " to input " + std::to_string(input + 1) + "."); return false; }
    const Geometry& g = c.input(input);
    if (g.images.empty() || !g.images[0].valid()) { c.error("Input " + std::to_string(input + 1) + " has no image."); return false; }
    const ImageLayer& L = g.images[0];
    img.w = L.w; img.h = L.h;
    img.px = *L.rgba;
    if (L.depth && L.depth->size() == L.pixels()) img.z = *L.depth;
    return true;
}

void putImg(CookContext& c, Img&& img, const std::string& name = "C") {
    ImageLayer L;
    L.name = name;
    L.w = img.w; L.h = img.h;
    L.rgba = std::make_shared<const std::vector<float>>(std::move(img.px));
    if (!img.z.empty()) L.depth = std::make_shared<const std::vector<float>>(std::move(img.z));
    c.out.clear();
    c.out.images.push_back(std::move(L));
}

inline float lumv(float r, float g, float b) { return 0.2126f * r + 0.7152f * g + 0.0722f * b; }

// separable gaussian on all four channels
void gaussian(Img& img, float sigma) {
    if (sigma <= 0.05f || img.w == 0) return;
    const int r = std::min(1000, int(std::ceil(sigma * 3)));
    std::vector<float> k(size_t(2 * r + 1));
    float sum = 0;
    for (int i = -r; i <= r; ++i) { k[size_t(i + r)] = std::exp(-0.5f * float(i * i) / (sigma * sigma)); sum += k[size_t(i + r)]; }
    for (auto& v : k) v /= sum;
    std::vector<float> tmp(img.px.size());
    const int W = img.w, H = img.h;
    parallelFor(H, [&](int y0, int y1) {
        for (int y = y0; y < y1; ++y)
            for (int x = 0; x < W; ++x) {
                float a[4] = {0, 0, 0, 0};
                for (int i = -r; i <= r; ++i) {
                    int xx = std::clamp(x + i, 0, W - 1);
                    const float* p = img.at(xx, y);
                    for (int c = 0; c < 4; ++c) a[c] += p[c] * k[size_t(i + r)];
                }
                for (int c = 0; c < 4; ++c) tmp[(size_t(y) * size_t(W) + size_t(x)) * 4 + size_t(c)] = a[c];
            }
    });
    parallelFor(H, [&](int y0, int y1) {
        for (int y = y0; y < y1; ++y)
            for (int x = 0; x < W; ++x) {
                float a[4] = {0, 0, 0, 0};
                for (int i = -r; i <= r; ++i) {
                    int yy = std::clamp(y + i, 0, H - 1);
                    const float* p = &tmp[(size_t(yy) * size_t(W) + size_t(x)) * 4];
                    for (int c = 0; c < 4; ++c) a[c] += p[c] * k[size_t(i + r)];
                }
                float* o = img.at(x, y);
                for (int c = 0; c < 4; ++c) o[c] = a[c];
            }
    });
}

// ------------------------------------------------------------------ generators
struct FileCache { std::string path; Img img; std::string err; };

void cookFile(CookContext& c) {
    FileCache& st = c.state<FileCache>();
    std::string path = resolvePath(expandFrame(c.s("file"), c.frame()));
    if (path.empty()) { c.error("Choose an image file."); return; }
    if (st.path != path) {
        st.path = path;
        st.err.clear();
        st.img = Img();
        Image im;
        if (!readImage(path, im, true, &st.err)) st.path.clear();
        else {
            st.img.alloc(im.w, im.h);
            for (int y = 0; y < im.h; ++y)
                for (int x = 0; x < im.w; ++x) {
                    float* o = st.img.at(x, y);
                    float r, g, b, a = 1;
                    if (im.c == 1) r = g = b = im.at(x, y, 0);
                    else if (im.c == 2) { r = g = b = im.at(x, y, 0); a = im.at(x, y, 1); }
                    else { r = im.at(x, y, 0); g = im.at(x, y, 1); b = im.at(x, y, 2); if (im.c > 3) a = im.at(x, y, 3); }
                    o[0] = r * a; o[1] = g * a; o[2] = b * a; o[3] = a; // premultiply
                }
            if (path.size() > 4 && (path.compare(path.size() - 4, 4, ".exr") == 0 || path.compare(path.size() - 4, 4, ".EXR") == 0)) {
                int w, h;
                std::vector<std::pair<std::string, std::vector<float>>> ch;
                if (readExrChannels(path, w, h, ch))
                    for (auto& cc : ch)
                        if (cc.first == "Z" && w == im.w && h == im.h) st.img.z = cc.second;
                // EXR colour is already premultiplied by convention
                for (int y = 0; y < im.h && im.c > 3; ++y)
                    for (int x = 0; x < im.w; ++x) { float* o = st.img.at(x, y); o[0] = im.at(x, y, 0); o[1] = im.at(x, y, 1); o[2] = im.at(x, y, 2); }
            }
        }
    }
    if (st.path.empty()) { c.error(st.err); return; }
    Img copy = st.img;
    putImg(c, std::move(copy));
}

void cookConstant(CookContext& c) {
    Img img;
    img.alloc(std::clamp(c.i("width"), 1, 16384), std::clamp(c.i("height"), 1, 16384));
    Vec3 col = c.v("color");
    float a = std::clamp(c.f("alpha"), 0.f, 1.f);
    Vec3 col2 = c.v("color2");
    const int mode = c.i("mode");
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) {
            Vec3 cc = col;
            if (mode == 1) cc = lerp(col, col2, float(y) / float(std::max(1, img.h - 1)));
            else if (mode == 2) cc = lerp(col, col2, float(x) / float(std::max(1, img.w - 1)));
            else if (mode == 3) cc = (((x / std::max(1, c.i("checker"))) + (y / std::max(1, c.i("checker")))) % 2) ? col2 : col;
            float* o = img.at(x, y);
            o[0] = cc.x * a; o[1] = cc.y * a; o[2] = cc.z * a; o[3] = a;
        }
    putImg(c, std::move(img));
}

void cookRender(CookContext& c) {
    const Geometry& geo = c.input(0);
    RenderSettings rs = c.graph().render;
    rs.width = std::clamp(c.i("width"), 16, 4096);
    rs.height = std::clamp(c.i("height"), 16, 4096);
    rs.samples = std::clamp(c.i("samples"), 1, 4096);
    rs.denoise = c.b("denoise");
    rs.transparentBackground = c.b("transparent");
    rs.fps = c.fps();
    // frame the geometry when the scene has no saved camera
    if (!rs.hasCamera) {
        Vec3 mn, mx;
        geo.bounds(mn, mx);
        rs.camTarget = (mn + mx) * 0.5f;
        rs.camDist = std::max(length(mx - mn) * 0.5f, 0.05f) / std::sin(radians(rs.camFov) * 0.5f) * 1.1f;
    }
    const float frame = c.frame();
    for (auto& cam : collectCameras(c.graph(), frame)) {
        if (cam.name != rs.cameraName) continue;
        rs.useEye = true; rs.camEye = cam.eye; rs.camTarget = cam.eye + cam.fwd;
        SceneCamera fitted = cam;
        fitted.resx = rs.width; fitted.resy = rs.height;
        rs.camFov = fitted.fovYDegrees();
        rs.sensorHeight = cam.aperture * float(rs.height) / float(rs.width);
        if (cam.dof) { rs.dof = true; rs.fstop = cam.fstop; rs.focusDist = cam.focus; }
    }
    PathTracer pt;
    pt.start(geo, rs, collectLights(c.graph(), frame));
    pt.wait();
    PathTracer::Buffers b;
    if (!pt.fetchBuffers(b)) { c.error("Render failed."); return; }
    if (!pt.warnings().empty()) c.warning(pt.warnings());
    Img img;
    img.alloc(b.w, b.h);
    const std::vector<float>& col = b.denoised.empty() ? b.beauty : b.denoised;
    for (size_t i = 0; i < size_t(b.w) * size_t(b.h); ++i) {
        img.px[i * 4] = col[i * 3]; img.px[i * 4 + 1] = col[i * 3 + 1]; img.px[i * 4 + 2] = col[i * 3 + 2];
        img.px[i * 4 + 3] = rs.transparentBackground ? std::clamp(b.alpha[i], 0.f, 1.f) : 1.f;
    }
    img.z = b.depth;
    putImg(c, std::move(img));
}

// ------------------------------------------------------------------ filters
void cookColor(CookContext& c) {
    Img img;
    if (!getImg(c, 0, img)) return;
    const float expo = std::pow(2.f, c.f("exposure"));
    const float gamma = std::max(0.01f, c.f("gamma"));
    const float sat = c.f("saturation"), contrast = c.f("contrast"), hue = radians(c.f("hue"));
    const Vec3 lift = c.v("lift"), gain = c.v("gain"), tint = c.v("tint");
    const bool invert = c.b("invert");
    const float mix = std::clamp(c.f("mix"), 0.f, 1.f);
    // hue rotation about the grey axis
    const float ch = std::cos(hue), sh = std::sin(hue), k = (1 - ch) / 3.f, s3 = sh / std::sqrt(3.f);
    parallelFor(img.h, [&](int y0, int y1) {
        for (int y = y0; y < y1; ++y)
            for (int x = 0; x < img.w; ++x) {
                float* p = img.at(x, y);
                float a = p[3];
                if (a <= 0) continue;
                Vec3 v(p[0] / a, p[1] / a, p[2] / a);
                Vec3 o = v * expo;
                if (hue != 0) o = Vec3(o.x * (ch + k) + o.y * (k - s3) + o.z * (k + s3), o.x * (k + s3) + o.y * (ch + k) + o.z * (k - s3),
                                       o.x * (k - s3) + o.y * (k + s3) + o.z * (ch + k));
                float l = lumv(o.x, o.y, o.z);
                o = Vec3(l) + (o - Vec3(l)) * sat;
                o = Vec3(0.18f) + (o - Vec3(0.18f)) * contrast;
                o = o * gain * tint + lift * (Vec3(1) - o);
                for (int cc = 0; cc < 3; ++cc) o[cc] = o[cc] > 0 ? std::pow(o[cc], 1.f / gamma) : o[cc];
                if (invert) o = Vec3(1) - o;
                o = lerp(v, o, mix);
                p[0] = o.x * a; p[1] = o.y * a; p[2] = o.z * a;
            }
    });
    putImg(c, std::move(img));
}

void cookBlur(CookContext& c) {
    Img img;
    if (!getImg(c, 0, img)) return;
    gaussian(img, std::max(0.f, c.f("size")) * 0.5f);
    putImg(c, std::move(img));
}

void cookGlow(CookContext& c) {
    Img img;
    if (!getImg(c, 0, img)) return;
    Img bright = img;
    const float thr = c.f("threshold");
    for (size_t i = 0; i < bright.px.size(); i += 4) {
        float l = lumv(bright.px[i], bright.px[i + 1], bright.px[i + 2]);
        float f = l > thr && l > 0 ? (l - thr) / l : 0.f;
        for (int k = 0; k < 4; ++k) bright.px[i + size_t(k)] *= f;
    }
    // three octaves of blur give a natural, long-tailed falloff
    const float size = std::max(0.5f, c.f("size")), amount = c.f("intensity");
    std::vector<float> acc(img.px.size(), 0.f);
    for (int o = 0; o < 3; ++o) {
        Img b = bright;
        gaussian(b, size * 0.5f * float(1 << o) / 2.f);
        for (size_t i = 0; i < acc.size(); ++i) acc[i] += b.px[i] / 3.f;
    }
    for (size_t i = 0; i < img.px.size(); i += 4) {
        for (int k = 0; k < 3; ++k) img.px[i + size_t(k)] += acc[i + size_t(k)] * amount;
        img.px[i + 3] = std::min(1.f, img.px[i + 3] + acc[i + 3] * amount);
    }
    putImg(c, std::move(img));
}

void cookMerge(CookContext& c) {
    Img A, B;
    if (!getImg(c, 0, A, "image (A, foreground)") || !getImg(c, 1, B, "image (B, background)")) return;
    const int op = c.i("operation");
    const float mix = std::clamp(c.f("mix"), 0.f, 1.f);
    // A is placed centred on B's canvas when the sizes differ
    const int ox = (B.w - A.w) / 2, oy = (B.h - A.h) / 2;
    Img out = B;
    parallelFor(B.h, [&](int y0, int y1) {
        for (int y = y0; y < y1; ++y)
            for (int x = 0; x < B.w; ++x) {
                int ax = x - ox, ay = y - oy;
                float a[4] = {0, 0, 0, 0};
                if (ax >= 0 && ay >= 0 && ax < A.w && ay < A.h) { const float* p = A.at(ax, ay); for (int k = 0; k < 4; ++k) a[k] = p[k]; }
                const float* b = B.at(x, y);
                float r[4];
                for (int k = 0; k < 4; ++k) {
                    switch (op) {
                    case 0: r[k] = a[k] + b[k] * (1 - a[3]); break;               // A over B
                    case 1: r[k] = b[k] + a[k] * (1 - b[3]); break;               // A under B
                    case 2: r[k] = a[k] + b[k]; break;                            // add
                    case 3: r[k] = b[k] - a[k]; break;                            // subtract
                    case 4: r[k] = a[k] * b[k]; break;                            // multiply
                    case 5: r[k] = a[k] + b[k] - a[k] * b[k]; break;              // screen
                    case 6: r[k] = std::max(a[k], b[k]); break;                   // maximum
                    case 7: r[k] = std::min(a[k], b[k]); break;                   // minimum
                    case 8: r[k] = a[k] * b[3]; break;                            // A inside B
                    default: r[k] = a[k] * (1 - b[3]); break;                     // A outside B
                    }
                }
                if (op == 3 || op == 2) r[3] = std::clamp(r[3], 0.f, 1.f);
                float* o = out.at(x, y);
                for (int k = 0; k < 4; ++k) o[k] = b[k] + (r[k] - b[k]) * mix;
            }
    });
    if (out.z.empty() && !A.z.empty() && A.w == B.w && A.h == B.h) out.z = A.z;
    putImg(c, std::move(out));
}

void cookZComp(CookContext& c) {
    Img A, B;
    if (!getImg(c, 0, A) || !getImg(c, 1, B)) return;
    if (A.w != B.w || A.h != B.h) { c.error("Depth compositing needs images of the same size."); return; }
    if (A.z.empty() || B.z.empty()) { c.error("Both images need depth (Z), e.g. from Render nodes or EXR files."); return; }
    Img out = B;
    for (size_t i = 0; i < size_t(A.w) * size_t(A.h); ++i) {
        const float* a = &A.px[i * 4];
        const float* b = &B.px[i * 4];
        bool aFront = A.z[i] <= B.z[i];
        const float* f = aFront ? a : b;
        const float* k = aFront ? b : a;
        for (int ch = 0; ch < 4; ++ch) out.px[i * 4 + size_t(ch)] = f[ch] + k[ch] * (1 - f[3]);
        out.z[i] = std::min(A.z[i], B.z[i]);
    }
    putImg(c, std::move(out));
}

void cookKey(CookContext& c) {
    Img img;
    if (!getImg(c, 0, img)) return;
    const Vec3 key = c.v("keycolor");
    const float tol = std::max(0.f, c.f("tolerance")), soft = std::max(1e-4f, c.f("softness"));
    const bool despill = c.b("despill");
    // compare hue/saturation: colours normalised by their sum (brightness-independent)
    auto chroma = [](Vec3 v) { float s = v.x + v.y + v.z; return s > 1e-6f ? v / s : Vec3(1.f / 3); };
    const Vec3 kc = chroma(key);
    const int dom = key.y >= key.x && key.y >= key.z ? 1 : (key.z >= key.x ? 2 : 0);
    for (size_t i = 0; i < img.px.size(); i += 4) {
        float a0 = img.px[i + 3];
        if (a0 <= 0) continue;
        Vec3 v(img.px[i] / a0, img.px[i + 1] / a0, img.px[i + 2] / a0);
        float d = length(chroma(v) - kc);
        float t = std::clamp((d - tol) / soft, 0.f, 1.f);
        float alpha = t * t * (3 - 2 * t);
        if (despill) {
            int o1 = (dom + 1) % 3, o2 = (dom + 2) % 3;
            float lim = std::max(v[o1], v[o2]);
            if (v[dom] > lim) v[dom] = lim;
        }
        float a = a0 * alpha;
        img.px[i] = v.x * a; img.px[i + 1] = v.y * a; img.px[i + 2] = v.z * a; img.px[i + 3] = a;
    }
    putImg(c, std::move(img));
}

void cookShape(CookContext& c) {
    // size from input 1 (an image) or the parameters; shapes from input 2 (closed curves, x/y in 0..1, y up)
    Img base;
    bool hasBase = c.hasInput(0) && !c.input(0).images.empty() && c.input(0).images[0].valid();
    if (hasBase) getImg(c, 0, base);
    int W = hasBase ? base.w : std::clamp(c.i("width"), 1, 16384), H = hasBase ? base.h : std::clamp(c.i("height"), 1, 16384);
    if (!c.hasInput(1)) { c.error("Connect the shape curves (closed, in 0-1 image space) to input 2."); return; }
    const Geometry& sh = c.input(1);
    struct Edge { float x0, y0, x1, y1; };
    std::vector<Edge> edges;
    for (auto& pr : sh.prims) {
        if (pr.count < 3) continue;
        for (int k = 0; k < pr.count; ++k) {
            Vec3 a = sh.P[size_t(sh.vertices[size_t(pr.start + k)])], b = sh.P[size_t(sh.vertices[size_t(pr.start + (k + 1) % pr.count)])];
            edges.push_back({a.x * float(W), (1 - a.y) * float(H), b.x * float(W), (1 - b.y) * float(H)});
        }
    }
    if (edges.empty()) { c.error("The shape input has no closed curves with 3 or more points."); return; }
    std::vector<float> mask(size_t(W) * size_t(H), 0.f);
    const int ss = 4; // 4x4 supersampling
    parallelFor(H, [&](int y0, int y1) {
        std::vector<float> xs;
        for (int y = y0; y < y1; ++y)
            for (int sy = 0; sy < ss; ++sy) {
                float py = float(y) + (float(sy) + 0.5f) / ss;
                xs.clear();
                for (auto& e : edges)
                    if ((e.y0 > py) != (e.y1 > py)) xs.push_back(e.x0 + (py - e.y0) / (e.y1 - e.y0) * (e.x1 - e.x0));
                std::sort(xs.begin(), xs.end());
                for (size_t k = 0; k + 1 < xs.size(); k += 2)
                    for (int x = std::max(0, int(std::floor(xs[k]))); x < std::min(W, int(std::ceil(xs[k + 1]))); ++x)
                        for (int sx = 0; sx < ss; ++sx) {
                            float px = float(x) + (float(sx) + 0.5f) / ss;
                            if (px >= xs[k] && px < xs[k + 1]) mask[size_t(y) * size_t(W) + size_t(x)] += 1.f / (ss * ss);
                        }
            }
    });
    // feather
    const float feather = std::max(0.f, c.f("feather"));
    if (feather > 0) {
        Img m;
        m.alloc(W, H);
        for (size_t i = 0; i < mask.size(); ++i) for (int k = 0; k < 4; ++k) m.px[i * 4 + size_t(k)] = mask[i];
        gaussian(m, feather * 0.5f);
        for (size_t i = 0; i < mask.size(); ++i) mask[i] = m.px[i * 4];
    }
    if (c.b("invert")) for (auto& v : mask) v = 1 - v;
    Img out;
    if (hasBase && c.i("mode") == 1) {
        out = base;
        for (size_t i = 0; i < mask.size(); ++i) for (int k = 0; k < 4; ++k) out.px[i * 4 + size_t(k)] *= mask[i];
    } else {
        out.alloc(W, H);
        for (size_t i = 0; i < mask.size(); ++i) for (int k = 0; k < 4; ++k) out.px[i * 4 + size_t(k)] = mask[i];
    }
    putImg(c, std::move(out));
}

void cookTransform(CookContext& c) {
    Img img;
    if (!getImg(c, 0, img)) return;
    const Vec3 t = c.v("t"), sc = c.v("s");
    const float rot = radians(c.f("r")), us = c.f("scale");
    const float sx = sc.x * us, sy = sc.y * us;
    if (std::fabs(sx) < 1e-6f || std::fabs(sy) < 1e-6f) { c.error("Scale is zero."); return; }
    const float cx = img.w * 0.5f, cy = img.h * 0.5f, cr = std::cos(rot), sr = std::sin(rot);
    Img out;
    out.alloc(img.w, img.h);
    if (!img.z.empty()) out.z.assign(img.z.size(), 1e10f);
    parallelFor(img.h, [&](int y0, int y1) {
        for (int y = y0; y < y1; ++y)
            for (int x = 0; x < img.w; ++x) {
                // inverse map: output pixel -> source pixel (y down in pixels, so rotation is counter-clockwise on screen)
                float px = float(x) + 0.5f - cx - t.x, py = float(y) + 0.5f - cy + t.y;
                float rx = (px * cr - py * sr) / sx, ry = (px * sr + py * cr) / sy;
                float s[4];
                img.sample(rx + cx, ry + cy, s);
                float* o = out.at(x, y);
                for (int k = 0; k < 4; ++k) o[k] = s[k];
                if (!img.z.empty()) {
                    int xi = int(rx + cx), yi = int(ry + cy);
                    if (xi >= 0 && yi >= 0 && xi < img.w && yi < img.h) out.z[size_t(y) * size_t(img.w) + size_t(x)] = img.z[size_t(yi) * size_t(img.w) + size_t(xi)];
                }
            }
    });
    putImg(c, std::move(out));
}

void cookLens(CookContext& c) {
    Img img;
    if (!getImg(c, 0, img)) return;
    const float k1 = c.f("distortion"), ca = c.f("aberration"), vig = std::clamp(c.f("vignette"), 0.f, 1.f), vsoft = std::max(0.05f, c.f("vignettesoft"));
    const float grain = std::max(0.f, c.f("grain"));
    const uint32_t seed = uint32_t(std::lround(c.frame())) * 7919u;
    Img out;
    out.alloc(img.w, img.h);
    const float cx = img.w * 0.5f, cy = img.h * 0.5f, norm = std::sqrt(cx * cx + cy * cy);
    parallelFor(img.h, [&](int y0, int y1) {
        for (int y = y0; y < y1; ++y)
            for (int x = 0; x < img.w; ++x) {
                float dx = (float(x) + 0.5f - cx) / norm, dy = (float(y) + 0.5f - cy) / norm;
                float r2 = dx * dx + dy * dy;
                float* o = out.at(x, y);
                // radial distortion (k1 > 0 barrel, < 0 pincushion) with a per-channel scale for chromatic aberration
                for (int ch = 0; ch < 4; ++ch) {
                    int band = ch == 3 ? 1 : ch;
                    float f = 1 + k1 * r2 + (float(band) - 1) * ca * 0.01f * r2;
                    float s[4];
                    img.sample(cx + dx * f * norm, cy + dy * f * norm, s);
                    o[ch] = s[ch];
                }
                float v = 1 - vig * std::clamp((std::sqrt(r2) - (1 - vsoft)) / vsoft, 0.f, 1.f);
                for (int ch = 0; ch < 3; ++ch) o[ch] *= v;
                if (grain > 0) {
                    float n = (hashRandom(seed + uint32_t(x + y * img.w) * 3u) - 0.5f) * grain;
                    for (int ch = 0; ch < 3; ++ch) o[ch] = std::max(0.f, o[ch] + n * o[3]);
                }
            }
    });
    putImg(c, std::move(out));
}

void cookDefocus(CookContext& c) {
    Img img;
    if (!getImg(c, 0, img)) return;
    if (img.z.empty()) { c.error("Defocus needs depth (Z): render the image with a Render node or read an EXR with Z."); return; }
    const float focus = c.f("focus"), range = std::max(1e-4f, c.f("range")), maxR = std::clamp(c.f("maxblur"), 0.f, 64.f);
    auto radius = [&](float z) { return std::clamp(std::fabs(z - focus) / range, 0.f, 1.f) * maxR; };
    const int N = 48; // Vogel disk samples
    Img out = img;
    parallelFor(img.h, [&](int y0, int y1) {
        for (int y = y0; y < y1; ++y)
            for (int x = 0; x < img.w; ++x) {
                const size_t i = size_t(y) * size_t(img.w) + size_t(x);
                float r = radius(img.z[i]);
                if (r < 0.5f) continue;
                float acc[4] = {0, 0, 0, 0}, ws = 0;
                for (int s = 0; s < N; ++s) {
                    float rr = r * std::sqrt((float(s) + 0.5f) / float(N)), a = float(s) * 2.39996323f;
                    int sx = int(std::lround(float(x) + rr * std::cos(a))), sy = int(std::lround(float(y) + rr * std::sin(a)));
                    if (sx < 0 || sy < 0 || sx >= img.w || sy >= img.h) continue;
                    size_t j = size_t(sy) * size_t(img.w) + size_t(sx);
                    // sharper things behind us don't spread over us: only take samples blurred enough to reach here
                    float rj = radius(img.z[j]);
                    if (img.z[j] > img.z[i] && rj < rr) continue;
                    for (int k = 0; k < 4; ++k) acc[k] += img.px[j * 4 + size_t(k)];
                    ws += 1;
                }
                if (ws > 0) for (int k = 0; k < 4; ++k) out.px[i * 4 + size_t(k)] = acc[k] / ws;
            }
    });
    putImg(c, std::move(out));
}

void cookResize(CookContext& c) {
    Img img;
    if (!getImg(c, 0, img)) return;
    int W = c.i("width"), H = c.i("height");
    if (W <= 0 && H <= 0) { putImg(c, std::move(img)); return; }
    if (W <= 0) W = std::max(1, int(std::lround(float(img.w) * float(H) / float(img.h))));
    if (H <= 0) H = std::max(1, int(std::lround(float(img.h) * float(W) / float(img.w))));
    W = std::min(W, 16384); H = std::min(H, 16384);
    // pre-filter when shrinking so detail averages instead of aliasing
    float shrink = std::max(float(img.w) / float(W), float(img.h) / float(H));
    if (shrink > 1.f) gaussian(img, (shrink - 1.f) * 0.5f);
    Img out;
    out.alloc(W, H);
    if (!img.z.empty()) out.z.resize(size_t(W) * size_t(H));
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            float sx = (float(x) + 0.5f) * float(img.w) / float(W), sy = (float(y) + 0.5f) * float(img.h) / float(H);
            float s[4];
            // clamp so the border does not fade to transparent
            img.sample(std::clamp(sx, 0.5f, float(img.w) - 0.5f), std::clamp(sy, 0.5f, float(img.h) - 0.5f), s);
            float* o = out.at(x, y);
            for (int k = 0; k < 4; ++k) o[k] = s[k];
            if (!img.z.empty()) out.z[size_t(y) * size_t(W) + size_t(x)] = img.z[size_t(std::min(img.h - 1, int(sy))) * size_t(img.w) + size_t(std::min(img.w - 1, int(sx)))];
        }
    putImg(c, std::move(out));
}

void cookWrite(CookContext& c) {
    c.out = c.input(0);
    if (c.out.images.empty() || !c.out.images[0].valid()) { c.error("Input has no image."); return; }
    if (!c.b("enable")) return;
    std::string err;
    std::string path = resolvePath(c.s("file"));
    if (path.empty()) { c.error("Choose an output file."); return; }
    if (!writeImageLayer(path, c.out.images[0], c.frame(), &err)) c.error(err);
}

} // namespace

bool writeImageLayer(const std::string& pathIn, const ImageLayer& L, float frame, std::string* err) {
    if (!L.valid()) { if (err) *err = "No image"; return false; }
    std::string path = expandFrame(pathIn, frame);
    const size_t n = L.pixels();
    const std::vector<float>& px = *L.rgba;
    bool exr = path.size() > 4 && (path.compare(path.size() - 4, 4, ".exr") == 0 || path.compare(path.size() - 4, 4, ".EXR") == 0);
    if (exr) {
        std::vector<std::vector<float>> planes(4, std::vector<float>(n));
        for (size_t i = 0; i < n; ++i) for (int k = 0; k < 4; ++k) planes[size_t(k)][i] = px[i * 4 + size_t(k)];
        std::vector<ExrChannel> ch = {{"R", planes[0].data()}, {"G", planes[1].data()}, {"B", planes[2].data()}, {"A", planes[3].data()}};
        if (L.depth && L.depth->size() == n) ch.push_back({"Z", L.depth->data()});
        return writeExr(path, L.w, L.h, ch, true, err);
    }
    std::vector<uint8_t> out(n * 4);
    auto enc = [](float v) {
        v = std::clamp(v, 0.f, 1.f);
        v = v <= 0.0031308f ? v * 12.92f : 1.055f * std::pow(v, 1.f / 2.4f) - 0.055f;
        return uint8_t(std::lround(v * 255.f));
    };
    bool opaque = true;
    for (size_t i = 0; i < n; ++i) {
        float a = px[i * 4 + 3];
        opaque = opaque && a >= 0.999f;
        for (int k = 0; k < 3; ++k) out[i * 4 + size_t(k)] = enc(a > 1e-6f ? px[i * 4 + size_t(k)] / a : 0.f);
        out[i * 4 + 3] = uint8_t(std::lround(std::clamp(a, 0.f, 1.f) * 255.f));
    }
    if (opaque) {
        std::vector<uint8_t> rgb(n * 3);
        for (size_t i = 0; i < n; ++i) for (int k = 0; k < 3; ++k) rgb[i * 3 + size_t(k)] = out[i * 4 + size_t(k)];
        return writePng8(path, rgb, L.w, L.h, 3, err);
    }
    return writePng8(path, out, L.w, L.h, 4, err);
}

void registerCompositingNodes(std::vector<NodeType>& types) {
    auto add = [&](NodeType t) { t.category = "Compositing"; types.push_back(std::move(t)); };
    {
        NodeType t; t.name = "copfile"; t.label = "Image File";
        t.description = "Reads a PNG, EXR or HDR image (EXR Z channels become depth). $F in the name reads a sequence.";
        t.timeDependent = true;
        t.parms = {FileP("file", "File", "e.g. $HIP/plates/shot.$F4.exr")};
        t.cook = cookFile;
        add(t);
    }
    {
        NodeType t; t.name = "copconstant"; t.label = "Constant / Ramp";
        t.description = "A solid color, a vertical or horizontal ramp, or a checkerboard.";
        t.parms = {I("width", "Width", 1280, 1, 8192), I("height", "Height", 720, 1, 8192), M("mode", "Pattern", {"Constant", "Vertical Ramp", "Horizontal Ramp", "Checker"}),
                   ColorP("color", "Color", Vec3(0.18f)), ColorP("color2", "Second Color", Vec3(0.8f)), F("alpha", "Alpha", 1, 0, 1), I("checker", "Checker Size", 64, 1, 512)};
        t.cook = cookConstant;
        add(t);
    }
    {
        NodeType t; t.name = "coprender"; t.label = "Render";
        t.description = "Path-traces the input geometry with the scene's camera and lights into an image with depth.";
        t.minInputs = 1; t.maxInputs = 1; t.inputLabels = {"Geometry to render"};
        t.parms = {I("width", "Width", 640, 16, 4096), I("height", "Height", 360, 16, 4096), I("samples", "Samples", 32, 1, 4096),
                   T("transparent", "Transparent Background", true, "Alpha from coverage, for merging over plates"), T("denoise", "Denoise", true)};
        t.cook = cookRender;
        add(t);
    }
    {
        NodeType t; t.name = "copcolor"; t.label = "Color Correct";
        t.description = "Exposure, hue, saturation, contrast, lift/gain, tint and gamma (on unpremultiplied colour).";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {F("exposure", "Exposure (stops)", 0, -6, 6), F("hue", "Hue Shift", 0, -180, 180), F("saturation", "Saturation", 1, 0, 3), F("contrast", "Contrast", 1, 0, 3),
                   V("lift", "Lift", Vec3(0), -0.5f, 0.5f), V("gain", "Gain", Vec3(1), 0, 3), ColorP("tint", "Tint", Vec3(1)), F("gamma", "Gamma", 1, 0.2f, 4),
                   T("invert", "Invert", false), F("mix", "Mix", 1, 0, 1)};
        t.cook = cookColor;
        add(t);
    }
    {
        NodeType t; t.name = "copblur"; t.label = "Blur";
        t.description = "Gaussian blur.";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {F("size", "Size (pixels)", 8, 0, 200)};
        t.cook = cookBlur;
        add(t);
    }
    {
        NodeType t; t.name = "copglow"; t.label = "Glow";
        t.description = "Blooms the bright parts of the image.";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {F("threshold", "Threshold", 1, 0, 10), F("size", "Size (pixels)", 30, 1, 300), F("intensity", "Intensity", 1, 0, 5)};
        t.cook = cookGlow;
        add(t);
    }
    {
        NodeType t; t.name = "copmerge"; t.label = "Merge Images";
        t.description = "Combines A (input 1) with B (input 2): over, under, add, multiply, screen, inside/outside masks...";
        t.minInputs = 2; t.maxInputs = 2; t.inputLabels = {"A (foreground)", "B (background)"};
        t.parms = {M("operation", "Operation", {"Over", "Under", "Add", "Subtract", "Multiply", "Screen", "Maximum", "Minimum", "Inside", "Outside"}), F("mix", "Mix", 1, 0, 1)};
        t.cook = cookMerge;
        add(t);
    }
    {
        NodeType t; t.name = "copzcomp"; t.label = "Z Composite";
        t.description = "Depth compositing: each pixel shows whichever image is nearer to the camera.";
        t.minInputs = 2; t.maxInputs = 2; t.inputLabels = {"Image with depth", "Image with depth"};
        t.cook = cookZComp;
        add(t);
    }
    {
        NodeType t; t.name = "copkey"; t.label = "Chroma Key";
        t.description = "Pulls a key from a coloured screen into alpha, with spill suppression.";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {ColorP("keycolor", "Key Color", Vec3(0.1f, 0.8f, 0.2f)), F("tolerance", "Tolerance", 0.08f, 0, 0.5f), F("softness", "Softness", 0.1f, 0.001f, 0.5f), T("despill", "Despill", true)};
        t.cook = cookKey;
        add(t);
    }
    {
        NodeType t; t.name = "copshape"; t.label = "Shape Mask (Roto)";
        t.description = "Rasterizes closed curves (x and y from 0 to 1 across the image, y up) into an anti-aliased, feathered mask. Animate the curves to rotoscope.";
        t.minInputs = 0; t.maxInputs = 2; t.inputLabels = {"Image (optional: size, or cut out)", "Shape curves"};
        t.parms = {I("width", "Width", 1280, 1, 8192), I("height", "Height", 720, 1, 8192), F("feather", "Feather (pixels)", 2, 0, 100), T("invert", "Invert", false),
                   M("mode", "Output", {"Mask", "Cut Out Input Image"})};
        t.cook = cookShape;
        add(t);
    }
    {
        NodeType t; t.name = "coptransform"; t.label = "Transform Image";
        t.description = "Moves (pixels), rotates and scales the image about its centre.";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {V("t", "Translate (pixels)", Vec3(0), -500, 500), F("r", "Rotate", 0, -180, 180), V("s", "Scale", Vec3(1, 1, 1), 0, 4), F("scale", "Uniform Scale", 1, 0, 4)};
        t.cook = cookTransform;
        add(t);
    }
    {
        NodeType t; t.name = "coplens"; t.label = "Lens Effects";
        t.description = "Barrel/pincushion distortion, chromatic aberration, vignette and film grain.";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {F("distortion", "Distortion", 0, -0.5f, 0.5f, "Positive = barrel, negative = pincushion"), F("aberration", "Chromatic Aberration", 0, 0, 5),
                   F("vignette", "Vignette", 0.3f, 0, 1), F("vignettesoft", "Vignette Softness", 0.6f, 0.05f, 1), F("grain", "Grain", 0, 0, 0.2f)};
        t.cook = cookLens;
        add(t);
    }
    {
        NodeType t; t.name = "copdefocus"; t.label = "Defocus (Z)";
        t.description = "Depth-of-field blur from the depth channel: sharp at the focus distance, blurrier away from it.";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {F("focus", "Focus Distance", 5, 0, 100), F("range", "Blur Range", 3, 0.01f, 100, "Distance from focus at which the blur is largest"), F("maxblur", "Max Blur (pixels)", 8, 0, 64)};
        t.cook = cookDefocus;
        add(t);
    }
    {
        NodeType t; t.name = "copresize"; t.label = "Resize";
        t.description = "Scales the image to a new size (0 keeps the aspect ratio).";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {I("width", "Width", 640, 0, 8192), I("height", "Height", 0, 0, 8192)};
        t.cook = cookResize;
        add(t);
    }
    {
        NodeType t; t.name = "copwrite"; t.label = "Write Image";
        t.description = "Writes the image to disk every time it cooks while enabled (play the timeline to write a sequence). PNG or EXR.";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {FileP("file", "Output File", "e.g. $HIP/comp/out.$F4.png"), T("enable", "Write Enabled", false)};
        t.cook = cookWrite;
        add(t);
    }
}

} // namespace hd

// Shader networks: Shading node types, the network compiler, the program evaluator, the shader
// registry and the Material Builder node.
//
// Compilation walks the network upstream from the Principled Shader Output node. Every node becomes
// one instruction writing one register; an unconnected input becomes a constant register holding
// the parameter of the same name (so a node with nothing wired in still does something useful, as
// in Houdini's VOPs). Position inputs left unconnected read the global P (or uv, by the node's
// Domain menu). Constants live in the program's initial register image, so evaluation is a copy
// plus one switch per instruction.
#include "Shader.h"
#include "Image.h"
#include "Node.h"
#include "Noise.h"
#include "Ops.h"
#include "Volume.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <mutex>
#include <set>

namespace hd {
namespace {

ParmDef F(const char* n, const char* l, float d, float mn = 0, float mx = 1, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Float; p.def[0] = d; p.min = mn; p.max = mx; p.help = help; return p;
}
ParmDef I(const char* n, const char* l, int d, int mn = 0, int mx = 10, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Int; p.def[0] = float(d); p.min = float(mn); p.max = float(mx); p.help = help; return p;
}
ParmDef S(const char* n, const char* l, const char* d = "", const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::String; p.defStr = d; p.help = help; return p;
}
ParmDef M(const char* n, const char* l, std::vector<std::string> items, int d = 0) {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Menu; p.menu = std::move(items); p.def[0] = float(d); return p;
}
ParmDef V(const char* n, const char* l, Vec3 d, float mn = -1, float mx = 1, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Vec3; p.def[0] = d.x; p.def[1] = d.y; p.def[2] = d.z; p.min = mn; p.max = mx; p.help = help; return p;
}
ParmDef C(const char* n, const char* l, Vec3 d) { ParmDef p = V(n, l, d, 0, 1); p.help = "color"; return p; }
ParmDef T(const char* n, const char* l, bool d, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Toggle; p.def[0] = d ? 1.f : 0.f; p.help = help; return p;
}
ParmDef FileP(const char* n, const char* l, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::File; p.help = help; return p;
}

enum Op {
    OpGlobal, OpAttrib, OpBin, OpUn, OpMix, OpFit, OpClamp, OpSmooth, OpNoise, OpVoronoi, OpChecker, OpRamp,
    OpTex, OpCompose, OpDecompose, OpRandom, OpWave
};
enum Global { GP, GN, GUv, GCd, GTime, GFrame };

inline float fract(float x) { return x - std::floor(x); }
inline Vec3 vabs(const Vec3& v) { return {std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)}; }
inline Vec3 vfloor(const Vec3& v) { return {std::floor(v.x), std::floor(v.y), std::floor(v.z)}; }
inline Vec3 vclamp(const Vec3& v, const Vec3& a, const Vec3& b) { return vmin(vmax(v, a), b); }
inline float lumf(const Vec3& c) { return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z; }

inline uint32_t mix32(uint32_t h) {
    h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15; h *= 0x846ca68bu; h ^= h >> 16;
    return h;
}
inline uint32_t hash3i(int x, int y, int z, uint32_t seed = 0) {
    return mix32(uint32_t(x) * 73856093u ^ mix32(uint32_t(y) * 19349663u ^ mix32(uint32_t(z) * 83492791u ^ seed)));
}
// floor to int, safe for huge and non-finite values (cells far away simply repeat)
inline int fl(float x) { return x >= 1e9f ? 1000000000 : x <= -1e9f ? -1000000000 : x == x ? int(std::floor(x)) : 0; }
inline float unit(uint32_t h) { return float(h >> 8) * (1.f / 16777216.f); }

// Fractal noise in a few flavours; every one returns about [0, 1] (Vector: [-1, 1] per component).
Vec3 fractal(const Vec3& q, int type, int octaves, float rough, float lac) {
    octaves = std::clamp(octaves, 1, 12);
    if (type == 0) return Vec3(std::clamp(perlin3(q.x, q.y, q.z) * 0.5f + 0.5f, 0.f, 1.f));
    if (type == 4) {
        Vec3 r;
        static const Vec3 off[3] = {Vec3(0), Vec3(31.416f, 47.853f, 12.679f), Vec3(-71.13f, 5.27f, 93.71f)};
        for (int c = 0; c < 3; ++c) {
            float a = 1, sum = 0, norm = 0;
            Vec3 p = q + off[c];
            for (int o = 0; o < octaves; ++o) { sum += perlin3(p.x, p.y, p.z) * a; norm += a; a *= rough; p = p * lac; }
            r[c] = std::clamp(sum / norm * 1.4f, -1.f, 1.f);
        }
        return r;
    }
    float a = 1, sum = 0, norm = 0;
    Vec3 p = q;
    for (int o = 0; o < octaves; ++o) {
        float n = perlin3(p.x, p.y, p.z);
        if (type == 1) sum += n * a;
        else if (type == 2) sum += std::fabs(n) * a;
        else { float r = 1 - std::fabs(n); sum += r * r * a; }
        norm += a;
        a *= rough;
        p = p * lac;
    }
    float v = sum / norm;
    if (type == 1) v = v * 0.7f + 0.5f;
    else if (type == 2) v = v * 1.6f;
    return Vec3(std::clamp(v, 0.f, 1.f));
}

// ------------------------------------------------------------------ registry

std::mutex gRegMutex;
std::vector<std::shared_ptr<const ShaderProgram>> gRegistry;

// Textures shared between successive compiles of the same file.
std::mutex gTexMutex;
std::map<std::string, std::weak_ptr<Texture>> gTexCache;

std::shared_ptr<Texture> loadShaderTexture(const std::string& path, std::string& err) {
    std::string key = resolvePath(path);
    std::lock_guard<std::mutex> lock(gTexMutex);
    if (auto t = gTexCache[key].lock()) return t;
    auto t = std::make_shared<Texture>();
    if (!t->load(path, false, &err)) return nullptr;
    gTexCache[key] = t;
    return t;
}

bool isLinearFile(const std::string& path) {
    std::string e = path.size() > 4 ? path.substr(path.size() - 4) : path;
    for (auto& ch : e) ch = char(std::tolower((unsigned char)ch));
    return e == ".exr" || e == ".hdr";
}

// ------------------------------------------------------------------ compiler

struct Compiler {
    const Graph& net;
    float frame, fps;
    ShaderProgram& P;
    std::string err;
    std::map<const Node*, int> memo;
    std::set<const Node*> active;
    int globals[6] = {-1, -1, -1, -1, -1, -1};
    std::map<std::string, int> attribRegs;

    int reg(Vec3 init = Vec3(0)) { P.init.push_back(init); return P.nregs++; }
    Vec3 parmV(const Node* n, const char* name) const {
        const Parm* p = n->parm(name);
        if (!p) return Vec3(0);
        if (p->def->type == ParmType::Vec3)
            return {n->evalFloat(*p, 0, frame, fps), n->evalFloat(*p, 1, frame, fps), n->evalFloat(*p, 2, frame, fps)};
        return Vec3(n->evalFloat(*p, 0, frame, fps));
    }
    float parmF(const Node* n, const char* name) const { return parmV(n, name).x; }
    int parmI(const Node* n, const char* name) const { return int(std::lround(parmF(n, name))); }
    const std::string& parmS(const Node* n, const char* name) const {
        static const std::string empty;
        const Parm* p = n->parm(name);
        return p ? p->str : empty;
    }
    int global(int which) {
        if (globals[which] >= 0) return globals[which];
        ShaderProgram::Ins in;
        in.op = OpGlobal; in.i0 = which; in.dst = reg();
        P.code.push_back(in);
        if (which == GCd) P.usesCd = true;
        return globals[which] = in.dst;
    }
    // Input idx of n: the upstream node's register, or a constant from the parameter of that name.
    int input(const Node* n, int idx, const char* parm) {
        const Node* src = idx < int(n->inputs.size()) ? n->inputs[size_t(idx)] : nullptr;
        if (src) return compile(src);
        return reg(parmV(n, parm));
    }
    int posInput(const Node* n, int idx) {
        const Node* src = idx < int(n->inputs.size()) ? n->inputs[size_t(idx)] : nullptr;
        if (src) return compile(src);
        return global(parmI(n, "domain") == 1 ? GUv : GP);
    }
    int emit(ShaderProgram::Ins in) {
        in.dst = reg();
        P.code.push_back(std::move(in));
        return P.code.back().dst;
    }

    int compile(const Node* n) {
        auto it = memo.find(n);
        if (it != memo.end()) return it->second;
        if (!err.empty()) return reg();
        if (active.count(n)) { err = "The shader network has a loop at " + n->name() + "."; return reg(); }
        active.insert(n);
        int r = compileNode(n);
        active.erase(n);
        memo[n] = r;
        return r;
    }

    int compileNode(const Node* n) {
        const std::string& t = n->type().name;
        if (n->bypass) return n->inputs.empty() || !n->inputs[0] ? reg() : compile(n->inputs[0]);
        ShaderProgram::Ins in;
        if (t == "shconstant" || t == "shfloat") return reg(parmV(n, "value"));
        if (t == "shglobals") return global(std::clamp(parmI(n, "global"), 0, 3));
        if (t == "shtime") return global(parmI(n, "which") == 1 ? GFrame : GTime);
        if (t == "shattrib") {
            std::string name = parmS(n, "name");
            if (name.empty()) { err = n->name() + ": no attribute name."; return reg(); }
            if (name == "P") return global(GP);
            if (name == "N") return global(GN);
            if (name == "uv") return global(GUv);
            if (name == "Cd") return global(GCd);
            auto f = std::find(P.attribs.begin(), P.attribs.end(), name);
            int idx = int(f - P.attribs.begin());
            if (f == P.attribs.end()) {
                if (P.attribs.size() >= 16) { err = n->name() + ": a shader can read at most 16 attributes."; return reg(); }
                P.attribs.push_back(name);
                P.attribDefaults.push_back(parmV(n, "default"));
            }
            in.op = OpAttrib; in.i0 = idx; in.k = parmV(n, "default");
            return emit(in);
        }
        if (t == "shmath") {
            in.op = OpBin; in.i0 = parmI(n, "op");
            in.a = input(n, 0, "a"); in.b = input(n, 1, "b");
            return emit(in);
        }
        if (t == "shfunc") {
            in.op = OpUn; in.i0 = parmI(n, "op");
            in.a = input(n, 0, "value");
            return emit(in);
        }
        if (t == "shmix") {
            in.op = OpMix;
            in.a = input(n, 0, "a"); in.b = input(n, 1, "b"); in.c = input(n, 2, "bias");
            return emit(in);
        }
        if (t == "shfit") {
            in.op = OpFit; in.i0 = parmI(n, "clamp");
            in.a = input(n, 0, "value"); in.b = input(n, 1, "srcmin"); in.c = input(n, 2, "srcmax");
            in.d = input(n, 3, "destmin"); in.e = input(n, 4, "destmax");
            return emit(in);
        }
        if (t == "shclamp") {
            in.op = OpClamp;
            in.a = input(n, 0, "value"); in.b = input(n, 1, "min"); in.c = input(n, 2, "max");
            return emit(in);
        }
        if (t == "shsmooth") {
            in.op = OpSmooth;
            in.a = input(n, 0, "value"); in.b = input(n, 1, "min"); in.c = input(n, 2, "max");
            return emit(in);
        }
        if (t == "shnoise") {
            in.op = OpNoise; in.i0 = parmI(n, "type");
            in.a = posInput(n, 0); in.b = input(n, 1, "freq"); in.c = input(n, 2, "offset"); in.d = input(n, 3, "amp");
            in.f[0] = parmF(n, "octaves"); in.f[1] = parmF(n, "rough"); in.f[2] = parmF(n, "lacunarity");
            return emit(in);
        }
        if (t == "shvoronoi") {
            in.op = OpVoronoi; in.i0 = parmI(n, "output");
            in.a = posInput(n, 0); in.b = input(n, 1, "freq");
            in.f[0] = std::clamp(parmF(n, "jitter"), 0.f, 1.f);
            in.i1 = parmI(n, "seed");
            return emit(in);
        }
        if (t == "shchecker") {
            in.op = OpChecker;
            in.a = posInput(n, 0); in.b = input(n, 1, "size"); in.c = input(n, 2, "color1"); in.d = input(n, 3, "color2");
            return emit(in);
        }
        if (t == "shramp") {
            in.op = OpRamp; in.i0 = parmI(n, "interp");
            in.a = input(n, 0, "value");
            struct Stop { float pos; int reg; };
            Stop st[3] = {{parmF(n, "pos1"), input(n, 1, "color1")}, {parmF(n, "pos2"), input(n, 2, "color2")}, {parmF(n, "pos3"), input(n, 3, "color3")}};
            std::stable_sort(st, st + 3, [](const Stop& x, const Stop& y) { return x.pos < y.pos; });
            in.b = st[0].reg; in.c = st[1].reg; in.d = st[2].reg;
            for (int k = 0; k < 3; ++k) in.f[k] = st[k].pos;
            return emit(in);
        }
        if (t == "shtexture") {
            in.op = OpTex;
            const Node* src = n->inputs.empty() ? nullptr : n->inputs[0];
            in.a = src ? compile(src) : global(GUv);
            std::string path = parmS(n, "file");
            Vec3 tile = parmV(n, "tile");
            in.f[0] = tile.x; in.f[1] = tile.y;
            in.i0 = parmI(n, "mode");
            in.i1 = isLinearFile(path) ? 1 : 0;
            in.k = Vec3(1, 0, 1);
            if (path.empty()) P.warnings += n->name() + ": no texture file.\n";
            else {
                std::string e;
                in.tex = loadShaderTexture(path, e);
                if (!in.tex) P.warnings += n->name() + ": " + e + "\n";
            }
            return emit(in);
        }
        if (t == "shcompose") {
            in.op = OpCompose;
            in.a = input(n, 0, "x"); in.b = input(n, 1, "y"); in.c = input(n, 2, "z");
            return emit(in);
        }
        if (t == "shdecompose") {
            in.op = OpDecompose; in.i0 = parmI(n, "component");
            in.a = input(n, 0, "value");
            return emit(in);
        }
        if (t == "shrandom") {
            in.op = OpRandom; in.i0 = parmI(n, "output"); in.i1 = parmI(n, "seed");
            in.a = input(n, 0, "value");
            return emit(in);
        }
        if (t == "shwave") {
            in.op = OpWave; in.i0 = parmI(n, "type"); in.i1 = parmI(n, "profile");
            in.a = posInput(n, 0); in.b = input(n, 1, "freq"); in.c = input(n, 2, "distortion");
            in.f[0] = parmF(n, "detail"); in.f[1] = parmF(n, "phase");
            return emit(in);
        }
        if (t == "principledshader") { err = "A Shader Output cannot feed another node: " + n->name() + " feeds another node."; return reg(); }
        err = n->name() + " (" + n->type().label + ") is not a shading node. Inside a Material Builder use the Shading nodes.";
        return reg();
    }
};

} // namespace

// ------------------------------------------------------------------ evaluation

void ShaderProgram::run(const ShaderInputs& g, Vec3* r) const {
    std::copy(init.begin(), init.end(), r);
    for (const Ins& I : code) {
        Vec3& d = r[I.dst];
        switch (I.op) {
        case OpGlobal:
            switch (I.i0) {
            case GP: d = g.P; break;
            case GN: d = g.N; break;
            case GUv: d = g.uv; break;
            case GCd: d = g.Cd; break;
            case GTime: d = Vec3(g.time); break;
            default: d = Vec3(g.frame); break;
            }
            break;
        case OpAttrib: d = g.attribs ? g.attribs[I.i0] : I.k; break;
        case OpBin: {
            const Vec3 a = r[I.a], b = r[I.b];
            switch (I.i0) {
            case 0: d = a + b; break;
            case 1: d = a - b; break;
            case 2: d = a * b; break;
            case 3: for (int c = 0; c < 3; ++c) d[c] = b[c] != 0 ? a[c] / b[c] : 0.f; break;
            case 4: for (int c = 0; c < 3; ++c) d[c] = std::pow(std::max(a[c], 0.f), b[c]); break;
            case 5: d = vmin(a, b); break;
            case 6: d = vmax(a, b); break;
            case 7: for (int c = 0; c < 3; ++c) d[c] = b[c] != 0 ? a[c] - b[c] * std::floor(a[c] / b[c]) : 0.f; break;
            case 8: d = Vec3(dot(a, b)); break;
            case 9: d = cross(a, b); break;
            case 10: d = Vec3(length(a - b)); break;
            default: for (int c = 0; c < 3; ++c) d[c] = std::atan2(a[c], b[c]); break;
            }
            break;
        }
        case OpUn: {
            const Vec3 a = r[I.a];
            switch (I.i0) {
            case 0: d = vabs(a); break;
            case 1: d = vfloor(a); break;
            case 2: for (int c = 0; c < 3; ++c) d[c] = std::ceil(a[c]); break;
            case 3: for (int c = 0; c < 3; ++c) d[c] = fract(a[c]); break;
            case 4: for (int c = 0; c < 3; ++c) d[c] = std::sin(a[c]); break;
            case 5: for (int c = 0; c < 3; ++c) d[c] = std::cos(a[c]); break;
            case 6: for (int c = 0; c < 3; ++c) d[c] = std::sqrt(std::max(a[c], 0.f)); break;
            case 7: d = -a; break;
            case 8: d = Vec3(1) - a; break;
            case 9: d = Vec3(length(a)); break;
            case 10: d = normalize(a); break;
            case 11: for (int c = 0; c < 3; ++c) d[c] = std::exp(std::min(a[c], 80.f)); break;
            case 12: for (int c = 0; c < 3; ++c) d[c] = std::log(std::max(a[c], 1e-20f)); break;
            case 13: d = Vec3(lumf(a)); break;
            default: for (int c = 0; c < 3; ++c) d[c] = float((a[c] > 0) - (a[c] < 0)); break;
            }
            break;
        }
        case OpMix: d = r[I.a] + (r[I.b] - r[I.a]) * r[I.c]; break;
        case OpFit: {
            const Vec3 v = r[I.a], s0 = r[I.b], s1 = r[I.c], d0 = r[I.d], d1 = r[I.e];
            for (int c = 0; c < 3; ++c) {
                float den = s1[c] - s0[c];
                float t = std::fabs(den) > 1e-20f ? (v[c] - s0[c]) / den : 0.f;
                if (I.i0) t = std::clamp(t, 0.f, 1.f);
                d[c] = d0[c] + t * (d1[c] - d0[c]);
            }
            break;
        }
        case OpClamp: d = vclamp(r[I.a], r[I.b], r[I.c]); break;
        case OpSmooth: {
            const Vec3 v = r[I.a], e0 = r[I.b], e1 = r[I.c];
            for (int c = 0; c < 3; ++c) {
                float den = e1[c] - e0[c];
                float t = std::fabs(den) > 1e-20f ? std::clamp((v[c] - e0[c]) / den, 0.f, 1.f) : (v[c] >= e1[c] ? 1.f : 0.f);
                d[c] = t * t * (3 - 2 * t);
            }
            break;
        }
        case OpNoise: {
            Vec3 q = r[I.a] * r[I.b] + r[I.c];
            d = fractal(q, I.i0, int(I.f[0]), I.f[1], I.f[2]) * r[I.d].x;
            break;
        }
        case OpVoronoi: {
            const Vec3 q = r[I.a] * r[I.b];
            const int cx = fl(q.x), cy = fl(q.y), cz = fl(q.z);
            float f1 = 1e30f, f2 = 1e30f;
            uint32_t id1 = 0;
            const uint32_t seed = uint32_t(I.i1) * 2654435761u;
            for (int dz = -1; dz <= 1; ++dz)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        int x = cx + dx, y = cy + dy, z = cz + dz;
                        uint32_t h = hash3i(x, y, z, seed);
                        Vec3 fp(float(x) + 0.5f + (unit(h) - 0.5f) * I.f[0], float(y) + 0.5f + (unit(mix32(h + 1)) - 0.5f) * I.f[0],
                                float(z) + 0.5f + (unit(mix32(h + 2)) - 0.5f) * I.f[0]);
                        Vec3 dv = q - fp;
                        float dd = dot(dv, dv);
                        if (dd < f1) { f2 = f1; f1 = dd; id1 = h; }
                        else if (dd < f2) f2 = dd;
                    }
            f1 = std::sqrt(f1); f2 = std::sqrt(f2);
            switch (I.i0) {
            case 0: d = Vec3(f1); break;
            case 1: d = Vec3(f2 - f1); break;
            case 2: d = Vec3(unit(mix32(id1 + 7))); break;
            default: d = Vec3(unit(mix32(id1 + 11)), unit(mix32(id1 + 13)), unit(mix32(id1 + 17))); break;
            }
            break;
        }
        case OpChecker: {
            float s = std::max(1e-6f, r[I.b].x);
            const Vec3 q = r[I.a] / s + Vec3(1e-4f);
            long par = long(fl(q.x)) + long(fl(q.y)) + long(fl(q.z));
            d = (par & 1) ? r[I.d] : r[I.c];
            break;
        }
        case OpRamp: {
            float t = r[I.a].x;
            const Vec3 c0 = r[I.b], c1 = r[I.c], c2 = r[I.d];
            auto seg = [&](const Vec3& x, const Vec3& y, float p0, float p1) {
                float u = p1 > p0 ? std::clamp((t - p0) / (p1 - p0), 0.f, 1.f) : 1.f;
                if (I.i0 == 1) u = u * u * (3 - 2 * u);
                else if (I.i0 == 2) u = u < 1 ? 0.f : 1.f;
                return x + (y - x) * u;
            };
            if (t <= I.f[0]) d = c0;
            else if (t < I.f[1]) d = seg(c0, c1, I.f[0], I.f[1]);
            else if (t < I.f[2]) d = seg(c1, c2, I.f[1], I.f[2]);
            else d = c2;
            break;
        }
        case OpTex: {
            if (!I.tex || !I.tex->valid()) { d = I.k; break; }
            const Vec3 uv = r[I.a];
            float px[4];
            I.tex->sample(uv.x * I.f[0], uv.y * I.f[1], px);
            d = Vec3(px[0], px[1], px[2]);
            // Colours in a network are display colours: a linear (EXR/HDR) colour map is converted.
            if (I.i0 == 0 && I.i1)
                for (int c = 0; c < 3; ++c) d[c] = std::pow(std::max(d[c], 0.f), 1.f / 2.2f);
            break;
        }
        case OpCompose: d = Vec3(r[I.a].x, r[I.b].x, r[I.c].x); break;
        case OpDecompose: {
            const Vec3 a = r[I.a];
            switch (I.i0) {
            case 0: d = Vec3(a.x); break;
            case 1: d = Vec3(a.y); break;
            case 2: d = Vec3(a.z); break;
            case 3: d = Vec3(lumf(a)); break;
            default: d = Vec3(std::max(a.x, std::max(a.y, a.z))); break;
            }
            break;
        }
        case OpRandom: {
            const Vec3 a = r[I.a];
            uint32_t h = hash3i(fl(a.x), fl(a.y), fl(a.z), uint32_t(I.i1) * 2246822519u);
            d = I.i0 == 0 ? Vec3(unit(h)) : Vec3(unit(h), unit(mix32(h + 1)), unit(mix32(h + 2)));
            break;
        }
        case OpWave: {
            const Vec3 p = r[I.a];
            float s;
            switch (I.i0) {
            case 0: s = p.x; break;
            case 1: s = p.y; break;
            case 2: s = p.z; break;
            case 3: s = std::sqrt(p.x * p.x + p.z * p.z); break;
            default: s = length(p); break;
            }
            s = s * r[I.b].x + I.f[1];
            float dist = r[I.c].x;
            if (dist != 0) s += dist * fbm3(p * std::max(1e-4f, I.f[0]), 4);
            float v;
            if (I.i1 == 0) v = 0.5f + 0.5f * std::sin(2 * kPi * s);
            else if (I.i1 == 1) v = fract(s);
            else v = 1 - std::fabs(2 * fract(s) - 1);
            d = Vec3(v);
            break;
        }
        default: break;
        }
    }
}

void ShaderProgram::eval(const ShaderInputs& in, ShaderOutputs& out) const {
    Vec3 stackRegs[192];
    std::vector<Vec3> heap;
    Vec3* r = stackRegs;
    if (nregs > 192) { heap.resize(size_t(nregs)); r = heap.data(); }
    run(in, r);
    out.base = oBase >= 0 ? r[oBase] : Vec3(0.8f);
    out.rough = oRough >= 0 ? std::clamp(r[oRough].x, 0.f, 1.f) : 0.5f;
    out.metal = oMetal >= 0 ? std::clamp(r[oMetal].x, 0.f, 1.f) : 0.f;
    out.spec = oSpec >= 0 ? std::clamp(r[oSpec].x, 0.f, 1.f) : 0.5f;
    out.transmit = oTransmit >= 0 ? std::clamp(r[oTransmit].x, 0.f, 1.f) : 0.f;
    out.ior = oIor >= 0 ? std::clamp(r[oIor].x, 1.0001f, 4.f) : 1.5f;
    out.sss = oSss >= 0 ? std::clamp(r[oSss].x, 0.f, 1.f) : 0.f;
    out.emitColor = oEmit >= 0 ? vmax(r[oEmit], Vec3(0)) : Vec3(1);
    out.emitStrength = oEmitStrength >= 0 ? std::max(0.f, r[oEmitStrength].x) : 0.f;
    out.N = in.N;
    if (oBump >= 0 && bumpScale != 0) {
        // height gradient by forward differences in P; only its part along the surface tilts N
        const float h0 = r[oBump].x, e = 1e-3f;
        ShaderInputs q = in;
        Vec3 grad;
        for (int k = 0; k < 3; ++k) {
            q.P = in.P;
            q.P[k] += e;
            run(q, r);
            grad[k] = (r[oBump].x - h0) / e;
        }
        Vec3 N = normalize(in.N);
        Vec3 sg = grad - N * dot(N, grad);
        Vec3 nb = normalize(N - sg * bumpScale);
        if (dot(nb, nb) > 0.5f) out.N = nb;
    }
}

std::shared_ptr<ShaderProgram> compileShader(const Graph& net, float frame, float fps, std::string& error) {
    const Node* out = nullptr;
    if (net.displayNode() && net.displayNode()->type().name == "principledshader") out = net.displayNode();
    for (auto& n : net.nodes())
        if (!out && n->type().name == "principledshader") out = n.get();
    if (!out) { error = "The Material Builder has no Shader Output node inside."; return nullptr; }
    auto prog = std::make_shared<ShaderProgram>();
    Compiler cc{net, frame, fps, *prog, {}, {}, {}, {-1, -1, -1, -1, -1, -1}, {}};
    prog->outputNode = out->name();
    prog->oBase = cc.input(out, 0, "basecolor");
    prog->oRough = cc.input(out, 1, "rough");
    prog->oMetal = cc.input(out, 2, "metal");
    prog->oSpec = cc.input(out, 3, "spec");
    prog->oEmit = cc.input(out, 4, "emitcolor");
    prog->oEmitStrength = cc.input(out, 5, "emitstrength");
    prog->oTransmit = cc.input(out, 6, "transmit");
    prog->oIor = cc.input(out, 7, "ior");
    prog->oSss = cc.input(out, 8, "sss");
    if (out->inputs.size() > 9 && out->inputs[9]) prog->oBump = cc.compile(out->inputs[9]);
    prog->bumpScale = cc.parmF(out, "bumpscale");
    if (!cc.err.empty()) { error = cc.err; return nullptr; }
    return prog;
}

int registerShader(std::shared_ptr<const ShaderProgram> p) {
    std::lock_guard<std::mutex> lock(gRegMutex);
    gRegistry.push_back(std::move(p));
    return int(gRegistry.size());
}

void updateShader(int id, std::shared_ptr<const ShaderProgram> p) {
    std::lock_guard<std::mutex> lock(gRegMutex);
    if (id >= 1 && id <= int(gRegistry.size())) gRegistry[size_t(id - 1)] = std::move(p);
}

std::shared_ptr<const ShaderProgram> shaderById(int id) {
    std::lock_guard<std::mutex> lock(gRegMutex);
    if (id < 1 || id > int(gRegistry.size())) return nullptr;
    return gRegistry[size_t(id - 1)];
}

// ------------------------------------------------------------------ Material Builder

namespace {

struct BuilderState { int id = 0; };

// Moves a prim attribute onto the points (prim values win, as they do in the renderers).
Attrib& pointChannel(Geometry& g, const char* name, int tuple, Vec3 def) {
    if (const Attrib* q = g.findAttrib(AttribClass::Prim, name)) {
        Attrib copy = *q;
        g.removeAttrib(AttribClass::Prim, name);
        Attrib* p = g.findAttrib(AttribClass::Point, name);
        if (!p) p = &g.addAttrib(AttribClass::Point, name, copy.tuple, def);
        for (size_t i = 0; i < g.numPrims(); ++i)
            for (int k = 0; k < g.prims[i].count; ++k) p->setV(size_t(g.primVertex(int(i), k)), copy.getV(i));
        return *p;
    }
    if (Attrib* p = g.findAttrib(AttribClass::Point, name)) return *p;
    return g.addAttrib(AttribClass::Point, name, tuple, def);
}

void cookMaterialBuilder(CookContext& c) {
    c.out = c.input(0);
    Geometry& g = c.out;
    Graph* net = c.node().subnet();
    if (!net) { c.error("The Material Builder has no network."); return; }
    std::string err;
    auto prog = compileShader(*net, c.frame(), c.fps(), err);
    if (!prog) { c.error(err); return; }
    if (!prog->warnings.empty()) c.warning(prog->warnings.substr(0, prog->warnings.size() - 1));
    auto& st = c.state<BuilderState>();
    if (!st.id) st.id = registerShader(prog);
    else updateShader(st.id, prog);
    if (!g.numPrims() && !g.numPoints()) {
        if (!g.packed.empty()) c.warning("The input is packed: apply the Material Builder before packing.");
        return;
    }

    const bool prims = g.numPrims() > 0;
    std::vector<char> sel;
    if (!selectElements(g, prims ? AttribClass::Prim : AttribClass::Point, c.s("group"), c.frame(), c.fps(), sel, err)) { c.error(err); return; }
    std::vector<char> ptSel(g.numPoints(), prims ? 0 : 1);
    if (prims) {
        for (size_t i = 0; i < g.numPrims(); ++i)
            if (sel[i])
                for (int k = 0; k < g.prims[i].count; ++k) ptSel[size_t(g.primVertex(int(i), k))] = 1;
    } else ptSel = sel;

    Attrib& sh = g.addAttrib(prims ? AttribClass::Prim : AttribClass::Point, "shader", 1, Vec3(0));
    for (size_t i = 0; i < sel.size(); ++i) if (sel[i]) sh.setF(i, float(st.id));
    for (const char* m : {"basemap", "roughmap", "metalmap", "normalmap"}) {
        if (Attrib* a = g.findAttrib(AttribClass::Prim, m)) for (size_t i = 0; i < g.numPrims(); ++i) if (prims && sel[i]) a->setF(i, 0);
        if (Attrib* a = g.findAttrib(AttribClass::Point, m)) for (size_t i = 0; i < g.numPoints(); ++i) if (ptSel[i]) a->setF(i, 0);
    }

    // inputs per point
    const size_t np = g.numPoints();
    std::vector<Vec3> N(np, Vec3(0)), Cd(np, Vec3(0.72f, 0.74f, 0.78f));
    if (const Attrib* pN = g.findAttrib(AttribClass::Point, "N"); pN && pN->tuple == 3) {
        for (size_t i = 0; i < np; ++i) N[i] = pN->getV(i);
    } else {
        for (size_t i = 0; i < g.numPrims(); ++i) {
            if (g.prims[i].count < 3) continue;
            Vec3 fn = g.primNormal(int(i));
            for (int k = 0; k < g.prims[i].count; ++k) N[size_t(g.primVertex(int(i), k))] += fn;
        }
    }
    for (auto& n : N) n = dot(n, n) > 1e-20f ? normalize(n) : Vec3(0, 1, 0);
    if (const Attrib* pc = g.findAttrib(AttribClass::Point, "Cd")) for (size_t i = 0; i < np; ++i) Cd[i] = pc->getV(i);
    else if (const Attrib* qc = g.findAttrib(AttribClass::Prim, "Cd"))
        for (size_t i = 0; i < g.numPrims(); ++i)
            for (int k = 0; k < g.prims[i].count; ++k) Cd[size_t(g.primVertex(int(i), k))] = qc->getV(i);
    const Attrib* uvA = g.findAttrib(AttribClass::Point, "uv");
    const size_t na = prog->attribs.size();
    std::vector<Vec3> attrVals(np * na, Vec3(0));
    for (size_t a = 0; a < na; ++a) {
        if (const Attrib* pa = g.findAttrib(AttribClass::Point, prog->attribs[a])) {
            for (size_t i = 0; i < np; ++i) attrVals[i * na + a] = pa->getV(i);
        } else if (const Attrib* qa = g.findAttrib(AttribClass::Prim, prog->attribs[a])) {
            for (size_t i = 0; i < g.numPrims(); ++i)
                for (int k = 0; k < g.prims[i].count; ++k) attrVals[size_t(g.primVertex(int(i), k)) * na + a] = qa->getV(i);
        } else {
            c.warning("Attribute '" + prog->attribs[a] + "' is not on the geometry: its default is used.");
            for (size_t i = 0; i < np; ++i) attrVals[i * na + a] = prog->attribDefaults[a];
        }
    }
    if (prog->usesCd) {
        Attrib& sc = g.addAttrib(AttribClass::Point, "shadercd", 3, Vec3(0.72f, 0.74f, 0.78f));
        for (size_t i = 0; i < np; ++i) if (ptSel[i]) sc.setV(i, Cd[i]);
    }

    // bake the viewport preview onto the points
    std::vector<ShaderOutputs> res(np);
    const float frame = c.frame(), time = c.time();
    const ShaderProgram& P = *prog;
    parallelFor(int(np), [&](int b, int e) {
        for (int i = b; i < e; ++i) {
            if (!ptSel[size_t(i)]) continue;
            ShaderInputs in;
            in.P = g.P[size_t(i)];
            in.N = N[size_t(i)];
            in.uv = uvA ? uvA->getV(size_t(i)) : Vec3(0);
            in.Cd = Cd[size_t(i)];
            in.time = time; in.frame = frame;
            in.attribs = na ? &attrVals[size_t(i) * na] : nullptr;
            P.eval(in, res[size_t(i)]);
        }
    });
    Attrib& aCd = pointChannel(g, "Cd", 3, Vec3(0.72f, 0.74f, 0.78f));
    Attrib& aR = pointChannel(g, "rough", 1, Vec3(0.65f));
    Attrib& aM = pointChannel(g, "metal", 1, Vec3(0));
    Attrib& aS = pointChannel(g, "spec", 1, Vec3(0.5f));
    Attrib& aT = pointChannel(g, "transmit", 1, Vec3(0));
    Attrib& aSss = pointChannel(g, "sss", 1, Vec3(0));
    Attrib& aE = pointChannel(g, "emit", 1, Vec3(0));
    for (size_t i = 0; i < np; ++i) {
        if (!ptSel[i]) continue;
        const ShaderOutputs& o = res[i];
        aCd.setV(i, o.base);
        aR.setF(i, o.rough); aM.setF(i, o.metal); aS.setF(i, o.spec); aT.setF(i, o.transmit); aSss.setF(i, o.sss);
        // the viewport multiplies colour by emit: carry the emission colour's brightness relative to the base
        float lb = std::max(1e-3f, lumf(o.base));
        aE.setF(i, o.emitStrength > 0 ? o.emitStrength * lumf(o.emitColor) / lb : 0.f);
    }
}

void noCook(CookContext& c) {
    if (!c.graph().owner() || c.graph().owner()->type().name != "materialbuilder")
        c.warning("Shading nodes work inside a Material Builder.");
}

} // namespace

void registerShaderNodes(std::vector<NodeType>& types) {
    auto add = [&](NodeType t, std::vector<std::string> inputs) {
        t.category = "Shading";
        t.minInputs = 0;
        t.maxInputs = int(inputs.size());
        t.inputLabels = std::move(inputs);
        if (!t.cook) t.cook = noCook;
        types.push_back(std::move(t));
    };
    const std::vector<std::string> domain = {"Position (P)", "Texture (uv)"};
    {
        NodeType t; t.name = "principledshader"; t.label = "Shader Output";
        t.description = "The principled surface the Material Builder produces. Wire patterns into its inputs; unconnected inputs use the parameter values.";
        t.parms = {C("basecolor", "Base Color", Vec3(0.8f)), F("rough", "Roughness", 0.5f), F("metal", "Metallic", 0),
                   F("spec", "Specular", 0.5f), C("emitcolor", "Emission Color", Vec3(1)), F("emitstrength", "Emission Strength", 0, 0, 10),
                   F("transmit", "Transmission", 0), F("ior", "Index of Refraction", 1.5f, 1, 3), F("sss", "Subsurface", 0),
                   F("bump", "Bump Height", 0, 0, 1, "Wire a pattern in: its slope tilts the shading normal"),
                   F("bumpscale", "Bump Scale", 0.05f, -1, 1, "How strongly the bump height tilts the normal")};
        add(t, {"Base Color", "Roughness", "Metallic", "Specular", "Emission Color", "Emission Strength", "Transmission", "IOR", "Subsurface", "Bump"});
    }
    {
        NodeType t; t.name = "shglobals"; t.label = "Global Variables";
        t.description = "Reads a value of the shading point: position P, normal N, texture coordinates uv or the colour Cd.";
        t.parms = {M("global", "Output", {"P (position)", "N (normal)", "uv", "Cd (colour)"})};
        add(t, {});
    }
    {
        NodeType t; t.name = "shtime"; t.label = "Time";
        t.description = "The current time in seconds, or the frame number. Makes the shader animate.";
        t.parms = {M("which", "Output", {"Time (seconds)", "Frame"})};
        t.timeDependent = true;
        add(t, {});
    }
    {
        NodeType t; t.name = "shconstant"; t.label = "Constant Color";
        t.description = "A colour or vector value.";
        t.parms = {C("value", "Value", Vec3(0.5f))};
        add(t, {});
    }
    {
        NodeType t; t.name = "shfloat"; t.label = "Constant Float";
        t.description = "A single number.";
        t.parms = {F("value", "Value", 0.5f, 0, 1)};
        add(t, {});
    }
    {
        NodeType t; t.name = "shattrib"; t.label = "Bind Attribute";
        t.description = "Reads a point or primitive attribute of the shaded geometry (interpolated across faces).";
        t.parms = {S("name", "Attribute", "rest"), V("default", "Default", Vec3(0), -1, 1, "Used when the geometry lacks the attribute")};
        add(t, {});
    }
    {
        NodeType t; t.name = "shmath"; t.label = "Math";
        t.description = "Combines two values per component (dot, cross and distance give vectors or scalars).";
        t.parms = {M("op", "Operation", {"Add", "Subtract", "Multiply", "Divide", "Power", "Minimum", "Maximum", "Modulo", "Dot", "Cross", "Distance", "Atan2"}, 2),
                   V("a", "A", Vec3(1), 0, 1), V("b", "B", Vec3(1), 0, 1)};
        add(t, {"A", "B"});
    }
    {
        NodeType t; t.name = "shfunc"; t.label = "Function";
        t.description = "Applies a function to a value per component.";
        t.parms = {M("op", "Function", {"Absolute", "Floor", "Ceiling", "Fraction", "Sine", "Cosine", "Square Root", "Negate", "One Minus",
                                        "Length", "Normalize", "Exponential", "Logarithm", "Luminance", "Sign"}, 8),
                   V("value", "Value", Vec3(0), -1, 1)};
        add(t, {"Value"});
    }
    {
        NodeType t; t.name = "shmix"; t.label = "Mix";
        t.description = "Blends from A to B by Bias (0 = A, 1 = B).";
        t.parms = {C("a", "A", Vec3(0)), C("b", "B", Vec3(1)), F("bias", "Bias", 0.5f)};
        add(t, {"A", "B", "Bias"});
    }
    {
        NodeType t; t.name = "shfit"; t.label = "Fit Range";
        t.description = "Remaps a value from one range to another.";
        t.parms = {F("value", "Value", 0), F("srcmin", "Source Min", 0), F("srcmax", "Source Max", 1), F("destmin", "Destination Min", 0),
                   F("destmax", "Destination Max", 1), T("clamp", "Clamp", true)};
        add(t, {"Value", "Source Min", "Source Max", "Destination Min", "Destination Max"});
    }
    {
        NodeType t; t.name = "shclamp"; t.label = "Clamp";
        t.description = "Limits a value to a range.";
        t.parms = {F("value", "Value", 0), F("min", "Minimum", 0), F("max", "Maximum", 1)};
        add(t, {"Value", "Minimum", "Maximum"});
    }
    {
        NodeType t; t.name = "shsmooth"; t.label = "Smooth Step";
        t.description = "0 below Minimum, 1 above Maximum, a smooth S-curve between.";
        t.parms = {F("value", "Value", 0), F("min", "Minimum", 0.4f), F("max", "Maximum", 0.6f)};
        add(t, {"Value", "Minimum", "Maximum"});
    }
    {
        NodeType t; t.name = "shnoise"; t.label = "Noise";
        t.description = "Perlin noise patterns (about 0 to 1; Vector gives -1 to 1 per component). Unconnected position uses P or uv.";
        t.parms = {M("domain", "Domain", domain), M("type", "Type", {"Perlin", "fBm", "Turbulence", "Ridged", "Vector fBm"}, 1),
                   V("freq", "Frequency", Vec3(2), 0, 20), V("offset", "Offset", Vec3(0), -10, 10), F("amp", "Amplitude", 1, 0, 2),
                   I("octaves", "Octaves", 5, 1, 10), F("rough", "Roughness", 0.5f, 0, 1, "How much each finer octave adds"),
                   F("lacunarity", "Lacunarity", 2, 1, 4, "Frequency step between octaves")};
        add(t, {"Position", "Frequency", "Offset", "Amplitude"});
    }
    {
        NodeType t; t.name = "shvoronoi"; t.label = "Voronoi (Cells)";
        t.description = "Cellular patterns: distance to the nearest cell point, cell edges, or a random value / colour per cell.";
        t.parms = {M("domain", "Domain", domain), M("output", "Output", {"Distance", "Edges", "Cell Value", "Cell Color"}),
                   V("freq", "Frequency", Vec3(4), 0, 20), F("jitter", "Jitter", 1, 0, 1), I("seed", "Seed", 0, 0, 100)};
        add(t, {"Position", "Frequency"});
    }
    {
        NodeType t; t.name = "shchecker"; t.label = "Checker";
        t.description = "A 3D (or uv) checkerboard of two colours.";
        t.parms = {M("domain", "Domain", domain), F("size", "Size", 0.25f, 0.01f, 2), C("color1", "Color 1", Vec3(0.9f)), C("color2", "Color 2", Vec3(0.1f))};
        add(t, {"Position", "Size", "Color 1", "Color 2"});
    }
    {
        NodeType t; t.name = "shramp"; t.label = "Color Ramp";
        t.description = "Maps a value to a colour with three stops.";
        t.parms = {F("value", "Value", 0.5f), M("interp", "Interpolation", {"Linear", "Smooth", "Constant"}),
                   F("pos1", "Stop 1 Position", 0), C("color1", "Stop 1 Color", Vec3(0)),
                   F("pos2", "Stop 2 Position", 0.5f), C("color2", "Stop 2 Color", Vec3(0.5f)),
                   F("pos3", "Stop 3 Position", 1), C("color3", "Stop 3 Color", Vec3(1))};
        add(t, {"Value", "Stop 1 Color", "Stop 2 Color", "Stop 3 Color"});
    }
    {
        NodeType t; t.name = "shtexture"; t.label = "Texture";
        t.description = "Looks up an image (PNG, EXR, HDR, <UDIM> tiles) by uv. Unconnected uv reads the geometry's uv.";
        t.parms = {FileP("file", "Texture File", "PNG, EXR or HDR; <UDIM> in the name for tiles"),
                   M("mode", "Contents", {"Color", "Data (roughness, masks, heights)"}),
                   V("tile", "UV Scale", Vec3(1, 1, 1), 0, 10, "Repeats the texture (x: u, y: v)")};
        add(t, {"UV"});
    }
    {
        NodeType t; t.name = "shcompose"; t.label = "Combine XYZ";
        t.description = "Builds a vector (or colour) from three numbers.";
        t.parms = {F("x", "X", 0), F("y", "Y", 0), F("z", "Z", 0)};
        add(t, {"X", "Y", "Z"});
    }
    {
        NodeType t; t.name = "shdecompose"; t.label = "Separate";
        t.description = "Takes one component of a vector, or the brightness of a colour.";
        t.parms = {M("component", "Component", {"X / Red", "Y / Green", "Z / Blue", "Luminance", "Maximum"}), V("value", "Value", Vec3(0))};
        add(t, {"Value"});
    }
    {
        NodeType t; t.name = "shrandom"; t.label = "Random";
        t.description = "A random value or colour for each integer cell of the input (use with Floor or a Voronoi cell value).";
        t.parms = {M("output", "Output", {"Float", "Color"}), I("seed", "Seed", 0, 0, 100), V("value", "Value", Vec3(0))};
        add(t, {"Value"});
    }
    {
        NodeType t; t.name = "shwave"; t.label = "Wave";
        t.description = "Stripes and rings, optionally distorted by noise: wood, marble, sand ripples.";
        t.parms = {M("domain", "Domain", domain), M("type", "Type", {"Bands X", "Bands Y", "Bands Z", "Rings (around Y)", "Spherical"}),
                   M("profile", "Profile", {"Sine", "Saw", "Triangle"}), F("freq", "Frequency", 4, 0, 40),
                   F("distortion", "Distortion", 0, 0, 5, "Noise added to the wave coordinate"), F("detail", "Noise Scale", 2, 0.1f, 20),
                   F("phase", "Phase", 0, 0, 1)};
        add(t, {"Position", "Frequency", "Distortion"});
    }
    {
        NodeType t; t.name = "materialbuilder"; t.label = "Material Builder"; t.category = "Attributes";
        t.description = "A surface shader built from Shading nodes (double-click to edit the network). The Render View evaluates the "
                        "network at every shading point (bump included); the viewport shows colour and material baked onto the points.";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {S("group", "Group", "", "Numbers (0-5 8) or expression (@P.y > 0); empty = everything")};
        t.cook = cookMaterialBuilder;
        types.push_back(std::move(t));
    }
}

} // namespace hd

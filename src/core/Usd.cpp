#include "Usd.h"
#include "Image.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

namespace hd {
namespace {

std::string fmt(float v) {
    if (!(v == v)) v = 0;
    char b[32];
    std::snprintf(b, sizeof b, "%.7g", double(v));
    return b;
}
std::string v3(const Vec3& v) { return "(" + fmt(v.x) + ", " + fmt(v.y) + ", " + fmt(v.z) + ")"; }
std::string v2(float a, float b) { return "(" + fmt(a) + ", " + fmt(b) + ")"; }
inline Vec3 lin(const Vec3& c) { return Vec3(std::pow(std::max(c.x, 0.f), 2.2f), std::pow(std::max(c.y, 0.f), 2.2f), std::pow(std::max(c.z, 0.f), 2.2f)); }

// asset paths are written absolute so other applications find the files wherever the .usda goes
std::string assetPath(const std::string& p) {
    std::error_code ec;
    auto a = std::filesystem::absolute(std::filesystem::path(p), ec);
    return ec ? p : a.generic_string();
}

std::string safeName(const std::string& n) {
    std::string s;
    for (char c : n) s += (std::isalnum((unsigned char)c) || c == '_') ? c : '_';
    if (s.empty() || std::isdigit((unsigned char)s[0])) s = "_" + s;
    return s;
}

// row-major matrix with a row-vector convention (USD): rows are the x, y, z axes and the translation
std::string matrixFrom(const Vec3& eye, const Vec3& fwd) {
    Vec3 z = normalize(-fwd);
    Vec3 up(0, 1, 0);
    if (std::fabs(dot(up, z)) > 0.999f) up = Vec3(0, 0, 1);
    Vec3 x = normalize(cross(up, z)), y = cross(z, x);
    return "( (" + fmt(x.x) + ", " + fmt(x.y) + ", " + fmt(x.z) + ", 0), (" + fmt(y.x) + ", " + fmt(y.y) + ", " + fmt(y.z) + ", 0), (" + fmt(z.x) + ", " +
           fmt(z.y) + ", " + fmt(z.z) + ", 0), (" + fmt(eye.x) + ", " + fmt(eye.y) + ", " + fmt(eye.z) + ", 1) )";
}

struct MatKey {
    float rough = 0.65f, metal = 0, emit = 0, transmit = 0, ior = 1.5f, spec = 0.5f;
    int base = 0, roughMap = 0, metalMap = 0, normalMap = 0;
    bool operator<(const MatKey& o) const {
        return std::tie(rough, metal, emit, transmit, ior, spec, base, roughMap, metalMap, normalMap) <
               std::tie(o.rough, o.metal, o.emit, o.transmit, o.ior, o.spec, o.base, o.roughMap, o.metalMap, o.normalMap);
    }
};

float attrF(const Geometry& g, const char* name, int prim, int pt, float def) {
    if (const Attrib* a = g.findAttrib(AttribClass::Prim, name)) if (prim >= 0) return a->getF(size_t(prim));
    if (const Attrib* a = g.findAttrib(AttribClass::Point, name)) if (pt >= 0) return a->getF(size_t(pt));
    return def;
}

} // namespace

bool writeUsda(const std::string& path, const UsdExport& o, const std::function<bool(float, Geometry&, std::string*)>& cook, std::string* err) {
    Geometry g0;
    if (!cook(float(o.start), g0, err)) return false;
    const bool anim = o.end > o.start;
    // classify primitives
    std::vector<int> faces, curves;
    std::vector<char> used(g0.numPoints(), 0);
    for (size_t pi = 0; pi < g0.numPrims(); ++pi) {
        const Prim& pr = g0.prims[pi];
        for (int k = 0; k < pr.count; ++k) used[size_t(g0.vertices[size_t(pr.start + k)])] = 1;
        if (pr.closed && pr.count >= 3) faces.push_back(int(pi));
        else if (pr.count >= 2) curves.push_back(int(pi));
    }
    std::vector<int> loose;
    for (size_t i = 0; i < g0.numPoints(); ++i) if (!used[i]) loose.push_back(int(i));

    // sample the animation (points must keep their count)
    std::vector<std::pair<int, std::vector<Vec3>>> samples;
    std::vector<std::pair<int, std::vector<Vec3>>> nsamples;
    const bool pointN = g0.findAttrib(AttribClass::Point, "N") && g0.findAttrib(AttribClass::Point, "N")->tuple == 3;
    if (anim) {
        for (int f = o.start; f <= o.end; ++f) {
            Geometry gf;
            if (f == o.start) gf = g0;
            else if (!cook(float(f), gf, err)) return false;
            if (gf.numPoints() != g0.numPoints() || gf.numPrims() != g0.numPrims()) {
                if (err) *err = "The point count changes at frame " + std::to_string(f) + "; USD animation export needs a fixed topology (export single frames instead).";
                return false;
            }
            samples.push_back({f, gf.P});
            if (pointN) {
                const Attrib* N = gf.findAttrib(AttribClass::Point, "N");
                std::vector<Vec3> n(gf.numPoints());
                for (size_t i = 0; i < n.size() && N; ++i) n[i] = N->getV(i);
                nsamples.push_back({f, n});
            }
        }
    }

    std::ostringstream s;
    s << "#usda 1.0\n(\n    defaultPrim = \"World\"\n    metersPerUnit = 1\n    upAxis = \"Y\"\n";
    if (anim) s << "    startTimeCode = " << o.start << "\n    endTimeCode = " << o.end << "\n";
    s << "    timeCodesPerSecond = " << fmt(o.fps) << "\n    doc = \"Exported by HD\"\n)\n\n";
    s << "def Xform \"World\"\n{\n";

    auto writePoints = [&](const std::string& ind, const std::string& type, const std::string& name, const std::vector<int>& sel) {
        // sel = point indices (in order) used by this prim
        if (!anim) {
            s << ind << type << " " << name << " = [";
            for (size_t i = 0; i < sel.size(); ++i) s << (i ? ", " : "") << v3(g0.P[size_t(sel[i])]);
            s << "]\n";
            return;
        }
        s << ind << type << " " << name << ".timeSamples = {\n";
        for (auto& [f, P] : samples) {
            s << ind << "    " << f << ": [";
            for (size_t i = 0; i < sel.size(); ++i) s << (i ? ", " : "") << v3(P[size_t(sel[i])]);
            s << "],\n";
        }
        s << ind << "}\n";
    };

    // materials
    std::map<MatKey, int> matIds;
    std::vector<MatKey> mats;
    std::vector<int> faceMat(faces.size(), 0);
    const bool hasMatAttrs = g0.findAttrib(AttribClass::Prim, "rough") || g0.findAttrib(AttribClass::Point, "rough") ||
                             g0.findAttrib(AttribClass::Prim, "basemap") || g0.findAttrib(AttribClass::Prim, "transmit");
    for (size_t fi = 0; fi < faces.size(); ++fi) {
        int pi = faces[fi], pt = g0.primVertex(pi, 0);
        MatKey k;
        k.rough = attrF(g0, "rough", pi, pt, 0.65f); k.metal = attrF(g0, "metal", pi, pt, 0); k.emit = attrF(g0, "emit", pi, pt, 0);
        k.transmit = attrF(g0, "transmit", pi, pt, 0); k.ior = attrF(g0, "ior", pi, pt, 1.5f); k.spec = attrF(g0, "spec", pi, pt, 0.5f);
        k.base = int(attrF(g0, "basemap", pi, pt, 0) + 0.5f); k.roughMap = int(attrF(g0, "roughmap", pi, pt, 0) + 0.5f);
        k.metalMap = int(attrF(g0, "metalmap", pi, pt, 0) + 0.5f); k.normalMap = int(attrF(g0, "normalmap", pi, pt, 0) + 0.5f);
        auto it = matIds.find(k);
        if (it == matIds.end()) { it = matIds.emplace(k, int(mats.size())).first; mats.push_back(k); }
        faceMat[fi] = it->second;
    }
    const Attrib* pCd = g0.findAttrib(AttribClass::Point, "Cd");
    const Attrib* qCd = g0.findAttrib(AttribClass::Prim, "Cd");
    const Attrib* pUv = g0.findAttrib(AttribClass::Point, "uv");
    if (pCd && pCd->tuple != 3) pCd = nullptr;
    if (qCd && qCd->tuple != 3) qCd = nullptr;

    if (!faces.empty()) {
        // the mesh uses all points, so indices stay the same
        std::vector<int> allPts(g0.numPoints());
        for (size_t i = 0; i < allPts.size(); ++i) allPts[i] = int(i);
        const bool bind = hasMatAttrs && !mats.empty();
        s << "    def Mesh \"geo\" (\n        prepend apiSchemas = [\"MaterialBindingAPI\"]\n    )\n    {\n";
        s << "        int[] faceVertexCounts = [";
        for (size_t i = 0; i < faces.size(); ++i) s << (i ? ", " : "") << g0.prims[size_t(faces[i])].count;
        s << "]\n        int[] faceVertexIndices = [";
        bool first = true;
        for (int pi : faces)
            for (int k = 0; k < g0.prims[size_t(pi)].count; ++k) { s << (first ? "" : ", ") << g0.primVertex(pi, k); first = false; }
        s << "]\n";
        writePoints("        ", "point3f[]", "points", allPts);
        Vec3 mn, mx;
        g0.bounds(mn, mx);
        s << "        float3[] extent = [" << v3(mn) << ", " << v3(mx) << "]\n";
        if (pointN) {
            if (!anim) {
                const Attrib* N = g0.findAttrib(AttribClass::Point, "N");
                s << "        normal3f[] normals = [";
                for (size_t i = 0; i < g0.numPoints(); ++i) s << (i ? ", " : "") << v3(N->getV(i));
                s << "] (\n            interpolation = \"vertex\"\n        )\n";
            } else {
                s << "        normal3f[] normals (\n            interpolation = \"vertex\"\n        )\n";
                s << "        normal3f[] normals.timeSamples = {\n";
                for (auto& [f, n] : nsamples) {
                    s << "            " << f << ": [";
                    for (size_t i = 0; i < n.size(); ++i) s << (i ? ", " : "") << v3(n[i]);
                    s << "],\n";
                }
                s << "        }\n";
            }
        }
        if (pCd) {
            s << "        color3f[] primvars:displayColor = [";
            for (size_t i = 0; i < g0.numPoints(); ++i) s << (i ? ", " : "") << v3(lin(pCd->getV(i)));
            s << "] (\n            interpolation = \"vertex\"\n        )\n";
        } else if (qCd) {
            s << "        color3f[] primvars:displayColor = [";
            for (size_t i = 0; i < faces.size(); ++i) s << (i ? ", " : "") << v3(lin(qCd->getV(size_t(faces[i]))));
            s << "] (\n            interpolation = \"uniform\"\n        )\n";
        }
        if (pUv && pUv->tuple == 3) {
            s << "        texCoord2f[] primvars:st = [";
            for (size_t i = 0; i < g0.numPoints(); ++i) s << (i ? ", " : "") << v2(pUv->getV(i).x, pUv->getV(i).y);
            s << "] (\n            interpolation = \"vertex\"\n        )\n";
        }
        s << "        uniform token orientation = \"rightHanded\"\n        uniform token subdivisionScheme = \"none\"\n";
        if (bind && mats.size() == 1) s << "        rel material:binding = </World/Looks/mat0>\n";
        if (bind && mats.size() > 1) {
            for (size_t m = 0; m < mats.size(); ++m) {
                s << "\n        def GeomSubset \"mat" << m << "_faces\" (\n            prepend apiSchemas = [\"MaterialBindingAPI\"]\n        )\n        {\n";
                s << "            uniform token elementType = \"face\"\n            uniform token familyName = \"materialBind\"\n            int[] indices = [";
                bool f1 = true;
                for (size_t fi = 0; fi < faces.size(); ++fi) if (faceMat[fi] == int(m)) { s << (f1 ? "" : ", ") << fi; f1 = false; }
                s << "]\n            rel material:binding = </World/Looks/mat" << m << ">\n        }\n";
            }
        }
        s << "    }\n\n";
    }
    if (!curves.empty()) {
        std::vector<int> sel;
        for (int pi : curves) for (int k = 0; k < g0.prims[size_t(pi)].count; ++k) sel.push_back(g0.primVertex(pi, k));
        s << "    def BasisCurves \"curves\"\n    {\n        uniform token type = \"linear\"\n        int[] curveVertexCounts = [";
        for (size_t i = 0; i < curves.size(); ++i) s << (i ? ", " : "") << g0.prims[size_t(curves[i])].count;
        s << "]\n";
        writePoints("        ", "point3f[]", "points", sel);
        s << "        float[] widths = [0.01] (\n            interpolation = \"constant\"\n        )\n    }\n\n";
    }
    if (!loose.empty()) {
        s << "    def Points \"points\"\n    {\n";
        writePoints("        ", "point3f[]", "points", loose);
        const Attrib* ps = g0.findAttrib(AttribClass::Point, "pscale");
        s << "        float[] widths = [";
        for (size_t i = 0; i < loose.size(); ++i) s << (i ? ", " : "") << fmt(ps ? ps->getF(size_t(loose[i])) * 2 : 0.02f);
        s << "] (\n            interpolation = \"vertex\"\n        )\n";
        if (pCd) {
            s << "        color3f[] primvars:displayColor = [";
            for (size_t i = 0; i < loose.size(); ++i) s << (i ? ", " : "") << v3(lin(pCd->getV(size_t(loose[i]))));
            s << "] (\n            interpolation = \"vertex\"\n        )\n";
        }
        s << "    }\n\n";
    }
    // materials (UsdPreviewSurface)
    if (hasMatAttrs && !mats.empty() && !faces.empty()) {
        s << "    def Scope \"Looks\"\n    {\n";
        for (size_t m = 0; m < mats.size(); ++m) {
            const MatKey& k = mats[m];
            std::string mp = "/World/Looks/mat" + std::to_string(m);
            s << "        def Material \"mat" << m << "\"\n        {\n";
            s << "            token outputs:surface.connect = <" << mp << "/surface.outputs:surface>\n";
            s << "            def Shader \"surface\"\n            {\n                uniform token info:id = \"UsdPreviewSurface\"\n";
            if (k.base) s << "                color3f inputs:diffuseColor.connect = <" << mp << "/baseTex.outputs:rgb>\n";
            else if (pCd || qCd) s << "                color3f inputs:diffuseColor.connect = <" << mp << "/colorReader.outputs:result>\n";
            else s << "                color3f inputs:diffuseColor = " << v3(lin(Vec3(0.72f, 0.74f, 0.78f))) << "\n";
            if (k.roughMap) s << "                float inputs:roughness.connect = <" << mp << "/roughTex.outputs:r>\n";
            else s << "                float inputs:roughness = " << fmt(k.rough) << "\n";
            if (k.metalMap) s << "                float inputs:metallic.connect = <" << mp << "/metalTex.outputs:r>\n";
            else s << "                float inputs:metallic = " << fmt(k.metal) << "\n";
            if (k.normalMap) s << "                normal3f inputs:normal.connect = <" << mp << "/normalTex.outputs:rgb>\n";
            s << "                float inputs:ior = " << fmt(k.ior) << "\n";
            s << "                float inputs:opacity = " << fmt(1 - k.transmit) << "\n";
            s << "                color3f inputs:specularColor = " << v3(Vec3(k.spec * 2)) << "\n";
            if (k.emit > 0) s << "                color3f inputs:emissiveColor = " << v3(Vec3(k.emit)) << "\n";
            s << "                token outputs:surface\n            }\n";
            if (pCd || qCd)
                s << "            def Shader \"colorReader\"\n            {\n                uniform token info:id = \"UsdPrimvarReader_float3\"\n"
                     "                string inputs:varname = \"displayColor\"\n                float3 outputs:result\n            }\n";
            bool anyTex = k.base || k.roughMap || k.metalMap || k.normalMap;
            if (anyTex)
                s << "            def Shader \"stReader\"\n            {\n                uniform token info:id = \"UsdPrimvarReader_float2\"\n"
                     "                string inputs:varname = \"st\"\n                float2 outputs:result\n            }\n";
            auto tex = [&](const char* name, int id, bool srgb, bool normal) {
                if (!id) return;
                s << "            def Shader \"" << name << "\"\n            {\n                uniform token info:id = \"UsdUVTexture\"\n";
                s << "                asset inputs:file = @" << assetPath(resolvePath(texturePath(id))) << "@\n";
                s << "                float2 inputs:st.connect = <" << mp << "/stReader.outputs:result>\n";
                s << "                token inputs:sourceColorSpace = \"" << (srgb ? "sRGB" : "raw") << "\"\n";
                if (normal) s << "                float4 inputs:scale = (2, 2, 2, 1)\n                float4 inputs:bias = (-1, -1, -1, 0)\n";
                s << "                float3 outputs:rgb\n                float outputs:r\n            }\n";
            };
            tex("baseTex", k.base, true, false);
            tex("roughTex", k.roughMap, false, false);
            tex("metalTex", k.metalMap, false, false);
            tex("normalTex", k.normalMap, false, true);
            s << "        }\n";
        }
        s << "    }\n\n";
    }
    // cameras
    for (auto& cam : o.cameras) {
        s << "    def Camera \"" << safeName(cam.name) << "\"\n    {\n";
        s << "        float focalLength = " << fmt(cam.focal) << "\n";
        s << "        float horizontalAperture = " << fmt(cam.aperture) << "\n";
        s << "        float verticalAperture = " << fmt(cam.aperture * float(cam.resy) / float(cam.resx)) << "\n";
        s << "        float2 clippingRange = (0.01, 10000)\n";
        if (cam.dof) s << "        float fStop = " << fmt(cam.fstop) << "\n        float focusDistance = " << fmt(cam.focus) << "\n";
        s << "        matrix4d xformOp:transform = " << matrixFrom(cam.eye, cam.fwd) << "\n";
        s << "        uniform token[] xformOpOrder = [\"xformOp:transform\"]\n    }\n\n";
    }
    // lights
    int li = 0;
    for (auto& l : o.lights) {
        std::string type = "SphereLight", name = l.name.empty() ? "light" + std::to_string(li) : safeName(l.name);
        ++li;
        if (l.type == Light::Area) type = "RectLight";
        else if (l.type == Light::Distant) type = "DistantLight";
        else if (l.type == Light::Env) type = "DomeLight";
        s << "    def " << type << " \"" << name << "\"" << (l.type == Light::Spot ? " (\n        prepend apiSchemas = [\"ShapingAPI\"]\n    )" : "") << "\n    {\n";
        s << "        float inputs:intensity = " << fmt(l.intensity) << "\n        color3f inputs:color = " << v3(l.color) << "\n";
        if (l.type == Light::Point || l.type == Light::Spot) s << "        float inputs:radius = " << fmt(std::max(l.radius, 1e-4f)) << "\n";
        if (l.type == Light::Spot) s << "        float inputs:shaping:cone:angle = " << fmt(l.coneAngle * 0.5f) << "\n        float inputs:shaping:cone:softness = " << fmt(std::clamp(l.coneDelta / std::max(1.f, l.coneAngle), 0.f, 1.f)) << "\n";
        if (l.type == Light::Area) s << "        float inputs:width = " << fmt(l.sizeX) << "\n        float inputs:height = " << fmt(l.sizeY) << "\n";
        if (l.type == Light::Distant) s << "        float inputs:angle = " << fmt(l.angle) << "\n";
        if (l.type == Light::Env && !l.map.empty()) s << "        asset inputs:texture:file = @" << assetPath(resolvePath(l.map)) << "@\n        token inputs:texture:format = \"latlong\"\n";
        if (l.type != Light::Env) {
            s << "        matrix4d xformOp:transform = " << matrixFrom(l.pos, l.dir) << "\n";
            s << "        uniform token[] xformOpOrder = [\"xformOp:transform\"]\n";
        }
        s << "    }\n\n";
    }
    s << "}\n";
    std::ofstream f(path, std::ios::binary);
    if (!f) { if (err) *err = "Cannot write " + path; return false; }
    f << s.str();
    return bool(f);
}

// ================================================================== import

namespace {

struct Tok { enum Kind { Ident, String, Path, Asset, Number, Punct, End } kind; std::string text; };

struct Lexer {
    const std::string& s;
    size_t p = 0;
    explicit Lexer(const std::string& src) : s(src) {}
    Tok next() {
        for (;;) {
            while (p < s.size() && std::isspace((unsigned char)s[p])) ++p;
            if (p < s.size() && s[p] == '#') { while (p < s.size() && s[p] != '\n') ++p; continue; }
            break;
        }
        if (p >= s.size()) return {Tok::End, ""};
        char c = s[p];
        if (c == '"') {
            // "..." or """..."""
            if (s.compare(p, 3, "\"\"\"") == 0) {
                size_t e = s.find("\"\"\"", p + 3);
                std::string t = s.substr(p + 3, e == std::string::npos ? std::string::npos : e - p - 3);
                p = e == std::string::npos ? s.size() : e + 3;
                return {Tok::String, t};
            }
            std::string t;
            ++p;
            while (p < s.size() && s[p] != '"') { if (s[p] == '\\' && p + 1 < s.size()) ++p; t += s[p++]; }
            ++p;
            return {Tok::String, t};
        }
        if (c == '@') {
            size_t e = s.find('@', p + 1);
            std::string t = s.substr(p + 1, e == std::string::npos ? std::string::npos : e - p - 1);
            p = e == std::string::npos ? s.size() : e + 1;
            return {Tok::Asset, t};
        }
        if (c == '<') {
            size_t e = s.find('>', p + 1);
            std::string t = s.substr(p + 1, e == std::string::npos ? std::string::npos : e - p - 1);
            p = e == std::string::npos ? s.size() : e + 1;
            return {Tok::Path, t};
        }
        if (std::isdigit((unsigned char)c) || ((c == '-' || c == '+' || c == '.') && p + 1 < s.size() && (std::isdigit((unsigned char)s[p + 1]) || s[p + 1] == '.'))) {
            size_t b = p++;
            while (p < s.size() && (std::isalnum((unsigned char)s[p]) || s[p] == '.' || s[p] == '-' || s[p] == '+')) ++p;
            return {Tok::Number, s.substr(b, p - b)};
        }
        if (std::isalpha((unsigned char)c) || c == '_') {
            size_t b = p++;
            while (p < s.size() && (std::isalnum((unsigned char)s[p]) || s[p] == '_' || s[p] == ':' || s[p] == '.' || s[p] == '[' || s[p] == ']')) {
                if (s[p] == '[' && (p + 1 >= s.size() || s[p + 1] != ']')) break;
                ++p;
            }
            return {Tok::Ident, s.substr(b, p - b)};
        }
        ++p;
        return {Tok::Punct, std::string(1, c)};
    }
};

struct UPrim {
    std::string type, name;
    std::map<std::string, std::vector<double>> attrs; // numeric values, flattened
    std::map<std::string, std::string> tokens;
    std::vector<std::string> opOrder;
    std::vector<UPrim> children;
};

struct Parser {
    Lexer lx;
    Tok cur;
    std::string err;
    explicit Parser(const std::string& s) : lx(s) { cur = lx.next(); }
    void adv() { cur = lx.next(); }
    bool punct(char c) const { return cur.kind == Tok::Punct && cur.text[0] == c; }
    // skips a balanced (...) / [...] / {...} group, or a single token
    void skipValue() {
        if (cur.kind == Tok::Punct && (cur.text[0] == '(' || cur.text[0] == '[' || cur.text[0] == '{')) {
            char open = cur.text[0], close = open == '(' ? ')' : open == '[' ? ']' : '}';
            int depth = 0;
            do {
                if (cur.kind == Tok::End) return;
                if (cur.kind == Tok::Punct && cur.text[0] == open) ++depth;
                if (cur.kind == Tok::Punct && cur.text[0] == close) --depth;
                adv();
            } while (depth > 0);
            return;
        }
        adv();
    }
    // reads numbers from a value (scalar, tuple, array, nested), returns false for non-numeric
    void numbers(std::vector<double>& out, std::vector<std::string>* strs) {
        if (cur.kind == Tok::Punct && (cur.text[0] == '(' || cur.text[0] == '[')) {
            char open = cur.text[0], close = open == '(' ? ')' : ']';
            int depth = 0;
            do {
                if (cur.kind == Tok::End) return;
                if (cur.kind == Tok::Punct && (cur.text[0] == '(' || cur.text[0] == '[')) ++depth;
                else if (cur.kind == Tok::Punct && (cur.text[0] == ')' || cur.text[0] == ']')) --depth;
                else if (cur.kind == Tok::Number) out.push_back(std::atof(cur.text.c_str()));
                else if (cur.kind == Tok::String && strs) strs->push_back(cur.text);
                (void)close;
                adv();
            } while (depth > 0);
            return;
        }
        if (cur.kind == Tok::Number) out.push_back(std::atof(cur.text.c_str()));
        if ((cur.kind == Tok::String || cur.kind == Tok::Ident) && strs) strs->push_back(cur.text);
        adv();
    }
    bool parseBody(UPrim& prim) {
        // until the closing brace
        while (cur.kind != Tok::End && !punct('}')) {
            if (cur.kind == Tok::Ident && (cur.text == "def" || cur.text == "over" || cur.text == "class")) {
                UPrim child;
                if (!parsePrim(child)) return false;
                prim.children.push_back(std::move(child));
                continue;
            }
            if (cur.kind == Tok::Ident && cur.text == "variantSet") {
                // variant sets: skip
                adv(); adv(); if (punct('=')) adv();
                skipValue();
                continue;
            }
            // attribute or relationship: [custom] [uniform] type name [= value] [(meta)]
            std::vector<std::string> words;
            while (cur.kind == Tok::Ident) { words.push_back(cur.text); adv(); }
            if (words.empty()) { adv(); continue; }
            std::string name = words.back();
            if (punct('=')) {
                adv();
                bool timeSamples = name.size() > 12 && name.compare(name.size() - 12, 12, ".timeSamples") == 0;
                if (timeSamples) {
                    // { t: value, ... } -> first sample
                    std::string base = name.substr(0, name.size() - 12);
                    if (punct('{')) {
                        adv();
                        bool first = true;
                        while (cur.kind != Tok::End && !punct('}')) {
                            if (cur.kind == Tok::Number) { adv(); if (punct(':')) adv(); std::vector<double> v; numbers(v, nullptr); if (first) prim.attrs[base] = v; first = false; }
                            else adv();
                            if (punct(',')) adv();
                        }
                        adv();
                    }
                } else if (name == "xformOpOrder") {
                    std::vector<double> dummy;
                    numbers(dummy, &prim.opOrder);
                } else {
                    std::vector<double> v;
                    std::vector<std::string> strs;
                    numbers(v, &strs);
                    if (!v.empty()) prim.attrs[name] = v;
                    if (!strs.empty()) prim.tokens[name] = strs[0];
                }
            }
            if (punct('(')) skipValue(); // attribute metadata
            if (name.size() > 14 && name.compare(name.size() - 14, 14, "interpolation") == 0) {}
        }
        return true;
    }
    bool parsePrim(UPrim& prim) {
        adv(); // def/over/class
        if (cur.kind == Tok::Ident) { prim.type = cur.text; adv(); }
        if (cur.kind != Tok::String) { err = "Expected a prim name"; return false; }
        prim.name = cur.text;
        adv();
        if (punct('(')) {
            // metadata: pick up interpolation hints is unnecessary; skip
            skipValue();
        }
        if (!punct('{')) { err = "Expected { after prim " + prim.name; return false; }
        adv();
        if (!parseBody(prim)) return false;
        adv(); // }
        return true;
    }
};

struct M4 { double m[4][4]; };
M4 ident() { M4 r{}; for (int i = 0; i < 4; ++i) r.m[i][i] = 1; return r; }
M4 mul(const M4& a, const M4& b) { M4 r{}; for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) for (int k = 0; k < 4; ++k) r.m[i][j] += a.m[i][k] * b.m[k][j]; return r; }
Vec3 xfPoint(const M4& m, const Vec3& p) { // row vector * matrix
    return Vec3(float(p.x * m.m[0][0] + p.y * m.m[1][0] + p.z * m.m[2][0] + m.m[3][0]), float(p.x * m.m[0][1] + p.y * m.m[1][1] + p.z * m.m[2][1] + m.m[3][1]),
                float(p.x * m.m[0][2] + p.y * m.m[1][2] + p.z * m.m[2][2] + m.m[3][2]));
}
Vec3 xfDir(const M4& m, const Vec3& p) {
    return Vec3(float(p.x * m.m[0][0] + p.y * m.m[1][0] + p.z * m.m[2][0]), float(p.x * m.m[0][1] + p.y * m.m[1][1] + p.z * m.m[2][1]),
                float(p.x * m.m[0][2] + p.y * m.m[1][2] + p.z * m.m[2][2]));
}

M4 localXform(const UPrim& p) {
    M4 r = ident();
    // xformOps compose in xformOpOrder; with row vectors the first op listed is applied last
    for (auto it = p.opOrder.rbegin(); it != p.opOrder.rend(); ++it) {
        const std::string& op = *it;
        auto a = p.attrs.find(op);
        if (a == p.attrs.end()) continue;
        const std::vector<double>& v = a->second;
        M4 m = ident();
        if (op.find("xformOp:transform") == 0 && v.size() >= 16) { for (int i = 0; i < 16; ++i) m.m[i / 4][i % 4] = v[size_t(i)]; }
        else if (op.find("xformOp:translate") == 0 && v.size() >= 3) { m.m[3][0] = v[0]; m.m[3][1] = v[1]; m.m[3][2] = v[2]; }
        else if (op.find("xformOp:scale") == 0 && v.size() >= 3) { m.m[0][0] = v[0]; m.m[1][1] = v[1]; m.m[2][2] = v[2]; }
        else if (op.find("xformOp:rotateXYZ") == 0 && v.size() >= 3) {
            // XYZ: X first. Row vectors: M = Rx * Ry * Rz
            auto rot = [](int axis, double deg) {
                M4 r = ident();
                double c = std::cos(deg * 3.14159265358979 / 180), s = std::sin(deg * 3.14159265358979 / 180);
                int a = (axis + 1) % 3, b = (axis + 2) % 3;
                r.m[a][a] = c; r.m[a][b] = s; r.m[b][a] = -s; r.m[b][b] = c;
                return r;
            };
            m = mul(mul(rot(0, v[0]), rot(1, v[1])), rot(2, v[2]));
        } else continue;
        r = mul(r, m);
    }
    return r;
}

void collect(const UPrim& p, const M4& parent, Geometry& out) {
    M4 world = mul(localXform(p), parent);
    auto get = [&](const char* n) -> const std::vector<double>* { auto it = p.attrs.find(n); return it == p.attrs.end() ? nullptr : &it->second; };
    const std::vector<double>* pts = get("points");
    if (pts && (p.type == "Mesh" || p.type == "Points" || p.type == "BasisCurves")) {
        Geometry g;
        for (size_t i = 0; i + 2 < pts->size(); i += 3) g.addPoint(xfPoint(world, Vec3(float((*pts)[i]), float((*pts)[i + 1]), float((*pts)[i + 2]))));
        const size_t np = g.numPoints();
        bool lefty = false;
        { auto t = p.tokens.find("orientation"); lefty = t != p.tokens.end() && t->second == "leftHanded"; }
        if (p.type == "Mesh") {
            const std::vector<double>* cnt = get("faceVertexCounts");
            const std::vector<double>* idx = get("faceVertexIndices");
            if (cnt && idx) {
                size_t k = 0;
                for (double cd : *cnt) {
                    int c = int(cd);
                    std::vector<int> f;
                    for (int j = 0; j < c && k < idx->size(); ++j, ++k) { int v = int((*idx)[k]); if (v >= 0 && size_t(v) < np) f.push_back(v); }
                    if (lefty) std::reverse(f.begin(), f.end());
                    if (f.size() >= 3) g.addPrim(f, true);
                }
            }
        } else if (p.type == "BasisCurves") {
            const std::vector<double>* cnt = get("curveVertexCounts");
            size_t k = 0;
            if (cnt)
                for (double cd : *cnt) {
                    std::vector<int> f;
                    for (int j = 0; j < int(cd) && k < np; ++j) f.push_back(int(k++));
                    if (f.size() >= 2) g.addPrim(f, false);
                }
        }
        if (const std::vector<double>* n = get("normals"); n && n->size() == np * 3) {
            Attrib& N = g.addAttrib(AttribClass::Point, "N", 3);
            for (size_t i = 0; i < np; ++i) N.setV(i, normalize(xfDir(world, Vec3(float((*n)[i * 3]), float((*n)[i * 3 + 1]), float((*n)[i * 3 + 2])))));
        }
        if (const std::vector<double>* c = get("primvars:displayColor")) {
            // displayColor is linear; Cd is display-referred here
            if (c->size() == np * 3) {
                Attrib& Cd = g.addAttrib(AttribClass::Point, "Cd", 3);
                for (size_t i = 0; i < np; ++i) Cd.setV(i, Vec3(std::pow(float((*c)[i * 3]), 1 / 2.2f), std::pow(float((*c)[i * 3 + 1]), 1 / 2.2f), std::pow(float((*c)[i * 3 + 2]), 1 / 2.2f)));
            } else if (c->size() == g.numPrims() * 3 && g.numPrims()) {
                Attrib& Cd = g.addAttrib(AttribClass::Prim, "Cd", 3);
                for (size_t i = 0; i < g.numPrims(); ++i) Cd.setV(i, Vec3(std::pow(float((*c)[i * 3]), 1 / 2.2f), std::pow(float((*c)[i * 3 + 1]), 1 / 2.2f), std::pow(float((*c)[i * 3 + 2]), 1 / 2.2f)));
            } else if (c->size() == 3) {
                Attrib& Cd = g.addAttrib(AttribClass::Point, "Cd", 3);
                for (size_t i = 0; i < np; ++i) Cd.setV(i, Vec3(std::pow(float((*c)[0]), 1 / 2.2f), std::pow(float((*c)[1]), 1 / 2.2f), std::pow(float((*c)[2]), 1 / 2.2f)));
            }
        }
        if (const std::vector<double>* st = get("primvars:st"); st && st->size() == np * 2) {
            Attrib& uv = g.addAttrib(AttribClass::Point, "uv", 3);
            for (size_t i = 0; i < np; ++i) uv.setV(i, Vec3(float((*st)[i * 2]), float((*st)[i * 2 + 1]), 0));
        }
        if (p.type == "Points") {
            if (const std::vector<double>* w = get("widths"); w && !w->empty()) {
                Attrib& ps = g.addAttrib(AttribClass::Point, "pscale", 1);
                for (size_t i = 0; i < np; ++i) ps.setF(i, float((*w)[std::min(i, w->size() - 1)]) * 0.5f);
            }
        }
        g.syncAttribSizes();
        out.merge(g);
    }
    for (auto& c : p.children) collect(c, world, out);
}

} // namespace

bool loadUsda(const std::string& path, Geometry& out, std::string* err) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { if (err) *err = "Cannot open " + path; return false; }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();
    if (text.rfind("#usda", 0) != 0) {
        if (err) *err = text.rfind("PXR-USDC", 0) == 0 ? "Binary .usd/.usdc files are not supported; save as .usda" : "Not a .usda file";
        return false;
    }
    Parser ps(text);
    if (ps.punct('(')) ps.skipValue(); // layer metadata
    UPrim root;
    while (ps.cur.kind != Tok::End) {
        if (ps.cur.kind == Tok::Ident && (ps.cur.text == "def" || ps.cur.text == "over" || ps.cur.text == "class")) {
            UPrim p;
            if (!ps.parsePrim(p)) { if (err) *err = ps.err; return false; }
            root.children.push_back(std::move(p));
        } else ps.adv();
    }
    out.clear();
    for (auto& c : root.children) collect(c, ident(), out);
    if (out.numPoints() == 0) { if (err) *err = "No Mesh, Points or BasisCurves prims found"; return false; }
    return true;
}

} // namespace hd

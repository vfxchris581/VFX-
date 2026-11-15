// Shading helpers: UV Project.
#include "Node.h"
#include <algorithm>
#include <cmath>

namespace hd {
namespace {

ParmDef V(const char* n, const char* l, Vec3 d, float mn = -1, float mx = 1, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Vec3; p.def[0] = d.x; p.def[1] = d.y; p.def[2] = d.z; p.min = mn; p.max = mx; p.help = help; return p;
}
ParmDef M(const char* n, const char* l, std::vector<std::string> items, int d = 0) {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Menu; p.menu = std::move(items); p.def[0] = float(d); return p;
}

// Gives every primitive its own points so uvs can differ across seams.
Geometry unshare(const Geometry& in) {
    Geometry out;
    for (auto& [name, a] : in.pointAttribs) { Attrib& o = out.addAttrib(AttribClass::Point, name, a.tuple); for (int k = 0; k < 3; ++k) o.defaults[k] = a.defaults[k]; }
    out.primAttribs = in.primAttribs;
    out.volumes = in.volumes;
    std::vector<char> used(in.numPoints(), 0);
    for (size_t pi = 0; pi < in.numPrims(); ++pi) {
        const Prim& pr = in.prims[pi];
        std::vector<int> ids;
        for (int k = 0; k < pr.count; ++k) {
            int src = in.vertices[size_t(pr.start + k)];
            used[size_t(src)] = 1;
            int id = out.addPoint(in.P[size_t(src)]);
            out.syncAttribSizes();
            out.copyPointAttribs(in, src, id);
            ids.push_back(id);
        }
        out.addPrim(ids, pr.closed);
    }
    // keep loose points
    for (size_t i = 0; i < in.numPoints(); ++i)
        if (!used[i]) { int id = out.addPoint(in.P[i]); out.syncAttribSizes(); out.copyPointAttribs(in, int(i), id); }
    out.primAttribs = in.primAttribs; // addPrim may have grown them; restore exact values
    out.syncAttribSizes();
    return out;
}

void cookUvProject(CookContext& c) {
    const int method = c.i("method");
    const int axis = std::clamp(c.i("axis"), 0, 2);
    Geometry g = method == 0 ? c.input(0) : unshare(c.input(0));
    Vec3 mn, mx;
    g.bounds(mn, mx);
    Vec3 size = vmax(mx - mn, Vec3(1e-6f));
    Vec3 ctr = (mn + mx) * 0.5f;
    const Vec3 scale = c.v("scale"), off = c.v("offset");
    Attrib& uv = g.addAttrib(AttribClass::Point, "uv", 3);
    g.syncAttribSizes();
    // the two axes across a projection plane whose normal is `ax`
    auto planeUv = [&](const Vec3& p, int ax) {
        int a = ax == 0 ? 2 : 0, b = ax == 1 ? 2 : 1;
        float u = (p[a] - mn[a]) / size[a], v = (p[b] - mn[b]) / size[b];
        if (ax == 0) u = 1 - u; // keep the image unmirrored when seen from +X
        if (ax == 1) v = 1 - v; // from +Y: z grows toward the viewer
        return Vec3(u, v, 0);
    };
    auto finish = [&](size_t i, Vec3 t) { uv.setV(i, Vec3(t.x * scale.x + off.x, t.y * scale.y + off.y, 0)); };
    if (method == 0) {
        for (size_t i = 0; i < g.numPoints(); ++i) finish(i, planeUv(g.P[i], axis));
    } else if (method == 1) {
        // box: each face uses the plane its normal points along
        for (size_t pi = 0; pi < g.numPrims(); ++pi) {
            Vec3 n = g.primNormal(int(pi));
            int ax = std::fabs(n.x) > std::fabs(n.y) ? (std::fabs(n.x) > std::fabs(n.z) ? 0 : 2) : (std::fabs(n.y) > std::fabs(n.z) ? 1 : 2);
            for (int k = 0; k < g.prims[pi].count; ++k) {
                int v = g.primVertex(int(pi), k);
                Vec3 t = planeUv(g.P[size_t(v)], ax);
                if (n[ax] < 0) t.x = 1 - t.x;
                finish(size_t(v), t);
            }
        }
    } else {
        // cylindrical (around axis) or spherical; seams fixed per primitive
        const int a = axis == 0 ? 1 : 0, b = axis == 2 ? 1 : 2;
        for (size_t pi = 0; pi < g.numPrims(); ++pi) {
            const Prim& pr = g.prims[pi];
            std::vector<Vec3> t(size_t(pr.count));
            for (int k = 0; k < pr.count; ++k) {
                Vec3 d = g.P[size_t(g.primVertex(int(pi), k))] - ctr;
                float u = std::atan2(d[a], d[b]) / (2 * kPi) + 0.5f;
                float v;
                if (method == 2) v = (d[axis] + size[axis] * 0.5f) / size[axis];
                else v = 1.f - std::acos(std::clamp(d[axis] / std::max(1e-9f, length(d)), -1.f, 1.f)) / kPi;
                t[size_t(k)] = Vec3(u, v, 0);
            }
            for (int k = 1; k < pr.count; ++k) {
                while (t[size_t(k)].x - t[0].x > 0.5f) t[size_t(k)].x -= 1;
                while (t[0].x - t[size_t(k)].x > 0.5f) t[size_t(k)].x += 1;
            }
            for (int k = 0; k < pr.count; ++k) finish(size_t(g.primVertex(int(pi), k)), t[size_t(k)]);
        }
    }
    c.out = std::move(g);
}

} // namespace

void registerShadingNodes(std::vector<NodeType>& types) {
    NodeType t; t.name = "uvproject"; t.label = "UV Project"; t.category = "Attributes";
    t.description = "Creates the uv attribute that texture maps use: planar, box, cylindrical or spherical projection fitted to the bounding box.";
    t.minInputs = 1; t.maxInputs = 1;
    t.parms = {M("method", "Projection", {"Planar", "Box (per face)", "Cylindrical", "Spherical"}, 1),
               M("axis", "Axis", {"X", "Y", "Z"}, 2),
               V("scale", "Scale (tiles)", Vec3(1, 1, 0), 0, 10, "How many times the texture repeats across the object"),
               V("offset", "Offset", Vec3(0), -1, 1)};
    t.cook = cookUvProject;
    types.push_back(t);
}

} // namespace hd

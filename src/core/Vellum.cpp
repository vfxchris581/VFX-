// Vellum Solver: XPBD (extended position-based dynamics) for cloth, ropes / hair and pressure soft bodies.
//
//  * Polygons become cloth: every edge is a stretch constraint and every pair of triangles sharing an
//    edge gets a bend constraint (distance between the two opposite points).
//  * Open polylines become ropes or hair: segment stretch plus a bend constraint over every two segments.
//  * Closed meshes can hold a pressure (volume) constraint, which turns them into soft, inflatable bodies.
//  * Pins: points with a "pin" attribute > 0.5 or matching the Pin Points group follow the input,
//    so pinned points can be animated.
//  * Collisions: ground plane, a sphere, the convex hulls of each part of the collider input (which may
//    be animated), and optional point-point self collision.
// XPBD with many small substeps makes stiffness independent of the step size.
#include "Fracture.h"
#include "Node.h"
#include "Ops.h"
#include "Volume.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <numeric>
#include <set>
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
ParmDef S(const char* n, const char* l, const char* d, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::String; p.defStr = d; p.help = help; return p;
}

struct DistC { int a, b; float rest, compliance; };

struct VellumSetup {
    std::vector<DistC> dist;          // stretch first, then bend
    size_t numStretch = 0;
    std::vector<int> volTris;         // closed-mesh triangles for the pressure constraint
    float restVolume = 0;
    std::vector<float> invMass;       // 0 = pinned
    std::vector<char> pinned;
    std::vector<std::vector<int>> neighbors; // points sharing an edge (excluded from self collision)
};

struct VellumFrame { std::vector<Vec3> P, V; };

struct VellumState {
    uint64_t sig = 0;
    bool ok = false;
    VellumSetup setup;
    Geometry rest;
    std::map<int, VellumFrame> frames;
};

float meshVolume(const std::vector<Vec3>& P, const std::vector<int>& tris) {
    double v = 0;
    for (size_t k = 0; k + 2 < tris.size(); k += 3)
        v += dot(P[size_t(tris[k])], cross(P[size_t(tris[k + 1])], P[size_t(tris[k + 2])]));
    return float(v / 6.0);
}

bool isClosedMesh(const Geometry& g) {
    std::map<std::pair<int, int>, int> cnt;
    size_t polys = 0;
    for (auto& pr : g.prims) {
        if (!pr.closed || pr.count < 3) continue;
        ++polys;
        for (int k = 0; k < pr.count; ++k) {
            int a = g.vertices[size_t(pr.start + k)], b = g.vertices[size_t(pr.start + (k + 1) % pr.count)];
            cnt[{std::min(a, b), std::max(a, b)}]++;
        }
    }
    if (!polys) return false;
    for (auto& [e, n] : cnt) if (n != 2) return false;
    return true;
}

struct Collider {
    std::vector<ConvexHull> hulls;
    void build(const Geometry& g) {
        hulls.clear();
        // connected parts
        std::vector<int> parent(g.numPoints());
        std::iota(parent.begin(), parent.end(), 0);
        std::function<int(int)> find = [&](int a) { while (parent[size_t(a)] != a) { parent[size_t(a)] = parent[size_t(parent[size_t(a)])]; a = parent[size_t(a)]; } return a; };
        for (auto& pr : g.prims)
            for (int k = 1; k < pr.count; ++k) {
                int a = find(g.vertices[size_t(pr.start)]), b = find(g.vertices[size_t(pr.start + k)]);
                if (a != b) parent[size_t(a)] = b;
            }
        std::map<int, std::vector<Vec3>> parts;
        std::vector<char> used(g.numPoints(), 0);
        for (int v : g.vertices) used[size_t(v)] = 1;
        for (size_t i = 0; i < g.numPoints(); ++i) if (used[i]) parts[find(int(i))].push_back(g.P[i]);
        for (auto& [r, pts] : parts) {
            ConvexHull h;
            if (pts.size() >= 4 && buildConvexHull(pts, h, 1e-3f) && h.valid()) hulls.push_back(std::move(h));
        }
    }
};

struct SimParams {
    float dt;
    int iterations;
    Vec3 gravity, wind;
    float drag, damping;
    float pressure, pressureCompliance;
    bool ground;
    float groundY;
    float friction;
    float thickness;
    bool sphere;
    Vec3 sphereC;
    float sphereR;
    bool self;
};

// Projects points out of colliders. contactN receives the push direction of points that touched a collider.
void collide(std::vector<Vec3>& x, const std::vector<Vec3>& xprev, const VellumSetup& su, const SimParams& sp, const Collider& col,
             std::vector<Vec3>& contactN) {
    const size_t n = x.size();
    const float th = sp.thickness;
    auto friction = [&](size_t i, const Vec3& nrm) {
        contactN[i] = nrm;
        // remove a share of the tangential motion this substep
        Vec3 d = x[i] - xprev[i];
        Vec3 dt = d - nrm * dot(d, nrm);
        x[i] -= dt * std::clamp(sp.friction, 0.f, 1.f);
    };
    for (size_t i = 0; i < n; ++i) {
        if (su.invMass[i] == 0) continue;
        if (sp.ground && x[i].y < sp.groundY + th) { x[i].y = sp.groundY + th; friction(i, Vec3(0, 1, 0)); }
        if (sp.sphere) {
            Vec3 d = x[i] - sp.sphereC;
            float l = length(d), r = sp.sphereR + th;
            if (l < r && l > 1e-9f) { Vec3 nn = d / l; x[i] = sp.sphereC + nn * r; friction(i, nn); }
        }
        for (const ConvexHull& h : col.hulls) {
            int pl = -1;
            float dd = h.distanceFrom(x[i], xprev[i], &pl);
            if (dd < th && pl >= 0) {
                // inside or within the thickness: push out through the face the point came in by
                const auto& P = h.planes[size_t(pl)];
                x[i] += P.n * (th - dd);
                friction(i, P.n);
            }
        }
    }
    if (sp.self && th > 0) {
        // point-point repulsion with a spatial hash; neighbours connected by an edge are skipped
        const float cs = 2 * th;
        std::unordered_map<int64_t, std::vector<int>> hash;
        auto key = [](int a, int b, int c) { return (int64_t(a) * 73856093) ^ (int64_t(b) * 19349663) ^ (int64_t(c) * 83492791); };
        for (size_t i = 0; i < n; ++i)
            hash[key(int(std::floor(x[i].x / cs)), int(std::floor(x[i].y / cs)), int(std::floor(x[i].z / cs)))].push_back(int(i));
        const float minD = 2 * th;
        for (size_t i = 0; i < n; ++i) {
            int a = int(std::floor(x[i].x / cs)), b = int(std::floor(x[i].y / cs)), c = int(std::floor(x[i].z / cs));
            for (int dz = -1; dz <= 1; ++dz) for (int dy = -1; dy <= 1; ++dy) for (int dxx = -1; dxx <= 1; ++dxx) {
                auto it = hash.find(key(a + dxx, b + dy, c + dz));
                if (it == hash.end()) continue;
                for (int j : it->second) {
                    if (j <= int(i)) continue;
                    const auto& nb = su.neighbors[i];
                    if (std::binary_search(nb.begin(), nb.end(), j)) continue;
                    Vec3 d = x[i] - x[size_t(j)];
                    float l = length(d);
                    if (l >= minD || l < 1e-9f) continue;
                    float wi = su.invMass[i], wj = su.invMass[size_t(j)];
                    if (wi + wj == 0) continue;
                    Vec3 corr = d / l * (minD - l) / (wi + wj);
                    x[i] += corr * wi;
                    x[size_t(j)] -= corr * wj;
                }
            }
        }
    }
}

void substep(std::vector<Vec3>& x, std::vector<Vec3>& v, const VellumSetup& su, const SimParams& sp, const Collider& col,
             const std::vector<Vec3>* pinTargets) {
    const size_t n = x.size();
    const float dt = sp.dt;
    std::vector<Vec3> xprev = x;
    for (size_t i = 0; i < n; ++i) {
        if (su.invMass[i] == 0) continue;
        Vec3 f = sp.gravity + (sp.wind - v[i]) * sp.drag;
        v[i] += f * dt;
        x[i] += v[i] * dt;
    }
    if (pinTargets)
        for (size_t i = 0; i < n; ++i) if (su.pinned[i]) x[i] = (*pinTargets)[i];
    std::vector<float> lambda(su.dist.size(), 0.f);
    float volLambda = 0;
    const float dt2 = dt * dt;
    for (int it = 0; it < sp.iterations; ++it) {
        for (size_t c = 0; c < su.dist.size(); ++c) {
            const DistC& d = su.dist[c];
            float wa = su.invMass[size_t(d.a)], wb = su.invMass[size_t(d.b)];
            if (wa + wb == 0) continue;
            Vec3 e = x[size_t(d.a)] - x[size_t(d.b)];
            float l = length(e);
            if (l < 1e-9f) continue;
            float C = l - d.rest;
            float at = d.compliance / dt2;
            float dl = (-C - at * lambda[c]) / (wa + wb + at);
            lambda[c] += dl;
            Vec3 g = e / l;
            x[size_t(d.a)] += g * (dl * wa);
            x[size_t(d.b)] -= g * (dl * wb);
        }
        if (!su.volTris.empty() && sp.pressure > 0) {
            // global volume constraint: C = V - pressure * V0
            std::vector<Vec3> grad(n, Vec3(0));
            for (size_t k = 0; k + 2 < su.volTris.size(); k += 3) {
                int a = su.volTris[k], b = su.volTris[k + 1], c = su.volTris[k + 2];
                grad[size_t(a)] += cross(x[size_t(b)], x[size_t(c)]) / 6.f;
                grad[size_t(b)] += cross(x[size_t(c)], x[size_t(a)]) / 6.f;
                grad[size_t(c)] += cross(x[size_t(a)], x[size_t(b)]) / 6.f;
            }
            float C = meshVolume(x, su.volTris) - sp.pressure * su.restVolume;
            float denom = 0;
            for (size_t i = 0; i < n; ++i) denom += su.invMass[i] * dot(grad[i], grad[i]);
            float at = sp.pressureCompliance / dt2;
            if (denom + at > 1e-20f) {
                float dl = (-C - at * volLambda) / (denom + at);
                volLambda += dl;
                for (size_t i = 0; i < n; ++i) x[i] += grad[i] * (dl * su.invMass[i]);
            }
        }
    }
    std::vector<Vec3> contactN(n, Vec3(0));
    collide(x, xprev, su, sp, col, contactN);
    const float keep = std::max(0.f, 1.f - sp.damping * dt);
    for (size_t i = 0; i < n; ++i) {
        v[i] = su.invMass[i] == 0 ? (x[i] - xprev[i]) / dt : (x[i] - xprev[i]) / dt * keep;
        // contacts are inelastic: being pushed out of a collider never launches a point
        float vn = dot(v[i], contactN[i]);
        if (vn > 0) v[i] -= contactN[i] * vn;
    }
}

bool buildSetup(CookContext& c, const Geometry& g, VellumSetup& su) {
    su = VellumSetup();
    const size_t n = g.numPoints();
    // mass: cloth area or rope length shared out to points
    std::vector<float> mass(n, 0.f);
    const float density = std::max(1e-4f, c.f("density"));
    std::set<std::pair<int, int>> edges;
    std::map<std::pair<int, int>, std::vector<int>> edgeOpp; // edge -> opposite point of each triangle using it
    for (size_t pi = 0; pi < g.numPrims(); ++pi) {
        const Prim& pr = g.prims[pi];
        if (pr.count < 2) continue;
        int ne = pr.closed && pr.count >= 3 ? pr.count : pr.count - 1;
        for (int k = 0; k < ne; ++k) {
            int a = g.vertices[size_t(pr.start + k)], b = g.vertices[size_t(pr.start + (k + 1) % pr.count)];
            if (a != b) edges.insert({std::min(a, b), std::max(a, b)});
        }
        if (pr.closed && pr.count >= 3) {
            float area = g.primArea(int(pi));
            for (int k = 0; k < pr.count; ++k) mass[size_t(g.vertices[size_t(pr.start + k)])] += density * area / float(pr.count);
            for (int k = 1; k + 1 < pr.count; ++k) {
                int t[3] = {g.vertices[size_t(pr.start)], g.vertices[size_t(pr.start + k)], g.vertices[size_t(pr.start + k + 1)]};
                for (int e = 0; e < 3; ++e) {
                    int a = t[e], b = t[(e + 1) % 3], o = t[(e + 2) % 3];
                    edgeOpp[{std::min(a, b), std::max(a, b)}].push_back(o);
                }
            }
        } else {
            for (int k = 0; k + 1 < pr.count; ++k) {
                int a = g.vertices[size_t(pr.start + k)], b = g.vertices[size_t(pr.start + k + 1)];
                float l = length(g.P[size_t(a)] - g.P[size_t(b)]) * density;
                mass[size_t(a)] += l * 0.5f; mass[size_t(b)] += l * 0.5f;
            }
        }
    }
    const float stretchK = std::max(1e-6f, c.f("stretch"));
    const float bendK = std::max(1e-9f, c.f("bend"));
    for (auto& e : edges) su.dist.push_back({e.first, e.second, length(g.P[size_t(e.first)] - g.P[size_t(e.second)]), 1.f / stretchK});
    su.numStretch = su.dist.size();
    // cloth bend: across every interior edge
    std::set<std::pair<int, int>> bendPairs;
    for (auto& [e, opp] : edgeOpp)
        if (opp.size() == 2 && opp[0] != opp[1]) {
            int a = std::min(opp[0], opp[1]), b = std::max(opp[0], opp[1]);
            if (bendPairs.insert({a, b}).second && !edges.count({a, b}))
                su.dist.push_back({a, b, length(g.P[size_t(a)] - g.P[size_t(b)]), 1.f / bendK});
        }
    // rope / hair bend: skip one point along open curves
    for (auto& pr : g.prims) {
        if (pr.closed && pr.count >= 3) continue;
        for (int k = 0; k + 2 < pr.count; ++k) {
            int a = g.vertices[size_t(pr.start + k)], b = g.vertices[size_t(pr.start + k + 2)];
            su.dist.push_back({a, b, length(g.P[size_t(a)] - g.P[size_t(b)]), 1.f / bendK});
        }
    }
    // pins
    su.pinned.assign(n, 0);
    if (const Attrib* pin = g.findAttrib(AttribClass::Point, "pin"))
        for (size_t i = 0; i < n; ++i) if (pin->getF(i) > 0.5f) su.pinned[i] = 1;
    const std::string& pat = c.s("pinpts");
    if (!pat.empty()) {
        std::vector<char> mask;
        std::string err;
        if (!selectElements(g, AttribClass::Point, pat, c.frame(), c.fps(), mask, err)) { c.error("Pin Points: " + err); return false; }
        for (size_t i = 0; i < n; ++i) if (mask[i]) su.pinned[i] = 1;
    }
    su.invMass.assign(n, 0.f);
    for (size_t i = 0; i < n; ++i) {
        float m = mass[i] > 0 ? mass[i] : density * 0.01f; // loose points get a small mass
        su.invMass[i] = su.pinned[i] ? 0.f : 1.f / m;
    }
    // pressure
    if (c.b("pressureon") && isClosedMesh(g)) {
        for (auto& pr : g.prims) {
            if (!pr.closed || pr.count < 3) continue;
            for (int k = 1; k + 1 < pr.count; ++k) {
                su.volTris.push_back(g.vertices[size_t(pr.start)]);
                su.volTris.push_back(g.vertices[size_t(pr.start + k)]);
                su.volTris.push_back(g.vertices[size_t(pr.start + k + 1)]);
            }
        }
        su.restVolume = meshVolume(g.P, su.volTris);
        if (su.restVolume < 0) {
            for (size_t k = 0; k + 2 < su.volTris.size(); k += 3) std::swap(su.volTris[k + 1], su.volTris[k + 2]);
            su.restVolume = -su.restVolume;
        }
    } else if (c.b("pressureon")) {
        c.warning("Pressure needs a closed mesh; it is ignored for this input.");
    }
    su.neighbors.assign(n, {});
    for (auto& e : edges) { su.neighbors[size_t(e.first)].push_back(e.second); su.neighbors[size_t(e.second)].push_back(e.first); }
    for (auto& b : su.dist) { su.neighbors[size_t(b.a)].push_back(b.b); su.neighbors[size_t(b.b)].push_back(b.a); }
    for (auto& nb : su.neighbors) { std::sort(nb.begin(), nb.end()); nb.erase(std::unique(nb.begin(), nb.end()), nb.end()); }
    return true;
}

void cookVellum(CookContext& c) {
    VellumState& st = c.state<VellumState>();
    const uint64_t sig = c.setupSignature();
    const int start = c.i("startframe");
    const int frame = int(std::floor(c.frame() + 0.5f));
    Node& node = c.node();
    node.simCacheStart = 0; node.simCacheEnd = -1;
    if (st.sig != sig) {
        st = VellumState();
        st.sig = sig;
        c.setEvalFrame(float(start));
        Geometry g0;
        if (!c.inputAtFrame(0, float(start), g0)) { c.clearEvalFrame(); c.error("Input failed to cook."); return; }
        st.rest = g0;
        st.ok = buildSetup(c, g0, st.setup);
        c.clearEvalFrame();
    }
    if (!st.ok) { c.error("Setup failed."); return; }
    auto output = [&](const VellumFrame* f) {
        Geometry g = st.rest;
        Attrib& v = g.addAttrib(AttribClass::Point, "v", 3);
        if (f) {
            g.P = f->P;
            for (size_t i = 0; i < f->V.size(); ++i) v.setV(i, f->V[i]);
        }
        if (g.findAttrib(AttribClass::Point, "N") || !g.prims.empty()) {
            bool polys = false;
            for (auto& pr : g.prims) if (pr.closed && pr.count >= 3) { polys = true; break; }
            if (polys) g.computePointNormals();
        }
        c.out = std::move(g);
    };
    if (frame <= start) { output(nullptr); return; }
    if (frame - start > 100000) { c.error("Frame is too far from the start frame."); return; }

    int have = start;
    { auto it = st.frames.upper_bound(frame); if (it != st.frames.begin()) { --it; have = it->first; } }
    VellumFrame cur;
    if (have > start && st.frames.count(have)) cur = st.frames[have];
    else { have = start; cur.P = st.rest.P; cur.V.assign(cur.P.size(), c.v("initv")); }

    const bool animIn = c.inputIsAnimated(0);
    const bool animCol = c.hasInput(1) && c.inputIsAnimated(1);
    Collider col;
    if (c.hasInput(1)) {
        Geometry cg;
        if (c.inputAtFrame(1, float(have), cg)) col.build(cg);
    }
    for (int k = have + 1; k <= frame; ++k) {
        c.setEvalFrame(float(k));
        SimParams sp;
        const int subs = std::clamp(c.i("substeps"), 1, 200);
        sp.dt = 1.f / (c.fps() * float(subs));
        sp.iterations = std::clamp(c.i("iterations"), 1, 100);
        sp.gravity = c.v("gravity");
        sp.wind = c.v("wind");
        sp.drag = std::max(0.f, c.f("drag"));
        sp.damping = std::max(0.f, c.f("damping"));
        sp.pressure = c.b("pressureon") ? std::max(0.f, c.f("pressure")) : 0.f;
        sp.pressureCompliance = 1.f / std::max(1e-6f, c.f("pressurestiff"));
        sp.ground = c.b("ground");
        sp.groundY = c.f("groundy");
        sp.friction = c.f("friction");
        sp.thickness = std::max(0.f, c.f("thickness"));
        sp.sphere = c.b("sphere");
        sp.sphereC = c.v("spherec");
        sp.sphereR = c.f("spherer");
        sp.self = c.b("selfcollide");
        // pinned points follow the (possibly animated) input, interpolated across the frame
        std::vector<Vec3> pinA, pinB, pinT;
        bool pins = animIn;
        if (pins) {
            Geometry ga, gb;
            pins = c.inputAtFrame(0, float(k - 1), ga) && c.inputAtFrame(0, float(k), gb) && ga.numPoints() == cur.P.size() && gb.numPoints() == cur.P.size();
            if (pins) { pinA = ga.P; pinB = gb.P; pinT = pinA; }
        }
        if (animCol) {
            Geometry cg;
            if (c.inputAtFrame(1, float(k), cg)) col.build(cg);
        }
        for (int s = 0; s < subs; ++s) {
            if (pins) {
                float u = float(s + 1) / float(subs);
                for (size_t i = 0; i < pinT.size(); ++i) pinT[i] = lerp(pinA[i], pinB[i], u);
            }
            substep(cur.P, cur.V, st.setup, sp, col, pins ? &pinT : nullptr);
        }
        st.frames[k] = cur;
    }
    c.clearEvalFrame();
    // bound memory
    size_t per = cur.P.size() * 2;
    while (st.frames.size() > 1 && st.frames.size() * per > 60u * 1000u * 1000u && st.frames.begin()->first != frame) st.frames.erase(st.frames.begin());
    output(&st.frames[frame]);
    int lo = frame, hi = frame;
    while (st.frames.count(lo - 1)) --lo;
    while (st.frames.count(hi + 1)) ++hi;
    node.simCacheStart = lo;
    node.simCacheEnd = hi;
}

} // namespace

void registerVellumNodes(std::vector<NodeType>& types) {
    NodeType t; t.name = "vellumsolver"; t.label = "Vellum Solver"; t.category = "Vellum";
    t.description = "Cloth (polygons), ropes and hair (curves) and pressurized soft bodies (closed meshes), with pins and collisions.";
    t.minInputs = 1; t.maxInputs = 2;
    t.inputLabels = {"Cloth / curves (point pin attribute)", "Collider (each part as its convex hull; may be animated)"};
    t.timeDependent = true;
    t.parms = {
        I("startframe", "Start Frame", 1, 1, 240),
        I("substeps", "Substeps per Frame", 20, 1, 100),
        I("iterations", "Constraint Iterations", 4, 1, 50),
        F("density", "Density (kg/m^2 or kg/m)", 0.2f, 0.001f, 5, "Mass per area of cloth, or per length of curves"),
        F("stretch", "Stretch Stiffness", 1e4f, 1, 1e6f, "Higher is less stretchy"),
        F("bend", "Bend Stiffness", 0.05f, 0, 100, "Higher is stiffer, like paper; low is silky"),
        S("pinpts", "Pin Points", "", "Group pattern, e.g. 0 20 or @P.y > 1.9"),
        V("gravity", "Gravity", Vec3(0, -9.81f, 0), -20, 20),
        V("wind", "Wind", Vec3(0), -10, 10),
        F("drag", "Air Drag", 0.2f, 0, 5),
        F("damping", "Velocity Damping", 0.1f, 0, 5),
        V("initv", "Initial Velocity", Vec3(0), -10, 10),
        T("pressureon", "Pressure (closed meshes)", false),
        F("pressure", "Pressure (volume ratio)", 1, 0, 3, "Target volume as a multiple of the rest volume"),
        F("pressurestiff", "Pressure Stiffness", 1e3f, 1, 1e6f),
        F("thickness", "Collision Thickness", 0.01f, 0, 0.2f),
        F("friction", "Friction", 0.3f, 0, 1),
        T("ground", "Ground Plane", true),
        F("groundy", "Ground Height", 0, -5, 5),
        T("sphere", "Sphere Collider", false),
        V("spherec", "Sphere Center", Vec3(0, 1, 0), -5, 5),
        F("spherer", "Sphere Radius", 0.5f, 0, 5),
        T("selfcollide", "Self Collision (points)", true),
    };
    t.cook = cookVellum;
    types.push_back(t);
}

} // namespace hd

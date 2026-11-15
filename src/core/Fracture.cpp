// Convex hulls, polygon triangulation, mass properties and Voronoi fracturing.
#include "Fracture.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <set>
#include <thread>
#include <tuple>
#include <unordered_map>

namespace hd {

namespace {

struct DVec { double x = 0, y = 0, z = 0; };
inline DVec dv(const Vec3& v) { return {v.x, v.y, v.z}; }
inline DVec operator-(const DVec& a, const DVec& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline DVec operator+(const DVec& a, const DVec& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline DVec operator*(const DVec& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double ddot(const DVec& a, const DVec& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline DVec dcross(const DVec& a, const DVec& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double dlen(const DVec& a) { return std::sqrt(ddot(a, a)); }

Vec3 anyPerpendicular(const Vec3& n) {
    Vec3 a = std::fabs(n.x) < 0.6f ? Vec3(1, 0, 0) : (std::fabs(n.y) < 0.6f ? Vec3(0, 1, 0) : Vec3(0, 0, 1));
    return normalize(cross(n, a));
}

Vec3 newellNormal(const std::vector<Vec3>& P, const std::vector<int>& loop) {
    Vec3 n(0);
    for (size_t k = 0; k < loop.size(); ++k) {
        const Vec3& a = P[size_t(loop[k])];
        const Vec3& b = P[size_t(loop[(k + 1) % loop.size()])];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    return normalize(n);
}

} // namespace

// ------------------------------------------------------------------ convex hull

float ConvexHull::distance(const Vec3& p, int* plane) const {
    float best = -1e30f;
    int bi = -1;
    for (size_t i = 0; i < planes.size(); ++i) {
        float d = dot(planes[i].n, p) - planes[i].d;
        if (d > best) { best = d; bi = int(i); }
    }
    if (plane) *plane = bi;
    return best;
}

namespace {

bool hullOnce(const std::vector<Vec3>& in, ConvexHull& out, double scale, bool& degenerate, Vec3& flatNormal, int& flatKind) {
    degenerate = false;
    const size_t n = in.size();
    const double eps = 1e-9 * scale;
    std::vector<DVec> p(n);
    for (size_t i = 0; i < n; ++i) p[i] = dv(in[i]);
    // initial tetrahedron
    size_t i0 = 0, i1 = 0;
    for (size_t i = 1; i < n; ++i) {
        if (p[i].x < p[i0].x) i0 = i;
        if (p[i].x > p[i1].x) i1 = i;
    }
    if (i0 == i1 || dlen(p[i1] - p[i0]) < 1e-7 * scale) {
        double best = -1;
        for (size_t i = 0; i < n; ++i) { double d = dlen(p[i] - p[i0]); if (d > best) { best = d; i1 = i; } }
    }
    DVec ab = p[i1] - p[i0];
    if (dlen(ab) < 1e-7 * scale) { degenerate = true; flatKind = 0; return false; }
    size_t i2 = i0;
    double best = -1;
    for (size_t i = 0; i < n; ++i) {
        double d = dlen(dcross(ab, p[i] - p[i0]));
        if (d > best) { best = d; i2 = i; }
    }
    if (best < 1e-7 * scale * dlen(ab)) {
        degenerate = true; flatKind = 1;
        DVec a = ab * (1.0 / dlen(ab));
        flatNormal = Vec3(float(a.x), float(a.y), float(a.z));
        return false;
    }
    DVec nrm = dcross(ab, p[i2] - p[i0]);
    nrm = nrm * (1.0 / dlen(nrm));
    size_t i3 = i0;
    best = -1;
    for (size_t i = 0; i < n; ++i) {
        double d = std::fabs(ddot(nrm, p[i] - p[i0]));
        if (d > best) { best = d; i3 = i; }
    }
    if (best < 1e-6 * scale) {
        degenerate = true; flatKind = 2;
        flatNormal = Vec3(float(nrm.x), float(nrm.y), float(nrm.z));
        return false;
    }

    struct Face { int a, b, c; DVec n; double d; bool alive; };
    std::vector<Face> faces;
    DVec interior = (p[i0] + p[i1] + p[i2] + p[i3]) * 0.25;
    auto makeFace = [&](int a, int b, int c) {
        Face f{a, b, c, {}, 0, true};
        DVec fn = dcross(p[size_t(b)] - p[size_t(a)], p[size_t(c)] - p[size_t(a)]);
        double l = dlen(fn);
        if (l > 0) fn = fn * (1.0 / l);
        f.n = fn;
        f.d = ddot(fn, p[size_t(a)]);
        return f;
    };
    int t[4] = {int(i0), int(i1), int(i2), int(i3)};
    int tf[4][3] = {{0, 1, 2}, {0, 3, 1}, {1, 3, 2}, {0, 2, 3}};
    for (auto& f3 : tf) {
        Face f = makeFace(t[f3[0]], t[f3[1]], t[f3[2]]);
        if (ddot(f.n, interior) - f.d > 0) f = makeFace(t[f3[0]], t[f3[2]], t[f3[1]]);
        faces.push_back(f);
    }
    // insert the remaining points, farthest-first ordering is not required for correctness
    std::vector<char> used(n, 0);
    for (int k : t) used[size_t(k)] = 1;
    std::set<std::pair<int, int>> vis;
    for (size_t i = 0; i < n; ++i) {
        if (used[i]) continue;
        vis.clear();
        std::vector<size_t> visFaces;
        for (size_t fi = 0; fi < faces.size(); ++fi) {
            Face& f = faces[fi];
            if (!f.alive) continue;
            if (ddot(f.n, p[i]) - f.d > eps) {
                visFaces.push_back(fi);
                vis.insert({f.a, f.b}); vis.insert({f.b, f.c}); vis.insert({f.c, f.a});
            }
        }
        if (visFaces.empty()) continue;
        for (size_t fi : visFaces) faces[fi].alive = false;
        for (auto& e : vis) {
            if (vis.count({e.second, e.first})) continue; // interior edge of the visible region
            Face f = makeFace(e.first, e.second, int(i));
            faces.push_back(f);
        }
        // compact occasionally to keep scans short
        if (faces.size() > 64 && faces.size() > 4 * (n + 4)) {
            std::vector<Face> keep;
            for (auto& f : faces) if (f.alive) keep.push_back(f);
            faces.swap(keep);
        }
    }

    out = ConvexHull();
    std::unordered_map<int, int> remap;
    for (auto& f : faces) {
        if (!f.alive) continue;
        for (int v : {f.a, f.b, f.c}) {
            auto it = remap.find(v);
            if (it == remap.end()) { remap[v] = int(out.pts.size()); out.pts.push_back(in[size_t(v)]); }
        }
        out.tris.push_back(remap[f.a]); out.tris.push_back(remap[f.b]); out.tris.push_back(remap[f.c]);
    }
    // planes (merged when coplanar) and edge midpoints
    for (size_t k = 0; k + 2 < out.tris.size(); k += 3) {
        const Vec3& a = out.pts[size_t(out.tris[k])];
        const Vec3& b = out.pts[size_t(out.tris[k + 1])];
        const Vec3& c = out.pts[size_t(out.tris[k + 2])];
        Vec3 fn = normalize(cross(b - a, c - a));
        if (dot(fn, fn) < 0.5f) continue;
        float d = dot(fn, a);
        bool dup = false;
        for (auto& pl : out.planes)
            if (dot(pl.n, fn) > 0.99999f && std::fabs(pl.d - d) < 1e-5f * float(scale)) { dup = true; break; }
        if (!dup) out.planes.push_back({fn, d});
    }
    std::set<std::pair<int, int>> edges;
    for (size_t k = 0; k + 2 < out.tris.size(); k += 3)
        for (int e = 0; e < 3; ++e) {
            int a = out.tris[k + size_t(e)], b = out.tris[k + size_t((e + 1) % 3)];
            if (edges.insert({std::min(a, b), std::max(a, b)}).second)
                out.edgeMids.push_back((out.pts[size_t(a)] + out.pts[size_t(b)]) * 0.5f);
        }
    return out.valid();
}

} // namespace

bool buildConvexHull(const std::vector<Vec3>& points, ConvexHull& out, float minThickness) {
    out = ConvexHull();
    if (points.empty()) return false;
    Vec3 mn = points[0], mx = points[0];
    for (auto& p : points) { mn = vmin(mn, p); mx = vmax(mx, p); }
    double scale = std::max(double(length(mx - mn)), double(minThickness));
    // de-duplicate (quantized) and limit the count by striding
    std::vector<Vec3> pts;
    {
        double q = 1e-6 * scale;
        std::set<std::tuple<long long, long long, long long>> seen;
        for (auto& p : points) {
            auto key = std::make_tuple((long long)std::llround((p.x - mn.x) / q), (long long)std::llround((p.y - mn.y) / q),
                                       (long long)std::llround((p.z - mn.z) / q));
            if (seen.insert(key).second) pts.push_back(p);
        }
        const size_t cap = 3000;
        if (pts.size() > cap) {
            // keep extreme points along many directions so the hull stays tight
            std::vector<char> keep(pts.size(), 0);
            for (int d = 0; d < 256; ++d) {
                float z = hashRandom(uint32_t(d) * 3u + 1u) * 2 - 1, a = hashRandom(uint32_t(d) * 3u + 2u) * 2 * kPi;
                float r = std::sqrt(std::max(0.f, 1 - z * z));
                Vec3 dir(r * std::cos(a), z, r * std::sin(a));
                size_t bi = 0; float bd = -1e30f;
                for (size_t i = 0; i < pts.size(); ++i) { float v = dot(pts[i], dir); if (v > bd) { bd = v; bi = i; } }
                keep[bi] = 1;
            }
            size_t stride = pts.size() / cap + 1;
            std::vector<Vec3> reduced;
            for (size_t i = 0; i < pts.size(); ++i) if (keep[i] || i % stride == 0) reduced.push_back(pts[i]);
            pts.swap(reduced);
        }
    }
    bool degenerate = false;
    Vec3 fn;
    int kind = 0;
    if (pts.size() >= 4 && hullOnce(pts, out, scale, degenerate, fn, kind)) return true;
    if (pts.size() >= 4 && !degenerate) return out.valid();
    // Thicken flat, collinear or single-point input into a thin slab / prism / cube.
    float t = std::max(minThickness, float(scale) * 1e-3f) * 0.5f;
    std::vector<Vec3> thick;
    std::vector<Vec3> offs;
    if (pts.size() < 3 || kind == 0) {
        for (int i = 0; i < 8; ++i) offs.push_back(Vec3(i & 1 ? t : -t, i & 2 ? t : -t, i & 4 ? t : -t));
    } else if (kind == 1) {
        Vec3 u = anyPerpendicular(fn), v = cross(fn, u);
        offs = {u * t, -u * t, v * t, -v * t};
    } else {
        offs = {fn * t, fn * -t};
    }
    for (auto& p : pts) for (auto& o : offs) thick.push_back(p + o);
    double s2 = scale + 4 * double(t);
    return hullOnce(thick, out, s2, degenerate, fn, kind);
}

// ------------------------------------------------------------------ mass properties

MassProps computeMassProps(const std::vector<Vec3>& pts, const std::vector<int>& tris) {
    MassProps mp;
    if (pts.empty()) return mp;
    // shift to the vertex centroid for precision
    DVec ref{0, 0, 0};
    for (auto& p : pts) ref = ref + dv(p);
    ref = ref * (1.0 / double(pts.size()));
    double V = 0;
    DVec c{0, 0, 0};
    double C[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (size_t k = 0; k + 2 < tris.size(); k += 3) {
        DVec a = dv(pts[size_t(tris[k])]) - ref, b = dv(pts[size_t(tris[k + 1])]) - ref, d = dv(pts[size_t(tris[k + 2])]) - ref;
        double det = ddot(a, dcross(b, d));
        V += det / 6.0;
        c = c + (a + b + d) * (det / 24.0);
        // covariance of tetra (0,a,b,d): det * A * Ccan * A^T, Ccan = [[2,1,1],[1,2,1],[1,1,2]]/120
        double A[3][3] = {{a.x, b.x, d.x}, {a.y, b.y, d.y}, {a.z, b.z, d.z}};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double s = 0;
                for (int k1 = 0; k1 < 3; ++k1)
                    for (int k2 = 0; k2 < 3; ++k2) s += A[i][k1] * (k1 == k2 ? 2.0 : 1.0) * A[j][k2];
                C[i][j] += det * s / 120.0;
            }
    }
    mp.volume = V;
    if (std::fabs(V) < 1e-30) { mp.com = Vec3(float(ref.x), float(ref.y), float(ref.z)); return mp; }
    c = c * (1.0 / V);
    double cc[3] = {c.x, c.y, c.z};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) C[i][j] -= V * cc[i] * cc[j];
    double tr = C[0][0] + C[1][1] + C[2][2];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) mp.inertia[i][j] = (i == j ? tr : 0.0) - C[i][j];
    DVec w = c + ref;
    mp.com = Vec3(float(w.x), float(w.y), float(w.z));
    return mp;
}

// ------------------------------------------------------------------ triangulation

namespace {

struct P2 { double x, y; };
inline double cross2(const P2& o, const P2& a, const P2& b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }

bool segmentsCross(const P2& a, const P2& b, const P2& c, const P2& d) {
    double d1 = cross2(c, d, a), d2 = cross2(c, d, b), d3 = cross2(a, b, c), d4 = cross2(a, b, d);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}


bool pointInLoop(const std::vector<P2>& q, const std::vector<int>& loop, const P2& p) {
    bool in = false;
    for (size_t i = 0, j = loop.size() - 1; i < loop.size(); j = i++) {
        const P2& a = q[size_t(loop[i])];
        const P2& b = q[size_t(loop[j])];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
    }
    return in;
}

} // namespace

bool triangulatePolygon(const std::vector<Vec3>& pts, const std::vector<std::vector<int>>& loops, const Vec3& n,
                        std::vector<int>& trisOut) {
    if (loops.empty() || loops[0].size() < 3) return true;
    Vec3 u = anyPerpendicular(n), v = cross(n, u);
    std::vector<P2> q(pts.size());
    for (auto& l : loops)
        for (int i : l) q[size_t(i)] = {double(dot(pts[size_t(i)], u)), double(dot(pts[size_t(i)], v))};

    // merge holes into the outer loop with bridge edges
    std::vector<int> poly = loops[0];
    std::vector<std::vector<int>> holes(loops.begin() + 1, loops.end());
    std::sort(holes.begin(), holes.end(), [&](const std::vector<int>& a, const std::vector<int>& b) {
        double ma = -1e300, mb = -1e300;
        for (int i : a) ma = std::max(ma, q[size_t(i)].x);
        for (int i : b) mb = std::max(mb, q[size_t(i)].x);
        return ma > mb;
    });
    for (size_t h = 0; h < holes.size(); ++h) {
        const std::vector<int>& hole = holes[h];
        if (hole.size() < 3) continue;
        size_t hm = 0;
        for (size_t k = 1; k < hole.size(); ++k) if (q[size_t(hole[k])].x > q[size_t(hole[hm])].x) hm = k;
        const P2& hp = q[size_t(hole[hm])];
        size_t bestK = SIZE_MAX;
        double bestD = 1e300;
        for (size_t k = 0; k < poly.size(); ++k) {
            const P2& op = q[size_t(poly[k])];
            double dd = (op.x - hp.x) * (op.x - hp.x) + (op.y - hp.y) * (op.y - hp.y);
            if (dd >= bestD) continue;
            bool blocked = false;
            auto testLoop = [&](const std::vector<int>& L) {
                for (size_t e = 0; e < L.size() && !blocked; ++e) {
                    int a = L[e], b = L[(e + 1) % L.size()];
                    if (a == poly[k] || b == poly[k] || a == hole[hm] || b == hole[hm]) continue;
                    if (segmentsCross(hp, op, q[size_t(a)], q[size_t(b)])) blocked = true;
                }
            };
            testLoop(poly);
            for (size_t h2 = h; h2 < holes.size() && !blocked; ++h2) testLoop(holes[h2]);
            if (!blocked) { bestD = dd; bestK = k; }
        }
        if (bestK == SIZE_MAX) continue; // could not bridge: drop the hole
        std::vector<int> merged(poly.begin(), poly.begin() + long(bestK) + 1);
        for (size_t k = 0; k <= hole.size(); ++k) merged.push_back(hole[(hm + k) % hole.size()]);
        merged.push_back(poly[bestK]);
        merged.insert(merged.end(), poly.begin() + long(bestK) + 1, poly.end());
        poly.swap(merged);
    }

    // ear clipping
    bool clean = true;
    std::vector<int> idx = poly;
    double scale = 0;
    for (int i : idx) scale = std::max(scale, std::fabs(q[size_t(i)].x) + std::fabs(q[size_t(i)].y));
    const double eps = 1e-12 * std::max(scale * scale, 1e-12);
    auto samePos = [&](int a, int b) { return a == b || (q[size_t(a)].x == q[size_t(b)].x && q[size_t(a)].y == q[size_t(b)].y); };
    while (idx.size() > 3) {
        size_t m = idx.size();
        bool found = false;
        for (size_t k = 0; k < m; ++k) {
            int a = idx[(k + m - 1) % m], b = idx[k], c = idx[(k + 1) % m];
            const P2 &pa = q[size_t(a)], &pb = q[size_t(b)], &pc = q[size_t(c)];
            double cr = cross2(pa, pb, pc);
            if (cr < -eps) continue; // reflex
            bool inside = false;
            if (cr > eps) {
                for (size_t j = 0; j < m && !inside; ++j) {
                    int o = idx[j];
                    if (samePos(o, a) || samePos(o, b) || samePos(o, c)) continue;
                    const P2& po = q[size_t(o)];
                    if (cross2(pa, pb, po) >= 0 && cross2(pb, pc, po) >= 0 && cross2(pc, pa, po) >= 0) inside = true;
                }
            }
            if (inside) continue;
            trisOut.push_back(a); trisOut.push_back(b); trisOut.push_back(c);
            idx.erase(idx.begin() + long(k));
            found = true;
            break;
        }
        if (!found) {
            clean = false;
            for (size_t k = 1; k + 1 < idx.size(); ++k) { trisOut.push_back(idx[0]); trisOut.push_back(idx[k]); trisOut.push_back(idx[k + 1]); }
            idx.clear();
            break;
        }
    }
    if (idx.size() == 3) { trisOut.push_back(idx[0]); trisOut.push_back(idx[1]); trisOut.push_back(idx[2]); }
    return clean;
}

// ------------------------------------------------------------------ inside test

bool pointInsideMesh(const Geometry& g, const Vec3& p) {
    // A slightly skewed direction avoids hitting edges exactly on axis-aligned meshes.
    const Vec3 dir = normalize(Vec3(0.5773f, 0.6123f, 0.5401f));
    int hits = 0;
    for (size_t pi = 0; pi < g.numPrims(); ++pi) {
        const Prim& pr = g.prims[pi];
        if (!pr.closed || pr.count < 3) continue;
        const Vec3& a = g.P[size_t(g.vertices[size_t(pr.start)])];
        for (int k = 1; k + 1 < pr.count; ++k) {
            const Vec3& b = g.P[size_t(g.vertices[size_t(pr.start + k)])];
            const Vec3& c = g.P[size_t(g.vertices[size_t(pr.start + k + 1)])];
            Vec3 e1 = b - a, e2 = c - a, h = cross(dir, e2);
            float det = dot(e1, h);
            if (std::fabs(det) < 1e-12f) continue;
            float inv = 1.f / det;
            Vec3 s = p - a;
            float uu = dot(s, h) * inv;
            if (uu < 0 || uu > 1) continue;
            Vec3 qq = cross(s, e1);
            float vv = dot(dir, qq) * inv;
            if (vv < 0 || uu + vv > 1) continue;
            if (dot(e2, qq) * inv > 0) ++hits;
        }
    }
    return hits % 2 == 1;
}

// ------------------------------------------------------------------ Voronoi fracture

namespace {

struct CellMesh {
    std::vector<Vec3> P;
    std::vector<float> attr;              // W floats per point (interpolated point attributes)
    std::vector<std::vector<int>> polys;
    std::vector<int> src;                 // source prim, or -1 for a cut face
    int W = 0;
};

// Clip to the half-space dot(n, x) <= d and cap the opening. Returns false if nothing is left.
bool clipCell(CellMesh& m, const Vec3& nf, double d, double eps, int& capFallbacks) {
    const DVec n = dv(nf);
    const size_t np = m.P.size();
    std::vector<double> s(np);
    bool anyOut = false, anyIn = false;
    std::vector<char> usedPt(np, 0);
    for (auto& poly : m.polys) for (int i : poly) usedPt[size_t(i)] = 1;
    for (size_t i = 0; i < np; ++i) {
        double v = ddot(n, dv(m.P[i])) - d;
        if (std::fabs(v) < eps) v = 0;
        s[i] = v;
        if (!usedPt[i]) continue;
        if (v > 0) anyOut = true; else anyIn = true;
    }
    if (!anyOut) return true;
    if (!anyIn) { m.polys.clear(); m.src.clear(); return false; }

    std::unordered_map<uint64_t, int> isect;
    auto cut = [&](int a, int b) -> int {
        // a is inside (s <= 0), b outside (s > 0)
        if (s[size_t(a)] == 0) return a;
        int lo = std::min(a, b), hi = std::max(a, b);
        uint64_t key = (uint64_t(uint32_t(lo)) << 32) | uint32_t(hi);
        auto it = isect.find(key);
        if (it != isect.end()) return it->second;
        double t = s[size_t(lo)] / (s[size_t(lo)] - s[size_t(hi)]);
        int id = int(m.P.size());
        Vec3 plo = m.P[size_t(lo)], phi = m.P[size_t(hi)];
        m.P.push_back(Vec3(float(plo.x + (phi.x - plo.x) * t), float(plo.y + (phi.y - plo.y) * t), float(plo.z + (phi.z - plo.z) * t)));
        for (int k = 0; k < m.W; ++k) {
            float va = m.attr[size_t(lo) * size_t(m.W) + size_t(k)], vb = m.attr[size_t(hi) * size_t(m.W) + size_t(k)];
            m.attr.push_back(float(va + (vb - va) * t));
        }
        s.push_back(0);
        isect[key] = id;
        return id;
    };

    std::vector<std::vector<int>> outPolys;
    std::vector<int> outSrc;
    std::multimap<int, int> segs; // cap edges: from -> to
    for (size_t pi = 0; pi < m.polys.size(); ++pi) {
        const std::vector<int>& poly = m.polys[pi];
        bool allIn = true, allOut = true;
        for (int i : poly) { if (s[size_t(i)] > 0) allIn = false; else allOut = false; }
        if (allIn) { outPolys.push_back(poly); outSrc.push_back(m.src[pi]); continue; }
        std::vector<int> o;
        std::vector<int> exits, entries; // in output order
        std::vector<std::pair<int, int>> events; // (kind 0 = exit, 1 = entry, point)
        const size_t nv = poly.size();
        for (size_t k = 0; k < nv; ++k) {
            int a = poly[k], b = poly[(k + 1) % nv];
            bool ia = s[size_t(a)] <= 0, ib = s[size_t(b)] <= 0;
            if (ia) o.push_back(a);
            if (ia && !ib) { int x = cut(a, b); o.push_back(x); events.push_back({0, x}); }
            else if (!ia && ib) { int x = cut(b, a); o.push_back(x); events.push_back({1, x}); }
        }
        if (allOut) continue;
        // pair each exit with the following entry: the polygon has edge exit->entry, so the cap has entry->exit
        for (size_t e = 0; e < events.size(); ++e) {
            if (events[e].first != 0) continue;
            for (size_t f = 1; f < events.size(); ++f) {
                auto& ev = events[(e + f) % events.size()];
                if (ev.first == 1) {
                    if (ev.second != events[e].second) segs.insert({ev.second, events[e].second});
                    break;
                }
            }
        }
        // remove consecutive duplicates
        std::vector<int> clean;
        for (int i : o) if (clean.empty() || clean.back() != i) clean.push_back(i);
        while (clean.size() > 1 && clean.front() == clean.back()) clean.pop_back();
        if (clean.size() >= 3) { outPolys.push_back(clean); outSrc.push_back(m.src[pi]); }
    }

    // chain cap segments into loops
    std::vector<std::vector<int>> loops;
    while (!segs.empty()) {
        auto it = segs.begin();
        int start = it->first;
        std::vector<int> loop{start};
        int cur = it->second;
        segs.erase(it);
        bool closed = false;
        for (size_t guard = 0; guard < 100000; ++guard) {
            if (cur == start) { closed = true; break; }
            loop.push_back(cur);
            auto nx = segs.find(cur);
            if (nx == segs.end()) break;
            int nxt = nx->second;
            segs.erase(nx);
            cur = nxt;
        }
        if (closed && loop.size() >= 3) loops.push_back(loop);
    }
    if (!loops.empty()) {
        // classify outer loops and holes by orientation about the plane normal
        Vec3 u = anyPerpendicular(nf), v = cross(nf, u);
        auto area = [&](const std::vector<int>& L) {
            double a = 0;
            for (size_t k = 0; k < L.size(); ++k) {
                const Vec3& p = m.P[size_t(L[k])];
                const Vec3& q = m.P[size_t(L[(k + 1) % L.size()])];
                a += double(dot(p, u)) * dot(q, v) - double(dot(q, u)) * dot(p, v);
            }
            return a * 0.5;
        };
        std::vector<std::vector<int>> outers, holes;
        for (auto& L : loops) (area(L) >= 0 ? outers : holes).push_back(L);
        std::vector<std::vector<std::vector<int>>> groups(outers.size());
        for (size_t o = 0; o < outers.size(); ++o) groups[o].push_back(outers[o]);
        for (auto& H : holes) {
            // the smallest outer loop that contains the hole
            Vec3 hp = m.P[size_t(H[0])];
            double best = 1e300;
            int bo = -1;
            for (size_t o = 0; o < outers.size(); ++o) {
                std::vector<P2> q(m.P.size());
                for (int i : outers[o]) q[size_t(i)] = {double(dot(m.P[size_t(i)], u)), double(dot(m.P[size_t(i)], v))};
                if (pointInLoop(q, outers[o], {double(dot(hp, u)), double(dot(hp, v))})) {
                    double a = area(outers[o]);
                    if (a < best) { best = a; bo = int(o); }
                }
            }
            if (bo >= 0) groups[size_t(bo)].push_back(H);
        }
        for (auto& gr : groups) {
            bool convex = gr.size() == 1;
            if (convex) {
                const auto& L = gr[0];
                for (size_t k = 0; k < L.size() && convex; ++k) {
                    Vec3 a = m.P[size_t(L[k])], b = m.P[size_t(L[(k + 1) % L.size()])], c = m.P[size_t(L[(k + 2) % L.size()])];
                    if (dot(cross(b - a, c - b), nf) < -1e-7f) convex = false;
                }
            }
            if (convex) { outPolys.push_back(gr[0]); outSrc.push_back(-1); continue; }
            std::vector<int> tris;
            if (!triangulatePolygon(m.P, gr, nf, tris)) ++capFallbacks;
            for (size_t k = 0; k + 2 < tris.size(); k += 3) { outPolys.push_back({tris[k], tris[k + 1], tris[k + 2]}); outSrc.push_back(-1); }
        }
    }
    m.polys.swap(outPolys);
    m.src.swap(outSrc);
    return !m.polys.empty();
}

bool polygonConvex(const std::vector<Vec3>& P, const std::vector<int>& poly, const Vec3& n) {
    for (size_t k = 0; k < poly.size(); ++k) {
        Vec3 a = P[size_t(poly[k])], b = P[size_t(poly[(k + 1) % poly.size()])], c = P[size_t(poly[(k + 2) % poly.size()])];
        if (dot(cross(b - a, c - b), n) < -1e-9f) return false;
    }
    return true;
}

} // namespace

Geometry voronoiFracture(const Geometry& in, const std::vector<Vec3>& sites, const FractureOptions& opt, FractureStats* stats) {
    FractureStats st;
    // base cell mesh: closed polygons of the input, non-convex ones triangulated
    CellMesh base;
    base.P = in.P;
    std::vector<std::pair<std::string, int>> rows; // point attributes carried along (name, tuple)
    for (auto& [name, a] : in.pointAttribs) if (name != "N" && name != "piece") { rows.push_back({name, a.tuple}); base.W += a.tuple; }
    base.attr.resize(in.numPoints() * size_t(base.W));
    {
        int off = 0;
        for (auto& [name, tup] : rows) {
            const Attrib& a = in.pointAttribs.at(name);
            for (size_t i = 0; i < in.numPoints(); ++i)
                for (int k = 0; k < tup; ++k) base.attr[i * size_t(base.W) + size_t(off + k)] = a.data[i * size_t(tup) + size_t(k)];
            off += tup;
        }
    }
    for (size_t pi = 0; pi < in.numPrims(); ++pi) {
        const Prim& pr = in.prims[pi];
        if (!pr.closed || pr.count < 3) continue;
        std::vector<int> poly(in.vertices.begin() + pr.start, in.vertices.begin() + pr.start + pr.count);
        Vec3 n = newellNormal(in.P, poly);
        if (poly.size() > 3 && !polygonConvex(in.P, poly, n)) {
            std::vector<int> tris;
            triangulatePolygon(in.P, {poly}, n, tris);
            for (size_t k = 0; k + 2 < tris.size(); k += 3) { base.polys.push_back({tris[k], tris[k + 1], tris[k + 2]}); base.src.push_back(int(pi)); }
        } else {
            base.polys.push_back(poly);
            base.src.push_back(int(pi));
        }
    }
    Vec3 mn, mx;
    in.bounds(mn, mx);
    const double scale = std::max(1e-6, double(length(mx - mn)));
    const double eps = 1e-7 * scale;

    const size_t nc = sites.size();
    std::vector<CellMesh> cells(nc);
    std::vector<int> capFallbacks(nc, 0);
    std::atomic<size_t> next{0};
    auto work = [&]() {
        std::vector<std::pair<double, size_t>> order;
        for (;;) {
            size_t i = next.fetch_add(1);
            if (i >= nc) break;
            const Vec3 si = sites[i];
            order.clear();
            for (size_t j = 0; j < nc; ++j)
                if (j != i) {
                    double d2 = double(dot(sites[j] - si, sites[j] - si));
                    if (d2 > 1e-20) order.push_back({d2, j});
                }
            std::sort(order.begin(), order.end());
            CellMesh m = base;
            double radius = 0;
            for (auto& p : m.P) radius = std::max(radius, double(length(p - si)));
            bool alive = !m.polys.empty();
            for (auto& [d2, j] : order) {
                if (!alive) break;
                double half = std::sqrt(d2) * 0.5;
                if (half > radius + eps) break; // no farther site can cut this cell
                Vec3 n = normalize(sites[j] - si);
                Vec3 mid = (si + sites[j]) * 0.5f;
                alive = clipCell(m, n, double(dot(n, mid)), eps, capFallbacks[i]);
                if (alive) {
                    radius = 0;
                    std::vector<char> used(m.P.size(), 0);
                    for (auto& poly : m.polys) for (int v : poly) if (!used[size_t(v)]) { used[size_t(v)] = 1; radius = std::max(radius, double(length(m.P[size_t(v)] - si))); }
                }
            }
            if (!alive) m.polys.clear();
            cells[i] = std::move(m);
        }
    };
    int nt = opt.threads > 0 ? opt.threads : int(std::max(1u, std::thread::hardware_concurrency()));
    nt = std::min<int>(nt, int(std::max<size_t>(1, nc)));
    std::vector<std::thread> pool;
    for (int t = 1; t < nt; ++t) pool.emplace_back(work);
    work();
    for (auto& th : pool) th.join();

    // assemble the output
    Geometry out;
    std::vector<std::string> primNames;
    for (auto& [name, a] : in.primAttribs) if (name != "piece" && name != "inside" && name != "N") primNames.push_back(name);
    const Attrib* inPtCd = in.findAttrib(AttribClass::Point, "Cd");
    const Attrib* inPrCd = in.findAttrib(AttribClass::Prim, "Cd");
    for (auto& [name, tup] : rows) {
        if (opt.colorInside && name == "Cd") continue; // becomes a primitive color
        Attrib& a = out.addAttrib(AttribClass::Point, name, tup);
        const Attrib& ia = in.pointAttribs.at(name);
        for (int k = 0; k < 3; ++k) a.defaults[k] = ia.defaults[k];
    }
    for (auto& name : primNames) {
        Attrib& a = out.addAttrib(AttribClass::Prim, name, in.primAttribs.at(name).tuple);
        for (int k = 0; k < 3; ++k) a.defaults[k] = in.primAttribs.at(name).defaults[k];
    }
    out.addAttrib(AttribClass::Point, "piece", 1);
    out.addAttrib(AttribClass::Prim, "piece", 1);
    out.addAttrib(AttribClass::Prim, "inside", 1);
    if (opt.colorInside) out.addAttrib(AttribClass::Prim, "Cd", 3, Vec3(0.72f, 0.74f, 0.78f));
    int pieceId = 0;
    for (size_t ci = 0; ci < nc; ++ci) {
        CellMesh& m = cells[ci];
        st.capFallbacks += capFallbacks[ci];
        if (m.polys.empty()) { ++st.emptyCells; continue; }
        std::vector<int> remap(m.P.size(), -1);
        size_t base0 = out.numPoints();
        for (auto& poly : m.polys)
            for (int v : poly)
                if (remap[size_t(v)] < 0) { remap[size_t(v)] = int(out.P.size()); out.P.push_back(m.P[size_t(v)]); }
        out.syncAttribSizes();
        // point attributes
        int off = 0;
        for (auto& [name, tup] : rows) {
            Attrib* a = (opt.colorInside && name == "Cd") ? nullptr : out.findAttrib(AttribClass::Point, name);
            if (a)
                for (size_t v = 0; v < m.P.size(); ++v)
                    if (remap[v] >= 0)
                        for (int k = 0; k < tup; ++k) a->data[size_t(remap[v]) * size_t(tup) + size_t(k)] = m.attr[v * size_t(m.W) + size_t(off + k)];
            off += tup;
        }
        Attrib& ppiece = *out.findAttrib(AttribClass::Point, "piece");
        for (size_t i = base0; i < out.numPoints(); ++i) ppiece.setF(i, float(pieceId));
        int cdOff = -1;
        { int o = 0; for (auto& [name, tup] : rows) { if (name == "Cd" && tup == 3) cdOff = o; o += tup; } }
        for (size_t pi = 0; pi < m.polys.size(); ++pi) {
            std::vector<int> ids;
            for (int v : m.polys[pi]) ids.push_back(remap[size_t(v)]);
            int np = out.addPrim(ids, true);
            out.syncAttribSizes();
            int s = m.src[pi];
            for (auto& name : primNames) {
                Attrib& a = *out.findAttrib(AttribClass::Prim, name);
                if (s >= 0) { const Attrib& ia = in.primAttribs.at(name); for (int k = 0; k < a.tuple; ++k) a.data[size_t(np) * size_t(a.tuple) + size_t(k)] = ia.data[size_t(s) * size_t(ia.tuple) + size_t(k)]; }
            }
            out.findAttrib(AttribClass::Prim, "piece")->setF(size_t(np), float(pieceId));
            out.findAttrib(AttribClass::Prim, "inside")->setF(size_t(np), s < 0 ? 1.f : 0.f);
            if (opt.colorInside) {
                Vec3 col(0.72f, 0.74f, 0.78f);
                if (s < 0) col = opt.insideColor;
                else if (inPtCd && inPtCd->tuple == 3 && cdOff >= 0) {
                    Vec3 sum(0);
                    for (int v : m.polys[pi]) sum += Vec3(m.attr[size_t(v) * size_t(m.W) + size_t(cdOff)], m.attr[size_t(v) * size_t(m.W) + size_t(cdOff + 1)], m.attr[size_t(v) * size_t(m.W) + size_t(cdOff + 2)]);
                    col = sum / float(m.polys[pi].size());
                } else if (inPrCd && inPrCd->tuple == 3) col = inPrCd->getV(size_t(s));
                out.findAttrib(AttribClass::Prim, "Cd")->setV(size_t(np), col);
            }
        }
        ++pieceId;
    }
    out.syncAttribSizes();
    st.pieces = pieceId;
    if (stats) *stats = st;
    return out;
}

} // namespace hd

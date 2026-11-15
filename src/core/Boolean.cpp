// Modelling nodes: Boolean (union, intersect, subtract, shatter of closed meshes) and Clip.
//
// Boolean
//   1. Both inputs are triangulated (fans of their closed polygons); each triangle remembers its
//      source polygon and source points so attributes can be carried over.
//   2. Every triangle of A is intersected with the nearby triangles of B (bounding volume
//      hierarchies) and vice versa. A crossing pair contributes the segment where they cross to both
//      triangles; a coplanar overlapping pair contributes each triangle's edges to the other, so the
//      overlap gets its own pieces.
//   3. Each triangle is cut into convex pieces: a piece that a segment passes through is split by
//      the line of that segment. Pieces never straddle the intersection curve.
//   4. Each piece is classified against the other mesh by casting rays from just above and just
//      below its centroid (majority of three ray directions, so a ray grazing an edge doesn't
//      decide alone). Equal answers give inside / outside; different answers mean the piece lies on
//      a face of the other mesh, facing the same way or the opposite way.
//   5. The operation keeps pieces by that classification (subtract keeps B's inside pieces, turned
//      over). Uncut polygons are kept whole, so quads stay quads.
//   6. Points are welded and points lying on the edges of neighbouring polygons are inserted into
//      those edges, so the result has no T-junction cracks.
#include "Node.h"
#include "Ops.h"
#include "Volume.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <set>
#include <unordered_map>

namespace hd {
namespace {

ParmDef T(const char* n, const char* l, bool d, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Toggle; p.def[0] = d ? 1.f : 0.f; p.help = help; return p;
}
ParmDef M(const char* n, const char* l, std::vector<std::string> items, int d = 0) {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Menu; p.menu = std::move(items); p.def[0] = float(d); return p;
}
ParmDef V(const char* n, const char* l, Vec3 d, float mn = -1, float mx = 1, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::Vec3; p.def[0] = d.x; p.def[1] = d.y; p.def[2] = d.z; p.min = mn; p.max = mx; p.help = help; return p;
}

struct Tri {
    Vec3 p[3];
    Vec3 n;      // unit normal
    int prim;    // source polygon
    int pt[3];   // source points
};

struct Mesh {
    const Geometry* g = nullptr;
    std::vector<Tri> tris;
    std::vector<int> firstTri, triCount; // per source prim
    // BVH over the triangles
    struct Node { Vec3 mn, mx; int left = -1, right = -1, first = 0, count = 0; };
    std::vector<Node> nodes;
    std::vector<int> order;

    void build(const Geometry& geo) {
        g = &geo;
        firstTri.assign(geo.numPrims(), 0);
        triCount.assign(geo.numPrims(), 0);
        for (size_t pi = 0; pi < geo.numPrims(); ++pi) {
            const Prim& pr = geo.prims[pi];
            firstTri[pi] = int(tris.size());
            if (!pr.closed || pr.count < 3) continue;
            const int a = geo.primVertex(int(pi), 0);
            for (int k = 1; k + 1 < pr.count; ++k) {
                Tri t;
                t.pt[0] = a; t.pt[1] = geo.primVertex(int(pi), k); t.pt[2] = geo.primVertex(int(pi), k + 1);
                for (int j = 0; j < 3; ++j) t.p[j] = geo.P[size_t(t.pt[j])];
                Vec3 c = cross(t.p[1] - t.p[0], t.p[2] - t.p[0]);
                if (dot(c, c) < 1e-30f) continue;
                t.n = normalize(c);
                t.prim = int(pi);
                tris.push_back(t);
                ++triCount[pi];
            }
        }
        order.resize(tris.size());
        for (size_t i = 0; i < tris.size(); ++i) order[i] = int(i);
        nodes.clear();
        if (tris.empty()) return;
        nodes.reserve(tris.size() * 2);
        nodes.push_back(Node());
        struct Job { int node, lo, hi; };
        std::vector<Job> stack{{0, 0, int(tris.size())}};
        while (!stack.empty()) {
            Job j = stack.back(); stack.pop_back();
            Vec3 mn(1e30f), mx(-1e30f), cmn(1e30f), cmx(-1e30f);
            for (int i = j.lo; i < j.hi; ++i) {
                const Tri& t = tris[size_t(order[size_t(i)])];
                Vec3 c(0);
                for (auto& p : t.p) { mn = vmin(mn, p); mx = vmax(mx, p); c += p; }
                c = c / 3.f;
                cmn = vmin(cmn, c); cmx = vmax(cmx, c);
            }
            nodes[size_t(j.node)].mn = mn; nodes[size_t(j.node)].mx = mx;
            Vec3 ext = cmx - cmn;
            int axis = ext.x > ext.y ? (ext.x > ext.z ? 0 : 2) : (ext.y > ext.z ? 1 : 2);
            if (j.hi - j.lo <= 4 || ext[axis] <= 0) { nodes[size_t(j.node)].first = j.lo; nodes[size_t(j.node)].count = j.hi - j.lo; continue; }
            int mid = (j.lo + j.hi) / 2;
            std::nth_element(order.begin() + j.lo, order.begin() + mid, order.begin() + j.hi, [&](int a, int b) {
                const Tri& ta = tris[size_t(a)]; const Tri& tb = tris[size_t(b)];
                return ta.p[0][axis] + ta.p[1][axis] + ta.p[2][axis] < tb.p[0][axis] + tb.p[1][axis] + tb.p[2][axis];
            });
            int l = int(nodes.size());
            nodes.push_back(Node()); nodes.push_back(Node());
            nodes[size_t(j.node)].left = l; nodes[size_t(j.node)].right = l + 1;
            stack.push_back({l, j.lo, mid});
            stack.push_back({l + 1, mid, j.hi});
        }
    }

    template <class Fn> void overlapping(const Vec3& mn, const Vec3& mx, Fn&& fn) const {
        if (nodes.empty()) return;
        int stack[128], sp = 0;
        stack[sp++] = 0;
        while (sp) {
            const Node& nd = nodes[size_t(stack[--sp])];
            if (nd.mn.x > mx.x || nd.mx.x < mn.x || nd.mn.y > mx.y || nd.mx.y < mn.y || nd.mn.z > mx.z || nd.mx.z < mn.z) continue;
            if (nd.left < 0) { for (int i = nd.first; i < nd.first + nd.count; ++i) fn(order[size_t(i)]); continue; }
            if (sp < 126) { stack[sp++] = nd.left; stack[sp++] = nd.right; }
        }
    }

    // Number of crossings of the ray o + t d (t > 0).
    int crossings(const Vec3& o, const Vec3& d) const {
        if (nodes.empty()) return 0;
        int count = 0;
        Vec3 inv(1.f / d.x, 1.f / d.y, 1.f / d.z);
        int stack[128], sp = 0;
        stack[sp++] = 0;
        while (sp) {
            const Node& nd = nodes[size_t(stack[--sp])];
            float t0 = 0, t1 = 1e30f;
            bool hit = true;
            for (int a = 0; a < 3 && hit; ++a) {
                float ta = (nd.mn[a] - o[a]) * inv[a], tb = (nd.mx[a] - o[a]) * inv[a];
                if (ta > tb) std::swap(ta, tb);
                t0 = std::max(t0, ta); t1 = std::min(t1, tb);
                hit = t0 <= t1;
            }
            if (!hit) continue;
            if (nd.left < 0) {
                for (int i = nd.first; i < nd.first + nd.count; ++i) {
                    const Tri& t = tris[size_t(order[size_t(i)])];
                    Vec3 e1 = t.p[1] - t.p[0], e2 = t.p[2] - t.p[0], pv = cross(d, e2);
                    float det = dot(e1, pv);
                    if (std::fabs(det) < 1e-30f) continue;
                    float id = 1.f / det;
                    Vec3 tv = o - t.p[0];
                    float u = dot(tv, pv) * id;
                    if (u < 0 || u > 1) continue;
                    Vec3 qv = cross(tv, e1);
                    float v = dot(d, qv) * id;
                    if (v < 0 || u + v > 1) continue;
                    if (dot(e2, qv) * id > 0) ++count;
                }
                continue;
            }
            if (sp < 126) { stack[sp++] = nd.left; stack[sp++] = nd.right; }
        }
        return count;
    }
    bool inside(const Vec3& p) const {
        static const Vec3 dirs[3] = {normalize(Vec3(0.5773f, 0.6211f, 0.5301f)), normalize(Vec3(-0.7071f, 0.1307f, 0.6948f)),
                                     normalize(Vec3(0.2113f, -0.9419f, 0.2611f))};
        int votes = 0;
        for (const Vec3& d : dirs) votes += crossings(p, d) & 1;
        return votes >= 2;
    }
};

struct Seg { Vec3 a, b; };

// Where triangles s and t cross: a segment (true) or nothing. Coplanar pairs set coplanar instead.
bool triTri(const Tri& s, const Tri& t, float eps, Seg& out, bool& coplanar) {
    coplanar = false;
    float ds[3], dt[3];
    for (int i = 0; i < 3; ++i) {
        ds[i] = dot(t.n, s.p[i] - t.p[0]); if (std::fabs(ds[i]) < eps) ds[i] = 0;
        dt[i] = dot(s.n, t.p[i] - s.p[0]); if (std::fabs(dt[i]) < eps) dt[i] = 0;
    }
    if ((ds[0] > 0 && ds[1] > 0 && ds[2] > 0) || (ds[0] < 0 && ds[1] < 0 && ds[2] < 0)) return false;
    if ((dt[0] > 0 && dt[1] > 0 && dt[2] > 0) || (dt[0] < 0 && dt[1] < 0 && dt[2] < 0)) return false;
    if (ds[0] == 0 && ds[1] == 0 && ds[2] == 0) { coplanar = true; return false; }
    if (dt[0] == 0 && dt[1] == 0 && dt[2] == 0) { coplanar = true; return false; }
    // points of each triangle on the other's plane
    // Edge crossings are computed from the edge's lower point id, so the two triangles sharing an
    // edge get bit-identical points and the cut lines up across it.
    auto onPlane = [](const Tri& tr, const float* d, Vec3* pts) {
        int n = 0;
        for (int i = 0; i < 3 && n < 2; ++i) {
            int j = (i + 1) % 3;
            if (d[i] == 0) pts[n++] = tr.p[i];
            else if ((d[i] > 0 && d[j] < 0) || (d[i] < 0 && d[j] > 0)) {
                int a = i, b = j;
                if (tr.pt[a] > tr.pt[b]) std::swap(a, b);
                pts[n++] = tr.p[a] + (tr.p[b] - tr.p[a]) * (d[a] / (d[a] - d[b]));
            }
        }
        return n;
    };
    Vec3 ps[2], pt[2];
    int ns = onPlane(s, ds, ps), nt = onPlane(t, dt, pt);
    if (ns == 0 || nt == 0) return false;
    if (ns == 1) ps[1] = ps[0];
    if (nt == 1) pt[1] = pt[0];
    Vec3 L = cross(s.n, t.n);
    if (dot(L, L) < 1e-20f) return false;
    L = normalize(L);
    float s0 = dot(L, ps[0]), s1 = dot(L, ps[1]), t0 = dot(L, pt[0]), t1 = dot(L, pt[1]);
    if (s0 > s1) { std::swap(s0, s1); std::swap(ps[0], ps[1]); }
    if (t0 > t1) { std::swap(t0, t1); std::swap(pt[0], pt[1]); }
    float lo = std::max(s0, t0), hi = std::min(s1, t1);
    if (hi - lo <= eps) return false;
    out.a = s0 >= t0 ? ps[0] : pt[0];
    out.b = s1 <= t1 ? ps[1] : pt[1];
    return true;
}

using Poly = std::vector<Vec3>;

// Constrained triangulation of one triangle with the cut segments as edges: the segments' end
// points (and their crossings) are inserted, then every segment is made an edge by flipping the
// edges that cross it. The resulting triangles never straddle the cut. 2D maths in double
// precision in the triangle's plane; vertices keep their exact 3D positions.
struct Cdt {
    struct V { double x, y; Vec3 p; };
    std::vector<V> v;
    std::vector<std::array<int, 3>> t; // counter-clockwise
    Vec3 o, ax, ay;
    double tol = 1e-9;

    static double orient(const V& a, const V& b, const V& c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }
    V make(const Vec3& p) const { Vec3 d = p - o; return {double(dot(d, ax)), double(dot(d, ay)), p}; }

    void init(const Tri& tr, double scale) {
        o = tr.p[0];
        ax = normalize(tr.p[1] - tr.p[0]);
        ay = normalize(cross(tr.n, ax));
        tol = scale * 1e-7;
        v = {make(tr.p[0]), make(tr.p[1]), make(tr.p[2])};
        v[0].p = tr.p[0]; v[1].p = tr.p[1]; v[2].p = tr.p[2];
        t = {{0, 1, 2}};
        if (orient(v[0], v[1], v[2]) < 0) std::swap(t[0][1], t[0][2]);
    }
    // area-relative orientation tolerance for points a, b
    double otol(const V& a, const V& b) const { return tol * std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y)); }

    int insert(const Vec3& p3) {
        V p = make(p3);
        for (size_t i = 0; i < v.size(); ++i)
            if (std::fabs(v[i].x - p.x) <= tol && std::fabs(v[i].y - p.y) <= tol) return int(i);
        // the triangle containing it: the one whose smallest signed edge distance is largest. A point
        // on an edge (or a hair outside the triangle, from rounding) splits that edge, so no
        // triangle is ever turned inside out.
        int best = -1, onEdge = -1;
        double bestMin = -1e300;
        for (size_t i = 0; i < t.size(); ++i) {
            double mn = 1e300;
            int mnEdge = -1, zeros = 0, zeroEdge = -1;
            for (int k = 0; k < 3; ++k) {
                const V& a = v[size_t(t[i][size_t(k)])];
                const V& b = v[size_t(t[i][size_t((k + 1) % 3)])];
                double l = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
                double dist = l > 0 ? orient(a, b, p) / l : 0;
                if (dist < mn) { mn = dist; mnEdge = k; }
                if (std::fabs(dist) <= tol) { ++zeros; zeroEdge = k; }
            }
            if (mn > bestMin) {
                bestMin = mn; best = int(i);
                onEdge = zeros == 1 ? zeroEdge : (mn < -tol ? mnEdge : -1);
            }
        }
        if (best < 0) return -1;
        const int id = int(v.size());
        v.push_back(p);
        auto tri = t[size_t(best)];
        if (onEdge < 0) {
            t[size_t(best)] = {tri[0], tri[1], id};
            t.push_back({tri[1], tri[2], id});
            t.push_back({tri[2], tri[0], id});
            return id;
        }
        // on edge (a, b): split this triangle and the one across the edge
        int a = tri[size_t(onEdge)], b = tri[size_t((onEdge + 1) % 3)], c = tri[size_t((onEdge + 2) % 3)];
        t[size_t(best)] = {a, id, c};
        t.push_back({id, b, c});
        for (size_t i = 0; i < t.size(); ++i) {
            for (int k = 0; k < 3; ++k)
                if (t[i][size_t(k)] == b && t[i][size_t((k + 1) % 3)] == a) {
                    int d = t[i][size_t((k + 2) % 3)];
                    t[i] = {b, id, d};
                    t.push_back({id, a, d});
                    return id;
                }
        }
        return id;
    }
    static double incircle(const V& a, const V& b, const V& c, const V& d) {
        double adx = a.x - d.x, ady = a.y - d.y, bdx = b.x - d.x, bdy = b.y - d.y, cdx = c.x - d.x, cdy = c.y - d.y;
        double ad = adx * adx + ady * ady, bd = bdx * bdx + bdy * bdy, cd = cdx * cdx + cdy * cdy;
        return adx * (bdy * cd - bd * cdy) - ady * (bdx * cd - bd * cdx) + ad * (bdx * cdy - bdy * cdx);
    }
    // Lawson flips toward a Delaunay triangulation: no skinny triangles, so the segment
    // recovery below has well-shaped quads to flip.
    void legalize() {
        for (int pass = 0; pass < 64; ++pass) {
            bool any = false;
            for (size_t i = 0; i < t.size(); ++i)
                for (int k = 0; k < 3; ++k) {
                    int u = t[i][size_t(k)], w = t[i][size_t((k + 1) % 3)], c = t[i][size_t((k + 2) % 3)];
                    for (size_t j = 0; j < t.size(); ++j) {
                        int kk = -1;
                        for (int q = 0; q < 3; ++q) if (t[j][size_t(q)] == w && t[j][size_t((q + 1) % 3)] == u) kk = q;
                        if (kk < 0) continue;
                        int d = t[j][size_t((kk + 2) % 3)];
                        if (incircle(v[size_t(u)], v[size_t(w)], v[size_t(c)], v[size_t(d)]) > 1e-18) {
                            std::array<int, 3> t1{c, u, d}, t2{d, w, c};
                            if (orient(v[size_t(c)], v[size_t(u)], v[size_t(d)]) > 1e-18 && orient(v[size_t(d)], v[size_t(w)], v[size_t(c)]) > 1e-18) {
                                t[i] = t1; t[j] = t2; any = true;
                            }
                        }
                        break;
                    }
                    if (any) break;
                }
            if (!any) break;
        }
    }
    bool hasEdge(int a, int b) const {
        for (auto& tr : t)
            for (int k = 0; k < 3; ++k)
                if ((tr[size_t(k)] == a && tr[size_t((k + 1) % 3)] == b) || (tr[size_t(k)] == b && tr[size_t((k + 1) % 3)] == a)) return true;
        return false;
    }
    // Makes (a, b) an edge. Vertices lying on it split it.
    bool constrain(int a, int b, int depth = 0) {
        if (a == b || hasEdge(a, b)) return true;
        if (a < 0 || b < 0 || depth > 32) return false;
        const V& A = v[size_t(a)];
        const V& B = v[size_t(b)];
        const double len2 = (B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y);
        if (len2 <= tol * tol) return true;
        const double et = otol(A, B);
        for (size_t i = 0; i < v.size(); ++i) {
            if (int(i) == a || int(i) == b) continue;
            double o2 = orient(A, B, v[i]);
            if (std::fabs(o2) > et) continue;
            double s = ((v[i].x - A.x) * (B.x - A.x) + (v[i].y - A.y) * (B.y - A.y)) / len2;
            if (s > 1e-9 && s < 1 - 1e-9) { bool ok = constrain(a, int(i), depth + 1); return constrain(int(i), b, depth + 1) && ok; }
        }
        for (int iter = 0; iter < 400 && !hasEdge(a, b); ++iter) {
            bool flipped = false;
            for (size_t i = 0; i < t.size() && !flipped; ++i)
                for (int k = 0; k < 3 && !flipped; ++k) {
                    int u = t[i][size_t(k)], w = t[i][size_t((k + 1) % 3)];
                    if (u == a || u == b || w == a || w == b) continue;
                    const V& U = v[size_t(u)];
                    const V& W = v[size_t(w)];
                    double ou = orient(A, B, U), ow = orient(A, B, W);
                    if (!((ou > et && ow < -et) || (ou < -et && ow > et))) continue;
                    double ea = orient(U, W, A), eb = orient(U, W, B);
                    double ut = otol(U, W);
                    if (!((ea > ut && eb < -ut) || (ea < -ut && eb > ut))) continue;
                    // the triangle on the other side of (u, w)
                    int c = t[i][size_t((k + 2) % 3)];
                    for (size_t j = 0; j < t.size(); ++j) {
                        int kk = -1;
                        for (int q = 0; q < 3; ++q) if (t[j][size_t(q)] == w && t[j][size_t((q + 1) % 3)] == u) kk = q;
                        if (kk < 0) continue;
                        int d = t[j][size_t((kk + 2) % 3)];
                        // flip only if the quad (u, d, w, c) is strictly convex
                        const V& C = v[size_t(c)];
                        const V& D = v[size_t(d)];
                        if (orient(C, D, U) * orient(C, D, W) >= 0) break;
                        if (orient(C, U, D) <= 0 && orient(C, W, D) <= 0) {}
                        std::array<int, 3> t1{c, u, d}, t2{d, w, c};
                        if (orient(v[size_t(t1[0])], v[size_t(t1[1])], v[size_t(t1[2])]) <= 0 ||
                            orient(v[size_t(t2[0])], v[size_t(t2[1])], v[size_t(t2[2])]) <= 0) break;
                        t[i] = t1; t[j] = t2;
                        flipped = true;
                        break;
                    }
                }
            if (!flipped) break;
        }
        return hasEdge(a, b);
    }
};

// 2D crossing of segments p1-p2 and q1-q2 (proper, interior to both); returns the 3D point on p.
bool crossSegs(const Cdt& cd, const Seg& p, const Seg& q, Vec3& x) {
    Cdt::V a = cd.make(p.a), b = cd.make(p.b), c = cd.make(q.a), d = cd.make(q.b);
    double den = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
    if (std::fabs(den) < 1e-30) return false;
    double s = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / den;
    double u = ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x)) / den;
    if (s <= 1e-7 || s >= 1 - 1e-7 || u <= 1e-7 || u >= 1 - 1e-7) return false;
    x = p.a + (p.b - p.a) * float(s);
    return true;
}

// The cut triangles of tr.
std::vector<Poly> cutTriangle(const Tri& tr, const std::vector<Seg>& segs, double scale) {
    Cdt cd;
    cd.init(tr, scale);
    // segments clipped to the triangle (coplanar neighbours' edges can reach outside it)
    std::vector<Seg> in;
    for (const Seg& sg : segs) {
        Cdt::V a = cd.make(sg.a), b = cd.make(sg.b);
        double t0 = 0, t1 = 1;
        const auto& T = cd.t[0];
        for (int k = 0; k < 3; ++k) {
            const Cdt::V& p = cd.v[size_t(T[size_t(k)])];
            const Cdt::V& q = cd.v[size_t(T[size_t((k + 1) % 3)])];
            double fa = Cdt::orient(p, q, a), fb = Cdt::orient(p, q, b), et = cd.otol(p, q);
            if (fa < -et && fb < -et) { t1 = -1; break; }
            if (fa < -et || fb < -et) {
                double tc = fa / (fa - fb);
                if (fa < 0) t0 = std::max(t0, tc); else t1 = std::min(t1, tc);
            }
        }
        if (t1 - t0 <= 1e-9) continue;
        Seg c{sg.a + (sg.b - sg.a) * float(t0), sg.a + (sg.b - sg.a) * float(t1)};
        if (t0 == 0) c.a = sg.a;
        if (t1 == 1) c.b = sg.b;
        in.push_back(c);
    }
    // split segments where they cross each other
    std::vector<std::vector<Vec3>> cuts(in.size());
    for (size_t i = 0; i < in.size(); ++i)
        for (size_t j = i + 1; j < in.size(); ++j) {
            Vec3 x;
            if (crossSegs(cd, in[i], in[j], x)) { cuts[i].push_back(x); cuts[j].push_back(x); }
        }
    std::vector<std::pair<int, int>> edges;
    for (size_t i = 0; i < in.size(); ++i) {
        std::vector<Vec3> chain{in[i].a};
        std::sort(cuts[i].begin(), cuts[i].end(), [&](const Vec3& p, const Vec3& q) { return length(p - in[i].a) < length(q - in[i].a); });
        for (auto& x : cuts[i]) chain.push_back(x);
        chain.push_back(in[i].b);
        std::vector<int> ids;
        for (auto& p : chain) ids.push_back(cd.insert(p));
        for (size_t k = 0; k + 1 < ids.size(); ++k) edges.push_back({ids[k], ids[k + 1]});
    }
    cd.legalize();
    for (auto& [a, b] : edges) cd.constrain(a, b);
    std::vector<Poly> out;
    out.reserve(cd.t.size());
    for (auto& tri : cd.t) out.push_back(Poly{cd.v[size_t(tri[0])].p, cd.v[size_t(tri[1])].p, cd.v[size_t(tri[2])].p});
    return out;
}

float polyArea(const Poly& p) {
    Vec3 s(0);
    for (size_t i = 1; i + 1 < p.size(); ++i) s += cross(p[i] - p[0], p[i + 1] - p[0]);
    return 0.5f * length(s);
}

enum Class { Outside, Inside, Same, Opposite };

// Barycentric coordinates of p in triangle t (clamped into it).
Vec3 bary(const Tri& t, const Vec3& p) {
    Vec3 v0 = t.p[1] - t.p[0], v1 = t.p[2] - t.p[0], v2 = p - t.p[0];
    float d00 = dot(v0, v0), d01 = dot(v0, v1), d11 = dot(v1, v1), d20 = dot(v2, v0), d21 = dot(v2, v1);
    float den = d00 * d11 - d01 * d01;
    if (std::fabs(den) < 1e-30f) return Vec3(1, 0, 0);
    float v = (d11 * d20 - d01 * d21) / den, w = (d00 * d21 - d01 * d20) / den;
    v = std::clamp(v, 0.f, 1.f); w = std::clamp(w, 0.f, 1.f);
    if (v + w > 1) { float s = v + w; v /= s; w /= s; }
    return Vec3(1 - v - w, v, w);
}

// Output builder: polygons from one source mesh, carrying its attributes.
struct Builder {
    const Geometry& src;
    Geometry out;
    explicit Builder(const Geometry& s) : src(s) {
        for (auto& [name, a] : src.pointAttribs) { Attrib na; na.tuple = a.tuple; std::copy(a.defaults, a.defaults + 3, na.defaults); out.pointAttribs[name] = na; }
        out.pointAttribs["__bcut"].tuple = 1; // 1 = made by cutting (not a corner of the input)
        for (auto& [name, a] : src.primAttribs) { Attrib na; na.tuple = a.tuple; std::copy(a.defaults, a.defaults + 3, na.defaults); out.primAttribs[name] = na; }
    }
    void primAttrs(int prim) { for (auto& [name, a] : src.primAttribs) out.primAttribs[name].appendFrom(a, size_t(prim)); }
    // a source polygon as it is (optionally reversed)
    void whole(int prim, bool flip) {
        const Prim& pr = src.prims[size_t(prim)];
        std::vector<int> ids;
        for (int k = 0; k < pr.count; ++k) {
            int s = src.primVertex(prim, flip ? pr.count - 1 - k : k);
            ids.push_back(int(out.P.size()));
            out.P.push_back(src.P[size_t(s)]);
            for (auto& [name, a] : src.pointAttribs) out.pointAttribs[name].appendFrom(a, size_t(s));
            out.pointAttribs["__bcut"].data.push_back(0);
        }
        out.vertices.insert(out.vertices.end(), ids.begin(), ids.end());
        Prim np; np.start = int(out.vertices.size()) - pr.count; np.count = pr.count; np.closed = true;
        out.prims.push_back(np);
        primAttrs(prim);
    }
    // a piece of source triangle t; point attributes interpolated
    void piece(const Tri& t, const Poly& p, bool flip) {
        const int n = int(p.size());
        for (int k = 0; k < n; ++k) {
            const Vec3& x = p[size_t(flip ? n - 1 - k : k)];
            Vec3 w = bary(t, x);
            out.vertices.push_back(int(out.P.size()));
            out.P.push_back(x);
            out.pointAttribs["__bcut"].data.push_back(std::max({w.x, w.y, w.z}) > 1 - 1e-6f ? 0.f : 1.f);
            for (auto& [name, a] : src.pointAttribs) {
                Attrib& o = out.pointAttribs[name];
                for (int c = 0; c < a.tuple; ++c) {
                    float v = 0;
                    for (int j = 0; j < 3; ++j) v += w[j] * a.data[size_t(t.pt[j]) * size_t(a.tuple) + size_t(c)];
                    o.data.push_back(v);
                }
            }
        }
        Prim np; np.start = int(out.vertices.size()) - n; np.count = n; np.closed = true;
        out.prims.push_back(np);
        primAttrs(t.prim);
    }
};

// Removes polygons thinner than tol (collinear slivers left by welding); their neighbours get the
// points back through fixTJunctions.
void dropSlivers(Geometry& g, float tol) {
    std::vector<int> keep;
    for (size_t pi = 0; pi < g.numPrims(); ++pi) {
        const Prim& pr = g.prims[pi];
        if (!pr.closed) { keep.push_back(int(pi)); continue; }
        float maxEdge = 0;
        for (int k = 0; k < pr.count; ++k) maxEdge = std::max(maxEdge, length(g.P[size_t(g.primVertex(int(pi), (k + 1) % pr.count))] - g.P[size_t(g.primVertex(int(pi), k))]));
        if (g.primArea(int(pi)) > tol * maxEdge * 0.5f) keep.push_back(int(pi));
    }
    if (keep.size() != g.numPrims()) g = g.extractPrims(keep);
}

// Inserts points that lie on polygon edges into those edges (removes T-junctions after welding).
bool fixTJunctionsPass(Geometry& g, float tol) {
    if (g.numPoints() == 0) return false;
    // only points made by cutting can sit in the middle of another polygon's edge
    const Attrib* cutA = g.findAttrib(AttribClass::Point, "__bcut");
    Vec3 mn, mx;
    g.bounds(mn, mx);
    float cell = std::max(length(mx - mn) / 128.f, tol * 4);
    auto key = [&](const Vec3& p) {
        return std::array<int, 3>{int(std::floor((p.x - mn.x) / cell)), int(std::floor((p.y - mn.y) / cell)), int(std::floor((p.z - mn.z) / cell))};
    };
    struct H { size_t operator()(const std::array<int, 3>& k) const { return size_t(k[0]) * 73856093u ^ size_t(k[1]) * 19349663u ^ size_t(k[2]) * 83492791u; } };
    // each candidate is listed in its cell and the 26 around it, so an edge looks up one cell per step
    std::unordered_map<std::array<int, 3>, std::vector<int>, H> grid;
    for (size_t i = 0; i < g.numPoints(); ++i) {
        if (cutA && cutA->getF(i) < 0.5f) continue;
        auto c = key(g.P[i]);
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz) grid[{c[0] + dx, c[1] + dy, c[2] + dz}].push_back(int(i));
    }
    if (grid.empty()) return false;
    std::vector<int> verts;
    std::vector<Prim> prims;
    verts.reserve(g.vertices.size());
    bool changed = false;
    for (size_t pi = 0; pi < g.numPrims(); ++pi) {
        const Prim& pr = g.prims[pi];
        std::vector<int> own;
        for (int k = 0; k < pr.count; ++k) own.push_back(g.primVertex(int(pi), k));
        Prim np = pr;
        np.start = int(verts.size());
        const int ne = pr.closed ? pr.count : pr.count - 1;
        for (int k = 0; k < pr.count; ++k) {
            int a = g.primVertex(int(pi), k);
            verts.push_back(a);
            if (k >= ne) continue;
            int b = g.primVertex(int(pi), (k + 1) % pr.count);
            Vec3 pa = g.P[size_t(a)], pb = g.P[size_t(b)], e = pb - pa;
            float len2 = dot(e, e);
            if (len2 <= tol * tol) continue;
            std::vector<std::pair<float, int>> on;
            const float len = std::sqrt(len2);
            const int steps = std::min(4096, int(len / cell) + 1);
            std::array<int, 3> last{INT32_MIN, 0, 0};
            for (int s = 0; s <= steps; ++s) {
                auto c = key(pa + e * (float(s) / float(steps)));
                if (c == last) continue;
                last = c;
                        {
                            auto it = grid.find(c);
                            if (it == grid.end()) continue;
                            for (int q : it->second) {
                                if (std::find(own.begin(), own.end(), q) != own.end()) continue;
                                Vec3 d = g.P[size_t(q)] - pa;
                                float t = dot(d, e) / len2;
                                if (t <= 1e-4f || t >= 1 - 1e-4f) continue;
                                if (length(d - e * t) > tol) continue;
                                on.push_back({t, q});
                            }
                        }
            }
            if (on.empty()) continue;
            std::sort(on.begin(), on.end());
            on.erase(std::unique(on.begin(), on.end(), [](auto& x, auto& y) { return x.second == y.second; }), on.end());
            for (auto& [t, q] : on) verts.push_back(q);
            changed = true;
        }
        np.count = int(verts.size()) - np.start;
        prims.push_back(np);
    }
    if (!changed) return false;
    g.vertices.swap(verts);
    g.prims.swap(prims);
    return true;
}
// Removes spikes (a b a) and repeated points that welding leaves in polygons; drops what collapses.
void removeSpikes(Geometry& g) {
    std::vector<int> keep;
    std::vector<int> verts;
    std::vector<Prim> prims;
    bool changed = false;
    for (size_t pi = 0; pi < g.numPrims(); ++pi) {
        const Prim& pr = g.prims[pi];
        std::vector<int> v;
        for (int k = 0; k < pr.count; ++k) v.push_back(g.primVertex(int(pi), k));
        if (pr.closed) {
            bool again = true;
            while (again && v.size() >= 3) {
                again = false;
                for (size_t k = 0; k < v.size() && v.size() >= 3; ++k) {
                    size_t n = v.size(), k1 = (k + 1) % n, k2 = (k + 2) % n;
                    if (v[k] == v[k1]) { v.erase(v.begin() + long(k1)); again = true; break; }
                    if (v[k] == v[k2]) { // a b a: drop b and one a
                        std::vector<int> w;
                        for (size_t q = 0; q < n; ++q) if (q != k1 && q != k2) w.push_back(v[q]);
                        v.swap(w); again = true; break;
                    }
                }
            }
        }
        if (int(v.size()) != pr.count) changed = true;
        if (pr.closed && v.size() < 3) continue;
        keep.push_back(int(pi));
        Prim np = pr; np.start = int(verts.size()); np.count = int(v.size());
        verts.insert(verts.end(), v.begin(), v.end());
        prims.push_back(np);
    }
    if (!changed) return;
    // prim attributes follow the kept prims
    for (auto& [name, a] : g.primAttribs) {
        Attrib na; na.tuple = a.tuple; std::copy(a.defaults, a.defaults + 3, na.defaults);
        for (int k : keep) na.appendFrom(a, size_t(k));
        a = std::move(na);
    }
    g.vertices.swap(verts);
    g.prims.swap(prims);
}

// Closes pinholes: loops of boundary edges (edges with no partner running the other way) of at most
// 8 edges and a perimeter below maxPerimeter get a polygon. Where the surfaces meet at a very
// shallow angle, intersection points from different triangle pairs can disagree by more than the
// weld distance and leave such holes.
void fillPinholes(Geometry& g, float maxPerimeter) {
    std::map<std::pair<int, int>, int> directed;
    for (size_t pi = 0; pi < g.numPrims(); ++pi) {
        const Prim& pr = g.prims[pi];
        if (!pr.closed) continue;
        for (int k = 0; k < pr.count; ++k) ++directed[{g.primVertex(int(pi), k), g.primVertex(int(pi), (k + 1) % pr.count)}];
    }
    std::multimap<int, int> next; // boundary edge a -> b (the hole runs b -> a)
    for (auto& [e, c] : directed)
        if (c == 1 && !directed.count({e.second, e.first})) next.insert({e.second, e.first});
    std::set<std::pair<int, int>> used;
    for (auto& [a0, b0] : next) {
        if (used.count({a0, b0})) continue;
        std::vector<int> loop{a0};
        int cur = b0;
        float per = length(g.P[size_t(b0)] - g.P[size_t(a0)]);
        bool closed = false;
        std::vector<std::pair<int, int>> path{{a0, b0}};
        for (int step = 0; step < 8; ++step) {
            if (cur == a0) { closed = true; break; }
            if (next.count(cur) != 1) break;
            int nx = next.find(cur)->second;
            loop.push_back(cur);
            path.push_back({cur, nx});
            per += length(g.P[size_t(nx)] - g.P[size_t(cur)]);
            cur = nx;
        }
        if (!closed || loop.size() < 3) continue;
        // small, or long but thin (a sliver between two nearly identical cut lines)
        Vec3 ar(0);
        float longest = 0;
        for (size_t k = 0; k < loop.size(); ++k) {
            const Vec3& p = g.P[size_t(loop[k])];
            const Vec3& q = g.P[size_t(loop[(k + 1) % loop.size()])];
            ar += cross(p - g.P[size_t(loop[0])], q - g.P[size_t(loop[0])]);
            longest = std::max(longest, length(q - p));
        }
        const float thickness = longest > 0 ? length(ar) / longest : 0.f;
        if (per > maxPerimeter && thickness > maxPerimeter * 0.25f) continue;
        for (auto& e : path) used.insert(e);
        g.addPrim(loop);
    }
}

// Points inserted into an edge make new edges that can have points of their own: repeat.
void fixTJunctions(Geometry& g, float tol) {
    removeSpikes(g);
    for (int pass = 0; pass < 4 && fixTJunctionsPass(g, tol); ++pass) {}
    removeSpikes(g);
}

// Edges used by other than exactly two polygons (open or non-manifold): 0 means closed.
size_t openEdges(const Geometry& g) {
    std::unordered_map<uint64_t, int> count;
    for (size_t pi = 0; pi < g.numPrims(); ++pi) {
        const Prim& pr = g.prims[pi];
        if (!pr.closed || pr.count < 3) continue;
        for (int k = 0; k < pr.count; ++k) {
            uint32_t a = uint32_t(g.primVertex(int(pi), k)), b = uint32_t(g.primVertex(int(pi), (k + 1) % pr.count));
            if (a > b) std::swap(a, b);
            ++count[(uint64_t(a) << 32) | b];
        }
    }
    size_t open = 0;
    for (auto& [e, c] : count) if (c != 2) ++open;
    return open;
}

} // namespace

// op: 0 union, 1 intersect, 2 subtract (A - B), 3 shatter (A cut by B: the part inside B and the part outside, both closed)
bool booleanMeshes(const Geometry& ain, const Geometry& bin, int op, Geometry& result, std::string& warn) {
    Geometry A = ain, B = bin;
    A.unpack(); B.unpack();
    // welding first makes "closed" meaningful for meshes whose polygons don't share points
    Vec3 amn, amx, bmn, bmx;
    A.bounds(amn, amx); B.bounds(bmn, bmx);
    const float scale = std::max({length(amx - amn), length(bmx - bmn), 1e-6f});
    const float eps = scale * 2e-6f;
    Mesh ma, mb;
    ma.build(A); mb.build(B);
    if (openEdges(fusePoints(A, eps * 10)) > 0) warn += "The first input is not closed: inside and outside are guessed.\n";
    if (openEdges(fusePoints(B, eps * 10)) > 0) warn += "The second input is not closed: inside and outside are guessed.\n";

    // 2. intersection segments per triangle
    std::vector<std::vector<Seg>> segA(ma.tris.size()), segB(mb.tris.size());
    std::vector<std::vector<std::pair<int, Seg>>> found(ma.tris.size()); // per A triangle: (B triangle, segment)
    std::vector<std::vector<int>> coplanarWith(ma.tris.size());
    parallelFor(int(ma.tris.size()), [&](int lo, int hi) {
        for (int i = lo; i < hi; ++i) {
            const Tri& s = ma.tris[size_t(i)];
            Vec3 mn = vmin(vmin(s.p[0], s.p[1]), s.p[2]) - Vec3(eps), mx = vmax(vmax(s.p[0], s.p[1]), s.p[2]) + Vec3(eps);
            mb.overlapping(mn, mx, [&](int j) {
                Seg sg;
                bool cop = false;
                if (triTri(s, mb.tris[size_t(j)], eps, sg, cop)) found[size_t(i)].push_back({j, sg});
                else if (cop) coplanarWith[size_t(i)].push_back(j);
            });
        }
    });
    for (size_t i = 0; i < found.size(); ++i) {
        for (auto& [j, sg] : found[i]) { segA[i].push_back(sg); segB[size_t(j)].push_back(sg); }
        for (int j : coplanarWith[i]) {
            const Tri& s = ma.tris[i]; const Tri& t = mb.tris[size_t(j)];
            for (int k = 0; k < 3; ++k) {
                segA[i].push_back({t.p[k], t.p[(k + 1) % 3]});
                segB[size_t(j)].push_back({s.p[k], s.p[(k + 1) % 3]});
            }
        }
    }

    // 3 + 4. pieces and their classification
    struct Piece { int tri; Poly poly; Class cls; };
    auto cutAndClassify = [&](const Mesh& m, const std::vector<std::vector<Seg>>& segs, const Mesh& other, std::vector<Piece>& pieces,
                              std::vector<Class>& wholeCls, const std::vector<char>& cop) {
        std::vector<std::vector<Piece>> per(m.tris.size());
        wholeCls.assign(m.tris.size(), Outside);
        const float off = scale * 1e-4f;
        auto classify = [&](const Vec3& c, const Vec3& n, bool mayBeCoplanar) {
            // only pieces of triangles lying in a face of the other mesh need the two-sided test: for
            // the rest an offset could step across the other surface where it meets this one at a shallow angle
            if (!mayBeCoplanar) return other.inside(c) ? Inside : Outside;
            bool up = other.inside(c + n * off), down = other.inside(c - n * off);
            if (up == down) return up ? Inside : Outside;
            return down ? Same : Opposite; // solid below and empty above: the other surface faces our way
        };
        // Uncut triangles joined by uncut triangles lie on the same side: classify one per connected
        // patch (the patch's largest triangle) instead of casting rays for each.
        const size_t nt = m.tris.size();
        std::vector<int> parent(nt);
        for (size_t i = 0; i < nt; ++i) parent[i] = int(i);
        std::function<int(int)> root = [&](int x) { while (parent[size_t(x)] != x) { parent[size_t(x)] = parent[size_t(parent[size_t(x)])]; x = parent[size_t(x)]; } return x; };
        {
            std::unordered_map<uint64_t, int> edgeOwner;
            for (size_t i = 0; i < nt; ++i) {
                if (!segs[i].empty() || cop[i]) continue;
                const Tri& t = m.tris[i];
                for (int k = 0; k < 3; ++k) {
                    uint32_t a = uint32_t(t.pt[k]), b = uint32_t(t.pt[(k + 1) % 3]);
                    if (a > b) std::swap(a, b);
                    uint64_t key = (uint64_t(a) << 32) | b;
                    auto it = edgeOwner.find(key);
                    if (it == edgeOwner.end()) edgeOwner[key] = int(i);
                    else { int ra = root(int(i)), rb = root(it->second); if (ra != rb) parent[size_t(ra)] = rb; }
                }
            }
        }
        std::vector<int> rep(nt, -1); // per root: its largest triangle
        std::vector<float> repArea(nt, -1);
        for (size_t i = 0; i < nt; ++i) {
            if (!segs[i].empty() || cop[i]) continue;
            int r = root(int(i));
            const Tri& t = m.tris[i];
            float a = length(cross(t.p[1] - t.p[0], t.p[2] - t.p[0]));
            if (a > repArea[size_t(r)]) { repArea[size_t(r)] = a; rep[size_t(r)] = int(i); }
        }
        std::vector<Class> rootCls(nt, Outside);
        parallelFor(int(nt), [&](int lo, int hi) {
            for (int r = lo; r < hi; ++r) {
                int i = rep[size_t(r)];
                if (i < 0) continue;
                const Tri& t = m.tris[size_t(i)];
                rootCls[size_t(r)] = classify((t.p[0] + t.p[1] + t.p[2]) / 3.f, t.n, false);
            }
        });
        parallelFor(int(m.tris.size()), [&](int lo, int hi) {
            for (int i = lo; i < hi; ++i) {
                const Tri& t = m.tris[size_t(i)];
                if (segs[size_t(i)].empty()) {
                    wholeCls[size_t(i)] = cop[size_t(i)] ? classify((t.p[0] + t.p[1] + t.p[2]) / 3.f, t.n, true) : rootCls[size_t(root(i))];
                    continue;
                }
                std::vector<Poly> pcs = cutTriangle(t, segs[size_t(i)], scale);
                for (Poly& p : pcs) {
                    if (polyArea(p) <= eps * eps) continue;
                    Vec3 c(0);
                    for (auto& x : p) c += x;
                    c = c / float(p.size());
                    per[size_t(i)].push_back({i, std::move(p), classify(c, t.n, cop[size_t(i)])});
                }
            }
        });
        (void)pieces;
        return per;
    };
    std::vector<Piece> piecesA, piecesB;
    std::vector<Class> wholeA, wholeB;
    // triangles with a coplanar partner (they and their pieces may lie in the other mesh's faces)
    std::vector<char> copA(ma.tris.size(), 0), copB(mb.tris.size(), 0);
    for (size_t i = 0; i < coplanarWith.size(); ++i)
        for (int j : coplanarWith[i]) { copA[i] = 1; copB[size_t(j)] = 1; }
    auto perA = cutAndClassify(ma, segA, mb, piecesA, wholeA, copA);
    auto perB = cutAndClassify(mb, segB, ma, piecesB, wholeB, copB);

    // 5. keep by the operation
    auto keepA = [&](Class c, int part) {
        switch (op) {
        case 0: return c == Outside || c == Same;
        case 1: return c == Inside || c == Same;
        case 2: return c == Outside || c == Opposite;
        default: return part == 0 ? (c == Outside || c == Opposite) : (c == Inside || c == Same);
        }
    };
    auto keepB = [&](Class c, int part) {
        switch (op) {
        case 0: return c == Outside;
        case 1: return c == Inside;
        case 2: return c == Inside;
        default: (void)part; return c == Inside;
        }
    };
    const bool flipB = op == 2;
    // (a triangle with segments but no surviving pieces, e.g. all slivers, has an empty piece list
    //  and must not be treated as uncut: segsX tells the two apart)
    auto build = [&](const Mesh& m, const std::vector<std::vector<Seg>>& segs, const std::vector<std::vector<Piece>>& per,
                     const std::vector<Class>& whole, const std::function<bool(Class)>& keep, bool flip, Builder& bd) {
        const Geometry& g = *m.g;
        for (size_t pi = 0; pi < g.numPrims(); ++pi) {
            const int t0 = m.firstTri[pi], nt = m.triCount[pi];
            if (!nt) continue;
            bool all = true;
            for (int t = t0; t < t0 + nt && all; ++t) all = segs[size_t(t)].empty() && keep(whole[size_t(t)]);
            if (all) { bd.whole(int(pi), flip); continue; }
            for (int t = t0; t < t0 + nt; ++t) {
                const Tri& tr = m.tris[size_t(t)];
                if (segs[size_t(t)].empty()) { if (keep(whole[size_t(t)])) bd.piece(tr, Poly{tr.p[0], tr.p[1], tr.p[2]}, flip); }
                else for (const Piece& p : per[size_t(t)]) if (keep(p.cls)) bd.piece(tr, p.poly, flip);
            }
        }
    };
    const int parts = op == 3 ? 2 : 1;
    result = Geometry();
    for (int part = 0; part < parts; ++part) {
        Builder ba(A), bb(B);
        build(ma, segA, perA, wholeA, [&](Class c) { return keepA(c, part); }, false, ba);
        // shatter: the outside part is closed by B's inside pieces turned over, the inside part by them as they are
        build(mb, segB, perB, wholeB, [&](Class c) { return keepB(c, part); }, op == 3 ? part == 0 : flipB, bb);
        Geometry g = std::move(ba.out);
        g.merge(bb.out);
        if (op == 3) {
            Attrib& pc = g.addAttrib(AttribClass::Prim, "piece", 1, Vec3(0));
            for (size_t i = 0; i < g.numPrims(); ++i) pc.setF(i, float(part));
        }
        if (part == 0) result = std::move(g);
        else result.merge(g);
    }
    // 6. weld (each shatter piece on its own, so the two stay separate) and close the T-junctions
    // Weld distance: intersection points found from different triangle pairs agree to about this
    // (worse where surfaces meet at a shallow angle), but stay well below the input's edge lengths.
    std::vector<float> edgeLen;
    for (const Mesh* m : {&ma, &mb})
        for (size_t i = 0; i < m->tris.size(); i += std::max<size_t>(1, m->tris.size() / 2000))
            edgeLen.push_back(length(m->tris[i].p[1] - m->tris[i].p[0]));
    float typical = scale;
    if (!edgeLen.empty()) { std::nth_element(edgeLen.begin(), edgeLen.begin() + long(edgeLen.size() / 10), edgeLen.end()); typical = edgeLen[edgeLen.size() / 10]; }
    const float weld = eps * 20;
    (void)typical;
    if (op == 3) {
        Geometry merged;
        for (int part = 0; part < 2; ++part) {
            std::vector<int> keepPrims;
            const Attrib* pc = result.findAttrib(AttribClass::Prim, "piece");
            for (size_t i = 0; i < result.numPrims(); ++i) if (int(pc->getF(i)) == part) keepPrims.push_back(int(i));
            Geometry one = fusePoints(result.extractPrims(keepPrims), weld);
            dropSlivers(one, weld);
            fixTJunctions(one, weld);
            fillPinholes(one, scale * 2e-3f);
            if (part == 0) merged = std::move(one); else merged.merge(one);
        }
        result = std::move(merged);
    } else {
        result = fusePoints(result, weld);
        dropSlivers(result, weld);
        fixTJunctions(result, weld);
        fillPinholes(result, scale * 2e-3f);
    }
    // the inputs' normals no longer match the new faces
    result.removeAttrib(AttribClass::Prim, "N");
    result.removeAttrib(AttribClass::Point, "__bcut");
    return true;
}

namespace {

// Splits polygons by a plane; keeps the side the normal points to (above), below, or both.
Geometry clipByPlane(const Geometry& g, const Vec3& origin, const Vec3& nrm, int keep, float& cutArea) {
    Vec3 n = normalize(nrm);
    Builder bd(g);
    cutArea = 0;
    std::vector<float> d(g.numPoints());
    for (size_t i = 0; i < g.numPoints(); ++i) d[i] = dot(g.P[i] - origin, n);
    for (size_t pi = 0; pi < g.numPrims(); ++pi) {
        const Prim& pr = g.prims[pi];
        bool above = false, below = false;
        for (int k = 0; k < pr.count; ++k) { float v = d[size_t(g.primVertex(int(pi), k))]; above |= v > 0; below |= v < 0; }
        for (int side = 0; side < 2; ++side) {
            if (keep != 2 && keep != side) continue;
            const bool wantAbove = side == 0;
            if (wantAbove ? !above : !below) { if (!above && !below && side == 0) bd.whole(int(pi), false); continue; }
            if (wantAbove ? !below : !above) { bd.whole(int(pi), false); continue; }
            // clip: walk the polygon (or polyline), interpolating attributes on crossing edges
            const int ne = pr.closed ? pr.count : pr.count - 1;
            auto inside = [&](float v) { return wantAbove ? v >= 0 : v <= 0; };
            std::vector<std::tuple<int, int, float>> verts; // (a, b, t): point a + (b - a) t
            for (int k = 0; k < pr.count; ++k) {
                int a = g.primVertex(int(pi), k);
                if (inside(d[size_t(a)])) verts.push_back({a, a, 0.f});
                if (k >= ne) continue;
                int b = g.primVertex(int(pi), (k + 1) % pr.count);
                float da = d[size_t(a)], db = d[size_t(b)];
                if ((da > 0 && db < 0) || (da < 0 && db > 0)) verts.push_back({a, b, da / (da - db)});
            }
            if (verts.size() < (pr.closed ? 3u : 2u)) continue;
            const int start = int(bd.out.P.size());
            for (auto& [a, b, t] : verts) {
                bd.out.P.push_back(g.P[size_t(a)] + (g.P[size_t(b)] - g.P[size_t(a)]) * t);
                for (auto& [name, at] : g.pointAttribs) {
                    Attrib& o = bd.out.pointAttribs[name];
                    for (int c = 0; c < at.tuple; ++c) {
                        float va = at.data[size_t(a) * size_t(at.tuple) + size_t(c)], vb = at.data[size_t(b) * size_t(at.tuple) + size_t(c)];
                        o.data.push_back(va + (vb - va) * t);
                    }
                }
            }
            for (int k = 0; k < int(verts.size()); ++k) bd.out.vertices.push_back(start + k);
            Prim np; np.start = int(bd.out.vertices.size()) - int(verts.size()); np.count = int(verts.size()); np.closed = pr.closed;
            bd.out.prims.push_back(np);
            bd.primAttrs(int(pi));
        }
    }
    return bd.out;
}

} // namespace

void registerModelingNodes(std::vector<NodeType>& types) {
    {
        NodeType t; t.name = "boolean"; t.label = "Boolean"; t.category = "Modifiers";
        t.description = "Union, intersection, subtraction or shatter of two closed meshes. Attributes carry over; the cut has no cracks.";
        t.minInputs = 2; t.maxInputs = 2;
        t.inputLabels = {"A", "B"};
        t.parms = {M("op", "Operation", {"Union", "Intersect", "Subtract (A - B)", "Shatter (A cut by B)"}, 2),
                   T("detriangulate", "Keep Uncut Polygons", true, "Polygons the cut doesn't touch stay as they were (quads stay quads)")};
        t.cook = [](CookContext& c) {
            const Geometry& a = c.input(0);
            const Geometry& b = c.input(1);
            std::string warn;
            Geometry out;
            booleanMeshes(a, b, c.i("op"), out, warn);
            if (!c.b("detriangulate")) {
                // triangulate everything for tools that want triangles
                Geometry tri;
                tri.pointAttribs = out.pointAttribs; tri.P = out.P;
                for (auto& [name, at] : out.primAttribs) { Attrib na; na.tuple = at.tuple; std::copy(at.defaults, at.defaults + 3, na.defaults); tri.primAttribs[name] = na; }
                for (size_t pi = 0; pi < out.numPrims(); ++pi) {
                    const Prim& pr = out.prims[pi];
                    for (int k = 1; k + 1 < pr.count; ++k) {
                        tri.addPrim({out.primVertex(int(pi), 0), out.primVertex(int(pi), k), out.primVertex(int(pi), k + 1)});
                        for (auto& [name, at] : out.primAttribs) tri.primAttribs[name].appendFrom(at, pi);
                    }
                }
                out = std::move(tri);
            }
            c.out = std::move(out);
            if (!warn.empty()) c.warning(warn.substr(0, warn.size() - 1));
        };
        types.push_back(t);
    }
    {
        NodeType t; t.name = "clip"; t.label = "Clip"; t.category = "Modifiers";
        t.description = "Cuts geometry with a plane and keeps the part above it, below it, or both (split along the plane).";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {M("keep", "Keep", {"Above (normal side)", "Below", "Both (split)"}), V("origin", "Origin", Vec3(0), -5, 5),
                   V("dir", "Direction", Vec3(0, 1, 0), -1, 1, "The plane's normal")};
        t.cook = [](CookContext& c) {
            Vec3 n = c.v("dir");
            if (length(n) < 1e-8f) { c.error("Direction must not be zero."); return; }
            float area = 0;
            c.out = clipByPlane(c.input(0), c.v("origin"), n, c.i("keep"), area);
            c.out.packed = c.input(0).packed;
        };
        types.push_back(t);
    }
}

} // namespace hd

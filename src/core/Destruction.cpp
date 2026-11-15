// Destruction nodes: Voronoi Fracture and the RBD (rigid body) Solver.
//
// RBD Solver overview
//   * Pieces come from the "piece" primitive attribute (as written by Voronoi Fracture) or, when it
//     is missing, from connectivity. Each piece collides as its convex hull; mass and inertia are
//     integrated from the hull volume at the given density.
//   * Glue: pieces that touch at the start frame are bonded. Bonded pieces move together as one
//     compound rigid body. When the impulse a piece receives in a substep exceeds
//     glue strength x piece mass (a velocity change in m/s) its bonds break and the compound splits.
//   * Contacts are found vertex-vs-hull (plus hull edge midpoints) between pieces of different bodies
//     and against the ground plane, then solved with sequential impulses (accumulated, clamped normal
//     impulses, Coulomb friction, restitution and Baumgarte position correction).
//   * Resting bodies go to sleep and wake when touched or when they lose support.
//   * Every simulated frame is cached; scrubbing back is instant and edits re-simulate.
#include "Fracture.h"
#include "Node.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <numeric>

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

// ------------------------------------------------------------------ small rigid-body math

struct Mat3 {
    float m[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    static Mat3 identity() { Mat3 r; r.m[0][0] = r.m[1][1] = r.m[2][2] = 1; return r; }
    Vec3 operator*(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
        return r;
    }
    Mat3 operator+(const Mat3& o) const { Mat3 r; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = m[i][j] + o.m[i][j]; return r; }
    Mat3 transposed() const { Mat3 r; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i]; return r; }
    Mat3 inverse() const {
        const float (&a)[3][3] = m;
        float c00 = a[1][1] * a[2][2] - a[1][2] * a[2][1], c01 = a[1][2] * a[2][0] - a[1][0] * a[2][2], c02 = a[1][0] * a[2][1] - a[1][1] * a[2][0];
        float det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;
        Mat3 r;
        if (std::fabs(det) < 1e-30f) return r;
        float id = 1.f / det;
        r.m[0][0] = c00 * id; r.m[1][0] = c01 * id; r.m[2][0] = c02 * id;
        r.m[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * id;
        r.m[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * id;
        r.m[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * id;
        r.m[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * id;
        r.m[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * id;
        r.m[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * id;
        return r;
    }
};

struct Quat {
    float w = 1, x = 0, y = 0, z = 0;
    Quat operator*(const Quat& b) const {
        return {w * b.w - x * b.x - y * b.y - z * b.z, w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y - x * b.z + y * b.w + z * b.x, w * b.z + x * b.y - y * b.x + z * b.w};
    }
    Quat conj() const { return {w, -x, -y, -z}; }
    void normalize() {
        float l = std::sqrt(w * w + x * x + y * y + z * z);
        if (l > 0) { w /= l; x /= l; y /= l; z /= l; } else { w = 1; x = y = z = 0; }
    }
    Vec3 rotate(const Vec3& v) const {
        Vec3 u(x, y, z);
        Vec3 t = cross(u, v) * 2.f;
        return v + t * w + cross(u, t);
    }
    Mat3 matrix() const {
        Mat3 r;
        r.m[0][0] = 1 - 2 * (y * y + z * z); r.m[0][1] = 2 * (x * y - w * z); r.m[0][2] = 2 * (x * z + w * y);
        r.m[1][0] = 2 * (x * y + w * z); r.m[1][1] = 1 - 2 * (x * x + z * z); r.m[1][2] = 2 * (y * z - w * x);
        r.m[2][0] = 2 * (x * z - w * y); r.m[2][1] = 2 * (y * z + w * x); r.m[2][2] = 1 - 2 * (x * x + y * y);
        return r;
    }
};

// ------------------------------------------------------------------ simulation data

struct Piece {
    ConvexHull hull;     // in piece space: world space at rest, centered on com0
    Vec3 com0;           // world center of mass at the start frame
    float mass = 1;
    Mat3 inertia;        // about com0, piece space
    float radius = 0;    // bounding sphere about com0
    std::vector<int> pts; // points of this piece in the rest geometry (dynamic pieces only)
    bool isStatic = false;
    Vec3 v0{0, 0, 0};    // initial velocity from the "v" point attribute
};

struct Body {
    Vec3 x;              // center of mass
    Quat q;
    Vec3 v, w;
    float invMass = 0;
    Mat3 invInertiaBody; // body space
    std::vector<int> members;
    std::vector<Vec3> offset; // member com in body space
    std::vector<Quat> rot;    // member piece-space -> body-space rotation
    bool isStatic = false;
    bool sleeping = false;
    float sleepTime = 0;
};

struct SimFrame {
    std::vector<Body> bodies;
    std::vector<char> bondAlive;
};

struct RbdState {
    uint64_t sig = 0;
    bool setupOk = false;
    std::string setupError;
    Geometry rest;                   // the input at the start frame; output keeps its topology
    std::vector<Piece> pieces;       // dynamic first, then static collider pieces
    int numDynamic = 0;
    std::vector<std::pair<int, int>> bonds;
    std::map<int, SimFrame> frames;
};

// Groups primitives into pieces by shared points (each connected part is one rigid piece).
std::vector<std::vector<int>> findPieces(const Geometry& g) {
    std::vector<std::vector<int>> out;
    std::vector<int> parent(g.numPoints());
    std::iota(parent.begin(), parent.end(), 0);
    std::function<int(int)> find = [&](int a) { while (parent[size_t(a)] != a) { parent[size_t(a)] = parent[size_t(parent[size_t(a)])]; a = parent[size_t(a)]; } return a; };
    for (auto& pr : g.prims)
        for (int k = 1; k < pr.count; ++k) {
            int a = find(g.vertices[size_t(pr.start)]), b = find(g.vertices[size_t(pr.start + k)]);
            if (a != b) parent[size_t(a)] = b;
        }
    std::map<int, int> idx;
    for (size_t pi = 0; pi < g.numPrims(); ++pi) {
        if (g.prims[pi].count == 0) continue;
        int r = find(g.primVertex(int(pi), 0));
        auto it = idx.find(r);
        if (it == idx.end()) { idx[r] = int(out.size()); out.push_back({}); it = idx.find(r); }
        out[size_t(it->second)].push_back(int(pi));
    }
    return out;
}

bool makePiece(const Geometry& pg, float density, Piece& p) {
    std::vector<Vec3> pts(pg.P.begin(), pg.P.end());
    ConvexHull hull;
    Vec3 mn, mx;
    pg.bounds(mn, mx);
    if (!buildConvexHull(pts, hull, std::max(1e-3f, length(mx - mn) * 0.01f))) return false;
    MassProps mp = computeMassProps(hull.pts, hull.tris);
    if (mp.volume <= 0) return false;
    p.com0 = mp.com;
    p.mass = float(mp.volume) * density;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) p.inertia.m[i][j] = float(mp.inertia[i][j]) * density;
    for (auto& v : hull.pts) v -= p.com0;
    for (auto& v : hull.edgeMids) v -= p.com0;
    for (auto& pl : hull.planes) pl.d -= dot(pl.n, p.com0);
    p.radius = 0;
    for (auto& v : hull.pts) p.radius = std::max(p.radius, length(v));
    p.hull = std::move(hull);
    return true;
}

// Rebuild bodies from bond connectivity. With `prev` the members inherit their current poses and velocities.
std::vector<Body> buildBodies(const RbdState& st, const std::vector<char>& bondAlive, const std::vector<Body>* prev,
                              const Vec3& initV, const Vec3& initW) {
    const int nd = st.numDynamic;
    std::vector<int> parent(static_cast<size_t>(nd));
    std::iota(parent.begin(), parent.end(), 0);
    auto find = [&](int a) { while (parent[size_t(a)] != a) { parent[size_t(a)] = parent[size_t(parent[size_t(a)])]; a = parent[size_t(a)]; } return a; };
    for (size_t b = 0; b < st.bonds.size(); ++b)
        if (bondAlive[b]) {
            int a = find(st.bonds[b].first), c = find(st.bonds[b].second);
            if (a != c) parent[size_t(a)] = c;
        }
    // current pose / velocity of every dynamic piece
    const size_t ndz = static_cast<size_t>(nd);
    std::vector<Vec3> pos(ndz), vel(ndz), angv(ndz);
    std::vector<Quat> ori(ndz);
    std::vector<char> asleep(ndz, 0);
    if (prev) {
        for (auto& b : *prev) {
            if (b.isStatic) continue;
            for (size_t k = 0; k < b.members.size(); ++k) {
                int m = b.members[k];
                pos[size_t(m)] = b.x + b.q.rotate(b.offset[k]);
                ori[size_t(m)] = b.q * b.rot[k];
                vel[size_t(m)] = b.v + cross(b.w, pos[size_t(m)] - b.x);
                angv[size_t(m)] = b.w;
                asleep[size_t(m)] = b.sleeping;
            }
        }
    } else {
        for (int m = 0; m < nd; ++m) {
            pos[size_t(m)] = st.pieces[size_t(m)].com0;
            ori[size_t(m)] = Quat();
            vel[size_t(m)] = initV + st.pieces[size_t(m)].v0;
            angv[size_t(m)] = initW;
        }
    }
    std::map<int, std::vector<int>> groups;
    for (int m = 0; m < nd; ++m) groups[find(m)].push_back(m);
    std::vector<Body> bodies;
    for (auto& [root, mem] : groups) {
        Body b;
        b.members = mem;
        float M = 0;
        Vec3 com(0);
        bool anyStatic = false;
        for (int m : mem) {
            M += st.pieces[size_t(m)].mass;
            com += pos[size_t(m)] * st.pieces[size_t(m)].mass;
            anyStatic = anyStatic || st.pieces[size_t(m)].isStatic;
        }
        com /= std::max(M, 1e-20f);
        b.x = com;
        // the body frame takes the orientation of its first member
        b.q = ori[size_t(mem[0])];
        Quat qi = b.q.conj();
        Mat3 I;
        Vec3 lin(0), ang(0);
        bool allAsleep = true;
        for (int m : mem) {
            const Piece& p = st.pieces[size_t(m)];
            Vec3 o = qi.rotate(pos[size_t(m)] - com);
            Quat r = qi * ori[size_t(m)];
            b.offset.push_back(o);
            b.rot.push_back(r);
            Mat3 R = r.matrix();
            Mat3 Ib = R * p.inertia * R.transposed();
            float oo = dot(o, o);
            float ov[3] = {o.x, o.y, o.z};
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) Ib.m[i][j] += p.mass * ((i == j ? oo : 0.f) - ov[i] * ov[j]);
            I = I + Ib;
            lin += vel[size_t(m)] * p.mass;
            ang += angv[size_t(m)] * p.mass;
            allAsleep = allAsleep && asleep[size_t(m)];
        }
        b.v = lin / std::max(M, 1e-20f);
        b.w = ang / std::max(M, 1e-20f);
        b.isStatic = anyStatic;
        b.invMass = anyStatic ? 0.f : 1.f / std::max(M, 1e-20f);
        b.invInertiaBody = anyStatic ? Mat3() : I.inverse();
        if (anyStatic) b.v = b.w = Vec3(0);
        b.sleeping = prev && allAsleep && mem.size() > 0;
        bodies.push_back(std::move(b));
    }
    // static collider pieces: one body each
    for (size_t m = size_t(nd); m < st.pieces.size(); ++m) {
        Body b;
        b.members = {int(m)};
        b.x = st.pieces[m].com0;
        b.offset = {Vec3(0)};
        b.rot = {Quat()};
        b.isStatic = true;
        bodies.push_back(std::move(b));
    }
    return bodies;
}

struct Contact {
    int a, b;          // body indices; b = -1 for the ground
    int pa, pb;        // piece indices (pb = -1 for the ground)
    Vec3 p, n;         // n points from b to a
    float pen;
    Vec3 ra, rb, t1, t2;
    float kn, kt1, kt2, bias;
    float Pn = 0, Pt1 = 0, Pt2 = 0;
};

struct SimParams {
    Vec3 gravity;
    float dt;
    int iterations;
    float restitution, friction;
    bool ground;
    float groundY;
    bool glue;
    float glueStrength;
    float propagate;
    float linDamp, angDamp;
    bool sleep;
};

inline Mat3 worldInvInertia(const Body& b, bool asStatic) {
    if (asStatic || b.isStatic) return Mat3();
    Mat3 R = b.q.matrix();
    return R * b.invInertiaBody * R.transposed();
}

// Advances one substep. Returns true if any bond broke (bodies must be rebuilt).
bool substep(const RbdState& st, SimFrame& fr, const SimParams& sp, std::vector<Contact>& contacts) {
    std::vector<Body>& bodies = fr.bodies;
    const float dt = sp.dt;
    // integrate velocities
    for (auto& b : bodies) {
        if (b.isStatic || b.sleeping) continue;
        b.v += sp.gravity * dt;
        b.v *= std::max(0.f, 1.f - sp.linDamp * dt);
        b.w *= std::max(0.f, 1.f - sp.angDamp * dt);
    }
    // piece world poses
    struct PW { Vec3 pos; Quat q; Mat3 R; int body; };
    std::vector<PW> pw(st.pieces.size());
    for (size_t bi = 0; bi < bodies.size(); ++bi) {
        const Body& b = bodies[bi];
        for (size_t k = 0; k < b.members.size(); ++k) {
            PW w;
            w.pos = b.x + b.q.rotate(b.offset[k]);
            w.q = b.q * b.rot[k];
            w.R = w.q.matrix();
            w.body = int(bi);
            pw[size_t(b.members[k])] = w;
        }
    }
    auto movable = [&](int bi) { return bi >= 0 && !bodies[size_t(bi)].isStatic && !bodies[size_t(bi)].sleeping; };

    // ---- contact generation
    contacts.clear();
    std::vector<int> touchCount(bodies.size(), 0);
    const float margin = 0.f;
    if (sp.ground) {
        for (int pi = 0; pi < st.numDynamic; ++pi) {
            const PW& w = pw[size_t(pi)];
            const Piece& p = st.pieces[size_t(pi)];
            if (w.pos.y - p.radius > sp.groundY + margin) continue;
            int added = 0;
            for (auto& hv : p.hull.pts) {
                Vec3 x = w.pos + w.R * hv;
                if (x.y >= sp.groundY) continue;
                Contact c{};
                c.a = w.body; c.b = -1; c.pa = pi; c.pb = -1;
                c.p = x; c.n = Vec3(0, 1, 0); c.pen = sp.groundY - x.y;
                contacts.push_back(c);
                ++added;
            }
            if (added) touchCount[size_t(w.body)] += added;
        }
    }
    // broadphase: sweep along x on bounding spheres
    std::vector<int> order(st.pieces.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return pw[size_t(a)].pos.x - st.pieces[size_t(a)].radius < pw[size_t(b)].pos.x - st.pieces[size_t(b)].radius; });
    for (size_t i = 0; i < order.size(); ++i) {
        int A = order[i];
        const Piece& PA = st.pieces[size_t(A)];
        const PW& WA = pw[size_t(A)];
        float maxX = WA.pos.x + PA.radius;
        for (size_t j = i + 1; j < order.size(); ++j) {
            int B = order[j];
            const Piece& PB = st.pieces[size_t(B)];
            const PW& WB = pw[size_t(B)];
            if (WB.pos.x - PB.radius > maxX) break;
            if (WA.body == WB.body) continue;
            const Body& ba = bodies[size_t(WA.body)];
            const Body& bb = bodies[size_t(WB.body)];
            bool aMov = !ba.isStatic && !ba.sleeping, bMov = !bb.isStatic && !bb.sleeping;
            // pairs where neither side moves are still tested so sleeping bodies know they are supported
            if (ba.isStatic && bb.isStatic) continue;
            (void)aMov; (void)bMov;
            Vec3 d = WA.pos - WB.pos;
            float rr = PA.radius + PB.radius + margin;
            if (dot(d, d) > rr * rr) continue;
            // gather penetrating samples both ways, keep the deepest few
            std::vector<Contact> local;
            auto test = [&](int X, const PW& WX, int Y, const PW& WY, bool flip) {
                const Piece& PX = st.pieces[size_t(X)];
                const Piece& PY = st.pieces[size_t(Y)];
                Mat3 RYt = WY.R.transposed();
                auto sample = [&](const Vec3& hv) {
                    Vec3 x = WX.pos + WX.R * hv;
                    Vec3 lp = RYt * (x - WY.pos);
                    if (dot(lp, lp) > PY.radius * PY.radius) return;
                    int pl = -1;
                    float dist = PY.hull.distance(lp, &pl);
                    if (dist >= 0 || pl < 0) return;
                    Vec3 nY = WY.R * PY.hull.planes[size_t(pl)].n; // outward of Y
                    Contact c{};
                    // normal must point from body b to body a: a = owner of X unless flipped
                    if (!flip) { c.a = WX.body; c.b = WY.body; c.pa = X; c.pb = Y; c.n = nY; }
                    else { c.a = WY.body; c.b = WX.body; c.pa = Y; c.pb = X; c.n = -nY; }
                    c.p = x; c.pen = -dist;
                    local.push_back(c);
                };
                for (auto& hv : PX.hull.pts) sample(hv);
                for (auto& hv : PX.hull.edgeMids) sample(hv);
            };
            test(A, WA, B, WB, false);
            test(B, WB, A, WA, true);
            if (local.empty()) continue;
            if (local.size() > 16) {
                std::partial_sort(local.begin(), local.begin() + 16, local.end(), [](const Contact& x, const Contact& y) { return x.pen > y.pen; });
                local.resize(16);
            }
            touchCount[size_t(WA.body)] += int(local.size());
            touchCount[size_t(WB.body)] += int(local.size());
            // wake sleeping bodies hit by moving ones
            for (int k = 0; k < 2; ++k) {
                Body& s = bodies[size_t(k ? WB.body : WA.body)];
                const Body& o = bodies[size_t(k ? WA.body : WB.body)];
                if (s.sleeping && !o.isStatic && !o.sleeping && (length(o.v) > 0.15f || length(o.w) > 0.3f)) { s.sleeping = false; s.sleepTime = 0; }
            }
            contacts.insert(contacts.end(), local.begin(), local.end());
        }
    }
    // sleeping bodies that lost all support wake up
    for (size_t bi = 0; bi < bodies.size(); ++bi)
        if (bodies[bi].sleeping && touchCount[bi] == 0) { bodies[bi].sleeping = false; bodies[bi].sleepTime = 0; }

    // ---- prepare
    std::vector<Mat3> invI(bodies.size());
    for (size_t bi = 0; bi < bodies.size(); ++bi) invI[bi] = worldInvInertia(bodies[bi], bodies[bi].sleeping);
    auto invM = [&](int bi) { return movable(bi) ? bodies[size_t(bi)].invMass : 0.f; };
    const float slop = 0.002f, beta = 0.2f, maxBias = 3.f;
    std::vector<Contact> solve;
    for (auto& c : contacts) {
        if (!movable(c.a) && !movable(c.b)) continue;
        const Body& A = bodies[size_t(c.a)];
        c.ra = c.p - A.x;
        c.rb = c.b >= 0 ? c.p - bodies[size_t(c.b)].x : Vec3(0);
        Vec3 t = std::fabs(c.n.x) < 0.57f ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
        c.t1 = normalize(cross(c.n, t));
        c.t2 = cross(c.n, c.t1);
        auto eff = [&](const Vec3& dir) {
            float k = invM(c.a) + (c.b >= 0 ? invM(c.b) : 0.f);
            if (movable(c.a)) k += dot(dir, cross(invI[size_t(c.a)] * cross(c.ra, dir), c.ra));
            if (c.b >= 0 && movable(c.b)) k += dot(dir, cross(invI[size_t(c.b)] * cross(c.rb, dir), c.rb));
            return k > 1e-20f ? 1.f / k : 0.f;
        };
        c.kn = eff(c.n); c.kt1 = eff(c.t1); c.kt2 = eff(c.t2);
        Vec3 va = A.v + cross(A.w, c.ra);
        Vec3 vb = c.b >= 0 ? bodies[size_t(c.b)].v + cross(bodies[size_t(c.b)].w, c.rb) : Vec3(0);
        float vn = dot(va - vb, c.n);
        c.bias = std::min(maxBias, beta / dt * std::max(0.f, c.pen - slop));
        if (vn < -1.f) c.bias = std::max(c.bias, -sp.restitution * vn);
        solve.push_back(c);
    }
    contacts.swap(solve);

    // ---- sequential impulses
    auto apply = [&](const Contact& c, const Vec3& P) {
        if (movable(c.a)) { Body& A = bodies[size_t(c.a)]; A.v += P * A.invMass; A.w += invI[size_t(c.a)] * cross(c.ra, P); }
        if (c.b >= 0 && movable(c.b)) { Body& B = bodies[size_t(c.b)]; B.v -= P * B.invMass; B.w -= invI[size_t(c.b)] * cross(c.rb, P); }
    };
    for (int it = 0; it < sp.iterations; ++it) {
        for (auto& c : contacts) {
            const Body& A = bodies[size_t(c.a)];
            Vec3 va = A.v + cross(A.w, c.ra);
            Vec3 vb = c.b >= 0 ? bodies[size_t(c.b)].v + cross(bodies[size_t(c.b)].w, c.rb) : Vec3(0);
            Vec3 dv = va - vb;
            // normal
            float lam = (c.bias - dot(dv, c.n)) * c.kn;
            float old = c.Pn;
            c.Pn = std::max(0.f, old + lam);
            apply(c, c.n * (c.Pn - old));
            // friction
            va = A.v + cross(A.w, c.ra);
            vb = c.b >= 0 ? bodies[size_t(c.b)].v + cross(bodies[size_t(c.b)].w, c.rb) : Vec3(0);
            dv = va - vb;
            float lim = sp.friction * c.Pn;
            float l1 = -dot(dv, c.t1) * c.kt1, o1 = c.Pt1;
            c.Pt1 = std::clamp(o1 + l1, -lim, lim);
            float l2 = -dot(dv, c.t2) * c.kt2, o2 = c.Pt2;
            c.Pt2 = std::clamp(o2 + l2, -lim, lim);
            apply(c, c.t1 * (c.Pt1 - o1) + c.t2 * (c.Pt2 - o2));
        }
    }

    // ---- glue breaking: impulse per piece vs strength * mass
    bool broke = false;
    if (sp.glue && !st.bonds.empty()) {
        std::vector<float> imp(st.pieces.size(), 0.f);
        for (auto& c : contacts) {
            float P = std::sqrt(c.Pn * c.Pn + c.Pt1 * c.Pt1 + c.Pt2 * c.Pt2);
            imp[size_t(c.pa)] += P;
            if (c.pb >= 0) imp[size_t(c.pb)] += P;
        }
        // impacts propagate through intact bonds: a piece that breaks passes part of its impulse on
        std::vector<char> breakPiece(st.pieces.size(), 0);
        std::vector<int> queue;
        for (int pi = 0; pi < st.numDynamic; ++pi)
            if (imp[size_t(pi)] > sp.glueStrength * st.pieces[size_t(pi)].mass) { breakPiece[size_t(pi)] = 1; queue.push_back(pi); }
        if (sp.propagate > 0 && !queue.empty()) {
            std::vector<std::vector<int>> adj(size_t(st.numDynamic));
            for (size_t b = 0; b < st.bonds.size(); ++b)
                if (fr.bondAlive[b]) { adj[size_t(st.bonds[b].first)].push_back(st.bonds[b].second); adj[size_t(st.bonds[b].second)].push_back(st.bonds[b].first); }
            for (size_t qi = 0; qi < queue.size(); ++qi) {
                int pi = queue[qi];
                if (adj[size_t(pi)].empty()) continue;
                float share = imp[size_t(pi)] * sp.propagate / float(adj[size_t(pi)].size());
                for (int nb : adj[size_t(pi)]) {
                    if (breakPiece[size_t(nb)]) continue;
                    imp[size_t(nb)] += share;
                    if (imp[size_t(nb)] > sp.glueStrength * st.pieces[size_t(nb)].mass) { breakPiece[size_t(nb)] = 1; queue.push_back(nb); }
                }
            }
        }
        for (size_t b = 0; b < st.bonds.size(); ++b)
            if (fr.bondAlive[b] && (breakPiece[size_t(st.bonds[b].first)] || breakPiece[size_t(st.bonds[b].second)])) { fr.bondAlive[b] = 0; broke = true; }
    }

    // ---- integrate positions, sleep
    for (auto& b : bodies) {
        if (b.isStatic || b.sleeping) continue;
        b.x += b.v * dt;
        Quat wq{0, b.w.x, b.w.y, b.w.z};
        Quat dq = wq * b.q;
        b.q.w += 0.5f * dt * dq.w; b.q.x += 0.5f * dt * dq.x; b.q.y += 0.5f * dt * dq.y; b.q.z += 0.5f * dt * dq.z;
        b.q.normalize();
        if (sp.sleep) {
            if (length(b.v) < 0.05f && length(b.w) < 0.1f) b.sleepTime += dt; else b.sleepTime = 0;
            if (b.sleepTime > 0.4f) { b.sleeping = true; b.v = b.w = Vec3(0); }
        }
    }
    return broke;
}

void cookRbd(CookContext& c) {
    RbdState& st = c.state<RbdState>();
    const uint64_t sig = c.setupSignature();
    const int start = c.i("startframe");
    const int frame = int(std::floor(c.frame() + 0.5f));
    Node& node = c.node();
    node.simCacheStart = 0; node.simCacheEnd = -1;

    if (st.sig != sig) {
        st = RbdState();
        st.sig = sig;
        c.setEvalFrame(float(start));
        Geometry in0, col;
        if (!c.inputAtFrame(0, float(start), in0)) { c.clearEvalFrame(); c.error("Input 1 (pieces) failed to cook."); return; }
        bool haveCol = c.hasInput(1) && c.inputAtFrame(1, float(start), col);
        const float density = std::max(1e-3f, c.f("density"));
        st.rest = in0;
        auto pieces = findPieces(in0);
        const Attrib* activeA = in0.findAttrib(AttribClass::Point, "active");
        const Attrib* vA = in0.findAttrib(AttribClass::Point, "v");
        if (vA && vA->tuple != 3) vA = nullptr;
        int skipped = 0;
        for (auto& prims : pieces) {
            Geometry pg = in0.extractPrims(prims);
            if (pg.numPoints() == 0) continue;
            Piece p;
            if (!makePiece(pg, density, p)) { ++skipped; continue; }
            float actSum = 0, n = float(pg.numPoints());
            Vec3 v0(0);
            const Attrib* pa = pg.findAttrib(AttribClass::Point, "active");
            const Attrib* pv = pg.findAttrib(AttribClass::Point, "v");
            for (size_t i = 0; i < pg.numPoints(); ++i) {
                if (pa) actSum += pa->getF(i);
                if (pv && pv->tuple == 3) v0 += pv->getV(i);
            }
            if (activeA && actSum / n < 0.5f) p.isStatic = true; // inactive pieces stay put but collide
            if (vA) p.v0 = v0 / n;
            {
                std::vector<char> seen(in0.numPoints(), 0);
                for (int pr : prims)
                    for (int k = 0; k < in0.prims[size_t(pr)].count; ++k) {
                        int v = in0.primVertex(pr, k);
                        if (!seen[size_t(v)]) { seen[size_t(v)] = 1; p.pts.push_back(v); }
                    }
            }
            st.pieces.push_back(std::move(p));
        }
        st.numDynamic = int(st.pieces.size());
        if (haveCol)
            for (auto& prims : findPieces(col)) {
                Geometry pg = col.extractPrims(prims);
                Piece p;
                if (!makePiece(pg, density, p)) { ++skipped; continue; }
                p.isStatic = true;
                st.pieces.push_back(std::move(p));
            }
        if (skipped) c.warning(std::to_string(skipped) + " piece(s) had no volume and were ignored.");
        // glue bonds between dynamic pieces that touch at the start
        if (c.b("glue")) {
            const float tol = std::max(0.f, c.f("bondtol"));
            for (int a = 0; a < st.numDynamic; ++a)
                for (int b = a + 1; b < st.numDynamic; ++b) {
                    const Piece& A = st.pieces[size_t(a)];
                    const Piece& B = st.pieces[size_t(b)];
                    if (A.isStatic || B.isStatic) continue;
                    float rr = A.radius + B.radius + tol;
                    if (dot(A.com0 - B.com0, A.com0 - B.com0) > rr * rr) continue;
                    int touching = 0;
                    for (auto& hv : A.hull.pts) {
                        Vec3 lp = hv + A.com0 - B.com0;
                        if (B.hull.distance(lp) <= tol) ++touching;
                        if (touching >= 3) break;
                    }
                    if (touching >= 3) st.bonds.push_back({a, b});
                }
        }
        st.setupOk = true;
        c.clearEvalFrame();
    }
    if (!st.setupOk) { c.error(st.setupError.empty() ? "Setup failed." : st.setupError); return; }

    auto output = [&](const SimFrame* fr) {
        Geometry g = st.rest;
        Attrib& vA = g.addAttrib(AttribClass::Point, "v", 3);
        if (fr) {
            for (auto& b : fr->bodies) {
                if (b.members.empty()) continue;
                for (size_t k = 0; k < b.members.size(); ++k) {
                    int m = b.members[k];
                    if (m >= st.numDynamic) continue;
                    const Piece& p = st.pieces[size_t(m)];
                    Vec3 pos = b.x + b.q.rotate(b.offset[k]);
                    Quat q = b.q * b.rot[k];
                    for (int i : p.pts) {
                        Vec3 local = g.P[size_t(i)] - p.com0;
                        Vec3 w = pos + q.rotate(local);
                        g.P[size_t(i)] = w;
                        vA.setV(size_t(i), b.v + cross(b.w, w - b.x));
                    }
                }
            }
            if (Attrib* N = g.findAttrib(AttribClass::Point, "N")) {
                if (N->tuple == 3)
                    for (auto& b : fr->bodies)
                        for (size_t k = 0; k < b.members.size(); ++k) {
                            int m = b.members[k];
                            if (m >= st.numDynamic) continue;
                            Quat q = b.q * b.rot[k];
                            for (int i : st.pieces[size_t(m)].pts) N->setV(size_t(i), q.rotate(N->getV(size_t(i))));
                        }
            }
        }
        c.out = std::move(g);
    };

    if (frame <= start) { output(nullptr); return; }
    if (frame - start > 100000) { c.error("Frame is too far from the start frame."); return; }

    int have = start;
    {
        auto it = st.frames.upper_bound(frame);
        if (it != st.frames.begin()) { --it; have = it->first; }
    }
    SimFrame cur;
    if (have > start && st.frames.count(have)) cur = st.frames[have];
    else {
        have = start;
        c.setEvalFrame(float(start));
        cur.bondAlive.assign(st.bonds.size(), 1);
        cur.bodies = buildBodies(st, cur.bondAlive, nullptr, c.v("initv"), c.v("initw"));
        c.clearEvalFrame();
    }
    std::vector<Contact> contacts;
    for (int k = have + 1; k <= frame; ++k) {
        c.setEvalFrame(float(k));
        SimParams sp;
        const int subs = std::clamp(c.i("substeps"), 1, 100);
        sp.dt = 1.f / (c.fps() * float(subs));
        sp.gravity = c.v("gravity");
        sp.iterations = std::clamp(c.i("iterations"), 1, 200);
        sp.restitution = std::clamp(c.f("bounce"), 0.f, 1.f);
        sp.friction = std::max(0.f, c.f("friction"));
        sp.ground = c.b("ground");
        sp.groundY = c.f("groundy");
        sp.glue = c.b("glue");
        sp.glueStrength = std::max(0.f, c.f("gluestrength"));
        sp.propagate = std::clamp(c.f("propagate"), 0.f, 1.f);
        sp.linDamp = std::max(0.f, c.f("lindamp"));
        sp.angDamp = std::max(0.f, c.f("angdamp"));
        sp.sleep = c.b("sleep");
        for (int s = 0; s < subs; ++s) {
            if (substep(st, cur, sp, contacts)) {
                std::vector<Body> prev = cur.bodies;
                cur.bodies = buildBodies(st, cur.bondAlive, &prev, Vec3(0), Vec3(0));
                for (auto& b : cur.bodies) if (!b.isStatic) { b.sleeping = false; b.sleepTime = 0; }
            }
        }
        st.frames[k] = cur;
    }
    c.clearEvalFrame();
    // bound memory: frames hold only poses, but cap the count anyway
    while (st.frames.size() > 5000 && st.frames.begin()->first != frame) st.frames.erase(st.frames.begin());
    output(&st.frames[frame]);
    int lo = frame, hi = frame;
    while (st.frames.count(lo - 1)) --lo;
    while (st.frames.count(hi + 1)) ++hi;
    node.simCacheStart = lo;
    node.simCacheEnd = hi;
}

void cookVoronoi(CookContext& c) {
    const Geometry& in = c.input(0);
    std::vector<Vec3> sites;
    if (c.hasInput(1)) {
        const Geometry& pts = c.input(1);
        sites = pts.P;
    } else {
        const int want = std::clamp(c.i("pieces"), 1, 5000);
        Vec3 mn, mx;
        in.bounds(mn, mx);
        uint32_t seed = uint32_t(c.i("seed")) * 7919u + 13u;
        int tries = 0;
        while (int(sites.size()) < want && tries < want * 200) {
            uint32_t s = seed + uint32_t(tries) * 3u;
            ++tries;
            Vec3 p(mn.x + (mx.x - mn.x) * hashRandom(s), mn.y + (mx.y - mn.y) * hashRandom(s + 1), mn.z + (mx.z - mn.z) * hashRandom(s + 2));
            if (pointInsideMesh(in, p)) sites.push_back(p);
        }
    }
    if (sites.empty()) { c.error("No cell points: connect points to input 2 or raise Pieces (the mesh must be closed)."); return; }
    if (sites.size() > 20000) { c.error("Too many cell points (limit 20000)."); return; }
    FractureOptions opt;
    opt.colorInside = c.b("colorinside");
    opt.insideColor = c.v("insidecolor");
    FractureStats stats;
    c.out = voronoiFracture(in, sites, opt, &stats);
    if (stats.pieces == 0) c.error("Fracturing produced no pieces. Is the input a closed mesh?");
    else if (stats.capFallbacks) c.warning(std::to_string(stats.capFallbacks) + " cut face(s) could not be triangulated cleanly.");
}

} // namespace

void registerDestructionNodes(std::vector<NodeType>& types) {
    {
        NodeType t; t.name = "voronoifracture"; t.label = "Voronoi Fracture"; t.category = "Destruction";
        t.description = "Cuts a closed mesh into Voronoi pieces around points. Cut faces get inside=1 and their own color; every piece gets a piece number.";
        t.minInputs = 1; t.maxInputs = 2;
        t.inputLabels = {"Geometry to fracture (closed)", "Cell points (optional)"};
        t.parms = {
            I("pieces", "Pieces (without input 2)", 20, 1, 500, "Random cell points inside the mesh when input 2 is empty"),
            I("seed", "Seed", 0, 0, 100),
            T("colorinside", "Color Inside Faces", true),
            V("insidecolor", "Inside Color", Vec3(0.85f, 0.35f, 0.15f), 0, 1),
        };
        for (auto& p : t.parms) if (p.name == "insidecolor") p.help = "color";
        t.cook = cookVoronoi;
        types.push_back(t);
    }
    {
        NodeType t; t.name = "rbdsolver"; t.label = "RBD Solver"; t.category = "Destruction";
        t.description = "Rigid body dynamics: pieces collide, bounce and pile up. Touching pieces are glued and break apart on impact.";
        t.minInputs = 1; t.maxInputs = 2;
        t.inputLabels = {"Pieces (each connected part; point v and active are read)", "Static collider (optional)"};
        t.timeDependent = true;
        t.parms = {
            I("startframe", "Start Frame", 1, 1, 240),
            I("substeps", "Substeps per Frame", 10, 1, 40),
            I("iterations", "Solver Iterations", 15, 1, 60),
            F("density", "Density (kg/m^3)", 1000, 1, 10000),
            V("gravity", "Gravity", Vec3(0, -9.81f, 0), -20, 20),
            V("initv", "Initial Velocity", Vec3(0, 0, 0), -20, 20, "Added to each piece's own v attribute"),
            V("initw", "Initial Angular Velocity (rad/s)", Vec3(0, 0, 0), -10, 10),
            F("bounce", "Bounce", 0.2f, 0, 1),
            F("friction", "Friction", 0.6f, 0, 2),
            T("ground", "Ground Plane", true),
            F("groundy", "Ground Height", 0, -5, 5),
            T("glue", "Glue Touching Pieces", true),
            F("gluestrength", "Glue Strength (m/s)", 3, 0, 50, "A piece breaks free when one substep's impulse on it exceeds this velocity change"),
            F("propagate", "Impact Propagation", 0.6f, 0, 1, "Share of a breaking piece's impulse passed on to the pieces glued to it"),
            F("bondtol", "Glue Distance", 0.01f, 0, 0.2f, "Pieces closer than this at the start are glued"),
            F("lindamp", "Linear Damping", 0.02f, 0, 2),
            F("angdamp", "Angular Damping", 0.1f, 0, 2),
            T("sleep", "Sleep Resting Pieces", true),
        };
        t.cook = cookRbd;
        types.push_back(t);
    }
}

} // namespace hd

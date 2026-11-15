// Rigging and character animation nodes.
//
// A skeleton is ordinary geometry: one point per joint, a "parent" point attribute (-1 for roots, parents
// always come before their children) and a two-point polyline for every bone. A posed skeleton also carries
// each joint's world rotation relative to its rest pose, as three column vectors "xfx", "xfy", "xfz".
// A joint's rotation turns everything below it about the joint's position.
//
//   Skeleton         build a joint chain, or turn polylines into skeletons
//   Joint Pose (FK)  local rotations per joint (the "rot" attribute plus up to four parameter slots)
//   IK Chain         FABRIK solve of a joint chain toward a target, with a pole vector
//   Capture          automatic skin weights by distance to the bones (up to four influences)
//   Bone Deform      linear blend skinning of a captured mesh by a posed skeleton
//   Blend Shapes     weighted sum of shape targets
//   Motion Capture   BVH file import, animated
//   Retarget         copy a posed skeleton's rotations onto another skeleton of the same hierarchy
//   Time Shift       evaluate the input at another frame
#include "Node.h"
#include "Image.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>

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
ParmDef FileP(const char* n, const char* l, const char* help = "") {
    ParmDef p; p.name = n; p.label = l; p.type = ParmType::File; p.help = help; return p;
}

// ------------------------------------------------------------------ 3x3 rotations
struct R3 {
    Vec3 c[3] = {Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)}; // columns
    Vec3 operator*(const Vec3& v) const { return c[0] * v.x + c[1] * v.y + c[2] * v.z; }
    R3 operator*(const R3& o) const { R3 r; for (int i = 0; i < 3; ++i) r.c[i] = (*this) * o.c[i]; return r; }
    R3 transposed() const {
        R3 r;
        r.c[0] = Vec3(c[0].x, c[1].x, c[2].x);
        r.c[1] = Vec3(c[0].y, c[1].y, c[2].y);
        r.c[2] = Vec3(c[0].z, c[1].z, c[2].z);
        return r;
    }
};

R3 axisAngle(Vec3 axis, float ang) {
    axis = normalize(axis);
    R3 r;
    if (dot(axis, axis) < 0.5f) return r;
    float c = std::cos(ang), s = std::sin(ang), t = 1 - c;
    float x = axis.x, y = axis.y, z = axis.z;
    r.c[0] = Vec3(t * x * x + c, t * x * y + s * z, t * x * z - s * y);
    r.c[1] = Vec3(t * x * y - s * z, t * y * y + c, t * y * z + s * x);
    r.c[2] = Vec3(t * x * z + s * y, t * y * z - s * x, t * z * z + c);
    return r;
}
R3 rotX(float a) { return axisAngle(Vec3(1, 0, 0), a); }
R3 rotY(float a) { return axisAngle(Vec3(0, 1, 0), a); }
R3 rotZ(float a) { return axisAngle(Vec3(0, 0, 1), a); }
// Euler degrees, X applied first (like Transform): R = Rz * Ry * Rx
R3 euler(const Vec3& deg) { return rotZ(radians(deg.z)) * rotY(radians(deg.y)) * rotX(radians(deg.x)); }

// Shortest rotation taking direction a onto direction b.
R3 alignRot(const Vec3& a, const Vec3& b) {
    Vec3 na = normalize(a), nb = normalize(b);
    float d = std::clamp(dot(na, nb), -1.f, 1.f);
    Vec3 ax = cross(na, nb);
    if (length(ax) < 1e-7f) {
        if (d > 0) return R3();
        Vec3 perp = std::fabs(na.x) < 0.9f ? cross(na, Vec3(1, 0, 0)) : cross(na, Vec3(0, 1, 0));
        return axisAngle(perp, kPi);
    }
    return axisAngle(ax, std::acos(d));
}

R3 getR(const Geometry& g, size_t i) {
    R3 r;
    const Attrib* a = g.findAttrib(AttribClass::Point, "xfx");
    const Attrib* b = g.findAttrib(AttribClass::Point, "xfy");
    const Attrib* c = g.findAttrib(AttribClass::Point, "xfz");
    if (a && b && c && a->tuple == 3 && b->tuple == 3 && c->tuple == 3 && i < a->count()) { r.c[0] = a->getV(i); r.c[1] = b->getV(i); r.c[2] = c->getV(i); }
    return r;
}
void setR(Geometry& g, size_t i, const R3& r) {
    g.findAttrib(AttribClass::Point, "xfx")->setV(i, r.c[0]);
    g.findAttrib(AttribClass::Point, "xfy")->setV(i, r.c[1]);
    g.findAttrib(AttribClass::Point, "xfz")->setV(i, r.c[2]);
}
void ensureSkelAttribs(Geometry& g) {
    g.addAttrib(AttribClass::Point, "parent", 1, Vec3(-1));
    g.addAttrib(AttribClass::Point, "xfx", 3, Vec3(1, 0, 0));
    g.addAttrib(AttribClass::Point, "xfy", 3, Vec3(0, 1, 0));
    g.addAttrib(AttribClass::Point, "xfz", 3, Vec3(0, 0, 1));
    g.syncAttribSizes();
}

std::vector<int> parentsOf(const Geometry& g, std::string* err) {
    std::vector<int> par(g.numPoints(), -1);
    const Attrib* a = g.findAttrib(AttribClass::Point, "parent");
    if (!a) { if (err) *err = "Input is not a skeleton (no parent attribute). Use a Skeleton node."; return {}; }
    for (size_t i = 0; i < g.numPoints(); ++i) {
        int p = int(std::lround(a->getF(i)));
        if (p >= int(i)) { if (err) *err = "Joint " + std::to_string(i) + " comes before its parent " + std::to_string(p) + "."; return {}; }
        par[i] = p;
    }
    return par;
}

// Builds world rotations and positions from local rotations (rest frames are the identity).
void solveFK(const Geometry& rest, const std::vector<int>& par, const std::vector<R3>& local, Geometry& out) {
    out = rest;
    ensureSkelAttribs(out);
    std::vector<R3> world(rest.numPoints());
    for (size_t j = 0; j < rest.numPoints(); ++j) {
        int p = par[j];
        if (p < 0) { world[j] = local[j]; out.P[j] = rest.P[j]; }
        else {
            out.P[j] = out.P[size_t(p)] + world[size_t(p)] * (rest.P[j] - rest.P[size_t(p)]);
            world[j] = world[size_t(p)] * local[j];
        }
        setR(out, j, world[j]);
    }
}

// ------------------------------------------------------------------ Skeleton
void cookSkeleton(CookContext& c) {
    Geometry& o = c.out;
    if (c.hasInput(0)) {
        // polylines (open curves) become chains; points shared between curves join the chains
        const Geometry& in = c.input(0);
        std::vector<int> par(in.numPoints(), -2);
        for (auto& pr : in.prims) {
            if (pr.count < 2) continue;
            for (int k = 0; k < pr.count; ++k) {
                int v = in.vertices[size_t(pr.start + k)];
                if (par[size_t(v)] == -2) par[size_t(v)] = k == 0 ? -1 : in.vertices[size_t(pr.start + k - 1)];
            }
        }
        // order points so parents come first
        std::vector<int> order, newIdx(in.numPoints(), -1);
        std::vector<char> done(in.numPoints(), 0);
        std::function<void(int)> visit = [&](int v) {
            if (done[size_t(v)] || par[size_t(v)] == -2) return;
            done[size_t(v)] = 1;
            if (par[size_t(v)] >= 0) visit(par[size_t(v)]);
            newIdx[size_t(v)] = int(order.size());
            order.push_back(v);
        };
        for (size_t i = 0; i < in.numPoints(); ++i) visit(int(i));
        if (order.empty()) { c.error("Input has no curves to turn into bones."); return; }
        for (int v : order) o.addPoint(in.P[size_t(v)]);
        ensureSkelAttribs(o);
        Attrib& pa = *o.findAttrib(AttribClass::Point, "parent");
        for (size_t i = 0; i < order.size(); ++i) {
            int p = par[size_t(order[i])];
            int np = p >= 0 ? newIdx[size_t(p)] : -1;
            pa.setF(i, float(np));
            if (np >= 0) o.addPrim({np, int(i)}, false);
        }
        return;
    }
    const int bones = std::clamp(c.i("bones"), 1, 200);
    const float len = std::max(1e-4f, c.f("length"));
    Vec3 dir = normalize(c.v("dir"));
    if (dot(dir, dir) < 0.5f) dir = Vec3(0, 1, 0);
    const Vec3 origin = c.v("origin");
    for (int i = 0; i <= bones; ++i) o.addPoint(origin + dir * (len * float(i)));
    ensureSkelAttribs(o);
    Attrib& pa = *o.findAttrib(AttribClass::Point, "parent");
    for (int i = 0; i <= bones; ++i) {
        pa.setF(size_t(i), float(i - 1));
        if (i > 0) o.addPrim({i - 1, i}, false);
    }
}

// ------------------------------------------------------------------ FK pose
void cookFkPose(CookContext& c) {
    const Geometry& in = c.input(0);
    std::string err;
    auto par = parentsOf(in, &err);
    if (par.empty() && in.numPoints()) { c.error(err); return; }
    std::vector<R3> local(in.numPoints());
    const Attrib* rot = in.findAttrib(AttribClass::Point, "rot");
    for (size_t j = 0; j < in.numPoints(); ++j) if (rot && rot->tuple == 3) local[j] = euler(rot->getV(j));
    for (int s = 1; s <= 4; ++s) {
        std::string js = "joint" + std::to_string(s), rs = "rot" + std::to_string(s);
        int j = c.i(js.c_str());
        Vec3 r = c.v(rs.c_str());
        if (j < 0 || j >= int(in.numPoints()) || (r.x == 0 && r.y == 0 && r.z == 0)) continue;
        local[size_t(j)] = local[size_t(j)] * euler(r);
    }
    solveFK(in, par, local, c.out);
}

// ------------------------------------------------------------------ IK chain
void cookIk(CookContext& c) {
    const Geometry& in = c.input(0);
    std::string err;
    auto par = parentsOf(in, &err);
    if (par.empty()) { c.error(in.numPoints() ? err : "Input has no joints."); return; }
    Geometry o = in;
    ensureSkelAttribs(o);
    int tip = c.i("tip");
    if (tip < 0) tip = int(in.numPoints()) - 1;
    if (tip >= int(in.numPoints())) { c.error("Tip joint does not exist."); return; }
    std::vector<int> chain{tip};
    const int nb = std::max(1, c.i("bones"));
    while (int(chain.size()) <= nb && par[size_t(chain.back())] >= 0) chain.push_back(par[size_t(chain.back())]);
    std::reverse(chain.begin(), chain.end()); // root .. tip
    if (chain.size() < 2) { c.error("The tip joint has no parent to bend."); return; }
    const size_t n = chain.size();
    std::vector<Vec3> p(n), old(n);
    std::vector<float> len(n, 0);
    for (size_t i = 0; i < n; ++i) { p[i] = old[i] = o.P[size_t(chain[i])]; if (i) len[i] = length(p[i] - p[i - 1]); }
    float total = 0;
    for (size_t i = 1; i < n; ++i) total += len[i];
    const Vec3 root = p[0];
    Vec3 target = c.v("target");
    const float blend = std::clamp(c.f("blend"), 0.f, 1.f);
    if (length(target - root) >= total) {
        Vec3 d = normalize(target - root);
        for (size_t i = 1; i < n; ++i) p[i] = p[i - 1] + d * len[i];
    } else {
        const int iters = std::clamp(c.i("iterations"), 1, 200);
        for (int it = 0; it < iters; ++it) {
            p[n - 1] = target;
            for (size_t i = n - 1; i-- > 0;) p[i] = p[i + 1] + normalize(p[i] - p[i + 1]) * len[i + 1];
            p[0] = root;
            for (size_t i = 1; i < n; ++i) p[i] = p[i - 1] + normalize(p[i] - p[i - 1]) * len[i];
            if (length(p[n - 1] - target) < 1e-5f * std::max(total, 1e-3f)) break;
        }
        // pole vector: swing the middle joints about the root-target axis toward the pole
        if (c.b("usepole") && n > 2) {
            Vec3 axis = normalize(target - root);
            Vec3 pole = c.v("pole") - root;
            Vec3 pp = pole - axis * dot(pole, axis);
            if (dot(pp, pp) > 1e-12f) {
                pp = normalize(pp);
                for (size_t i = 1; i + 1 < n; ++i) {
                    Vec3 v = p[i] - root;
                    Vec3 vp = v - axis * dot(v, axis);
                    float r = length(vp);
                    if (r < 1e-7f) continue;
                    p[i] = root + axis * dot(v, axis) + pp * r;
                }
            }
        }
    }
    for (size_t i = 0; i < n; ++i) p[i] = lerp(old[i], p[i], blend);
    // world rotations of the chain joints: align each bone's old direction with its new one
    const size_t np = o.numPoints();
    std::vector<R3> Rold(np), Rnew(np);
    std::vector<Vec3> Pold = o.P;
    for (size_t j = 0; j < np; ++j) Rold[j] = Rnew[j] = getR(o, j);
    std::vector<char> inChain(np, 0);
    for (size_t i = 0; i + 1 < n; ++i) {
        size_t j = size_t(chain[i]);
        inChain[j] = 1;
        Rnew[j] = alignRot(old[i + 1] - old[i], p[i + 1] - p[i]) * Rold[j];
        o.P[j] = p[i];
    }
    // tip keeps its orientation relative to its parent; everything else follows its parent rigidly
    for (size_t j = 0; j < np; ++j) {
        int pj = par[j];
        if (inChain[j] || pj < 0) continue;
        R3 delta = Rnew[size_t(pj)] * Rold[size_t(pj)].transposed();
        Rnew[j] = delta * Rold[j];
        o.P[j] = o.P[size_t(pj)] + delta * (Pold[j] - Pold[size_t(pj)]);
    }
    for (size_t j = 0; j < np; ++j) setR(o, j, Rnew[j]);
    c.out = std::move(o);
}

// ------------------------------------------------------------------ capture (skin weights)
float segDist(const Vec3& p, const Vec3& a, const Vec3& b) {
    Vec3 ab = b - a;
    float t = dot(ab, ab) > 0 ? std::clamp(dot(p - a, ab) / dot(ab, ab), 0.f, 1.f) : 0.f;
    return length(p - (a + ab * t));
}

void cookCapture(CookContext& c) {
    c.out = c.input(0);
    if (!c.hasInput(1)) { c.error("Connect the rest skeleton to input 2."); return; }
    const Geometry& sk = c.input(1);
    std::string err;
    auto par = parentsOf(sk, &err);
    if (par.empty()) { c.error(err.empty() ? "The skeleton has no joints." : err); return; }
    // bones: (driving joint, child)
    std::vector<std::pair<int, int>> bones;
    for (size_t j = 0; j < sk.numPoints(); ++j) if (par[j] >= 0) bones.push_back({par[j], int(j)});
    if (bones.empty()) { c.error("The skeleton has no bones."); return; }
    const float power = std::max(0.5f, c.f("falloff"));
    const int maxInf = std::clamp(c.i("influences"), 1, 4);
    Geometry& g = c.out;
    Attrib* bw[4]; Attrib* bi[4];
    for (int k = 0; k < 4; ++k) {
        bw[k] = &g.addAttrib(AttribClass::Point, "bw" + std::to_string(k), 1);
        bi[k] = &g.addAttrib(AttribClass::Point, "bi" + std::to_string(k), 1);
    }
    g.syncAttribSizes();
    Vec3 mn, mx;
    sk.bounds(mn, mx);
    const float eps = std::max(1e-4f, length(mx - mn) * 1e-3f);
    std::vector<float> best(sk.numPoints());
    for (size_t i = 0; i < g.numPoints(); ++i) {
        std::fill(best.begin(), best.end(), 0.f);
        for (auto& b : bones) {
            float d = segDist(g.P[i], sk.P[size_t(b.first)], sk.P[size_t(b.second)]);
            float w = 1.f / std::pow(d + eps, power);
            best[size_t(b.first)] = std::max(best[size_t(b.first)], w);
        }
        std::vector<int> idx(sk.numPoints());
        for (size_t k = 0; k < idx.size(); ++k) idx[k] = int(k);
        std::partial_sort(idx.begin(), idx.begin() + std::min<size_t>(size_t(maxInf), idx.size()), idx.end(), [&](int a, int b) { return best[size_t(a)] > best[size_t(b)]; });
        float sum = 0;
        int m = std::min(maxInf, int(idx.size()));
        for (int k = 0; k < m; ++k) sum += best[size_t(idx[size_t(k)])];
        for (int k = 0; k < 4; ++k) {
            if (k < m && sum > 0) { bw[k]->setF(i, best[size_t(idx[size_t(k)])] / sum); bi[k]->setF(i, float(idx[size_t(k)])); }
            else { bw[k]->setF(i, 0); bi[k]->setF(i, 0); }
        }
    }
}

// ------------------------------------------------------------------ bone deform (linear blend skinning)
void cookBoneDeform(CookContext& c) {
    c.out = c.input(0);
    if (!c.hasInput(1) || !c.hasInput(2)) { c.error("Connect the rest skeleton (input 2) and the posed skeleton (input 3)."); return; }
    const Geometry& rest = c.input(1);
    const Geometry& pose = c.input(2);
    if (rest.numPoints() != pose.numPoints()) { c.error("Rest and posed skeletons have different joint counts."); return; }
    Geometry& g = c.out;
    const Attrib* bw[4]; const Attrib* bi[4];
    for (int k = 0; k < 4; ++k) {
        bw[k] = g.findAttrib(AttribClass::Point, "bw" + std::to_string(k));
        bi[k] = g.findAttrib(AttribClass::Point, "bi" + std::to_string(k));
        if (!bw[k] || !bi[k]) { c.error("Input has no skin weights. Add a Capture node."); return; }
    }
    const size_t nj = rest.numPoints();
    std::vector<R3> R(nj);
    for (size_t j = 0; j < nj; ++j) R[j] = getR(pose, j);
    Attrib* N = g.findAttrib(AttribClass::Point, "N");
    if (N && N->tuple != 3) N = nullptr;
    for (size_t i = 0; i < g.numPoints(); ++i) {
        Vec3 p(0), n(0);
        float wsum = 0;
        const Vec3 p0 = g.P[i];
        for (int k = 0; k < 4; ++k) {
            float w = bw[k]->getF(i);
            if (w <= 0) continue;
            int j = int(std::lround(bi[k]->getF(i)));
            if (j < 0 || size_t(j) >= nj) continue;
            p += (pose.P[size_t(j)] + R[size_t(j)] * (p0 - rest.P[size_t(j)])) * w;
            if (N) n += R[size_t(j)] * N->getV(i) * w;
            wsum += w;
        }
        if (wsum <= 0) continue;
        g.P[i] = p / wsum;
        if (N) N->setV(i, normalize(n));
    }
}

// ------------------------------------------------------------------ blend shapes
void cookBlendShapes(CookContext& c) {
    c.out = c.input(0);
    Geometry& g = c.out;
    const std::vector<Vec3> base = g.P;
    bool any = false;
    for (int k = 1; k <= 4; ++k) {
        if (!c.hasInput(k)) continue;
        const Geometry& t = c.input(k);
        if (t.numPoints() != base.size()) { c.error("Target " + std::to_string(k) + " has a different point count."); return; }
        std::string wn = "w" + std::to_string(k);
        float w = c.f(wn.c_str());
        if (w == 0) continue;
        any = true;
        for (size_t i = 0; i < base.size(); ++i) g.P[i] += (t.P[i] - base[i]) * w;
    }
    if (any && g.findAttrib(AttribClass::Point, "N")) g.computePointNormals();
}

// ------------------------------------------------------------------ BVH motion capture
struct BvhJoint { std::string name; int parent = -1; Vec3 offset; std::vector<int> channels; int chanStart = 0; bool endSite = false; };
struct BvhData {
    std::vector<BvhJoint> joints;
    int numChannels = 0;
    float frameTime = 1.f / 30.f;
    std::vector<std::vector<float>> frames;
};

// channel codes: 0-2 position X Y Z, 3-5 rotation X Y Z
bool parseBvh(const std::string& text, BvhData& d, std::string& err) {
    std::istringstream in(text);
    std::string tok;
    std::vector<int> stack;
    auto expect = [&](const char* w) { std::string t; in >> t; if (t != w) { err = std::string("Expected ") + w + ", found '" + t + "'"; return false; } return true; };
    if (!expect("HIERARCHY")) return false;
    int pending = -1;
    while (in >> tok) {
        if (tok == "ROOT" || tok == "JOINT") {
            BvhJoint j;
            in >> j.name;
            j.parent = stack.empty() ? -1 : stack.back();
            d.joints.push_back(j);
            pending = int(d.joints.size()) - 1;
        } else if (tok == "End") {
            std::string site;
            in >> site;
            BvhJoint j;
            j.name = (stack.empty() ? std::string("end") : d.joints[size_t(stack.back())].name) + "_end";
            j.parent = stack.empty() ? -1 : stack.back();
            j.endSite = true;
            d.joints.push_back(j);
            pending = int(d.joints.size()) - 1;
        } else if (tok == "{") {
            if (pending < 0) { err = "Unexpected {"; return false; }
            stack.push_back(pending);
            pending = -1;
        } else if (tok == "}") {
            if (stack.empty()) { err = "Unbalanced }"; return false; }
            stack.pop_back();
        } else if (tok == "OFFSET") {
            if (stack.empty()) { err = "OFFSET outside a joint"; return false; }
            Vec3& o = d.joints[size_t(stack.back())].offset;
            in >> o.x >> o.y >> o.z;
        } else if (tok == "CHANNELS") {
            if (stack.empty()) { err = "CHANNELS outside a joint"; return false; }
            int n = 0;
            in >> n;
            BvhJoint& j = d.joints[size_t(stack.back())];
            j.chanStart = d.numChannels;
            for (int i = 0; i < n; ++i) {
                std::string ch;
                in >> ch;
                static const char* names[6] = {"Xposition", "Yposition", "Zposition", "Xrotation", "Yrotation", "Zrotation"};
                int code = -1;
                for (int k = 0; k < 6; ++k) if (ch == names[k]) code = k;
                if (code < 0) { err = "Unknown channel " + ch; return false; }
                j.channels.push_back(code);
            }
            d.numChannels += n;
        } else if (tok == "MOTION") {
            break;
        } else { err = "Unexpected '" + tok + "'"; return false; }
    }
    if (d.joints.empty()) { err = "No joints"; return false; }
    std::string w1, w2;
    int nf = 0;
    in >> w1 >> nf;
    if (w1 != "Frames:") { err = "Missing Frames:"; return false; }
    in >> w1 >> w2 >> d.frameTime;
    if (w1 != "Frame" || w2 != "Time:" || !(d.frameTime > 0)) { err = "Missing Frame Time:"; return false; }
    for (int f = 0; f < nf; ++f) {
        std::vector<float> v(size_t(d.numChannels));
        for (auto& x : v) if (!(in >> x)) { err = "Motion data ends early (frame " + std::to_string(f) + ")"; return false; }
        d.frames.push_back(std::move(v));
    }
    if (d.frames.empty()) { err = "No motion frames"; return false; }
    return true;
}

struct BvhState { std::string path; std::string err; std::shared_ptr<BvhData> data; };

void cookBvh(CookContext& c) {
    BvhState& st = c.state<BvhState>();
    std::string path = resolvePath(c.s("file"));
    if (path.empty()) { c.error("Choose a .bvh file."); return; }
    if (st.path != path) {
        st.path = path;
        st.err.clear();
        st.data.reset();
        std::ifstream f(path, std::ios::binary);
        if (!f) st.err = "Cannot open " + path;
        else {
            std::stringstream ss;
            ss << f.rdbuf();
            auto d = std::make_shared<BvhData>();
            if (parseBvh(ss.str(), *d, st.err)) st.data = d;
        }
    }
    if (!st.data) { c.error("BVH: " + st.err); return; }
    const BvhData& d = *st.data;
    const float scale = c.f("scale");
    const bool rest = c.b("rest");
    // scene time -> BVH frame (linear interpolation of channels)
    float t = (c.frame() - float(c.i("startframe"))) / c.fps() * std::max(0.01f, c.f("speed"));
    float fi = t / d.frameTime;
    const int nf = int(d.frames.size());
    if (c.b("loop")) { fi = std::fmod(fi, float(nf)); if (fi < 0) fi += float(nf); }
    fi = std::clamp(fi, 0.f, float(nf - 1));
    int f0 = int(fi), f1 = std::min(nf - 1, f0 + 1);
    float u = fi - float(f0);
    Geometry& o = c.out;
    for (size_t i = 0; i < d.joints.size(); ++i) o.addPoint(Vec3(0));
    ensureSkelAttribs(o);
    Attrib& pa = *o.findAttrib(AttribClass::Point, "parent");
    std::vector<R3> world(d.joints.size());
    for (size_t i = 0; i < d.joints.size(); ++i) {
        const BvhJoint& j = d.joints[i];
        R3 local;
        Vec3 pos = j.offset * scale;
        if (!rest) {
            Vec3 tr = j.offset * scale;
            bool hasPos = false;
            for (size_t k = 0; k < j.channels.size(); ++k) {
                float v = d.frames[size_t(f0)][size_t(j.chanStart) + k] * (1 - u) + d.frames[size_t(f1)][size_t(j.chanStart) + k] * u;
                int code = j.channels[k];
                if (code < 3) { if (!hasPos) { tr = Vec3(0); hasPos = true; } tr[code] = v * scale; }
                else local = local * axisAngle(code == 3 ? Vec3(1, 0, 0) : code == 4 ? Vec3(0, 1, 0) : Vec3(0, 0, 1), radians(v));
            }
            if (hasPos && j.parent < 0) pos = tr;
            else if (hasPos) pos = tr;
        }
        if (j.parent < 0) { o.P[i] = pos; world[i] = local; }
        else {
            o.P[i] = o.P[size_t(j.parent)] + world[size_t(j.parent)] * pos;
            world[i] = world[size_t(j.parent)] * local;
            o.addPrim({j.parent, int(i)}, false);
        }
        pa.setF(i, float(j.parent));
        setR(o, i, world[i]);
    }
}

// ------------------------------------------------------------------ retarget
void cookRetarget(CookContext& c) {
    if (!c.hasInput(1) || !c.hasInput(2)) { c.error("Inputs: target rest skeleton, source rest skeleton, source posed skeleton."); return; }
    const Geometry& tgt = c.input(0);
    const Geometry& srest = c.input(1);
    const Geometry& spose = c.input(2);
    std::string err;
    auto tp = parentsOf(tgt, &err);
    if (tp.empty()) { c.error("Target: " + err); return; }
    auto sp = parentsOf(srest, &err);
    if (sp.empty()) { c.error("Source: " + err); return; }
    if (tp != sp || spose.numPoints() != srest.numPoints()) { c.error("Source and target skeletons need the same joints in the same order."); return; }
    // world rotations are copied joint by joint; the root's travel is scaled by the size ratio of the skeletons
    Vec3 a0, a1, b0, b1;
    tgt.bounds(a0, a1);
    srest.bounds(b0, b1);
    float ratio = length(b1 - b0) > 1e-9f ? length(a1 - a0) / length(b1 - b0) : 1.f;
    Geometry o = tgt;
    ensureSkelAttribs(o);
    std::vector<R3> world(tgt.numPoints());
    for (size_t j = 0; j < tgt.numPoints(); ++j) {
        world[j] = getR(spose, j);
        int p = tp[j];
        if (p < 0) o.P[j] = tgt.P[j] + (spose.P[j] - srest.P[j]) * ratio;
        else o.P[j] = o.P[size_t(p)] + world[size_t(p)] * (tgt.P[j] - tgt.P[size_t(p)]);
        setR(o, j, world[j]);
    }
    c.out = std::move(o);
}

} // namespace

void registerRiggingNodes(std::vector<NodeType>& types) {
    auto add = [&](NodeType t) { t.category = "Rigging"; types.push_back(std::move(t)); };
    {
        NodeType t; t.name = "skeleton"; t.label = "Skeleton";
        t.description = "A chain of joints, or skeletons made from the input's curves (shared points join chains).";
        t.minInputs = 0; t.maxInputs = 1; t.inputLabels = {"Curves (optional)"};
        t.parms = {I("bones", "Bones", 3, 1, 50), F("length", "Bone Length", 0.5f, 0.01f, 5), V("dir", "Direction", Vec3(0, 1, 0)), V("origin", "Origin", Vec3(0), -5, 5)};
        t.cook = cookSkeleton;
        add(t);
    }
    {
        NodeType t; t.name = "fkpose"; t.label = "Joint Pose (FK)";
        t.description = "Rotates joints (XYZ degrees, about each joint, in world axes at rest). Uses a rot point attribute if present, plus these slots. Rotations can be keyframed.";
        t.minInputs = 1; t.maxInputs = 1;
        for (int s = 1; s <= 4; ++s) {
            static std::string jn[5], jl[5], rn[5], rl[5];
            jn[s] = "joint" + std::to_string(s); jl[s] = "Joint " + std::to_string(s);
            rn[s] = "rot" + std::to_string(s); rl[s] = "Rotate " + std::to_string(s);
            t.parms.push_back(I(jn[s].c_str(), jl[s].c_str(), s == 1 ? 0 : -1, -1, 50, "Joint number (-1 = unused)"));
            t.parms.push_back(V(rn[s].c_str(), rl[s].c_str(), Vec3(0), -180, 180));
        }
        t.cook = cookFkPose;
        add(t);
    }
    {
        NodeType t; t.name = "ikchain"; t.label = "IK Chain";
        t.description = "Bends a chain of joints so its tip reaches the target (FABRIK), keeping bone lengths. The pole vector picks the bend direction.";
        t.minInputs = 1; t.maxInputs = 1;
        t.parms = {I("tip", "Tip Joint", -1, -1, 50, "-1 = last joint"), I("bones", "Chain Bones", 2, 1, 50),
                   V("target", "Target", Vec3(0.5f, 0.8f, 0), -5, 5), T("usepole", "Use Pole Vector", true), V("pole", "Pole Vector", Vec3(0, 0, 1), -5, 5),
                   I("iterations", "Iterations", 30, 1, 200), F("blend", "IK Blend", 1, 0, 1, "0 = input pose, 1 = solved")};
        t.cook = cookIk;
        add(t);
    }
    {
        NodeType t; t.name = "capture"; t.label = "Capture (Skin Weights)";
        t.description = "Automatic skin weights by distance to the bones of the rest skeleton (bw0-3 weights, bi0-3 joints).";
        t.minInputs = 2; t.maxInputs = 2; t.inputLabels = {"Mesh (rest pose)", "Rest skeleton"};
        t.parms = {F("falloff", "Falloff Power", 4, 0.5f, 12, "Higher = stiffer, sharper bends"), I("influences", "Max Influences", 4, 1, 4)};
        t.cook = cookCapture;
        add(t);
    }
    {
        NodeType t; t.name = "bonedeform"; t.label = "Bone Deform";
        t.description = "Linear blend skinning: moves a captured mesh with a posed skeleton.";
        t.minInputs = 3; t.maxInputs = 3; t.inputLabels = {"Captured mesh", "Rest skeleton", "Posed skeleton"};
        t.cook = cookBoneDeform;
        add(t);
    }
    {
        NodeType t; t.name = "blendshapes"; t.label = "Blend Shapes";
        t.description = "Adds weighted differences of up to four shape targets (same point count) to the base.";
        t.minInputs = 1; t.maxInputs = 5; t.inputLabels = {"Base", "Target 1", "Target 2", "Target 3", "Target 4"};
        t.parms = {F("w1", "Weight 1", 0, 0, 1), F("w2", "Weight 2", 0, 0, 1), F("w3", "Weight 3", 0, 0, 1), F("w4", "Weight 4", 0, 0, 1)};
        t.cook = cookBlendShapes;
        add(t);
    }
    {
        NodeType t; t.name = "bvhimport"; t.label = "Motion Capture (BVH)";
        t.description = "Loads a BVH motion capture file as an animated skeleton.";
        t.minInputs = 0; t.maxInputs = 0;
        t.timeDependent = true;
        t.parms = {FileP("file", "BVH File", "Motion capture file (.bvh); $HIP = scene folder"), F("scale", "Scale", 0.01f, 0.001f, 1, "BVH files are often in cm: 0.01 gives metres"),
                   I("startframe", "Start Frame", 1, -100, 1000), F("speed", "Speed", 1, 0.1f, 4), T("loop", "Loop", true), T("rest", "Rest Pose", false)};
        t.cook = cookBvh;
        add(t);
    }
    {
        NodeType t; t.name = "retarget"; t.label = "Retarget";
        t.description = "Copies the motion of a posed skeleton onto another skeleton with the same joints (rotations joint by joint, root travel scaled by size).";
        t.minInputs = 3; t.maxInputs = 3; t.inputLabels = {"Target rest skeleton", "Source rest skeleton", "Source posed skeleton"};
        t.cook = cookRetarget;
        add(t);
    }
    {
        NodeType t; t.name = "timeshift"; t.label = "Time Shift";
        t.description = "Evaluates the input at another frame (e.g. $F - 10, or a fixed frame to freeze it).";
        t.category = "Utility";
        t.minInputs = 1; t.maxInputs = 1;
        t.timeDependent = true;
        ParmDef fr = F("frame", "Frame", 1, -100, 500);
        fr.help = "Defaults to the current frame; add an expression such as $F - 12";
        t.parms = {fr, F("offset", "Offset", 0, -100, 100), T("usecurrent", "Use Current Frame + Offset", true)};
        t.cook = [](CookContext& c) {
            float f = c.b("usecurrent") ? c.frame() + c.f("offset") : c.f("frame") + c.f("offset");
            if (!c.inputAtFrame(0, f, c.out)) c.error("Input failed to cook at frame " + std::to_string(f));
        };
        types.push_back(t);
    }
}

} // namespace hd

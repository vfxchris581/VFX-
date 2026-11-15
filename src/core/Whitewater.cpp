// Whitewater Solver: spray, foam and bubbles from a FLIP simulation.
//
// Follows the approach of Ihmsen et al., "Unified Spray, Foam and Bubbles for Particle-Based Fluids"
// (2012), adapted to FLIP particles:
//  * Every frame each fluid particle gets three potentials from its neighbours (within the kernel
//    radius h, weight W = 1 - r/h): trapped air (relative velocities of particles moving toward each
//    other), wave crests (curvature of a convex surface moving along its normal) and kinetic energy.
//    Each is clamped to [min, max] and mapped to [0, 1].
//  * The number of new whitewater particles is  I_ke * (K_ta * I_ta + K_wc * I_wc) * dt, rounded
//    stochastically, placed in a cylinder around the fluid particle along its velocity.
//  * Whitewater particles are classified by how many fluid particles surround them (relative to a
//    fluid particle deep inside): few -> spray (ballistic, gravity + air drag), many -> bubbles
//    (buoyancy and drag toward the local fluid velocity), in between -> foam (moves with the fluid
//    velocity and ages; it disappears when its life runs out).
// The input is the FLIP particles (P and v), read at every simulated frame.
#include "Node.h"
#include "Volume.h"
#include <algorithm>
#include <cmath>
#include <map>

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

enum WhiteType { kSpray = 0, kFoam = 1, kBubble = 2 };

// Uniform hash grid over points (counting sort; cell size = query radius).
struct PointGrid {
    float inv = 1;
    size_t mask = 0;
    std::vector<int> start, order;
    static uint32_t hash3(int a, int b, int c) { return (uint32_t(a) * 73856093u) ^ (uint32_t(b) * 19349663u) ^ (uint32_t(c) * 83492791u); }
    void build(const std::vector<Vec3>& P, float h) {
        inv = 1.f / h;
        size_t nb = 1;
        while (nb < P.size() * 2 + 2) nb <<= 1;
        mask = nb - 1;
        start.assign(nb + 1, 0);
        order.resize(P.size());
        std::vector<uint32_t> cell(P.size());
        for (size_t i = 0; i < P.size(); ++i) {
            cell[i] = hash3(int(std::floor(P[i].x * inv)), int(std::floor(P[i].y * inv)), int(std::floor(P[i].z * inv))) & uint32_t(mask);
            start[cell[i] + 1]++;
        }
        for (size_t k = 0; k < nb; ++k) start[k + 1] += start[k];
        std::vector<int> fill(start.begin(), start.end() - 1);
        for (size_t i = 0; i < P.size(); ++i) order[size_t(fill[cell[i]]++)] = int(i);
    }
    // Calls fn(j) for every point in the 27 cells around p (a superset of the points within h).
    template <class Fn> void forNear(const Vec3& p, Fn&& fn) const {
        if (order.empty()) return;
        const int a = int(std::floor(p.x * inv)), b = int(std::floor(p.y * inv)), c = int(std::floor(p.z * inv));
        size_t seen[27];
        int ns = 0;
        for (int dz = -1; dz <= 1; ++dz)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    size_t bk = hash3(a + dx, b + dy, c + dz) & mask;
                    if (start[bk] == start[bk + 1]) continue;
                    bool dup = false;
                    for (int q = 0; q < ns; ++q) dup |= seen[q] == bk;
                    if (dup) continue;
                    seen[ns++] = bk;
                    for (int k = start[bk]; k < start[bk + 1]; ++k) fn(order[size_t(k)]);
                }
    }
};

float clampMap(float v, float lo, float hi) { return hi > lo ? std::clamp((v - lo) / (hi - lo), 0.f, 1.f) : (v >= lo ? 1.f : 0.f); }

struct WhiteFrame {
    std::vector<Vec3> P, V;
    std::vector<float> life;
    std::vector<unsigned char> type;
    size_t emitted = 0; // emitted this frame (for info)
};

struct WhiteState {
    uint64_t sig = 0;
    std::map<int, WhiteFrame> frames;
    size_t total = 0;
};

struct WhiteParams {
    float h = 0.08f, dt = 1.f / 24.f;
    float taMin = 5, taMax = 20, wcMin = 2, wcMax = 8, keMin = 5, keMax = 50;
    float kTa = 200, kWc = 200;
    float life = 2, lifeVar = 0.5f;
    Vec3 gravity{0, -9.81f, 0};
    float sprayDrag = 0.2f, buoyancy = 2, bubbleDrag = 0.5f;
    float sprayFrac = 0.1f, bubbleFrac = 0.75f;
    size_t maxPts = 300000;
    uint32_t seed = 0;
};

// Advances whitewater particles over one frame using the fluid at the end of the frame, then emits
// new particles from that fluid.
void stepWhitewater(WhiteFrame& w, const std::vector<Vec3>& fp, const std::vector<Vec3>& fv, const WhiteParams& prm, int frame) {
    const size_t nf = fp.size();
    const float h = prm.h, dt = prm.dt;
    PointGrid grid;
    grid.build(fp, h);
    auto W = [&](float r) { return r < h ? 1.f - r / h : 0.f; };

    // How many fluid neighbours a particle deep inside the liquid has (90th percentile of a sample).
    float interior = 1;
    if (nf) {
        std::vector<int> counts;
        const size_t stride = std::max<size_t>(1, nf / 2000);
        for (size_t i = 0; i < nf; i += stride) {
            int cnt = 0;
            grid.forNear(fp[i], [&](int j) { cnt += length(fp[size_t(j)] - fp[i]) < h; });
            counts.push_back(cnt);
        }
        std::sort(counts.begin(), counts.end());
        interior = float(std::max(1, counts[std::min(counts.size() - 1, counts.size() * 9 / 10)]));
    }
    Vec3 bmin(1e30f), bmax(-1e30f);
    for (const Vec3& p : fp) { bmin = vmin(bmin, p); bmax = vmax(bmax, p); }
    // keep particles near the liquid; below its lowest point is the container floor (or a collider)
    bmin = bmin - Vec3(4 * h, 0.5f * h, 4 * h); bmax = bmax + Vec3(4 * h);

    // ---- advance existing whitewater (two substeps against this frame's fluid)
    const int subs = 2;
    const float sdt = dt / float(subs);
    const size_t nw = w.P.size();
    parallelFor(int(nw), [&](int b0, int b1) {
        for (int ii = b0; ii < b1; ++ii) {
            const size_t i = size_t(ii);
            for (int s = 0; s < subs; ++s) {
                float wsum = 0;
                int cnt = 0;
                Vec3 vavg(0);
                grid.forNear(w.P[i], [&](int j) {
                    float r = length(fp[size_t(j)] - w.P[i]);
                    if (r >= h) return;
                    ++cnt;
                    float k = W(r);
                    wsum += k;
                    vavg += fv[size_t(j)] * k;
                });
                if (wsum > 0) vavg = vavg / wsum;
                const float frac = float(cnt) / interior;
                if (frac < prm.sprayFrac) {
                    w.type[i] = kSpray;
                    w.V[i] = (w.V[i] + prm.gravity * sdt) * std::max(0.f, 1.f - prm.sprayDrag * sdt);
                    w.P[i] += w.V[i] * sdt;
                } else if (frac > prm.bubbleFrac) {
                    w.type[i] = kBubble;
                    w.V[i] += (prm.gravity * -prm.buoyancy + (vavg - w.V[i]) * (prm.bubbleDrag / sdt)) * sdt;
                    w.P[i] += w.V[i] * sdt;
                } else {
                    w.type[i] = kFoam;
                    w.V[i] = vavg;
                    w.P[i] += vavg * sdt;
                    w.life[i] -= sdt;
                }
            }
        }
    });
    // ---- remove dead or escaped particles (order kept, so results are deterministic)
    size_t keep = 0;
    for (size_t i = 0; i < nw; ++i) {
        const Vec3& p = w.P[i];
        bool alive = w.life[i] > 0 && p.x >= bmin.x && p.y >= bmin.y && p.z >= bmin.z && p.x <= bmax.x && p.y <= bmax.y && p.z <= bmax.z &&
                     std::isfinite(p.x + p.y + p.z);
        if (!alive) continue;
        w.P[keep] = w.P[i]; w.V[keep] = w.V[i]; w.life[keep] = w.life[i]; w.type[keep] = w.type[i];
        ++keep;
    }
    w.P.resize(keep); w.V.resize(keep); w.life.resize(keep); w.type.resize(keep);

    // ---- emission potentials
    std::vector<Vec3> nrm(nf, Vec3(0));
    parallelFor(int(nf), [&](int b0, int b1) {
        for (int ii = b0; ii < b1; ++ii) {
            const size_t i = size_t(ii);
            Vec3 g(0);
            grid.forNear(fp[i], [&](int j) {
                Vec3 d = fp[i] - fp[size_t(j)];
                float r = length(d);
                if (r >= h || r < 1e-9f) return;
                g += d / r * W(r);
            });
            float l = length(g);
            nrm[i] = l > 1e-6f ? g / l : Vec3(0); // points out of the liquid
        }
    });
    std::vector<int> spawn(nf, 0);
    parallelFor(int(nf), [&](int b0, int b1) {
        for (int ii = b0; ii < b1; ++ii) {
            const size_t i = size_t(ii);
            const float ke = 0.5f * dot(fv[i], fv[i]);
            const float Ike = clampMap(ke, prm.keMin, prm.keMax);
            if (Ike <= 0) continue;
            const float speed = std::sqrt(2 * ke);
            const Vec3 vhat = fv[i] / std::max(speed, 1e-9f);
            float ta = 0, wc = 0;
            grid.forNear(fp[i], [&](int j) {
                if (size_t(j) == i) return;
                Vec3 d = fp[i] - fp[size_t(j)];
                float r = length(d);
                if (r >= h || r < 1e-9f) return;
                Vec3 dh = d / r;
                float k = W(r);
                Vec3 vij = fv[i] - fv[size_t(j)];
                float vl = length(vij);
                if (vl > 1e-9f) ta += vl * (1 - dot(vij / vl, dh)) * k;
                if (dot(-dh, nrm[i]) < 0) wc += (1 - dot(nrm[i], nrm[size_t(j)])) * k; // convex neighbourhood
            });
            if (dot(vhat, nrm[i]) < 0.6f) wc = 0; // only crests moving outward
            const float n = Ike * (prm.kTa * clampMap(ta, prm.taMin, prm.taMax) + prm.kWc * clampMap(wc, prm.wcMin, prm.wcMax)) * dt;
            const uint32_t hs = uint32_t(i) * 2654435761u ^ (uint32_t(frame) * 40503u + prm.seed * 9973u);
            int count = int(n);
            if (hashRandom(hs) < n - float(count)) ++count;
            spawn[i] = count;
        }
    });
    size_t total = 0;
    for (int s : spawn) total += size_t(s);
    const size_t room = prm.maxPts > w.P.size() ? prm.maxPts - w.P.size() : 0;
    const float thin = total > room ? float(room) / float(total) : 1.f; // keep under the cap evenly
    w.emitted = 0;
    for (size_t i = 0; i < nf; ++i) {
        if (!spawn[i]) continue;
        const float speed = length(fv[i]);
        const Vec3 vhat = speed > 1e-9f ? fv[i] / speed : Vec3(0, 1, 0);
        Vec3 e1 = std::fabs(vhat.y) < 0.9f ? normalize(cross(vhat, Vec3(0, 1, 0))) : normalize(cross(vhat, Vec3(1, 0, 0)));
        Vec3 e2 = cross(vhat, e1);
        for (int m = 0; m < spawn[i]; ++m) {
            const uint32_t hs = (uint32_t(i) * 7919u + uint32_t(m) * 104729u) ^ (uint32_t(frame) * 2246822519u + prm.seed);
            if (thin < 1 && hashRandom(hs + 5u) >= thin) continue;
            const float r = 0.5f * h * std::sqrt(hashRandom(hs)), th = 6.2831853f * hashRandom(hs + 1u), along = hashRandom(hs + 2u) * speed * dt;
            const Vec3 radial = e1 * std::cos(th) + e2 * std::sin(th);
            w.P.push_back(fp[i] + radial * r + vhat * along);
            w.V.push_back(fv[i] + radial * (r / dt) * 0.25f);
            w.life.push_back(std::max(0.05f, prm.life + prm.lifeVar * (2 * hashRandom(hs + 3u) - 1)));
            w.type.push_back(kFoam);
            ++w.emitted;
        }
    }
}

void cookWhitewater(CookContext& c) {
    WhiteState& st = c.state<WhiteState>();
    const uint64_t sig = c.setupSignature();
    if (st.sig != sig) { st = WhiteState(); st.sig = sig; }
    Node& node = c.node();
    node.simCacheStart = 0; node.simCacheEnd = -1;
    const int start = c.i("startframe");
    const int frame = int(std::floor(c.frame() + 0.5f));

    const Geometry& in0 = c.input(0);
    float h = c.f("radius");
    float autoSize = 0.01f;
    if (const Attrib* ps = in0.findAttrib(AttribClass::Point, "pscale"); ps && in0.numPoints()) {
        const float dx = ps->getF(0) / 0.35f; // FLIP writes pscale = 0.35 * voxel size
        if (h <= 0) h = dx;
        autoSize = dx * 0.12f;
    }
    if (in0.numPoints() == 0 && st.frames.empty()) { c.warning("No fluid particles in the input; connect a FLIP Solver."); return; }
    if (h <= 0) {
        // no pscale: twice the mean particle spacing (FLIP seeds two particles per voxel along each axis)
        Vec3 mn, mx;
        in0.bounds(mn, mx);
        Vec3 ext = vmax(mx - mn, Vec3(1e-3f));
        const float spacing = std::cbrt(ext.x * ext.y * ext.z / float(std::max<size_t>(1, in0.numPoints())));
        h = 2 * spacing;
        autoSize = h * 0.12f;
    }
    const float psize = c.f("particlesize") > 0 ? c.f("particlesize") : autoSize;

    auto output = [&](const WhiteFrame& f) {
        Geometry& o = c.out;
        o = Geometry();
        o.P = f.P;
        Attrib& v = o.addAttrib(AttribClass::Point, "v", 3);
        Attrib& life = o.addAttrib(AttribClass::Point, "life", 1);
        Attrib& type = o.addAttrib(AttribClass::Point, "wwtype", 1);
        Attrib& ps = o.addAttrib(AttribClass::Point, "pscale", 1);
        Attrib& cd = o.addAttrib(AttribClass::Point, "Cd", 3);
        o.syncAttribSizes();
        const Vec3 cols[3] = {c.v("spraycolor"), c.v("foamcolor"), c.v("bubblecolor")};
        for (size_t i = 0; i < f.P.size(); ++i) {
            v.setV(i, f.V[i]);
            life.setF(i, f.life[i]);
            type.setF(i, float(f.type[i]));
            ps.setF(i, psize);
            cd.setV(i, cols[f.type[i] < 3 ? f.type[i] : 1]);
        }
        if (!c.b("keepfluid")) return;
        // optionally pass the fluid through after the whitewater points
        o.merge(in0);
    };
    if (frame < start) { output(WhiteFrame()); return; }
    if (frame - start > 100000) { c.error("Frame is too far from the start frame."); return; }

    WhiteParams prm;
    prm.h = h;
    prm.dt = 1.f / c.fps();
    prm.taMin = c.f("tamin"); prm.taMax = c.f("tamax");
    prm.wcMin = c.f("wcmin"); prm.wcMax = c.f("wcmax");
    prm.keMin = c.f("kemin"); prm.keMax = c.f("kemax");
    prm.kTa = std::max(0.f, c.f("trappedair")); prm.kWc = std::max(0.f, c.f("wavecrest"));
    prm.life = std::max(0.01f, c.f("life")); prm.lifeVar = std::max(0.f, c.f("lifevar"));
    prm.gravity = c.v("gravity");
    prm.sprayDrag = std::max(0.f, c.f("spraydrag"));
    prm.buoyancy = c.f("buoyancy");
    prm.bubbleDrag = std::clamp(c.f("bubbledrag"), 0.f, 1.f);
    prm.sprayFrac = std::clamp(c.f("sprayfrac"), 0.f, 1.f);
    prm.bubbleFrac = std::clamp(c.f("bubblefrac"), prm.sprayFrac, 1.f);
    prm.maxPts = size_t(std::max(0, c.i("maxpts")));
    prm.seed = uint32_t(c.i("seed"));

    int have = start - 1;
    { auto it = st.frames.upper_bound(frame); if (it != st.frames.begin()) { --it; have = it->first; } }
    WhiteFrame cur;
    if (have >= start) cur = st.frames[have];
    std::string inputErr;
    for (int k = std::max(have + 1, start); k <= frame; ++k) {
        c.setEvalFrame(float(k));
        Geometry fluid;
        if (k == frame) fluid = in0;
        else if (!c.inputAtFrame(0, float(k), fluid)) { inputErr = "Could not read the fluid at frame " + std::to_string(k) + "."; break; }
        const Attrib* va = fluid.findAttrib(AttribClass::Point, "v");
        std::vector<Vec3> fv(fluid.numPoints(), Vec3(0));
        if (va && va->tuple == 3) for (size_t i = 0; i < fv.size(); ++i) fv[i] = va->getV(i);
        stepWhitewater(cur, fluid.P, fv, prm, k);
        auto old = st.frames.find(k);
        if (old != st.frames.end()) st.total -= old->second.P.size();
        st.frames[k] = cur;
        st.total += cur.P.size();
    }
    c.clearEvalFrame();
    if (!inputErr.empty()) { c.error(inputErr); return; }
    if (!in0.findAttrib(AttribClass::Point, "v")) c.warning("The input has no v (velocity) attribute; connect a FLIP Solver.");
    const size_t cap = 40u * 1000u * 1000u;
    while (st.total > cap && st.frames.size() > 1 && st.frames.begin()->first != frame) {
        st.total -= st.frames.begin()->second.P.size();
        st.frames.erase(st.frames.begin());
    }
    output(st.frames[frame]);
    int lo = frame, hi = frame;
    while (st.frames.count(lo - 1)) --lo;
    while (st.frames.count(hi + 1)) ++hi;
    node.simCacheStart = lo;
    node.simCacheEnd = hi;
}

} // namespace

void registerWhitewaterNodes(std::vector<NodeType>& types) {
    NodeType t; t.name = "whitewater"; t.label = "Whitewater Solver"; t.category = "Fluids";
    t.description = "Spray, foam and bubbles generated from a FLIP simulation (trapped air, wave crests, kinetic energy).";
    t.minInputs = 1; t.maxInputs = 1;
    t.inputLabels = {"FLIP particles (P and v)"};
    t.timeDependent = true;
    t.parms = {
        I("startframe", "Start Frame", 1, 1, 240),
        F("radius", "Kernel Radius", 0, 0, 1, "Neighbourhood size. 0 = the FLIP voxel size (from pscale), or twice the particle spacing"),
        F("trappedair", "Trapped Air Emission", 200, 0, 2000, "New particles per second at full trapped-air and kinetic-energy potential"),
        F("wavecrest", "Wave Crest Emission", 200, 0, 2000, "New particles per second at full wave-crest and kinetic-energy potential"),
        F("tamin", "Trapped Air Min", 5, 0, 100), F("tamax", "Trapped Air Max", 20, 0, 200),
        F("wcmin", "Wave Crest Min", 2, 0, 50), F("wcmax", "Wave Crest Max", 8, 0, 100),
        F("kemin", "Kinetic Energy Min", 5, 0, 100, "Slower liquid (0.5 v^2 below this) emits nothing"),
        F("kemax", "Kinetic Energy Max", 50, 0, 500),
        F("life", "Foam Life (s)", 2, 0.05f, 20), F("lifevar", "Life Variance (s)", 0.5f, 0, 10),
        V("gravity", "Gravity", Vec3(0, -9.81f, 0), -20, 20),
        F("spraydrag", "Spray Air Drag", 0.2f, 0, 5),
        F("buoyancy", "Bubble Buoyancy", 2, 0, 10, "Times gravity, upward"),
        F("bubbledrag", "Bubble Drag", 0.5f, 0, 1, "How quickly bubbles take the surrounding liquid's velocity"),
        F("sprayfrac", "Spray Below", 0.1f, 0, 1, "Fraction of an interior particle's fluid neighbours below which a particle is spray"),
        F("bubblefrac", "Bubbles Above", 0.75f, 0, 1, "Fraction above which a particle is a bubble (between: foam)"),
        I("maxpts", "Max Particles", 300000, 0, 5000000),
        I("seed", "Seed", 0, 0, 1000),
        F("particlesize", "Particle Size (pscale)", 0, 0, 0.1f, "0 = 12% of the FLIP voxel size"),
        V("spraycolor", "Spray Color", Vec3(0.85f, 0.92f, 1.f), 0, 1),
        V("foamcolor", "Foam Color", Vec3(0.97f, 0.97f, 0.97f), 0, 1),
        V("bubblecolor", "Bubble Color", Vec3(0.55f, 0.75f, 0.9f), 0, 1),
        T("keepfluid", "Output Fluid Too", false, "Append the FLIP particles after the whitewater points"),
    };
    for (auto& p : t.parms) if (p.name == "spraycolor" || p.name == "foamcolor" || p.name == "bubblecolor") p.help = "color";
    t.cook = cookWhitewater;
    types.push_back(t);
}

} // namespace hd

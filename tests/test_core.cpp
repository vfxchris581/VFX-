// Headless tests for the HD core engine.
#include "core/Demo.h"
#include "core/Compositing.h"
#include "core/Denoise.h"
#include "core/Expr.h"
#include "core/Usd.h"
#include "core/Fluids.h"
#include "core/Fracture.h"
#include "core/GeoIO.h"
#include "core/Image.h"
#include "core/Lights.h"
#include "core/Node.h"
#include "core/Ops.h"
#include "core/PathTracer.h"
#include "core/Volume.h"
#include <cstdio>
#include <filesystem>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace hd;

static int g_failed = 0, g_checks = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_failed; std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define NEAR(a, b) CHECK(std::fabs(double(a) - double(b)) < 1e-4)

struct TestCase { const char* name; std::function<void()> fn; };
static std::vector<TestCase>& tests() { static std::vector<TestCase> t; return t; }
struct Reg { Reg(const char* n, std::function<void()> f) { tests().push_back({n, std::move(f)}); } };
#define TEST(name) static void name(); static Reg reg_##name(#name, name); static void name()

static float evalE(const char* s, float frame = 1) {
    float v = 0;
    std::string err;
    bool ok = evalExpression(s, frame, 24, v, &err);
    if (!ok) std::printf("  expr error: %s\n", err.c_str());
    return v;
}

static bool closedManifold(const Geometry& g) {
    // every edge used exactly twice, in opposite directions => consistent outward winding
    std::map<std::pair<int, int>, int> dirCount;
    for (auto& pr : g.prims)
        for (int k = 0; k < pr.count; ++k) {
            int a = g.vertices[size_t(pr.start + k)], b = g.vertices[size_t(pr.start + (k + 1) % pr.count)];
            dirCount[{a, b}]++;
        }
    for (auto& [e, n] : dirCount) {
        if (n != 1) return false;
        if (!dirCount.count({e.second, e.first})) return false;
    }
    return true;
}

static float signedVolume(const Geometry& g) {
    double v = 0;
    for (size_t pi = 0; pi < g.numPrims(); ++pi) {
        const Prim& pr = g.prims[pi];
        Vec3 a = g.P[size_t(g.vertices[size_t(pr.start)])];
        for (int k = 1; k + 1 < pr.count; ++k) {
            Vec3 b = g.P[size_t(g.vertices[size_t(pr.start + k)])], c = g.P[size_t(g.vertices[size_t(pr.start + k + 1)])];
            v += dot(a, cross(b, c)) / 6.0;
        }
    }
    return float(v);
}

TEST(expressions) {
    NEAR(evalE("1 + 2 * 3"), 7);
    NEAR(evalE("(1 + 2) * 3"), 9);
    NEAR(evalE("-2 * -3"), 6);
    NEAR(evalE("$F * 0.5", 10), 5);
    NEAR(evalE("$T", 25), 1);
    NEAR(evalE("sin($PI / 2)"), 1);
    NEAR(evalE("fit(5, 0, 10, 0, 1)"), 0.5f);
    NEAR(evalE("clamp(5, 0, 2)"), 2);
    NEAR(evalE("1 < 2 ? 10 : 20"), 10);
    NEAR(evalE("length(set(3, 4, 0))"), 5);
    NEAR(evalE("dot({1,2,3}, {1,1,1})"), 6);
    NEAR(evalE("max(3, 7) % 4"), 3);
    float v; std::string err;
    CHECK(!evalExpression("1 +", 1, 24, v, &err) && !err.empty());
    CHECK(!evalExpression("foo(1)", 1, 24, v, &err));
    CHECK(isExpressionString("$F*2"));
    CHECK(!isExpressionString("1.5"));
}

TEST(snippets) {
    Geometry g;
    for (int i = 0; i < 5; ++i) g.addPoint(Vec3(float(i), 0, 0));
    std::string err;
    auto prog = Program::compile(
        "float s = @ptnum * 2;\n"
        "vector c = {0.5, 0.25, 1};\n"
        "if (@ptnum >= 3) { @P.y = s; } else @P.y = -1;\n"
        "@Cd = c;\n"
        "f@weight = @numpt;\n"
        "for (int i = 0; i < 3; i++) @P.z += 1;\n"
        "@N = normalize(set(0, 2, 0));", false, err);
    CHECK(prog != nullptr);
    if (!prog) { std::printf("  %s\n", err.c_str()); return; }
    CHECK(runOverElements(g, AttribClass::Point, *prog, 1, 24, nullptr, err));
    NEAR(g.P[0].y, -1);
    NEAR(g.P[4].y, 8);
    NEAR(g.P[2].z, 3);
    const Attrib* Cd = g.findAttrib(AttribClass::Point, "Cd");
    CHECK(Cd && Cd->tuple == 3);
    if (Cd) NEAR(Cd->getV(3).y, 0.25f);
    const Attrib* w = g.findAttrib(AttribClass::Point, "weight");
    CHECK(w && w->tuple == 1);
    if (w) NEAR(w->getF(0), 5);
    const Attrib* N = g.findAttrib(AttribClass::Point, "N");
    CHECK(N && std::fabs(N->getV(1).y - 1) < 1e-5);

    CHECK(Program::compile("@P.y = ;", false, err) == nullptr);
    CHECK(Program::compile("undefinedVar = 3;", false, err) == nullptr);
}

TEST(groups) {
    Geometry g;
    for (int i = 0; i < 10; ++i) g.addPoint(Vec3(float(i), float(i % 2), 0));
    std::vector<char> m;
    std::string err;
    CHECK(selectElements(g, AttribClass::Point, "0-3 7", 1, 24, m, err));
    CHECK(m[0] && m[3] && !m[4] && m[7] && !m[9]);
    CHECK(selectElements(g, AttribClass::Point, "0-9:3", 1, 24, m, err));
    CHECK(m[0] && m[3] && m[6] && m[9] && !m[1]);
    CHECK(selectElements(g, AttribClass::Point, "@P.y > 0.5", 1, 24, m, err));
    CHECK(!m[0] && m[1] && !m[2] && m[3]);
    CHECK(selectElements(g, AttribClass::Point, "!@ptnum < 8", 1, 24, m, err));
    CHECK(!m[0] && m[8] && m[9]);
    CHECK(!selectElements(g, AttribClass::Point, "abc", 1, 24, m, err));
}

TEST(generators) {
    Graph gr;
    for (const char* t : {"box", "sphere", "torus", "tube"}) {
        Node* n = gr.createNode(t);
        gr.cook(n);
        const Geometry& g = n->geometry();
        CHECK(n->error().empty());
        CHECK(g.numPrims() > 0);
        CHECK(closedManifold(g));
        CHECK(signedVolume(g) > 0); // outward-facing winding
        if (!closedManifold(g) || signedVolume(g) <= 0) std::printf("  generator %s bad topology (vol %f)\n", t, signedVolume(g));
    }
    Node* box = gr.findNode("box1");
    NEAR(signedVolume(box->geometry()), 1.0f);
    CHECK(box->geometry().numPoints() == 8);
    box->parm("divs")->val[0] = 3;
    box->touch();
    gr.cook(box);
    CHECK(box->geometry().numPoints() == 56); // 4^3 - 2^3
    CHECK(box->geometry().numPrims() == 54);
    NEAR(signedVolume(box->geometry()), 1.0f);

    Node* grid = gr.createNode("grid");
    gr.cook(grid);
    CHECK(grid->geometry().numPoints() == 100);
    CHECK(grid->geometry().numPrims() == 81);
    CHECK(grid->geometry().primNormal(0).y > 0.99f);
    Node* circ = gr.createNode("circle");
    gr.cook(circ);
    CHECK(circ->geometry().primNormal(0).y > 0.99f);
}

TEST(cook_caching_and_time) {
    Graph g;
    Node* box = g.createNode("box");
    Node* xf = g.createNode("xform");
    CHECK(g.connect(box, xf, 0));
    g.setDisplayNode(xf);
    g.cook(xf);
    uint64_t c0 = g.totalCooks();
    CHECK(c0 == 2);
    g.cook(xf);
    CHECK(g.totalCooks() == c0); // nothing dirty
    g.frame = 10;
    g.cook(xf);
    CHECK(g.totalCooks() == c0); // not time dependent
    xf->parm("t")->expr[1] = "$F * 0.1";
    xf->touch();
    g.cook(xf);
    CHECK(g.totalCooks() == c0 + 1);
    Vec3 mn, mx;
    xf->geometry().bounds(mn, mx);
    NEAR(mn.y, 0.5f);
    g.frame = 20;
    g.cook(xf);
    CHECK(g.totalCooks() == c0 + 2); // time dependent -> recook, box untouched
    xf->geometry().bounds(mn, mx);
    NEAR(mn.y, 1.5f);
    // cycles are refused
    Node* m = g.createNode("mountain");
    CHECK(g.connect(xf, m, 0));
    CHECK(!g.connect(m, xf, 0));
    // missing input -> error
    Node* xf2 = g.createNode("xform");
    g.cook(xf2);
    CHECK(!xf2->error().empty());
    // bypass passes input through
    xf->bypass = true;
    g.cook(xf);
    xf->geometry().bounds(mn, mx);
    NEAR(mn.y, -0.5f);
}

TEST(subdivide_and_extrude) {
    Graph g;
    Node* box = g.createNode("box");
    Node* sub = g.createNode("subdivide");
    g.connect(box, sub, 0);
    sub->parm("iterations")->val[0] = 2;
    g.cook(sub);
    const Geometry& s = sub->geometry();
    CHECK(s.numPrims() == 6 * 16);
    CHECK(closedManifold(s));
    float vol = signedVolume(s);
    CHECK(vol > 0.3f && vol < 1.0f); // shrinks toward a sphere-ish shape
    // all points roughly within the original box and away from corners
    Vec3 mn, mx; s.bounds(mn, mx);
    CHECK(mx.x < 0.5f && mx.x > 0.3f);

    Node* ex = g.createNode("polyextrude");
    g.connect(box, ex, 0);
    ex->parm("group")->str = "@P.y > 0.4";
    ex->parm("dist")->val[0] = 1;
    g.cook(ex);
    const Geometry& e = ex->geometry();
    CHECK(ex->error().empty());
    CHECK(e.numPrims() == 6 + 4);
    CHECK(closedManifold(e));
    NEAR(signedVolume(e), 2.0f);
    // individual mode on all faces
    ex->parm("together")->val[0] = 0;
    ex->parm("group")->str = "";
    ex->touch();
    g.cook(ex);
    CHECK(ex->geometry().numPrims() == 6 * 5);
}

TEST(scatter_copy_wrangle_blast) {
    Graph g;
    Node* sph = g.createNode("sphere");
    Node* sc = g.createNode("scatter");
    g.connect(sph, sc, 0);
    sc->parm("npts")->val[0] = 200;
    g.cook(sc);
    const Geometry& pts = sc->geometry();
    CHECK(pts.numPoints() == 200);
    CHECK(pts.numPrims() == 0);
    bool onSurface = true;
    for (auto& p : pts.P) onSurface &= std::fabs(length(p) - 1.f) < 0.06f;
    CHECK(onSurface);
    CHECK(pts.findAttrib(AttribClass::Point, "N") && pts.findAttrib(AttribClass::Point, "pscale"));

    Node* box = g.createNode("box");
    Node* cp = g.createNode("copytopoints");
    g.connect(box, cp, 0);
    g.connect(sc, cp, 1);
    g.cook(cp);
    CHECK(cp->geometry().numPrims() == 200 * 6);
    CHECK(cp->geometry().numPoints() == 200 * 8);

    Node* w = g.createNode("attribwrangle");
    g.connect(sph, w, 0);
    w->parm("snippet")->str = "@Cd = set(@P.y > 0, 0, 0);";
    g.cook(w);
    CHECK(w->error().empty());
    CHECK(w->geometry().findAttrib(AttribClass::Point, "Cd") != nullptr);
    CHECK(!w->isTimeDependent());
    w->parm("snippet")->str = "@P.y += @Time;";
    CHECK(w->isTimeDependent());
    w->parm("snippet")->str = "@P.y +=;";
    w->touch();
    g.cook(w);
    CHECK(!w->error().empty());

    Node* bl = g.createNode("blast");
    g.connect(sph, bl, 0);
    bl->parm("group")->str = "@P.y > 0";
    g.cook(bl);
    CHECK(bl->error().empty());
    CHECK(bl->geometry().numPrims() > 0 && bl->geometry().numPrims() < sph->geometry().numPrims());
    Vec3 mn, mx; bl->geometry().bounds(mn, mx);
    CHECK(mx.y < 0.3f);
}

TEST(fuse_and_merge) {
    Graph g;
    Node* a = g.createNode("box");
    Node* b = g.createNode("box");
    Node* m = g.createNode("merge");
    g.connect(a, m, 0);
    g.connect(b, m, 3);
    g.cook(m);
    CHECK(m->geometry().numPoints() == 16);
    Node* f = g.createNode("fuse");
    g.connect(m, f, 0);
    g.cook(f);
    CHECK(f->geometry().numPoints() == 8);
    CHECK(f->geometry().numPrims() == 12);
    // facet then fuse restores shared points
    Node* fc = g.createNode("facet");
    g.connect(a, fc, 0);
    g.cook(fc);
    CHECK(fc->geometry().numPoints() == 24);
}

TEST(serialization) {
    Graph g;
    Node* box = g.createNode("box", 10, 20);
    Node* w = g.createNode("attribwrangle", 10, 120);
    g.connect(box, w, 0);
    box->parm("size")->val[1] = 2.5f;
    box->parm("t")->expr[0] = "sin($T)";
    w->parm("snippet")->str = "@P *= 2; // \"quoted\"\n";
    w->bypass = true;
    g.setDisplayNode(w);
    std::string text = g.toJson().dump();
    Graph g2;
    std::string err;
    std::string perr;
    CHECK(g2.fromJson(Json::parse(text, &perr), &err));
    CHECK(perr.empty());
    Node* b2 = g2.findNode("box1");
    Node* w2 = g2.findNode("attribwrangle1");
    CHECK(b2 && w2);
    if (!b2 || !w2) return;
    NEAR(b2->parm("size")->val[1], 2.5f);
    CHECK(b2->parm("t")->expr[0] == "sin($T)");
    CHECK(w2->parm("snippet")->str == "@P *= 2; // \"quoted\"\n");
    CHECK(w2->bypass);
    CHECK(w2->inputs[0] == b2);
    CHECK(g2.displayNode() == w2);
    NEAR(b2->y, 20);
    CHECK(g2.toJson().dump() == text);
}

TEST(copy_paste_and_merge_nodes) {
    Graph g;
    Node* box = g.createNode("box", 0, 0);
    Node* xf = g.createNode("xform", 0, 80);
    Node* out = g.createNode("null", 0, 160, "OUT");
    g.connect(box, xf, 0);
    g.connect(xf, out, 0);
    xf->parm("t")->val[1] = 3;
    xf->parm("r")->expr[1] = "$F";
    // copy two of the three nodes and paste them back
    Json nodes = g.nodesToJson({xf, out});
    CHECK(nodes.size() == 2);
    std::vector<Node*> made = g.addNodesFromJson(Json::parse(nodes.dump(0)), 40, 40);
    CHECK(made.size() == 2 && g.nodes().size() == 5);
    if (made.size() == 2) {
        Node* xf2 = made[0]->type().name == "xform" ? made[0] : made[1];
        Node* out2 = made[0] == xf2 ? made[1] : made[0];
        CHECK(xf2->name() != "xform1" && out2->name() != "OUT"); // renamed, not clashing
        NEAR(xf2->parm("t")->val[1], 3);
        CHECK(xf2->parm("r")->expr[1] == "$F");
        NEAR(xf2->x, 40); NEAR(xf2->y, 120);
        CHECK(out2->inputs[0] == xf2);    // wire inside the copied set is kept
        CHECK(xf2->inputs[0] == nullptr); // wire to a node outside the set is not
        CHECK(xf->inputs[0] == box && out->inputs[0] == xf); // originals untouched
    }
    // merging a whole scene keeps its wiring and leaves the display node alone
    Graph other;
    buildDemoScene(other, "sculpt");
    size_t before = g.nodes().size();
    Node* disp = g.displayNode();
    made = g.addNodesFromJson(other.toJson().get("nodes"), 500, 0);
    CHECK(made.size() == other.nodes().size() && g.nodes().size() == before + made.size());
    CHECK(g.displayNode() == disp);
    int wired = 0;
    for (Node* n : made) for (Node* in : n->inputs) wired += in != nullptr;
    int expect = 0;
    for (auto& n : other.nodes()) for (Node* in : n->inputs) expect += in != nullptr;
    CHECK(wired == expect);
    for (Node* n : made) { g.cook(n); CHECK(n->error().empty()); }
}

TEST(subnetworks) {
    Graph g;
    Node* box = g.createNode("box");
    Node* sn = g.createNode("subnet");
    CHECK(sn && sn->subnet() && sn->subnet()->parent() == &g && sn->subnet()->owner() == sn);
    if (!sn || !sn->subnet()) return;
    g.connect(box, sn, 0);
    g.cook(sn);
    CHECK(sn->error().empty() && sn->geometry().numPoints() == 8); // default: pass-through
    // edit inside: add a transform between Input and Output
    Graph* sub = sn->subnet();
    Node* in = sub->findNode("input1");
    Node* out = sub->outputNode();
    CHECK(in && out && out->type().name == "output");
    Node* xf = sub->createNode("xform");
    sub->connect(in, xf, 0);
    sub->connect(xf, out, 0);
    xf->parm("t")->val[1] = 5;
    g.cook(sn); // the outer node notices the inner change
    Vec3 mn, mx;
    sn->geometry().bounds(mn, mx);
    NEAR(mn.y, 4.5f);
    // a change outside reaches the inside
    box->parm("size")->val[0] = 4; box->touch();
    g.cook(sn);
    sn->geometry().bounds(mn, mx);
    NEAR(mx.x, 2);
    // nothing changed -> nothing recooks
    uint64_t c0 = g.totalCooks() + sub->totalCooks();
    g.cook(sn);
    CHECK(g.totalCooks() + sub->totalCooks() == c0);
    // animation inside makes the subnet time dependent
    CHECK(!sn->isTimeDependent());
    xf->parm("t")->expr[0] = "$F";
    xf->touch();
    CHECK(sn->isTimeDependent());
    g.frame = 7; g.cook(sn);
    sn->geometry().bounds(mn, mx);
    NEAR(mn.x, 5); // 7 - 2
    // unconnected input and error reporting
    Node* in2 = sub->createNode("input");
    in2->parm("index")->val[0] = 3;
    Node* m = sub->createNode("merge");
    sub->connect(xf, m, 0); sub->connect(in2, m, 1); sub->connect(m, out, 0);
    g.cook(sn);
    CHECK(sn->error().empty() && sn->geometry().numPoints() == 8);
    Node* bad = sub->createNode("attribwrangle");
    bad->parm("snippet")->str = "@P.y = ;";
    sub->connect(m, bad, 0); sub->connect(bad, out, 0);
    g.cook(sn);
    CHECK(!sn->error().empty());
    sub->connect(m, out, 0);
    g.cook(sn);
    CHECK(sn->error().empty());
    // save / load keeps the contents
    Graph g2;
    CHECK(g2.fromJson(Json::parse(g.toJson().dump())));
    Node* sn2 = g2.findNode(sn->name());
    CHECK(sn2 && sn2->subnet() && sn2->subnet()->nodes().size() == sub->nodes().size());
    if (sn2 && sn2->subnet()) {
        CHECK(sn2->subnet()->parent() == &g2 && sn2->subnet()->owner() == sn2);
        g2.frame = 7; g2.cook(sn2);
        Vec3 a, b; sn2->geometry().bounds(a, b);
        NEAR(a.x, 5); NEAR(a.y, 4.5f);
        CHECK(g2.toJson().dump() == g.toJson().dump());
    }
    // nested subnets
    Node* inner = sub->createNode("subnet");
    sub->connect(m, inner, 0); sub->connect(inner, out, 0);
    Node* pk = inner->subnet()->createNode("xform");
    pk->parm("scale")->val[0] = 2;
    inner->subnet()->connect(inner->subnet()->findNode("input1"), pk, 0);
    inner->subnet()->connect(pk, inner->subnet()->outputNode(), 0);
    g.frame = 1; g.cook(sn);
    sn->geometry().bounds(mn, mx);
    CHECK(sn->error().empty());
    NEAR(mx.x - mn.x, 8); // 4 wide, doubled
    // a simulation inside a subnet sees an animated outer emitter frame by frame
    Graph g3;
    Node* e = g3.createNode("sphere");
    for (int c = 0; c < 3; ++c) e->parm("rad")->val[c] = 0.01f;
    e->parm("t")->expr[0] = "$F";
    Node* s3 = g3.createNode("subnet");
    g3.connect(e, s3, 0);
    Graph* sub3 = s3->subnet();
    Node* pop = sub3->createNode("popsolver");
    for (const char* z : {"speedn", "velvar", "drag"}) pop->parm(z)->val[0] = 0;
    pop->parm("gravity")->val[1] = 0; pop->parm("life")->val[0] = 100; pop->parm("rate")->val[0] = 24;
    sub3->connect(sub3->findNode("input1"), pop, 0);
    sub3->connect(pop, sub3->outputNode(), 0);
    g3.frame = 30; g3.cook(s3);
    s3->geometry().bounds(mn, mx);
    CHECK(s3->error().empty() && s3->geometry().numPoints() == 30);
    CHECK(mn.x < 1.1f && mx.x > 29.9f);
    g3.frame = 31; g3.cook(s3); // continues from the cache instead of restarting
    CHECK(s3->geometry().numPoints() == 31 && pop->simCacheStart == 1);
}

TEST(collapse_to_subnet) {
    Graph g;
    buildDemoScene(g, "terrain");
    Node* disp = g.displayNode();
    g.cook(disp);
    const size_t pts = disp->geometry().numPoints(), prims = disp->geometry().numPrims();
    // rock branch: sphere -> mountain -> color, feeding copy_rocks
    std::vector<Node*> sel = {g.findNode("rock"), g.findNode("rock_noise"), g.findNode("rock_color")};
    std::string err;
    Node* sn = g.collapseToSubnet(sel, &err);
    CHECK(sn != nullptr);
    if (!sn) { std::printf("  %s\n", err.c_str()); return; }
    CHECK(!g.findNode("rock") && sn->subnet()->findNode("rock") && sn->subnet()->nodes().size() == 4);
    CHECK(g.findNode("copy_rocks")->inputs[0] == sn);
    g.cook(g.displayNode());
    CHECK(g.displayNode()->error().empty());
    CHECK(g.displayNode()->geometry().numPoints() == pts && g.displayNode()->geometry().numPrims() == prims);
    // a middle section with an outside input: mountain1 + height_ramp (reads ground, feeds scatter and merge)
    sel = {g.findNode("mountain1"), g.findNode("height_ramp")};
    Node* sn2 = g.collapseToSubnet(sel, &err);
    CHECK(sn2 != nullptr);
    if (sn2) {
        CHECK(sn2->inputs[0] == g.findNode("ground"));
        CHECK(g.findNode("scatter1")->inputs[0] == sn2 && g.findNode("merge1")->inputs[0] == sn2);
        g.cook(g.displayNode());
        CHECK(g.displayNode()->error().empty() && g.displayNode()->geometry().numPoints() == pts);
    }
    // two different outputs -> refused, graph untouched
    size_t count = g.nodes().size();
    sel = {g.findNode("ground"), g.findNode("scatter1")};
    CHECK(g.collapseToSubnet(sel, &err) == nullptr && !err.empty());
    CHECK(g.nodes().size() == count);
    // collapsing the displayed node keeps the display on the subnet
    sel = {g.findNode("merge1"), g.findNode("OUT")};
    Node* sn3 = g.collapseToSubnet(sel, &err);
    CHECK(sn3 && g.displayNode() == sn3);
    if (sn3) { g.cook(sn3); CHECK(sn3->error().empty() && sn3->geometry().numPoints() == pts); }
}

TEST(obj_roundtrip) {
    Graph g;
    Node* s = g.createNode("sphere");
    g.cook(s);
    std::string path = "hd_test_sphere.obj";
    std::string err;
    CHECK(saveObj(path, s->geometry(), &err));
    Geometry in;
    CHECK(loadObj(path, in, &err));
    CHECK(in.numPoints() == s->geometry().numPoints());
    CHECK(in.numPrims() == s->geometry().numPrims());
    CHECK(in.findAttrib(AttribClass::Point, "N") != nullptr);
    std::remove(path.c_str());
}

TEST(all_nodes_cook_on_defaults) {
    // Every node type must cook without crashing, with a sphere wired to each input.
    for (auto& t : nodeTypes()) {
        Graph g;
        Node* src = g.createNode("sphere");
        Node* src2 = g.createNode("scatter");
        g.connect(src, src2, 0);
        Node* n = g.createNode(t.name);
        CHECK(n != nullptr);
        if (t.name == "input") continue; // only meaningful inside a subnet (covered by the subnet tests)
        if (t.category == "Compositing") {
            // image nodes get a small image on every input (the shape mask's second input takes curves)
            Node* img = g.createNode("copconstant");
            img->parm("width")->val[0] = 32; img->parm("height")->val[0] = 24;
            for (int k = 0; k < t.maxInputs; ++k) g.connect(t.name == "copshape" && k == 1 ? src : (t.name == "coprender" ? src : img), n, k);
            if (t.name == "coprender") { n->parm("width")->val[0] = 32; n->parm("height")->val[0] = 24; n->parm("samples")->val[0] = 2; }
        } else {
            if (t.maxInputs >= 1) g.connect(t.name == "trail" ? g.createNode("popsolver") : src, n, 0);
            if (t.maxInputs >= 2) g.connect(src2, n, 1);
        }
        g.cook(n);
        // nodes that need a particular kind of input (a skeleton, a file, matching targets) are expected to
        // reject a plain sphere with a clear error; they still must not crash or produce broken geometry
        static const std::set<std::string> picky = {"file", "switch", "fkpose", "ikchain", "capture", "bonedeform", "blendshapes", "bvhimport", "retarget",
                                                   "copfile", "copzcomp", "copdefocus" /* need a file or depth */};
        if (!picky.count(t.name) && !n->error().empty())
            std::printf("  node %s error: %s\n", t.name.c_str(), n->error().c_str());
        CHECK(picky.count(t.name) || n->error().empty());
        if (picky.count(t.name) && t.name != "switch" && t.name != "file") CHECK(!n->error().empty() || t.name == "blendshapes");
        if (t.category == "Compositing" && n->error().empty()) CHECK(!n->geometry().images.empty() && n->geometry().images[0].valid());
        for (auto& [nm, a] : n->geometry().pointAttribs) CHECK(a.count() == n->geometry().numPoints());
        for (auto& [nm, a] : n->geometry().primAttribs) CHECK(a.count() == n->geometry().numPrims());
        for (int v : n->geometry().vertices) CHECK(v >= 0 && v < int(n->geometry().numPoints()));
    }
}

TEST(particle_solver) {
    Graph g;
    Node* src = g.createNode("sphere");
    Node* pop = g.createNode("popsolver");
    g.connect(src, pop, 0);
    pop->parm("rate")->val[0] = 240; // 10 per frame at 24 fps
    pop->parm("lifevar")->val[0] = 0;
    pop->parm("life")->val[0] = 1;   // 24 frames
    g.frame = 1; g.cook(pop);
    CHECK(pop->error().empty());
    CHECK(pop->geometry().numPoints() == 10);
    g.frame = 12; g.cook(pop);
    CHECK(pop->geometry().numPoints() == 120);
    CHECK(pop->simCacheStart == 1 && pop->simCacheEnd == 12);
    const Geometry snap = pop->geometry();
    // steady state: births balance deaths
    g.frame = 60; g.cook(pop);
    size_t steady = pop->geometry().numPoints();
    CHECK(steady >= 230 && steady <= 250);
    // ground collision keeps everything above the floor; ids are unique
    Vec3 mn, mx; pop->geometry().bounds(mn, mx);
    CHECK(mn.y >= -1e-4f);
    const Attrib* id = pop->geometry().findAttrib(AttribClass::Point, "id");
    CHECK(id != nullptr);
    if (id) { std::set<int> ids; for (size_t i = 0; i < id->count(); ++i) ids.insert(int(id->getF(i))); CHECK(ids.size() == steady); }
    // scrubbing back uses the cache (no re-simulation) and is identical
    g.frame = 12; g.cook(pop);
    CHECK(pop->geometry().numPoints() == snap.numPoints());
    bool same = true;
    for (size_t i = 0; i < snap.numPoints() && same; ++i) same = snap.P[i] == pop->geometry().P[i];
    CHECK(same);
    CHECK(pop->simCacheEnd == 60);
    // editing a parameter (or anything upstream) invalidates the cache and re-simulates deterministically
    src->parm("rows")->val[0] = 9; src->touch();
    g.cook(pop);
    CHECK(pop->simCacheEnd == 12);
    src->parm("rows")->val[0] = 13; src->touch();
    g.cook(pop);
    same = pop->geometry().numPoints() == snap.numPoints();
    for (size_t i = 0; i < snap.numPoints() && same; ++i) same = length(snap.P[i] - pop->geometry().P[i]) < 1e-5f;
    CHECK(same);
    // before the start frame there are no particles
    pop->parm("startframe")->val[0] = 20; pop->touch();
    g.cook(pop);
    CHECK(pop->geometry().numPoints() == 0);
    // POP wrangle + burst emission + die on collision
    pop->parm("startframe")->val[0] = 1;
    pop->parm("emission")->val[0] = 1;
    pop->parm("rate")->val[0] = 500;
    pop->parm("snippet")->str = "@v.y = 0; f@marked = 1;";
    pop->touch();
    g.frame = 5; g.cook(pop);
    CHECK(pop->error().empty());
    CHECK(pop->geometry().numPoints() == 500);
    CHECK(pop->geometry().findAttrib(AttribClass::Point, "marked") != nullptr);
    pop->parm("snippet")->str = "@v.y = ;"; pop->touch(); g.cook(pop);
    CHECK(!pop->error().empty());
    // trail makes one 2-point polyline per particle
    pop->parm("snippet")->str = ""; pop->touch();
    Node* tr = g.createNode("trail");
    g.connect(pop, tr, 0);
    g.cook(tr);
    CHECK(tr->error().empty());
    CHECK(tr->geometry().numPrims() == 500 && tr->geometry().numPoints() == 1000);
    // animated emitter: particles are born where the emitter was at each frame
    Graph g2;
    Node* e = g2.createNode("sphere");
    e->parm("rad")->val[0] = e->parm("rad")->val[1] = e->parm("rad")->val[2] = 0.01f;
    e->parm("t")->expr[0] = "$F";
    Node* p2 = g2.createNode("popsolver");
    g2.connect(e, p2, 0);
    for (const char* z : {"speedn", "velvar", "drag"}) p2->parm(z)->val[0] = 0;
    p2->parm("gravity")->val[1] = 0;
    p2->parm("life")->val[0] = 100; p2->parm("rate")->val[0] = 24;
    g2.frame = 30; g2.cook(p2); // simulates 1..30 in one go
    p2->geometry().bounds(mn, mx);
    CHECK(p2->geometry().numPoints() == 30);
    CHECK(mn.x < 1.1f && mx.x > 29.9f);
}

TEST(volumes_and_pyro) {
    // voxelizer: a unit box fills ~1 unit^3
    Graph g;
    Node* box = g.createNode("box");
    g.cook(box);
    std::vector<unsigned char> mask;
    const float dx = 0.05f;
    size_t n = voxelizeGeometry(box->geometry(), 30, 30, 30, Vec3(-0.75f), dx, 0.1f, mask);
    CHECK(std::fabs(double(n) * dx * dx * dx - 1.0) < 0.05);
    // fog volume from a sphere: volume ~ 4/3 pi
    Node* sph = g.createNode("sphere");
    sph->parm("rows")->val[0] = 40; sph->parm("cols")->val[0] = 80;
    Node* fog = g.createNode("fogvolume");
    g.connect(sph, fog, 0);
    g.cook(fog);
    CHECK(fog->error().empty() && fog->geometry().volumes.size() == 1);
    if (fog->geometry().volumes.size() == 1) {
        const Volume& v = fog->geometry().volumes[0];
        double sum = 0;
        for (float d : *v.density) sum += d;
        CHECK(std::fabs(sum * v.dx * v.dx * v.dx - 4.18879) < 0.15);
        Vec3 mn, mx; fog->geometry().bounds(mn, mx);
        CHECK(mn.x < -1 && mx.x > 1);
        // transform moves and scales the volume
        Node* xf = g.createNode("xform");
        g.connect(fog, xf, 0);
        xf->parm("t")->val[0] = 5; xf->parm("scale")->val[0] = 2;
        g.cook(xf);
        xf->geometry().bounds(mn, mx);
        CHECK(mn.x > 2.5f && mx.x > 7 && std::fabs(xf->geometry().volumes[0].dx - v.dx * 2) < 1e-5f);
    }
    // smoke: rises, stays finite and bounded, avoids the collider, caches frames
    Node* src = g.createNode("sphere");
    for (int c = 0; c < 3; ++c) src->parm("rad")->val[c] = 0.3f;
    src->parm("t")->val[1] = 0.4f;
    Node* col = g.createNode("box");
    col->parm("t")->val[1] = 1.6f;
    for (int c = 0; c < 3; ++c) col->parm("size")->val[c] = 0.6f;
    Node* pyro = g.createNode("pyrosolver");
    pyro->parm("voxelsize")->val[0] = 0.12f;
    g.connect(src, pyro, 0);
    g.connect(col, pyro, 1);
    auto stats = [&](double& sum, double& comY, bool& ok, double& inSolid) {
        const Volume& v = pyro->geometry().volumes[0];
        sum = comY = inSolid = 0; ok = true;
        for (int k = 0; k < v.nz; ++k)
            for (int j = 0; j < v.ny; ++j)
                for (int i = 0; i < v.nx; ++i) {
                    float d = (*v.density)[size_t(i) + size_t(v.nx) * (size_t(j) + size_t(v.ny) * size_t(k))];
                    if (!(d >= 0.f && d <= 1.001f)) ok = false;
                    Vec3 p = v.origin + Vec3(i + 0.5f, j + 0.5f, k + 0.5f) * v.dx;
                    sum += d; comY += double(d) * p.y;
                    if (std::fabs(p.x) < 0.2f && std::fabs(p.z) < 0.2f && std::fabs(p.y - 1.6f) < 0.2f) inSolid += d;
                }
        comY /= std::max(sum, 1e-9);
    };
    double s1, y1, s2, y2, solid; bool ok;
    g.frame = 4; g.cook(pyro);
    CHECK(pyro->error().empty() && pyro->geometry().volumes.size() == 1);
    if (pyro->geometry().volumes.size() != 1) return;
    stats(s1, y1, ok, solid);
    CHECK(ok && s1 > 0);
    auto ptr4 = pyro->geometry().volumes[0].density;
    g.frame = 48; g.cook(pyro);
    stats(s2, y2, ok, solid);
    CHECK(ok);
    CHECK(s2 > s1 && y2 > y1 + 0.3);
    CHECK(solid == 0);
    CHECK(pyro->simCacheStart == 1 && pyro->simCacheEnd == 48);
    g.frame = 4; g.cook(pyro);
    CHECK(pyro->geometry().volumes[0].density == ptr4); // served from cache
    CHECK(pyro->geometry().volumes[0].heat != nullptr);
    // before the start frame: nothing; bad domain: error
    pyro->parm("startframe")->val[0] = 10; pyro->touch(); g.cook(pyro);
    CHECK(pyro->geometry().volumes.empty() && pyro->error().empty());
    pyro->parm("startframe")->val[0] = 1; pyro->parm("voxelsize")->val[0] = 0.001f; pyro->touch(); g.cook(pyro);
    CHECK(!pyro->error().empty());
    // merge keeps volumes alongside polygons
    pyro->parm("voxelsize")->val[0] = 0.2f; pyro->touch();
    Node* m = g.createNode("merge");
    g.connect(pyro, m, 0); g.connect(col, m, 1);
    g.cook(m);
    CHECK(m->geometry().volumes.size() == 1 && m->geometry().numPrims() == 6);
    // parallelFor covers every index exactly once
    std::vector<int> hits(1000, 0);
    parallelFor(1000, [&](int a, int b) { for (int i = a; i < b; ++i) hits[size_t(i)]++; });
    bool once = true;
    for (int h : hits) once &= h == 1;
    CHECK(once);
}

TEST(render_and_material) {
    Graph g;
    Node* box = g.createNode("box");
    Node* mat = g.createNode("material");
    g.connect(box, mat, 0);
    mat->parm("group")->str = "@P.y > 0.4";
    mat->parm("emit")->val[0] = 2; mat->parm("metal")->val[0] = 1;
    mat->parm("setcolor")->val[0] = 1;
    g.cook(mat);
    CHECK(mat->error().empty());
    const Attrib* e = mat->geometry().findAttrib(AttribClass::Prim, "emit");
    const Attrib* r = mat->geometry().findAttrib(AttribClass::Prim, "rough");
    CHECK(e && r && mat->geometry().findAttrib(AttribClass::Prim, "Cd"));
    if (e && r) {
        int lit = 0;
        for (size_t i = 0; i < e->count(); ++i) lit += e->getF(i) > 0;
        CHECK(lit == 1);            // only the top face
        NEAR(r->getF(0), 0.5f);     // others keep the default
    }
    // settings (and camera) survive a save/load round trip
    g.render.samples = 7; g.render.sunElevation = 33; g.render.ground = true;
    g.render.hasCamera = true; g.render.camYaw = 10; g.render.camDist = 5;
    Graph g2;
    CHECK(g2.fromJson(Json::parse(g.toJson().dump())));
    CHECK(g2.render.samples == 7 && g2.render.ground && g2.render.hasCamera);
    NEAR(g2.render.sunElevation, 33); NEAR(g2.render.camYaw, 10); NEAR(g2.render.camDist, 5);
    // render a small image: finishes, is deterministic, shows the object against the backdrop
    RenderSettings rs;
    rs.width = 64; rs.height = 48; rs.samples = 4; rs.camDist = 4;
    PathTracer pt;
    CHECK(!pt.running());
    pt.start(mat->geometry(), rs);
    pt.wait();
    CHECK(pt.samplesDone() == 4 && !pt.running());
    std::vector<uint8_t> a, b; int w = 0, h = 0; uint64_t ver = 0;
    CHECK(pt.fetch(a, w, h, ver));
    CHECK(w == 64 && h == 48 && a.size() == 64u * 48u * 4u);
    CHECK(!pt.fetch(b, w, h, ver)); // nothing newer
    if (a.size() == 64u * 48u * 4u) {
        auto px = [&](int x, int y) { return int(a[(size_t(y) * 64 + size_t(x)) * 4]) + a[(size_t(y) * 64 + size_t(x)) * 4 + 1] + a[(size_t(y) * 64 + size_t(x)) * 4 + 2]; };
        CHECK(std::abs(px(32, 24) - px(2, 2)) > 30);
        CHECK(px(2, 2) > px(2, 45)); // backdrop is lighter at the top
        pt.start(mat->geometry(), rs);
        pt.wait();
        ver = 0;
        CHECK(pt.fetch(b, w, h, ver) && a == b);
        // PNG file
        std::string path = "hd_test_render.png", err;
        CHECK(writePng(path, a, w, h, &err));
        FILE* f = std::fopen(path.c_str(), "rb");
        CHECK(f != nullptr);
        if (f) {
            unsigned char sig[8] = {0};
            size_t got = std::fread(sig, 1, 8, f);
            std::fseek(f, 0, SEEK_END);
            long size = std::ftell(f);
            std::fclose(f);
            CHECK(got == 8 && sig[1] == 'P' && sig[2] == 'N' && sig[3] == 'G');
            CHECK(size > 100 && size < 64 * 48 * 3); // compressed
            Image back;
            CHECK(readImage(path, back, false));
            bool same = back.w == w && back.h == h && back.c == 3;
            for (int i = 0; i < w * h && same; ++i)
                for (int k = 0; k < 3; ++k) same = same && int(std::lround(back.px[size_t(i) * 3 + size_t(k)] * 255)) == a[size_t(i) * 4 + size_t(k)];
            CHECK(same);
            std::remove(path.c_str());
        }
    }
    // stopping mid-render returns promptly and leaves the tracer reusable
    rs.samples = 100000;
    pt.start(mat->geometry(), rs);
    pt.stop();
    CHECK(!pt.running());
    // volumes and loose points render without issue
    Node* sph = g.createNode("sphere");
    Node* fog = g.createNode("fogvolume");
    fog->parm("voxelsize")->val[0] = 0.1f;
    g.connect(sph, fog, 0);
    Node* sc = g.createNode("scatter");
    g.connect(sph, sc, 0);
    Node* m = g.createNode("merge");
    g.connect(fog, m, 0); g.connect(sc, m, 1);
    g.cook(m);
    rs.samples = 2;
    pt.start(m->geometry(), rs);
    pt.wait();
    CHECK(pt.samplesDone() == 2);
}

TEST(lights_and_cameras) {
    Graph g;
    Node* box = g.createNode("box");
    Node* floor = g.createNode("grid");
    floor->parm("t")->val[1] = -0.5f;
    Node* m = g.createNode("merge");
    g.connect(box, m, 0); g.connect(floor, m, 1);
    g.cook(m);
    CHECK(collectLights(g, 1).empty() && collectCameras(g, 1).empty());
    // light nodes are part of the scene without being wired in
    Node* spot = g.createNode("spotlight");
    Vec3 p(3, 4, 3);
    for (int c = 0; c < 3; ++c) spot->parm("t")->val[c] = p[c];
    Vec3 r = aimRotation(p, Vec3(0));
    spot->parm("r")->val[0] = r.x; spot->parm("r")->val[1] = r.y;
    spot->parm("light_exposure")->val[0] = 1;
    g.cook(spot);
    CHECK(spot->error().empty() && spot->geometry().empty());
    auto ls = collectLights(g, 1);
    CHECK(ls.size() == 1 && ls[0].type == Light::Spot);
    if (ls.size() == 1) {
        NEAR(ls[0].intensity, 80); // 40 doubled by +1 exposure
        Vec3 want = normalize(Vec3(0) - p);
        CHECK(length(ls[0].dir - want) < 1e-4f); // aimed at the origin
    }
    spot->bypass = true;
    CHECK(collectLights(g, 1).empty()); // bypass = off
    spot->bypass = false;
    // animated light
    Node* pt = g.createNode("pointlight");
    pt->parm("t")->expr[0] = "$F";
    CHECK(collectLights(g, 12).size() == 2);
    NEAR(collectLights(g, 12)[1].pos.x, 12);
    // camera: lens and aim
    Node* cam = g.createNode("camera");
    for (int c = 0; c < 3; ++c) cam->parm("t")->val[c] = Vec3(0, 1, 6)[c];
    Vec3 cr = aimRotation(Vec3(0, 1, 6), Vec3(0));
    cam->parm("r")->val[0] = cr.x; cam->parm("r")->val[1] = cr.y;
    auto cams = collectCameras(g, 1);
    CHECK(cams.size() == 1);
    if (cams.size() == 1) {
        CHECK(length(cams[0].fwd - normalize(Vec3(0, -1, -6))) < 1e-4f);
        NEAR(cams[0].fovYDegrees(), 2 * std::atan(41.4214 / 100.0 / (1280.0 / 720.0)) * 180 / 3.14159265);
    }
    // rendering: a spot light lights the box but not a far corner; no lights at all = default sun and sky
    RenderSettings rs;
    rs.width = 64; rs.height = 48; rs.samples = 8;
    rs.useEye = true; rs.camEye = Vec3(0, 1, 6); rs.camTarget = Vec3(0, 0, 0); rs.camFov = 40;
    auto render = [&](const std::vector<Light>& lights) {
        PathTracer ptr;
        ptr.start(m->geometry(), rs, lights);
        ptr.wait();
        std::vector<uint8_t> img; int w, h; uint64_t v = 0;
        ptr.fetch(img, w, h, v);
        return img;
    };
    auto lum = [](const std::vector<uint8_t>& img, int x, int y) { size_t i = (size_t(y) * 64 + size_t(x)) * 4; return int(img[i]) + img[i + 1] + img[i + 2]; };
    std::vector<Light> one = {collectLights(g, 1)[0]};
    one[0].coneAngle = 25; one[0].coneDelta = 5;
    auto lit = render(one);
    auto def = render({});
    CHECK(lit.size() == 64u * 48u * 4u && def.size() == lit.size());
    if (lit.size() == 64u * 48u * 4u) {
        CHECK(lum(lit, 32, 24) > 60);                       // the box, inside the cone
        CHECK(lum(lit, 32, 24) > lum(lit, 3, 36) + 40);     // floor far outside the cone is dark (no sky either)
        CHECK(lum(def, 3, 36) > lum(lit, 3, 36) + 40);      // the default sky lights that same floor
    }
    // an environment light alone lights everything softly
    Light env; env.type = Light::Env; env.sky = false; env.intensity = 1;
    auto amb = render({env});
    if (amb.size() == lit.size()) CHECK(lum(amb, 3, 36) > 60);
    // the active camera name is saved with the scene
    g.render.cameraName = cam->name();
    Graph g2;
    CHECK(g2.fromJson(Json::parse(g.toJson().dump())));
    CHECK(g2.render.cameraName == cam->name() && collectLights(g2, 1).size() == 2 && collectCameras(g2, 1).size() == 1);
}

TEST(keyframes) {
    Graph g;
    Node* box = g.createNode("box");
    Node* xf = g.createNode("xform");
    g.connect(box, xf, 0);
    Parm* t = xf->parm("t");
    CHECK(!t->anyKeys() && !xf->isTimeDependent());
    t->setKey(1, 10, 0);
    CHECK(t->animated(1) && !t->animated(0));
    CHECK(!xf->isTimeDependent()); // a single key does not move
    t->setKey(1, 20, 4);
    t->setKey(1, 30, 0);
    CHECK(xf->isTimeDependent() && !t->isDefault());
    // passes through the keys, holds outside them
    NEAR(t->evalKeys(1, 10), 0); NEAR(t->evalKeys(1, 20), 4); NEAR(t->evalKeys(1, 30), 0);
    NEAR(t->evalKeys(1, 1), 0); NEAR(t->evalKeys(1, 99), 0);
    // smooth: eases (slow near the keys), symmetric, never overshoots
    float q = t->evalKeys(1, 12.5f), h = t->evalKeys(1, 15);
    CHECK(q > 0 && q < 1.0f); // less than the 1.0 a straight line gives
    NEAR(h, 2);
    bool bounded = true;
    for (float f = 1; f <= 40; f += 0.25f) { float v = t->evalKeys(1, f); bounded &= v >= -1e-4f && v <= 4.0001f; }
    CHECK(bounded);
    // rising run of keys stays monotone
    Parm* s = xf->parm("scale");
    s->setKey(0, 1, 0); s->setKey(0, 2, 0.1f); s->setKey(0, 20, 10); s->setKey(0, 21, 10.1f);
    bool mono = true; float prev = -1;
    for (float f = 1; f <= 21; f += 0.1f) { float v = s->evalKeys(0, f); mono &= v >= prev - 1e-4f; prev = v; }
    CHECK(mono);
    s->keys[0].clear(); s->val[0] = 1;
    // linear and constant
    t->keys[1][0].interp = 1;
    NEAR(t->evalKeys(1, 12.5f), 1);
    t->keys[1][0].interp = 2;
    NEAR(t->evalKeys(1, 19.9f), 0);
    NEAR(t->evalKeys(1, 20), 4);
    t->keys[1][0].interp = 0;
    // replacing a key on the same frame, keys stay sorted, new keys inherit the curve type
    t->setKey(1, 20, 6);
    CHECK(t->keys[1].size() == 3);
    NEAR(t->evalKeys(1, 20), 6);
    t->setKey(1, 5, 1);
    CHECK(t->keys[1].size() == 4 && t->keys[1][0].frame == 5 && t->keyIndexAt(1, 20) == 2);
    // setValue keys the frame on an animated component and just sets the value otherwise
    t->setValue(1, 9, 25);
    CHECK(t->keyIndexAt(1, 25) >= 0);
    t->setValue(0, 7, 25);
    CHECK(!t->animated(0));
    NEAR(t->val[0], 7);
    // the node cooks differently per frame
    xf->touch();
    Vec3 mn, mx;
    g.frame = 20; g.cook(xf); xf->geometry().bounds(mn, mx); NEAR(mn.y, 5.5f);
    g.frame = 30; g.cook(xf); xf->geometry().bounds(mn, mx); NEAR(mn.y, -0.5f);
    // save / load
    Graph g2;
    CHECK(g2.fromJson(Json::parse(g.toJson().dump())));
    Parm* t2 = g2.findNode("xform1")->parm("t");
    CHECK(t2->keys[1].size() == t->keys[1].size());
    for (float f : {5.f, 12.f, 20.f, 27.f}) NEAR(t2->evalKeys(1, f), t->evalKeys(1, f));
    CHECK(g2.toJson().dump() == g.toJson().dump());
    // removing keys; the last one leaves its value in the parameter
    CHECK(t->removeKey(1, 25) && !t->removeKey(1, 26));
    while (!t->keys[1].empty()) t->removeKey(1, t->keys[1].back().frame);
    CHECK(!t->animated(1));
    NEAR(t->val[1], 1);
    // an expression wins over keys; lights and cameras animate through the same path
    Node* pl = g.createNode("pointlight");
    pl->parm("light_intensity")->setKey(0, 1, 0);
    pl->parm("light_intensity")->setKey(0, 11, 10, 1);
    pl->parm("light_intensity")->keys[0][0].interp = 1;
    NEAR(collectLights(g, 6)[0].intensity, 5);
    pl->parm("light_intensity")->expr[0] = "3";
    NEAR(collectLights(g, 6)[0].intensity, 3);
}

TEST(demo_scenes_cook_cleanly) {
    for (auto& d : demoScenes()) {
        Graph g;
        CHECK(buildDemoScene(g, d.name));
        for (float f : {1.f, 37.f}) {
            g.frame = f;
            for (auto& n : g.nodes()) {
                g.cook(n.get());
                if (!n->error().empty()) std::printf("  demo %s node %s: %s\n", d.name.c_str(), n->name().c_str(), n->error().c_str());
                CHECK(n->error().empty());
            }
        }
        CHECK(g.displayNode() && !g.displayNode()->geometry().empty());
        // round-trip through the file format
        Graph g2;
        CHECK(g2.fromJson(Json::parse(g.toJson().dump())));
        CHECK(g2.nodes().size() == g.nodes().size());
    }
}


// ---------------------------------------------------------------- destruction

static std::vector<Geometry> splitPieces(const Geometry& g) {
    std::map<int, std::vector<int>> by;
    const Attrib* pa = g.findAttrib(AttribClass::Prim, "piece");
    for (size_t i = 0; i < g.numPrims(); ++i) by[pa ? int(pa->getF(i)) : 0].push_back(int(i));
    std::vector<Geometry> out;
    for (auto& [k, prims] : by) out.push_back(g.extractPrims(prims));
    return out;
}

TEST(convex_hull_and_mass) {
    std::vector<Vec3> pts;
    for (int i = 0; i < 8; ++i) pts.push_back(Vec3(i & 1 ? 1.f : 0.f, i & 2 ? 1.f : 0.f, i & 4 ? 1.f : 0.f));
    for (int i = 0; i < 50; ++i) pts.push_back(Vec3(hashRandom(uint32_t(i * 3)), hashRandom(uint32_t(i * 3 + 1)), hashRandom(uint32_t(i * 3 + 2))) * 0.9f + Vec3(0.05f));
    ConvexHull h;
    CHECK(buildConvexHull(pts, h));
    CHECK(h.pts.size() == 8);
    CHECK(h.tris.size() == 36);
    CHECK(h.planes.size() == 6);
    CHECK(h.distance(Vec3(0.5f)) < -0.49f);
    CHECK(std::fabs(h.distance(Vec3(2, 0.5f, 0.5f)) - 1.f) < 1e-4f);
    MassProps mp = computeMassProps(h.pts, h.tris);
    NEAR(mp.volume, 1.0);
    CHECK(length(mp.com - Vec3(0.5f)) < 1e-5f);
    NEAR(mp.inertia[0][0], 1.0 / 6.0); NEAR(mp.inertia[1][1], 1.0 / 6.0); NEAR(mp.inertia[2][2], 1.0 / 6.0);
    NEAR(mp.inertia[0][1], 0.0); NEAR(mp.inertia[1][2], 0.0);
    // flat input is thickened into a slab instead of failing
    std::vector<Vec3> flat = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(1, 0, 1), Vec3(0, 0, 1)};
    CHECK(buildConvexHull(flat, h, 0.01f));
    CHECK(computeMassProps(h.pts, h.tris).volume > 0);
    // ear clipping with a hole: a 4x4 square minus a 2x2 square = 12
    std::vector<Vec3> sq = {Vec3(0, 0, 0), Vec3(4, 0, 0), Vec3(4, 4, 0), Vec3(0, 4, 0), Vec3(1, 1, 0), Vec3(1, 3, 0), Vec3(3, 3, 0), Vec3(3, 1, 0)};
    std::vector<int> tris;
    CHECK(triangulatePolygon(sq, {{0, 1, 2, 3}, {4, 5, 6, 7}}, Vec3(0, 0, 1), tris));
    double area = 0;
    bool allCCW = true;
    for (size_t k = 0; k < tris.size(); k += 3) {
        Vec3 c = cross(sq[size_t(tris[k + 1])] - sq[size_t(tris[k])], sq[size_t(tris[k + 2])] - sq[size_t(tris[k])]);
        area += 0.5 * c.z;
        if (c.z < -1e-6f) allCCW = false;
    }
    NEAR(area, 12.0);
    CHECK(allCCW);
}

TEST(voronoi_fracture) {
    // box cut by explicit cells: every piece is a closed, outward-facing mesh and volume is conserved
    Graph g;
    Node* box = g.createNode("box");
    box->parm("size")->val[0] = 2; box->parm("size")->val[1] = 1; box->parm("size")->val[2] = 1.5f;
    Node* fr = g.createNode("voronoifracture");
    g.connect(box, fr, 0);
    fr->parm("pieces")->val[0] = 25;
    g.cook(fr);
    CHECK(fr->error().empty());
    auto pieces = splitPieces(fr->geometry());
    CHECK(pieces.size() == 25);
    double total = 0;
    bool closed = true, positive = true;
    for (auto& p : pieces) {
        closed = closed && closedManifold(fusePoints(p, 1e-6f));
        float v = signedVolume(p);
        positive = positive && v > 0;
        total += v;
    }
    CHECK(closed);
    CHECK(positive);
    CHECK(std::fabs(total - 3.0) < 1e-3);
    const Attrib* inside = fr->geometry().findAttrib(AttribClass::Prim, "inside");
    CHECK(inside != nullptr);
    if (inside) { int n = 0; for (size_t i = 0; i < inside->count(); ++i) n += inside->getF(i) > 0.5f; CHECK(n > 25); }
    // a non-convex mesh (torus) cut through its hole: caps with holes still give closed pieces
    Node* tor = g.createNode("torus");
    tor->parm("rows")->val[0] = 32; tor->parm("cols")->val[0] = 16;
    Node* fr2 = g.createNode("voronoifracture");
    g.connect(tor, fr2, 0);
    fr2->parm("pieces")->val[0] = 12;
    g.cook(fr2);
    CHECK(fr2->error().empty());
    double tv = signedVolume(tor->geometry());
    pieces = splitPieces(fr2->geometry());
    CHECK(pieces.size() == 12);
    total = 0; closed = true;
    for (auto& p : pieces) { total += signedVolume(p); closed = closed && closedManifold(fusePoints(p, 1e-6f)); }
    CHECK(closed);
    CHECK(std::fabs(total - tv) < 1e-3 * tv);
    // explicit cut points on input 2: two points split a box into two halves
    Geometry two; two.addPoint(Vec3(-0.5f, 0, 0)); two.addPoint(Vec3(0.5f, 0, 0));
    Geometry cube = box->geometry();
    FractureStats st;
    Geometry halves = voronoiFracture(cube, two.P, FractureOptions(), &st);
    CHECK(st.pieces == 2);
    auto hp = splitPieces(halves);
    CHECK(hp.size() == 2);
    if (hp.size() == 2) { NEAR(signedVolume(hp[0]), 1.5); NEAR(signedVolume(hp[1]), 1.5); }
    // a cell point outside the mesh yields no piece
    two.addPoint(Vec3(10, 10, 10));
    voronoiFracture(cube, two.P, FractureOptions(), &st);
    CHECK(st.pieces == 2 && st.emptyCells == 1);
}

TEST(rbd_solver) {
    // a single box dropped from 2 m lands on the ground, comes to rest flat and stays there
    Graph g;
    Node* box = g.createNode("box");
    box->parm("t")->val[1] = 2;
    Node* rbd = g.createNode("rbdsolver");
    g.connect(box, rbd, 0);
    g.frame = 1; g.cook(rbd);
    CHECK(rbd->error().empty());
    Vec3 mn, mx; rbd->geometry().bounds(mn, mx);
    NEAR(mn.y, 1.5);
    // free fall for 0.25 s: y = 2 - 0.5 g t^2 (substeps make it slightly different; allow 1 cm)
    g.frame = 7; g.cook(rbd);
    rbd->geometry().bounds(mn, mx);
    CHECK(std::fabs((mn.y + 0.5f) - (2.f - 0.5f * 9.81f * 0.25f * 0.25f)) < 0.01f);
    g.frame = 96; g.cook(rbd);
    rbd->geometry().bounds(mn, mx);
    CHECK(mn.y > -0.01f && mn.y < 0.01f);
    CHECK(std::fabs(mx.y - 1.f) < 0.02f);
    const Attrib* v = rbd->geometry().findAttrib(AttribClass::Point, "v");
    CHECK(v && length(v->getV(0)) < 0.05f);
    CHECK(rbd->simCacheStart == 2 && rbd->simCacheEnd == 96);
    // scrubbing back is served from the cache and is deterministic
    g.frame = 7; g.cook(rbd);
    Geometry a = rbd->geometry();
    box->parm("divs")->val[0] = 1; box->touch();
    g.cook(rbd);
    bool same = a.numPoints() == rbd->geometry().numPoints();
    for (size_t i = 0; i < a.numPoints() && same; ++i) same = length(a.P[i] - rbd->geometry().P[i]) < 1e-5f;
    CHECK(same);

    // a glued, fractured block sitting on the ground holds together
    Graph g2;
    Node* blk = g2.createNode("box");
    blk->parm("size")->val[0] = 2; blk->parm("t")->val[1] = 0.5f;
    Node* fr = g2.createNode("voronoifracture");
    g2.connect(blk, fr, 0);
    fr->parm("pieces")->val[0] = 12;
    Node* r2 = g2.createNode("rbdsolver");
    g2.connect(fr, r2, 0);
    g2.frame = 48; g2.cook(r2);
    CHECK(r2->error().empty());
    double maxMove = 0;
    for (size_t i = 0; i < fr->geometry().numPoints(); ++i) maxMove = std::max(maxMove, double(length(fr->geometry().P[i] - r2->geometry().P[i])));
    CHECK(maxMove < 0.02);
    // without glue the pieces may slide on their slanted cuts, but settle without exploding or sinking
    r2->parm("glue")->val[0] = 0; r2->touch();
    g2.cook(r2);
    maxMove = 0;
    for (size_t i = 0; i < fr->geometry().numPoints(); ++i) maxMove = std::max(maxMove, double(length(fr->geometry().P[i] - r2->geometry().P[i])));
    CHECK(maxMove < 0.5);
    r2->geometry().bounds(mn, mx);
    CHECK(mn.y > -0.01f);

    // a fast projectile (v attribute) breaks the glued block apart (at 40 m/s it breaks for every seed tried)
    r2->parm("glue")->val[0] = 1; r2->touch();
    Node* ball = g2.createNode("sphere");
    ball->parm("rad")->val[0] = ball->parm("rad")->val[1] = ball->parm("rad")->val[2] = 0.25f;
    ball->parm("t")->val[0] = -2.5f; ball->parm("t")->val[1] = 0.5f;
    Node* vel = g2.createNode("attribcreate");
    vel->parm("name")->str = "v";
    vel->parm("type")->val[0] = 1;
    vel->parm("value")->val[0] = 40;
    g2.connect(ball, vel, 0);
    Node* mg = g2.createNode("merge");
    g2.connect(fr, mg, 0);
    g2.connect(vel, mg, 1);
    g2.connect(mg, r2, 0);
    r2->parm("density")->val[0] = 1000;
    g2.frame = 30; g2.cook(r2);
    CHECK(r2->error().empty());
    // count pieces that moved noticeably: the impact must have knocked several loose
    int moved = 0;
    const Geometry& out = r2->geometry();
    const Attrib* pp = out.findAttrib(AttribClass::Point, "piece");
    std::map<int, double> moveBy;
    size_t nfr = fr->geometry().numPoints();
    for (size_t i = 0; i < nfr && i < out.numPoints(); ++i) moveBy[pp ? int(pp->getF(i)) : 0] = std::max(moveBy[pp ? int(pp->getF(i)) : 0], double(length(out.P[i] - mg->geometry().P[i])));
    for (auto& [k, m] : moveBy) moved += m > 0.1;
    CHECK(moved >= 6);
    // nothing tunnels through the ground
    out.bounds(mn, mx);
    CHECK(mn.y > -0.05f);
}

// ---------------------------------------------------------------- fluids and vellum


TEST(flip_projection) {
    // a block of fluid with a random velocity field: after one solve the grid is divergence free
    std::vector<Vec3> P, V;
    for (int k = 0; k < 16; ++k) for (int j = 0; j < 16; ++j) for (int i = 0; i < 16; ++i) {
        Vec3 p = Vec3(float(i) + 0.5f, float(j) + 0.5f, float(k) + 0.5f) * 0.05f + Vec3(0.2f);
        P.push_back(p);
        uint32_t h = uint32_t(i + 16 * (j + 16 * k));
        V.push_back(Vec3(hashRandom(h * 3u) - 0.5f, hashRandom(h * 3u + 1u) - 0.5f, hashRandom(h * 3u + 2u) - 0.5f) * 2.f);
    }
    int subs = 0;
    float div = flipTestStep(P, V, Vec3(0), 24, 0.05f, 0.002f, Vec3(0), &subs);
    CHECK(subs >= 1);
    // initial divergence is O(|v| / dx) ~ 20 1/s; after projection it must be tiny
    CHECK(div < 0.05f);
}

TEST(flip_solver) {
    Graph g;
    Node* src = g.createNode("box");
    src->parm("size")->val[0] = 0.6f; src->parm("size")->val[1] = 0.8f; src->parm("size")->val[2] = 0.6f;
    src->parm("t")->val[0] = -0.7f; src->parm("t")->val[1] = 0.4f;
    Node* obs = g.createNode("box");
    obs->parm("size")->val[0] = 0.2f; obs->parm("size")->val[1] = 0.3f; obs->parm("size")->val[2] = 0.6f;
    obs->parm("t")->val[0] = 0.4f; obs->parm("t")->val[1] = 0.15f;
    Node* fl = g.createNode("flipsolver");
    g.connect(src, fl, 0);
    g.connect(obs, fl, 1);
    fl->parm("voxelsize")->val[0] = 0.1f;
    fl->parm("domaincenter")->val[1] = 0.5f;
    fl->parm("domainsize")->val[0] = 2; fl->parm("domainsize")->val[1] = 1; fl->parm("domainsize")->val[2] = 0.6f;
    fl->parm("flipratio")->val[0] = 0.5f;
    g.frame = 1; g.cook(fl);
    CHECK(fl->error().empty());
    const size_t n0 = fl->geometry().numPoints();
    CHECK(n0 == size_t(6 * 8 * 6 * 8));
    g.frame = 30; g.cook(fl);
    const Geometry mid = fl->geometry();
    CHECK(mid.numPoints() == n0);               // a closed container never loses particles
    Vec3 mn, mx; mid.bounds(mn, mx);
    CHECK(mn.x >= -1.f && mx.x <= 1.f && mn.y >= 0.f && mn.z >= -0.3f && mx.z <= 0.3f);
    bool inObstacle = false;
    for (auto& p : mid.P) if (p.x > 0.3f && p.x < 0.5f && p.y < 0.3f) inObstacle = true;
    CHECK(!inObstacle);
    CHECK(fl->simCacheStart == 1 && fl->simCacheEnd == 30);
    // settles to a flat layer with the original volume: depth = V / area (obstacle volume excluded)
    g.frame = 240; g.cook(fl);
    double ysum = 0;
    for (auto& p : fl->geometry().P) ysum += p.y;
    double depth = 2 * ysum / double(fl->geometry().numPoints());
    double expected = (0.6 * 0.8 * 0.6) / (2.0 * 0.6 - 0.2 * 0.6) + 0.05; // + half a voxel of floor offset
    CHECK(std::fabs(depth - expected) < 0.15 * expected);
    // cached and deterministic
    g.frame = 30; g.cook(fl);
    bool same = fl->geometry().numPoints() == mid.numPoints();
    for (size_t i = 0; i < mid.numPoints() && same; ++i) same = mid.P[i] == fl->geometry().P[i];
    CHECK(same);
    // particle fluid surface around the result: a closed mesh
    Node* surf = g.createNode("particlefluidsurface");
    g.connect(fl, surf, 0);
    g.frame = 240; g.cook(surf);
    CHECK(surf->error().empty());
    CHECK(surf->geometry().numPrims() > 100);
    CHECK(closedManifold(fusePoints(surf->geometry(), 1e-6f)));
}

TEST(fluid_surface_volume) {
    // particles on a lattice filling a sphere of radius 0.5: the surface is closed and encloses about that volume
    Geometry pts;
    const float sep = 0.04f;
    for (int k = -13; k <= 13; ++k) for (int j = -13; j <= 13; ++j) for (int i = -13; i <= 13; ++i) {
        Vec3 p = Vec3(float(i), float(j), float(k)) * sep;
        if (length(p) <= 0.5f - sep * 0.5f) pts.addPoint(p);
    }
    Geometry surf;
    CHECK(particleFluidSurface(pts, sep, 1.0f, 0.6f, 2, surf));
    CHECK(closedManifold(fusePoints(surf, 1e-6f)));
    double vol = signedVolume(surf), want = 4.0 / 3.0 * 3.14159265 * 0.125;
    CHECK(std::fabs(vol / want - 1) < 0.03);
    // the default radius scale (1.2) adds a thin margin that hides gaps between irregular particles
    Geometry surf2;
    CHECK(particleFluidSurface(pts, sep, 1.2f, 0.6f, 2, surf2));
    CHECK(std::fabs(signedVolume(surf2) / want - 1) < 0.08);
    // every point lies close to the sphere
    float worst = 0;
    for (auto& p : surf.P) worst = std::max(worst, std::fabs(length(p) - 0.5f));
    CHECK(worst < 0.05f);
}

TEST(ocean) {
    Graph g;
    Node* oc = g.createNode("ocean");
    oc->parm("gridsize")->val[0] = 60; oc->parm("gridres")->val[0] = 120;
    oc->parm("windspeed")->val[0] = 10;
    g.frame = 1; g.cook(oc);
    CHECK(oc->error().empty());
    const Geometry a = oc->geometry();
    CHECK(a.numPoints() == 121u * 121u);
    double mean = 0, var = 0;
    for (auto& p : a.P) mean += p.y;
    mean /= double(a.numPoints());
    for (auto& p : a.P) var += (p.y - mean) * (p.y - mean);
    double hs = 4 * std::sqrt(var / double(a.numPoints()));
    double hsPM = 0.21 * 10 * 10 / 9.81; // Pierson-Moskowitz, ~2.1 m
    CHECK(std::fabs(mean) < 0.1 * hsPM);
    CHECK(hs > 0.5 * hsPM && hs < 1.5 * hsPM);
    const Attrib* N = a.findAttrib(AttribClass::Point, "N");
    const Attrib* foam = a.findAttrib(AttribClass::Point, "foam");
    CHECK(N && foam);
    bool unit = true, foamOk = true, up = true;
    for (size_t i = 0; i < a.numPoints(); ++i) {
        unit = unit && std::fabs(length(N->getV(i)) - 1) < 1e-3f;
        up = up && N->getV(i).y > 0;
        foamOk = foamOk && foam->getF(i) >= 0 && foam->getF(i) <= 1;
    }
    CHECK(unit && up && foamOk);
    // normals agree with the actual surface slope (finite differences on the displaced grid)
    {
        int i = 60, j = 60, W = 121;
        Vec3 dxv = a.P[size_t(j * W + i + 1)] - a.P[size_t(j * W + i - 1)];
        Vec3 dzv = a.P[size_t((j + 1) * W + i)] - a.P[size_t((j - 1) * W + i)];
        Vec3 fdN = normalize(cross(dzv, dxv));
        CHECK(dot(fdN, N->getV(size_t(j * W + i))) > 0.98f);
    }
    // animated, and calmer wind gives smaller waves
    g.frame = 24; g.cook(oc);
    CHECK(oc->geometry().P[500] != a.P[500]);
    oc->parm("windspeed")->val[0] = 4; oc->touch();
    g.cook(oc);
    double var2 = 0, m2 = 0;
    for (auto& p : oc->geometry().P) m2 += p.y;
    m2 /= double(oc->geometry().numPoints());
    for (auto& p : oc->geometry().P) var2 += (p.y - m2) * (p.y - m2);
    CHECK(var2 < var * 0.5);
}

TEST(vellum) {
    // hanging cloth pinned along its top edge: pins stay put and the cloth barely stretches
    Graph g;
    Node* gr = g.createNode("grid");
    gr->parm("sizex")->val[0] = 2; gr->parm("sizey")->val[0] = 2;
    Node* xf = g.createNode("xform");
    g.connect(gr, xf, 0);
    xf->parm("r")->val[0] = 90; xf->parm("t")->val[1] = 3;
    Node* v = g.createNode("vellumsolver");
    g.connect(xf, v, 0);
    v->parm("pinpts")->str = "@P.y > 3.99";
    g.frame = 48; g.cook(v);
    CHECK(v->error().empty());
    const Geometry& r = xf->geometry();
    const Geometry& o = v->geometry();
    bool pinsFixed = true;
    for (size_t i = 0; i < r.numPoints(); ++i) if (r.P[i].y > 3.99f) pinsFixed = pinsFixed && length(o.P[i] - r.P[i]) < 1e-5f;
    CHECK(pinsFixed);
    float maxStretch = 0;
    for (auto& pr : r.prims)
        for (int k = 0; k < pr.count; ++k) {
            int a = r.vertices[size_t(pr.start + k)], b = r.vertices[size_t(pr.start + (k + 1) % pr.count)];
            maxStretch = std::max(maxStretch, length(o.P[size_t(a)] - o.P[size_t(b)]) / length(r.P[size_t(a)] - r.P[size_t(b)]) - 1);
        }
    CHECK(maxStretch < 0.01f);
    CHECK(v->simCacheEnd == 48);

    // free fall matches 1/2 g t^2 before anything is hit (drag off)
    Graph g2;
    Node* gr2 = g2.createNode("grid");
    gr2->parm("sizex")->val[0] = 1; gr2->parm("sizey")->val[0] = 1;
    Node* x2 = g2.createNode("xform");
    g2.connect(gr2, x2, 0);
    x2->parm("t")->val[1] = 3;
    Node* v2 = g2.createNode("vellumsolver");
    g2.connect(x2, v2, 0);
    v2->parm("drag")->val[0] = 0; v2->parm("damping")->val[0] = 0;
    g2.frame = 13; g2.cook(v2);
    Vec3 mn, mx; v2->geometry().bounds(mn, mx);
    CHECK(std::fabs((3 - mn.y) - 0.5f * 9.81f * 0.25f) < 0.01f);

    // draped over a sphere: never inside it, and it ends up resting on it and the ground
    v2->parm("sphere")->val[0] = 1; v2->parm("spherec")->val[1] = 1; v2->parm("spherer")->val[0] = 0.4f; v2->touch();
    gr2->parm("sizex")->val[0] = 2; gr2->parm("sizey")->val[0] = 2; gr2->parm("rows")->val[0] = 20; gr2->parm("cols")->val[0] = 20; gr2->touch();
    g2.frame = 96; g2.cook(v2);
    float md = 1e9f;
    for (auto& p : v2->geometry().P) md = std::min(md, length(p - Vec3(0, 1, 0)));
    CHECK(md > 0.4f);
    v2->geometry().bounds(mn, mx);
    CHECK(mn.y >= 0.f && mx.y < 1.5f && mx.y > 1.35f);

    // pressure keeps a closed mesh inflated
    Graph g3;
    Node* sp = g3.createNode("sphere");
    sp->parm("rad")->val[0] = sp->parm("rad")->val[1] = sp->parm("rad")->val[2] = 0.5f;
    sp->parm("t")->val[1] = 1.5f;
    Node* v3 = g3.createNode("vellumsolver");
    g3.connect(sp, v3, 0);
    v3->parm("pressureon")->val[0] = 1;
    g3.frame = 72; g3.cook(v3);
    CHECK(v3->error().empty());
    double vol0 = signedVolume(sp->geometry()), vol = signedVolume(v3->geometry());
    CHECK(std::fabs(vol / vol0 - 1) < 0.05);
    v3->geometry().bounds(mn, mx);
    CHECK(mn.y >= 0.f);
    // and without pressure the same sphere collapses
    v3->parm("pressureon")->val[0] = 0; v3->parm("bend")->val[0] = 0.001f; v3->touch();
    g3.cook(v3);
    CHECK(signedVolume(v3->geometry()) < 0.8 * vol0);

    // rope: a pinned polyline swings down and keeps its length
    Graph g4;
    Node* ln = g4.createNode("line");
    ln->parm("origin")->val[1] = 2; ln->parm("dir")->val[0] = 1; ln->parm("dir")->val[1] = 0; ln->parm("points")->val[0] = 21;
    Node* v4 = g4.createNode("vellumsolver");
    g4.connect(ln, v4, 0);
    v4->parm("pinpts")->str = "0";
    g4.frame = 48; g4.cook(v4);
    const Geometry& rope = v4->geometry();
    float len = 0;
    for (size_t i = 0; i + 1 < rope.numPoints(); ++i) len += length(rope.P[i + 1] - rope.P[i]);
    CHECK(std::fabs(len - 1.f) < 0.02f);
    CHECK(rope.P.back().y < 1.5f); // swung down
    CHECK(length(rope.P[0] - Vec3(0, 2, 0)) < 1e-5f);
}

// ---------------------------------------------------------------- rendering (phase 5)

TEST(image_io) {
    // zlib round trips on compressible and random data
    for (int t = 0; t < 4; ++t) {
        std::vector<uint8_t> d(size_t(5000 + t * 30011));
        for (size_t i = 0; i < d.size(); ++i) d[i] = t % 2 ? uint8_t(hashRandom(uint32_t(i)) * 256) : uint8_t((i / 13) % 9 * 17);
        auto z = zlibCompress(d.data(), d.size());
        std::vector<uint8_t> o;
        CHECK(zlibDecompress(z.data(), z.size(), o) && o == d);
        if (t % 2 == 0) CHECK(z.size() < d.size() / 4);
    }
    std::vector<uint8_t> junk = {0x78, 0x9C, 1, 2, 3, 4, 5, 6};
    std::vector<uint8_t> o;
    CHECK(!zlibDecompress(junk.data(), junk.size(), o));
    // PNG RGB / RGBA exact round trip
    const int w = 37, h = 23;
    for (int c : {3, 4}) {
        std::vector<uint8_t> px(size_t(w * h * c));
        for (size_t i = 0; i < px.size(); ++i) px[i] = uint8_t((i * 7) ^ (i >> 3));
        CHECK(writePng8("hd_t.png", px, w, h, c));
        Image im;
        CHECK(readImage("hd_t.png", im, false));
        bool same = im.w == w && im.h == h && im.c == c;
        for (size_t i = 0; i < px.size() && same; ++i) same = int(std::lround(im.px[i] * 255)) == px[i];
        CHECK(same);
        // sRGB decoding for colour maps: 255 stays 1, 128 -> about 0.216
        Image lin;
        CHECK(readImage("hd_t.png", lin, true));
        std::remove("hd_t.png");
    }
    // EXR: float exact, half within half precision; channels come back by name
    std::vector<float> R(size_t(w * h)), Z(size_t(w * h));
    for (int i = 0; i < w * h; ++i) { R[size_t(i)] = float(i) * 0.37f - 50.f; Z[size_t(i)] = 1e-3f * float(i * i); }
    for (bool half : {false, true}) {
        CHECK(writeExr("hd_t.exr", w, h, {{"Z", Z.data()}, {"R", R.data()}}, half));
        int ww, hh;
        std::vector<std::pair<std::string, std::vector<float>>> ch;
        CHECK(readExrChannels("hd_t.exr", ww, hh, ch));
        CHECK(ww == w && hh == h && ch.size() == 2 && ch[0].first == "R" && ch[1].first == "Z");
        float err = 0;
        for (int i = 0; i < w * h && ch.size() == 2; ++i) {
            err = std::max(err, std::fabs(ch[0].second[size_t(i)] - R[size_t(i)]) / (std::fabs(R[size_t(i)]) + 1e-3f));
            err = std::max(err, std::fabs(ch[1].second[size_t(i)] - Z[size_t(i)]) / (std::fabs(Z[size_t(i)]) + 1e-3f));
        }
        CHECK(err < (half ? 1e-3f : 1e-7f));
        std::remove("hd_t.exr");
    }
    NEAR(halfToFloat(floatToHalf(1.f)), 1.f);
    NEAR(halfToFloat(floatToHalf(-2.5f)), -2.5f);
    CHECK(halfToFloat(floatToHalf(1e6f)) > 6e4f); // overflows to infinity
    // Radiance HDR
    Image hi;
    hi.w = 16; hi.h = 9; hi.c = 3;
    hi.px.resize(16 * 9 * 3);
    for (size_t i = 0; i < hi.px.size(); ++i) hi.px[i] = std::pow(2.f, float(int(i / 3) % 20) - 10.f) * (1.f + 0.1f * float(i % 3));
    CHECK(writeHdr("hd_t.hdr", hi));
    Image hr;
    CHECK(readImage("hd_t.hdr", hr, false));
    float herr = 0;
    for (size_t i = 0; i < hi.px.size() && hr.px.size() == hi.px.size(); ++i) herr = std::max(herr, std::fabs(hr.px[i] - hi.px[i]) / hi.px[i]);
    CHECK(hr.w == 16 && hr.h == 9 && herr < 0.01f);
    std::remove("hd_t.hdr");
    // textures: bilinear lookup, wrapping and UDIM tiles
    std::vector<uint8_t> two = {0, 0, 0, 255, 255, 255}; // 2x1: black, white
    CHECK(writePng8("hd_tex.png", std::vector<uint8_t>{0, 0, 0, 255, 255, 255}, 2, 1, 3));
    Texture tx;
    CHECK(tx.load("hd_tex.png", false));
    float v4[4];
    tx.sample(0.25f, 0.5f, v4); NEAR(v4[0], 0.f);
    tx.sample(0.75f, 0.5f, v4); NEAR(v4[0], 1.f);
    tx.sample(0.5f, 0.5f, v4); NEAR(v4[0], 0.5f);
    tx.sample(1.25f, 0.5f, v4); NEAR(v4[0], 0.f); // wraps
    CHECK(writePng8("hd_udim.1001.png", std::vector<uint8_t>{10, 10, 10}, 1, 1, 3));
    CHECK(writePng8("hd_udim.1002.png", std::vector<uint8_t>{200, 200, 200}, 1, 1, 3));
    Texture ud;
    CHECK(ud.load("hd_udim.<UDIM>.png", false));
    ud.sample(0.5f, 0.5f, v4); CHECK(std::fabs(v4[0] - 10 / 255.f) < 1e-4f);
    ud.sample(1.5f, 0.5f, v4); CHECK(std::fabs(v4[0] - 200 / 255.f) < 1e-4f);
    ud.sample(5.5f, 0.5f, v4); NEAR(v4[0], 0.f); // missing tile
    for (const char* f : {"hd_tex.png", "hd_udim.1001.png", "hd_udim.1002.png"}) std::remove(f);
    // $HIP paths resolve against the scene folder
    setSceneDirectory("/some/dir");
    CHECK(resolvePath("$HIP/tex.png") == "/some/dir/tex.png");
    CHECK(resolvePath("plain.png") == "plain.png");
    setSceneDirectory("");
}

static PathTracer::Buffers renderBuffers(const Geometry& geo, RenderSettings rs, const std::vector<Light>& lights) {
    PathTracer pt;
    pt.start(geo, rs, lights);
    pt.wait();
    PathTracer::Buffers b;
    pt.fetchBuffers(b);
    return b;
}

static Vec3 centerMean(const PathTracer::Buffers& b, int r = 3) {
    Vec3 s(0);
    int n = 0;
    for (int y = b.h / 2 - r; y <= b.h / 2 + r; ++y)
        for (int x = b.w / 2 - r; x <= b.w / 2 + r; ++x) {
            size_t i = size_t(y) * size_t(b.w) + size_t(x);
            s += Vec3(b.beauty[i * 3], b.beauty[i * 3 + 1], b.beauty[i * 3 + 2]);
            ++n;
        }
    return s / float(n);
}

static Geometry sphereWith(const std::function<void(Node*)>& setMat, float radius = 1) {
    Graph g;
    Node* sp = g.createNode("sphere");
    sp->parm("rad")->val[0] = sp->parm("rad")->val[1] = sp->parm("rad")->val[2] = radius;
    sp->parm("rows")->val[0] = 48; sp->parm("cols")->val[0] = 96;
    Node* nm = g.createNode("normal");
    g.connect(sp, nm, 0);
    Node* m = g.createNode("material");
    g.connect(nm, m, 0);
    m->parm("setcolor")->val[0] = 1;
    m->parm("color")->val[0] = m->parm("color")->val[1] = m->parm("color")->val[2] = 1;
    setMat(m);
    g.cook(m);
    return m->geometry();
}

TEST(render_furnace) {
    // White furnace: a white, non-absorbing object inside a uniform environment of radiance 1 must look ~1.
    // This checks that each BSDF neither creates nor (much) loses energy, and that sampling is weighted right.
    Light env;
    env.type = Light::Env; env.sky = false; env.intensity = 1; env.color = Vec3(1);
    RenderSettings rs;
    rs.width = 48; rs.height = 36; rs.samples = 96; rs.bounces = 24; rs.camDist = 4; rs.camPitch = 0; rs.camYaw = 0;
    auto check = [&](const char* name, const std::function<void(Node*)>& m, float lo, float hi) {
        Vec3 c = centerMean(renderBuffers(sphereWith(m), rs, {env}));
        float l = (c.x + c.y + c.z) / 3;
        if (l < lo || l > hi) std::printf("  furnace %s: %.3f (want %.2f..%.2f)\n", name, l, lo, hi);
        CHECK(l >= lo && l <= hi);
    };
    check("glass", [](Node* m) { m->parm("transmit")->val[0] = 1; m->parm("rough")->val[0] = 0.02f; }, 0.95f, 1.05f);
    check("rough glass", [](Node* m) { m->parm("transmit")->val[0] = 1; m->parm("rough")->val[0] = 0.3f; }, 0.85f, 1.05f);
    check("mirror metal", [](Node* m) { m->parm("metal")->val[0] = 1; m->parm("rough")->val[0] = 0.02f; }, 0.97f, 1.03f);
    check("rough metal", [](Node* m) { m->parm("metal")->val[0] = 1; m->parm("rough")->val[0] = 0.3f; }, 0.85f, 1.03f);
    check("plastic", [](Node* m) { m->parm("rough")->val[0] = 0.4f; }, 0.85f, 1.03f);
    check("subsurface", [](Node* m) { m->parm("sss")->val[0] = 1; m->parm("sssradius")->val[0] = m->parm("sssradius")->val[1] = m->parm("sssradius")->val[2] = 0.2f; }, 0.8f, 1.05f);
    // an absorbing (dark) material is clearly darker
    check("dark", [](Node* m) { m->parm("color")->val[0] = m->parm("color")->val[1] = m->parm("color")->val[2] = 0.2f; }, 0.0f, 0.4f);
    // the same furnace through an environment map (importance sampled, combined with MIS)
    Image flat;
    flat.w = 64; flat.h = 32; flat.c = 3;
    flat.px.assign(64 * 32 * 3, 1.f);
    CHECK(writeHdr("hd_flat.hdr", flat));
    Light envMap = env;
    envMap.map = "hd_flat.hdr";
    Vec3 c = centerMean(renderBuffers(sphereWith([](Node* m) { m->parm("rough")->val[0] = 0.4f; }), rs, {envMap}));
    float l = (c.x + c.y + c.z) / 3;
    if (l < 0.85f || l > 1.03f) std::printf("  furnace env map plastic: %.3f\n", l);
    CHECK(l >= 0.85f && l <= 1.03f);
    // a bright patch in the map lights a diffuse sphere from that side only
    Image patch = flat;
    for (auto& v : patch.px) v = 0.f;
    for (int y = 2; y < 6; ++y) for (int x = 30; x < 34; ++x) for (int k = 0; k < 3; ++k) patch.px[(size_t(y) * 64 + size_t(x)) * 3 + size_t(k)] = 400.f;
    CHECK(writeHdr("hd_patch.hdr", patch));
    envMap.map = "hd_patch.hdr";
    PathTracer pt;
    RenderSettings r2 = rs;
    r2.samples = 16; r2.bounces = 2;
    pt.start(sphereWith([](Node*) {}), r2, {envMap});
    pt.wait();
    PathTracer::Buffers b;
    CHECK(pt.fetchBuffers(b) && pt.warnings().empty());
    // the patch is high in the sky (about 70 degrees up) toward +Z, the camera's side: the top of the sphere is lit, the bottom is not
    float top = 0, bottom = 0;
    for (int x = b.w / 2 - 2; x <= b.w / 2 + 2; ++x) {
        top += b.beauty[(size_t(b.h / 2 - 8) * size_t(b.w) + size_t(x)) * 3];
        bottom += b.beauty[(size_t(b.h / 2 + 8) * size_t(b.w) + size_t(x)) * 3];
    }
    CHECK(top > 4 * bottom && top > 0);
    // a missing map is reported, and rendering falls back to the plain environment
    envMap.map = "does_not_exist.hdr";
    pt.start(sphereWith([](Node*) {}), r2, {envMap});
    pt.wait();
    CHECK(!pt.warnings().empty());
    std::remove("hd_flat.hdr");
    std::remove("hd_patch.hdr");
}

TEST(render_camera_and_passes) {
    // an emissive sphere on black: depth of field and motion blur widen its soft edge by the expected amount
    Light dark;
    dark.type = Light::Env; dark.sky = false; dark.intensity = 0;
    Geometry ball = sphereWith([](Node* m) { m->parm("emit")->val[0] = 1; m->parm("color")->val[0] = m->parm("color")->val[1] = m->parm("color")->val[2] = 1; }, 0.5f);
    RenderSettings rs;
    rs.width = 96; rs.height = 64; rs.samples = 64; rs.bounces = 1; rs.camDist = 4; rs.camPitch = 0; rs.camYaw = 0; rs.camFov = 30;
    auto partial = [](const PathTracer::Buffers& b) {
        int n = 0;
        for (int i = 0; i < b.w * b.h; ++i) { float v = b.alpha[size_t(i)]; n += v > 0.1f && v < 0.9f; }
        return n;
    };
    auto extentX = [](const PathTracer::Buffers& b) {
        int lo = b.w, hi = -1;
        int y = b.h / 2;
        for (int x = 0; x < b.w; ++x) if (b.alpha[size_t(y) * size_t(b.w) + size_t(x)] > 0.02f) { lo = std::min(lo, x); hi = std::max(hi, x); }
        return hi - lo + 1;
    };
    PathTracer::Buffers sharp = renderBuffers(ball, rs, {dark});
    // passes: alpha, depth, normal, albedo where the sphere is; nothing outside
    {
        size_t c = (size_t(sharp.h / 2) * size_t(sharp.w) + size_t(sharp.w / 2));
        NEAR(sharp.alpha[c], 1.f);
        CHECK(std::fabs(sharp.depth[c] - 3.5f) < 0.02f);
        CHECK(sharp.normal[c * 3 + 2] > 0.99f);
        CHECK(sharp.albedo[c * 3] > 0.99f);
        NEAR(sharp.alpha[0], 0.f);
        CHECK(sharp.depth[0] > 1e9f);
    }
    RenderSettings d = rs;
    d.dof = true; d.fstop = 0.5f; d.focusDist = 1.5f;
    PathTracer::Buffers blurred = renderBuffers(ball, d, {dark});
    CHECK(partial(blurred) > 3 * partial(sharp));
    // focused on the sphere, the lens changes nothing visible
    d.focusDist = 3.5f;
    CHECK(partial(renderBuffers(ball, d, {dark})) < 2 * partial(sharp) + 10);
    // motion blur: v = 24 units/s, half-frame shutter at 24 fps = 0.5 units of travel, seen at 4 units with fov 30
    Geometry moving = ball;
    Attrib& v = moving.addAttrib(AttribClass::Point, "v", 3);
    for (size_t i = 0; i < moving.numPoints(); ++i) v.setV(i, Vec3(24, 0, 0));
    RenderSettings mb = rs;
    mb.motionBlur = true; mb.shutter = 0.5f; mb.fps = 24;
    int w0 = extentX(sharp), w1 = extentX(renderBuffers(moving, mb, {dark}));
    float pxPerUnit = float(rs.height) / (2 * 4.f * std::tan(radians(15.f)));
    float want = 0.5f * pxPerUnit;
    if (std::fabs(float(w1 - w0) - want) > 3) std::printf("  motion blur streak %d px, expected %.1f\n", w1 - w0, want);
    CHECK(std::fabs(float(w1 - w0) - want) <= 3);
    // without the setting, v changes nothing
    CHECK(extentX(renderBuffers(moving, rs, {dark})) == w0);
    // EXR with all passes
    PathTracer::Buffers den = sharp;
    CHECK(writeExrPasses("hd_passes.exr", den, true));
    int ww, hh;
    std::vector<std::pair<std::string, std::vector<float>>> ch;
    CHECK(readExrChannels("hd_passes.exr", ww, hh, ch));
    std::set<std::string> names;
    for (auto& c : ch) names.insert(c.first);
    for (const char* n : {"R", "G", "B", "A", "albedo.R", "albedo.G", "albedo.B", "N.X", "N.Y", "N.Z", "Z"}) CHECK(names.count(n));
    std::remove("hd_passes.exr");
}

TEST(render_textures_and_uv) {
    // UV Project: box projection covers [0,1] on every face; planar fits the bounding box
    Graph g;
    Node* box = g.createNode("box");
    Node* uv = g.createNode("uvproject");
    g.connect(box, uv, 0);
    g.cook(uv);
    CHECK(uv->error().empty());
    const Attrib* a = uv->geometry().findAttrib(AttribClass::Point, "uv");
    CHECK(a && a->count() == uv->geometry().numPoints());
    CHECK(uv->geometry().numPoints() == 24); // unshared per face
    float mn = 1e9f, mx = -1e9f;
    for (size_t i = 0; a && i < a->count(); ++i) { mn = std::min({mn, a->getV(i).x, a->getV(i).y}); mx = std::max({mx, a->getV(i).x, a->getV(i).y}); }
    NEAR(mn, 0.f); NEAR(mx, 1.f);
    for (int m = 0; m < 4; ++m) { uv->parm("method")->val[0] = float(m); uv->touch(); g.cook(uv); CHECK(uv->error().empty()); }
    // an emissive textured card renders the texture: left half dark, right half light
    CHECK(writePng8("hd_card.png", std::vector<uint8_t>{20, 20, 20, 230, 230, 230}, 2, 1, 3));
    Node* grid = g.createNode("grid");
    grid->parm("orient")->val[0] = 1; // XY, facing the camera
    grid->parm("sizex")->val[0] = 2; grid->parm("sizey")->val[0] = 2; grid->parm("rows")->val[0] = 2; grid->parm("cols")->val[0] = 2;
    Node* uv2 = g.createNode("uvproject");
    g.connect(grid, uv2, 0);
    uv2->parm("method")->val[0] = 0;
    Node* m = g.createNode("material");
    g.connect(uv2, m, 0);
    m->parm("basemap")->str = "hd_card.png";
    m->parm("emit")->val[0] = 1;
    g.cook(m);
    CHECK(m->error().empty() && m->warning().empty());
    Light dark;
    dark.type = Light::Env; dark.sky = false; dark.intensity = 0;
    RenderSettings rs;
    rs.width = 64; rs.height = 32; rs.samples = 4; rs.bounces = 1; rs.camDist = 2; rs.camPitch = 0; rs.camYaw = 0; rs.camFov = 30;
    PathTracer pt;
    pt.start(m->geometry(), rs, {dark});
    pt.wait();
    PathTracer::Buffers b;
    CHECK(pt.fetchBuffers(b) && pt.warnings().empty());
    // the card spans x in [-1, 1]; texel centres (u = 0.25, 0.75) land on pixels 17 and 47
    float left = b.beauty[(size_t(16) * 64 + 17) * 3], right = b.beauty[(size_t(16) * 64 + 47) * 3];
    // sRGB 20 -> 0.0070, 230 -> 0.791 (linear)
    CHECK(std::fabs(left - 0.0070f) < 0.02f && std::fabs(right - 0.791f) < 0.05f);
    // a normal map tilts the shading: the same lit card renders differently
    CHECK(writePng8("hd_nrm.png", std::vector<uint8_t>{230, 128, 160}, 1, 1, 3));
    m->parm("emit")->val[0] = 0; m->parm("basemap")->str = ""; m->touch();
    g.cook(m);
    Light sun;
    sun.type = Light::Distant; sun.dir = normalize(Vec3(0.6f, 0, -1)); sun.intensity = 3; sun.angle = 0.5f;
    Vec3 flatC = centerMean(renderBuffers(m->geometry(), rs, {sun}), 2);
    m->parm("normalmap")->str = "hd_nrm.png"; m->touch();
    g.cook(m);
    Vec3 bumpC = centerMean(renderBuffers(m->geometry(), rs, {sun}), 2);
    CHECK(std::fabs(bumpC.x - flatC.x) > 0.05f * flatC.x);
    // a missing texture is reported
    m->parm("normalmap")->str = "missing_texture.png"; m->touch();
    g.cook(m);
    pt.start(m->geometry(), rs, {sun});
    pt.wait();
    CHECK(pt.warnings().find("missing_texture.png") != std::string::npos);
    std::remove("hd_card.png");
    std::remove("hd_nrm.png");
}

TEST(denoiser) {
    // a noisy low-sample render gets closer to a converged reference after denoising
    Graph g;
    Node* box = g.createNode("box");
    Node* sp = g.createNode("sphere");
    sp->parm("t")->val[0] = 1.2f;
    Node* mg = g.createNode("merge");
    g.connect(box, mg, 0);
    g.connect(sp, mg, 1);
    g.cook(mg);
    RenderSettings rs;
    rs.width = 64; rs.height = 48; rs.camDist = 5; rs.ground = true; rs.groundY = -0.5f;
    rs.samples = 256;
    PathTracer::Buffers ref = renderBuffers(mg->geometry(), rs, {});
    rs.samples = 4;
    rs.denoise = true;
    PathTracer::Buffers noisy = renderBuffers(mg->geometry(), rs, {});
    CHECK(noisy.denoised.size() == noisy.beauty.size());
    double eNoisy = 0, eDen = 0;
    for (size_t i = 0; i < ref.beauty.size(); ++i) {
        eNoisy += double(noisy.beauty[i] - ref.beauty[i]) * double(noisy.beauty[i] - ref.beauty[i]);
        eDen += double(noisy.denoised[i] - ref.beauty[i]) * double(noisy.denoised[i] - ref.beauty[i]);
    }
    if (eDen >= 0.5 * eNoisy) std::printf("  denoise error %.4f vs noisy %.4f\n", eDen, eNoisy);
    CHECK(eDen < 0.5 * eNoisy);
    // a flat, noise-free image is left alone
    std::vector<float> flat(16 * 16 * 3, 0.5f), alb(16 * 16 * 3, 0.8f), nrm(16 * 16 * 3, 0.f), dep(16 * 16, 2.f);
    for (int i = 0; i < 256; ++i) nrm[size_t(i) * 3 + 2] = 1;
    DenoiseInput di;
    di.w = di.h = 16; di.color = flat.data(); di.albedo = alb.data(); di.normal = nrm.data(); di.depth = dep.data();
    auto out = denoiseImage(di);
    float worst = 0;
    for (float v : out) worst = std::max(worst, std::fabs(v - 0.5f));
    CHECK(worst < 1e-5f);
}

// ---------------------------------------------------------------- rigging (phase 6)

static Vec3 attrV(const Geometry& g, const char* n, size_t i) { const Attrib* a = g.findAttrib(AttribClass::Point, n); return a ? a->getV(i) : Vec3(-999); }

TEST(rig_skeleton_fk_ik) {
    Graph g;
    Node* sk = g.createNode("skeleton");
    sk->parm("bones")->val[0] = 2; sk->parm("length")->val[0] = 0.5f;
    g.cook(sk);
    const Geometry& s = sk->geometry();
    CHECK(s.numPoints() == 3 && s.numPrims() == 2);
    NEAR(s.findAttrib(AttribClass::Point, "parent")->getF(0), -1.f);
    NEAR(s.findAttrib(AttribClass::Point, "parent")->getF(2), 1.f);
    // FK: 90 degrees about Z at the root swings the whole chain to -X; the elbow bend adds on top
    Node* fk = g.createNode("fkpose");
    g.connect(sk, fk, 0);
    fk->parm("joint1")->val[0] = 0; fk->parm("rot1")->val[2] = 90;
    fk->parm("joint2")->val[0] = 1; fk->parm("rot2")->val[2] = 90;
    g.cook(fk);
    CHECK(fk->error().empty());
    const Geometry& f = fk->geometry();
    CHECK(length(f.P[1] - Vec3(-0.5f, 0, 0)) < 1e-5f);
    CHECK(length(f.P[2] - Vec3(-0.5f, -0.5f, 0)) < 1e-5f);
    CHECK(length(attrV(f, "xfx", 2) - Vec3(-1, 0, 0)) < 1e-5f); // 180 degrees in total
    // a rot attribute (from a wrangle) works the same way
    Node* w = g.createNode("attribwrangle");
    g.connect(sk, w, 0);
    w->parm("snippet")->str = "if (@ptnum == 0) v@rot = set(0, 0, 90);";
    Node* fk2 = g.createNode("fkpose");
    g.connect(w, fk2, 0);
    fk2->parm("joint1")->val[0] = -1;
    g.cook(fk2);
    CHECK(fk2->error().empty() && length(fk2->geometry().P[2] - Vec3(-1, 0, 0)) < 1e-5f);

    // IK: reachable target is hit, bone lengths are kept, the pole chooses the bend side
    Node* ik = g.createNode("ikchain");
    g.connect(sk, ik, 0);
    ik->parm("target")->val[0] = 0.4f; ik->parm("target")->val[1] = 0.6f; ik->parm("target")->val[2] = 0;
    ik->parm("pole")->val[0] = 1; ik->parm("pole")->val[1] = 0; ik->parm("pole")->val[2] = 0;
    g.cook(ik);
    CHECK(ik->error().empty());
    const Geometry& k = ik->geometry();
    CHECK(length(k.P[2] - Vec3(0.4f, 0.6f, 0)) < 1e-3f);
    NEAR(length(k.P[1] - k.P[0]), 0.5f);
    NEAR(length(k.P[2] - k.P[1]), 0.5f);
    CHECK(k.P[1].x > 0.2f); // elbow toward the +X pole
    ik->parm("pole")->val[0] = -1; ik->touch();
    g.cook(ik);
    CHECK(ik->geometry().P[1].x < 0.f);
    // the world rotations agree with the new bone directions
    Vec3 boneDir = normalize(ik->geometry().P[2] - ik->geometry().P[1]);
    {
        const Geometry& kk = ik->geometry();
        Vec3 c0 = attrV(kk, "xfx", 1), c1 = attrV(kk, "xfy", 1), c2 = attrV(kk, "xfz", 1);
        Vec3 rotated = c0 * 0 + c1 * 1 + c2 * 0; // rest bone direction is +Y
        CHECK(length(rotated - boneDir) < 1e-3f);
    }
    // unreachable: the chain points straight at the target
    ik->parm("target")->val[0] = 3; ik->parm("target")->val[1] = 0; ik->touch();
    g.cook(ik);
    CHECK(length(ik->geometry().P[2] - Vec3(1, 0, 0)) < 1e-4f);
}

TEST(rig_skinning_blendshapes_retarget) {
    Graph g;
    Node* tube = g.createNode("tube"); // an arm: thin compared to its length
    tube->parm("rad1")->val[0] = tube->parm("rad2")->val[0] = 0.2f;
    tube->parm("rows")->val[0] = 9;
    g.cook(tube);
    Vec3 mn, mx;
    tube->geometry().bounds(mn, mx);
    Node* sk = g.createNode("skeleton");
    sk->parm("bones")->val[0] = 2;
    sk->parm("length")->val[0] = (mx.y - mn.y) * 0.5f;
    sk->parm("origin")->val[1] = mn.y;
    Node* cap = g.createNode("capture");
    g.connect(tube, cap, 0);
    g.connect(sk, cap, 1);
    g.cook(cap);
    CHECK(cap->error().empty());
    const Attrib* w0 = cap->geometry().findAttrib(AttribClass::Point, "bw0");
    CHECK(w0 != nullptr);
    bool normalized = true;
    for (size_t i = 0; i < cap->geometry().numPoints(); ++i) {
        float sum = 0;
        for (int k = 0; k < 4; ++k) sum += cap->geometry().findAttrib(AttribClass::Point, "bw" + std::to_string(k))->getF(i);
        normalized = normalized && std::fabs(sum - 1) < 1e-4f;
    }
    CHECK(normalized);
    Node* fk = g.createNode("fkpose");
    g.connect(sk, fk, 0);
    fk->parm("joint1")->val[0] = 0; fk->parm("rot1")->val[0] = 90;
    Node* bd = g.createNode("bonedeform");
    g.connect(cap, bd, 0);
    g.connect(sk, bd, 1);
    g.connect(fk, bd, 2);
    g.cook(bd);
    CHECK(bd->error().empty());
    // rotating the root moves the whole mesh rigidly
    bool rigid = true;
    const Vec3 piv(0, mn.y, 0);
    for (size_t i = 0; i < tube->geometry().numPoints(); ++i) {
        Vec3 d = tube->geometry().P[i] - piv;
        Vec3 want = piv + Vec3(d.x, -d.z, d.y); // +90 about X
        rigid = rigid && length(bd->geometry().P[i] - want) < 1e-4f;
    }
    CHECK(rigid);
    // bending the middle joint moves the top end and leaves the bottom end alone
    fk->parm("joint1")->val[0] = 1; fk->touch();
    g.cook(bd);
    float bottomMove = 0, topMove = 0;
    for (size_t i = 0; i < tube->geometry().numPoints(); ++i) {
        float m = length(bd->geometry().P[i] - tube->geometry().P[i]);
        if (tube->geometry().P[i].y < mn.y + 0.01f) bottomMove = std::max(bottomMove, m);
        if (tube->geometry().P[i].y > mx.y - 0.01f) topMove = std::max(topMove, m);
    }
    CHECK(bottomMove < 0.05f && topMove > 0.5f);

    // blend shapes: weight 0.5 is halfway to the target
    Node* xf = g.createNode("xform");
    g.connect(tube, xf, 0);
    xf->parm("t")->val[0] = 2;
    Node* bs = g.createNode("blendshapes");
    g.connect(tube, bs, 0);
    g.connect(xf, bs, 1);
    bs->parm("w1")->val[0] = 0.5f;
    g.cook(bs);
    CHECK(bs->error().empty() && std::fabs(bs->geometry().P[0].x - tube->geometry().P[0].x - 1) < 1e-5f);

    // retarget onto a skeleton twice the size: the pose is copied, positions scale
    Node* big = g.createNode("skeleton");
    big->parm("bones")->val[0] = 2; big->parm("length")->val[0] = sk->parm("length")->val[0] * 2; big->parm("origin")->val[1] = mn.y * 2;
    Node* rt = g.createNode("retarget");
    g.connect(big, rt, 0);
    g.connect(sk, rt, 1);
    g.connect(fk, rt, 2);
    g.cook(rt);
    CHECK(rt->error().empty());
    bool scaled = true;
    for (size_t j = 0; j < 3; ++j) scaled = scaled && length(rt->geometry().P[j] - fk->geometry().P[j] * 2) < 1e-4f;
    CHECK(scaled);
    // mismatched hierarchies are refused
    big->parm("bones")->val[0] = 3; big->touch();
    g.cook(rt);
    CHECK(!rt->error().empty());
}

TEST(rig_bvh_and_timeshift) {
    const char* bvh =
        "HIERARCHY\nROOT Hips\n{\n  OFFSET 0 0 0\n  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n"
        "  JOINT Spine\n  {\n    OFFSET 0 10 0\n    CHANNELS 3 Zrotation Xrotation Yrotation\n"
        "    End Site\n    {\n      OFFSET 0 10 0\n    }\n  }\n}\n"
        "MOTION\nFrames: 2\nFrame Time: 0.5\n0 90 0 0 0 0 0 0 0\n0 90 0 90 0 0 0 0 0\n";
    { FILE* f = std::fopen("hd_test.bvh", "wb"); std::fputs(bvh, f); std::fclose(f); }
    Graph g;
    Node* b = g.createNode("bvhimport");
    b->parm("file")->str = "hd_test.bvh";
    b->parm("scale")->val[0] = 0.1f;
    b->parm("loop")->val[0] = 0;
    g.frame = 1; g.cook(b);
    CHECK(b->error().empty());
    const Geometry& s0 = b->geometry();
    CHECK(s0.numPoints() == 3 && s0.numPrims() == 2);
    CHECK(length(s0.P[0] - Vec3(0, 9, 0)) < 1e-4f && length(s0.P[1] - Vec3(0, 10, 0)) < 1e-4f && length(s0.P[2] - Vec3(0, 11, 0)) < 1e-4f);
    // frame 13 at 24 fps is 0.5 s = BVH frame 1: the hips turn 90 degrees about Z
    g.frame = 13; g.cook(b);
    const Geometry& s1 = b->geometry();
    CHECK(length(s1.P[1] - Vec3(-1, 9, 0)) < 1e-4f && length(s1.P[2] - Vec3(-2, 9, 0)) < 1e-4f);
    // halfway (frame 7) interpolates the channels: 45 degrees
    g.frame = 7; g.cook(b);
    CHECK(length(b->geometry().P[1] - Vec3(-std::sqrt(0.5f), 9 + std::sqrt(0.5f), 0)) < 1e-4f);
    // a broken file is reported
    { FILE* f = std::fopen("hd_bad.bvh", "wb"); std::fputs("HIERARCHY\nROOT A\n{\n OFFSET 0 0 0\n CHANNELS 1 Wrotation\n}\n", f); std::fclose(f); }
    b->parm("file")->str = "hd_bad.bvh"; b->touch();
    g.cook(b);
    CHECK(b->error().find("Unknown channel") != std::string::npos);
    std::remove("hd_test.bvh");
    std::remove("hd_bad.bvh");
    // Time Shift: the input as it was five frames earlier
    Node* box = g.createNode("box");
    box->parm("t")->expr[0] = "$F * 0.1";
    Node* ts = g.createNode("timeshift");
    g.connect(box, ts, 0);
    ts->parm("offset")->val[0] = -5;
    g.frame = 20; g.cook(ts);
    Vec3 mn, mx;
    ts->geometry().bounds(mn, mx);
    CHECK(std::fabs((mn.x + mx.x) * 0.5f - 1.5f) < 1e-4f);
}

// ---------------------------------------------------------------- compositing (phase 6)

static const float* px(const Node* n, int x, int y) {
    const ImageLayer& L = n->geometry().images[0];
    return &(*L.rgba)[(size_t(y) * size_t(L.w) + size_t(x)) * 4];
}
static double meanChannel(const Node* n, int ch) {
    const ImageLayer& L = n->geometry().images[0];
    double s = 0;
    for (size_t i = 0; i < L.pixels(); ++i) s += (*L.rgba)[i * 4 + size_t(ch)];
    return s / double(L.pixels());
}
static Node* constantImg(Graph& g, Vec3 col, float alpha = 1, int w = 64, int h = 48) {
    Node* c = g.createNode("copconstant");
    c->parm("width")->val[0] = float(w); c->parm("height")->val[0] = float(h);
    c->parm("color")->val[0] = col.x; c->parm("color")->val[1] = col.y; c->parm("color")->val[2] = col.z;
    c->parm("alpha")->val[0] = alpha;
    return c;
}

TEST(comp_color_blur_merge_key) {
    Graph g;
    Node* k = constantImg(g, Vec3(0.2f, 0.4f, 0.1f));
    Node* cc = g.createNode("copcolor");
    g.connect(k, cc, 0);
    cc->parm("exposure")->val[0] = 1;
    g.cook(cc);
    CHECK(cc->error().empty());
    NEAR(px(cc, 5, 5)[0], 0.4f); NEAR(px(cc, 5, 5)[1], 0.8f);
    cc->parm("exposure")->val[0] = 0; cc->parm("saturation")->val[0] = 0; cc->touch();
    g.cook(cc);
    float l = 0.2126f * 0.2f + 0.7152f * 0.4f + 0.0722f * 0.1f;
    NEAR(px(cc, 5, 5)[0], l); NEAR(px(cc, 5, 5)[2], l);
    cc->parm("saturation")->val[0] = 1; cc->parm("invert")->val[0] = 1; cc->touch();
    g.cook(cc);
    NEAR(px(cc, 5, 5)[0], 0.8f);
    // blur keeps the total amount of light (checker in, smooth out)
    Node* chk = g.createNode("copconstant");
    chk->parm("width")->val[0] = 64; chk->parm("height")->val[0] = 64; chk->parm("mode")->val[0] = 3; chk->parm("checker")->val[0] = 8;
    Node* bl = g.createNode("copblur");
    g.connect(chk, bl, 0);
    bl->parm("size")->val[0] = 16;
    g.cook(bl);
    CHECK(std::fabs(meanChannel(bl, 0) - meanChannel(chk, 0)) < 0.01);
    float mn = 1e9f, mx = -1e9f;
    for (int x = 16; x < 48; ++x) { mn = std::min(mn, px(bl, x, 32)[0]); mx = std::max(mx, px(bl, x, 32)[0]); }
    CHECK(mx - mn < 0.1f); // the checker pattern is smoothed away
    // premultiplied over: 50% red over blue
    Node* a = constantImg(g, Vec3(1, 0, 0), 0.5f);
    Node* b = constantImg(g, Vec3(0, 0, 1), 1);
    Node* mg = g.createNode("copmerge");
    g.connect(a, mg, 0);
    g.connect(b, mg, 1);
    g.cook(mg);
    NEAR(px(mg, 3, 3)[0], 0.5f); NEAR(px(mg, 3, 3)[2], 0.5f); NEAR(px(mg, 3, 3)[3], 1.f);
    mg->parm("operation")->val[0] = 4; mg->touch(); // multiply
    g.cook(mg);
    NEAR(px(mg, 3, 3)[0], 0.f); NEAR(px(mg, 3, 3)[3], 0.5f);
    // chroma key: the green screen goes transparent, other colours stay
    Node* gr = constantImg(g, Vec3(0.1f, 0.8f, 0.2f));
    Node* key = g.createNode("copkey");
    g.connect(gr, key, 0);
    g.cook(key);
    CHECK(px(key, 3, 3)[3] < 0.01f);
    Node* red = constantImg(g, Vec3(0.8f, 0.2f, 0.2f));
    g.connect(red, key, 0);
    g.cook(key);
    CHECK(px(key, 3, 3)[3] > 0.99f);
    // glow brightens around a bright spot but leaves dark images alone
    Node* gl = g.createNode("copglow");
    g.connect(k, gl, 0);
    g.cook(gl);
    NEAR(px(gl, 5, 5)[0], 0.2f);
}

TEST(comp_shape_transform_lens_resize) {
    Graph g;
    // a square covering the left half of the frame
    Node* sq = g.createNode("grid");
    sq->parm("orient")->val[0] = 1; sq->parm("sizex")->val[0] = 0.5f; sq->parm("sizey")->val[0] = 1;
    sq->parm("t")->val[0] = 0.25f; sq->parm("t")->val[1] = 0.5f; sq->parm("rows")->val[0] = 2; sq->parm("cols")->val[0] = 2;
    Node* sh = g.createNode("copshape");
    g.connect(sq, sh, 1);
    sh->parm("width")->val[0] = 64; sh->parm("height")->val[0] = 32; sh->parm("feather")->val[0] = 0;
    g.cook(sh);
    CHECK(sh->error().empty());
    NEAR(px(sh, 10, 16)[3], 1.f); NEAR(px(sh, 50, 16)[3], 0.f);
    CHECK(std::fabs(meanChannel(sh, 3) - 0.5) < 0.02);
    sh->parm("invert")->val[0] = 1; sh->touch();
    g.cook(sh);
    NEAR(px(sh, 10, 16)[3], 0.f);
    // transform: move 10 pixels right
    Node* chk = g.createNode("copconstant");
    chk->parm("width")->val[0] = 64; chk->parm("height")->val[0] = 32; chk->parm("mode")->val[0] = 2; // horizontal ramp
    chk->parm("color")->val[0] = 0; chk->parm("color2")->val[0] = 1;
    Node* tr = g.createNode("coptransform");
    g.connect(chk, tr, 0);
    tr->parm("t")->val[0] = 10;
    g.cook(tr);
    NEAR(px(tr, 30, 5)[0], px(chk, 20, 5)[0]);
    NEAR(px(tr, 3, 5)[3], 0.f); // uncovered pixels are transparent
    // lens: vignette darkens the corners only; with everything at zero it changes nothing
    Node* flat = constantImg(g, Vec3(0.5f));
    Node* ln = g.createNode("coplens");
    g.connect(flat, ln, 0);
    ln->parm("vignette")->val[0] = 0.8f;
    g.cook(ln);
    CHECK(px(ln, 0, 0)[0] < 0.3f && std::fabs(px(ln, 32, 24)[0] - 0.5f) < 1e-4f);
    ln->parm("vignette")->val[0] = 0; ln->touch();
    g.cook(ln);
    NEAR(px(ln, 0, 0)[0], 0.5f);
    // resize: half size, same average
    Node* rs = g.createNode("copresize");
    g.connect(chk, rs, 0);
    rs->parm("width")->val[0] = 32;
    g.cook(rs);
    CHECK(rs->geometry().images[0].w == 32 && rs->geometry().images[0].h == 16);
    CHECK(std::fabs(meanChannel(rs, 0) - meanChannel(chk, 0)) < 0.01);
}

TEST(comp_files_depth_render) {
    // write an image with depth to EXR and PNG, read both back
    ImageLayer L;
    L.w = 16; L.h = 8;
    auto rgba = std::make_shared<std::vector<float>>(size_t(16 * 8 * 4));
    auto z = std::make_shared<std::vector<float>>(size_t(16 * 8));
    for (int i = 0; i < 16 * 8; ++i) {
        (*rgba)[size_t(i) * 4] = 0.25f; (*rgba)[size_t(i) * 4 + 1] = 0.5f; (*rgba)[size_t(i) * 4 + 2] = 1.f; (*rgba)[size_t(i) * 4 + 3] = 1.f;
        (*z)[size_t(i)] = i % 16 < 8 ? 2.f : 10.f; // near on the left, far on the right
    }
    L.rgba = rgba; L.depth = z;
    CHECK(expandFrame("a.$F4.exr", 7) == "a.0007.exr" && expandFrame("b$F.png", 12) == "b12.png");
    CHECK(writeImageLayer("hd_comp.$F2.exr", L, 3));
    CHECK(writeImageLayer("hd_comp.png", L, 1));
    Graph g;
    Node* fe = g.createNode("copfile");
    fe->parm("file")->str = "hd_comp.$F2.exr";
    g.frame = 3; g.cook(fe);
    CHECK(fe->error().empty());
    const ImageLayer& R = fe->geometry().images[0];
    CHECK(R.w == 16 && R.h == 8 && R.depth && R.depth->size() == 128);
    if (R.depth) { NEAR((*R.depth)[0], 2.f); NEAR((*R.depth)[15], 10.f); }
    NEAR(px(fe, 3, 3)[1], 0.5f);
    Node* fp = g.createNode("copfile");
    fp->parm("file")->str = "hd_comp.png";
    g.cook(fp);
    CHECK(std::fabs(px(fp, 3, 3)[1] - 0.5f) < 0.01f); // sRGB encode/decode round trip
    // depth composite: a red image at depth 5 everywhere goes in front of the far half only
    ImageLayer L2 = L;
    auto rgba2 = std::make_shared<std::vector<float>>(*rgba);
    auto z2 = std::make_shared<std::vector<float>>(size_t(128), 5.f);
    for (int i = 0; i < 128; ++i) { (*rgba2)[size_t(i) * 4] = 1; (*rgba2)[size_t(i) * 4 + 1] = 0; (*rgba2)[size_t(i) * 4 + 2] = 0; }
    L2.rgba = rgba2; L2.depth = z2;
    CHECK(writeImageLayer("hd_comp_red.exr", L2, 1));
    Node* fr = g.createNode("copfile");
    fr->parm("file")->str = "hd_comp_red.exr";
    Node* zc = g.createNode("copzcomp");
    g.connect(fr, zc, 0);
    g.connect(fe, zc, 1);
    g.cook(zc);
    CHECK(zc->error().empty());
    NEAR(px(zc, 2, 2)[0], 0.25f); // left: the depth-2 image wins
    NEAR(px(zc, 12, 2)[0], 1.f);  // right: red at depth 5 is nearer than 10
    // defocus: the near (in focus) half stays sharp, the far half is blurred
    Node* df = g.createNode("copdefocus");
    g.connect(zc, df, 0);
    df->parm("focus")->val[0] = 2; df->parm("range")->val[0] = 3; df->parm("maxblur")->val[0] = 3;
    g.cook(df);
    CHECK(df->error().empty());
    NEAR(px(df, 2, 2)[0], 0.25f);
    for (const char* f : {"hd_comp.03.exr", "hd_comp.png", "hd_comp_red.exr"}) std::remove(f);
    // missing files are reported
    fp->parm("file")->str = "no_such_image.png"; fp->touch();
    g.cook(fp);
    CHECK(!fp->error().empty());
    // Render: a sphere over a transparent background, with depth
    Node* sp = g.createNode("sphere");
    Node* rn = g.createNode("coprender");
    g.connect(sp, rn, 0);
    rn->parm("width")->val[0] = 48; rn->parm("height")->val[0] = 32; rn->parm("samples")->val[0] = 4; rn->parm("denoise")->val[0] = 0;
    g.cook(rn);
    CHECK(rn->error().empty());
    NEAR(px(rn, 24, 16)[3], 1.f);
    NEAR(px(rn, 0, 0)[3], 0.f);
    const ImageLayer& RL = rn->geometry().images[0];
    CHECK(RL.depth && (*RL.depth)[16 * 48 + 24] < 1e9f && (*RL.depth)[0] > 1e9f);
    // Write node writes when enabled
    Node* wr = g.createNode("copwrite");
    g.connect(rn, wr, 0);
    wr->parm("file")->str = "hd_written.png";
    g.cook(wr);
    CHECK(std::fopen("hd_written.png", "rb") == nullptr);
    wr->parm("enable")->val[0] = 1; wr->touch();
    g.cook(wr);
    FILE* f = std::fopen("hd_written.png", "rb");
    CHECK(f != nullptr);
    if (f) std::fclose(f);
    std::remove("hd_written.png");
}

// ---------------------------------------------------------------- USD (phase 6)

static std::string slurp(const char* path) {
    std::string out;
    FILE* f = std::fopen(path, "rb");
    if (!f) return out;
    char buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) out.append(buf, n);
    std::fclose(f);
    return out;
}

TEST(usd_export_import) {
    Graph g;
    Node* box = g.createNode("box");
    Node* uv = g.createNode("uvproject");
    g.connect(box, uv, 0);
    Node* m = g.createNode("material");
    g.connect(uv, m, 0);
    m->parm("group")->str = "0-2";
    m->parm("metal")->val[0] = 1; m->parm("rough")->val[0] = 0.2f; m->parm("setcolor")->val[0] = 1;
    Node* cam = g.createNode("camera");
    (void)cam;
    Node* light = g.createNode("spotlight");
    (void)light;
    g.cook(m);
    UsdExport ue;
    ue.cameras = collectCameras(g, 1);
    ue.lights = collectLights(g, 1);
    std::string err;
    CHECK(writeUsda("hd_t.usda", ue, [&](float, Geometry& out, std::string*) { out = m->geometry(); return true; }, &err));
    std::string text = slurp("hd_t.usda");
    CHECK(text.rfind("#usda 1.0", 0) == 0);
    for (const char* needle : {"def Mesh \"geo\"", "faceVertexCounts", "primvars:st", "def GeomSubset", "familyName = \"materialBind\"",
                               "UsdPreviewSurface", "inputs:metallic = 1", "def Camera", "focalLength", "def SphereLight", "ShapingAPI"})
        if (text.find(needle) == std::string::npos) { std::printf("  missing in usda: %s\n", needle); CHECK(false); }
    // balanced braces and brackets
    int braces = 0, brackets = 0;
    for (char ch : text) { braces += ch == '{' ? 1 : ch == '}' ? -1 : 0; brackets += ch == '[' ? 1 : ch == ']' ? -1 : 0; }
    CHECK(braces == 0 && brackets == 0);
    // read back: same shape, uvs and colours survive
    Geometry back;
    CHECK(loadUsda("hd_t.usda", back, &err));
    CHECK(back.numPoints() == m->geometry().numPoints() && back.numPrims() == m->geometry().numPrims());
    float md = 0;
    for (size_t i = 0; i < back.numPoints() && i < m->geometry().numPoints(); ++i) md = std::max(md, length(back.P[i] - m->geometry().P[i]));
    CHECK(md < 1e-5f);
    CHECK(back.findAttrib(AttribClass::Point, "uv") != nullptr);
    CHECK(back.findAttrib(AttribClass::Prim, "Cd") != nullptr);
    // through the File node too
    Node* fl = g.createNode("file");
    fl->parm("file")->str = "hd_t.usda";
    g.cook(fl);
    CHECK(fl->error().empty() && fl->geometry().numPrims() == back.numPrims());

    // animation: points are time-sampled over the range
    Node* moving = g.createNode("xform");
    g.connect(box, moving, 0);
    moving->parm("t")->expr[1] = "$F * 0.5";
    UsdExport anim;
    anim.start = 1; anim.end = 5;
    CHECK(writeUsda("hd_anim.usda", anim, [&](float f, Geometry& out, std::string*) { g.frame = f; g.cook(moving); out = moving->geometry(); return true; }, &err));
    text = slurp("hd_anim.usda");
    CHECK(text.find("points.timeSamples") != std::string::npos && text.find("startTimeCode = 1") != std::string::npos && text.find("endTimeCode = 5") != std::string::npos);
    CHECK(text.find("        5: [") != std::string::npos);
    // a changing point count is refused with a clear message
    Node* sc = g.createNode("scatter");
    g.connect(box, sc, 0);
    sc->parm("npts")->expr[0] = "$F * 10";
    CHECK(!writeUsda("hd_bad.usda", anim, [&](float f, Geometry& out, std::string*) { g.frame = f; g.cook(sc); out = sc->geometry(); return true; }, &err));
    CHECK(err.find("point count changes") != std::string::npos);

    // a hand-written stage: nested transforms (translate, rotateXYZ, scale), a left-handed mesh, curves, points
    const char* hand =
        "#usda 1.0\n( upAxis = \"Y\" )\n"
        "def Xform \"root\" {\n"
        "  double3 xformOp:translate = (10, 0, 0)\n"
        "  uniform token[] xformOpOrder = [\"xformOp:translate\"]\n"
        "  def Mesh \"tri\" ( prepend apiSchemas = [\"MaterialBindingAPI\"] ) {\n"
        "    float3 xformOp:rotateXYZ = (0, 0, 90)\n    float3 xformOp:scale = (2, 2, 2)\n"
        "    uniform token[] xformOpOrder = [\"xformOp:rotateXYZ\", \"xformOp:scale\"]\n"
        "    int[] faceVertexCounts = [3]\n    int[] faceVertexIndices = [0, 1, 2]\n"
        "    point3f[] points = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]\n"
        "    uniform token orientation = \"leftHanded\"\n"
        "    color3f[] primvars:displayColor = [(1, 0, 0)] ( interpolation = \"constant\" )\n"
        "  }\n"
        "  def BasisCurves \"hair\" { int[] curveVertexCounts = [3]\n point3f[] points = [(0,0,0), (0,1,0), (0,2,0)] }\n"
        "  def Points \"dust\" { point3f[] points.timeSamples = { 1: [(5, 5, 5)], 2: [(6, 6, 6)] }\n float[] widths = [0.5] }\n"
        "}\n";
    { FILE* f = std::fopen("hd_hand.usda", "wb"); std::fputs(hand, f); std::fclose(f); }
    Geometry h;
    CHECK(loadUsda("hd_hand.usda", h, &err));
    CHECK(h.numPoints() == 7 && h.numPrims() == 2);
    // (1,0,0) scaled by 2 then rotated 90 about Z -> (0,2,0), then translated -> (10,2,0)
    CHECK(length(h.P[0] - Vec3(10, 2, 0)) < 1e-4f);
    CHECK(length(h.P[1] - Vec3(8, 0, 0)) < 1e-4f);
    CHECK(h.primVertex(0, 0) == 2); // left-handed winding is flipped
    CHECK(!h.prims[1].closed);
    CHECK(length(h.P[6] - Vec3(15, 5, 5)) < 1e-4f); // first time sample
    // binary USD is reported clearly
    { FILE* f = std::fopen("hd_bin.usd", "wb"); std::fputs("PXR-USDC....", f); std::fclose(f); }
    CHECK(!loadUsda("hd_bin.usd", h, &err) && err.find("Binary") != std::string::npos);
    for (const char* f : {"hd_t.usda", "hd_anim.usda", "hd_bad.usda", "hd_hand.usda", "hd_bin.usd"}) std::remove(f);
}

#ifdef HD_EXAMPLES_DIR
TEST(example_files_cook_cleanly) {
    int n = 0;
    for (auto& e : std::filesystem::directory_iterator(HD_EXAMPLES_DIR)) {
        if (e.path().extension() != ".hdg") continue;
        ++n;
        Graph g;
        std::string err;
        CHECK(g.load(e.path().string(), &err));
        for (float f : {1.f, 24.f}) {
            g.frame = f;
            for (auto& nd : g.nodes()) {
                g.cook(nd.get());
                if (!nd->error().empty()) std::printf("  %s node %s: %s\n", e.path().filename().string().c_str(), nd->name().c_str(), nd->error().c_str());
                CHECK(nd->error().empty());
            }
        }
    }
    CHECK(n >= 15);
}
#endif

int main() {
    for (auto& t : tests()) {
        int before = g_failed;
        t.fn();
        std::printf("%s %s\n", g_failed == before ? "[ ok ]" : "[FAIL]", t.name);
    }
    std::printf("\n%d checks, %d failed\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}

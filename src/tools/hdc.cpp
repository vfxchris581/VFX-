// hdc — headless HD command line (like hbatch): cook scenes, export geometry, make demos.
#include "core/Demo.h"
#include "core/GeoIO.h"
#include "core/Lights.h"
#include "core/Node.h"
#include "core/Compositing.h"
#include "core/PathTracer.h"
#include "core/Usd.h"
#include <cstdio>
#include <cstring>
#include <functional>
#include <chrono>
#include "core/Version.h"
#include <string>

using namespace hd;

static void usage() {
    std::printf(
        "hdc %s - HD headless tool\n\n"
        "  hdc info   <scene.hdg> [--frame N]            list nodes and cook stats\n"
        "  hdc cook   <scene.hdg> <out.obj|out.ply> [--node NAME] [--frame N]\n"
        "  hdc frames <scene.hdg> <out_$F.obj> <start> <end> [--node NAME]   export a sequence\n"
        "  hdc render <scene.hdg> <out.png|out.exr> [--frame N | --frames A B] [--samples N] [--size W H] [--node NAME] [--camera NAME]\n"
        "             [--denoise | --no-denoise] [--motionblur]   (.exr writes RGBA plus albedo, N and Z passes)\n"
        "             path-trace the scene (a sequence writes out.0001.png, out.0002.png, ...)\n"
        "  hdc usd    <scene.hdg> <out.usda> [--node NAME] [--frame N | --frames A B]\n"
        "             export to USD (mesh, materials, cameras, lights; animated over a frame range)\n"
        "  hdc image  <scene.hdg> <out.png|out.exr> [--node NAME] [--frame N | --frames A B]\n"
        "             write the image a compositing node makes ($F in the name for sequences)\n"
        "  hdc demo   <name> <out.hdg>                   write a demo scene\n"
        "  hdc bench  [--quick] [--examples DIR]          time standard simulation and render workloads\n"
        "  hdc demos                                     list demo scenes\n"
        "  hdc nodes                                     list node types\n"
        "  hdc --version                                 print version, commit and platform\n", versionString());
}

static const char* opt(int argc, char** argv, const char* name) {
    for (int i = 1; i + 1 < argc; ++i) if (!std::strcmp(argv[i], name)) return argv[i + 1];
    return nullptr;
}

static bool exportGeo(const std::string& path, const Geometry& g, std::string& err) {
    if (path.size() > 4 && path.substr(path.size() - 4) == ".ply") return savePly(path, g, &err);
    return saveObj(path, g, &err);
}


// ---------------------------------------------------------------- bench
// Fixed workloads for measuring performance (results are machine dependent; compare runs on one machine).
static int runBench(const std::string& examplesDir, bool quick) {
    using clock = std::chrono::steady_clock;
    struct Row { std::string name; double ms; std::string info; };
    std::vector<Row> rows;
    auto timeIt = [&](const std::string& name, const std::function<std::string()>& fn) {
        auto t0 = clock::now();
        std::string info = fn();
        double ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
        rows.push_back({name, ms, info});
        std::printf("%-22s %10.1f ms   %s\n", name.c_str(), ms, info.c_str());
        std::fflush(stdout);
    };
    auto sim = [&](const char* file, int frames, const char* node) {
        return [=]() -> std::string {
            Graph g;
            std::string err;
            if (!g.load(examplesDir + "/" + file, &err)) return "load failed: " + err;
            Node* n = g.findNode(node);
            if (!n) return std::string("no node ") + node;
            for (int f = 1; f <= frames; ++f) { g.frame = float(f); g.cook(n); }
            char b[96];
            std::snprintf(b, sizeof b, "%d frames, %zu points%s", frames, n->geometry().numPoints(), n->error().empty() ? "" : " (error)");
            return b;
        };
    };
    const int k = quick ? 1 : 2;
    timeIt("pop (sparks)", sim("sparks.hdg", 48 * k, "sparks"));
    timeIt("pyro (smoke)", sim("smoke.hdg", 30 * k, "smoke"));
    timeIt("rbd (destruction)", sim("destruction.hdg", 30 * k, "rbd"));
    timeIt("flip (dambreak)", sim("dambreak.hdg", 20 * k, "flip"));
    timeIt("fluid surface", [&]() -> std::string {
        Graph g; std::string err;
        g.load(examplesDir + "/dambreak.hdg", &err);
        g.frame = 20;
        Node* n = g.findNode("surface");
        g.cook(g.findNode("flip"));
        auto t0 = clock::now();
        g.cook(n);
        (void)t0;
        char b[64]; std::snprintf(b, sizeof b, "%zu triangles", n->geometry().numPrims());
        return b;
    });
    timeIt("vellum (cloth)", sim("cloth.hdg", 36 * k, "vellum"));
    timeIt("fracture 300 pieces", [&]() -> std::string {
        Graph g;
        Node* t = g.createNode("torus");
        t->parm("rows")->val[0] = 64; t->parm("cols")->val[0] = 32;
        Node* f = g.createNode("voronoifracture");
        g.connect(t, f, 0);
        f->parm("pieces")->val[0] = 300;
        g.cook(f);
        char b[64]; std::snprintf(b, sizeof b, "%zu polygons", f->geometry().numPrims());
        return b;
    });
    auto render = [&](const std::string& file, int w, int h, int spp) {
        return [=]() -> std::string {
            Graph g; std::string err;
            if (!g.load(examplesDir + "/" + file, &err)) return "load failed";
            Node* d = g.displayNode();
            g.cook(d);
            RenderSettings r = g.render;
            r.width = w; r.height = h; r.samples = spp; r.denoise = false;
            PathTracer pt;
            pt.start(d->geometry(), r, collectLights(g, g.frame));
            pt.wait();
            char b[96]; std::snprintf(b, sizeof b, "%dx%d, %d spp", w, h, spp);
            return std::string(b);
        };
    };
    timeIt("render (materials)", render("materials.hdg", 320, 180, 16 * k));
    timeIt("render (destruction)", render("destruction.hdg", 320, 180, 16 * k));
    timeIt("render 500k triangles", [&]() -> std::string {
        Graph g;
        Node* sp = g.createNode("sphere");
        sp->parm("rows")->val[0] = 500; sp->parm("cols")->val[0] = 1000;
        Node* m = g.createNode("mountain");
        g.connect(sp, m, 0);
        g.cook(m);
        RenderSettings r;
        r.width = 320; r.height = 180; r.samples = 8 * k; r.camDist = 4;
        PathTracer pt;
        pt.start(m->geometry(), r);
        pt.wait();
        char b[64]; std::snprintf(b, sizeof b, "%zu prims", m->geometry().numPrims());
        return b;
    });
    timeIt("comp (render+grade)", [&]() -> std::string {
        Graph g; std::string err;
        g.load(examplesDir + "/comp.hdg", &err);
        g.cook(g.displayNode());
        return g.displayNode()->error().empty() ? "ok" : g.displayNode()->error();
    });
    double total = 0;
    for (auto& r : rows) total += r.ms;
    std::printf("%-22s %10.1f ms\n", "TOTAL", total);
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 2) { usage(); return 1; }
    std::string cmd = argv[1];
    if (cmd == "--version" || cmd == "-v" || cmd == "version") {
        std::printf("HD %s (%s, %s, built %s)\n", versionString(), gitRevision(), platformName(), buildDate());
        return 0;
    }
    if (cmd == "--help" || cmd == "-h" || cmd == "help") { usage(); return 0; }
    if (cmd == "nodes") {
        for (auto& t : nodeTypes())
            std::printf("%-15s %-18s %-11s %s\n", t.name.c_str(), t.label.c_str(), t.category.c_str(), t.description.c_str());
        return 0;
    }
    if (cmd == "demos") {
        for (auto& d : demoScenes()) std::printf("%-10s %s\n", d.name.c_str(), d.description.c_str());
        return 0;
    }
    if (cmd == "demo" && argc >= 4) {
        Graph g;
        if (!buildDemoScene(g, argv[2])) { std::printf("unknown demo '%s'\n", argv[2]); return 1; }
        std::string err;
        if (!g.save(argv[3], &err)) { std::printf("%s\n", err.c_str()); return 1; }
        std::printf("wrote %s\n", argv[3]);
        return 0;
    }
    if (cmd == "render" && argc >= 4) {
        Graph g;
        std::string err;
        if (!g.load(argv[2], &err)) { std::printf("error: %s\n", err.c_str()); return 1; }
        Node* target = g.displayNode();
        if (const char* n = opt(argc, argv, "--node")) target = g.findNode(n);
        if (!target) { std::printf("error: no node to render\n"); return 1; }
        RenderSettings rs = g.render;
        if (const char* s = opt(argc, argv, "--samples")) rs.samples = std::atoi(s);
        rs.fps = g.fps;
        for (int i = 1; i < argc; ++i) {
            if (!std::strcmp(argv[i], "--denoise")) rs.denoise = true;
            if (!std::strcmp(argv[i], "--no-denoise")) rs.denoise = false;
            if (!std::strcmp(argv[i], "--motionblur")) rs.motionBlur = true;
        }
        for (int i = 1; i + 2 < argc; ++i)
            if (!std::strcmp(argv[i], "--size")) { rs.width = std::atoi(argv[i + 1]); rs.height = std::atoi(argv[i + 2]); }
        int a = int(g.frame), b = int(g.frame);
        bool seq = false;
        if (const char* f = opt(argc, argv, "--frame")) a = b = std::atoi(f);
        for (int i = 1; i + 2 < argc; ++i)
            if (!std::strcmp(argv[i], "--frames")) { a = std::atoi(argv[i + 1]); b = std::atoi(argv[i + 2]); seq = true; }
        PathTracer pt;
        for (int f = a; f <= b; ++f) {
            g.frame = float(f);
            g.cook(target);
            if (!target->error().empty()) { std::printf("cook error in %s: %s\n", target->name().c_str(), target->error().c_str()); return 1; }
            RenderSettings r = rs;
            if (!r.hasCamera) { // frame the geometry like the viewport does
                Vec3 mn, mx;
                target->geometry().bounds(mn, mx);
                r.camTarget = (mn + mx) * 0.5f;
                r.camDist = std::max(length(mx - mn) * 0.5f, 0.05f) / std::sin(radians(r.camFov) * 0.5f) * 1.1f;
            }
            // camera node: --camera NAME, else the one saved with the scene
            std::string camName = r.cameraName;
            if (const char* cn = opt(argc, argv, "--camera")) camName = cn;
            bool sizeGiven = false;
            for (int i = 1; i < argc; ++i) sizeGiven |= !std::strcmp(argv[i], "--size");
            for (auto& cam : collectCameras(g, g.frame)) {
                if (cam.name != camName) continue;
                r.useEye = true; r.camEye = cam.eye; r.camTarget = cam.eye + cam.fwd;
                if (!sizeGiven) { r.width = cam.resx; r.height = cam.resy; }
                SceneCamera fitted = cam;
                fitted.resx = r.width; fitted.resy = r.height;
                r.camFov = fitted.fovYDegrees();
                r.sensorHeight = cam.aperture * float(r.height) / float(r.width);
                if (cam.dof) { r.dof = true; r.fstop = cam.fstop; r.focusDist = cam.focus; }
            }
            pt.start(target->geometry(), r, collectLights(g, g.frame));
            pt.wait();
            std::vector<uint8_t> img; int w = 0, h = 0; uint64_t ver = 0;
            pt.fetch(img, w, h, ver);
            std::string path = argv[3];
            if (seq) {
                char num[16]; std::snprintf(num, sizeof num, ".%04d", f);
                size_t dot = path.rfind('.');
                path.insert(dot == std::string::npos ? path.size() : dot, num);
            }
            for (const std::string& wl : {pt.warnings()}) if (!wl.empty()) std::printf("warning: %s", wl.c_str());
            bool exr = path.size() > 4 && (path.substr(path.size() - 4) == ".exr" || path.substr(path.size() - 4) == ".EXR");
            if (exr) {
                PathTracer::Buffers buf;
                if (!pt.fetchBuffers(buf) || !writeExrPasses(path, buf, true, &err)) { std::printf("%s\n", err.c_str()); return 1; }
            } else if (!writePng(path, img, w, h, &err)) { std::printf("%s\n", err.c_str()); return 1; }
            std::printf("frame %d -> %s (%dx%d, %d samples, %.1fs)\n", f, path.c_str(), w, h, pt.samplesDone(), pt.secondsElapsed());
        }
        return 0;
    }
    if (cmd == "bench") {
        std::string dir = "examples";
        if (const char* d = opt(argc, argv, "--examples")) dir = d;
        bool quick = false;
        for (int i = 1; i < argc; ++i) quick |= !std::strcmp(argv[i], "--quick");
        return runBench(dir, quick);
    }
    if (cmd == "usd" && argc >= 4) {
        Graph g;
        std::string err;
        if (!g.load(argv[2], &err)) { std::printf("error: %s\n", err.c_str()); return 1; }
        Node* target = g.displayNode();
        if (const char* n = opt(argc, argv, "--node")) target = g.findNode(n);
        if (!target) { std::printf("error: no node\n"); return 1; }
        UsdExport ue;
        ue.start = ue.end = int(g.frame);
        if (const char* f = opt(argc, argv, "--frame")) ue.start = ue.end = std::atoi(f);
        for (int i = 1; i + 2 < argc; ++i)
            if (!std::strcmp(argv[i], "--frames")) { ue.start = std::atoi(argv[i + 1]); ue.end = std::atoi(argv[i + 2]); }
        ue.fps = g.fps;
        g.frame = float(ue.start);
        ue.cameras = collectCameras(g, g.frame);
        ue.lights = collectLights(g, g.frame);
        bool ok = writeUsda(argv[3], ue, [&](float f, Geometry& out, std::string* e) {
            g.frame = f;
            g.cook(target);
            if (!target->error().empty()) { if (e) *e = target->name() + ": " + target->error(); return false; }
            out = target->geometry();
            return true;
        }, &err);
        if (!ok) { std::printf("error: %s\n", err.c_str()); return 1; }
        std::printf("%s frames %d-%d -> %s\n", target->name().c_str(), ue.start, ue.end, argv[3]);
        return 0;
    }
    if (cmd == "image" && argc >= 4) {
        // write the image a (compositing) node produces
        Graph g;
        std::string err;
        if (!g.load(argv[2], &err)) { std::printf("error: %s\n", err.c_str()); return 1; }
        Node* target = g.displayNode();
        if (const char* n = opt(argc, argv, "--node")) target = g.findNode(n);
        if (!target) { std::printf("error: no node\n"); return 1; }
        int a = int(g.frame), b = int(g.frame);
        bool seq = false;
        if (const char* f = opt(argc, argv, "--frame")) a = b = std::atoi(f);
        for (int i = 1; i + 2 < argc; ++i)
            if (!std::strcmp(argv[i], "--frames")) { a = std::atoi(argv[i + 1]); b = std::atoi(argv[i + 2]); seq = true; }
        for (int f = a; f <= b; ++f) {
            g.frame = float(f);
            g.cook(target);
            if (!target->error().empty()) { std::printf("cook error in %s: %s\n", target->name().c_str(), target->error().c_str()); return 1; }
            if (target->geometry().images.empty()) { std::printf("error: %s has no image\n", target->name().c_str()); return 1; }
            std::string path = argv[3];
            if (seq && path.find("$F") == std::string::npos) {
                char num[16]; std::snprintf(num, sizeof num, ".%04d", f);
                size_t dot = path.rfind('.');
                path.insert(dot == std::string::npos ? path.size() : dot, num);
            }
            if (!writeImageLayer(path, target->geometry().images[0], float(f), &err)) { std::printf("%s\n", err.c_str()); return 1; }
            std::printf("frame %d -> %s (%d x %d)\n", f, expandFrame(path, float(f)).c_str(), target->geometry().images[0].w, target->geometry().images[0].h);
        }
        return 0;
    }
    if ((cmd == "info" || cmd == "cook" || cmd == "frames") && argc >= 3) {
        Graph g;
        std::string err;
        if (!g.load(argv[2], &err)) { std::printf("error: %s\n", err.c_str()); return 1; }
        if (!err.empty()) std::printf("warning: %s\n", err.c_str());
        if (const char* f = opt(argc, argv, "--frame")) g.frame = float(std::atof(f));
        Node* target = g.displayNode();
        if (const char* n = opt(argc, argv, "--node")) target = g.findNode(n);
        if (!target) { std::printf("error: no node to cook\n"); return 1; }
        if (cmd == "info") {
            for (auto& n : g.nodes()) g.cook(n.get());
            for (auto& n : g.nodes()) {
                std::printf("%c %-18s %-15s %7zu pts %7zu prims %8.2f ms %s\n", n.get() == g.displayNode() ? '*' : ' ',
                            n->name().c_str(), n->type().name.c_str(), n->geometry().numPoints(), n->geometry().numPrims(),
                            n->cookTimeMs(), n->error().c_str());
            }
            return 0;
        }
        if (cmd == "cook" && argc >= 4) {
            g.cook(target);
            if (!target->error().empty()) { std::printf("cook error in %s: %s\n", target->name().c_str(), target->error().c_str()); return 1; }
            if (!exportGeo(argv[3], target->geometry(), err)) { std::printf("%s\n", err.c_str()); return 1; }
            std::printf("%s: %zu points, %zu prims -> %s\n", target->name().c_str(), target->geometry().numPoints(), target->geometry().numPrims(), argv[3]);
            return 0;
        }
        if (cmd == "frames" && argc >= 6) {
            std::string pattern = argv[3];
            int a = std::atoi(argv[4]), b = std::atoi(argv[5]);
            for (int f = a; f <= b; ++f) {
                g.frame = float(f);
                g.cook(target);
                if (!target->error().empty()) { std::printf("frame %d: %s\n", f, target->error().c_str()); return 1; }
                std::string path = pattern;
                char num[16]; std::snprintf(num, sizeof num, "%04d", f);
                size_t pos = path.find("$F");
                if (pos != std::string::npos) path.replace(pos, 2, num);
                else path += std::string(".") + num;
                if (!exportGeo(path, target->geometry(), err)) { std::printf("%s\n", err.c_str()); return 1; }
                std::printf("frame %d -> %s\n", f, path.c_str());
            }
            return 0;
        }
    }
    usage();
    return 1;
}

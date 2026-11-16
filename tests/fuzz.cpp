// Mutation fuzzer for every parser in HD: images (PNG, EXR, HDR), zlib, BVH, USDA, OBJ, scene files,
// expressions and VEX-lite snippets. Valid seed inputs are mutated (bit flips, byte inserts/deletes,
// truncation, chunk duplication, interesting values) and fed to the parsers, which must reject bad
// input without crashing. Run under AddressSanitizer/UBSan to also catch out-of-bounds reads.
//
//   hd_fuzz [iterations per target] [seed]
#include "core/Compositing.h"
#include "core/Expr.h"
#include "core/GeoIO.h"
#include "core/Image.h"
#include "core/Json.h"
#include "core/Node.h"
#include "core/Ops.h"
#include "core/Shader.h"
#include "core/Video.h"
#include "core/Usd.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include <csignal>
#include <filesystem>
#if defined(__has_include) && __has_include(<execinfo.h>) && __has_include(<unistd.h>)
#include <execinfo.h>
#include <unistd.h>
#define HD_FUZZ_BACKTRACE 1
#endif

using namespace hd;
using Bytes = std::vector<uint8_t>;

// On a crash, print which target and input caused it (as hex, replayable with HD_FUZZ_REPLAY) and a stack trace.
static const char* g_target = "";
static const uint8_t* g_input = nullptr;
static size_t g_inputSize = 0;
static void crashHandler(int sig) {
    std::fprintf(stderr, "\nfuzz: signal %d in target '%s', input (%zu bytes):\n", sig, g_target, g_inputSize);
    for (size_t i = 0; i < g_inputSize && i < 4096; ++i) std::fprintf(stderr, "%02x", g_input[i]);
    std::fprintf(stderr, "\n");
#ifdef HD_FUZZ_BACKTRACE
    void* frames[64];
    int n = backtrace(frames, 64);
    backtrace_symbols_fd(frames, n, 2);
#endif
    std::fflush(stderr);
    std::signal(sig, SIG_DFL);
    std::raise(sig);
}

struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed) : s(seed * 6364136223846793005ULL + 1442695040888963407ULL) {}
    uint32_t next() { s = s * 6364136223846793005ULL + 1442695040888963407ULL; return uint32_t(s >> 33); }
    size_t below(size_t n) { return n ? next() % n : 0; }
};

static Bytes mutate(const Bytes& in, Rng& r) {
    Bytes b = in;
    int ops = 1 + int(r.below(4));
    for (int k = 0; k < ops; ++k) {
        switch (r.below(8)) {
        case 0: if (!b.empty()) b[r.below(b.size())] ^= uint8_t(1u << r.below(8)); break;               // bit flip
        case 1: if (!b.empty()) b[r.below(b.size())] = uint8_t(r.next()); break;                        // random byte
        case 2: b.insert(b.begin() + long(r.below(b.size() + 1)), uint8_t(r.next())); break;             // insert
        case 3: if (!b.empty()) b.erase(b.begin() + long(r.below(b.size()))); break;                     // delete
        case 4: if (!b.empty()) b.resize(r.below(b.size())); break;                                       // truncate
        case 5: {                                                                                          // duplicate a chunk
            if (b.size() < 2) break;
            size_t a = r.below(b.size()), n = 1 + r.below(std::min<size_t>(64, b.size() - a));
            Bytes chunk(b.begin() + long(a), b.begin() + long(a + n));
            b.insert(b.begin() + long(r.below(b.size() + 1)), chunk.begin(), chunk.end());
            break;
        }
        case 6: {                                                                                          // interesting 32-bit value
            if (b.size() < 4) break;
            static const uint32_t vals[] = {0, 1, 0xFFFFFFFFu, 0x7FFFFFFFu, 0x80000000u, 0xFFFFu, 0x10000u, 65535u * 65535u};
            uint32_t v = vals[r.below(8)];
            size_t p = r.below(b.size() - 3);
            std::memcpy(&b[p], &v, 4);
            break;
        }
        default: {                                                                                         // ASCII digit / sign twiddle (text formats)
            if (b.empty()) break;
            size_t p = r.below(b.size());
            static const char cs[] = "-0123456789.e+{}[]()\"@<>:,\n ";
            b[p] = uint8_t(cs[r.below(sizeof cs - 1)]);
            break;
        }
        }
    }
    if (b.size() > (1u << 20)) b.resize(1u << 20);
    return b;
}

static Bytes readAll(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    return Bytes(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}
static void writeAll(const std::string& p, const Bytes& b) {
    std::ofstream f(p, std::ios::binary);
    f.write(reinterpret_cast<const char*>(b.data()), std::streamsize(b.size()));
}
static Bytes fromString(const std::string& s) { return Bytes(s.begin(), s.end()); }

int main(int argc, char** argv) {
    const int iters = argc > 1 ? std::atoi(argv[1]) : 2000;
#if !defined(__SANITIZE_ADDRESS__) // the sanitizers print their own, better report
    for (int sig : {SIGSEGV, SIGABRT, SIGFPE, SIGILL}) std::signal(sig, crashHandler);
#else
    (void)crashHandler;
#endif
    const uint64_t seed = argc > 2 ? uint64_t(std::atoll(argv[2])) : 1;
    // ---- seed inputs (valid files)
    {
        std::vector<uint8_t> px(8 * 6 * 4);
        for (size_t i = 0; i < px.size(); ++i) px[i] = uint8_t(i * 37);
        writePng8("fz_seed.png", px, 8, 6, 4);
        std::vector<float> R(48), G(48), Z(48);
        for (int i = 0; i < 48; ++i) { R[size_t(i)] = float(i); G[size_t(i)] = float(i) * 0.5f; Z[size_t(i)] = 3; }
        writeExr("fz_seed.exr", 8, 6, {{"R", R.data()}, {"G", G.data()}, {"Z", Z.data()}}, true);
        Image hdr; hdr.w = 9; hdr.h = 4; hdr.c = 3; hdr.px.assign(9 * 4 * 3, 0.7f);
        writeHdr("fz_seed.hdr", hdr);
    }
    // an RLE-encoded HDR (the reader's other path): width >= 8, new-style runs
    Bytes hdrRle = fromString("#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y 2 +X 8\n");
    for (int y = 0; y < 2; ++y) {
        Bytes line = {2, 2, 0, 8};
        for (int ch = 0; ch < 4; ++ch) { line.push_back(128 + 8); line.push_back(uint8_t(ch == 3 ? 128 : 100)); }
        hdrRle.insert(hdrRle.end(), line.begin(), line.end());
    }
    const Bytes bvh = fromString(
        "HIERARCHY\nROOT Hips\n{\n OFFSET 0 0 0\n CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n"
        " JOINT Spine\n {\n  OFFSET 0 10 0\n  CHANNELS 3 Zrotation Xrotation Yrotation\n  End Site\n  {\n   OFFSET 0 10 0\n  }\n }\n}\n"
        "MOTION\nFrames: 2\nFrame Time: 0.5\n0 90 0 0 0 0 0 0 0\n0 90 0 90 0 0 0 0 0\n");
    const Bytes usda = fromString(
        "#usda 1.0\n( upAxis = \"Y\" )\ndef Xform \"root\" {\n double3 xformOp:translate = (1, 2, 3)\n uniform token[] xformOpOrder = [\"xformOp:translate\"]\n"
        " def Mesh \"m\" {\n  int[] faceVertexCounts = [3, 4]\n  int[] faceVertexIndices = [0, 1, 2, 0, 2, 3, 4]\n"
        "  point3f[] points.timeSamples = { 1: [(0,0,0), (1,0,0), (1,1,0), (0,1,0), (0,2,0)] }\n"
        "  color3f[] primvars:displayColor = [(1, 0, 0)] ( interpolation = \"constant\" )\n"
        "  texCoord2f[] primvars:st = [(0,0), (1,0), (1,1), (0,1), (0,2)]\n }\n"
        " def BasisCurves \"c\" { int[] curveVertexCounts = [2]\n point3f[] points = [(0,0,0), (0,1,0)] }\n}\n");
    const Bytes obj = fromString("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1 4//1\nl 1 2 3\n");
    std::string scene;
    {
        Graph g;
        Node* b = g.createNode("box");
        Node* m = g.createNode("mountain");
        g.connect(b, m, 0);
        Node* w = g.createNode("attribwrangle");
        g.connect(m, w, 0);
        w->parm("snippet")->str = "@P.y += sin(@ptnum) * 0.1;";
        b->parm("size")->expr[0] = "sin($T) + 2";
        Node* sub = g.createNode("subnet");
        (void)sub;
        g.setDisplayNode(w);
        scene = g.toJson().dump(1);
    }
    const Bytes sceneB = fromString(scene);
    const std::vector<std::string> exprSeeds = {"sin($T * 2) + $F / 24", "fit($F, 1, 100, 0, 1) * max(1, 2)", "clamp(noise(1.5), 0, 1) * $PI"};
    const std::vector<std::string> vexSeeds = {
        "float d = length(@P);\n@P.y += sin(d * 4 - @Time) * 0.1;\nif (@ptnum % 2 == 0) @pscale = rand(@ptnum);\nv@vel = curlnoise(@P);",
        "for (int i = 0; i < 3; i++) { @Cd = hsvtorgb(set(fit(i, 0, 3, 0, 1), 1, 1)); }",
    };

    Rng rng(seed);
    long total = 0;
    auto run = [&](const char* name, const Bytes& seedBytes, const std::function<void(const Bytes&)>& fn) {
        // HD_FUZZ_ONLY=<target> runs one target with the same inputs it gets in a full run
        const char* only = std::getenv("HD_FUZZ_ONLY");
        for (int i = 0; i < iters; ++i) {
            Bytes b = i == 0 ? seedBytes : mutate(seedBytes, rng);
            if (only && std::strcmp(only, name) != 0) { ++total; continue; }
            // HD_FUZZ_REPLAY=<hex> runs that input instead (with HD_FUZZ_ONLY, to replay a reported crash)
            if (const char* hex = std::getenv("HD_FUZZ_REPLAY")) {
                b.clear();
                for (size_t k = 0; hex[k] && hex[k + 1]; k += 2) { char two[3] = {hex[k], hex[k + 1], 0}; b.push_back(uint8_t(std::strtoul(two, nullptr, 16))); }
            }
            g_target = name; g_input = b.data(); g_inputSize = b.size();
            fn(b);
            ++total;
        }
        std::printf("%-12s %d inputs ok\n", name, iters);
        std::fflush(stdout);
    };
    auto viaFile = [&](const char* path, const std::function<void(const std::string&)>& fn) {
        return [=](const Bytes& b) { writeAll(path, b); fn(path); };
    };
    run("zlib", zlibCompress(readAll("fz_seed.png").data(), 300), [](const Bytes& b) { std::vector<uint8_t> o; zlibDecompress(b.data(), b.size(), o); });
    run("png", readAll("fz_seed.png"), viaFile("fz_case.png", [](const std::string& p) { Image im; readImage(p, im, true); }));
    run("exr", readAll("fz_seed.exr"), viaFile("fz_case.exr", [](const std::string& p) {
        int w, h; std::vector<std::pair<std::string, std::vector<float>>> ch; readExrChannels(p, w, h, ch); Image im; readImage(p, im, false); }));
    run("hdr", readAll("fz_seed.hdr"), viaFile("fz_case.hdr", [](const std::string& p) { Image im; readImage(p, im, false); }));
    run("hdr-rle", hdrRle, viaFile("fz_case.hdr", [](const std::string& p) { Image im; readImage(p, im, false); }));
    run("bvh", bvh, viaFile("fz_case.bvh", [](const std::string& p) {
        Graph g; Node* n = g.createNode("bvhimport"); n->parm("file")->str = p;
        for (float f : {1.f, 7.f, 500.f}) { g.frame = f; n->touch(); g.cook(n); } }));
    run("usda", usda, viaFile("fz_case.usda", [](const std::string& p) { Geometry g; loadUsda(p, g); }));
    run("obj", obj, viaFile("fz_case.obj", [](const std::string& p) { Geometry g; loadObj(p, g); }));
    run("scene", sceneB, [](const Bytes& b) {
        std::string err;
        Json j = Json::parse(std::string(b.begin(), b.end()), &err);
        if (!err.empty()) return;
        Graph g;
        if (g.fromJson(j, &err) && g.displayNode()) g.cook(g.displayNode());
    });
    for (auto& e : exprSeeds)
        run("expression", fromString(e), [](const Bytes& b) { float v; std::string err; evalExpression(std::string(b.begin(), b.end()), 3, 24, v, &err); });
    for (auto& v : vexSeeds)
        run("vex", fromString(v), [](const Bytes& b) {
            std::string err;
            auto prog = Program::compile(std::string(b.begin(), b.end()), false, err);
            if (!prog) return;
            Geometry g;
            for (int i = 0; i < 5; ++i) g.addPoint(Vec3(float(i), 0, 0));
            g.addPrim({0, 1, 2});
            runOverElements(g, AttribClass::Point, *prog, 1, 24, nullptr, err);
        });
    // movies: bytes pick the size, quantiser, slices, keyframe interval and the pixels
    {
        Bytes seed;
        for (int i = 0; i < 64; ++i) seed.push_back(uint8_t(i * 29 + 3));
        run("movie", seed, [](const Bytes& b) {
            size_t pos = 0;
            auto next = [&]() -> uint32_t { return pos < b.size() ? b[pos++] : uint32_t(b.size() * 7 % 251); };
            const int w = 2 + int(next() % 70), h = 2 + int(next() % 50);
            MovieOptions o;
            o.qp = int(next() % 52);
            o.slices = int(next() % 5);
            o.keyint = int(next() % 4);
            o.deblock = next() % 4 != 0;
            MovieWriter mw;
            std::string err;
            const std::string path = (std::filesystem::temp_directory_path() / "hd_fuzz_movie.mp4").string();
            if (!mw.open(path, w, h, o, err)) return;
            std::vector<uint8_t> px(size_t(w) * size_t(h) * 4);
            const int frames = 1 + int(next() % 4);
            for (int f = 0; f < frames; ++f) {
                for (size_t i = 0; i < px.size(); ++i) px[i] = uint8_t(next() * (i + 1) + uint32_t(f) * 13);
                if (!mw.addFrame(px.data(), err)) std::abort();
                if (mw.lastReconstruction().size() != size_t(w & ~1) * size_t(h & ~1) * 3 / 2) std::abort();
            }
            if (!mw.finish(err)) std::abort();
            std::remove(path.c_str());
        });
    }
    // booleans: bytes pick the shapes, transforms (including degenerate scales and exact overlaps) and operation
    {
        Bytes seed;
        for (int i = 0; i < 24; ++i) seed.push_back(uint8_t(i * 53 + 7));
        // inputs that crashed once (kept as regression seeds; they also start the mutations)
        static const char* regress[] = {"073c71a6db10457aafe4194e83b801000000c1f6ffffffff"};
        auto fromHex = [](const char* hex) {
            Bytes b;
            for (size_t k = 0; hex[k] && hex[k + 1]; k += 2) { char two[3] = {hex[k], hex[k + 1], 0}; b.push_back(uint8_t(std::strtoul(two, nullptr, 16))); }
            return b;
        };
        const auto booleanCase = [](const Bytes& b) {
            size_t pos = 0;
            auto next = [&]() -> uint32_t { return pos < b.size() ? b[pos++] : 0u; };
            static const char* shapes[] = {"box", "sphere", "torus", "tube", "grid"};
            Graph g;
            Node* in[2];
            for (int k = 0; k < 2; ++k) {
                Node* s = g.createNode(shapes[next() % 5]);
                if (Parm* r = s->parm("rows")) r->val[0] = float(3 + next() % 6);
                if (Parm* c = s->parm("cols")) c->val[0] = float(3 + next() % 8);
                Node* x = g.createNode("xform");
                g.connect(s, x, 0);
                for (int c = 0; c < 3; ++c) {
                    x->parm("t")->val[c] = (float(next() % 9) - 4) * 0.25f;
                    x->parm("r")->val[c] = float(next() % 8) * 45.f;
                    x->parm("s")->val[c] = float(next() % 4) * 0.5f; // 0 flattens the shape
                }
                in[k] = x;
            }
            Node* bo = g.createNode("boolean");
            g.connect(in[0], bo, 0);
            g.connect(in[1], bo, 1);
            bo->parm("op")->val[0] = float(next() % 4);
            g.cook(bo);
            const Geometry& r = bo->geometry();
            for (int v : r.vertices) if (v < 0 || v >= int(r.numPoints())) std::abort();
            for (auto& [n, a] : r.primAttribs) if (a.count() != r.numPrims()) std::abort();
            for (auto& [n, a] : r.pointAttribs) if (a.count() != r.numPoints()) std::abort();
        };
        for (const char* hex : regress) run("boolean", fromHex(hex), booleanCase);
        run("boolean", seed, booleanCase);
    }
    // shader networks: bytes pick shading nodes, wiring and parameter values (huge, tiny, NaN-free extremes)
    {
        Bytes seed;
        for (int i = 0; i < 96; ++i) seed.push_back(uint8_t(i * 37 + 11));
        run("shader", seed, [](const Bytes& b) {
            std::vector<const NodeType*> shading;
            for (auto& t : nodeTypes()) if (t.category == "Shading") shading.push_back(&t);
            size_t pos = 0;
            auto next = [&]() -> uint32_t { return pos < b.size() ? b[pos++] : 0u; };
            Graph g;
            Node* sp = g.createNode("sphere");
            sp->parm("rows")->val[0] = 6; sp->parm("cols")->val[0] = 8;
            Node* mb = g.createNode("materialbuilder");
            g.connect(sp, mb, 0);
            Graph& net = *mb->subnet();
            std::vector<Node*> made{net.findNode("surface_output")};
            const int count = int(next() % 10);
            for (int i = 0; i < count; ++i) made.push_back(net.createNode(shading[next() % shading.size()]->name));
            static const float values[] = {0, 1, -1, 0.5f, 3, 1e-7f, 1e7f, -1e9f, 2e9f, 64, 1e30f, -0.0f};
            for (Node* n : made)
                for (auto& p : n->parms()) {
                    if (next() % 3) continue;
                    for (int c = 0; c < 3; ++c) p.val[c] = values[next() % (sizeof values / sizeof values[0])];
                    if (p.def->type == ParmType::String) p.str = next() % 2 ? "Cd" : "nothing";
                }
            for (int i = 0; i < count * 2; ++i) {
                Node* src = made[next() % made.size()];
                Node* dst = made[next() % made.size()];
                if (dst->inputs.empty()) continue;
                net.connect(src, dst, int(next() % dst->inputs.size()));
            }
            g.cook(mb);
            std::string err;
            if (auto prog = compileShader(net, 1, 24, err)) {
                ShaderInputs in;
                ShaderOutputs o;
                for (float x : {0.f, 1e9f, -3.5f}) { in.P = Vec3(x, 0.25f, -x); prog->eval(in, o); }
            }
        });
    }
    for (const char* f : {"fz_seed.png", "fz_seed.exr", "fz_seed.hdr", "fz_case.png", "fz_case.exr", "fz_case.hdr", "fz_case.bvh", "fz_case.usda", "fz_case.obj"}) std::remove(f);
    std::printf("fuzz: %ld inputs, no crashes\n", total);
    return 0;
}

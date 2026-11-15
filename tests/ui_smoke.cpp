// Smoke test of the real application, headless (GLFW null platform + EGL surfaceless context).
// Loads every example scene into the app, plays frames, opens each shelf tab and the main windows,
// renders one scene in the Render View, and fails on node errors, OpenGL errors or crashes.
// Writes a screenshot per example (PPM) so the UI can be inspected.
//   hd_ui_smoke <examples_dir> <out_dir> [render_samples]
#include "app/App.h"
#include "core/Python.h"
#include "core/Demo.h"
#include "render/GL.h"
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_internal.h>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <filesystem>
#include <chrono>
#include <string>
#include <thread>

typedef void* EGLDisplay; typedef void* EGLConfig; typedef void* EGLContext;
typedef int EGLint; typedef unsigned EGLenum; typedef unsigned EGLBoolean;
static void* (*p_eglGetProcAddress)(const char*);
static void* getProc(const char* n) { return p_eglGetProcAddress(n); }

static bool makeContext() {
    void* lib = dlopen("libEGL.so.1", RTLD_NOW | RTLD_GLOBAL);
    if (!lib) return false;
    p_eglGetProcAddress = (void* (*)(const char*))dlsym(lib, "eglGetProcAddress");
    auto gpd = (EGLDisplay(*)(EGLenum, void*, const EGLint*))getProc("eglGetPlatformDisplayEXT");
    auto init = (EGLBoolean(*)(EGLDisplay, EGLint*, EGLint*))dlsym(lib, "eglInitialize");
    auto choose = (EGLBoolean(*)(EGLDisplay, const EGLint*, EGLConfig*, EGLint, EGLint*))dlsym(lib, "eglChooseConfig");
    auto bind = (EGLBoolean(*)(EGLenum))dlsym(lib, "eglBindAPI");
    auto create = (EGLContext(*)(EGLDisplay, EGLConfig, EGLContext, const EGLint*))dlsym(lib, "eglCreateContext");
    auto current = (EGLBoolean(*)(EGLDisplay, void*, void*, EGLContext))dlsym(lib, "eglMakeCurrent");
    EGLDisplay d = gpd(0x31DD, nullptr, nullptr);
    EGLint a, b;
    if (!init(d, &a, &b)) return false;
    EGLint ca[] = {0x3033, 1, 0x3040, 8, 0x3038};
    EGLConfig cfg; EGLint n = 0;
    choose(d, ca, &cfg, 1, &n);
    bind(0x30A2);
    EGLint xa[] = {0x3098, 3, 0x30FB, 3, 0x30FD, 1, 0x3038};
    EGLContext c = create(d, n ? cfg : nullptr, nullptr, xa);
    return c && current(d, nullptr, nullptr, c);
}

static void savePPM(const std::string& path, int w, int h) {
    std::vector<unsigned char> px(size_t(w) * h * 4);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
    FILE* f = std::fopen(path.c_str(), "wb");
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int y = h - 1; y >= 0; --y)
        for (int x = 0; x < w; ++x) std::fwrite(&px[(size_t(y) * w + x) * 4], 1, 3, f);
    std::fclose(f);
    std::printf("wrote %s\n", path.c_str());
}


int main(int argc, char** argv) {
    if (argc < 3) { std::printf("usage: hd_ui_smoke <examples_dir> <out_dir> [render_samples]\n"); return 2; }
    const std::string exDir = argv[1], outDir = argv[2];
    const int renderSamples = argc > 3 ? std::atoi(argv[3]) : 8;
    std::filesystem::create_directories(outDir);
    const int W = 1600, H = 940;
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_NULL);
    if (!glfwInit()) { std::printf("glfw init failed\n"); return 1; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* win = glfwCreateWindow(W, H, "HD", nullptr, nullptr);
    if (!win || !makeContext()) { std::printf("context failed\n"); return 1; }
    if (!hd::loadGL(getProc)) { std::printf("loadGL failed\n"); return 1; }
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(float(W), float(H));
    ImFontConfig cfg; cfg.OversampleH = 2;
    if (std::filesystem::exists("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
        { static const ImWchar r[] = {0x0020, 0x00FF, 0x2010, 0x2027, 0x2190, 0x2193, 0x2212, 0x2212, 0}; io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14.f, &cfg, r); }
    else io.Fonts->AddFontDefault();
    hd::applyHoudiniTheme(1.f);
    if (!ImGui_ImplOpenGL3_Init("#version 330 core")) { std::printf("imgui gl3 init failed\n"); return 1; }
    unsigned fbo, tex;
    glGenFramebuffers(1, &fbo);
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);

    hd::App app;
    if (!app.init(win, "")) { std::printf("app init failed\n"); return 1; }
    if (std::getenv("HD_SMOKE_CLAY")) app.viewOpts.materials = false; // compare against clay shading
    int failures = 0, glErrors = 0;
    auto runFrame = [&]() {
        io.DeltaTime = 1.f / 60.f;
        ImGui_ImplOpenGL3_NewFrame();
        ImGui::NewFrame();
        app.frame();
        ImGui::Render();
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, W, H);
        glClearColor(0.1f, 0.1f, 0.1f, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        for (GLenum e = glGetError(); e != 0; e = glGetError()) { ++glErrors; std::printf("  GL error 0x%x\n", e); }
    };
    auto save = [&](const std::string& name) { savePPM(outDir + "/" + name + ".ppm", W, H); };
    for (int i = 0; i < 4; ++i) runFrame();

    std::vector<std::filesystem::path> scenes;
    for (auto& e : std::filesystem::directory_iterator(exDir)) if (e.path().extension() == ".hdg") scenes.push_back(e.path());
    std::sort(scenes.begin(), scenes.end());
    for (auto& path : scenes) {
        auto t0 = std::chrono::steady_clock::now();
        if (!app.loadScene(path.string())) { std::printf("FAIL %s: could not load\n", path.filename().string().c_str()); ++failures; continue; }
        for (int i = 0; i < 3; ++i) runFrame();
        // play a few frames like the timeline does, then jump ahead
        for (int f = 2; f <= 6; ++f) { app.graph.frame = float(f); runFrame(); }
        app.graph.frame = 24;
        for (int i = 0; i < 3; ++i) runFrame();
        app.frameGeometry();
        runFrame(); runFrame();
        int nodeErrors = 0;
        for (auto& n : app.graph.nodes()) {
            app.graph.cook(n.get());
            if (!n->error().empty()) { std::printf("  %s: node %s: %s\n", path.filename().string().c_str(), n->name().c_str(), n->error().c_str()); ++nodeErrors; }
        }
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        hd::Node* d = app.graph.displayNode();
        const hd::Geometry* g = d ? &d->geometry() : nullptr;
        std::printf("%s %-18s %8.0f ms  %zu nodes  display %s: %zu pts %zu prims %zu images\n", nodeErrors ? "FAIL" : "ok  ", path.stem().string().c_str(), ms,
                    app.graph.nodes().size(), d ? d->name().c_str() : "-", g ? g->numPoints() : 0, g ? g->numPrims() : 0, g ? g->images.size() : 0);
        for (const std::string& e : app.renderer.textureErrors()) { std::printf("  %s: %s\n", path.filename().string().c_str(), e.c_str()); ++nodeErrors; }
        if (nodeErrors) ++failures;
        save("example_" + path.stem().string());
    }
    // every shelf tab and the main windows
    app.loadScene((std::filesystem::path(exDir) / "materials.hdg").string());
    for (int i = 0; i < 3; ++i) runFrame();
    if (ImGuiWindow* shelf = ImGui::FindWindowByName("Shelf")) {
        ImGuiTabBar* tb = nullptr;
        (void)tb;
        // click along the tab row
        float y = shelf->Pos.y + 14;
        for (float x = shelf->Pos.x + 20; x < shelf->Pos.x + 900; x += 60) {
            io.AddMousePosEvent(x, y); runFrame();
            io.AddMouseButtonEvent(0, true); runFrame();
            io.AddMouseButtonEvent(0, false); runFrame();
        }
    }
    app.showRenderSettings = true;
    for (int i = 0; i < 3; ++i) runFrame();
    save("render_settings");
    app.showRenderSettings = false;
    // Ground Plane from the shelf on the terrain example: always shown, display flag untouched
    {
        app.loadScene(exDir + "/terrain.hdg");
        for (int i = 0; i < 3; ++i) runFrame();
        hd::Node* before = app.graph.displayNode();
        app.shelfTool("groundplane");
        for (int i = 0; i < 4; ++i) runFrame();
        hd::Node* gp = nullptr;
        for (auto& n : app.graph.nodes()) if (n->type().name == "groundplane") gp = n.get();
        if (!gp || app.graph.displayNode() != before || !gp->error().empty()) { std::printf("FAIL ground plane tool\n"); ++failures; }
        if (app.renderer.stats().triangles < before->geometry().numPrims() + 1000) { std::printf("FAIL ground plane not drawn\n"); ++failures; }
        save("ground_plane");
    }
    // Material Builder: diving in shows the shader network while the viewport keeps showing the scene,
    // and the Shading shelf adds pattern nodes into the network
    {
        app.loadScene(exDir + "/shaders.hdg");
        for (int i = 0; i < 3; ++i) runFrame();
        hd::Node* mb = app.graph.findNode("rust");
        if (!mb || !mb->subnet()) { std::printf("FAIL shaders.hdg: no rust material builder\n"); ++failures; }
        else {
            const size_t tris = app.renderer.stats().triangles;
            app.diveInto(mb);
            for (int i = 0; i < 3; ++i) runFrame();
            runFrame(); runFrame();
            if (app.renderer.stats().triangles != tris) { std::printf("FAIL viewport changed inside the material builder (%zu vs %zu)\n", app.renderer.stats().triangles, tris); ++failures; }
            const size_t before = mb->subnet()->nodes().size();
            app.shelfTool("shvoronoi");
            for (int i = 0; i < 3; ++i) runFrame();
            if (mb->subnet()->nodes().size() != before + 1) { std::printf("FAIL shading shelf tool\n"); ++failures; }
            app.graph.cook(mb);
            if (!mb->error().empty()) { std::printf("FAIL rust: %s\n", mb->error().c_str()); ++failures; }
            save("shader_network");
            app.goToLevel(0);
            for (int i = 0; i < 3; ++i) runFrame();
        }
    }
    // an empty scene: what you see when you start modelling (the reference grid on its own)
    app.newScene(false);
    for (int i = 0; i < 4; ++i) runFrame();
    save("empty_scene");
    // zoomed out a long way and with the ruler: lines thin out instead of turning into noise
    app.viewOpts.gridRulerType = 1;
    app.camera.dist = 120;
    for (int i = 0; i < 3; ++i) runFrame();
    save("grid_far_ruler");
    app.camera.dist = 2; app.camera.pitch = 60;
    for (int i = 0; i < 3; ++i) runFrame();
    save("grid_close");
    app.viewOpts.gridRulerType = 0;
    app.camera.home();
    app.showAbout = true;
    for (int i = 0; i < 3; ++i) runFrame();
    save("about");
    app.showAbout = false;
    // Python Shell: a command changes the scene as one undo step (skipped if no Python is installed)
    app.showPythonShell = true;
    for (int i = 0; i < 2; ++i) runFrame();
    if (hd::python::available()) {
        size_t n0 = app.graph.nodes().size();
        bool ok = app.runPython("t = hd.createNode('torus', 'py_torus')\nt.parm('rad1').set(1.5)\nt.setDisplayFlag()\nprint('created', t)", "<shell>", true);
        for (int i = 0; i < 3; ++i) runFrame();
        save("python_shell");
        if (!ok || app.graph.nodes().size() != n0 + 1 || !app.graph.findNode("py_torus") || app.pythonLog.find("created") == std::string::npos) {
            std::printf("FAIL python shell command: %s\n", app.pythonLog.c_str()); ++failures;
        }
        app.undo();
        runFrame();
        if (app.graph.findNode("py_torus")) { std::printf("FAIL python command was not undone\n"); ++failures; }
        std::printf("python shell ok (Python %s)\n", hd::python::ensure().version.c_str());
    } else std::printf("python shell: Python not available (%s)\n", hd::python::ensure().error.c_str());
    app.showPythonShell = false;
    app.graph.render.samples = renderSamples;
    app.graph.render.width = 480; app.graph.render.height = 270;
    app.startRender();
    ImGui::SetWindowFocus("Render View");
    while (app.rendering()) { runFrame(); std::this_thread::sleep_for(std::chrono::milliseconds(50)); }
    for (int i = 0; i < 3; ++i) runFrame();
    save("render_view");
    // the composite shows as an image in the Scene View
    app.loadScene((std::filesystem::path(exDir) / "comp.hdg").string());
    ImGui::SetWindowFocus("Scene View");
    for (int i = 0; i < 4; ++i) runFrame();
    save("comp_viewer");
    // undo / redo stress: create, connect and delete nodes, then undo everything and redo it
    app.loadScene((std::filesystem::path(exDir) / "terrain.hdg").string());
    const size_t before = app.graph.nodes().size();
    for (const char* t : {"box", "sphere", "voronoifracture", "copcolor", "skeleton", "flipsolver"}) { app.shelfTool(t); runFrame(); }
    const size_t after = app.graph.nodes().size();
    for (int i = 0; i < 12; ++i) { app.undo(); runFrame(); }
    const size_t undone = app.graph.nodes().size();
    for (int i = 0; i < 12; ++i) { app.redo(); runFrame(); }
    std::printf("undo/redo: %zu -> %zu -> %zu -> %zu nodes\n", before, after, undone, app.graph.nodes().size());
    if (undone != before || app.graph.nodes().size() != after) { std::printf("FAIL undo/redo did not restore the node count\n"); ++failures; }

    app.shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui::DestroyContext();
    std::printf("%zu scenes, %d failures, %d GL errors\n", scenes.size(), failures, glErrors);
    return failures || glErrors ? 1 : 0;
}

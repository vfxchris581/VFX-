#pragma once
#include "core/Lights.h"
#include "core/Node.h"
#include "core/PathTracer.h"
#include "core/ImageSequence.h"
#include "core/Video.h"
#include "render/Renderer.h"
#include <atomic>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

struct GLFWwindow;
struct ImFont;

namespace hd {

class App;

// Houdini-style network editor (vertical flow, Tab menu, flags, wires).
class NetworkEditor {
public:
    void draw(App& app);
    void frameAll(App& app);
    void requestFrameAll() { wantFrameAll_ = true; }

private:
    enum class Drag { None, Pan, Move, Wire, Box };
    struct Vec2 { float x = 0, y = 0; };
    Vec2 offset_{60, 60};
    float zoom_ = 1.f;
    Drag drag_ = Drag::None;
    bool moveCheckpointed_ = false;
    int wireSrc_ = -1;            // node id whose output is being dragged
    int wireDetachedDst_ = -1;    // when re-wiring from an input
    int wireDetachedIdx_ = -1;
    Vec2 boxStart_;
    bool wantFrameAll_ = true;
    int frameAllDelay_ = 0;
    // Tab menu
    bool openMenu_ = false;
    Vec2 menuWorld_;
    int menuConnectFrom_ = -1;
    char search_[64] = {0};
    int menuHighlight_ = 0;
    // rename
    int diveId_ = -1;
    bool upRequested_ = false;
    int renameId_ = -1;
    char renameBuf_[64] = {0};
    void drawNodeMenu(App& app);
};

enum class FileDialogMode { None, Open, SaveAs, ImportObj, ExportObj, ChooseFileParm, SaveImage, SaveSequence, MergeScene, InstallAsset, ExportUsd, RunPython, OpenSequence, SaveMovie };

class App {
public:
    bool init(GLFWwindow* window, const std::string& startupFile);
    void frame();
    void shutdown();
    bool handleCloseRequest(); // returns true if it is OK to close now
    void openPath(const std::string& path); // drag & drop / command line

    // --- state shared with panels
    Graph graph;
    Renderer renderer;
    Camera camera;
    ViewOptions viewOpts;
    bool showPointNumbers = false;
    bool showPrimNumbers = false;
    std::set<int> selection;   // node ids
    int currentId = -1;        // node shown in the parameter pane
    NetworkEditor network;
    ImFont* monoFont = nullptr;

    // The network level being edited: the scene itself, or the inside of a subnet.
    Graph& net();
    // The network whose display node is shown: net(), or inside a Material Builder the network around it.
    Graph& viewNet();
    // Perspective or an orthographic axis view in the viewport (leaves a camera node's view).
    void setViewportView(int view);
    // ---- Sequence Player (SequencePlayer.cpp): plays rendered image sequences, like Houdini's MPlay
    bool openSequence(const std::string& anyFrame, bool focus = true); // any frame of a PNG / EXR / HDR sequence
    void drawSequencePlayer();
    void playerStep(int frames); // moves the playhead, following the loop mode
    bool showSequencePlayer = false;
    struct Player {
        int index = 0;          // into the sequence's frames
        bool playing = false;
        int dir = 1;            // playing forward (+1) or backward (-1)
        float fps = 24;
        int loop = 0;           // 0 loop, 1 play once, 2 ping-pong
        int channel = 0;        // 0 RGB, 1 R, 2 G, 3 B, 4 alpha, 5 luminance
        float exposure = 0;     // stops, for EXR / HDR
        float zoom = 0;         // 0 = fit to the panel, else image pixels per screen pixel
        float panX = 0, panY = 0;
        double accum = 0;       // fractional frames waiting to be played
        bool buffering = false; // playing but the next frame isn't loaded yet
        float realFps = 0;
        double fpsClock = 0;
        int fpsFrames = 0;
        unsigned tex = 0;
        int texW = 0, texH = 0;
        const void* texFrame = nullptr; // frame data the texture holds
        int texChannel = -1;
        std::string lastRendered; // a frame of the last finished Render Sequence
        std::string error;
        bool focusNext = false;
    } player;
    std::unique_ptr<SequenceCache> seqCache_;
    // movies: export from the Sequence Player (background thread) and Render Sequence to .mp4 / .mov
    struct MovieJob {
        std::thread thread;
        std::atomic<int> done{0}, total{0};
        std::atomic<bool> cancel{false}, finished{false};
        bool ok = false;
        std::string error, path;
        ~MovieJob() { cancel = true; if (thread.joinable()) thread.join(); }
    };
    std::unique_ptr<MovieJob> movieJob_;
    void startMovieExport(const std::string& path);
    void pollMovieJob();
    int movieQuality = 1; // 0 high, 1 medium, 2 small
    std::unique_ptr<MovieWriter> seqMovie_;
    std::string seqMoviePath_;
    void endSequence(bool stopped); // finishes a Render Sequence (and its movie)
    void startRenderSequence(const std::string& path); // .png / .exr frames, or an .mp4 / .mov movie
    bool renderingSequence() const { return seqActive_; }
    Node* current() { return net().findNode(currentId); }
    const std::vector<std::string>& netPath() const { return netPath_; }
    void diveInto(Node* subnetNode);
    void goToLevel(size_t depth); // 0 = top level
    void collapseSelected();
    // reusable assets (saved subnets)
    std::string assetDir() const;
    const std::vector<std::string>& assets() const { return assets_; }
    void refreshAssets();
    void saveAsset(Node* subnetNode);
    Node* createAsset(const std::string& name, float x, float y, int connectFrom = -1);
    void deleteAsset(const std::string& name);
    void installAsset(const std::string& path);
    void drawAssetManager();
    bool showAssetManager = false;
    // keyframe animation (Animation.cpp)
    void setKeyframe(Node* n, Parm& p);          // key every component at the current frame
    void keySelected();                          // K: key the animated parameters of the selected nodes
    void jumpToKey(int direction);               // previous (-1) / next (+1) keyframe
    void drawKeyMarks(float x0, float x1, float y0, float y1); // ticks on the timeline
    void drawAnimEditor();
    bool showAnimEditor = false;
    // shelf, lights and cameras (Shelf.cpp)
    void drawShelf();
    void shelfTool(const std::string& tool);
    void updateSceneCamera(bool navigated, float viewAspect); // look-through / lock handling
    void updateGuides();
    Node* activeCameraNode();
    bool showShelf = true;
    bool lockCamera = false;
    float activeCamAspect = 0; // > 0 while looking through a camera node
    bool camEditActive_ = false;
    void select(Node* n, bool additive = false);
    void clearSelection() { selection.clear(); currentId = -1; }

    // undo
    void checkpoint();
    void undo();
    void redo();
    bool canUndo() const { return !undo_.empty(); }
    bool canRedo() const { return !redo_.empty(); }
    void markDirty() { dirty_ = true; }

    // actions
    Node* createNode(const std::string& type, float x, float y, int connectFrom = -1);
    void deleteSelected();
    void duplicateSelected();
    void setDisplay(Node* n);
    void newScene(bool withDemo);
    bool loadScene(const std::string& path);
    bool saveScene(const std::string& path);
    void openFileDialog(FileDialogMode mode);
    void status(const std::string& msg, bool error = false);
    void frameGeometry();

    // panels
    void drawViewport();
    void drawParameters();
    void drawSpreadsheet();
    void drawTimeline();
    void drawRenderView();
    void startRender();
    bool rendering() const { return tracer_.running(); }
    void drawStatusLine();
    void drawMenuBar();
    void drawFileDialog();
    void drawHelpWindows();

    // menus, preferences, clipboard (Menus.cpp)
    void drawDisplayOptions();
    void drawRenderSettings();
    void drawPythonShell();
    void stepSequence();
    void loadPrefs();
    void savePrefs();
    void pushRecent(const std::string& path);
    void copySelected();
    void cutSelected();
    void pasteNodes();
    void mergeScene(const std::string& path);
    void saveIncrement();
    void openExample(const std::string& name);
    bool showSceneView = true, showNetwork = true, showParms = true, showSpreadsheet = true, showRenderView = true, showTimeline = true;
    bool showDisplayOptions = false, showRenderSettings = false;
    bool showPythonShell = false;
    // Runs Python against the scene (one undo step if it changed anything); output goes to the shell.
    bool runPython(const std::string& code, const std::string& name, bool shell);
    std::string pythonLog;

    bool playing = false;
    bool realtime = true;
    bool showShortcuts = false;
    bool showVexHelp = false;
    bool showAbout = false;
    bool showImGuiDemo = false;
    std::string fileParmTarget; // "nodeName/parmName" for ChooseFileParm
    std::string examplesDir;    // bundled example scenes (next to the executable)

private:
    void buildDefaultLayout(unsigned dockId);
    void handleGlobalShortcuts();
    void updateTitle();
    void cookDisplay();

    struct Snapshot { std::string graph; std::vector<std::string> selected; std::string current; std::vector<std::string> path; };
    Snapshot snapshot();
    void restore(const Snapshot& s);
    std::vector<Snapshot> undo_, redo_;

    GLFWwindow* window_ = nullptr;
    std::string currentFile_;
    bool dirty_ = false;
    bool layoutBuilt_ = false;
    bool forceLayout_ = false;
    int focusSceneView_ = 0; // frames until the Scene View tab is brought to the front
    bool confirmClose_ = false;
    bool allowClose_ = false;
    double lastTime_ = 0;
    double playAccum_ = 0;
    std::string statusMsg_;
    bool statusErr_ = false;
    double statusTime_ = 0;
    std::string lastTitle_;
    float viewportW_ = 0, viewportH_ = 0;
    bool frameOnNextCook_ = true;
    uint64_t groundStamp_ = 0; // what the viewport last got when Ground Plane nodes were added

    // render view
    PathTracer tracer_;
    unsigned compTex_ = 0;       // image shown in the Scene View when the display node carries one
    uint64_t compStamp_ = 0;
    int compW_ = 0, compH_ = 0;
    unsigned renderTex_ = 0;
    int renderW_ = 0, renderH_ = 0;
    uint64_t renderVersion_ = 0;
    std::vector<uint8_t> renderImg_;
    bool seqActive_ = false, seqWaiting_ = false;
    int seqFrame_ = 0, seqEnd_ = 0, seqCount_ = 0;
    std::string seqBase_;
    void syncCameraToScene();

    std::vector<std::string> recent_;
    std::vector<std::string> netPath_; // names of the subnets we have dived into
    std::vector<std::string> assets_;
    std::string clipboard_;
    // animation editor drag
    struct KeyDrag { bool active = false; std::string node, parm; int comp = 0; float frame = 0; } keyDrag_;

    // file dialog
    FileDialogMode fileMode_ = FileDialogMode::None;
    bool fileDialogOpenRequest_ = false;
    std::string fileDir_;
    char fileName_[512] = {0};

    friend class NetworkEditor;
};

// Style
void applyHoudiniTheme(float scale);

} // namespace hd

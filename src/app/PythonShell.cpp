// Python Shell pane: an interactive prompt with the `hd` module bound to the open scene.
#include "App.h"
#include "core/Python.h"
#include <imgui.h>
#include <cstring>

namespace hd {

namespace {
std::vector<std::string> g_history;
int g_histPos = -1;
char g_input[16384] = "";
size_t g_lastLogSize = 0;

void setInput(const std::string& s) {
    std::strncpy(g_input, s.c_str(), sizeof g_input - 1);
    g_input[sizeof g_input - 1] = 0;
}
} // namespace

void App::drawPythonShell() {
    if (!ImGui::Begin("Python Shell", &showPythonShell)) { ImGui::End(); return; }
    const python::Info& inf = python::ensure(); // first use: find and start Python
    if (!inf.ok) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.65f, 0.35f, 1.f));
        ImGui::TextWrapped("Python is not available: %s", inf.error.c_str());
        ImGui::PopStyleColor();
        ImGui::Spacing();
        ImGui::TextWrapped("HD uses the Python 3 (3.8 or newer) installed on this computer. Install it from python.org "
                           "(or your package manager), or set the HD_PYTHON environment variable to the python executable, "
                           "then restart HD.");
        ImGui::End();
        return;
    }
    // Python SOP output arrives while nodes cook
    std::string sopLog = python::takeLog();
    if (!sopLog.empty()) pythonLog += sopLog;

    ImGui::TextDisabled("Python %s  (%s)   hd module loaded: try  hd.nodes()  or  help(hd)", inf.version.c_str(), inf.executable.c_str());
    const float inputH = ImGui::GetTextLineHeightWithSpacing() * 4 + ImGui::GetStyle().FramePadding.y * 2;
    ImGui::BeginChild("##pylog", ImVec2(0, -inputH - ImGui::GetFrameHeightWithSpacing() - 4), true, ImGuiWindowFlags_HorizontalScrollbar);
    ImGui::PushFont(nullptr);
    ImGui::TextUnformatted(pythonLog.c_str(), pythonLog.c_str() + pythonLog.size());
    ImGui::PopFont();
    if (pythonLog.size() != g_lastLogSize) {
        ImGui::SetScrollHereY(1.f);
        g_lastLogSize = pythonLog.size();
    }
    ImGui::EndChild();

    bool run = false;
    ImGui::SetNextItemWidth(-1);
    static bool refocus = false;
    if (refocus) { ImGui::SetKeyboardFocusHere(); refocus = false; }
    if (ImGui::InputTextMultiline("##pyin", g_input, sizeof g_input, ImVec2(-1, inputH),
                                  ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CtrlEnterForNewLine | ImGuiInputTextFlags_AllowTabInput))
        run = true;
    if (ImGui::IsItemActive() && ImGui::GetIO().KeyCtrl && !g_history.empty()) {
        // Ctrl+Up / Ctrl+Down: history
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
            g_histPos = g_histPos < 0 ? int(g_history.size()) - 1 : std::max(0, g_histPos - 1);
            setInput(g_history[size_t(g_histPos)]);
            refocus = true;
        } else if (ImGui::IsKeyPressed(ImGuiKey_DownArrow) && g_histPos >= 0) {
            ++g_histPos;
            if (g_histPos >= int(g_history.size())) { g_histPos = -1; setInput(""); }
            else setInput(g_history[size_t(g_histPos)]);
            refocus = true;
        }
    }
    if (ImGui::Button("Run")) run = true;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Enter runs; Ctrl+Enter adds a line; Ctrl+Up/Down recall earlier commands");
    ImGui::SameLine();
    if (ImGui::Button("Run Script...")) openFileDialog(FileDialogMode::RunPython);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) pythonLog.clear();
    ImGui::SameLine();
    ImGui::TextDisabled("Changes from a command are one undo step");

    if (run && g_input[0]) {
        std::string code = g_input;
        // echo like the interactive prompt
        bool first = true;
        size_t start = 0;
        while (start <= code.size()) {
            size_t nl = code.find('\n', start);
            std::string line = code.substr(start, nl == std::string::npos ? std::string::npos : nl - start);
            pythonLog += (first ? ">>> " : "... ") + line + "\n";
            first = false;
            if (nl == std::string::npos) break;
            start = nl + 1;
        }
        if (g_history.empty() || g_history.back() != code) g_history.push_back(code);
        g_histPos = -1;
        runPython(code, "<shell>", true);
        g_input[0] = 0;
        refocus = true;
    }
    ImGui::End();
}

} // namespace hd

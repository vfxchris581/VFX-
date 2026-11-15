// macOS: receive files opened from Finder (double-click on a .hdg, "Open With", dropping on the Dock icon).
// macOS delivers these as Apple Events, not as command-line arguments. Elsewhere these are no-ops.
#pragma once
#include <string>
#include <vector>

namespace hd {
#if defined(__APPLE__)
void installMacOpenFileHandler();          // call before glfwInit()
std::vector<std::string> takeMacOpenedFiles(); // paths received since the last call
#else
inline void installMacOpenFileHandler() {}
inline std::vector<std::string> takeMacOpenedFiles() { return {}; }
#endif
} // namespace hd

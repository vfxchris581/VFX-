// macOS open-document handling. GLFW installs its own NSApplication delegate class
// (GLFWApplicationDelegate) and has no open-file callback, so we add
// -application:openFiles: to that class with the Objective-C runtime. It has to be
// added before glfwInit(), because the launch-time open event arrives while GLFW
// finishes launching the application.
#include "MacOpenFiles.h"
#import <Cocoa/Cocoa.h>
#include <objc/runtime.h>
#include <mutex>

namespace {
std::mutex g_mutex;
std::vector<std::string> g_files;

void hdApplicationOpenFiles(id, SEL, NSApplication* sender, NSArray* filenames) {
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        for (NSString* name in filenames) {
            const char* p = [name fileSystemRepresentation];
            if (p) g_files.emplace_back(p);
        }
    }
    [sender replyToOpenOrPrint:NSApplicationDelegateReplySuccess];
}
} // namespace

namespace hd {
void installMacOpenFileHandler() {
    Class cls = objc_getClass("GLFWApplicationDelegate");
    if (!cls) return;
    class_addMethod(cls, @selector(application:openFiles:), (IMP)hdApplicationOpenFiles, "v@:@@");
}

std::vector<std::string> takeMacOpenedFiles() {
    std::lock_guard<std::mutex> lock(g_mutex);
    std::vector<std::string> out;
    out.swap(g_files);
    return out;
}
} // namespace hd

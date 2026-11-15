#include "core/Version.h"

#ifndef HD_VERSION_STRING
#define HD_VERSION_STRING "0.0.0"
#endif
#ifndef HD_GIT_REVISION
#define HD_GIT_REVISION "unknown"
#endif

namespace hd {
const char* versionString() { return HD_VERSION_STRING; }
const char* gitRevision() { return HD_GIT_REVISION; }
const char* buildDate() { return __DATE__; }
const char* platformName() {
#if defined(__APPLE__)
#  if defined(__aarch64__) || defined(__arm64__)
    return "macOS arm64";
#  else
    return "macOS x86_64";
#  endif
#elif defined(_WIN32)
#  if defined(_M_ARM64)
    return "Windows arm64";
#  else
    return "Windows x64";
#  endif
#elif defined(__linux__)
#  if defined(__aarch64__)
    return "Linux arm64";
#  else
    return "Linux x86_64";
#  endif
#else
    return "unknown platform";
#endif
}
} // namespace hd

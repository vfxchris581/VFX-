// Build identification for HD (filled in by CMake at configure time).
#pragma once

namespace hd {
const char* versionString(); // e.g. "0.7.0"
const char* gitRevision();   // short commit hash, or "unknown" outside a git checkout
const char* buildDate();     // compile date, e.g. "Nov 15 2026"
const char* platformName();  // "macOS arm64", "Windows x64", "Linux x86_64", ...
} // namespace hd

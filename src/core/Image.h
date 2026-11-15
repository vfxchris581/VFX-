// Image I/O and textures, with no external dependencies:
//   zlib inflate/deflate, PNG (read: all standard colour types and bit depths, non-interlaced;
//   write: 8-bit RGB/RGBA, compressed), Radiance HDR (read/write), OpenEXR (scanline, NONE / ZIPS / ZIP,
//   HALF / FLOAT; read and multi-channel write).
#pragma once
#include "Math.h"
#include <cstdint>
#include <string>
#include <vector>

namespace hd {

// zlib streams (RFC 1950/1951).
std::vector<uint8_t> zlibCompress(const uint8_t* data, size_t n, int level = 6);
bool zlibDecompress(const uint8_t* data, size_t n, std::vector<uint8_t>& out, size_t expected = 0);

// Linear float image, c channels interleaved, top row first.
struct Image {
    int w = 0, h = 0, c = 0;
    std::vector<float> px;
    float at(int x, int y, int ch) const { return px[(size_t(y) * size_t(w) + size_t(x)) * size_t(c) + size_t(ch)]; }
};

// Reads .png, .hdr or .exr. 8/16-bit PNGs are converted from sRGB to linear when `srgbToLinear`
// (set it for colour maps, clear it for data maps such as roughness and normals). HDR and EXR are linear.
bool readImage(const std::string& path, Image& img, bool srgbToLinear, std::string* error = nullptr);

bool writePng8(const std::string& path, const std::vector<uint8_t>& data, int w, int h, int channels, std::string* error = nullptr);
bool writeHdr(const std::string& path, const Image& img, std::string* error = nullptr);

// One EXR channel: a name (e.g. "R", "albedo.G", "Z") and a w*h plane of values.
struct ExrChannel { std::string name; const float* data; };
// Writes a scanline EXR with ZIP compression. half = 16-bit floats, else 32-bit.
bool writeExr(const std::string& path, int w, int h, const std::vector<ExrChannel>& channels, bool half, std::string* error = nullptr);
// Reads every channel of a scanline EXR (NONE, ZIPS or ZIP compression).
bool readExrChannels(const std::string& path, int& w, int& h, std::vector<std::pair<std::string, std::vector<float>>>& channels,
                     std::string* error = nullptr);

float halfToFloat(uint16_t h);
uint16_t floatToHalf(float f);

// ---- file paths
// "$HIP" at the start of a path means the folder of the scene file (set when a scene is loaded or saved).
void setSceneDirectory(const std::string& dir);
std::string resolvePath(const std::string& path);

// ---- textures used by the renderer
// Material nodes register file paths; geometry stores the returned id (a float attribute).
int registerTexturePath(const std::string& path);
std::string texturePath(int id);

// A loaded texture, or a UDIM set (path contains <UDIM>: tiles 1001.. by uv).
class Texture {
public:
    bool load(const std::string& path, bool color, std::string* error = nullptr);
    // Bilinear lookup with wrapping (or the UDIM tile under uv). Returns 4 channels (RGBA).
    void sample(float u, float v, float out[4]) const;
    bool valid() const { return !tiles_.empty(); }
private:
    struct Tile { int udim = 0; Image img; };
    std::vector<Tile> tiles_;
    bool udim_ = false;
    const Image* tileFor(float u, float v, float& lu, float& lv) const;
};

} // namespace hd

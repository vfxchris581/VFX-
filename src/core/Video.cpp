// H.264 Constrained Baseline encoder (CAVLC) and an MP4 / MOV writer. See Video.h.
//
// Everything that a decoder reconstructs (prediction, dequantisation, inverse transforms, the loop
// filter) follows ITU-T H.264 exactly, so the encoder's reference pictures match the decoder's and
// P frames don't drift. The encoder-only choices (mode decision, motion search, quantiser rounding)
// are simple and fast.
#include "Video.h"
#include "Image.h"
#include "ImageSequence.h"
#include "Volume.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

namespace hd {

int movieQpForQuality(int preset) { return preset <= 0 ? 18 : preset == 1 ? 23 : 28; }

namespace {

// ------------------------------------------------------------------ bits

struct Bits {
    std::vector<uint8_t> buf;
    uint64_t acc = 0;
    int n = 0;
    void u(int bits, uint32_t v) {
        while (bits > 0) {
            int take = std::min(bits, 24);
            uint32_t part = bits > 24 ? (v >> (bits - take)) & ((1u << take) - 1) : v & ((take == 32 ? 0xFFFFFFFFu : (1u << take) - 1));
            acc = (acc << take) | part;
            n += take;
            bits -= take;
            while (n >= 8) { buf.push_back(uint8_t(acc >> (n - 8))); n -= 8; }
            acc &= n ? ((1ull << n) - 1) : 0;
        }
    }
    void ue(uint32_t k) {
        uint64_t v = uint64_t(k) + 1;
        int len = 0;
        while ((v >> len) > 1) ++len; // floor(log2)
        u(len, 0);
        u(len + 1, uint32_t(v));
    }
    void se(int v) { ue(v > 0 ? uint32_t(2 * v - 1) : uint32_t(-2 * v)); }
    void trailing() { u(1, 1); while (n) u(1, 0); }
};

int ueBits(uint32_t k) {
    uint64_t v = uint64_t(k) + 1;
    int len = 0;
    while ((v >> len) > 1) ++len;
    return 2 * len + 1;
}
int seBits(int v) { return ueBits(v > 0 ? uint32_t(2 * v - 1) : uint32_t(-2 * v)); }

// NAL unit: header byte, then the payload with emulation prevention bytes.
std::vector<uint8_t> nal(int refIdc, int type, const std::vector<uint8_t>& rbsp) {
    std::vector<uint8_t> out;
    out.reserve(rbsp.size() + rbsp.size() / 64 + 8);
    out.push_back(uint8_t((refIdc << 5) | type));
    int zeros = 0;
    for (uint8_t b : rbsp) {
        if (zeros >= 2 && b <= 3) { out.push_back(3); zeros = 0; }
        out.push_back(b);
        zeros = b == 0 ? zeros + 1 : 0;
    }
    return out;
}

// ------------------------------------------------------------------ CAVLC tables (ITU-T H.264 9.2)

const uint8_t kCoeffTokenLen[4][4 * 17] = {
    {1, 0, 0, 0, 6, 2, 0, 0, 8, 6, 3, 0, 9, 8, 7, 5, 10, 9, 8, 6, 11, 10, 9, 7, 13, 11, 10, 8, 13, 13, 11, 9, 13, 13, 13, 10,
     14, 14, 13, 11, 14, 14, 14, 13, 15, 15, 14, 14, 15, 15, 15, 14, 16, 15, 15, 15, 16, 16, 16, 15, 16, 16, 16, 16, 16, 16, 16, 16},
    {2, 0, 0, 0, 6, 2, 0, 0, 6, 5, 3, 0, 7, 6, 6, 4, 8, 6, 6, 4, 8, 7, 7, 5, 9, 8, 8, 6, 11, 9, 9, 6, 11, 11, 11, 7,
     12, 11, 11, 9, 12, 12, 12, 11, 12, 12, 12, 11, 13, 13, 13, 12, 13, 13, 13, 13, 13, 14, 13, 13, 14, 14, 14, 13, 14, 14, 14, 14},
    {4, 0, 0, 0, 6, 4, 0, 0, 6, 5, 4, 0, 6, 5, 5, 4, 7, 5, 5, 4, 7, 5, 5, 4, 7, 6, 6, 4, 7, 6, 6, 4, 8, 7, 7, 5,
     8, 8, 7, 6, 9, 8, 8, 7, 9, 9, 8, 8, 9, 9, 9, 8, 10, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
    {6, 0, 0, 0, 6, 6, 0, 0, 6, 6, 6, 0, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
     6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6},
};
const uint8_t kCoeffTokenBits[4][4 * 17] = {
    {1, 0, 0, 0, 5, 1, 0, 0, 7, 4, 1, 0, 7, 6, 5, 3, 7, 6, 5, 3, 7, 6, 5, 4, 15, 6, 5, 4, 11, 14, 5, 4, 8, 10, 13, 4,
     15, 14, 9, 4, 11, 10, 13, 12, 15, 14, 9, 12, 11, 10, 13, 8, 15, 1, 9, 12, 11, 14, 13, 8, 7, 10, 9, 12, 4, 6, 5, 8},
    {3, 0, 0, 0, 11, 2, 0, 0, 7, 7, 3, 0, 7, 10, 9, 5, 7, 6, 5, 4, 4, 6, 5, 6, 7, 6, 5, 8, 15, 6, 5, 4, 11, 14, 13, 4,
     15, 10, 9, 4, 11, 14, 13, 12, 8, 10, 9, 8, 15, 14, 13, 12, 11, 10, 9, 12, 7, 11, 6, 8, 9, 8, 10, 1, 7, 6, 5, 4},
    {15, 0, 0, 0, 15, 14, 0, 0, 11, 15, 13, 0, 8, 12, 14, 12, 15, 10, 11, 11, 11, 8, 9, 10, 9, 14, 13, 9, 8, 10, 9, 8, 15, 14, 13, 13,
     11, 14, 10, 12, 15, 10, 13, 12, 11, 14, 9, 12, 8, 10, 13, 8, 13, 7, 9, 12, 9, 12, 11, 10, 5, 8, 7, 6, 1, 4, 3, 2},
    {3, 0, 0, 0, 0, 1, 0, 0, 4, 5, 6, 0, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
     32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63},
};
const uint8_t kChromaDcTokenLen[4 * 5] = {2, 0, 0, 0, 6, 1, 0, 0, 6, 6, 3, 0, 6, 7, 7, 6, 6, 8, 8, 7};
const uint8_t kChromaDcTokenBits[4 * 5] = {1, 0, 0, 0, 7, 1, 0, 0, 4, 6, 1, 0, 3, 3, 2, 5, 2, 3, 2, 0};
const uint8_t kTotalZerosLen[15][16] = {
    {1, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9}, {3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 6, 6}, {4, 3, 3, 3, 4, 4, 3, 3, 4, 5, 5, 6, 5, 6},
    {5, 3, 4, 4, 3, 3, 3, 4, 3, 4, 5, 5, 5},          {4, 4, 4, 3, 3, 3, 3, 3, 4, 5, 4, 5},          {6, 5, 3, 3, 3, 3, 3, 3, 4, 3, 6},
    {6, 5, 3, 3, 3, 2, 3, 4, 3, 6},                   {6, 4, 5, 3, 2, 2, 3, 3, 6},                   {6, 6, 4, 2, 2, 3, 2, 5},
    {5, 5, 3, 2, 2, 2, 4},                            {4, 4, 3, 3, 1, 3},                            {4, 4, 2, 1, 3},
    {3, 3, 1, 2},                                     {2, 2, 1},                                     {1, 1},
};
const uint8_t kTotalZerosBits[15][16] = {
    {1, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 1}, {7, 6, 5, 4, 3, 5, 4, 3, 2, 3, 2, 3, 2, 1, 0}, {5, 7, 6, 5, 4, 3, 4, 3, 2, 3, 2, 1, 1, 0},
    {3, 7, 5, 4, 6, 5, 4, 3, 3, 2, 2, 1, 0},          {5, 4, 3, 7, 6, 5, 4, 3, 2, 1, 1, 0},          {1, 1, 7, 6, 5, 4, 3, 2, 1, 1, 0},
    {1, 1, 5, 4, 3, 3, 2, 1, 1, 0},                   {1, 1, 1, 3, 3, 2, 2, 1, 0},                   {1, 0, 1, 3, 2, 1, 1, 1},
    {1, 0, 1, 3, 2, 1, 1},                            {0, 1, 1, 2, 1, 3},                            {0, 1, 1, 1, 1},
    {0, 1, 1, 1},                                     {0, 1, 1},                                     {0, 1},
};
const uint8_t kChromaDcTotalZerosLen[3][4] = {{1, 2, 3, 3}, {1, 2, 2, 0}, {1, 1, 0, 0}};
const uint8_t kChromaDcTotalZerosBits[3][4] = {{1, 1, 1, 0}, {1, 1, 0, 0}, {1, 0, 0, 0}};
const uint8_t kRunLen[7][16] = {
    {1, 1}, {1, 2, 2}, {2, 2, 2, 2}, {2, 2, 2, 3, 3}, {2, 2, 3, 3, 3, 3}, {2, 3, 3, 3, 3, 3, 3}, {3, 3, 3, 3, 3, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11},
};
const uint8_t kRunBits[7][16] = {
    {1, 0}, {1, 1, 0}, {3, 2, 1, 0}, {3, 2, 1, 1, 0}, {3, 2, 3, 2, 1, 0}, {3, 0, 1, 3, 2, 5, 4}, {7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};
// coded_block_pattern for inter macroblocks: codeNum -> cbp (Table 9-4)
const uint8_t kGolombToInterCbp[48] = {0,  16, 1,  2,  4,  8,  32, 3,  5,  10, 12, 15, 47, 7,  11, 13, 14, 6,  9,  31, 35, 37, 42, 44,
                                       33, 34, 36, 40, 39, 43, 45, 46, 17, 18, 20, 24, 19, 21, 26, 28, 23, 27, 29, 30, 22, 25, 38, 41};

const int kZigzag[16] = {0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15}; // scan index -> raster (y*4+x)

// Writes one residual block (coefficients in scan order). Returns TotalCoeff.
int writeBlock(Bits& b, const int* c, int maxNum, int nC) {
    int lev[16], pos[16], total = 0;
    for (int i = maxNum - 1; i >= 0; --i)
        if (c[i]) { lev[total] = c[i]; pos[total] = i; ++total; }
    int t1 = 0;
    for (int k = 0; k < total && k < 3; ++k) {
        if (std::abs(lev[k]) == 1) ++t1;
        else break;
    }
    if (nC == -1) b.u(kChromaDcTokenLen[total * 4 + t1], kChromaDcTokenBits[total * 4 + t1]);
    else {
        int tab = nC < 2 ? 0 : nC < 4 ? 1 : nC < 8 ? 2 : 3;
        b.u(kCoeffTokenLen[tab][total * 4 + t1], kCoeffTokenBits[tab][total * 4 + t1]);
    }
    if (!total) return 0;
    int suffixLength = total > 10 && t1 < 3 ? 1 : 0;
    for (int k = 0; k < total; ++k) {
        if (k < t1) { b.u(1, lev[k] < 0 ? 1 : 0); continue; }
        int level = lev[k];
        int levelCode = level > 0 ? 2 * level - 2 : -2 * level - 1;
        if (k == t1 && t1 < 3) levelCode -= 2;
        int prefix, suffix = 0, suffixSize = 0;
        if (suffixLength == 0) {
            if (levelCode < 14) prefix = levelCode;
            else if (levelCode < 30) { prefix = 14; suffix = levelCode - 14; suffixSize = 4; }
            else { prefix = 15; suffix = levelCode - 30; suffixSize = 12; }
        } else {
            if (levelCode < (15 << suffixLength)) { prefix = levelCode >> suffixLength; suffix = levelCode & ((1 << suffixLength) - 1); suffixSize = suffixLength; }
            else { prefix = 15; suffix = levelCode - (15 << suffixLength); suffixSize = 12; }
        }
        b.u(prefix, 0);
        b.u(1, 1);
        if (suffixSize) b.u(suffixSize, uint32_t(suffix));
        if (suffixLength == 0) suffixLength = 1;
        if (std::abs(level) > (3 << (suffixLength - 1)) && suffixLength < 6) ++suffixLength;
    }
    int totalZeros = pos[0] + 1 - total;
    if (total < maxNum) {
        if (maxNum == 4) b.u(kChromaDcTotalZerosLen[total - 1][totalZeros], kChromaDcTotalZerosBits[total - 1][totalZeros]);
        else b.u(kTotalZerosLen[total - 1][totalZeros], kTotalZerosBits[total - 1][totalZeros]);
    }
    int zerosLeft = totalZeros;
    for (int k = 0; k < total - 1 && zerosLeft > 0; ++k) {
        int run = pos[k] - pos[k + 1] - 1;
        int t = std::min(zerosLeft, 7) - 1;
        b.u(kRunLen[t][run], kRunBits[t][run]);
        zerosLeft -= run;
    }
    return total;
}

// ------------------------------------------------------------------ transforms and quantisation

const int kMF[6][3] = {{13107, 5243, 8066}, {11916, 4660, 7490}, {10082, 4194, 6554}, {9362, 3647, 5825}, {8192, 3355, 5243}, {7282, 2893, 4559}};
const int kV[6][3] = {{10, 16, 13}, {11, 18, 14}, {13, 20, 16}, {14, 23, 18}, {16, 25, 20}, {18, 29, 23}};
inline int posClass(int r) {
    int x = r & 3, y = r >> 2;
    if (!(x & 1) && !(y & 1)) return 0;
    if ((x & 1) && (y & 1)) return 1;
    return 2;
}
const int kChromaQp[52] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
                           26, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 39, 39};

void fwd4(const int* in, int* out) {
    int t[16];
    for (int y = 0; y < 4; ++y) {
        const int* r = in + y * 4;
        int s03 = r[0] + r[3], d03 = r[0] - r[3], s12 = r[1] + r[2], d12 = r[1] - r[2];
        t[y * 4 + 0] = s03 + s12; t[y * 4 + 1] = 2 * d03 + d12; t[y * 4 + 2] = s03 - s12; t[y * 4 + 3] = d03 - 2 * d12;
    }
    for (int x = 0; x < 4; ++x) {
        int a = t[x], b = t[4 + x], c = t[8 + x], d = t[12 + x];
        int s03 = a + d, d03 = a - d, s12 = b + c, d12 = b - c;
        out[x] = s03 + s12; out[4 + x] = 2 * d03 + d12; out[8 + x] = s03 - s12; out[12 + x] = d03 - 2 * d12;
    }
}
// 8.5.12.2: rows (horizontal) first, then columns, then (x + 32) >> 6
void inv4(int* d) {
    for (int y = 0; y < 4; ++y) {
        int* r = d + y * 4;
        int e0 = r[0] + r[2], e1 = r[0] - r[2], e2 = (r[1] >> 1) - r[3], e3 = r[1] + (r[3] >> 1);
        r[0] = e0 + e3; r[1] = e1 + e2; r[2] = e1 - e2; r[3] = e0 - e3;
    }
    for (int x = 0; x < 4; ++x) {
        int a = d[x], b = d[4 + x], c = d[8 + x], e = d[12 + x];
        int e0 = a + c, e1 = a - c, e2 = (b >> 1) - e, e3 = b + (e >> 1);
        d[x] = (e0 + e3 + 32) >> 6; d[4 + x] = (e1 + e2 + 32) >> 6; d[8 + x] = (e1 - e2 + 32) >> 6; d[12 + x] = (e0 - e3 + 32) >> 6;
    }
}
void hadamard4(const int* in, int* out) {
    int t[16];
    for (int y = 0; y < 4; ++y) {
        const int* r = in + y * 4;
        int a = r[0] + r[1], b = r[0] - r[1], c = r[2] + r[3], d = r[2] - r[3];
        t[y * 4 + 0] = a + c; t[y * 4 + 1] = a - c; t[y * 4 + 2] = b - d; t[y * 4 + 3] = b + d; // rows of H: 1 1 1 1 / 1 1 -1 -1 / 1 -1 -1 1 / 1 -1 1 -1
    }
    for (int x = 0; x < 4; ++x) {
        int r0 = t[x], r1 = t[4 + x], r2 = t[8 + x], r3 = t[12 + x];
        int a = r0 + r1, b = r0 - r1, c = r2 + r3, d = r2 - r3;
        out[x] = a + c; out[4 + x] = a - c; out[8 + x] = b - d; out[12 + x] = b + d;
    }
}
inline int quant(int w, int mf, int f, int qbits) {
    int a = (std::abs(w) * mf + f) >> qbits;
    a = std::min(a, 2047);
    return w < 0 ? -a : a;
}
inline uint8_t clip1(int v) { return uint8_t(v < 0 ? 0 : v > 255 ? 255 : v); }

// ------------------------------------------------------------------ deblocking tables (8.7)

const uint8_t kAlpha[52] = {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  4,  4,  5,  6,  7,  8,  9,  10, 12, 13,
                            15, 17, 20, 22, 25, 28, 32, 36, 40, 45, 50, 56, 63, 71, 80, 90, 101, 113, 127, 144, 162, 182, 203, 226, 255, 255};
const uint8_t kBeta[52] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4,
                           6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18};
const uint8_t kTc0[52][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
                             {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 1},
                             {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 1, 1}, {0, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1},
                             {1, 1, 2}, {1, 1, 2}, {1, 1, 2}, {1, 1, 2}, {1, 2, 3}, {1, 2, 3}, {2, 2, 3}, {2, 2, 4}, {2, 3, 4},
                             {2, 3, 4}, {3, 3, 5}, {3, 4, 6}, {3, 4, 6}, {4, 5, 7}, {4, 5, 8}, {4, 6, 9}, {5, 7, 10}, {6, 8, 11},
                             {6, 8, 13}, {7, 10, 14}, {8, 11, 16}, {9, 12, 18}, {10, 13, 20}, {11, 15, 23}, {13, 17, 25}};

// ------------------------------------------------------------------ pictures

struct Plane {
    int w = 0, h = 0;
    std::vector<uint8_t> p;
    void alloc(int W, int H) { w = W; h = H; p.assign(size_t(W) * size_t(H), 0); }
    uint8_t* row(int y) { return &p[size_t(y) * size_t(w)]; }
    const uint8_t* row(int y) const { return &p[size_t(y) * size_t(w)]; }
    uint8_t at(int x, int y) const { return p[size_t(std::clamp(y, 0, h - 1)) * size_t(w) + size_t(std::clamp(x, 0, w - 1))]; }
};
struct Picture {
    Plane y, u, v;
    void alloc(int W, int H) { y.alloc(W, H); u.alloc(W / 2, H / 2); v.alloc(W / 2, H / 2); }
};

struct MbInfo {
    int8_t nz[16];   // TotalCoeff per luma 4x4 block (blkIdx order)
    int8_t nzc[2][4]; // chroma AC TotalCoeff per 4x4 block (raster)
    int16_t mvx, mvy; // quarter-pel
    int8_t ref;      // 0 inter, -1 intra
    bool intra, skip;
    int slice;
    int8_t qp;
    // deblocking: non-zero transform coefficients per luma 4x4 block (raster 4x4 inside the MB)
    uint16_t coded; // bit (y*4+x) set when that 4x4 block has non-zero coefficients
};

inline int blkIdxOf(int x4, int y4) { return (y4 >> 1) * 8 + (x4 >> 1) * 4 + (y4 & 1) * 2 + (x4 & 1); }
inline void blkXY(int blk, int& x4, int& y4) {
    x4 = ((blk >> 2) & 1) * 2 + (blk & 1);
    y4 = (blk >> 3) * 2 + ((blk >> 1) & 1);
}

// ------------------------------------------------------------------ the encoder

struct Encoder {
    int W = 0, H = 0;       // output size (even)
    int mbw = 0, mbh = 0;   // in macroblocks
    int qp = 23, qpc = 23;
    float fps = 24;
    int keyint = 48;
    int nslices = 1;
    bool deblock = true;
    int levelIdc = 40;
    int frameNum = 0, idrId = 0, coded = 0;
    Picture src, rec, ref;
    std::vector<MbInfo> mb;
    std::vector<uint8_t> sps, pps;
    float lambda = 1;

    void init(int w, int h, const MovieOptions& o) {
        W = w; H = h;
        mbw = (W + 15) / 16; mbh = (H + 15) / 16;
        qp = std::clamp(o.qp, 0, 51);
        qpc = kChromaQp[qp];
        fps = o.fps > 0 ? o.fps : 24;
        keyint = o.keyint > 0 ? o.keyint : std::max(1, int(std::lround(fps * 2)));
        deblock = o.deblock;
        int s = o.slices;
        if (s <= 0) s = int(std::clamp(std::thread::hardware_concurrency(), 1u, 8u));
        nslices = std::clamp(s, 1, mbh);
        src.alloc(mbw * 16, mbh * 16);
        rec.alloc(mbw * 16, mbh * 16);
        ref.alloc(mbw * 16, mbh * 16);
        mb.assign(size_t(mbw) * size_t(mbh), MbInfo());
        lambda = 0.92f * std::pow(2.f, float(qp - 12) / 6.f);
        // level: frame size and macroblock rate (Table A-1)
        const int fs = mbw * mbh;
        const double mbps = double(fs) * double(fps);
        struct L { int idc, maxFs; double maxMbps; };
        static const L levels[] = {{30, 1620, 40500}, {31, 3600, 108000}, {32, 5120, 216000}, {40, 8192, 245760}, {42, 8704, 522240},
                                   {50, 22080, 589824}, {51, 36864, 983040}, {52, 36864, 2073600}};
        levelIdc = 52;
        for (auto& l : levels)
            if (fs <= l.maxFs && mbps <= l.maxMbps && mbw * mbw <= l.maxFs * 8 && mbh * mbh <= l.maxFs * 8) { levelIdc = l.idc; break; }
        writeParameterSets();
    }

    void writeParameterSets() {
        Bits b;
        b.u(8, 66);   // profile: baseline
        b.u(8, 0xC0); // constraint_set0 and set1: constrained baseline
        b.u(8, uint32_t(levelIdc));
        b.ue(0);      // sps id
        b.ue(12);     // log2_max_frame_num - 4 (16 bits)
        b.ue(2);      // pic_order_cnt_type 2: output order = decode order
        b.ue(1);      // max_num_ref_frames
        b.u(1, 0);    // gaps_in_frame_num_value_allowed_flag
        b.ue(uint32_t(mbw - 1));
        b.ue(uint32_t(mbh - 1));
        b.u(1, 1);    // frame_mbs_only_flag
        b.u(1, 1);    // direct_8x8_inference_flag
        const int cropR = (mbw * 16 - W) / 2, cropB = (mbh * 16 - H) / 2;
        b.u(1, cropR || cropB ? 1 : 0);
        if (cropR || cropB) { b.ue(0); b.ue(uint32_t(cropR)); b.ue(0); b.ue(uint32_t(cropB)); }
        b.u(1, 1);    // vui_parameters_present_flag
        b.u(1, 1);    // aspect_ratio_info_present_flag
        b.u(8, 1);    // square pixels
        b.u(1, 0);    // overscan_info_present_flag
        b.u(1, 1);    // video_signal_type_present_flag
        b.u(3, 5);    // video_format: unspecified
        b.u(1, 0);    // video_full_range_flag: video range
        b.u(1, 1);    // colour_description_present_flag
        b.u(8, 1); b.u(8, 1); b.u(8, 1); // BT.709 primaries, transfer, matrix
        b.u(1, 0);    // chroma_loc_info_present_flag
        b.u(1, 1);    // timing_info_present_flag
        uint32_t tick = 1000, scale = uint32_t(std::lround(double(fps) * 2000.0));
        b.u(32, tick);
        b.u(32, scale);
        b.u(1, 1);    // fixed_frame_rate_flag
        b.u(1, 0); b.u(1, 0); // nal / vcl hrd
        b.u(1, 0);    // pic_struct_present_flag
        b.u(1, 1);    // bitstream_restriction_flag
        b.u(1, 1);    // motion_vectors_over_pic_boundaries_flag
        b.ue(0); b.ue(0); // max_bytes_per_pic_denom, max_bits_per_mb_denom
        b.ue(16); b.ue(16); // log2_max_mv_length
        b.ue(0);      // max_num_reorder_frames
        b.ue(1);      // max_dec_frame_buffering
        b.trailing();
        sps = nal(3, 7, b.buf);
        Bits p;
        p.ue(0); p.ue(0); // pps id, sps id
        p.u(1, 0);    // entropy_coding_mode_flag: CAVLC
        p.u(1, 0);    // bottom_field_pic_order_in_frame_present_flag
        p.ue(0);      // num_slice_groups_minus1
        p.ue(0); p.ue(0); // num_ref_idx_l0/l1_default_active_minus1
        p.u(1, 0); p.u(2, 0); // weighted prediction off
        p.se(qp - 26); // pic_init_qp_minus26
        p.se(0);      // pic_init_qs_minus26
        p.se(0);      // chroma_qp_index_offset
        p.u(1, 1);    // deblocking_filter_control_present_flag
        p.u(1, 0);    // constrained_intra_pred_flag
        p.u(1, 0);    // redundant_pic_cnt_present_flag
        p.trailing();
        pps = nal(3, 8, p.buf);
    }

    // RGBA -> BT.709 video-range YUV 4:2:0, edges repeated out to the macroblock grid
    void loadSource(const uint8_t* rgba, int sw) {
        const int PW = mbw * 16, PH = mbh * 16;
        std::vector<float> Y(size_t(PW) * PH), Cb(size_t(PW) * PH), Cr(size_t(PW) * PH);
        for (int y = 0; y < PH; ++y) {
            const int sy = std::min(y, H - 1);
            for (int x = 0; x < PW; ++x) {
                const int sx = std::min(x, W - 1);
                const uint8_t* p = rgba + (size_t(sy) * size_t(sw) + size_t(sx)) * 4;
                float a = p[3] / 255.f; // transparent areas over black
                float r = p[0] / 255.f * a, g = p[1] / 255.f * a, bl = p[2] / 255.f * a;
                float yy = 0.2126f * r + 0.7152f * g + 0.0722f * bl;
                size_t i = size_t(y) * size_t(PW) + size_t(x);
                Y[i] = 16 + 219 * yy;
                Cb[i] = 128 + 224 * (bl - yy) / 1.8556f;
                Cr[i] = 128 + 224 * (r - yy) / 1.5748f;
            }
        }
        for (size_t i = 0; i < Y.size(); ++i) src.y.p[i] = clip1(int(std::lround(Y[i])));
        for (int y = 0; y < PH / 2; ++y)
            for (int x = 0; x < PW / 2; ++x) {
                size_t i0 = size_t(2 * y) * size_t(PW) + size_t(2 * x), i1 = i0 + size_t(PW);
                float cb = (Cb[i0] + Cb[i0 + 1] + Cb[i1] + Cb[i1 + 1]) * 0.25f;
                float cr = (Cr[i0] + Cr[i0 + 1] + Cr[i1] + Cr[i1 + 1]) * 0.25f;
                src.u.p[size_t(y) * size_t(PW / 2) + size_t(x)] = clip1(int(std::lround(cb)));
                src.v.p[size_t(y) * size_t(PW / 2) + size_t(x)] = clip1(int(std::lround(cr)));
            }
    }

    // ---- availability (same slice, already coded)
    bool avail(int mx, int my, int slice) const {
        if (mx < 0 || my < 0 || mx >= mbw || my >= mbh) return false;
        return mb[size_t(my) * size_t(mbw) + size_t(mx)].slice == slice;
    }
    const MbInfo& info(int mx, int my) const { return mb[size_t(my) * size_t(mbw) + size_t(mx)]; }
    MbInfo& info(int mx, int my) { return mb[size_t(my) * size_t(mbw) + size_t(mx)]; }

    // ---- intra prediction (8.3.3, 8.3.4)
    void predLuma16(int mode, int mx, int my, bool hasL, bool hasT, uint8_t* pred) const {
        const Plane& P = rec.y;
        const int x0 = mx * 16, y0 = my * 16;
        if (mode == 0) { for (int y = 0; y < 16; ++y) for (int x = 0; x < 16; ++x) pred[y * 16 + x] = P.row(y0 - 1)[x0 + x]; }
        else if (mode == 1) { for (int y = 0; y < 16; ++y) for (int x = 0; x < 16; ++x) pred[y * 16 + x] = P.row(y0 + y)[x0 - 1]; }
        else if (mode == 2) {
            int s = 0, dc = 128;
            if (hasL && hasT) { for (int i = 0; i < 16; ++i) s += P.row(y0 - 1)[x0 + i] + P.row(y0 + i)[x0 - 1]; dc = (s + 16) >> 5; }
            else if (hasL) { for (int i = 0; i < 16; ++i) s += P.row(y0 + i)[x0 - 1]; dc = (s + 8) >> 4; }
            else if (hasT) { for (int i = 0; i < 16; ++i) s += P.row(y0 - 1)[x0 + i]; dc = (s + 8) >> 4; }
            std::memset(pred, dc, 256);
        } else {
            auto top = [&](int x) { return int(P.row(y0 - 1)[x0 + x]); }; // x = -1 is the corner
            auto left = [&](int y) { return int(P.row(y0 + y)[x0 - 1]); };
            int Hs = 0, Vs = 0;
            for (int i = 0; i < 8; ++i) {
                Hs += (i + 1) * (top(8 + i) - top(6 - i));
                Vs += (i + 1) * (left(8 + i) - (6 - i >= 0 ? left(6 - i) : top(-1)));
            }
            int a = 16 * (left(15) + top(15)), b = (5 * Hs + 32) >> 6, c = (5 * Vs + 32) >> 6;
            for (int y = 0; y < 16; ++y)
                for (int x = 0; x < 16; ++x) pred[y * 16 + x] = clip1((a + b * (x - 7) + c * (y - 7) + 16) >> 5);
        }
    }
    void predChroma(int mode, const Plane& P, int mx, int my, bool hasL, bool hasT, uint8_t* pred) const {
        const int x0 = mx * 8, y0 = my * 8;
        if (mode == 0) {
            for (int by = 0; by < 2; ++by)
                for (int bx = 0; bx < 2; ++bx) {
                    int sT = 0, sL = 0;
                    if (hasT) for (int i = 0; i < 4; ++i) sT += P.row(y0 - 1)[x0 + bx * 4 + i];
                    if (hasL) for (int i = 0; i < 4; ++i) sL += P.row(y0 + by * 4 + i)[x0 - 1];
                    int dc = 128;
                    if ((bx == 0 && by == 0) || (bx == 1 && by == 1)) {
                        if (hasT && hasL) dc = (sT + sL + 4) >> 3;
                        else if (hasL) dc = (sL + 2) >> 2;
                        else if (hasT) dc = (sT + 2) >> 2;
                    } else if (bx == 1) { // top right
                        if (hasT) dc = (sT + 2) >> 2;
                        else if (hasL) dc = (sL + 2) >> 2;
                    } else { // bottom left
                        if (hasL) dc = (sL + 2) >> 2;
                        else if (hasT) dc = (sT + 2) >> 2;
                    }
                    for (int y = 0; y < 4; ++y)
                        for (int x = 0; x < 4; ++x) pred[(by * 4 + y) * 8 + bx * 4 + x] = uint8_t(dc);
                }
        } else if (mode == 1) { for (int y = 0; y < 8; ++y) for (int x = 0; x < 8; ++x) pred[y * 8 + x] = P.row(y0 + y)[x0 - 1]; }
        else if (mode == 2) { for (int y = 0; y < 8; ++y) for (int x = 0; x < 8; ++x) pred[y * 8 + x] = P.row(y0 - 1)[x0 + x]; }
        else {
            auto top = [&](int x) { return int(P.row(y0 - 1)[x0 + x]); };
            auto left = [&](int y) { return int(P.row(y0 + y)[x0 - 1]); };
            int Hs = 0, Vs = 0;
            for (int i = 0; i < 4; ++i) {
                Hs += (i + 1) * (top(4 + i) - top(2 - i));
                Vs += (i + 1) * (left(4 + i) - (2 - i >= 0 ? left(2 - i) : top(-1)));
            }
            int a = 16 * (left(7) + top(7)), b = (34 * Hs + 32) >> 6, c = (34 * Vs + 32) >> 6;
            for (int y = 0; y < 8; ++y)
                for (int x = 0; x < 8; ++x) pred[y * 8 + x] = clip1((a + b * (x - 3) + c * (y - 3) + 16) >> 5);
        }
    }

    // ---- inter prediction (8.4.2.2): full-pel luma, bilinear chroma
    void predInter(int mx, int my, int mvx, int mvy, uint8_t* py, uint8_t* pu, uint8_t* pv) const {
        const int x0 = mx * 16 + (mvx >> 2), y0 = my * 16 + (mvy >> 2);
        const Plane& R = ref.y;
        if (x0 >= 0 && y0 >= 0 && x0 + 16 <= R.w && y0 + 16 <= R.h) {
            for (int y = 0; y < 16; ++y) std::memcpy(py + y * 16, R.row(y0 + y) + x0, 16);
        } else {
            for (int y = 0; y < 16; ++y)
                for (int x = 0; x < 16; ++x) py[y * 16 + x] = R.at(x0 + x, y0 + y);
        }
        const int cx = mx * 8 + (mvx >> 3), cy = my * 8 + (mvy >> 3), fx = mvx & 7, fy = mvy & 7;
        for (int c = 0; c < 2; ++c) {
            const Plane& C = c ? ref.v : ref.u;
            uint8_t* out = c ? pv : pu;
            for (int y = 0; y < 8; ++y)
                for (int x = 0; x < 8; ++x) {
                    int A = C.at(cx + x, cy + y), B = C.at(cx + x + 1, cy + y), Cc = C.at(cx + x, cy + y + 1), D = C.at(cx + x + 1, cy + y + 1);
                    out[y * 8 + x] = uint8_t(((8 - fx) * (8 - fy) * A + fx * (8 - fy) * B + (8 - fx) * fy * Cc + fx * fy * D + 32) >> 6);
                }
        }
    }
    int sad16(int mx, int my, int mvx, int mvy) const {
        const int x0 = mx * 16 + mvx, y0 = my * 16 + mvy; // full pel
        const Plane& R = ref.y;
        int s = 0;
        if (x0 >= 0 && y0 >= 0 && x0 + 16 <= R.w && y0 + 16 <= R.h) {
            for (int y = 0; y < 16; ++y) {
                const uint8_t* a = src.y.row(my * 16 + y) + mx * 16;
                const uint8_t* b = R.row(y0 + y) + x0;
                for (int x = 0; x < 16; ++x) s += std::abs(int(a[x]) - int(b[x]));
            }
        } else {
            for (int y = 0; y < 16; ++y) {
                const uint8_t* a = src.y.row(my * 16 + y) + mx * 16;
                for (int x = 0; x < 16; ++x) s += std::abs(int(a[x]) - int(R.at(x0 + x, y0 + y)));
            }
        }
        return s;
    }

    // ---- motion vector prediction (8.4.1.3) for 16x16 partitions
    struct Nb { bool avail; int ref, mvx, mvy; };
    Nb neighbour(int mx, int my, int slice) const {
        if (!avail(mx, my, slice)) return {false, -1, 0, 0};
        const MbInfo& m = info(mx, my);
        if (m.intra) return {true, -1, 0, 0};
        return {true, 0, m.mvx, m.mvy};
    }
    void mvPred(int mx, int my, int slice, int& px, int& py, int& skx, int& sky) const {
        Nb A = neighbour(mx - 1, my, slice), B = neighbour(mx, my - 1, slice), C = neighbour(mx + 1, my - 1, slice);
        if (!C.avail) C = neighbour(mx - 1, my - 1, slice);
        // P_Skip (8.4.1.1)
        bool skipZero = !A.avail || !B.avail || (A.ref == 0 && A.mvx == 0 && A.mvy == 0) || (B.ref == 0 && B.mvx == 0 && B.mvy == 0);
        if (!B.avail && !C.avail && A.avail) { B = A; C = A; }
        int match = (A.ref == 0) + (B.ref == 0) + (C.ref == 0);
        if (match == 1) {
            const Nb& s = A.ref == 0 ? A : B.ref == 0 ? B : C;
            px = s.mvx; py = s.mvy;
        } else {
            auto med = [](int a, int b, int c) { return std::max(std::min(a, b), std::min(std::max(a, b), c)); };
            px = med(A.mvx, B.mvx, C.mvx);
            py = med(A.mvy, B.mvy, C.mvy);
        }
        if (skipZero) { skx = 0; sky = 0; }
        else { skx = px; sky = py; }
    }

    // ---- nC for a luma 4x4 block (9.2.1)
    int nCLuma(int mx, int my, int x4, int y4, int slice, const int8_t* curNz) const {
        int nA = -1, nB = -1;
        if (x4 > 0) nA = curNz[blkIdxOf(x4 - 1, y4)];
        else if (avail(mx - 1, my, slice)) nA = info(mx - 1, my).nz[blkIdxOf(3, y4)];
        if (y4 > 0) nB = curNz[blkIdxOf(x4, y4 - 1)];
        else if (avail(mx, my - 1, slice)) nB = info(mx, my - 1).nz[blkIdxOf(x4, 3)];
        if (nA >= 0 && nB >= 0) return (nA + nB + 1) >> 1;
        if (nA >= 0) return nA;
        if (nB >= 0) return nB;
        return 0;
    }
    int nCChroma(int mx, int my, int c, int bx, int by, int slice, const int8_t* curNz) const {
        int nA = -1, nB = -1;
        if (bx > 0) nA = curNz[by * 2 + bx - 1];
        else if (avail(mx - 1, my, slice)) nA = info(mx - 1, my).nzc[c][by * 2 + 1];
        if (by > 0) nB = curNz[bx];
        else if (avail(mx, my - 1, slice)) nB = info(mx, my - 1).nzc[c][2 + bx];
        if (nA >= 0 && nB >= 0) return (nA + nB + 1) >> 1;
        if (nA >= 0) return nA;
        if (nB >= 0) return nB;
        return 0;
    }

    // ---- residual coding of one 4x4 block: forward, quantise, dequantise, inverse
    // levels: raster order. Returns the reconstructed residual in res (raster), including dcIn at (0,0) if acOnly.
    void dequantInv(const int* lev, int q, bool acOnly, int dcVal, int* res) const {
        int d[16];
        const int q6 = q / 6, qm = q % 6;
        for (int i = 0; i < 16; ++i) d[i] = lev[i] * kV[qm][posClass(i)] * (1 << q6);
        if (acOnly) d[0] = dcVal;
        std::memcpy(res, d, sizeof d);
        inv4(res);
    }

    // Chroma residual for one MB (both planes): returns cbpChroma; fills levels and reconstruction.
    struct ChromaRes {
        int dc[2][4];          // quantised DC (raster of blocks)
        int ac[2][4][16];      // quantised AC (raster; [0] unused)
        uint8_t rec[2][64];
    };
    int chromaResidual(int mx, int my, const uint8_t* predU, const uint8_t* predV, bool intra, ChromaRes& r) const {
        const int q6 = qpc / 6, qm = qpc % 6, qbits = 15 + q6;
        const int f = intra ? (1 << qbits) / 3 : (1 << qbits) / 6;
        bool anyAc = false, anyDc = false;
        int W4[2][4][16];
        for (int c = 0; c < 2; ++c) {
            const Plane& S = c ? src.v : src.u;
            const uint8_t* pred = c ? predV : predU;
            int dcs[4];
            for (int b = 0; b < 4; ++b) {
                int bx = b & 1, by = b >> 1, diff[16];
                for (int y = 0; y < 4; ++y)
                    for (int x = 0; x < 4; ++x)
                        diff[y * 4 + x] = int(S.row(my * 8 + by * 4 + y)[mx * 8 + bx * 4 + x]) - int(pred[(by * 4 + y) * 8 + bx * 4 + x]);
                fwd4(diff, W4[c][b]);
                dcs[b] = W4[c][b][0];
                r.ac[c][b][0] = 0;
                for (int i = 1; i < 16; ++i) {
                    r.ac[c][b][i] = quant(W4[c][b][i], kMF[qm][posClass(i)], f, qbits);
                    anyAc |= r.ac[c][b][i] != 0;
                }
            }
            // 2x2 Hadamard of the DCs
            int h[4] = {dcs[0] + dcs[1] + dcs[2] + dcs[3], dcs[0] - dcs[1] + dcs[2] - dcs[3], dcs[0] + dcs[1] - dcs[2] - dcs[3], dcs[0] - dcs[1] - dcs[2] + dcs[3]};
            for (int i = 0; i < 4; ++i) {
                r.dc[c][i] = quant(h[i], kMF[qm][0], 2 * f, qbits + 1);
                anyDc |= r.dc[c][i] != 0;
            }
        }
        int cbp = anyAc ? 2 : anyDc ? 1 : 0;
        for (int c = 0; c < 2; ++c) {
            if (cbp < 2) for (int b = 0; b < 4; ++b) std::fill(r.ac[c][b], r.ac[c][b] + 16, 0);
            if (cbp < 1) std::fill(r.dc[c], r.dc[c] + 4, 0);
            // decoder: f = H c H, dcC = ((f * LevelScale) << (qp / 6)) >> 5
            const int* cc = r.dc[c];
            int fh[4] = {cc[0] + cc[1] + cc[2] + cc[3], cc[0] - cc[1] + cc[2] - cc[3], cc[0] + cc[1] - cc[2] - cc[3], cc[0] - cc[1] - cc[2] + cc[3]};
            const int ls = 16 * kV[qm][0];
            const uint8_t* pred = c ? predV : predU;
            for (int b = 0; b < 4; ++b) {
                int dcC = (fh[b] * ls * (1 << q6)) >> 5;
                int res[16];
                dequantInv(r.ac[c][b], qpc, true, dcC, res);
                int bx = b & 1, by = b >> 1;
                for (int y = 0; y < 4; ++y)
                    for (int x = 0; x < 4; ++x) {
                        int o = (by * 4 + y) * 8 + bx * 4 + x;
                        r.rec[c][o] = clip1(pred[o] + res[y * 4 + x]);
                    }
            }
        }
        return cbp;
    }

    void writeChroma(Bits& b, int mx, int my, int slice, int cbpC, const ChromaRes& r, MbInfo& m) const {
        for (int c = 0; c < 2; ++c) for (int i = 0; i < 4; ++i) m.nzc[c][i] = 0;
        if (cbpC == 0) return;
        for (int c = 0; c < 2; ++c) writeBlock(b, r.dc[c], 4, -1);
        if (cbpC < 2) return;
        for (int c = 0; c < 2; ++c)
            for (int blk = 0; blk < 4; ++blk) {
                int scan[15];
                for (int i = 1; i < 16; ++i) scan[i - 1] = r.ac[c][blk][kZigzag[i]];
                int nC = nCChroma(mx, my, c, blk & 1, blk >> 1, slice, m.nzc[c]);
                m.nzc[c][blk] = int8_t(writeBlock(b, scan, 15, nC));
            }
    }

    // Encodes one macroblock. Returns false if it was skipped (P slices).
    void encodeMb(Bits& b, int mx, int my, int slice, bool pFrame, int& skipRun) {
        MbInfo& m = info(mx, my);
        std::memset(m.nz, 0, sizeof m.nz);
        std::memset(m.nzc, 0, sizeof m.nzc);
        m.qp = int8_t(qp);
        m.coded = 0;
        const bool hasL = avail(mx - 1, my, slice), hasT = avail(mx, my - 1, slice), hasTL = avail(mx - 1, my - 1, slice);

        // ---- intra 16x16 candidate
        int bestMode = 2, bestSad = INT32_MAX;
        uint8_t predY[256], tmp[256];
        for (int mode = 0; mode < 4; ++mode) {
            if ((mode == 0 && !hasT) || (mode == 1 && !hasL) || (mode == 3 && !(hasT && hasL && hasTL))) continue;
            predLuma16(mode, mx, my, hasL, hasT, tmp);
            int s = 0;
            for (int y = 0; y < 16; ++y) {
                const uint8_t* a = src.y.row(my * 16 + y) + mx * 16;
                for (int x = 0; x < 16; ++x) s += std::abs(int(a[x]) - int(tmp[y * 16 + x]));
            }
            if (s < bestSad) { bestSad = s; bestMode = mode; std::memcpy(predY, tmp, 256); }
        }

        if (pFrame) {
            int px, py, skx, sky;
            mvPred(mx, my, slice, px, py, skx, sky);
            // skip: prediction with the skip vector leaves nothing to code
            uint8_t iy[256], iu[64], iv[64];
            predInter(mx, my, skx, sky, iy, iu, iv);
            int lev[16][16];
            int cbpL = interLuma(mx, my, iy, lev, nullptr, true);
            if (cbpL == 0) {
                ChromaRes cr;
                if (chromaResidual(mx, my, iu, iv, false, cr) == 0) {
                    ++skipRun;
                    m.intra = false; m.skip = true; m.ref = 0;
                    m.mvx = int16_t(skx); m.mvy = int16_t(sky);
                    store(mx, my, iy, iu, iv);
                    return;
                }
            }
            // motion search (full pel) from the best of a few candidates
            auto cost = [&](int vx, int vy) { return sad16(mx, my, vx, vy) + int(lambda * float(seBits(vx * 4 - px) + seBits(vy * 4 - py))); };
            int bx = 0, by = 0, bc = cost(0, 0);
            auto tryMv = [&](int vx, int vy) {
                vx = std::clamp(vx, -mx * 16 - 32, (mbw - mx) * 16 + 16);
                vy = std::clamp(vy, -my * 16 - 32, (mbh - my) * 16 + 16);
                vx = std::clamp(vx, -256, 255); vy = std::clamp(vy, -128, 127);
                int c = cost(vx, vy);
                if (c < bc) { bc = c; bx = vx; by = vy; return true; }
                return false;
            };
            tryMv(px / 4, py / 4);
            tryMv(skx / 4, sky / 4);
            if (hasL) tryMv(info(mx - 1, my).mvx / 4, info(mx - 1, my).mvy / 4);
            if (hasT) tryMv(info(mx, my - 1).mvx / 4, info(mx, my - 1).mvy / 4);
            for (int step = 8; step >= 1; step /= 2)
                for (int it = 0; it < 8; ++it) {
                    int cx = bx, cy = by;
                    bool moved = false;
                    moved |= tryMv(cx + step, cy);
                    moved |= tryMv(cx - step, cy);
                    moved |= tryMv(cx, cy + step);
                    moved |= tryMv(cx, cy - step);
                    if (!moved) break;
                }
            const int interCost = bc;
            const int intraCost = bestSad + int(lambda * 24);
            if (interCost <= intraCost) {
                const int mvx = bx * 4, mvy = by * 4;
                predInter(mx, my, mvx, mvy, iy, iu, iv);
                int recY[256];
                int cbpLuma = interLuma(mx, my, iy, lev, recY, false);
                ChromaRes cr;
                int cbpC = chromaResidual(mx, my, iu, iv, false, cr);
                if (mvx == skx && mvy == sky && cbpLuma == 0 && cbpC == 0) {
                    ++skipRun;
                    m.intra = false; m.skip = true; m.ref = 0; m.mvx = int16_t(mvx); m.mvy = int16_t(mvy);
                    store(mx, my, iy, iu, iv);
                    return;
                }
                b.ue(uint32_t(skipRun));
                skipRun = 0;
                m.intra = false; m.skip = false; m.ref = 0; m.mvx = int16_t(mvx); m.mvy = int16_t(mvy);
                b.ue(0); // P_L0_16x16
                b.se(mvx - px);
                b.se(mvy - py);
                const int cbp = cbpLuma | (cbpC << 4);
                int code = 0;
                for (int k = 0; k < 48; ++k) if (kGolombToInterCbp[k] == cbp) { code = k; break; }
                b.ue(uint32_t(code));
                if (cbp) b.se(0); // mb_qp_delta
                for (int b8 = 0; b8 < 4; ++b8) {
                    if (!(cbpLuma & (1 << b8))) continue;
                    for (int s = 0; s < 4; ++s) {
                        int blk = b8 * 4 + s, x4, y4;
                        blkXY(blk, x4, y4);
                        int scan[16];
                        for (int i = 0; i < 16; ++i) scan[i] = lev[blk][kZigzag[i]];
                        m.nz[blk] = int8_t(writeBlock(b, scan, 16, nCLuma(mx, my, x4, y4, slice, m.nz)));
                        if (m.nz[blk]) m.coded |= uint16_t(1u << (y4 * 4 + x4));
                    }
                }
                writeChroma(b, mx, my, slice, cbpC, cr, m);
                uint8_t ry[256];
                for (int i = 0; i < 256; ++i) ry[i] = uint8_t(recY[i]);
                store(mx, my, ry, cr.rec[0], cr.rec[1]);
                return;
            }
            b.ue(uint32_t(skipRun));
            skipRun = 0;
        }

        // ---- intra 16x16
        m.intra = true; m.skip = false; m.ref = -1; m.mvx = m.mvy = 0;
        const int q6 = qp / 6, qm = qp % 6, qbits = 15 + q6, f = (1 << qbits) / 3;
        int ac[16][16], dcIn[16];
        bool anyAc = false;
        for (int blk = 0; blk < 16; ++blk) {
            int x4, y4, diff[16], w[16];
            blkXY(blk, x4, y4);
            for (int y = 0; y < 4; ++y)
                for (int x = 0; x < 4; ++x)
                    diff[y * 4 + x] = int(src.y.row(my * 16 + y4 * 4 + y)[mx * 16 + x4 * 4 + x]) - int(predY[(y4 * 4 + y) * 16 + x4 * 4 + x]);
            fwd4(diff, w);
            dcIn[y4 * 4 + x4] = w[0];
            ac[blk][0] = 0;
            for (int i = 1; i < 16; ++i) { ac[blk][i] = quant(w[i], kMF[qm][posClass(i)], f, qbits); anyAc |= ac[blk][i] != 0; }
        }
        int hd[16], dcLev[16];
        hadamard4(dcIn, hd);
        for (int i = 0; i < 16; ++i) dcLev[i] = quant(hd[i] / 2, kMF[qm][0], 2 * f, qbits + 1);
        const int cbpL = anyAc ? 15 : 0;
        if (!anyAc) for (auto& a : ac) std::fill(a, a + 16, 0);
        // chroma
        int bestC = 0, bestCs = INT32_MAX;
        uint8_t pu[64], pv[64], tu[64], tv[64];
        for (int mode = 0; mode < 4; ++mode) {
            if ((mode == 1 && !hasL) || (mode == 2 && !hasT) || (mode == 3 && !(hasT && hasL && hasTL))) continue;
            predChroma(mode, rec.u, mx, my, hasL, hasT, tu);
            predChroma(mode, rec.v, mx, my, hasL, hasT, tv);
            int s = 0;
            for (int y = 0; y < 8; ++y)
                for (int x = 0; x < 8; ++x)
                    s += std::abs(int(src.u.row(my * 8 + y)[mx * 8 + x]) - int(tu[y * 8 + x])) + std::abs(int(src.v.row(my * 8 + y)[mx * 8 + x]) - int(tv[y * 8 + x]));
            if (s < bestCs) { bestCs = s; bestC = mode; std::memcpy(pu, tu, 64); std::memcpy(pv, tv, 64); }
        }
        ChromaRes cr;
        const int cbpC = chromaResidual(mx, my, pu, pv, true, cr);
        b.ue(uint32_t((pFrame ? 5 : 0) + 1 + bestMode + 4 * cbpC + (cbpL ? 12 : 0)));
        b.ue(uint32_t(bestC));
        b.se(0); // mb_qp_delta
        {
            int scan[16];
            for (int i = 0; i < 16; ++i) scan[i] = dcLev[kZigzag[i]];
            int dcCount = writeBlock(b, scan, 16, nCLuma(mx, my, 0, 0, slice, m.nz));
            if (dcCount) m.coded = 0xFFFF; // (deblocking treats intra edges as strong anyway)
        }
        if (cbpL)
            for (int blk = 0; blk < 16; ++blk) {
                int x4, y4, scan[15];
                blkXY(blk, x4, y4);
                for (int i = 1; i < 16; ++i) scan[i - 1] = ac[blk][kZigzag[i]];
                m.nz[blk] = int8_t(writeBlock(b, scan, 15, nCLuma(mx, my, x4, y4, slice, m.nz)));
            }
        writeChroma(b, mx, my, slice, cbpC, cr, m);
        // reconstruction: DC (8.5.10) then each block
        int fdc[16], ry[256];
        hadamard4(dcLev, fdc);
        const int ls = 16 * kV[qm][0];
        for (int blk = 0; blk < 16; ++blk) {
            int x4, y4, res[16];
            blkXY(blk, x4, y4);
            int fv = fdc[y4 * 4 + x4];
            int dcY = qp >= 36 ? fv * ls * (1 << (q6 - 6)) : (fv * ls + (1 << (5 - q6))) >> (6 - q6);
            dequantInv(ac[blk], qp, true, dcY, res);
            for (int y = 0; y < 4; ++y)
                for (int x = 0; x < 4; ++x) {
                    int o = (y4 * 4 + y) * 16 + x4 * 4 + x;
                    ry[o] = clip1(predY[o] + res[y * 4 + x]);
                }
        }
        uint8_t ry8[256];
        for (int i = 0; i < 256; ++i) ry8[i] = uint8_t(ry[i]);
        store(mx, my, ry8, cr.rec[0], cr.rec[1]);
    }

    // Inter luma residual: quantised levels per block (blkIdx), cbp, and (unless probe) the reconstruction.
    int interLuma(int mx, int my, const uint8_t* pred, int lev[16][16], int* recY, bool probe) const {
        const int q6 = qp / 6, qm = qp % 6, qbits = 15 + q6, f = (1 << qbits) / 6;
        int cbp = 0;
        for (int blk = 0; blk < 16; ++blk) {
            int x4, y4, diff[16], w[16];
            blkXY(blk, x4, y4);
            for (int y = 0; y < 4; ++y)
                for (int x = 0; x < 4; ++x)
                    diff[y * 4 + x] = int(src.y.row(my * 16 + y4 * 4 + y)[mx * 16 + x4 * 4 + x]) - int(pred[(y4 * 4 + y) * 16 + x4 * 4 + x]);
            fwd4(diff, w);
            bool nzb = false;
            for (int i = 0; i < 16; ++i) { lev[blk][i] = quant(w[i], kMF[qm][posClass(i)], f, qbits); nzb |= lev[blk][i] != 0; }
            if (nzb) cbp |= 1 << (blk >> 2);
            if (probe && cbp) return cbp;
        }
        // an 8x8 with only a couple of +-1 levels costs more bits than it is worth
        for (int b8 = 0; b8 < 4; ++b8) {
            if (!(cbp & (1 << b8))) continue;
            int score = 0;
            for (int s = 0; s < 4; ++s)
                for (int i = 0; i < 16; ++i) { int a = std::abs(lev[b8 * 4 + s][i]); score += a > 1 ? 100 : a; }
            if (score <= 2) {
                cbp &= ~(1 << b8);
                for (int s = 0; s < 4; ++s) std::fill(lev[b8 * 4 + s], lev[b8 * 4 + s] + 16, 0);
            }
        }
        if (probe) return cbp;
        for (int blk = 0; blk < 16; ++blk) {
            int x4, y4, res[16];
            blkXY(blk, x4, y4);
            if (cbp & (1 << (blk >> 2))) dequantInv(lev[blk], qp, false, 0, res);
            else std::fill(res, res + 16, 0);
            for (int y = 0; y < 4; ++y)
                for (int x = 0; x < 4; ++x) {
                    int o = (y4 * 4 + y) * 16 + x4 * 4 + x;
                    recY[o] = clip1(pred[o] + res[y * 4 + x]);
                }
        }
        return cbp;
    }

    void store(int mx, int my, const uint8_t* y16, const uint8_t* u8, const uint8_t* v8) {
        for (int y = 0; y < 16; ++y) std::memcpy(rec.y.row(my * 16 + y) + mx * 16, y16 + y * 16, 16);
        for (int y = 0; y < 8; ++y) {
            std::memcpy(rec.u.row(my * 8 + y) + mx * 8, u8 + y * 8, 8);
            std::memcpy(rec.v.row(my * 8 + y) + mx * 8, v8 + y * 8, 8);
        }
    }

    // ---- loop filter (8.7), applied to the whole picture after all slices
    void filterEdge(uint8_t* p0p, int step, int across, int bS, int qpAvg, bool chroma, int count) {
        // p0p points at q0 of the first sample row/column; `across` is the offset between samples across the edge
        const int indexA = std::clamp(qpAvg, 0, 51), indexB = std::clamp(qpAvg, 0, 51);
        const int alpha = kAlpha[indexA], beta = kBeta[indexB];
        if (!alpha || !beta) return;
        for (int k = 0; k < count; ++k) {
            uint8_t* q = p0p + k * step;
            int p0 = q[-across], p1 = q[-2 * across], q0 = q[0], q1 = q[across];
            if (!(std::abs(p0 - q0) < alpha && std::abs(p1 - p0) < beta && std::abs(q1 - q0) < beta)) continue;
            if (chroma) {
                if (bS < 4) {
                    int tc = kTc0[indexA][bS - 1] + 1;
                    int delta = std::clamp(((q0 - p0) * 4 + (p1 - q1) + 4) >> 3, -tc, tc);
                    q[-across] = clip1(p0 + delta);
                    q[0] = clip1(q0 - delta);
                } else {
                    q[-across] = uint8_t((2 * p1 + p0 + q1 + 2) >> 2);
                    q[0] = uint8_t((2 * q1 + q0 + p1 + 2) >> 2);
                }
                continue;
            }
            int p2 = q[-3 * across], q2 = q[2 * across];
            int ap = std::abs(p2 - p0), aq = std::abs(q2 - q0);
            if (bS < 4) {
                int tc0 = kTc0[indexA][bS - 1];
                int tc = tc0 + (ap < beta) + (aq < beta);
                int delta = std::clamp(((q0 - p0) * 4 + (p1 - q1) + 4) >> 3, -tc, tc);
                q[-across] = clip1(p0 + delta);
                q[0] = clip1(q0 - delta);
                if (ap < beta) q[-2 * across] = uint8_t(p1 + std::clamp((p2 + ((p0 + q0 + 1) >> 1) - (p1 << 1)) >> 1, -tc0, tc0));
                if (aq < beta) q[across] = uint8_t(q1 + std::clamp((q2 + ((p0 + q0 + 1) >> 1) - (q1 << 1)) >> 1, -tc0, tc0));
            } else {
                int p3 = q[-4 * across], q3 = q[3 * across];
                bool small = std::abs(p0 - q0) < ((alpha >> 2) + 2);
                if (ap < beta && small) {
                    q[-across] = uint8_t((p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3);
                    q[-2 * across] = uint8_t((p2 + p1 + p0 + q0 + 2) >> 2);
                    q[-3 * across] = uint8_t((2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3);
                } else q[-across] = uint8_t((2 * p1 + p0 + q1 + 2) >> 2);
                if (aq < beta && small) {
                    q[0] = uint8_t((p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) >> 3);
                    q[across] = uint8_t((p0 + q0 + q1 + q2 + 2) >> 2);
                    q[2 * across] = uint8_t((2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) >> 3);
                } else q[0] = uint8_t((2 * q1 + q0 + p1 + 2) >> 2);
            }
        }
    }
    // boundary strength between 4x4 block (x4a,y4a) of mb a and block (x4b,y4b) of mb b
    int strength(const MbInfo& a, int ba, const MbInfo& b, int bb, bool mbEdge) const {
        if (a.intra || b.intra) return mbEdge ? 4 : 3;
        if ((a.coded >> ba & 1) || (b.coded >> bb & 1)) return 2;
        if (std::abs(a.mvx - b.mvx) >= 4 || std::abs(a.mvy - b.mvy) >= 4) return 1;
        return 0;
    }
    void loopFilter() {
        for (int my = 0; my < mbh; ++my)
            for (int mx = 0; mx < mbw; ++mx) {
                const MbInfo& cur = info(mx, my);
                // vertical edges (left to right), then horizontal (top to bottom)
                for (int dir = 0; dir < 2; ++dir)
                    for (int e = 0; e < 4; ++e) {
                        const bool mbEdge = e == 0;
                        int nmx = mx - (dir == 0 && mbEdge), nmy = my - (dir == 1 && mbEdge);
                        if (mbEdge && (nmx < 0 || nmy < 0)) continue; // picture edge (one slice per filtering: all edges filtered)
                        const MbInfo& nb = info(nmx, nmy);
                        for (int k = 0; k < 4; ++k) {
                            // block on the q side (current) and p side
                            int qx = dir == 0 ? e : k, qy = dir == 0 ? k : e;
                            int px = dir == 0 ? (mbEdge ? 3 : e - 1) : k, py = dir == 0 ? k : (mbEdge ? 3 : e - 1);
                            int bS = strength(nb, py * 4 + px, cur, qy * 4 + qx, mbEdge);
                            if (!bS) continue;
                            int qpAvg = (cur.qp + nb.qp + 1) >> 1;
                            uint8_t* q0 = dir == 0 ? rec.y.row(my * 16 + qy * 4) + mx * 16 + qx * 4 : rec.y.row(my * 16 + qy * 4) + mx * 16 + qx * 4;
                            if (dir == 0) filterEdge(q0, rec.y.w, 1, bS, qpAvg, false, 4);
                            else filterEdge(q0, 1, rec.y.w, bS, qpAvg, false, 4);
                            // chroma: edges 0 and 2 of luma map to chroma edges 0 and 1 (each luma 4-sample segment -> 2 chroma)
                            if (e == 0 || e == 2) {
                                int ce = e / 2;
                                int qpc2 = (kChromaQp[cur.qp] + kChromaQp[nb.qp] + 1) >> 1;
                                for (int c = 0; c < 2; ++c) {
                                    Plane& P = c ? rec.v : rec.u;
                                    int cx = dir == 0 ? mx * 8 + ce * 4 : mx * 8 + k * 2, cy = dir == 0 ? my * 8 + k * 2 : my * 8 + ce * 4;
                                    uint8_t* cq = P.row(cy) + cx;
                                    if (dir == 0) filterEdge(cq, P.w, 1, bS, qpc2, true, 2);
                                    else filterEdge(cq, 1, P.w, bS, qpc2, true, 2);
                                }
                            }
                        }
                    }
            }
    }

    // Encodes one picture: returns its NAL units.
    std::vector<std::vector<uint8_t>> encodeFrame(const uint8_t* rgba, int stride) {
        loadSource(rgba, stride);
        const bool idr = coded % keyint == 0;
        if (idr) frameNum = 0;
        std::vector<std::vector<uint8_t>> nals(static_cast<size_t>(nslices));
        const int rowsPer = (mbh + nslices - 1) / nslices;
        // slice ids first: availability only ever compares them, and they don't change while encoding
        for (int my = 0; my < mbh; ++my)
            for (int mx = 0; mx < mbw; ++mx) info(mx, my).slice = my / rowsPer;
        auto slice = [&](int s) {
            const int r0 = s * rowsPer, r1 = std::min(mbh, r0 + rowsPer);
            if (r0 >= r1) return;
            Bits b;
            b.ue(uint32_t(r0 * mbw));      // first_mb_in_slice
            b.ue(idr ? 7 : 5);             // slice_type: all I / all P
            b.ue(0);                       // pps id
            b.u(16, uint32_t(frameNum & 0xFFFF));
            if (idr) b.ue(uint32_t(idrId & 0xFFFF));
            if (!idr) { b.u(1, 0); b.u(1, 0); } // num_ref_idx_active_override_flag, ref_pic_list_modification_flag_l0
            if (idr) { b.u(1, 0); b.u(1, 0); }  // no_output_of_prior_pics_flag, long_term_reference_flag
            else b.u(1, 0);                     // adaptive_ref_pic_marking_mode_flag
            b.se(0);                       // slice_qp_delta
            b.ue(deblock ? 0 : 1);         // disable_deblocking_filter_idc
            if (deblock) { b.se(0); b.se(0); } // slice_alpha_c0_offset_div2, slice_beta_offset_div2
            int skipRun = 0;
            for (int my = r0; my < r1; ++my)
                for (int mx = 0; mx < mbw; ++mx) encodeMb(b, mx, my, s, !idr, skipRun);
            if (skipRun) b.ue(uint32_t(skipRun));
            b.trailing();
            nals[size_t(s)] = nal(3, idr ? 5 : 1, b.buf);
        };
        parallelFor(nslices, [&](int lo, int hi) { for (int s = lo; s < hi; ++s) slice(s); });
        if (deblock) loopFilter();
        std::swap(rec, ref);
        if (idr) ++idrId;
        frameNum = (frameNum + 1) & 0xFFFF;
        ++coded;
        std::vector<std::vector<uint8_t>> out;
        for (auto& n : nals) if (!n.empty()) out.push_back(std::move(n));
        return out;
    }
};

// ------------------------------------------------------------------ MP4 / MOV

struct Box {
    std::vector<uint8_t> d;
    size_t open(const char* type) { size_t at = d.size(); u32(0); for (int i = 0; i < 4; ++i) d.push_back(uint8_t(type[i])); return at; }
    void close(size_t at) { uint32_t s = uint32_t(d.size() - at); d[at] = uint8_t(s >> 24); d[at + 1] = uint8_t(s >> 16); d[at + 2] = uint8_t(s >> 8); d[at + 3] = uint8_t(s); }
    void u8(uint32_t v) { d.push_back(uint8_t(v)); }
    void u16(uint32_t v) { u8(v >> 8); u8(v); }
    void u32(uint32_t v) { u16(v >> 16); u16(v & 0xFFFF); }
    void u64(uint64_t v) { u32(uint32_t(v >> 32)); u32(uint32_t(v)); }
    void str(const char* s, size_t n) { for (size_t i = 0; i < n; ++i) u8(uint8_t(s[i])); }
    void zeros(size_t n) { d.insert(d.end(), n, 0); }
    void bytes(const std::vector<uint8_t>& b) { d.insert(d.end(), b.begin(), b.end()); }
};

enum class Container { Mp4, Mov, AnnexB };

} // namespace

struct MovieWriter::Impl {
    Encoder enc;
    Container kind = Container::Mp4;
    std::FILE* f = nullptr;
    std::string path;
    int srcW = 0, srcH = 0;
    uint64_t mdatStart = 0, written = 0;
    std::vector<uint64_t> offsets;
    std::vector<uint32_t> sizes;
    std::vector<uint32_t> sync;
    std::vector<uint8_t> reconOut;
    std::vector<uint8_t> scratch;
    bool failed = false;

    bool put(const void* p, size_t n) {
        if (std::fwrite(p, 1, n, f) != n) { failed = true; return false; }
        written += n;
        return true;
    }
};

MovieWriter::MovieWriter() : d_(new Impl) {}
MovieWriter::~MovieWriter() {
    if (d_->f) {
        std::fclose(d_->f);
        std::remove(d_->path.c_str()); // not finished: don't leave a broken file
    }
}

bool MovieWriter::open(const std::string& path, int width, int height, const MovieOptions& opt, std::string& error) {
    if (d_->f) { error = "Movie already open"; return false; }
    std::string ext = path.size() >= 4 ? path.substr(path.rfind('.') == std::string::npos ? path.size() : path.rfind('.')) : "";
    for (auto& c : ext) c = char(std::tolower((unsigned char)c));
    if (ext == ".mp4" || ext == ".m4v") d_->kind = Container::Mp4;
    else if (ext == ".mov" || ext == ".qt") d_->kind = Container::Mov;
    else if (ext == ".h264" || ext == ".264") d_->kind = Container::AnnexB;
    else { error = "Movie files must end in .mp4 or .mov: " + path; return false; }
    const int w = width & ~1, h = height & ~1;
    if (w < 2 || h < 2) { error = "The frames are too small for a movie"; return false; }
    if (w > 8192 || h > 8192) { error = "The frames are too large for H.264 (at most 8192 x 8192)"; return false; }
    d_->srcW = width;
    d_->srcH = height;
    d_->enc.init(w, h, opt);
    d_->path = path;
    d_->f = std::fopen(path.c_str(), "wb");
    if (!d_->f) { error = "Could not write " + path; return false; }
    if (d_->kind == Container::AnnexB) return true;
    Box b;
    size_t ftyp = b.open("ftyp");
    if (d_->kind == Container::Mp4) { b.str("isom", 4); b.u32(512); b.str("isom", 4); b.str("iso2", 4); b.str("avc1", 4); b.str("mp41", 4); }
    else { b.str("qt  ", 4); b.u32(0x20050300); b.str("qt  ", 4); }
    b.close(ftyp);
    b.u32(8); b.str("wide", 4); // becomes the 64-bit mdat size if the video passes 4 GB
    d_->mdatStart = b.d.size();
    b.u32(0); b.str("mdat", 4);
    return d_->put(b.d.data(), b.d.size()) || (error = "Could not write " + path, false);
}

int MovieWriter::frames() const { return int(d_->sizes.size()); }
int MovieWriter::width() const { return d_->enc.W; }
int MovieWriter::height() const { return d_->enc.H; }
size_t MovieWriter::bytes() const { return size_t(d_->written); }
const std::vector<uint8_t>& MovieWriter::lastReconstruction() const { return d_->reconOut; }

bool MovieWriter::addFrame(const uint8_t* rgba, std::string& error) {
    if (!d_->f) { error = "Movie not open"; return false; }
    Impl& m = *d_;
    const bool key = m.enc.coded % m.enc.keyint == 0;
    auto nals = m.enc.encodeFrame(rgba, m.srcW);
    uint64_t at = m.written;
    uint32_t size = 0;
    auto emit = [&](const std::vector<uint8_t>& n) {
        if (m.kind == Container::AnnexB) {
            static const uint8_t sc[4] = {0, 0, 0, 1};
            m.put(sc, 4);
            size += 4;
        } else {
            uint8_t len[4] = {uint8_t(n.size() >> 24), uint8_t(n.size() >> 16), uint8_t(n.size() >> 8), uint8_t(n.size())};
            m.put(len, 4);
            size += 4;
        }
        m.put(n.data(), n.size());
        size += uint32_t(n.size());
    };
    if (m.kind == Container::AnnexB && key) { emit(m.enc.sps); emit(m.enc.pps); }
    for (auto& n : nals) emit(n);
    if (m.failed) { error = "Could not write " + m.path + " (disk full?)"; return false; }
    m.offsets.push_back(at);
    m.sizes.push_back(size);
    if (key) m.sync.push_back(uint32_t(m.sizes.size()));
    // the decoder's picture, cropped
    const Picture& p = m.enc.ref;
    const int W = m.enc.W, H = m.enc.H;
    m.reconOut.resize(size_t(W) * H * 3 / 2);
    uint8_t* o = m.reconOut.data();
    for (int y = 0; y < H; ++y, o += W) std::memcpy(o, p.y.row(y), size_t(W));
    for (int y = 0; y < H / 2; ++y, o += W / 2) std::memcpy(o, p.u.row(y), size_t(W / 2));
    for (int y = 0; y < H / 2; ++y, o += W / 2) std::memcpy(o, p.v.row(y), size_t(W / 2));
    return true;
}

bool MovieWriter::finish(std::string& error) {
    Impl& m = *d_;
    if (!m.f) { error = "Movie not open"; return false; }
    if (m.sizes.empty()) { error = "No frames to write"; return false; }
    if (m.kind == Container::AnnexB) {
        bool ok = std::fclose(m.f) == 0 && !m.failed;
        m.f = nullptr;
        if (!ok) error = "Could not write " + m.path;
        return ok;
    }
    // mdat size
    const uint64_t mdatSize = m.written - m.mdatStart;
    if (mdatSize > 0xFFFFFFFFull) {
        // turn "wide" + "mdat" into one 64-bit mdat header
        std::fseek(m.f, long(m.mdatStart - 8), SEEK_SET);
        uint8_t hdr[16] = {0, 0, 0, 1, 'm', 'd', 'a', 't'};
        uint64_t s = mdatSize + 8;
        for (int i = 0; i < 8; ++i) hdr[8 + i] = uint8_t(s >> (56 - 8 * i));
        std::fwrite(hdr, 1, 16, m.f);
    } else {
        std::fseek(m.f, long(m.mdatStart), SEEK_SET);
        uint8_t hdr[4] = {uint8_t(mdatSize >> 24), uint8_t(mdatSize >> 16), uint8_t(mdatSize >> 8), uint8_t(mdatSize)};
        std::fwrite(hdr, 1, 4, m.f);
    }
    std::fseek(m.f, 0, SEEK_END);

    const bool mov = m.kind == Container::Mov;
    const Encoder& e = m.enc;
    // time: timescale = fps * 1000 (rounded), 1000 ticks per frame
    const uint32_t timescale = uint32_t(std::max(1L, std::lround(double(e.fps) * 1000.0))), delta = 1000;
    const uint64_t duration = uint64_t(m.sizes.size()) * delta;
    const uint32_t n = uint32_t(m.sizes.size());
    Box b;
    size_t moov = b.open("moov");
    {
        size_t mvhd = b.open("mvhd");
        b.u32(0); b.u32(0); b.u32(0); b.u32(timescale); b.u32(uint32_t(duration));
        b.u32(0x00010000); b.u16(0x0100); b.zeros(10);
        const uint32_t matrix[9] = {0x00010000, 0, 0, 0, 0x00010000, 0, 0, 0, 0x40000000};
        for (uint32_t v : matrix) b.u32(v);
        b.zeros(24);
        b.u32(2); // next track id
        b.close(mvhd);
    }
    size_t trak = b.open("trak");
    {
        size_t tkhd = b.open("tkhd");
        b.u32(mov ? 0x0000000F : 0x00000003); b.u32(0); b.u32(0); b.u32(1); b.u32(0); b.u32(uint32_t(duration));
        b.zeros(8); b.u16(0); b.u16(0); b.u16(0); b.u16(0);
        const uint32_t matrix[9] = {0x00010000, 0, 0, 0, 0x00010000, 0, 0, 0, 0x40000000};
        for (uint32_t v : matrix) b.u32(v);
        b.u32(uint32_t(e.W) << 16); b.u32(uint32_t(e.H) << 16);
        b.close(tkhd);
    }
    size_t mdia = b.open("mdia");
    {
        size_t mdhd = b.open("mdhd");
        b.u32(0); b.u32(0); b.u32(0); b.u32(timescale); b.u32(uint32_t(duration));
        b.u16(mov ? 0x7FFF : 0x55C4); b.u16(0); // language: 'und' (MOV: unspecified)
        b.close(mdhd);
        size_t hdlr = b.open("hdlr");
        b.u32(0);
        if (mov) b.str("mhlr", 4); else b.u32(0);
        b.str("vide", 4); b.zeros(12);
        if (mov) { const char nm[] = "VideoHandler"; b.u8(uint32_t(sizeof nm - 1)); b.str(nm, sizeof nm - 1); }
        else { const char nm[] = "VideoHandler"; b.str(nm, sizeof nm); }
        b.close(hdlr);
    }
    size_t minf = b.open("minf");
    {
        size_t vmhd = b.open("vmhd");
        b.u32(1); b.zeros(8);
        b.close(vmhd);
        if (mov) { // data handler reference
            size_t h2 = b.open("hdlr");
            b.u32(0); b.str("dhlr", 4); b.str("alis", 4); b.zeros(12);
            const char nm[] = "DataHandler";
            b.u8(uint32_t(sizeof nm - 1)); b.str(nm, sizeof nm - 1);
            b.close(h2);
        }
        size_t dinf = b.open("dinf");
        size_t dref = b.open("dref");
        b.u32(0); b.u32(1);
        size_t url = b.open(mov ? "alis" : "url ");
        b.u32(1);
        b.close(url);
        b.close(dref);
        b.close(dinf);
    }
    size_t stbl = b.open("stbl");
    {
        size_t stsd = b.open("stsd");
        b.u32(0); b.u32(1);
        size_t avc1 = b.open("avc1");
        b.zeros(6); b.u16(1);
        b.u16(0); b.u16(0);
        if (mov) { b.str("appl", 4); b.u32(0); b.u32(512); } else b.zeros(12);
        b.u16(uint32_t(e.W)); b.u16(uint32_t(e.H));
        b.u32(0x00480000); b.u32(0x00480000); b.u32(0); b.u16(1);
        const char comp[] = "HD H.264";
        b.u8(uint32_t(sizeof comp - 1)); b.str(comp, sizeof comp - 1); b.zeros(31 - (sizeof comp - 1));
        b.u16(0x0018); b.u16(0xFFFF);
        size_t avcC = b.open("avcC");
        b.u8(1); b.u8(e.sps[1]); b.u8(e.sps[2]); b.u8(e.sps[3]); b.u8(0xFF); b.u8(0xE1);
        b.u16(uint32_t(e.sps.size())); b.bytes(e.sps);
        b.u8(1); b.u16(uint32_t(e.pps.size())); b.bytes(e.pps);
        b.close(avcC);
        size_t colr = b.open("colr");
        if (mov) { b.str("nclc", 4); b.u16(1); b.u16(1); b.u16(1); }
        else { b.str("nclx", 4); b.u16(1); b.u16(1); b.u16(1); b.u8(0); }
        b.close(colr);
        size_t pasp = b.open("pasp");
        b.u32(1); b.u32(1);
        b.close(pasp);
        b.close(avc1);
        b.close(stsd);

        size_t stts = b.open("stts");
        b.u32(0); b.u32(1); b.u32(n); b.u32(delta);
        b.close(stts);
        if (m.sync.size() < m.sizes.size()) {
            size_t stss = b.open("stss");
            b.u32(0); b.u32(uint32_t(m.sync.size()));
            for (uint32_t s : m.sync) b.u32(s);
            b.close(stss);
        }
        size_t stsc = b.open("stsc");
        b.u32(0); b.u32(1); b.u32(1); b.u32(1); b.u32(1);
        b.close(stsc);
        size_t stsz = b.open("stsz");
        b.u32(0); b.u32(0); b.u32(n);
        for (uint32_t s : m.sizes) b.u32(s);
        b.close(stsz);
        const bool big = m.offsets.back() > 0xFFFFFFFFull;
        size_t stco = b.open(big ? "co64" : "stco");
        b.u32(0); b.u32(n);
        for (uint64_t o : m.offsets) { if (big) b.u64(o); else b.u32(uint32_t(o)); }
        b.close(stco);
    }
    b.close(stbl);
    b.close(minf);
    b.close(mdia);
    b.close(trak);
    b.close(moov);
    m.put(b.d.data(), b.d.size());
    bool ok = !m.failed && std::fclose(m.f) == 0;
    m.f = nullptr;
    if (!ok) error = "Could not write " + m.path + " (disk full?)";
    return ok;
}

bool writeMovie(const SequenceInfo& seq, const std::string& path, const MovieOptions& opt,
                const std::function<bool(int, int)>& progress, std::string& error) {
    if (seq.size() == 0) { error = "No frames"; return false; }
    MovieWriter mw;
    int w = 0, h = 0;
    const int n = int(seq.size());
    for (int i = 0; i < n; ++i) {
        Image img;
        std::string err;
        if (!readImage(seq.files[size_t(i)], img, false, &err)) { error = "Frame " + std::to_string(seq.frames[size_t(i)]) + ": " + err; return false; }
        if (i == 0) {
            w = img.w; h = img.h;
            if (!mw.open(path, w, h, opt, error)) return false;
        } else if (img.w != w || img.h != h) {
            error = "Frame " + std::to_string(seq.frames[size_t(i)]) + " is " + std::to_string(img.w) + " x " + std::to_string(img.h) +
                    ", the first frame is " + std::to_string(w) + " x " + std::to_string(h);
            return false;
        }
        SequenceCache::Frame fr;
        SequenceCache::toDisplay(img, seq.linear, 0, fr);
        if (!mw.addFrame(fr.rgba.data(), error)) return false;
        if (progress && !progress(i + 1, n)) { error = "Cancelled"; return false; }
    }
    return mw.finish(error);
}

} // namespace hd

#include "Image.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <mutex>

namespace hd {

// ================================================================== checksums

namespace {

uint32_t crc32(const uint8_t* d, size_t n, uint32_t crc = 0) {
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
        init = true;
    }
    crc = ~crc;
    for (size_t i = 0; i < n; ++i) crc = table[(crc ^ d[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

uint32_t adler32(const uint8_t* d, size_t n) {
    uint32_t a = 1, b = 0;
    while (n) {
        size_t k = std::min<size_t>(n, 5552);
        n -= k;
        while (k--) { a += *d++; b += a; }
        a %= 65521; b %= 65521;
    }
    return (b << 16) | a;
}

// ================================================================== inflate

struct Huff { int count[16] = {0}; std::vector<int> symbol; };

void buildHuff(Huff& h, const int* lengths, int n) {
    std::fill(std::begin(h.count), std::end(h.count), 0);
    for (int i = 0; i < n; ++i) h.count[lengths[i]]++;
    h.count[0] = 0;
    int offs[16] = {0};
    for (int len = 1; len < 15; ++len) offs[len + 1] = offs[len] + h.count[len];
    h.symbol.assign(size_t(n), 0);
    for (int i = 0; i < n; ++i) if (lengths[i]) h.symbol[size_t(offs[lengths[i]]++)] = i;
}

struct BitIn {
    const uint8_t* d; size_t n, pos = 0;
    uint32_t buf = 0; int cnt = 0;
    bool err = false;
    int bits(int need) {
        uint32_t v = buf;
        while (cnt < need) {
            if (pos >= n) { err = true; return 0; }
            v |= uint32_t(d[pos++]) << cnt;
            cnt += 8;
        }
        buf = v >> need;
        cnt -= need;
        return int(v & ((1u << need) - 1));
    }
    int decode(const Huff& h) {
        int code = 0, first = 0, index = 0;
        for (int len = 1; len < 16; ++len) {
            code |= bits(1);
            if (err) return -1;
            int count = h.count[len];
            if (code - count < first) return h.symbol[size_t(index + (code - first))];
            index += count; first += count; first <<= 1; code <<= 1;
        }
        return -1;
    }
};

const int kLBase[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
const int kLExt[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
const int kDBase[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
const int kDExt[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

bool inflateRaw(const uint8_t* d, size_t n, std::vector<uint8_t>& out) {
    BitIn in{d, n};
    int last;
    do {
        last = in.bits(1);
        int type = in.bits(2);
        if (in.err) return false;
        if (type == 0) {
            in.buf = 0; in.cnt = 0; // byte align
            if (in.pos + 4 > n) return false;
            unsigned len = d[in.pos] | (d[in.pos + 1] << 8), nlen = d[in.pos + 2] | (d[in.pos + 3] << 8);
            in.pos += 4;
            if (len != (~nlen & 0xFFFF) || in.pos + len > n) return false;
            out.insert(out.end(), d + in.pos, d + in.pos + len);
            in.pos += len;
            continue;
        }
        Huff lit, dist;
        if (type == 1) {
            int l[288];
            for (int i = 0; i < 144; ++i) l[i] = 8;
            for (int i = 144; i < 256; ++i) l[i] = 9;
            for (int i = 256; i < 280; ++i) l[i] = 7;
            for (int i = 280; i < 288; ++i) l[i] = 8;
            buildHuff(lit, l, 288);
            int dl[30];
            for (int& x : dl) x = 5;
            buildHuff(dist, dl, 30);
        } else if (type == 2) {
            int nlen = in.bits(5) + 257, ndist = in.bits(5) + 1, ncode = in.bits(4) + 4;
            static const int order[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
            int lengths[320] = {0};
            for (int i = 0; i < ncode; ++i) lengths[order[i]] = in.bits(3);
            Huff lencode;
            buildHuff(lencode, lengths, 19);
            int idx = 0;
            while (idx < nlen + ndist) {
                int sym = in.decode(lencode);
                if (sym < 0 || in.err) return false;
                if (sym < 16) lengths[idx++] = sym;
                else {
                    int len = 0, rep;
                    if (sym == 16) { if (idx == 0) return false; len = lengths[idx - 1]; rep = 3 + in.bits(2); }
                    else if (sym == 17) rep = 3 + in.bits(3);
                    else rep = 11 + in.bits(7);
                    if (idx + rep > nlen + ndist) return false;
                    while (rep--) lengths[idx++] = len;
                }
            }
            buildHuff(lit, lengths, nlen);
            buildHuff(dist, lengths + nlen, ndist);
        } else return false;
        for (;;) {
            int sym = in.decode(lit);
            if (sym < 0 || in.err) return false;
            if (sym < 256) out.push_back(uint8_t(sym));
            else if (sym == 256) break;
            else {
                sym -= 257;
                if (sym >= 29) return false;
                int len = kLBase[sym] + in.bits(kLExt[sym]);
                int ds = in.decode(dist);
                if (ds < 0 || ds >= 30) return false;
                size_t back = size_t(kDBase[ds] + in.bits(kDExt[ds]));
                if (in.err || back > out.size()) return false;
                size_t from = out.size() - back;
                for (int i = 0; i < len; ++i) out.push_back(out[from + size_t(i)]);
            }
        }
    } while (!last);
    return true;
}

// ================================================================== deflate (LZ77 + fixed Huffman, stored fallback)

struct BitOut {
    std::vector<uint8_t>& o;
    uint32_t buf = 0; int cnt = 0;
    void put(uint32_t v, int n) {
        buf |= v << cnt;
        cnt += n;
        while (cnt >= 8) { o.push_back(uint8_t(buf)); buf >>= 8; cnt -= 8; }
    }
    void putRev(uint32_t code, int n) { // Huffman codes go most significant bit first
        uint32_t r = 0;
        for (int i = 0; i < n; ++i) r |= ((code >> i) & 1u) << (n - 1 - i);
        put(r, n);
    }
    void flush() { if (cnt) { o.push_back(uint8_t(buf)); buf = 0; cnt = 0; } }
};

void putLit(BitOut& b, int v) {
    if (v < 144) b.putRev(uint32_t(0x30 + v), 8);
    else if (v < 256) b.putRev(uint32_t(0x190 + v - 144), 9);
    else if (v < 280) b.putRev(uint32_t(v - 256), 7);
    else b.putRev(uint32_t(0xC0 + v - 280), 8);
}

std::vector<uint8_t> deflateRaw(const uint8_t* d, size_t n, int level) {
    std::vector<uint8_t> out;
    if (level > 0 && n > 0) {
        BitOut b{out};
        b.put(1, 1); b.put(1, 2); // final block, fixed Huffman
        const int HB = 15, HS = 1 << HB;
        std::vector<int> head(HS, -1), prev(n, -1);
        const int maxChain = level >= 9 ? 256 : level >= 6 ? 32 : 8;
        auto hash = [&](size_t i) { return ((uint32_t(d[i]) << 10) ^ (uint32_t(d[i + 1]) << 5) ^ d[i + 2]) & (HS - 1); };
        size_t i = 0;
        while (i < n) {
            int bestLen = 0; size_t bestDist = 0;
            if (i + 2 < n) {
                uint32_t h = hash(i);
                int cand = head[h];
                int chain = maxChain;
                const size_t maxLen = std::min<size_t>(258, n - i);
                while (cand >= 0 && chain-- > 0 && i - size_t(cand) <= 32768) {
                    const uint8_t* a = d + cand;
                    const uint8_t* c = d + i;
                    if (a[bestLen] == c[bestLen]) {
                        size_t l = 0;
                        while (l < maxLen && a[l] == c[l]) ++l;
                        if (int(l) > bestLen) { bestLen = int(l); bestDist = i - size_t(cand); if (l == maxLen) break; }
                    }
                    cand = prev[size_t(cand)];
                }
                prev[i] = head[h];
                head[h] = int(i);
            }
            if (bestLen >= 3) {
                int li = 0;
                while (li < 28 && kLBase[li + 1] <= bestLen) ++li;
                putLit(b, 257 + li);
                b.put(uint32_t(bestLen - kLBase[li]), kLExt[li]);
                int di = 0;
                while (di < 29 && size_t(kDBase[di + 1]) <= bestDist) ++di;
                b.putRev(uint32_t(di), 5);
                b.put(uint32_t(bestDist - size_t(kDBase[di])), kDExt[di]);
                // index the skipped positions too
                for (size_t k = i + 1; k < i + size_t(bestLen) && k + 2 < n; ++k) {
                    uint32_t h = hash(k);
                    prev[k] = head[h];
                    head[h] = int(k);
                }
                i += size_t(bestLen);
            } else {
                putLit(b, d[i]);
                ++i;
            }
        }
        putLit(b, 256);
        b.flush();
        if (out.size() < n + n / 100 + 16) return out;
        out.clear();
    }
    // stored blocks
    size_t pos = 0;
    do {
        size_t len = std::min<size_t>(65535, n - pos);
        bool last = pos + len >= n;
        out.push_back(last ? 1 : 0);
        out.push_back(uint8_t(len)); out.push_back(uint8_t(len >> 8));
        out.push_back(uint8_t(~len)); out.push_back(uint8_t((~len) >> 8));
        out.insert(out.end(), d + pos, d + pos + len);
        pos += len;
    } while (pos < n);
    return out;
}

bool readFile(const std::string& path, std::vector<uint8_t>& data, std::string* err) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { if (err) *err = "Cannot open " + path; return false; }
    data.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
}

bool writeFile(const std::string& path, const std::vector<uint8_t>& data, std::string* err) {
    std::ofstream f(path, std::ios::binary);
    if (!f) { if (err) *err = "Cannot write " + path; return false; }
    f.write(reinterpret_cast<const char*>(data.data()), std::streamsize(data.size()));
    if (!f) { if (err) *err = "Write failed: " + path; return false; }
    return true;
}

inline float srgbToLin(float c) { return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f); }

std::string lower(std::string s) { for (auto& ch : s) ch = char(std::tolower((unsigned char)ch)); return s; }
bool endsWith(const std::string& s, const char* suf) { size_t n = std::strlen(suf); return s.size() >= n && lower(s.substr(s.size() - n)) == suf; }

} // namespace

std::vector<uint8_t> zlibCompress(const uint8_t* data, size_t n, int level) {
    std::vector<uint8_t> out{0x78, 0x9C};
    auto body = deflateRaw(data, n, level);
    out.insert(out.end(), body.begin(), body.end());
    uint32_t a = adler32(data, n);
    out.push_back(uint8_t(a >> 24)); out.push_back(uint8_t(a >> 16)); out.push_back(uint8_t(a >> 8)); out.push_back(uint8_t(a));
    return out;
}

bool zlibDecompress(const uint8_t* data, size_t n, std::vector<uint8_t>& out, size_t expected) {
    out.clear();
    if (expected) out.reserve(expected);
    if (n < 6 || (data[0] & 0x0F) != 8 || ((data[0] << 8) | data[1]) % 31 != 0) return false;
    if (!inflateRaw(data + 2, n - 6, out)) return false;
    uint32_t a = (uint32_t(data[n - 4]) << 24) | (uint32_t(data[n - 3]) << 16) | (uint32_t(data[n - 2]) << 8) | data[n - 1];
    return a == adler32(out.data(), out.size());
}

// ================================================================== half floats

float halfToFloat(uint16_t h) {
    uint32_t s = uint32_t(h >> 15) << 31, e = (h >> 10) & 0x1F, m = h & 0x3FF, f;
    if (e == 0) {
        if (m == 0) f = s;
        else { // subnormal
            e = 127 - 15 + 1;
            while (!(m & 0x400)) { m <<= 1; --e; }
            m &= 0x3FF;
            f = s | (e << 23) | (m << 13);
        }
    } else if (e == 31) f = s | 0x7F800000u | (m << 13);
    else f = s | ((e + 127 - 15) << 23) | (m << 13);
    float r;
    std::memcpy(&r, &f, 4);
    return r;
}

uint16_t floatToHalf(float x) {
    uint32_t f;
    std::memcpy(&f, &x, 4);
    uint32_t s = (f >> 16) & 0x8000;
    int e = int((f >> 23) & 0xFF) - 127 + 15;
    uint32_t m = f & 0x7FFFFF;
    if (((f >> 23) & 0xFF) == 0xFF) return uint16_t(s | 0x7C00 | (m ? 0x200 : 0));
    if (e >= 31) return uint16_t(s | 0x7C00);
    if (e <= 0) {
        if (e < -10) return uint16_t(s);
        m |= 0x800000;
        uint32_t shift = uint32_t(14 - e);
        uint32_t hm = m >> shift;
        if ((m >> (shift - 1)) & 1) hm += 1; // round
        return uint16_t(s | hm);
    }
    uint32_t hm = m >> 13;
    uint16_t r = uint16_t(s | (uint32_t(e) << 10) | hm);
    if (m & 0x1000) ++r; // round to nearest (carry into the exponent is correct)
    return r;
}

// ================================================================== PNG

bool writePng8(const std::string& path, const std::vector<uint8_t>& data, int w, int h, int channels, std::string* err) {
    if (w <= 0 || h <= 0 || (channels != 3 && channels != 4) || data.size() < size_t(w) * size_t(h) * size_t(channels)) {
        if (err) *err = "Bad image";
        return false;
    }
    const size_t stride = size_t(w) * size_t(channels);
    std::vector<uint8_t> raw;
    raw.reserve((stride + 1) * size_t(h));
    std::vector<uint8_t> best(stride), cur(stride);
    for (int y = 0; y < h; ++y) {
        const uint8_t* row = data.data() + size_t(y) * stride;
        const uint8_t* up = y ? data.data() + size_t(y - 1) * stride : nullptr;
        long bestScore = -1;
        int bestF = 0;
        for (int f = 0; f < 5; ++f) {
            long score = 0;
            for (size_t i = 0; i < stride; ++i) {
                int a = i >= size_t(channels) ? row[i - size_t(channels)] : 0;
                int b = up ? up[i] : 0;
                int c = (up && i >= size_t(channels)) ? up[i - size_t(channels)] : 0;
                int pred = 0;
                if (f == 1) pred = a;
                else if (f == 2) pred = b;
                else if (f == 3) pred = (a + b) / 2;
                else if (f == 4) {
                    int p = a + b - c, pa = std::abs(p - a), pb = std::abs(p - b), pc = std::abs(p - c);
                    pred = (pa <= pb && pa <= pc) ? a : (pb <= pc ? b : c);
                }
                uint8_t v = uint8_t(row[i] - pred);
                cur[i] = v;
                score += v < 128 ? v : 256 - v;
            }
            if (bestScore < 0 || score < bestScore) { bestScore = score; bestF = f; best.swap(cur); }
        }
        raw.push_back(uint8_t(bestF));
        raw.insert(raw.end(), best.begin(), best.end());
    }
    std::vector<uint8_t> out = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    auto be32 = [&](std::vector<uint8_t>& v, uint32_t x) { v.push_back(uint8_t(x >> 24)); v.push_back(uint8_t(x >> 16)); v.push_back(uint8_t(x >> 8)); v.push_back(uint8_t(x)); };
    auto chunk = [&](const char* type, const std::vector<uint8_t>& payload) {
        be32(out, uint32_t(payload.size()));
        std::vector<uint8_t> td(type, type + 4);
        td.insert(td.end(), payload.begin(), payload.end());
        out.insert(out.end(), td.begin(), td.end());
        be32(out, crc32(td.data(), td.size()));
    };
    std::vector<uint8_t> ihdr;
    be32(ihdr, uint32_t(w)); be32(ihdr, uint32_t(h));
    ihdr.push_back(8); ihdr.push_back(channels == 4 ? 6 : 2); ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);
    chunk("IHDR", ihdr);
    chunk("IDAT", zlibCompress(raw.data(), raw.size(), 6));
    chunk("IEND", {});
    return writeFile(path, out, err);
}

namespace {

bool readPng(const std::vector<uint8_t>& f, Image& img, bool srgb, std::string* err) {
    auto fail = [&](const char* m) { if (err) *err = m; return false; };
    static const uint8_t sig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    if (f.size() < 8 || std::memcmp(f.data(), sig, 8)) return fail("Not a PNG file");
    auto be32 = [&](size_t p) { return (uint32_t(f[p]) << 24) | (uint32_t(f[p + 1]) << 16) | (uint32_t(f[p + 2]) << 8) | f[p + 3]; };
    size_t p = 8;
    int w = 0, h = 0, depth = 0, ctype = 0, interlace = 0;
    std::vector<uint8_t> idat, plte, trns;
    while (p + 8 <= f.size()) {
        uint32_t len = be32(p);
        if (p + 12 + len > f.size()) return fail("Truncated PNG");
        const char* type = reinterpret_cast<const char*>(&f[p + 4]);
        const uint8_t* d = &f[p + 8];
        if (!std::memcmp(type, "IHDR", 4)) {
            w = int(be32(p + 8)); h = int(be32(p + 12)); depth = d[8]; ctype = d[9]; interlace = d[12];
        } else if (!std::memcmp(type, "PLTE", 4)) plte.assign(d, d + len);
        else if (!std::memcmp(type, "tRNS", 4)) trns.assign(d, d + len);
        else if (!std::memcmp(type, "IDAT", 4)) idat.insert(idat.end(), d, d + len);
        else if (!std::memcmp(type, "IEND", 4)) break;
        p += 12 + len;
    }
    if (w <= 0 || h <= 0) return fail("PNG has no image header");
    if (interlace) return fail("Interlaced PNGs are not supported; re-save without interlacing");
    int spp = ctype == 0 ? 1 : ctype == 2 ? 3 : ctype == 3 ? 1 : ctype == 4 ? 2 : ctype == 6 ? 4 : 0;
    if (!spp || !(depth == 1 || depth == 2 || depth == 4 || depth == 8 || depth == 16)) return fail("Unsupported PNG format");
    const size_t bitsPP = size_t(spp) * size_t(depth);
    const size_t stride = (size_t(w) * bitsPP + 7) / 8;
    const size_t bpp = std::max<size_t>(1, bitsPP / 8);
    std::vector<uint8_t> raw;
    if (!zlibDecompress(idat.data(), idat.size(), raw, (stride + 1) * size_t(h))) return fail("Corrupt PNG data");
    if (raw.size() < (stride + 1) * size_t(h)) return fail("Truncated PNG data");
    std::vector<uint8_t> pix(stride * size_t(h));
    for (int y = 0; y < h; ++y) {
        uint8_t ft = raw[size_t(y) * (stride + 1)];
        const uint8_t* src = &raw[size_t(y) * (stride + 1) + 1];
        uint8_t* row = &pix[size_t(y) * stride];
        const uint8_t* up = y ? &pix[size_t(y - 1) * stride] : nullptr;
        for (size_t i = 0; i < stride; ++i) {
            int a = i >= bpp ? row[i - bpp] : 0, b = up ? up[i] : 0, c = (up && i >= bpp) ? up[i - bpp] : 0, pred = 0;
            if (ft == 1) pred = a;
            else if (ft == 2) pred = b;
            else if (ft == 3) pred = (a + b) / 2;
            else if (ft == 4) { int pp = a + b - c, pa = std::abs(pp - a), pb = std::abs(pp - b), pc = std::abs(pp - c); pred = (pa <= pb && pa <= pc) ? a : (pb <= pc ? b : c); }
            else if (ft != 0) return fail("Bad PNG filter");
            row[i] = uint8_t(src[i] + pred);
        }
    }
    const bool alphaPal = ctype == 3 && !trns.empty();
    int outC = ctype == 3 ? (alphaPal ? 4 : 3) : spp;
    img.w = w; img.h = h; img.c = outC;
    img.px.assign(size_t(w) * size_t(h) * size_t(outC), 0.f);
    const float maxv = float((1 << depth) - 1);
    for (int y = 0; y < h; ++y) {
        const uint8_t* row = &pix[size_t(y) * stride];
        for (int x = 0; x < w; ++x) {
            float* o = &img.px[(size_t(y) * size_t(w) + size_t(x)) * size_t(outC)];
            auto sample = [&](int s) -> uint32_t {
                size_t idx = size_t(x) * size_t(spp) + size_t(s);
                if (depth == 8) return row[idx];
                if (depth == 16) return (uint32_t(row[idx * 2]) << 8) | row[idx * 2 + 1];
                size_t bit = idx * size_t(depth);
                return (row[bit / 8] >> (8 - depth - int(bit % 8))) & ((1u << depth) - 1);
            };
            if (ctype == 3) {
                uint32_t i = sample(0);
                if (size_t(i) * 3 + 2 >= plte.size()) return fail("Bad PNG palette index");
                for (int k = 0; k < 3; ++k) o[k] = plte[i * 3 + uint32_t(k)] / 255.f;
                if (alphaPal) o[3] = i < trns.size() ? trns[i] / 255.f : 1.f;
            } else {
                for (int s = 0; s < spp; ++s) o[s] = float(sample(s)) / maxv;
            }
            if (srgb) {
                int colorCh = (outC == 2 || outC == 4) ? outC - 1 : outC;
                for (int k = 0; k < colorCh; ++k) o[k] = srgbToLin(o[k]);
            }
        }
    }
    return true;
}

bool readHdr(const std::vector<uint8_t>& f, Image& img, std::string* err) {
    auto fail = [&](const char* m) { if (err) *err = m; return false; };
    size_t p = 0;
    auto line = [&]() {
        std::string s;
        while (p < f.size() && f[p] != '\n') s.push_back(char(f[p++]));
        ++p;
        return s;
    };
    std::string first = line();
    if (first.rfind("#?", 0) != 0) return fail("Not a Radiance HDR file");
    for (;;) {
        if (p >= f.size()) return fail("Truncated HDR header");
        std::string l = line();
        if (l.empty()) break;
        if (l.rfind("FORMAT=", 0) == 0 && l != "FORMAT=32-bit_rle_rgbe") return fail("Only RGBE HDR files are supported");
    }
    std::string res = line();
    int w = 0, h = 0;
    char ya[3] = {0}, xa[3] = {0};
    if (std::sscanf(res.c_str(), "%2s %d %2s %d", ya, &h, xa, &w) != 4 || std::string(ya) != "-Y" || std::string(xa) != "+X" || w <= 0 || h <= 0)
        return fail("Unsupported HDR orientation");
    img.w = w; img.h = h; img.c = 3;
    img.px.assign(size_t(w) * size_t(h) * 3, 0.f);
    std::vector<uint8_t> scan(size_t(w) * 4);
    for (int y = 0; y < h; ++y) {
        if (p + 4 > f.size()) return fail("Truncated HDR data");
        if (w >= 8 && w < 32768 && f[p] == 2 && f[p + 1] == 2 && ((f[p + 2] << 8) | f[p + 3]) == w) {
            p += 4;
            for (int ch = 0; ch < 4; ++ch) {
                int x = 0;
                while (x < w) {
                    if (p >= f.size()) return fail("Truncated HDR data");
                    int count = f[p++];
                    if (count > 128) {
                        count -= 128;
                        if (x + count > w || p >= f.size()) return fail("Bad HDR run");
                        uint8_t v = f[p++];
                        while (count--) scan[size_t(x++) * 4 + size_t(ch)] = v;
                    } else {
                        if (count == 0 || x + count > w || p + size_t(count) > f.size()) return fail("Bad HDR run");
                        while (count--) scan[size_t(x++) * 4 + size_t(ch)] = f[p++];
                    }
                }
            }
        } else {
            if (p + size_t(w) * 4 > f.size()) return fail("Truncated HDR data");
            std::memcpy(scan.data(), &f[p], size_t(w) * 4);
            p += size_t(w) * 4;
        }
        for (int x = 0; x < w; ++x) {
            const uint8_t* q = &scan[size_t(x) * 4];
            float* o = &img.px[(size_t(y) * size_t(w) + size_t(x)) * 3];
            if (q[3] == 0) { o[0] = o[1] = o[2] = 0; continue; }
            float s = std::ldexp(1.f, int(q[3]) - (128 + 8));
            o[0] = q[0] * s; o[1] = q[1] * s; o[2] = q[2] * s;
        }
    }
    return true;
}

// ---- EXR helpers
void putU32(std::vector<uint8_t>& v, uint32_t x) { for (int i = 0; i < 4; ++i) v.push_back(uint8_t(x >> (8 * i))); }
void putU64(std::vector<uint8_t>& v, uint64_t x) { for (int i = 0; i < 8; ++i) v.push_back(uint8_t(x >> (8 * i))); }
void putF32(std::vector<uint8_t>& v, float f) { uint32_t u; std::memcpy(&u, &f, 4); putU32(v, u); }
void putStr(std::vector<uint8_t>& v, const std::string& s) { v.insert(v.end(), s.begin(), s.end()); v.push_back(0); }
uint32_t getU32(const uint8_t* p) { return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); }
uint64_t getU64(const uint8_t* p) { return uint64_t(getU32(p)) | (uint64_t(getU32(p + 4)) << 32); }

void exrZipEncode(std::vector<uint8_t>& data) {
    std::vector<uint8_t> t(data.size());
    size_t half = (data.size() + 1) / 2, a = 0, b = half;
    for (size_t i = 0; i < data.size(); ++i) (i % 2 == 0 ? t[a++] : t[b++]) = data[i];
    int prev = t.empty() ? 0 : t[0];
    for (size_t i = 1; i < t.size(); ++i) { int d = int(t[i]) - prev + (128 + 256); prev = t[i]; t[i] = uint8_t(d); }
    data.swap(t);
}

void exrZipDecode(std::vector<uint8_t>& t) {
    for (size_t i = 1; i < t.size(); ++i) t[i] = uint8_t(int(t[i - 1]) + int(t[i]) - 128);
    std::vector<uint8_t> o(t.size());
    size_t half = (t.size() + 1) / 2, a = 0, b = half;
    for (size_t i = 0; i < o.size(); ++i) o[i] = (i % 2 == 0) ? t[a++] : t[b++];
    t.swap(o);
}

} // namespace

bool writeHdr(const std::string& path, const Image& img, std::string* err) {
    std::string head = "#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y " + std::to_string(img.h) + " +X " + std::to_string(img.w) + "\n";
    std::vector<uint8_t> out(head.begin(), head.end());
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) {
            float r = img.at(x, y, 0), g = img.c > 1 ? img.at(x, y, 1) : r, b = img.c > 2 ? img.at(x, y, 2) : r;
            float v = std::max(r, std::max(g, b));
            if (v < 1e-32f) { for (int k = 0; k < 4; ++k) out.push_back(0); continue; }
            int e;
            float m = std::frexp(v, &e) * 256.f / v;
            out.push_back(uint8_t(std::max(0.f, r * m))); out.push_back(uint8_t(std::max(0.f, g * m))); out.push_back(uint8_t(std::max(0.f, b * m)));
            out.push_back(uint8_t(e + 128));
        }
    return writeFile(path, out, err);
}

bool writeExr(const std::string& path, int w, int h, const std::vector<ExrChannel>& chans, bool half, std::string* err) {
    if (w <= 0 || h <= 0 || chans.empty()) { if (err) *err = "Nothing to write"; return false; }
    std::vector<ExrChannel> ch = chans;
    std::sort(ch.begin(), ch.end(), [](const ExrChannel& a, const ExrChannel& b) { return a.name < b.name; });
    std::vector<uint8_t> o;
    putU32(o, 20000630); putU32(o, 2);
    auto attr = [&](const char* name, const char* type, const std::vector<uint8_t>& val) {
        putStr(o, name); putStr(o, type); putU32(o, uint32_t(val.size())); o.insert(o.end(), val.begin(), val.end());
    };
    std::vector<uint8_t> v;
    for (auto& c : ch) { putStr(v, c.name); putU32(v, half ? 1 : 2); v.push_back(0); v.push_back(0); v.push_back(0); v.push_back(0); putU32(v, 1); putU32(v, 1); }
    v.push_back(0);
    attr("channels", "chlist", v);
    attr("compression", "compression", {3}); // ZIP, 16 scanlines per block
    v.clear(); putU32(v, 0); putU32(v, 0); putU32(v, uint32_t(w - 1)); putU32(v, uint32_t(h - 1));
    attr("dataWindow", "box2i", v);
    attr("displayWindow", "box2i", v);
    attr("lineOrder", "lineOrder", {0});
    v.clear(); putF32(v, 1); attr("pixelAspectRatio", "float", v);
    v.clear(); putF32(v, 0); putF32(v, 0); attr("screenWindowCenter", "v2f", v);
    v.clear(); putF32(v, 1); attr("screenWindowWidth", "float", v);
    o.push_back(0);
    const int lines = 16;
    const int chunks = (h + lines - 1) / lines;
    size_t tablePos = o.size();
    for (int i = 0; i < chunks; ++i) putU64(o, 0);
    const size_t bpv = half ? 2 : 4;
    for (int cidx = 0; cidx < chunks; ++cidx) {
        int y0 = cidx * lines, y1 = std::min(h, y0 + lines);
        std::vector<uint8_t> raw;
        raw.reserve(size_t(y1 - y0) * size_t(w) * ch.size() * bpv);
        for (int y = y0; y < y1; ++y)
            for (auto& c : ch)
                for (int x = 0; x < w; ++x) {
                    float f = c.data[size_t(y) * size_t(w) + size_t(x)];
                    if (half) { uint16_t hv = floatToHalf(f); raw.push_back(uint8_t(hv)); raw.push_back(uint8_t(hv >> 8)); }
                    else { uint32_t u; std::memcpy(&u, &f, 4); for (int k = 0; k < 4; ++k) raw.push_back(uint8_t(u >> (8 * k))); }
                }
        std::vector<uint8_t> enc = raw;
        exrZipEncode(enc);
        std::vector<uint8_t> z = zlibCompress(enc.data(), enc.size(), 6);
        const std::vector<uint8_t>& payload = z.size() < raw.size() ? z : raw; // stored raw when compression does not help
        uint64_t off = o.size();
        for (int k = 0; k < 8; ++k) o[tablePos + size_t(cidx) * 8 + size_t(k)] = uint8_t(off >> (8 * k));
        putU32(o, uint32_t(y0));
        putU32(o, uint32_t(payload.size()));
        o.insert(o.end(), payload.begin(), payload.end());
    }
    return writeFile(path, o, err);
}

bool readExrChannels(const std::string& path, int& w, int& h, std::vector<std::pair<std::string, std::vector<float>>>& out, std::string* err) {
    auto fail = [&](const std::string& m) { if (err) *err = m; return false; };
    std::vector<uint8_t> f;
    if (!readFile(path, f, err)) return false;
    if (f.size() < 8 || getU32(f.data()) != 20000630) return fail("Not an OpenEXR file");
    uint32_t version = getU32(&f[4]);
    if (version & 0x200) return fail("Tiled EXR files are not supported");
    if (version & 0x1000) return fail("Multi-part EXR files are not supported");
    size_t p = 8;
    struct Ch { std::string name; int type; };
    std::vector<Ch> chans;
    int comp = 0, xmin = 0, ymin = 0, xmax = -1, ymax = -1;
    auto readStr = [&](std::string& s) { s.clear(); while (p < f.size() && f[p]) s.push_back(char(f[p++])); ++p; return p <= f.size(); };
    for (;;) {
        std::string name, type;
        if (!readStr(name)) return fail("Truncated EXR header");
        if (name.empty()) break;
        if (!readStr(type) || p + 4 > f.size()) return fail("Truncated EXR header");
        uint32_t sz = getU32(&f[p]); p += 4;
        if (p + sz > f.size()) return fail("Truncated EXR header");
        const uint8_t* d = &f[p];
        if (name == "channels") {
            size_t q = 0;
            while (q < sz && d[q]) {
                std::string cn;
                while (q < sz && d[q]) cn.push_back(char(d[q++]));
                ++q;
                if (q + 16 > sz) return fail("Bad EXR channel list");
                int t = int(getU32(d + q));
                if (getU32(d + q + 8) != 1 || getU32(d + q + 12) != 1) return fail("Subsampled EXR channels are not supported");
                chans.push_back({cn, t});
                q += 16;
            }
        } else if (name == "compression") comp = d[0];
        else if (name == "dataWindow") { xmin = int(getU32(d)); ymin = int(getU32(d + 4)); xmax = int(getU32(d + 8)); ymax = int(getU32(d + 12)); }
        p += sz;
    }
    if (comp != 0 && comp != 2 && comp != 3) return fail("EXR compression not supported (use ZIP, ZIPS or none)");
    w = xmax - xmin + 1; h = ymax - ymin + 1;
    if (w <= 0 || h <= 0 || chans.empty()) return fail("Empty EXR");
    const int lines = comp == 3 ? 16 : 1;
    const int chunks = (h + lines - 1) / lines;
    if (p + size_t(chunks) * 8 > f.size()) return fail("Truncated EXR offsets");
    std::vector<uint64_t> offs(static_cast<size_t>(chunks));
    for (int i = 0; i < chunks; ++i) offs[size_t(i)] = getU64(&f[p + size_t(i) * 8]);
    out.clear();
    for (auto& c : chans) out.push_back({c.name, std::vector<float>(size_t(w) * size_t(h), 0.f)});
    size_t lineBytes = 0;
    for (auto& c : chans) lineBytes += size_t(w) * (c.type == 1 ? 2 : 4);
    for (int ci = 0; ci < chunks; ++ci) {
        size_t q = size_t(offs[size_t(ci)]);
        if (q + 8 > f.size()) return fail("Bad EXR chunk offset");
        int y0 = int(getU32(&f[q])) - ymin;
        uint32_t sz = getU32(&f[q + 4]);
        if (q + 8 + sz > f.size() || y0 < 0 || y0 >= h) return fail("Bad EXR chunk");
        int nl = std::min(lines, h - y0);
        size_t expect = lineBytes * size_t(nl);
        std::vector<uint8_t> data(&f[q + 8], &f[q + 8] + sz);
        if (comp != 0 && sz < expect) {
            std::vector<uint8_t> dec;
            if (!zlibDecompress(data.data(), data.size(), dec, expect) || dec.size() != expect) return fail("Corrupt EXR data");
            exrZipDecode(dec);
            data.swap(dec);
        }
        if (data.size() < expect) return fail("Truncated EXR chunk");
        size_t r = 0;
        for (int l = 0; l < nl; ++l)
            for (size_t c = 0; c < chans.size(); ++c)
                for (int x = 0; x < w; ++x) {
                    float val;
                    if (chans[c].type == 1) { val = halfToFloat(uint16_t(data[r] | (data[r + 1] << 8))); r += 2; }
                    else if (chans[c].type == 2) { uint32_t u = getU32(&data[r]); std::memcpy(&val, &u, 4); r += 4; }
                    else { val = float(getU32(&data[r])); r += 4; }
                    out[c].second[size_t(y0 + l) * size_t(w) + size_t(x)] = val;
                }
    }
    return true;
}

bool readImage(const std::string& path, Image& img, bool srgb, std::string* err) {
    if (endsWith(path, ".exr")) {
        int w, h;
        std::vector<std::pair<std::string, std::vector<float>>> ch;
        if (!readExrChannels(path, w, h, ch, err)) return false;
        auto find = [&](const char* n) -> const std::vector<float>* { for (auto& c : ch) if (c.first == n) return &c.second; return nullptr; };
        const std::vector<float>* r = find("R");
        const std::vector<float>* g = find("G");
        const std::vector<float>* b = find("B");
        const std::vector<float>* a = find("A");
        const std::vector<float>* y = find("Y");
        if (!r && !y) r = &ch[0].second;
        img.w = w; img.h = h;
        if (r && g && b) {
            img.c = a ? 4 : 3;
            img.px.resize(size_t(w) * size_t(h) * size_t(img.c));
            for (size_t i = 0; i < size_t(w) * size_t(h); ++i) {
                img.px[i * size_t(img.c)] = (*r)[i]; img.px[i * size_t(img.c) + 1] = (*g)[i]; img.px[i * size_t(img.c) + 2] = (*b)[i];
                if (a) img.px[i * 4 + 3] = (*a)[i];
            }
        } else {
            const std::vector<float>& one = y ? *y : *r;
            img.c = 1;
            img.px = one;
        }
        return true;
    }
    std::vector<uint8_t> f;
    if (!readFile(path, f, err)) return false;
    if (endsWith(path, ".png")) return readPng(f, img, srgb, err);
    if (endsWith(path, ".hdr") || endsWith(path, ".pic")) return readHdr(f, img, err);
    if (err) *err = "Unsupported image format (use .png, .exr or .hdr): " + path;
    return false;
}

// ================================================================== paths

namespace {
std::mutex& hipMutex() { static std::mutex m; return m; }
std::string& hipDir() { static std::string d; return d; }
} // namespace

void setSceneDirectory(const std::string& dir) {
    std::lock_guard<std::mutex> lk(hipMutex());
    hipDir() = dir;
}

std::string resolvePath(const std::string& path) {
    if (path.rfind("$HIP", 0) != 0) return path;
    std::lock_guard<std::mutex> lk(hipMutex());
    std::string d = hipDir().empty() ? std::string(".") : hipDir();
    return d + path.substr(4);
}

// ================================================================== textures

namespace {
std::mutex& regMutex() { static std::mutex m; return m; }
std::vector<std::string>& regPaths() { static std::vector<std::string> v; return v; }
} // namespace

int registerTexturePath(const std::string& path) {
    if (path.empty()) return 0;
    std::lock_guard<std::mutex> lk(regMutex());
    auto& v = regPaths();
    for (size_t i = 0; i < v.size(); ++i) if (v[i] == path) return int(i) + 1;
    v.push_back(path);
    return int(v.size());
}

std::string texturePath(int id) {
    std::lock_guard<std::mutex> lk(regMutex());
    auto& v = regPaths();
    if (id <= 0 || size_t(id) > v.size()) return {};
    return v[size_t(id) - 1];
}

bool Texture::load(const std::string& pathIn, bool color, std::string* err) {
    tiles_.clear();
    const std::string path = resolvePath(pathIn);
    size_t tok = path.find("<UDIM>");
    udim_ = tok != std::string::npos;
    if (!udim_) {
        Tile t;
        if (!readImage(path, t.img, color, err)) return false;
        tiles_.push_back(std::move(t));
        return true;
    }
    for (int tile = 1001; tile <= 1100; ++tile) {
        std::string p = path;
        p.replace(tok, 6, std::to_string(tile));
        std::ifstream test(p, std::ios::binary);
        if (!test) continue;
        Tile t;
        t.udim = tile;
        if (!readImage(p, t.img, color, err)) return false;
        tiles_.push_back(std::move(t));
    }
    if (tiles_.empty()) { if (err) *err = "No UDIM tiles found for " + path; return false; }
    return true;
}

const Image* Texture::tileFor(float u, float v, float& lu, float& lv) const {
    if (!udim_) { lu = u - std::floor(u); lv = v - std::floor(v); return &tiles_[0].img; }
    int tu = int(std::floor(u)), tv = int(std::floor(v));
    if (tu < 0 || tu > 9 || tv < 0 || tv > 9) return nullptr;
    int id = 1001 + tu + 10 * tv;
    lu = u - float(tu); lv = v - float(tv);
    for (auto& t : tiles_) if (t.udim == id) return &t.img;
    return nullptr;
}

void Texture::sample(float u, float v, float out[4]) const {
    out[0] = out[1] = out[2] = 0; out[3] = 1;
    if (tiles_.empty() || !(u == u) || !(v == v)) return;
    float lu, lv;
    const Image* im = tileFor(u, v, lu, lv);
    if (!im || im->w == 0) return;
    // texel centres; v = 0 is the bottom row
    float x = lu * float(im->w) - 0.5f, y = (1.f - lv) * float(im->h) - 0.5f;
    int x0 = int(std::floor(x)), y0 = int(std::floor(y));
    float fx = x - float(x0), fy = y - float(y0);
    auto wrap = [](int a, int n) { a %= n; return a < 0 ? a + n : a; };
    float acc[4] = {0, 0, 0, 0};
    for (int k = 0; k < 4; ++k) {
        int xi = wrap(x0 + (k & 1), im->w), yi = udim_ ? std::clamp(y0 + (k >> 1), 0, im->h - 1) : wrap(y0 + (k >> 1), im->h);
        if (udim_) xi = std::clamp(x0 + (k & 1), 0, im->w - 1);
        float wgt = ((k & 1) ? fx : 1 - fx) * ((k >> 1) ? fy : 1 - fy);
        const float* px = &im->px[(size_t(yi) * size_t(im->w) + size_t(xi)) * size_t(im->c)];
        float rgba[4];
        if (im->c == 1) { rgba[0] = rgba[1] = rgba[2] = px[0]; rgba[3] = 1; }
        else if (im->c == 2) { rgba[0] = rgba[1] = rgba[2] = px[0]; rgba[3] = px[1]; }
        else { rgba[0] = px[0]; rgba[1] = px[1]; rgba[2] = px[2]; rgba[3] = im->c > 3 ? px[3] : 1; }
        for (int c = 0; c < 4; ++c) acc[c] += rgba[c] * wgt;
    }
    for (int c = 0; c < 4; ++c) out[c] = acc[c];
}

} // namespace hd

#include "Renderer.h"
#include "GL.h"
#include "core/Image.h"
#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <set>

namespace hd {

// ---------------------------------------------------------------- Camera
Vec3 Camera::eye() const {
    float p = radians(pitch), y = radians(yaw);
    return target + Vec3(std::cos(p) * std::sin(y), std::sin(p), std::cos(p) * std::cos(y)) * dist;
}
Mat4 Camera::view() const { return Mat4::lookAt(eye(), target, Vec3(0, 1, 0)); }
Mat4 Camera::proj(float aspect) const {
    float n = std::max(nearClip, dist * 0.002f);
    float f = std::max(farClip, dist * 100.f);
    return Mat4::perspective(radians(fovY), std::max(aspect, 1e-3f), n, f);
}
void Camera::tumble(float dx, float dy) {
    yaw -= dx * 0.3f;
    pitch = std::clamp(pitch + dy * 0.3f, -89.5f, 89.5f);
}
void Camera::pan(float dx, float dy, float vh) {
    float p = radians(pitch), y = radians(yaw);
    Vec3 fwd = -Vec3(std::cos(p) * std::sin(y), std::sin(p), std::cos(p) * std::cos(y));
    Vec3 right = normalize(cross(fwd, Vec3(0, 1, 0)));
    Vec3 up = cross(right, fwd);
    float worldPerPixel = 2.f * dist * std::tan(radians(fovY) * 0.5f) / std::max(vh, 1.f);
    target += (right * -dx + up * dy) * worldPerPixel;
}
void Camera::dolly(float amount) { dist = std::clamp(dist * std::exp(-amount), 1e-3f, 1e5f); }
void Camera::frame(const Vec3& mn, const Vec3& mx) {
    target = (mn + mx) * 0.5f;
    float r = std::max(length(mx - mn) * 0.5f, 0.05f);
    dist = r / std::sin(radians(fovY) * 0.5f) * 1.05f;
}
void Camera::home() { *this = Camera(); }

// ---------------------------------------------------------------- shaders
namespace {
const char* kMeshVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec3 aCol;
uniform mat4 uView, uProj;
out vec3 vN; out vec3 vCol; out vec3 vPosV;
void main() {
    vec4 pv = uView * vec4(aPos, 1.0);
    vPosV = pv.xyz;
    vN = mat3(uView) * aNrm;
    vCol = aCol;
    gl_Position = uProj * pv;
})";

const char* kMeshFS = R"(#version 330 core
in vec3 vN; in vec3 vCol; in vec3 vPosV;
uniform int uBackTint;
out vec4 frag;
void main() {
    vec3 N = normalize(vN);
    vec3 V = normalize(-vPosV);
    vec3 col = vCol;
    if (!gl_FrontFacing) { N = -N; if (uBackTint == 1) col *= vec3(0.55, 0.6, 1.1); }
    if (dot(N, V) < 0.0) N = -N; // flat-shaded fallbacks: always light the visible side
    vec3 L1 = normalize(vec3(-0.35, 0.65, 0.68));
    vec3 L2 = normalize(vec3(0.6, -0.15, 0.45));
    float d1 = max(dot(N, L1), 0.0);
    float d2 = max(dot(N, L2), 0.0);
    float hemi = 0.5 + 0.5 * N.y;
    vec3 H = normalize(L1 + V);
    float spec = pow(max(dot(N, H), 0.0), 48.0) * 0.18;
    float rim = pow(1.0 - max(dot(N, V), 0.0), 3.0) * 0.12;
    vec3 lit = col * (0.16 + 0.12 * hemi + 0.68 * d1 + 0.18 * d2) + vec3(spec + rim);
    frag = vec4(pow(lit, vec3(1.0/1.1)), 1.0);
})";

// Material preview: GGX specular + Lambert diffuse under a fixed studio rig (key and fill lights that
// follow the camera, a sky/ground environment), with the same material attributes the path tracer
// reads. Glass is drawn in two blended passes: a multiplicative tint, then additive reflections.
const char* kMatVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec3 aCol;
layout(location=3) in vec2 aUV;
layout(location=4) in vec3 aMat;   // rough, metal, spec
layout(location=5) in vec3 aMat2;  // transmit, emit, sss
uniform mat4 uView, uProj;
out vec3 vN; out vec3 vCol; out vec3 vPosV; out vec2 vUV; out vec3 vMat; out vec3 vMat2;
void main() {
    vec4 pv = uView * vec4(aPos, 1.0);
    vPosV = pv.xyz;
    vN = mat3(uView) * aNrm;
    vCol = aCol; vUV = aUV; vMat = aMat; vMat2 = aMat2;
    gl_Position = uProj * pv;
})";

const char* kMatFS = R"(#version 330 core
in vec3 vN; in vec3 vCol; in vec3 vPosV; in vec2 vUV; in vec3 vMat; in vec3 vMat2;
uniform sampler2DArray uMap[4];      // base, rough, metal, normal
uniform int uHas[4];
uniform int uUdim[4];
uniform int uTiles[400];             // per map: layer of UDIM tile 1001+i, -1 if missing
uniform int uPass;                   // 0 opaque, 1 glass tint (multiply), 2 glass reflections (add)
uniform int uBackTint;
uniform vec3 uUpV;                   // world up in view space
out vec4 frag;
const float PI = 3.14159265;

vec4 sampleMap(int m, vec2 uv) {
    vec2 gx = dFdx(uv), gy = dFdy(uv);
    float layer = 0.0;
    vec2 local;
    if (uUdim[m] == 1) {
        int tu = int(floor(uv.x)), tv = int(floor(uv.y));
        if (tu < 0 || tu > 9 || tv < 0 || tv > 9) return vec4(0.0, 0.0, 0.0, 1.0);
        int l = uTiles[m * 100 + tu + 10 * tv];
        if (l < 0) return vec4(0.0, 0.0, 0.0, 1.0);
        layer = float(l);
        local = uv - vec2(tu, tv);
    } else local = uv;
    // images are stored top row first; v = 0 is the bottom of the image
    vec2 st = vec2(local.x, 1.0 - local.y);
    gx.y = -gx.y; gy.y = -gy.y;
    if (m == 0) return textureGrad(uMap[0], vec3(st, layer), gx, gy);
    if (m == 1) return textureGrad(uMap[1], vec3(st, layer), gx, gy);
    if (m == 2) return textureGrad(uMap[2], vec3(st, layer), gx, gy);
    return textureGrad(uMap[3], vec3(st, layer), gx, gy);
}

vec3 envColor(vec3 dir, float rough) {
    float h = dot(dir, uUpV);
    vec3 sky = vec3(0.55, 0.62, 0.75), horizon = vec3(0.75, 0.74, 0.72), ground = vec3(0.16, 0.15, 0.14);
    vec3 sharp = h > 0.0 ? mix(horizon, sky, smoothstep(0.0, 0.6, h)) : mix(horizon * 0.6, ground, smoothstep(0.0, 0.15, -h));
    // a soft studio light panel above and in front, so smooth surfaces show a highlight
    vec3 panelDir = normalize(uUpV * 0.75 + vec3(-0.3, 0.0, 0.6));
    float panel = pow(max(dot(dir, panelDir), 0.0), mix(600.0, 6.0, rough));
    vec3 avg = vec3(0.42, 0.43, 0.45);
    return mix(sharp, avg, rough * rough) + vec3(2.5) * panel * (1.0 - 0.7 * rough);
}

float D_GGX(float NdotH, float a) {
    float a2 = a * a;
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}
float G_Smith(float NdotV, float NdotL, float a) {
    float k = a * 0.5;
    return (NdotV / (NdotV * (1.0 - k) + k)) * (NdotL / (NdotL * (1.0 - k) + k));
}
vec3 aces(vec3 x) { return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0); }

void main() {
    float transmit = clamp(vMat2.x, 0.0, 1.0);
    if (uPass == 0 && transmit > 0.01) discard;
    if (uPass != 0 && transmit <= 0.01) discard;

    vec3 N = normalize(vN);
    vec3 V = normalize(-vPosV);
    vec3 base = pow(max(vCol, vec3(0.0)), vec3(2.2)); // Cd is a display colour, as in the path tracer
    if (!gl_FrontFacing) { N = -N; if (uBackTint == 1) base *= vec3(0.55, 0.6, 1.1); }
    if (dot(N, V) < 0.0) N = -N;
    float rough = vMat.x, metal = vMat.y, spec = vMat.z;
    if (uHas[0] == 1) base *= sampleMap(0, vUV).rgb;
    if (uHas[1] == 1) rough = sampleMap(1, vUV).r;
    if (uHas[2] == 1) metal = sampleMap(2, vUV).r;
    if (uHas[3] == 1) {
        // cotangent frame from screen-space derivatives (no tangent attribute needed)
        vec3 dp1 = dFdx(vPosV), dp2 = dFdy(vPosV);
        vec2 du1 = dFdx(vUV), du2 = dFdy(vUV);
        vec3 dp2perp = cross(dp2, N), dp1perp = cross(N, dp1);
        vec3 T = dp2perp * du1.x + dp1perp * du2.x;
        vec3 B = dp2perp * du1.y + dp1perp * du2.y;
        float m = max(dot(T, T), dot(B, B));
        if (m > 1e-20) {
            float inv = inversesqrt(m);
            vec3 t = sampleMap(3, vUV).xyz * 2.0 - 1.0;
            vec3 mapped = normalize(T * inv * t.x + B * inv * t.y + N * max(0.05, t.z));
            if (dot(mapped, V) > 0.0) N = mapped;
        }
    }
    rough = clamp(rough, 0.03, 1.0);
    metal = clamp(metal, 0.0, 1.0);
    float emit = max(vMat2.y, 0.0), sss = clamp(vMat2.z, 0.0, 1.0);
    float a = rough * rough;
    float NdotV = max(dot(N, V), 1e-4);
    vec3 F0 = mix(vec3(0.08 * clamp(spec, 0.0, 1.0)), base, metal);
    vec3 diffCol = base * (1.0 - metal) * (1.0 - transmit);
    vec3 Fenv = F0 + (max(vec3(1.0 - rough), F0) - F0) * pow(1.0 - NdotV, 5.0);

    vec3 direct = vec3(0.0), directSpec = vec3(0.0);
    vec3 Ls[2] = vec3[2](normalize(vec3(-0.35, 0.65, 0.68)), normalize(vec3(0.6, -0.15, 0.45)));
    vec3 Lc[2] = vec3[2](vec3(2.05, 1.98, 1.86), vec3(0.4, 0.44, 0.52));
    float wrap = 0.5 * sss;
    for (int i = 0; i < 2; ++i) {
        vec3 L = Ls[i];
        float NdotL = max(dot(N, L), 0.0);
        float NdotLw = max((dot(N, L) + wrap) / (1.0 + wrap), 0.0);
        vec3 H = normalize(L + V);
        float NdotH = max(dot(N, H), 0.0);
        vec3 F = F0 + (1.0 - F0) * pow(1.0 - max(dot(H, V), 0.0), 5.0);
        vec3 sp = NdotL > 0.0 ? D_GGX(NdotH, a) * G_Smith(NdotV, NdotL, a) * F / (4.0 * NdotV * NdotL + 1e-4) * NdotL : vec3(0.0);
        direct += Lc[i] * diffCol / PI * (1.0 - F) * mix(NdotL, NdotLw, sss);
        directSpec += Lc[i] * sp;
    }
    vec3 R = reflect(-V, N);
    vec3 hemi = mix(vec3(0.14, 0.13, 0.12), vec3(0.36, 0.39, 0.46), 0.5 + 0.5 * dot(N, uUpV));
    vec3 reflection = Fenv * envColor(R, rough) + directSpec;

    if (uPass == 1) {
        // glass: what is behind is tinted by the base colour and dimmed by what reflects
        vec3 through = mix(vec3(1.0), base, 0.85) * (1.0 - max(Fenv.r, max(Fenv.g, Fenv.b)));
        frag = vec4(mix(vec3(1.0), through, transmit), 1.0);
        return;
    }
    if (uPass == 2) {
        vec3 c = reflection * transmit + base * emit;
        frag = vec4(pow(aces(c), vec3(1.0 / 2.2)), 1.0);
        return;
    }
    vec3 c = direct + diffCol * hemi + reflection + base * emit;
    frag = vec4(pow(aces(c), vec3(1.0 / 2.2)), 1.0);
})";

const char* kLineVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aCol;
uniform mat4 uView, uProj;
uniform float uDepthBias;
out vec3 vCol;
void main() {
    vCol = aCol;
    vec4 p = uProj * (uView * vec4(aPos, 1.0));
    p.z -= uDepthBias * p.w;
    gl_Position = p;
})";

const char* kLineFS = R"(#version 330 core
in vec3 vCol;
uniform vec4 uOverride;
uniform float uAlpha;
out vec4 frag;
void main() { frag = vec4(mix(vCol, uOverride.rgb, uOverride.a), uAlpha); }
)";

const char* kPointVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aCol;
layout(location=2) in float aScale;
uniform mat4 uView, uProj;
uniform float uPixelSize;    // used when uUseScale == 0
uniform int uUseScale;
uniform float uViewportH;
out vec3 vCol;
void main() {
    vec4 pv = uView * vec4(aPos, 1.0);
    gl_Position = uProj * pv;
    vCol = aCol;
    float s = uPixelSize;
    if (uUseScale == 1) s = 2.0 * aScale * uProj[1][1] * uViewportH * 0.5 / max(-pv.z, 1e-4);
    gl_PointSize = clamp(s, 1.5, 96.0);
})";

const char* kPointFS = R"(#version 330 core
in vec3 vCol;
out vec4 frag;
void main() {
    vec2 c = gl_PointCoord * 2.0 - 1.0;
    float r2 = dot(c, c);
    if (r2 > 1.0) discard;
    float z = sqrt(1.0 - r2);
    vec3 N = vec3(c.x, -c.y, z);
    float d = max(dot(N, normalize(vec3(-0.35, 0.65, 0.68))), 0.0);
    frag = vec4(vCol * (0.3 + 0.75 * d), 1.0);
})";

const char* kBgVS = R"(#version 330 core
out float vY;
void main() {
    vec2 p = vec2((gl_VertexID == 1) ? 3.0 : -1.0, (gl_VertexID == 2) ? 3.0 : -1.0);
    vY = p.y * 0.5 + 0.5;
    gl_Position = vec4(p, 0.999, 1.0);
})";

const char* kBgFS = R"(#version 330 core
in float vY;
out vec4 frag;
uniform vec3 uTop, uBot;
void main() {
    frag = vec4(mix(uBot, uTop, smoothstep(0.0, 1.0, vY)), 1.0);
})";

const char* kVolVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uView, uProj;
uniform vec3 uBoxMin, uBoxMax;
out vec3 vWorld;
void main() {
    vWorld = mix(uBoxMin, uBoxMax, aPos);
    gl_Position = uProj * (uView * vec4(vWorld, 1.0));
})";

// Raymarches a 3D texture: r = density, g = heat, b = precomputed light transmittance.
const char* kVolFS = R"(#version 330 core
in vec3 vWorld;
uniform sampler3D uVol;
uniform sampler2D uDepth;
uniform vec3 uEye, uFwd, uBoxMin, uBoxMax, uColor;
uniform vec2 uViewport;
uniform float uProjA, uProjB, uDensity, uFire, uStep;
out vec4 frag;
void main() {
    vec3 rd = normalize(vWorld - uEye);
    vec3 inv = 1.0 / rd;
    vec3 t0 = (uBoxMin - uEye) * inv, t1 = (uBoxMax - uEye) * inv;
    vec3 tn = min(t0, t1), tf = max(t0, t1);
    float tmin = max(max(tn.x, max(tn.y, tn.z)), 0.0);
    float tmax = min(tf.x, min(tf.y, tf.z));
    float depth = texture(uDepth, gl_FragCoord.xy / uViewport).r;
    if (depth < 1.0) {
        float zv = -uProjB / (depth * 2.0 - 1.0 + uProjA);   // view-space z (negative)
        tmax = min(tmax, -zv / max(dot(rd, uFwd), 1e-4));
    }
    if (tmax <= tmin) discard;
    float jitter = fract(sin(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 43758.5453);
    float t = tmin + uStep * jitter;
    vec3 col = vec3(0.0);
    float T = 1.0;
    vec3 size = uBoxMax - uBoxMin;
    for (int i = 0; i < 220; ++i) {
        if (t >= tmax || T < 0.01) break;
        vec3 s = texture(uVol, (uEye + rd * t - uBoxMin) / size).rgb;
        if (s.r > 0.002) {
            float a = 1.0 - exp(-s.r * uDensity * uStep * 6.0);
            vec3 lit = uColor * (0.22 + 0.95 * s.b) + vec3(0.03, 0.04, 0.06);
            col += T * a * lit;
            T *= 1.0 - a;
        }
        if (uFire > 0.0 && s.g > 0.02) {
            float h = s.g;
            vec3 glow = vec3(h * 3.2, h * h * 1.7, h * h * h * 0.7);
            col += T * glow * (uFire * uStep * 5.0);
        }
        t += uStep;
    }
    frag = vec4(col, 1.0 - T);
})";

unsigned compileShader(unsigned type, const char* src, std::string& err) {
    unsigned s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048]; GLsizei n = 0;
        glGetShaderInfoLog(s, sizeof log, &n, log);
        err = std::string("shader compile failed: ") + std::string(log, size_t(n));
        glDeleteShader(s);
        return 0;
    }
    return s;
}

unsigned linkProgram(const char* vs, const char* fs, std::string& err) {
    unsigned v = compileShader(GL_VERTEX_SHADER, vs, err);
    if (!v) return 0;
    unsigned f = compileShader(GL_FRAGMENT_SHADER, fs, err);
    if (!f) { glDeleteShader(v); return 0; }
    unsigned p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048]; GLsizei n = 0;
        glGetProgramInfoLog(p, sizeof log, &n, log);
        err = std::string("program link failed: ") + std::string(log, size_t(n));
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

void setMat(unsigned prog, const char* name, const Mat4& m) {
    glUniformMatrix4fv(glGetUniformLocation(prog, name), 1, GL_FALSE, m.m);
}

void push3(std::vector<float>& v, const Vec3& a) { v.push_back(a.x); v.push_back(a.y); v.push_back(a.z); }
} // namespace

// ---------------------------------------------------------------- Renderer
Renderer::~Renderer() { shutdown(); }

bool Renderer::init(std::string& error) {
    if (inited_) return true;
    progMesh_ = linkProgram(kMeshVS, kMeshFS, error);
    progLine_ = progMesh_ ? linkProgram(kLineVS, kLineFS, error) : 0;
    progPoint_ = progLine_ ? linkProgram(kPointVS, kPointFS, error) : 0;
    progBg_ = progPoint_ ? linkProgram(kBgVS, kBgFS, error) : 0;
    progVol_ = progBg_ ? linkProgram(kVolVS, kVolFS, error) : 0;
    progMat_ = progVol_ ? linkProgram(kMatVS, kMatFS, error) : 0;
    if (!progMat_) return false;
    {
        // unit cube, outward-facing triangles
        static const float q[6][4][3] = {
            {{0, 0, 0}, {0, 0, 1}, {0, 1, 1}, {0, 1, 0}}, {{1, 0, 0}, {1, 1, 0}, {1, 1, 1}, {1, 0, 1}},
            {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}}, {{0, 1, 0}, {0, 1, 1}, {1, 1, 1}, {1, 1, 0}},
            {{0, 0, 0}, {0, 1, 0}, {1, 1, 0}, {1, 0, 0}}, {{0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}}};
        std::vector<float> cube;
        for (auto& f : q)
            for (int idx : {0, 1, 2, 0, 2, 3})
                for (int k = 0; k < 3; ++k) cube.push_back(f[idx][k]);
        const int layout[1] = {3};
        upload(cube_, cube, 3, layout, 1);
    }
    glGenVertexArrays(1, &bgVao_);
    GLint maxSamples = 0;
    glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);
    samples_ = std::max(1, std::min(4, int(maxSamples)));

    inited_ = true;
    return true;
}

void Renderer::rebuildGrid(const ViewOptions& opt) {
    const float key[8] = {float(opt.gridPlane), opt.gridOrigin.x, opt.gridOrigin.y, opt.gridOrigin.z, opt.gridSpacing, float(opt.gridCells), float(opt.gridRuler), 1.f};
    bool same = true;
    for (int i = 0; i < 8; ++i) same &= key[i] == gridKey_[i];
    if (same) return;
    for (int i = 0; i < 8; ++i) gridKey_[i] = key[i];
    // axes spanning the plane; axis lines take the usual X = red, Y = green, Z = blue
    const int u = opt.gridPlane == 2 ? 1 : 0, v = opt.gridPlane == 1 ? 1 : 2;
    const Vec3 axisCol[3] = {Vec3(0.85f, 0.25f, 0.25f), Vec3(0.35f, 0.8f, 0.3f), Vec3(0.3f, 0.45f, 0.95f)};
    const int half = std::clamp(opt.gridCells, 2, 400) / 2;
    const float sp = std::max(opt.gridSpacing, 1e-4f), ext = float(half) * sp;
    const int ruler = std::max(1, opt.gridRuler);
    std::vector<float> g;
    for (int i = -half; i <= half; ++i) {
        Vec3 c = (i % ruler == 0) ? Vec3(0.55f) : Vec3(0.38f);
        float pos = float(i) * sp;
        Vec3 a(0), b(0), c0(0), d(0);
        a[u] = -ext; a[v] = pos; b[u] = ext; b[v] = pos;   // parallel to u
        c0[v] = -ext; c0[u] = pos; d[v] = ext; d[u] = pos; // parallel to v
        Vec3 cu = i == 0 ? axisCol[u] : c, cv = i == 0 ? axisCol[v] : c;
        push3(g, a + opt.gridOrigin); push3(g, cu);
        push3(g, b + opt.gridOrigin); push3(g, cu);
        push3(g, c0 + opt.gridOrigin); push3(g, cv);
        push3(g, d + opt.gridOrigin); push3(g, cv);
    }
    const int lineLayout[2] = {3, 3};
    upload(grid_, g, 6, lineLayout, 2);
}

void Renderer::shutdown() {
    if (!inited_) return;
    for (auto& v : volumes_) if (v.tex) glDeleteTextures(1, &v.tex);
    volumes_.clear();
    if (depthTex_) glDeleteTextures(1, &depthTex_);
    if (volFbo_) glDeleteFramebuffers(1, &volFbo_);
    depthTex_ = volFbo_ = 0;
    for (auto& gr : groups_) {
        if (gr.batch.vbo) glDeleteBuffers(1, &gr.batch.vbo);
        if (gr.batch.vao) glDeleteVertexArrays(1, &gr.batch.vao);
    }
    groups_.clear();
    for (auto& t : texCache_) if (t.second.tex) glDeleteTextures(1, &t.second.tex);
    texCache_.clear();
    if (progMat_) glDeleteProgram(progMat_);
    progMat_ = 0;
    for (Batch* b : {&tris_, &wire_, &curves_, &points_, &loose_, &normals_, &grid_, &cube_, &guides_}) {
        if (b->vbo) glDeleteBuffers(1, &b->vbo);
        if (b->vao) glDeleteVertexArrays(1, &b->vao);
        *b = Batch();
    }
    for (unsigned p : {progMesh_, progLine_, progPoint_, progBg_, progVol_}) if (p) glDeleteProgram(p);
    if (bgVao_) glDeleteVertexArrays(1, &bgVao_);
    if (msFbo_) glDeleteFramebuffers(1, &msFbo_);
    if (resolveFbo_) glDeleteFramebuffers(1, &resolveFbo_);
    if (msColor_) glDeleteRenderbuffers(1, &msColor_);
    if (msDepth_) glDeleteRenderbuffers(1, &msDepth_);
    if (resolveTex_) glDeleteTextures(1, &resolveTex_);
    msFbo_ = resolveFbo_ = msColor_ = msDepth_ = resolveTex_ = 0;
    fbW_ = fbH_ = 0;
    inited_ = false;
}

void Renderer::upload(Batch& b, const std::vector<float>& data, int fpv, const int* layout, int nAttribs) {
    if (!b.vao) glGenVertexArrays(1, &b.vao);
    if (!b.vbo) glGenBuffers(1, &b.vbo);
    glBindVertexArray(b.vao);
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(data.size() * sizeof(float)), data.empty() ? nullptr : data.data(), GL_STATIC_DRAW);
    size_t off = 0;
    for (int i = 0; i < nAttribs; ++i) {
        glEnableVertexAttribArray(GLuint(i));
        glVertexAttribPointer(GLuint(i), layout[i], GL_FLOAT, GL_FALSE, GLsizei(fpv * sizeof(float)), reinterpret_cast<const void*>(off * sizeof(float)));
        off += size_t(layout[i]);
    }
    glBindVertexArray(0);
    b.count = int(data.size() / size_t(fpv));
}

void Renderer::setGuides(const std::vector<float>& lines) {
    if (!inited_ || lines == guideData_) return;
    guideData_ = lines;
    const int lineLayout[2] = {3, 3};
    upload(guides_, lines, 6, lineLayout, 2);
}

void Renderer::clearGeometry() {
    Geometry empty;
    setGeometry(empty, 0);
}

void Renderer::setGeometry(const Geometry& g, uint64_t stamp) {
    if (stamp != 0 && stamp == stamp_) return;
    stamp_ = stamp;
    const Attrib* pN = g.findAttrib(AttribClass::Point, "N");
    const Attrib* qN = g.findAttrib(AttribClass::Prim, "N");
    const Attrib* pCd = g.findAttrib(AttribClass::Point, "Cd");
    const Attrib* qCd = g.findAttrib(AttribClass::Prim, "Cd");
    const Attrib* pscale = g.findAttrib(AttribClass::Point, "pscale");
    if (pN && pN->tuple != 3) pN = nullptr;
    if (qN && qN->tuple != 3) qN = nullptr;
    const Vec3 defCol(0.72f, 0.74f, 0.78f);

    // Material channels, read like the path tracer: a prim attribute wins over a point attribute.
    struct MatA { const Attrib* prim; const Attrib* point; float def; };
    auto ma = [&](const char* n, float def) { return MatA{g.findAttrib(AttribClass::Prim, n), g.findAttrib(AttribClass::Point, n), def}; };
    const MatA aRough = ma("rough", 0.65f), aMetal = ma("metal", 0.f), aSpec = ma("spec", 0.5f), aTrans = ma("transmit", 0.f),
               aEmit = ma("emit", 0.f), aSss = ma("sss", 0.f);
    const MatA aMaps[4] = {ma("basemap", 0.f), ma("roughmap", 0.f), ma("metalmap", 0.f), ma("normalmap", 0.f)};
    auto val = [](const MatA& a, size_t prim, int pt) {
        if (a.prim) return a.prim->getF(prim);
        if (a.point) return a.point->getF(size_t(pt));
        return a.def;
    };
    const Attrib* uvA = g.findAttrib(AttribClass::Point, "uv");
    if (uvA && uvA->tuple < 2) uvA = nullptr;
    bool full = uvA != nullptr;
    for (const MatA* m : {&aRough, &aMetal, &aSpec, &aTrans, &aEmit, &aSss, &aMaps[0], &aMaps[1], &aMaps[2], &aMaps[3]})
        full |= m->prim || m->point;
    const int fpv = full ? 17 : 9;
    struct Key { int m[4]; bool operator==(const Key& o) const { return m[0] == o.m[0] && m[1] == o.m[1] && m[2] == o.m[2] && m[3] == o.m[3]; } };
    struct KeyHash { size_t operator()(const Key& k) const { return size_t(k.m[0]) * 73856093u ^ size_t(k.m[1]) * 19349663u ^ size_t(k.m[2]) * 83492791u ^ size_t(k.m[3]); } };
    std::unordered_map<Key, size_t, KeyHash> groupOf;
    std::vector<Key> keys;
    std::vector<std::vector<float>> gdata;
    std::vector<char> gtrans;
    auto groupFor = [&](const Key& k) -> size_t {
        auto it = groupOf.find(k);
        if (it != groupOf.end()) return it->second;
        size_t i = keys.size();
        groupOf.emplace(k, i);
        keys.push_back(k);
        gdata.emplace_back();
        gtrans.push_back(0);
        return i;
    };

    std::vector<float> wire, curve, pts, nrm;
    std::set<std::pair<int, int>> edges;
    for (size_t pi = 0; pi < g.numPrims(); ++pi) {
        const Prim& pr = g.prims[pi];
        if (pr.count < 2) continue;
        auto colorOf = [&](int pt) {
            if (pCd) return pCd->getV(size_t(pt));
            if (qCd) return qCd->getV(pi);
            return defCol;
        };
        if (pr.closed && pr.count >= 3) {
            Vec3 fn = qN ? qN->getV(pi) : g.primNormal(int(pi));
            const int first = g.vertices[size_t(pr.start)];
            Key key{{0, 0, 0, 0}};
            if (full)
                for (int m = 0; m < 4; ++m) key.m[m] = std::max(0, int(val(aMaps[m], pi, first) + 0.5f));
            const size_t gi = groupFor(key);
            std::vector<float>& tri = gdata[gi];
            for (int k = 1; k + 1 < pr.count; ++k) {
                int ids[3] = {first, g.vertices[size_t(pr.start + k)], g.vertices[size_t(pr.start + k + 1)]};
                for (int id : ids) {
                    push3(tri, g.P[size_t(id)]);
                    Vec3 n = pN ? pN->getV(size_t(id)) : fn;
                    if (dot(n, n) < 1e-12f) n = fn; // missing (zero) normals after merges
                    push3(tri, n);
                    push3(tri, colorOf(id));
                    if (full) {
                        Vec3 uv = uvA ? uvA->getV(size_t(id)) : Vec3(0);
                        tri.push_back(uv.x); tri.push_back(uv.y);
                        tri.push_back(val(aRough, pi, id)); tri.push_back(val(aMetal, pi, id)); tri.push_back(val(aSpec, pi, id));
                        float tr = val(aTrans, pi, id);
                        tri.push_back(tr); tri.push_back(val(aEmit, pi, id)); tri.push_back(val(aSss, pi, id));
                        if (tr > 0.01f) gtrans[gi] = 1;
                    }
                }
            }
            for (int k = 0; k < pr.count; ++k) {
                int a = g.vertices[size_t(pr.start + k)], b = g.vertices[size_t(pr.start + (k + 1) % pr.count)];
                if (edges.insert({std::min(a, b), std::max(a, b)}).second) {
                    push3(wire, g.P[size_t(a)]); push3(wire, Vec3(0.05f, 0.055f, 0.06f));
                    push3(wire, g.P[size_t(b)]); push3(wire, Vec3(0.05f, 0.055f, 0.06f));
                }
            }
        } else {
            int ne = pr.closed ? pr.count : pr.count - 1;
            for (int k = 0; k < ne; ++k) {
                int a = g.vertices[size_t(pr.start + k)], b = g.vertices[size_t(pr.start + (k + 1) % pr.count)];
                Vec3 ca = (pCd || qCd) ? colorOf(a) : Vec3(0.9f, 0.9f, 0.6f);
                Vec3 cb = (pCd || qCd) ? colorOf(b) : Vec3(0.9f, 0.9f, 0.6f);
                push3(curve, g.P[size_t(a)]); push3(curve, ca);
                push3(curve, g.P[size_t(b)]); push3(curve, cb);
            }
        }
    }
    Vec3 mn, mx;
    g.bounds(mn, mx);
    boundsMin_ = mn; boundsMax_ = mx;
    float diag = std::max(length(mx - mn), 1e-3f);
    pts.reserve(g.numPoints() * 7);
    // points not used by any primitive (particles, scattered points) are always drawn
    std::vector<char> used(g.numPoints(), 0);
    for (int vtx : g.vertices) used[size_t(vtx)] = 1;
    std::vector<float> loose;
    for (size_t i = 0; i < g.numPoints(); ++i) {
        Vec3 pc = pCd ? pCd->getV(i) : Vec3(0.95f, 0.85f, 0.35f);
        float psz = pscale ? pscale->getF(i) : 0.02f;
        push3(pts, g.P[i]);
        push3(pts, pc);
        pts.push_back(psz);
        if (!used[i]) { push3(loose, g.P[i]); push3(loose, pc); loose.push_back(psz); }
        if (pN) {
            push3(nrm, g.P[i]); push3(nrm, Vec3(0.2f, 0.9f, 1.f));
            push3(nrm, g.P[i] + pN->getV(i) * (diag * 0.04f)); push3(nrm, Vec3(0.2f, 0.9f, 1.f));
        }
    }
    hasPscale_ = pscale != nullptr;
    pointCloud_ = g.numPrims() == 0 && g.numPoints() > 0;

    const int meshLayout[3] = {3, 3, 3};
    const int lineLayout[2] = {3, 3};
    const int pointLayout[3] = {3, 3, 1};
    for (auto& gr : groups_) {
        if (gr.batch.vbo) glDeleteBuffers(1, &gr.batch.vbo);
        if (gr.batch.vao) glDeleteVertexArrays(1, &gr.batch.vao);
    }
    groups_.clear();
    fullVertex_ = full;
    const int fullLayout[6] = {3, 3, 3, 2, 3, 3};
    size_t triCount = 0;
    for (size_t i = 0; i < keys.size(); ++i) {
        if (gdata[i].empty()) continue;
        MatGroup mg;
        for (int m = 0; m < 4; ++m) mg.maps[m] = keys[i].m[m];
        mg.transmissive = gtrans[i] != 0;
        if (full) upload(mg.batch, gdata[i], fpv, fullLayout, 6);
        else upload(mg.batch, gdata[i], fpv, meshLayout, 3);
        triCount += size_t(mg.batch.count / 3);
        groups_.push_back(mg);
    }
    // texture maps: load now so errors are known with the geometry (cached by file)
    textureErrors_.clear();
    for (const MatGroup& mg : groups_)
        for (int m = 0; m < 4; ++m)
            if (mg.maps[m]) gpuTexture(mg.maps[m], m == 0);
    upload(wire_, wire, 6, lineLayout, 2);
    upload(curves_, curve, 6, lineLayout, 2);
    upload(points_, pts, 7, pointLayout, 3);
    upload(loose_, loose, 7, pointLayout, 3);
    upload(normals_, nrm, 6, lineLayout, 2);
    // ---- volumes -> 3D textures (density, heat, light)
    for (auto& v : volumes_) if (v.tex) glDeleteTextures(1, &v.tex);
    volumes_.clear();
    for (const Volume& vol : g.volumes) {
        if (volumes_.size() >= 8 || !vol.density || vol.density->size() != vol.voxels() || vol.voxels() == 0) continue;
        const int nx = vol.nx, ny = vol.ny, nz = vol.nz;
        const std::vector<float>& d = *vol.density;
        const std::vector<float>* heat = vol.heat && vol.heat->size() == d.size() ? vol.heat.get() : nullptr;
        std::vector<float> tex(d.size() * 3);
        // light transmittance toward a fixed key light, marched on the CPU
        const Vec3 L = normalize(Vec3(0.4f, 0.8f, 0.45f)) * 1.5f;
        const float k = 1.5f * vol.dx * 6.f * vol.densityScale * 0.7f;
        for (int z = 0; z < nz; ++z)
            for (int y = 0; y < ny; ++y)
                for (int x = 0; x < nx; ++x) {
                    size_t i = size_t(x) + size_t(nx) * (size_t(y) + size_t(ny) * size_t(z));
                    float light = 1.f;
                    if (d[i] > 0.002f) {
                        float sum = 0;
                        for (int s = 1; s <= 10; ++s) {
                            int sx = int(float(x) + L.x * float(s) + 0.5f), sy = int(float(y) + L.y * float(s) + 0.5f), sz = int(float(z) + L.z * float(s) + 0.5f);
                            if (sx < 0 || sy < 0 || sz < 0 || sx >= nx || sy >= ny || sz >= nz) break;
                            sum += d[size_t(sx) + size_t(nx) * (size_t(sy) + size_t(ny) * size_t(sz))];
                        }
                        light = std::exp(-sum * k);
                    }
                    tex[i * 3] = d[i];
                    tex[i * 3 + 1] = heat ? (*heat)[i] : 0.f;
                    tex[i * 3 + 2] = light;
                }
        GpuVolume gv;
        gv.mn = vol.bmin(); gv.mx = vol.bmax(); gv.color = vol.color;
        gv.density = vol.densityScale; gv.fire = vol.fire;
        gv.step = std::max(vol.dx * 0.7f, length(gv.mx - gv.mn) / 200.f);
        glGenTextures(1, &gv.tex);
        glBindTexture(GL_TEXTURE_3D, gv.tex);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB16F, nx, ny, nz, 0, GL_RGB, GL_FLOAT, tex.data());
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_3D, 0);
        volumes_.push_back(gv);
    }
    stats_.volumes = volumes_.size();
    stats_.triangles = triCount;
    stats_.lines = size_t(wire_.count / 2 + curves_.count / 2);
    stats_.points = size_t(points_.count);
}

void Renderer::ensureTargets(int w, int h) {
    if (w == fbW_ && h == fbH_ && msFbo_) return;
    if (!msFbo_) {
        glGenFramebuffers(1, &msFbo_);
        glGenFramebuffers(1, &resolveFbo_);
        glGenRenderbuffers(1, &msColor_);
        glGenRenderbuffers(1, &msDepth_);
        glGenTextures(1, &resolveTex_);
        glGenTextures(1, &depthTex_);
        glGenFramebuffers(1, &volFbo_);
    }
    fbW_ = w; fbH_ = h;
    glBindRenderbuffer(GL_RENDERBUFFER, msColor_);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples_, GL_RGBA8, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, msDepth_);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples_, GL_DEPTH24_STENCIL8, w, h);
    glBindFramebuffer(GL_FRAMEBUFFER, msFbo_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, msColor_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, msDepth_);

    glBindTexture(GL_TEXTURE_2D, resolveTex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    // resolved scene depth, sampled by the volume pass
    glBindTexture(GL_TEXTURE_2D, depthTex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8, w, h, 0, GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, resolveFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, resolveTex_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, depthTex_, 0);
    // colour-only target sharing the resolved image, so the depth texture can be read while drawing
    glBindFramebuffer(GL_FRAMEBUFFER, volFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, resolveTex_, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

const Renderer::GpuTexture* Renderer::gpuTexture(int id, bool color) {
    const std::string raw = texturePath(id);
    if (raw.empty()) return nullptr;
    const std::string path = resolvePath(raw);
    const std::string key = path + (color ? "|c" : "|d");
    auto it = texCache_.find(key);
    if (it != texCache_.end()) {
        if (!it->second.ok) noteTextureError("Viewport: cannot load " + raw);
        return it->second.ok ? &it->second : nullptr;
    }
    GpuTexture& gt = texCache_[key];
    gt.tileLayer.assign(100, -1);
    // gather the image(s): one, or every UDIM tile that exists
    std::vector<Image> layers;
    std::string err;
    const size_t tok = path.find("<UDIM>");
    gt.udim = tok != std::string::npos;
    if (!gt.udim) {
        Image im;
        if (readImage(path, im, color, &err)) layers.push_back(std::move(im));
    } else {
        for (int tile = 1001; tile <= 1100; ++tile) {
            std::string p = path;
            p.replace(tok, 6, std::to_string(tile));
            Image im;
            std::string e;
            if (!readImage(p, im, color, &e)) continue;
            gt.tileLayer[size_t(tile - 1001)] = int(layers.size());
            layers.push_back(std::move(im));
        }
    }
    if (layers.empty() || layers[0].w <= 0 || layers[0].h <= 0) {
        noteTextureError("Viewport: cannot load " + raw + (err.empty() ? "" : " (" + err + ")"));
        return nullptr;
    }
    // Every layer gets the first tile's size, halved until it fits in 2048 (and the GL limit).
    GLint maxSize = 2048;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxSize);
    const int limit = std::min(2048, int(maxSize));
    int W = layers[0].w, H = layers[0].h;
    while (W > limit || H > limit) { W = std::max(1, W / 2); H = std::max(1, H / 2); }
    std::vector<float> rgba(size_t(W) * size_t(H) * 4 * layers.size());
    for (size_t l = 0; l < layers.size(); ++l) {
        const Image& im = layers[l];
        float* dst = &rgba[size_t(W) * size_t(H) * 4 * l];
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) {
                // box filter over the source texels this destination texel covers
                int x0 = x * im.w / W, x1 = std::max(x0 + 1, (x + 1) * im.w / W);
                int y0 = y * im.h / H, y1 = std::max(y0 + 1, (y + 1) * im.h / H);
                float acc[4] = {0, 0, 0, 0};
                for (int sy = y0; sy < y1; ++sy)
                    for (int sx = x0; sx < x1; ++sx) {
                        const float* px = &im.px[(size_t(sy) * size_t(im.w) + size_t(sx)) * size_t(im.c)];
                        float r = px[0], g = im.c >= 3 ? px[1] : px[0], b = im.c >= 3 ? px[2] : px[0];
                        float a = im.c == 2 ? px[1] : im.c >= 4 ? px[3] : 1.f;
                        acc[0] += r; acc[1] += g; acc[2] += b; acc[3] += a;
                    }
                float inv = 1.f / float((x1 - x0) * (y1 - y0));
                for (int c = 0; c < 4; ++c) dst[(size_t(y) * size_t(W) + size_t(x)) * 4 + size_t(c)] = acc[c] * inv;
            }
    }
    glGenTextures(1, &gt.tex);
    glBindTexture(GL_TEXTURE_2D_ARRAY, gt.tex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA16F, W, H, GLsizei(layers.size()), 0, GL_RGBA, GL_FLOAT, rgba.data());
    glGenerateMipmap(GL_TEXTURE_2D_ARRAY);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, gt.udim ? GL_CLAMP_TO_EDGE : GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, gt.udim ? GL_CLAMP_TO_EDGE : GL_REPEAT);
    glBindTexture(GL_TEXTURE_2D_ARRAY, 0);
    if (!gt.udim) gt.tileLayer[0] = 0;
    gt.ok = true;
    return &gt;
}

void Renderer::drawMaterials(const Mat4& V, const Mat4& P, const ViewOptions& opt) {
    glUseProgram(progMat_);
    setMat(progMat_, "uView", V);
    setMat(progMat_, "uProj", P);
    glUniform1i(glGetUniformLocation(progMat_, "uBackTint"), opt.backfaceTint ? 1 : 0);
    Vec3 up = normalize(Vec3(V.m[4], V.m[5], V.m[6])); // view-space image of world +Y (column-major)
    glUniform3f(glGetUniformLocation(progMat_, "uUpV"), up.x, up.y, up.z);
    const GLint locPass = glGetUniformLocation(progMat_, "uPass");
    const GLint locTiles = glGetUniformLocation(progMat_, "uTiles");
    GLint locMap[4], locHas[4], locUdim[4];
    for (int m = 0; m < 4; ++m) {
        std::string i = "[" + std::to_string(m) + "]";
        locMap[m] = glGetUniformLocation(progMat_, ("uMap" + i).c_str());
        locHas[m] = glGetUniformLocation(progMat_, ("uHas" + i).c_str());
        locUdim[m] = glGetUniformLocation(progMat_, ("uUdim" + i).c_str());
        glUniform1i(locMap[m], m);
    }
    if (!fullVertex_) {
        // light vertex format: material channels come from constant attribute values (defaults)
        glVertexAttrib3f(3, 0.f, 0.f, 0.f);
        glVertexAttrib3f(4, 0.65f, 0.f, 0.5f);
        glVertexAttrib3f(5, 0.f, 0.f, 0.f);
    }
    std::vector<GLint> tiles(400, -1);
    auto bindGroup = [&](const MatGroup& mg) {
        for (int m = 0; m < 4; ++m) {
            const GpuTexture* t = (opt.textures && mg.maps[m]) ? gpuTexture(mg.maps[m], m == 0) : nullptr;
            glActiveTexture(GL_TEXTURE0 + unsigned(m));
            glBindTexture(GL_TEXTURE_2D_ARRAY, t ? t->tex : 0);
            glUniform1i(locHas[m], t ? 1 : 0);
            glUniform1i(locUdim[m], t && t->udim ? 1 : 0);
            for (int k = 0; k < 100; ++k) tiles[size_t(m * 100 + k)] = t ? t->tileLayer[size_t(k)] : -1;
        }
        glUniform1iv(locTiles, 400, tiles.data());
    };
    textureErrors_.clear(); // gpuTexture() re-reports cached failures below
    bool anyGlass = false;
    glUniform1i(locPass, 0);
    for (const MatGroup& mg : groups_) {
        bindGroup(mg);
        drawBatch(mg.batch, GL_TRIANGLES);
        anyGlass |= mg.transmissive;
    }
    if (anyGlass) {
        glEnable(GL_BLEND);
        glDepthMask(GL_FALSE);
        for (int pass = 1; pass <= 2; ++pass) {
            glUniform1i(locPass, pass);
            if (pass == 1) glBlendFunc(GL_ZERO, GL_SRC_COLOR); // dst *= tint
            else glBlendFunc(GL_ONE, GL_ONE);                  // dst += reflections
            for (const MatGroup& mg : groups_)
                if (mg.transmissive) { bindGroup(mg); drawBatch(mg.batch, GL_TRIANGLES); }
        }
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
    for (int m = 3; m >= 0; --m) { glActiveTexture(GL_TEXTURE0 + unsigned(m)); glBindTexture(GL_TEXTURE_2D_ARRAY, 0); }
}

void Renderer::drawBatch(const Batch& b, unsigned mode) {
    if (!b.count) return;
    glBindVertexArray(b.vao);
    glDrawArrays(mode, 0, b.count);
}

unsigned Renderer::render(int w, int h, const Camera& cam, const ViewOptions& opt) {
    if (!inited_ || w <= 0 || h <= 0) return resolveTex_;
    GLint prevFbo = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFbo);
    GLint prevViewport[4] = {0, 0, 0, 0};
    glGetIntegerv(GL_VIEWPORT, prevViewport);

    ensureTargets(w, h);
    glBindFramebuffer(GL_FRAMEBUFFER, msFbo_);
    glViewport(0, 0, w, h);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);
    glClearColor(0.15f, 0.15f, 0.17f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // background gradient
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glUseProgram(progBg_);
    glUniform3f(glGetUniformLocation(progBg_, "uTop"), opt.bgTop.x, opt.bgTop.y, opt.bgTop.z);
    glUniform3f(glGetUniformLocation(progBg_, "uBot"), opt.bgBottom.x, opt.bgBottom.y, opt.bgBottom.z);
    glBindVertexArray(bgVao_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    Mat4 V = cam.view(), P = cam.proj(float(w) / float(h));

    // grid (alpha blended, no depth write)
    if (opt.grid) {
        rebuildGrid(opt);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glUseProgram(progLine_);
        setMat(progLine_, "uView", V);
        setMat(progLine_, "uProj", P);
        glUniform1f(glGetUniformLocation(progLine_, "uDepthBias"), 0.f);
        glUniform4f(glGetUniformLocation(progLine_, "uOverride"), 0, 0, 0, 0);
        glUniform1f(glGetUniformLocation(progLine_, "uAlpha"), std::clamp(opt.gridOpacity, 0.f, 1.f));
        drawBatch(grid_, GL_LINES);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    // shaded surfaces
    if (opt.shaded) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.f, 1.f);
        if (opt.materials) {
            drawMaterials(V, P, opt);
        } else {
            glUseProgram(progMesh_);
            setMat(progMesh_, "uView", V);
            setMat(progMesh_, "uProj", P);
            glUniform1i(glGetUniformLocation(progMesh_, "uBackTint"), opt.backfaceTint ? 1 : 0);
            for (const MatGroup& mg : groups_) drawBatch(mg.batch, GL_TRIANGLES);
        }
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    // lines: wireframe, curves, normals
    glUseProgram(progLine_);
    setMat(progLine_, "uView", V);
    setMat(progLine_, "uProj", P);
    glUniform1f(glGetUniformLocation(progLine_, "uAlpha"), 1.f);
    glUniform1f(glGetUniformLocation(progLine_, "uDepthBias"), 0.00002f);
    GLint ovr = glGetUniformLocation(progLine_, "uOverride");
    if (opt.wireframe || !opt.shaded) {
        if (opt.shaded) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glUniform1f(glGetUniformLocation(progLine_, "uAlpha"), 0.55f);
            glUniform4f(ovr, 0, 0, 0, 0);
        } else {
            glUniform4f(ovr, 0.82f, 0.86f, 0.9f, 1.f);
        }
        drawBatch(wire_, GL_LINES);
        glDisable(GL_BLEND);
        glUniform1f(glGetUniformLocation(progLine_, "uAlpha"), 1.f);
    }
    glUniform4f(ovr, 0, 0, 0, 0);
    drawBatch(curves_, GL_LINES);
    if (opt.normals) drawBatch(normals_, GL_LINES);
    drawBatch(guides_, GL_LINES);

    // points: loose points always (world-sized when pscale exists), all points on request
    if (opt.points || loose_.count) {
        glEnable(GL_PROGRAM_POINT_SIZE);
        glUseProgram(progPoint_);
        setMat(progPoint_, "uView", V);
        setMat(progPoint_, "uProj", P);
        glUniform1f(glGetUniformLocation(progPoint_, "uPixelSize"), opt.pointSize);
        glUniform1f(glGetUniformLocation(progPoint_, "uViewportH"), float(h));
        glUniform1i(glGetUniformLocation(progPoint_, "uUseScale"), hasPscale_ ? 1 : 0);
        drawBatch(loose_, GL_POINTS);
        if (opt.points) {
            glUniform1i(glGetUniformLocation(progPoint_, "uUseScale"), 0);
            drawBatch(points_, GL_POINTS);
        }
        glDisable(GL_PROGRAM_POINT_SIZE);
    }
    glBindVertexArray(0);
    glUseProgram(0);

    // resolve MSAA
    glBindFramebuffer(GL_READ_FRAMEBUFFER, msFbo_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, resolveFbo_);
    glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    if (!volumes_.empty()) drawVolumes(w, h, cam);
    glBindFramebuffer(GL_FRAMEBUFFER, GLuint(prevFbo));
    glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);
    glDisable(GL_DEPTH_TEST);
    return resolveTex_;
}

void Renderer::drawVolumes(int w, int h, const Camera& cam) {
    glBindFramebuffer(GL_FRAMEBUFFER, volFbo_);
    glViewport(0, 0, w, h);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); // premultiplied
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT); // draw back faces so it also works with the camera inside the volume
    Mat4 V = cam.view(), P = cam.proj(float(w) / float(h));
    Vec3 eye = cam.eye(), fwd = normalize(cam.target - eye);
    glUseProgram(progVol_);
    setMat(progVol_, "uView", V);
    setMat(progVol_, "uProj", P);
    glUniform3f(glGetUniformLocation(progVol_, "uEye"), eye.x, eye.y, eye.z);
    glUniform3f(glGetUniformLocation(progVol_, "uFwd"), fwd.x, fwd.y, fwd.z);
    glUniform1f(glGetUniformLocation(progVol_, "uProjA"), P.at(2, 2));
    glUniform1f(glGetUniformLocation(progVol_, "uProjB"), P.at(2, 3));
    glUniform1i(glGetUniformLocation(progVol_, "uVol"), 0);
    glUniform1i(glGetUniformLocation(progVol_, "uDepth"), 1);
    glUniform2f(glGetUniformLocation(progVol_, "uViewport"), float(w), float(h));
    glActiveTexture(GL_TEXTURE0 + 1);
    glBindTexture(GL_TEXTURE_2D, depthTex_);
    glActiveTexture(GL_TEXTURE0);
    // far-to-near so overlapping volumes blend in order
    std::vector<const GpuVolume*> order;
    for (auto& v : volumes_) order.push_back(&v);
    std::sort(order.begin(), order.end(), [&](const GpuVolume* a, const GpuVolume* b) {
        return length((a->mn + a->mx) * 0.5f - eye) > length((b->mn + b->mx) * 0.5f - eye);
    });
    for (const GpuVolume* v : order) {
        glBindTexture(GL_TEXTURE_3D, v->tex);
        glUniform3f(glGetUniformLocation(progVol_, "uBoxMin"), v->mn.x, v->mn.y, v->mn.z);
        glUniform3f(glGetUniformLocation(progVol_, "uBoxMax"), v->mx.x, v->mx.y, v->mx.z);
        glUniform3f(glGetUniformLocation(progVol_, "uColor"), v->color.x, v->color.y, v->color.z);
        glUniform1f(glGetUniformLocation(progVol_, "uDensity"), v->density);
        glUniform1f(glGetUniformLocation(progVol_, "uFire"), v->fire);
        glUniform1f(glGetUniformLocation(progVol_, "uStep"), v->step);
        drawBatch(cube_, GL_TRIANGLES);
    }
    glBindTexture(GL_TEXTURE_3D, 0);
    glActiveTexture(GL_TEXTURE0 + 1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(0);
    glUseProgram(0);
    glCullFace(GL_BACK);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
}

void Renderer::readPixels(std::vector<uint8_t>& rgba, int& w, int& h) {
    w = fbW_; h = fbH_;
    rgba.assign(size_t(w) * size_t(h) * 4, 0);
    if (!resolveFbo_) return;
    GLint prev = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev);
    glBindFramebuffer(GL_FRAMEBUFFER, resolveFbo_);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
    glBindFramebuffer(GL_FRAMEBUFFER, GLuint(prev));
}

} // namespace hd

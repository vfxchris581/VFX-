// Shader networks (Houdini-style Material Builder with VOP-like shading nodes).
//
// A Material Builder node owns a network of Shading nodes. When it cooks, the network is compiled
// into a small register program (ShaderProgram) that the path tracer evaluates at every shading
// point, and that the Material Builder also evaluates at every point of its input to bake a
// viewport preview (Cd, roughness, metallic, ...). Prims get a "shader" attribute with the
// program's id.
//
// Values in a network are all 3-vectors; scalars are stored in every component. Colours are
// display (sRGB) colours, like Cd: the renderer converts the base colour to linear.
#pragma once
#include "Math.h"
#include <memory>
#include <string>
#include <vector>

namespace hd {
class Graph;
class Texture;
struct NodeType;

struct ShaderInputs {
    Vec3 P{0}, N{0, 1, 0}, uv{0}, Cd{0.72f, 0.74f, 0.78f};
    float time = 0, frame = 1;
    const Vec3* attribs = nullptr; // one value per ShaderProgram::attribs entry
};

struct ShaderOutputs {
    Vec3 base{0.8f};
    float rough = 0.5f, metal = 0, spec = 0.5f, transmit = 0, ior = 1.5f, sss = 0;
    Vec3 emitColor{1};  // display colour
    float emitStrength = 0;
    Vec3 N{0, 1, 0};    // shading normal after bump (== input N without bump)
};

class ShaderProgram {
public:
    struct Ins {
        int op = 0, dst = 0;
        int a = -1, b = -1, c = -1, d = -1, e = -1; // source registers
        int i0 = 0, i1 = 0;
        float f[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        Vec3 k{0};
        std::shared_ptr<Texture> tex;
    };
    std::vector<Ins> code;
    std::vector<Vec3> init; // initial register values (constants), size nregs
    int nregs = 0;
    std::string warnings;
    std::vector<std::string> attribs; // bound attribute names
    std::vector<Vec3> attribDefaults; // used where the geometry lacks the attribute
    // output registers (-1 = default)
    int oBase = -1, oRough = -1, oMetal = -1, oSpec = -1, oEmit = -1, oEmitStrength = -1, oTransmit = -1, oIor = -1, oSss = -1, oBump = -1;
    float bumpScale = 1;
    bool usesCd = false;   // reads the Cd global (renderers pass the colour from before the bake)
    std::string outputNode; // name of the output node (for messages)
    // Evaluates the network (with bump: three more evaluations to find the height gradient).
    void eval(const ShaderInputs& in, ShaderOutputs& out) const;
private:
    void run(const ShaderInputs& in, Vec3* regs) const;
};

// Compiles the network of a Material Builder (its subnet graph). Returns nullptr and sets error.
std::shared_ptr<ShaderProgram> compileShader(const Graph& net, float frame, float fps, std::string& error);

// Programs are kept by id (ids start at 1) for the renderers; prims carry the id in "shader".
int registerShader(std::shared_ptr<const ShaderProgram> p);
// Replaces the program behind an id (a Material Builder keeps its id across recooks).
void updateShader(int id, std::shared_ptr<const ShaderProgram> p);
std::shared_ptr<const ShaderProgram> shaderById(int id);

void registerShaderNodes(std::vector<NodeType>& types);

} // namespace hd

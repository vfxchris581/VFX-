// OpenUSD interchange through the text format (.usda), without the USD library.
//
// Export writes one stage: /World/geo (a Mesh for polygons with GeomSubsets per material, BasisCurves for
// open curves, Points for loose points) with normals, displayColor and st primvars, UsdPreviewSurface
// materials (with UsdUVTexture for texture maps) under /World/Looks, time-sampled points for animation,
// and the scene's cameras and lights (UsdGeomCamera, UsdLux). Any USD-based tool can open the result.
//
// Import reads Mesh, Points and BasisCurves prims (points, topology, normals, displayColor, st; the first
// time sample when animated), applying their xformOps. It does not resolve references, payloads,
// variants or sublayers: export flattened stages from other tools to bring them in.
#pragma once
#include "Geometry.h"
#include "Lights.h"
#include <functional>
#include <string>
#include <vector>

namespace hd {

struct UsdExport {
    int start = 1, end = 1;  // frames; start == end writes a single, unanimated shape
    float fps = 24;
    std::vector<SceneCamera> cameras;
    std::vector<Light> lights;
};

// cook(frame, geo, err) supplies the geometry at each frame.
bool writeUsda(const std::string& path, const UsdExport& opts,
               const std::function<bool(float, Geometry&, std::string*)>& cook, std::string* error = nullptr);

bool loadUsda(const std::string& path, Geometry& out, std::string* error = nullptr);

} // namespace hd

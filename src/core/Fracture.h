// Geometry helpers for destruction: convex hulls, polygon triangulation, mass properties
// and Voronoi fracturing. Shared by the Voronoi Fracture SOP and the RBD Solver.
#pragma once
#include "Geometry.h"
#include <vector>

namespace hd {

// Convex hull of a point set as outward-facing triangles.
struct ConvexHull {
    std::vector<Vec3> pts;               // hull vertices
    std::vector<int> tris;               // 3 indices per triangle, counter-clockwise seen from outside
    struct Plane { Vec3 n; float d; };   // dot(n, x) <= d is inside
    std::vector<Plane> planes;           // de-duplicated face planes
    std::vector<Vec3> edgeMids;          // midpoints of hull edges (extra contact samples)
    bool valid() const { return tris.size() >= 12; }
    // Signed distance (positive outside) and the face plane that achieves it.
    float distance(const Vec3& p, int* plane = nullptr) const;
    // Like distance(), but when p is inside, the reported plane is the face the point crossed coming
    // from prev (the deepest plane prev was outside of), not merely the nearest face.
    float distanceFrom(const Vec3& p, const Vec3& prev, int* plane) const;
};

// Builds the hull. Flat or collinear inputs are thickened by `minThickness`
// so they still enclose a volume. Returns false if there are fewer than one point.
bool buildConvexHull(const std::vector<Vec3>& points, ConvexHull& out, float minThickness = 1e-3f);

// Mass properties of a closed, outward-oriented triangle mesh with unit density.
struct MassProps {
    double volume = 0;
    Vec3 com{0, 0, 0};
    double inertia[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // about the center of mass
};
MassProps computeMassProps(const std::vector<Vec3>& pts, const std::vector<int>& tris);

// Triangulates a planar polygon (outer loop plus optional holes) given as 3D points on a
// plane with normal n. Loops are lists of indices into pts. Outer loops must wind
// counter-clockwise around n and holes clockwise. Appends triangles (3 indices each).
// Returns false if ear clipping had to fall back to a fan.
bool triangulatePolygon(const std::vector<Vec3>& pts, const std::vector<std::vector<int>>& loops, const Vec3& n,
                        std::vector<int>& trisOut);

struct FractureOptions {
    Vec3 insideColor{0.85f, 0.35f, 0.15f};
    bool colorInside = true;
    int threads = 0; // 0 = hardware concurrency
};

struct FractureStats {
    int pieces = 0;
    int emptyCells = 0;
    int capFallbacks = 0; // caps that could not be ear-clipped cleanly
};

// Cuts the closed polygon mesh `in` into one piece per site (Voronoi cell clipped to the mesh).
// Output pieces don't share points. Adds prim attributes "piece" and "inside" (1 on cut faces)
// and point attribute "piece". Point attributes are interpolated onto new points; "N" is removed.
Geometry voronoiFracture(const Geometry& in, const std::vector<Vec3>& sites, const FractureOptions& opt,
                         FractureStats* stats = nullptr);

// Even-odd ray test against closed polygons.
bool pointInsideMesh(const Geometry& g, const Vec3& p);

} // namespace hd

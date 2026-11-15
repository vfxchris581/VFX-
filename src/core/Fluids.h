// Fluid helpers shared by the fluid nodes and the tests.
#pragma once
#include "Geometry.h"
#include <string>
#include <vector>

namespace hd {

// Closed surface around particles: union-of-spheres distance field (radius = separation * radiusScale),
// smoothed `smooth` times, meshed by marching tetrahedra with voxel size separation * voxelScale.
// Returns false (and sets err) if the grid would be too large.
bool particleFluidSurface(const Geometry& particles, float separation, float radiusScale, float voxelScale, int smooth,
                          Geometry& out, std::string* err = nullptr);

// Runs one frame (dt) of FLIP on particles in an n^3 container; returns the largest divergence (1/s)
// left in a fluid cell after the pressure projection.
float flipTestStep(std::vector<Vec3>& P, std::vector<Vec3>& V, Vec3 origin, int n, float dx, float dt, Vec3 gravity, int* substeps);

} // namespace hd

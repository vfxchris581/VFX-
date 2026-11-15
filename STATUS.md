# Status against the product spec

The goal is a procedural 3D and VFX application at the level of SideFX Houdini. That is many years of work for a large team, so it's being built in phases. This file says plainly what works today, what's partial, and what hasn't been started. Anything listed as **Working** is covered by the automated tests (`ctest`) or was checked by rendering it.

Legend: **Working**: usable and tested. **Partial**: works, with the limits noted. **Planned**: not started. Menu items and shelf buttons for planned features are greyed out, not dead.

## Phase 1: architecture, interface, viewport, camera, node editor

| Area | Status | Notes |
|---|---|---|
| Engine / UI split | Working | `hd_core` (no UI deps), `hd_render` (OpenGL 3.3), `hd_app` (Dear ImGui), `hdc` headless CLI |
| Node graph with cook caching and time dependence | Working | Recooks only what changed |
| Network editor, parameter pane, spreadsheet, timeline | Working | Tab menu, wiring, display/bypass flags, subnets, assets |
| Houdini-style menus and shelf | Partial | File, Edit, Render, Assets, Windows, Help; shelf tabs Create, Modify, Lights and Cameras, Particles, Pyro FX, Rigid Bodies, Fluids, Vellum. No separate Modeling / Rigging / Solaris / Compositing menus yet |
| Dockable panes | Partial | Panes can be shown, hidden and reset; free docking and multi-monitor windows are not done |
| Viewport | Working | Perspective, MSAA, wireframe, points, normals, ground grid, Display Options |
| Orthographic views | Planned | |
| Cameras | Partial | Camera nodes, look-through, lock-to-view, keyframed cameras. No focal length/sensor model, DOF, motion blur or lens distortion yet |
| Camera tracking | Planned | |
| Toolkit choice | Note | The spec suggests Qt. The UI is Dear ImGui today. Moving to Qt is possible because the engine has no UI dependencies, but it hasn't been started |
| GPU APIs | Note | OpenGL 3.3 only. Vulkan / Metal / DirectX 12 are planned |

## Phase 2: procedural geometry and modeling

| Area | Status | Notes |
|---|---|---|
| Polygon generators and modifiers | Working | Box, Sphere, Grid, Tube, Torus, Circle, Line, File (OBJ), Transform, Merge, PolyExtrude, Fuse, Facet, Reverse, Blast, PolyWire |
| Subdivision surfaces | Working | Catmull-Clark |
| Deformers | Working | Mountain, Smooth, Peak, Point Jitter |
| Attributes, groups, wrangles | Working | VEX-lite snippet language; float and vector attributes (no string attributes yet) |
| Scatter, Copy to Points | Working | Orients to N/up, scales by pscale/scale |
| Fracturing | Working | **Voronoi Fracture**. See Phase 3 |
| Booleans, remeshing, retopology, UV tools, sculpting, NURBS, procedural architecture, terrain heightfields | Planned | |
| VDB workflows | Planned | Volumes are dense grids, not OpenVDB |
| Packed primitives / instancing | Planned | Copy to Points makes real copies |

## Phase 3: pyro, particles, destruction

| Area | Status | Notes |
|---|---|---|
| POP particles | Working | Emission, gravity, wind, drag, curl-noise turbulence, ground/sphere collisions, color and size over life, POP wrangle, per-frame cache, Trail |
| Particle collisions with arbitrary meshes, flocking, particle-particle forces | Planned | |
| Pyro smoke and fire | Partial | Grid fluid with buoyancy, vorticity, turbulence, dissipation, cooling, collider mesh, fire emission, cached. Dense grids only; no combustion/fuel model, gas expansion, shockwaves or sparse grids |
| Voronoi Fracture | Working | Clips any closed mesh into Voronoi cells around points (input 2) or random interior points. Caps are triangulated with holes, so non-convex shapes (e.g. a torus) give closed pieces. Inside faces get `inside=1` and a color. Multithreaded. Tests check that every piece is closed, faces outward and that volume is conserved |
| RBD Solver | Working | Rigid bodies with mass and inertia from the piece volume, convex-hull contacts, ground plane, static collider input, friction, bounce, sleeping. Glue between touching pieces with impact strength and propagation, so pieces break apart on impact. `v` sets initial velocity per piece and `active=0` pins a piece. Cached per frame. Tests check free fall against the analytic formula, settling, determinism, glue holding and breaking |
| RBD limits | Note | Each piece collides as its **convex hull**, so concave pieces collide as if filled in. Contacts are vertex/edge-midpoint against hull, so very thin pieces can miss edge-on hits at high speed. No hinge/spring constraints, no Bullet integration, no debris emission yet |

## Phase 4: fluids, Vellum, advanced physics

| Area | Status | Notes |
|---|---|---|
| FLIP liquids | Working | MAC grid, PIC/FLIP blend, MIC(0)-preconditioned CG pressure solve with free surface and solid walls, implicit viscosity, RK2 advection, CFL substeps, static collider, open top, continuous emission, per-frame cache. Particle separation keeps volume: a dam break settles at 0.308 m against an expected 0.31 m, where it falls to 0.16 m without separation. Tests check divergence after projection, particle count, container and collider bounds, settled depth and determinism |
| FLIP limits | Note | Dense grid (no sparse or narrow-band), CPU only, static colliders, no surface tension, no whitewater (foam, spray, bubbles) yet |
| Fluid meshing | Working | Particle Fluid Surface: smoothed union-of-spheres distance field, marching tetrahedra (always closed). A sphere of particles meshes to within 1% of its volume |
| Ocean | Partial | Wind-driven directional spectrum (Phillips shape, scaled to Pierson-Moskowitz wave height) as Gerstner waves with choppiness, exact normals, velocity and a Jacobian foam mask. Tested: wave height matches the wind, normals match the surface. It isn't an FFT ocean and doesn't tile; no ocean-to-FLIP blending |
| Vellum cloth | Working | XPBD stretch + bend (across shared edges), pins (attribute or group, may be animated), ground, sphere, convex collider parts (may be animated), friction, inelastic contacts, point self-collision. Tested: pins hold, under 1% stretch, free fall matches the formula, drapes without entering the collider |
| Vellum ropes / hair | Working | Open curves: stretch + bend over two segments. Tested: a pinned rope swings down and keeps its length |
| Vellum soft bodies | Partial | Pressure (volume) constraint on closed meshes keeps them inflated (tested within 5%). No tetrahedral solids yet |
| Vellum limits | Note | Self collision is point to point (no triangle tests), so fine cloth can still pass through itself. Colliders are convex hulls per part. No grains, fur grooming, welds/tearing, or Bullet |

## Phase 5: materials, lighting, rendering

| Area | Status | Notes |
|---|---|---|
| Path-traced Render View (CPU, progressive) | Working | GI, glossy/metallic, emissive, volumes with shadows, sun/sky |
| Lights | Working | Point, Spot, Area, Distant, Environment, Sky, Ambient |
| Material node | Partial | Roughness, metallic, glow. No node-based shader networks, textures, SSS, glass/refraction, displacement or UDIMs |
| Image output | Partial | PNG (uncompressed). No OpenEXR, AOVs, Cryptomatte or denoising |
| GPU rendering | Planned | |

## Phase 6: USD, animation, compositing

| Area | Status | Notes |
|---|---|---|
| Keyframes, curves, expressions | Working | Every numeric parameter; Animation Editor |
| Rigging, IK/FK, skinning, blend shapes, mocap, crowds | Planned | |
| OpenUSD / Solaris | Planned | |
| COP compositing | Planned | |
| Python scripting | Planned | VEX-lite and parameter expressions exist |

## Phase 7: testing and stability

| Area | Status |
|---|---|
| Automated core tests | Working: about 900,000 checks covering geometry, the node graph, every solver, the renderer and every example scene |
| CI on macOS, Windows and Linux | Working: every push builds, tests and packages |

## Phase 8: installers

| Platform | Status |
|---|---|
| macOS | Working: universal (Apple Silicon + Intel) `HD-macOS.dmg` with an Applications shortcut. Not code-signed or notarized |
| Windows 10/11 | Working: `HD-Windows-Setup.exe` (NSIS, 64-bit) with Start menu and desktop shortcuts, `.hdg` file association and an uninstaller, plus a portable zip. Not code-signed, so SmartScreen will warn |
| Linux | Working: tarball |

# Status against the product spec

The goal is a procedural 3D and VFX application at the level of SideFX Houdini. That is many years of work for a large team, so it's being built in phases. This file says plainly what works today, what's partial, and what hasn't been started. Anything listed as **Working** is covered by the automated tests (`ctest`) or was checked by rendering it.

Legend: **Working**: usable and tested. **Partial**: works, with the limits noted. **Planned**: not started. Menu items and shelf buttons for planned features are greyed out, not dead.

## Phase 1: architecture, interface, viewport, camera, node editor

| Area | Status | Notes |
|---|---|---|
| Engine / UI split | Working | `hd_core` (no UI deps), `hd_render` (OpenGL 3.3), `hd_app` (Dear ImGui), `hdc` headless CLI |
| Node graph with cook caching and time dependence | Working | Recooks only what changed |
| Network editor, parameter pane, spreadsheet, timeline | Working | Tab menu, wiring, display/bypass flags, subnets, assets |
| Houdini-style menus and shelf | Partial | File, Edit, Render, Assets, Windows, Help; shelf tabs Create, Modify, Lights and Cameras, Particles, Pyro FX, Rigid Bodies, Fluids, Vellum, Rigging, Compositing. No separate Modeling / Rigging / Solaris / Compositing menus yet |
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
| Path tracing (CPU, progressive) | Working | GI, next-event estimation for every light, Russian roulette, volumes with shadows |
| Principled material | Working | Lambert under a GGX coat, GGX metal, rough GGX dielectric (exact Fresnel, Walter 2007 transmission, Heitz 2018 visible-normal sampling), smooth limits handled as perfect mirrors/refraction. White-furnace tests (glass, rough glass, mirror and rough metal, plastic, subsurface) check that energy is conserved |
| Glass / water | Working | Transmission + IOR on the Material node. Shadows through glass are tinted, not caustic (refraction is ignored on shadow rays) |
| Subsurface scattering | Working | Random walk inside closed meshes, per-channel scatter distance, albedo inversion (Chiang 2016). Open meshes lose the light that leaves through holes |
| Texture maps, UDIM | Working | Base color, roughness, metallic, normal maps via point `uv`; PNG (all standard formats except interlaced), OpenEXR (scanline NONE/ZIPS/ZIP), Radiance HDR. JPEG and TIFF aren't read. Bilinear filtering, no mipmaps (distant textures can shimmer) |
| UV Project | Working | Planar, box, cylindrical, spherical. No UV unwrapping, layout or editor |
| Procedural textures | Partial | Through Attribute Wrangle (noise, etc.) into `Cd`; no texture nodes in the material |
| Node-based shader networks (VOPs) | Planned | |
| Displacement | Planned | Normal maps only |
| Lights | Working | Point, spot, area, distant, sky, constant and HDRI environment (importance sampled + MIS). No light linking, IES profiles or geometry lights |
| Depth of field | Working | Thin lens (f-stop, focus distance; camera nodes or Render Settings). Tested: out-of-focus edges widen, in-focus ones don't |
| Motion blur | Working | Linear, from point `v`, centred shutter. Tested: the streak matches v x shutter. No deformation blur from sub-frame geometry, no camera motion blur |
| Denoising | Working | Edge-avoiding a-trous (Dammertz 2010) with variance guidance, demodulated by albedo. Tested: error vs a 256-sample reference more than halves at 4 samples. It isn't Intel Open Image Denoise and leaves blotches at very low sample counts |
| Render passes (AOVs) | Partial | Beauty, alpha, albedo, normal, depth (and denoised). No per-light, diffuse/specular split or Cryptomatte yet |
| OpenEXR output | Working | Half float, ZIP compression, all passes in one file. Single-part scanline only |
| PNG output | Working | Compressed (own zlib) |
| GPU rendering | Planned | |
| Render layers | Planned | |

## Phase 6: USD, animation, compositing

| Area | Status | Notes |
|---|---|---|
| Keyframes, curves, expressions | Working | Every numeric parameter; Animation Editor |
| Skeletons, FK | Working | Joint chains or bones from curves; local rotations from parameters or a `rot` attribute. Tested against hand-computed poses |
| IK | Working | FABRIK chains with a pole vector and blend. Tested: reaches the target, keeps bone lengths, bends toward the pole, straightens toward unreachable targets |
| Skinning | Working | Automatic distance-based weights (4 influences) and linear blend skinning. Tested: rigid moves are exact, bends move only the right part. No weight painting, dual-quaternion skinning or heat-diffusion weights |
| Blend shapes | Working | Up to four weighted targets |
| Motion capture | Working | BVH import (any channel order), interpolated and optionally looped. Tested against hand-computed joint positions |
| Retargeting | Partial | Between skeletons with the same joints in the same order (rotations copied, root travel scaled). No name-based joint mapping between different hierarchies |
| Time Shift | Working | |
| Crowds | Planned | |
| Node-based compositing | Working | 15 COP nodes: render, file (sequences, EXR Z), constant/ramp/checker, color correct, blur, glow, merge (10 modes), Z composite, chroma key + despill, shape mask (roto), transform, lens effects, defocus, resize, write. Premultiplied linear RGBA. Tests check the maths of each |
| Rotoscoping | Partial | Masks from animated curves (anti-aliased, feathered). No interactive shape editing in the viewer |
| Tracking | Planned | No 2D or camera tracking |
| Image viewing | Partial | Images show in the Scene View (sRGB). No channel/AOV switching, zoom/pan or pixel readout yet |
| USD export | Working | .usda: mesh (normals, displayColor, st), GeomSubsets per material, UsdPreviewSurface + UsdUVTexture, curves, points, cameras, UsdLux lights (HDRI dome), time-sampled animation. Written without the USD library and checked by round-tripping (exact positions); not yet validated with usdchecker |
| USD import | Partial | .usda meshes, curves and points with xformOps, colours, normals, uvs (first time sample). No binary .usdc, references, payloads, variants or sublayers |
| LOPs / Solaris (stage editing, layers, variants) | Planned | |
| Python scripting | Planned | VEX-lite and parameter expressions exist |

## Phase 7: testing and stability

| Area | Status |
|---|---|
| Automated core tests | Working: about 900,000 checks covering geometry, the node graph, every solver, the renderer and every example scene |
| CI on macOS, Windows and Linux | Working: every push builds, tests and packages |
| Sanitizers | Working: a CI job builds with AddressSanitizer + UndefinedBehaviorSanitizer and runs the full test suite and the fuzzer. Local run: all ~900,000 checks clean |
| Fuzzing | Working: `hd_fuzz` mutates valid inputs for zlib, PNG, EXR, HDR (flat + RLE), BVH, USDA, OBJ, scene JSON, expressions and VEX. 280,000 inputs under ASan/UBSan found and fixed: a 51 GB allocation from a corrupt PNG header, unbounded inflate output, and signed overflows in EXR data windows. Decoders now cap dimensions (65,536) and pixels (2^28) |
| App smoke test | Working: `hd_ui_smoke` runs the real application headless (GLFW null platform + Mesa EGL), opens all 18 example scenes, plays frames, opens Render Settings, Render View and the comp viewer, stress-tests undo/redo, fails on any node error or OpenGL error, and saves screenshots (uploaded as a CI artifact) |
| Benchmarks | Working: `hdc bench` times every major subsystem |
| Performance | Improved, see below |
| Viewport material preview | Working (after 0.7.0): the OpenGL viewport shades roughness, metal, specular, emission, SSS (wrap lighting) and glass (tint + reflections, no refraction) with GGX under a fixed studio rig, and shows base colour, roughness, metallic and normal maps including UDIM tiles (textures are cut to at most 2048 px). Scene lights are not used in the viewport, and overlapping glass is not depth-sorted. "Mat" in the viewport toolbar switches back to clay shading; missing maps are listed in the viewport |
| Memory-leak / long-session soak tests, GPU profiling | Planned |

### Measured speedups (`hdc bench`, same machine, before vs after Phase 7)

| Workload | Before | After | Change |
|---|---|---|---|
| FLIP dam break, 40 frames, 92k particles | 12,066 ms | 5,775 ms | 2.1x faster (parallel particle-to-grid, sorted-grid particle separation) |
| Fluid surface (includes the FLIP cook) | 6,397 ms | 3,319 ms | 1.9x |
| Vellum cloth, 72 frames | 5,054 ms | 3,336 ms | 1.5x (sorted-bucket self-collision) |
| Render, materials scene | 2,000 ms | 1,857 ms | 7% (binned SAH BVH, near-child-first traversal) |
| Render, 500k triangles | 1,162 ms | 1,145 ms | ~same (time is dominated by geometry generation) |
| Pyro smoke, 60 frames | 4,453 ms | ~3,500 ms | ~1.25x after Phase 7 (branch-free pressure solve; 35% fewer instructions, results bit-identical) |
| All workloads | 33,553 ms | 22,109 ms | 34% less time |

## Phase 8: installers and releases

| Item | Status |
|---|---|
| macOS | Working: universal (Apple Silicon + Intel) `HD-macOS.dmg`, macOS 11+. App icon, `.hdg` document type (Finder double-click / Open With / Dock drop open the scene), OBJ/PLY/USDA/BVH as alternate types, ad-hoc signature, "Read Me First" with Gatekeeper steps. CI mounts the image and checks Info.plist, icon, both architectures, signature and that HD and `hdc` run. The Finder open-file path is compiled in CI but has not been tried on a real Mac yet |
| Windows 10/11 | Working: `HD-Windows-Setup.exe` (NSIS, 64-bit) with HD icon, Start menu, desktop and Example Scenes shortcuts, `.hdg` association, Apps & Features entry with size, silent mode (`/S /D=path`) and an uninstaller, plus a portable zip. `HD.exe`/`hdc.exe` carry icon and version resources. CI installs silently, checks files, registry, association and version, runs `hdc`, uninstalls and checks everything is removed |
| Linux | Working: `HD-Linux.tar.gz` with `install.sh` (to `~/.local` or any prefix): desktop entry, icon, `.hdg` MIME type, uninstaller. CI installs, validates the desktop file, runs HD/`hdc` and uninstalls. Built on Ubuntu 24.04, so needs glibc 2.39 or newer |
| Version info | Working: Help > About, `HD --version`, `hdc --version` (version, commit, platform, build date) |
| GitHub Releases | Working: pushing a tag `vX.Y.Z` (must match the CMake version) runs every job and, if all pass, publishes one release with the four packages, `SHA256SUMS.txt` and notes from CHANGELOG.md |
| Code signing / notarization | Not done: needs the owner's Apple Developer ID certificate and a Windows code-signing certificate (paid, issued to a person or company). Until then macOS asks once on first launch and Windows SmartScreen may warn |
| Auto-update, AppImage/Flatpak, Linux ARM, Windows ARM | Planned |

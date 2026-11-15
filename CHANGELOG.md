# Changelog

Version numbers follow the build phases. Each release lists what changed; STATUS.md lists
what works, what is partial and what is planned.

## 0.7.0

Phases 8 to 11: installers and releases, packed primitives and instancing, rigid-body constraints, Python scripting, plus new simulation and viewport features.

### Python scripting (Phase 11)

- HD uses the Python 3.8+ installed on the computer (loaded at runtime through the stable ABI; nothing to install with HD). `hdc python --info` shows which.
- `hd` module for nodes, parameters, expressions, keyframes, wiring, cooking, frames, files and geometry.
- Python Shell pane, File → Run Python Script, `hdc python`, and a Python SOP node.
- Example scripts in `examples/scripts`.

### Rigid-body constraints and packed pieces (Phase 10)

- RBD Solver constraint input: pin (ball joint), hinge (with motor) and spring constraints, breakable by force; pieces joined by a pin or hinge don't collide.
- New RBD Constraints node: nearby pieces, chains, world anchors. "Show Constraints" draws them on the solver output.
- Packed prims simulate as RBD pieces and stay packed.
- New example: `wreckingball.hdg`.

### Packed primitives and instancing (Phase 9)

- Packed primitives: shared, transformable instances of geometry (nested packing allowed). Pack (whole input or one per piece) and Unpack nodes; Copy to Points has a "Pack and Instance" option.
- Path tracer renders instances with a two-level BVH: 100,000 instances of a 4,900-polygon sphere (about 490M polygons) in about 2 s and about 70 MB.
- Viewport draws instances with GPU instancing in clay and material shading.
- OBJ, PLY and USD exports unpack instances. New example: `forest.hdg` (12,000 trees). New bench workload.

### Installers, viewport and simulation (Phase 8)

- Whitewater Solver: spray, foam and bubbles from FLIP (trapped air, wave crests, kinetic energy), added to the dam-break example.
- Viewport material preview: roughness, metal, emission, SSS and glass from Material nodes, plus base colour, roughness, metallic and normal maps (UDIM too). Toggle with "Mat" in the viewport toolbar or Display Options; textures that fail to load are listed in the viewport.
- Vellum: optional point-triangle self collision (Self Collision (triangles) on the solver). A sheet dropped onto another now stacks on it instead of falling through between its points; contacts are inelastic.
- Pyro: about 1.25x faster with bit-identical results (branch-free interior pressure solve).
- App icon (original artwork) for macOS, Windows, Linux and the window title bar.
- Version shown in Help > About, `HD --version` and `hdc --version`, with the git commit.
- macOS: `.hdg` files are registered to HD and open from Finder (double-click, Open With, drop on the Dock icon); OBJ, PLY, USDA and BVH can be opened with HD. Disk image is ad-hoc signed and contains first-launch instructions.
- Windows: icon and version information in `HD.exe` and `hdc.exe`; installer has the HD icon, an Example Scenes shortcut, installed size in Apps & Features, and refreshes Explorer after registering `.hdg`.
- Linux: `install.sh` installs to `~/.local` (or any prefix) with a desktop entry, icon, `.hdg` MIME type and an uninstaller.
- The app now finds its example scenes when started through a symlink or from PATH.
- UI fonts include dashes, quotes, bullets and arrows (they showed as "?" before).
- CI installs and uninstalls each package (DMG, NSIS installer, tarball) and checks versions, file associations and that `hdc` runs from the installed copy. Tags publish a GitHub Release with SHA-256 checksums.

## 0.6.0

Phase 7: optimization, testing and stability. `hdc bench`; AddressSanitizer/UBSan CI job; a mutation fuzzer for every file parser (fixed a 51 GB allocation from a corrupt PNG and signed overflows in EXR); a headless smoke test of the real app; FLIP 2.1x and Vellum 1.5x faster.

## 0.5.0

Phase 6: rigging and animation (skeletons, FK, IK, skinning, blend shapes, BVH import, retargeting), compositing (15 COP nodes), USD export and import.

## 0.4.0

Phase 5: principled BSDF materials, textures and UDIMs, HDRI lighting, depth of field, motion blur, EXR with AOVs, denoising.

## 0.3.0

Phase 4: FLIP liquids, fluid surfacing, ocean and Vellum cloth/soft bodies.

## 0.2.0

Phase 3: Voronoi fracture and rigid bodies; Windows installer.

## 0.1.0

Phases 1-2, imported from vfxchris581/hd: node graph, viewport, procedural modeling, particles, pyro.

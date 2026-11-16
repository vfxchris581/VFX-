# Changelog

Version numbers follow the build phases. Each release lists what changed; STATUS.md lists
what works, what is partial and what is planned.

## 0.7.0

Phases 8 to 13: installers and releases, packed primitives and instancing, rigid-body constraints, Python scripting, node-based shaders, modelling tools, plus new simulation and viewport features.

### Sequence Player

- A Sequence Player tab next to the Render View plays rendered image sequences inside the app, like Houdini's MPlay. Open it with Render > Sequence Player, the Render View's "Play Sequence..." button, or by dropping an image onto the window; pick any frame and it finds the rest (`name.0001.png`, `.exr`, `.hdr`, gaps allowed).
- Play forward or backward at any frame rate, loop, play once or ping-pong, step frame by frame, scrub, jump to a frame. Frames load in the background ahead of the playhead (up to 1.5 GB), and playback waits for frames still loading instead of skipping them; the scrub bar shows what is loaded.
- Zoom and pan (wheel, drag, double-click to fit, 1:1), view one channel or alpha, exposure for EXR / HDR, the pixel value under the mouse. Keys: Space play/pause, arrows step, Home/End, F fit.
- A finished Render Sequence is loaded into the player automatically.

### Fixes

- Lights and Cameras shelf: a new light now appears inside the current view (where the scene's preferred spot is out of view, lights line up across the top of the frame), and the Camera tool shows the new camera in the scene instead of silently looking through it (look through it from the viewport's camera menu). The network view pans to every node made from the shelf, which used to land outside the visible area.

### Movies (MP4 and MOV)

- Save a sequence as an H.264 movie in `.mp4` or QuickTime `.mov`: "Save Movie..." in the Sequence Player (runs in the background, with progress and Cancel), or name the Render Sequence output `.mp4` / `.mov` to render straight to a movie (the frames are also kept as PNGs in a `_frames` folder so the Sequence Player can play them). Quality: High, Medium or Small.
- `hdc movie <frame> out.mp4 [--fps N] [--quality high|medium|small]` makes a movie from rendered frames, and `hdc render scene.hdg out.mov --frames A B` renders straight to a movie.
- The encoder is built in (no ffmpeg or other libraries needed): Constrained Baseline H.264 with motion-compensated P frames, skipped macroblocks, the deblocking filter, and slices encoded in parallel (720p at about 20 frames per second). BT.709 colour, square pixels and the frame rate are tagged, so players show the right colours and speed. CI decodes the movies with ffmpeg and requires the pictures to match the encoder's bit for bit.

### Modelling tools (Phase 13)

- Boolean node: union, intersect, subtract and shatter of closed meshes. Faces lying in the same plane are handled, attributes carry over (colours, uvs, materials), uncut polygons stay whole, and the result is welded with no T-junction cracks. About 0.6 s for an 80,000-polygon sphere against a 20,000-polygon torus.
- Clip node: cut geometry with a plane, keeping one side or both.
- Orthographic viewport views: Top, Front, Right, Bottom, Back, Left from the viewport's view menu or keys 1 to 7. The grid turns to face side views; tumbling returns to perspective.
- New example: `boolean.hdg` (a die and a drilled, slotted bracket).

### Shader networks (Phase 12)

- Material Builder node: a surface shader built from Shading nodes, like Houdini's material builder with VOPs. Double-click (or "Edit Shader Network") to edit it; inside, the Tab menu and the Shading shelf offer only shading nodes and the viewport keeps showing the scene.
- 21 Shading nodes: Shader Output (principled: base colour, roughness, metallic, specular, emission, transmission, IOR, subsurface, bump), Global Variables (P, N, uv, Cd), Time, Constant Color/Float, Bind Attribute, Math, Function, Mix, Fit Range, Clamp, Smooth Step, Noise (Perlin, fBm, turbulence, ridged, vector), Voronoi (distance, edges, cell value/colour), Checker, Color Ramp, Texture (PNG/EXR/HDR, UDIM), Combine XYZ, Separate, Random, Wave (bands, rings; sine, saw, triangle; noise distortion).
- The Render View runs the network at every shading point, including packed instances, shadows through shader glass and bump mapping. The viewport shows it baked onto the points.
- New example: `shaders.hdg` (marble, wood, rusted metal, tiles, glowing lava).

### Modelling aids

- Ground Plane node: a checker floor that is always shown in the viewport and renders, whichever node is displayed (Create shelf).
- Reference grid redone like Houdini's: endless, crisp on Retina screens, fading with distance and zoom, optional ruler numbers on the axes.

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

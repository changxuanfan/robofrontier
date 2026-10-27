# Robo Factory

Current implementation (2026-10-25):
[R10 sources, V6 factory, offline replay and bounded parallel Oracle batch](../plans/suite-v2/implementation-r10.md).
The historical [launch plan](../plans/suite-v2/factory-oracle-next.md) is retained.
Current V6 scenes are generated under `outputs/robo-factory/v6-workstations/`;
full 36-task Oracle success has not yet been established.

The Cycles adapter also accepts an actual recorded pose through
`scripts/export_factory_recorded_pose.py` followed by
`scripts/render_factory_blender.py --recorded-pose ... --views recorded_task`.
The pose is derived from source FK and the fixed factory placement; no commands
or physics are replayed. A real120-second dial frame moved244 geometries from
the initial factory pose. Readback errors were below1 micrometre in position
and5e-7 in rotation-matrix components. The example and audit are under
`outputs/robo-factory/v6-recorded-dial/`. This is a single recorded-state render,
not certification of a complete animation or Isaac dynamics.

## Scope

Build a static, enclosed MuJoCo factory background on macOS, with four adjustable
experiment regions and a continuous perimeter equipment strip. Task tables,
robots, fixtures and individual workstation outlines are supplied by other work
and are excluded from this build. Read this document when changing factory
assets, layout, rendering, cameras or future workstation/replay integration.

The approved industrial reference is `robo-factory-references/industrial-style.png`.
Its architecture, finishes and lighting are references; its uniform workstations
and image-generated geometry are not dimensional requirements. The earlier
concepts in `outputs/robo-factory/concepts/` remain exploratory references.

## Construction sequence

1. Prove the renderer can produce a material/lighting sample on the current Mac.
2. Build the shell, adjustable area markings, structural framing and openings.
3. Add repeated lighting, cable trays, electrical equipment and wall-side storage.
4. Inspect actual renders at interior, close, overhead and four exterior cutaway views.
5. Measure frame time, memory, asset sizes and scene complexity; preserve the results.
6. Deliver a reproducible build, camera controls and a browsable factory view.

## Scene contract

- Metres; right-handed coordinates; +X east, +Y north, +Z up; floor at Z=0.
- Four complete walls and a roof exist in the model. Cutaway visibility affects
  presentation only and is recorded with each camera. Interior presets keep the
  enclosure visible. Region names are painted onto real perimeter walls.
- North wall: TRANSPORT and MOSAIC; south wall: AIR RELAY and DIAL LAB.
- Four region outlines are adjustable; their extents do not freeze future table sizes.
- Perimeter props fit their service strips and remain outside the four reserved regions.
- Geometry and materials use `factory_` names and descriptive body identifiers.
- Display collision/density defaults are scoped to `factory_visual`. Future
  composition imports background assets, bodies and that named class, preserving
  the target task's compiler, solver options, actuators and camera contract.
- This background is display geometry, with contact filtering disabled. It adds no
  actuators or dynamic joints. Future physical collision geometry is an explicit
  extension, not an incidental effect of changing visual assets.
- A workstation is eventually placed using a rigid transform at its own root frame;
  its internal dimensions, mounting arrangement and recorded poses retain their scale.
- Replay records bind the exact source scene and assets. Different task revisions
  are separate inputs. Preserve source geometry, local transforms, names, appearance
  bindings and physics metadata for later conversion; USD/Isaac compatibility is
  unverified until an actual conversion is exercised.

## Asset and rendering policy

Source settings live in `configs/backgrounds/robo_factory.json`. Reusable assets
live in `assets/robo_factory/`; generated scenes, views and reports live in
`outputs/robo-factory/`. A build includes a local copy of its referenced assets,
source configuration, camera definitions, semantic manifest and hashes.

Use chamfered silhouettes where visible, repeated models and materials, and tiled
1K/2K textures. Keep functional industrial connections and realistic material scale.
The initial background targets 300k mesh triangles and 256 MiB decoded texture
data; primitive tessellation and renderer overhead are reported separately where
known. These are build budgets, not measured performance guarantees.

The viewer updates static scenes when camera/settings change. Render/export a
chosen view on demand instead of continuously producing every camera. Keep
lighting, material quality and exposure configurable. Preserve classic OpenGL as
an explicitly selected comparison path while checking Filament availability.

## Initial renderer findings

MuJoCo 3.16.0 is installed. Default Filament context selected Vulkan and aborted on
this Mac. An explicit OpenGL context using the bundled Studio headless window
worked on Apple M5 Pro (OpenGL 4.1 via Metal). This is a rendering backend result,
not proof of direct Metal API exposure or of Linux execution.

## Acceptance

- Empty task regions: no task tables, arms or individual workstation rectangles.
- Continuous single-level circulation; props remain in their perimeter strips.
- Five standard geometry views agree, with additional interior and detail cameras.
- No missing assets, obvious floating props, overlapping paint surfaces or severed supports.
- Factory build and viewing require no mutation of task scenes, evaluation or Agent tools.
- Reports identify renderer, dimensions, camera, hidden layers, frame timing,
  memory and failures. Linux and Isaac support are labelled unverified until exercised.

## Build and view

From the repository root, using the existing Python 3.12 environment:

```sh
.venv/bin/python -m scripts.build_robo_factory
.venv/bin/python -m scripts.view_robo_factory
```

The viewer prints its localhost URL and writes `outputs/robo-factory/viewer-connection.json`.
Drag to orbit, Shift-drag to pan, wheel to dolly; WASD moves horizontally and QE
changes height. Presets include closed interior views, an orthographic top view,
and four cutaways. Layer toggles preserve the building geometry in the source.
Capture saves both PNG and full camera settings under `outputs/robo-factory/captures/`.
The viewer caps updates at the preview profile rate and reuses the image while idle.

```sh
.venv/bin/python -m scripts.view_robo_factory --render --width 1600 --height 1000
.venv/bin/python -m scripts.check_robo_factory
```

Use `--renderer classic` for an explicitly selected OpenGL comparison. For another
machine, the factory environment requirements are in `assets/robo_factory/requirements.txt`.
The generated `scene/` directory includes relative assets and can be copied on its
own; the repository viewer is still required for the browser camera controls.

Camera automation uses the same local viewer: POST JSON to `/camera` with
`op: preset` and `name`, `op: frame_region` and a region ID, or `op: set` and
`position_m`, `target_m`, `vertical_fov_deg`, `hidden_layers`. The returned 202 is
queue acceptance; `/state` gives the applied camera and any error. These endpoints
control factory presentation only, with no robot operations.

## Initial v1 evidence (2026-10-24)

The empty factory implementation is usable on the current Mac. It contains a
48 x 42 m floor, four adjustable reserved regions, four walls, glazed openings,
two roller doors, a pitched roof, trusses, suspended lights, cable trays, conduits,
wall-side cabinets, bin racks, plants and extinguishers. It contains zero task
tables, arms, dynamic degrees of freedom or actuators. Background defaults are
scoped and display collision masks are disabled.

Eleven fixed-view PNGs are in `outputs/robo-factory/previews/`. The report there
records the exact scene hash and camera/layer settings. The engineering audit in
`outputs/robo-factory/reports/scene-check.json` checks assets, reserved-area
clearance, enclosure presence and compilation after copying the scene elsewhere.
The camera dropdown, top-view layer switching, FOV adjustment and screenshot
capture were exercised in the in-app browser.

On Apple M5 Pro with MuJoCo 3.16.0 / Filament OpenGL, warmed static 1600 x 1000
renders measured about 3.2–12.1 ms per view in a short sample including readback.
Peak Python-process RSS was about 596 MiB, model buffers about 62.2 MiB and decoded
model textures about 60.9 MiB. Texture bytes are included in model buffers and
must not be added twice. Process RSS is not total GPU memory; these measurements
exclude future task workstations and physical simulation.

Concrete uses locally cached Poly Haven `concrete_floor_02` CC0 maps with softened
contrast and normal strength; other finishes and meshes are project authored.
The asset manifest includes sources, hashes and the bundled font's OFL license.

The result is a lightweight real-time visualization. The photographic concept
remains the art target; reflections, lighting and fine props are not equivalent
to that generated photograph. Linux execution, motion recording/integration and
USD/Isaac conversion have not been exercised in this delivery.

## Detail revision v2 (2026-10-24)

The perimeter is denser: extra bin racks and cartons, maintenance drawer units
with shadow boards, two-level service carts, hose reels, structural bay numbers,
safety/operations boards, junction boxes, conduit drops, ventilation ducts with
outlets and supports, and cross-hall utility piping. All four experiment regions
and their circulation remain clear. No tables or robots were added.

`appearance.perimeter_detail` controls the additional dressing;
`appearance.batch_static_geometry` merges opaque plain-colour boxes/chamfers
within each semantic body, material and visibility layer. It preserves body
names and transforms; textured panels and glass remain separate. Batches are
visual meshes with contacts disabled, not physical convex collision hulls.
Tiny equipment labels use appropriately sized textures; repeated panels share
textures. Real-light count, exposure, shadows, resolution and AO quality were
not increased.

The enriched scene would contain 4,339 separate geoms without batching. The
retained version has 1,184, compared with v1's 2,539. This is a geometry count,
not an exact GPU draw-call count. The 116 semantic bodies' world bounds before
and after batching agree within 1.5 micrometres in the local check. There are
91,246 instanced mesh triangles, below the 300k budget; primitive tessellation
is additional and renderer-dependent. Decoded textures are 57.7 MiB and model
buffers 73.6 MiB.

A same-camera 1600 x 1000 Filament/OpenGL comparison alternated v1/v2 order over
three rounds, with 80 measured frames per view per round. Median trial means:

| View | v1 ms/frame | v2 ms/frame |
| --- | ---: | ---: |
| Interior south | 8.08 | 7.29 |
| Interior diagonal | 11.28 | 9.96 |
| Storage detail | 3.11 | 2.74 |
| Ceiling detail | 6.26 | 5.40 |
| Overhead | 7.95 | 7.50 |

This workload did not get slower; these results are hardware/load-specific and
include pixel readback, not future robot physics. Peak process RSS increased
from 583 to 690 MiB, so the added detail is not free in memory. The report is
`outputs/robo-factory/reports/v1-v2-comparison/comparison.json`; original scenes
and previews remain under `outputs/robo-factory/baselines/v1/`. Reproduce with:

```sh
.venv/bin/python -m scripts.benchmark_robo_factory --baseline outputs/robo-factory/baselines/v1/scene/scene.xml --candidate outputs/robo-factory/scene/scene.xml --output outputs/robo-factory/reports/v1-v2-comparison
```

## Material and lighting revision v3 (2026-10-24)

The layout, four reserved regions and perimeter equipment positions are unchanged.
This revision changes only static background appearance and camera-viewer timing:

- Replace the roller doors' thin black geometry strips with a filterable 1K x 2K
  shutter finish on a flat panel, keeping the backing, frame and windows as geometry.
  Close-range rib lines were complete in v2, but at the diagonal interior camera
  they broke into dark dots. Disabling sun shadows, disabling AO and moving the
  near plane individually did not cure that symptom. The replacement improves
  the observed distant artifact without supersampling or larger shadow maps.
- Use the same 2K floor-map budget for a whole-floor neutral concrete finish,
  retaining fine derived CC0 source grain with subdued service-strip scuffs,
  nonrepeating maintenance patches and broader roughness variation. The original
  `floor_tile_m` path remains available when `whole_floor_finish` is false.
- Distinguish bare galvanized metal, painted cabinet panels and matte plastic.
  Soften window highlights. Keep the original light count and one shadow-casting
  directional light; redistribute the four unshadowed fills toward side windows
  and balance their intensity against the environment. This is an efficient
  lighting approximation, not a path-traced solution for window bounce light.
- Exclude initial shader-compilation warm-up frames from the viewer frame-time
  display. This changes timing reporting, not rendering resolution or cadence.

The fixed-camera regression command below fails on archived v2 and passes on v3.
It samples horizontal variation in broad door bands, with a 4/255 acceptance
threshold for this symptom: v2 far/near 5.85/0.60; v3 far/near 2.78/0.29. This
proxy is not a universal image-quality score. Extremely thin distant rods and
rails still show aliasing; the scene is not claimed to be alias-free.

```sh
.venv/bin/python -m scripts.check_robo_factory_render
.venv/bin/python -m scripts.check_robo_factory
```

The current scene contains 1,186 geoms, 90,386 instanced mesh triangles, 63.7 MiB
of decoded textures and 79.4 MiB of model buffers. Full enclosure, static-only
scope, disabled display contacts, asset hashes, portability and reserved-area
clearance pass the existing engineering audit. Physical task/control parameters
and Agent tools are untouched. These texture and lighting features follow the
[MuJoCo material/rendering interfaces](https://mujoco.readthedocs.io/en/latest/programming/rendering.html).

The final same-camera 1600 x 1000 comparison alternates revision order across
three rounds, measuring 80 frames per view per round, including readback. Median
trial means are 7.82 ms (interior south), 10.26 ms (diagonal), 2.90 ms (storage),
5.21 ms (ceiling) and 7.10 ms (overhead). Corresponding v2 means were 7.99, 10.86,
3.01, 5.41 and 7.56 ms. Peak process RSS was 694/709 MiB. No meaningful slowdown
was observed in this final workload; it is not an acceleration guarantee. Short
exploratory measurements were noisier and are retained separately; use
`reports/v2-v3-comparison-final/comparison.json` for the final comparison.
Earlier v2 scenes, source and previews remain in `baselines/v2/`.

## Workstation preparation v4 (2026-10-25)

The current configuration is55×55 m with four23×23 m regions. The previous v3
configuration remains in configs/backgrounds/robo_factory_v3.json; previous
factory outputs are unchanged. The empty shell is outputs/robo-factory/v4-shell.
The separately composed visual scene at outputs/robo-factory/v4-workstations
contains all36 workstations. It has no physical task DOFs/actuators: source
physics stays in the simple environments and replay uses recorded poses only.

Use scripts/build_factory_workstations.py after generating the selected suite.
Check it with scripts/check_factory_workstations.py --render. Source motion
recording, replay evidence and limitations are documented in
[pre-Oracle preparation](../plans/suite-v2/pre-oracle-preparation.md).
The factory viewer defaults to the composed scene when it exists. Factory
replay does not establish Isaac/USD compatibility or physical task success.

## Presentation revision v5 (2026-10-25)

Current building55×50 m, shortening only Y by5 m. Four regions are23×20.5 m.
Mosaic shelf gaps increase1.2→1.6 m in both axes; other gaps remain1.2 m.
Source tasks/physics and existing motion recordings do not change. Current
artifacts are outputs/robo-factory/v5-shell, v5-workstations and v5-checks.
The previous55×55 v4 configuration and outputs remain available. The existing
two-second record also passed pose/video replay checks in the new layout.

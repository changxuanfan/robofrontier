# Frontier v2 — simple-background environment build

User-authorized scope: construct 36 new environments; preserve all v1 assets,
runs and factory files. No GPT execution, factory composition or full-task Oracle.
Compilation, static endpoint checks, rendering and short passive settling checks
are engineering checks, not a manipulation pass.

## Frozen design intent

- Transport: 4/8/12 Panda arms, 3 items per station. 100 mm solid cubes;
  flat target board / two flat plug blades and rectangular blind holes /
  three offset latch stations on one z-roll-pitch spring board. Latch: rigid
  left five-sided pocket, passive spring-return right sliding pocket. No pegs
  additionally imposed on latch objects. Stock rests on tabletop, blades face
  up or sideways. No supply/buffer pedestals.
- Mosaic: 4 arms 6x12 four colors, 6 arms 8x18 five colors, 8 arms 6x24 six
  colors. Opposed rows contain 2/3/4 arms per side. 100 mm cubes, 125 mm clear
  pockets, 1 mm walls, 126 mm pitch for all three levels. Pocket depth 1/3 cube,
  printed grid, unmarked tabletop. One attractive right-pointing rocket; color
  counts balanced, allowing a difference of two for mirror symmetry. Exact
  stock conservation, single-layer tabletop stock, no gray supports.
- Relay: 8 arms straight, 12 current quarter-turn arrangement, 18 arms on
  semicircle (12 outer + 6 inner), flexible support teams. Red straight beam /
  red rigid 120-degree arc beam plus cable/weight / same arc and three equal
  cables to three 120-degree-spaced tray rim attachments; free weight on small
  shallow tray. Chord remains 1.2 m; square section 45 mm, wood density 600 kg/m3.
  Start and end are marked tabletop regions; ropes initially slack, all parts
  supported above the table without hidden fixtures. At least two beam grasps
  in airborne transport. No prescribed grasp points or fixed pair allocation.
- Dial: 4/6/8 modules; N arms fixed gear, 2N floating gear and floating crank.
  Compact fixed layouts; compress six-module back-to-back row gap; inset ring
  by about 0.1 m. Remove gray handle parking racks. Blue handle tip and white
  disk naturally contact table at initial rest. Existing count-only feedback.
- Simple pale-blue background and matte brown tables. Robot/device IDs placed
  without obscuring targets. Factory is a separate visual layer, never copied
  into these assets.

## Physics and robot contracts

The old 40 mm / 50 g cube implies density 781.25 kg/m3; solid 100 mm cube is
0.78125 kg before attachments. Compute mass, COM and inertia from actual
components. Re-size spring-board stiffness/preload for this load. Friction and
contact settings are declared engineering assumptions, not measured materials.
No live object pose writes, grasp welds, automatic snap/lock or runtime physics
retuning. Solver remains fixed per case.

Panda 7R and each finger's 0..40 mm slide remain unchanged. Transport/mosaic use
an explicit manufactured-offset finger candidate, 40..120 mm opposing flat
grasp-face opening. Its solid mounting bridge/stem/pad, real mass and collision
geometry are visible. Other families retain standard fingers for thin handles.
The servo reads a robot-owned width-offset numeric; real gap is offset+q1+q2.
This is an unvalidated custom fingertip design, not a factory-certified model.

## Portability

Each generated case is self-contained, with scene, robot meshes, local textures,
task/physical parameters, initial keyframe and workstation manifest. Metres,
radians, right-handed Z-up, wxyz quaternions. Stable local IDs and geometry
hierarchy. Source/spec/compiled-physics/appearance checksums. A workstation may
later receive only translation and yaw, no scale. All moving bodies, cable
segments, passive sliders and display events are enumerated for state replay.
No USD/Isaac compatibility or full recorder integration is claimed by a manifest.

## Environment delivery, 2026-10-24

- All 36 new scenes compile, with 330 robots in total. V1 assets/runs and the
  separate factory sources are preserved. V2 directories use frontier-v2-*.
- Each scene passed a 3-second initial stationary-hold engineering check:
  finite state, no robot-to-robot or robot-to-apparatus penetration over 0.5 mm,
  endpoint attachment consistency and no free body drift over 15 mm. Observed
  maximum free body drift was 4.60 mm. The L3 relay weights stayed in their
  trays. These are initial checks, not task completion evidence.
- All 1,296 source instances and 1,296 goal instances across the transport and
  mosaic cases have a downward approach/end IK witness. This counts cases
  separately across levels; it proves neither full-path clearance nor contact
  assembly. Stock was moved away from initially unreachable base-adjacent cells.
- Final views: four 1920x1200 renders per environment, plus four 5976x8236
  top/front contact sheets and an HTML gallery in docs/generated/frontier-v2.
  The images are compiled initial states, not an Oracle movie.
- New tests cover physical pad gaps, unchanged finger travel, upper-cube grasp
  fit, blind-slot and latch clearance, cable attachments/moving-body registry,
  conserved inventory, matching L2/L3 mosaic dynamics, self-contained copies
  and saved-qpos reconstruction under rigid placement without mj_step.
- Servo/suite/geometry checks passed: 63 tests in the final combined run.
  Robot READY facts now read the installed opening range rather than advertising
  a contradictory fixed 80 mm maximum. No model planning rules changed.
- Replay boundary: workstation.py and export_workstation_pose.py reconstruct
  actual saved qpos into named body/geometry poses, including passive parts.
  Copies load without external vendor paths; translating/yawing changes only
  exported presentation poses. No factory scene was instantiated or edited.
- V2 tasks deliberately carry `execution_readiness=environment_preview_only`.
  `World.start_trial` refuses to silently run v1 evaluators on new latch/relay
  mechanisms. This is environment delivery, not GPT/Oracle commissioning.
- One optional argument was added to legacy `scenes.build_dial` for v2 compact
  layouts. Its defaults preserve v1 behavior. `suite.tasks` dispatches v2
  loading/projection without changing v1 contracts.
- Not started: manipulation Oracle, GPT, factory placement, Isaac Sim.

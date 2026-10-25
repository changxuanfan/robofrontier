# Frontier v2 R2 — simple-background environment iteration

The previous build is preserved as configs/suites/frontier-v2-r1.json, its
original frontier-v2-* model folders, builder-r1.py, implementation-r1.md and
docs/generated/frontier-v2-r1/. Current models use frontier-v2-r2-* folders.
No GPT, full manipulation Oracle, factory composition or Isaac import ran.

## Accepted revision

- 36 cases: transport 4/8/12 arms, mosaic 4/6/8 arms, relay 8/12/16 arms,
  codebreaker 4/6/8 disks. Three physical levels per scale; 324 robots total.
- All stock cubes: 60 mm. Transport identities G onward use cylinders of
  60 mm diameter/height, reusing the six colors. Nominal density 781.25 kg/m3:
  cube 168.75 g, cylinder 132.536 g; two steel blades add 3.9936 g. Actual
  component inertia is compiled into each frozen workstation manifest.
- All robots use original Panda fingers and 80 mm nominal maximum opening.
  No finger travel, actuator capacity or control-speed limit is enlarged.
- Transport L1: flat marked board; L2: fixed tabletop inline plug board;
  L3: identical three inline plug sockets on a compliant z/roll/pitch board.
  No latch, sliding sleeve or retaining crossbar. Blade 2x8x16 mm, separation
  24 mm; socket 3.5x10 mm, depth 17 mm; board 23 mm thick. Target pitch 100 mm.
  L3 vertical K=1400 N/m,C=45 Ns/m; angular K=.60 Nm/rad,C=.075 Nms/rad.
  Limits: z -3..18 mm (positive downward), roll/pitch +/- .35 rad.
- Mosaic: 4 arms 6x12; 6 arms 8x18; 8 arms 10x24. Exactly four colors at
  every scale. 60 mm cubes, 75 mm clear pocket, 1 mm wall, 76 mm common pitch.
  L1 depth20 mm, L2 red 3 mm grid lines, L3 unmarked. Single outlined rocket
  with white body/red nose and fins/blue portholes and background/yellow exhaust.
  Artwork defines inventory counts; no equal-color-count or equal-region quota.
- Relay: straight / 90-degree red arc+weight / same arc+three-string tray.
  Arc chord1.2 m,45 mm square wood section,rho600 kg/m3. 16-arm semicircle:
  10 outer at2.50 m radius,6 inner at1.10 m radius, flexible teams.
  Tables are wider with source/end landing areas .38 m beyond end robot
  baselines. No cradles; initial assemblies rest on tabletop before first pair.
  Tray outer100 mm, flat floor70 mm, ramp rise10 mm/run15 mm, material2 mm;
  free250 g weight50 mm diameter/height. Genuine passive cables/contacts.
- Dial L1: concentric annular tables for4/6/8 disks. L2/L3 six modules form
  one horizontal .70 m pitch row with opposed robot rows. Other floating
  scales retain their layouts, no gray supports. Count-only puzzle unchanged.

## Evidence and boundaries

All 36 compiled and passed 3 s stationary-hold engineering checks. Checks include
finite states, robot/robot and robot/apparatus penetration, real mesh support
under robot mount centers/corners, passive cable constraints, tray weight
retention and free-body drift. Maximum free-body drift 4.60 mm.
All 1584 source instances and 1584 goals across transport/mosaic have downward
approach/end IK witnesses. No claim of full-path clearance or task completion.

67 unit/regression checks passed, including physical slot clearance, masses,
unchanged finger travel, annular layout, one-row dials, cable attachments,
matching mosaic L2/L3 dynamics, portable copied assets, saved-state pose export
without physics and previous controller behavior. Gallery includes four views
per case and four 5976x8236 top/front sheets, plus clear rocket references.

The migration contract is retained: SI/Z-up/wxyz, stable identities, frozen
assets and hashes, actual qpos reconstruction, translation/yaw placement only.
No factory assets changed. Unified live recorder/Isaac conversion are separate.
New tasks remain environment_preview_only pending full physical commissioning.

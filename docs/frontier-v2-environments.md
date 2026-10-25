Current environment draft: [Transport R13](plans/suite-v2/transport-r13.md),
covering only the nine transport scenes; the other27 scenes retain R10.
`configs/suites/frontier-v2.json` selects this mixed catalogue.
[R10 implementation and Oracle evidence](plans/suite-v2/implementation-r10.md)
and `configs/suites/frontier-v2-r10.json` remain frozen historical inputs.
R13 now also has [targeted physical units and a two-job parallel comparison](plans/suite-v2/transport-r13-units.md), not full-task qualification. R11/R12 retain static-preview status; none inherit R10 Oracle qualification.
The descriptions below are historical where they name earlier revisions.

# RoboFrontier v2 — current environment previews

The current catalogue is `configs/suites/frontier-v2.json`: 36 simple-background
workstations: three R4, eighteen R5, nine R6, three R8 relay L2 and three R9
mixed-hinge relay L3 cases.
Previous catalogues, model folders and builder snapshots remain available.
The existing V5 factory still contains R7 relay assets; R8 factory composition,
full Oracle, GPT commissioning and Isaac conversion have not run. See
[R8 apparatus](plans/suite-v2/relay-r8.md) and
[R9 mixed-axis assignments](plans/suite-v2/relay-r9.md).

## Current layouts

| Family | Scales | Levels |
| --- | --- | --- |
| Item transport | 4 / 8 / 12 arms | Flat placement / fixed plug board / spring-supported plug board |
| Mosaic | 4 arms 6×12 / 6 arms 9×18 / 8 arms 12×24 | Physical pockets / red printed grid / unmarked tabletop |
| Airborne relay | 8 / 12 / 16 arms, carrying 2 / 3 / 4 rods | Straight rod / 90° curved rod with closed-eye weight / spring-hinged135° elbow with rigid pendulum weight |
| Dial codebreaker | 4 / 6 / 8 disks | Fixed gear / floating gear / floating handcrank |

### Item transport

Table edges remain 1.938 / 2.720 / 3.536 m, and the four-arm table is solid.
Stock identities remain constant across scales: A–F each supply three cubes in
red, blue, yellow, white, cyan, purple; G–L each supply three cylinders in the
same color order. Four arms use A–D; eight use A–H; twelve use A–L. Final orders
vary by scale. Cubes are 60 mm; cylinders have 60 mm diameter and height.

L2 and L3 use the same pair of thin steel blades and matching blind sockets:

| Quantity | R5 value |
| --- | --- |
| Each blade: thickness × width × insertion length | 2.5 × 12 × 24 mm |
| Blade center spacing | 24 mm |
| Each socket clear opening | 4.5 × 15 mm |
| Socket depth / board thickness | 26 / 32 mm |
| Total opening clearance: thickness / width | 2 / 3 mm |
| Depth allowance below a fully inserted blade | 2 mm |

These are real empty collision apertures, not painted marks. The fixed board
bottom rests on the table. L3 retains a passive z/roll/pitch support; it has no
latch or scripted attachment. Kz=1400 N/m, Cz=45 Ns/m, Kangle=0.60 Nm/rad,
Cangle=0.075 Nms/rad. The visible spring coil is cosmetic; the joints implement
an equivalent compliant support, not finite-element spring mechanics.

At wood density 781.25 kg/m³, a cube is 168.75 g and a cylinder 132.536 g.
The two steel blades at 7800 kg/m³ add 11.232 g. MuJoCo compiles the new masses
and inertias from this actual geometry. Initial blades point up or sideways,
with support height recomputed from the rotated object bounds.

### Mosaic

The original user upload remains `configs/patterns/tulip_parameters.json`.
`configs/patterns/tulip_parameters_r5.json` changes only three 9×18 petal cells,
mirroring the left petal mask onto the right; the other two pattern grids stay
unchanged. The whole portrait artwork rotates 90° clockwise for the horizontal
workstation, without resampling or count rebalancing.

The selected **Bright Tulip** palette is cream #FFF4DA, navy #173F73,
petal #F03F5B and leaf #16A16E. Options and the user selection are stored in
`configs/patterns/tulip_palette_options.json`.

| Portrait size | Cream | Navy | Petal | Leaf |
| --- | --- | --- | --- | --- |
| 6×12 | 21 | 14 | 22 | 15 |
| 9×18 | 65 | 32 | 32 | 33 |
| 12×24 | 102 | 58 | 70 | 58 |

All cubes remain 60 mm. L1 has 80 mm clear pockets, 81 mm common pitch and
20 mm depth; L2 has 3 mm red lines; L3 has no physical or printed grid.
Same-side robot X spacing remains contracted 5% from R3. Compared with R4,
only four/eight-arm table Y lengths shrink 10%; table X widths and the six-arm
table stay fixed. Table X×Y dimensions are 2.465×2.66175, 3.0725×3.41125,
3.680×3.3435 m respectively. Stock is kept within reach, outside the goal grid.

Every mosaic source and target has a sampled geometric access candidate:
rotate the empty wrist at a high pose, then approach at +120 mm, +60 mm and
the operation point. A nominal 70 mm opening is used for engineering screening;
the runtime agent remains free to choose its grasp. Checks include initial
self, table, neighbor-stock and other-robot collision geometry. Two rotation
samples and three approach poses do not certify the continuous paths between
them, nor grasp stability, later clutter states or complete task success.

### Airborne relay

The rods are distributed along each route, not all stacked at the left source.
Each assembly has stable numbered IDs and its own pickup patch. All must progress
to the final right-hand region; no fixed transport order is imposed. The green
final landing patch is enlarged within the tabletop to allow multiple arrivals;
complete dynamic delivery and final packing remain unverified.

L2 has two physically closed, contact-linked steel eyes and a600 g weight.
L3 has two straight timber segments: neutral interior angle135°, endpoint span
1.2 m, rise248.528 mm, square section45 mm. A passive spring-return hinge joins
the halves; a short rigid pendulum and600 g weight hang through closed eyes.
Weights are independent free bodies. No Cable, tray or robot attachment is used.
L3 mixes in-plane and out-of-plane hinges per numbered rod:8 arms1+1,
12 arms1+2,16 arms2+2. Public mechanism metadata identifies each axis.

Twelve and sixteen arms retain upper-semicircle layouts and left-to-right route
order. Sixteen arms retain outer10/inner6 bases. Table support is checked for
robot mounting footprints, pickup/final-region corners and every rod footprint.
Initial assembly contacts and inter-assembly penetrations are checked.
Each rod has a close camera, plus weight and L3 hinge inspection views.

### Dial codebreaker

White disks now have **300 mm diameter**, with 40 mm tall black
0/1/2/3 numerals at radius122 mm, leaving a nominal8 mm outer-rim margin.
The larger rim accommodates doubled numerals clear of the unchanged crank sweep.
Disk mass scales with face area to510.397 g;
compiled inertia follows the larger collision shape. Floating initial rest is
recomputed from the disk rim and blue handle tip, without gray parking blocks.

L3 uses one round radial crank, with no protruding opposite balance arm. The
shaft is83 mm long and20 mm in diameter; the handgrip is27.5 mm in diameter,
25% thicker than before. The crank has no orange arrow or stripe; its own
radial direction indicates the setting. L1/L2 gear knobs retain an orange arrow. Shaft
and grip masses are rescaled from their geometry. The unilateral spring is
retuned to0.12 Nm/rad; **return dynamics and reachable torque are not yet
commissioned**. The enlarged disk also requires renewed dynamic commissioning. A long-lens frontal inspection camera makes edge numerals readable
without changing any physical geometry or the ordinary global task views.

L1 remains rectangular opposed rows for four/six disks, with eight in3-2-3.
Six floating disks retain the shared upper semicircle with twelve arms. Each
floating module still requires supporting the blue handle while manipulating
its rotor; the environment preview is not evidence that this is dynamically solved.

## Evidence and migration boundary

This revision runs static initial-state checks and sampled IK/access checks,
with zero physics time advanced and no task commands. Tasks retain
`execution_readiness=environment_preview_only`; no task-success claim follows
from these checks. Standard Panda fingers and travel/force/speed limits remain.
No runtime robot harness, model scheduling policy or controller was changed.

Every model retains self-contained meshes/textures, stable body/joint/geometry
IDs, initial keyframes, compiled masses/inertias and checksum manifests. Units
are metres/radians, Z-up, quaternion wxyz. Future placement is rigid translation
and yaw without scale. Saved qpos can be reconstructed without simulating.
The recording contract is preserved; a unified live recorder, factory import
and Isaac/USD compatibility still require separate implementation/verification.
Private engineering state/export files are not policy observations.

## Previews and regeneration

`docs/generated/frontier-v2/index.html` links current views. Family top sheets
are5976×4276 px; relay also has a nine-case close sheet and individual views of
all27 rods. Plug/socket inspection views accompany L2/L3 transport scenes.

Build selected cases with `scripts/build_frontier_v2.py`; `--replace-draft`
replaces only uncommissioned drafts. Do not indiscriminately replace preserved
R4 inputs. Run `scripts/check_frontier_v2.py --coverage` for engineering checks
and `scripts/preview_frontier_v2.py` for actual static MuJoCo renderings.

## Archived pre-Oracle R7 preparation

The previous six relay L2/L3 cases used R7 official elastic Cable; these and
older R5 inputs are preserved. That snapshot contains threeR4, eighteenR5,
nineR6 and sixR7 cases; current relay inputs useR8.
All36 candidate physical inputs are frozen. The same shared60 Hz float32
recording format is connected to Oracle and policy execution, and offline
factory pose replay is checked. Future GPT defaults to Adaptive through
configs/harness-frontier-v2.json. Task execution remains gated until full-task
commissioning; no Oracle or GPT result is claimed.

Cable numerical stability requires0.5 ms in the retained16-segment candidate.
Real-material calibration is unverified, and the largest relay is slow in the
short capacity sample. See [pre-Oracle preparation](plans/suite-v2/pre-oracle-preparation.md)
for evidence, performance, code entry points and unresolved physical questions.

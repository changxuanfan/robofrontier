> 此文件是准备前的历史审查。当前结果见 [pre-oracle-preparation.md](pre-oracle-preparation.md)。

# Pre-Oracle review — 2026-10-25

Scope: read-only source/model inspection, static dimensions and analytical load
estimates. No Oracle, GPT run, dynamic commissioning, factory rebuild or task
scene modification. Two audit/proposal JSONs and a scaled planning diagram were
written under docs/generated/frontier-v2. Future GPT mode is Adaptive, as requested.

## Factory fit

Current factory v3 is48×42 m with four empty reserved regions. It contains no
workstations. Region sizes: transport/mosaic19.5×14.2 m; relay/dial19.5×18.8 m.
Coordinate conventions match current workstations: SI, Z-up, same floorZ=0.
The tabletop height remains0.4 m; do not scale the source scene to fit a bay.

Planning envelopes union initial physical/table geometry with an indicative1 m
XY radius around each robot base. This is NOT a loaded-motion swept-volume
certificate. An initial3×3 arrangement with1.2 m inter-cell gaps gives:

| Family | Planning grid W×H, m | Current region |
| --- | --- | --- |
| Transport | 17.886×15.472, grouping scales by row | Too short in Y |
| Mosaic | 12.0708×13.146 | Fits before added edge margin |
| Relay | 20.18×15.5793 | Too narrow in X |
| Dial | 19.91×19.24 | Too narrow in X and short in Y |

A52×50 m factory is a feasible *candidate* for these planning envelopes. Proposed
region bounds: transport[-23,3.8,-1.5,22], mosaic[1.5,3.8,23,22],
relay[-23,-22,-1.5,0.8], dial[1.5,-22,23,0.8]. All36 planned bays fit with
at least0.6 m region-edge clearance. The narrowest is relay0.66 m. Preserve
central aisles and perimeter dressing; regenerate/review doors, columns, racks
and cameras if adopting the expanded building. No current config was changed.

The proposal stores per-case rigid translations/yaw and case namespaces. After
recording, calculate actual all-body swept bounds. If a long rod or robot exceeds
a planned bay, move bays or expand the factory; never alter recorded motions.

All36 workstations contain324 robot arms in total. A factory replay should be a
visual scene, not a new324-arm dynamics simulation. Shared assets can be reused;
nonselected workstations can remain in their declared static initial states.
Combined playback of separately recorded tasks must be labelled a presentation,
not evidence of one concurrent experiment.

## Offline replay contract and gaps

Existing workstation.json contains stable bodies/geometries, assets/hashes,
initial state and future recording schema. Workstation.frame reconstructs all
body/geom poses from qpos without physics steps. A factory composer/animated
replay integration does not yet exist.

Existing legacy Oracle Trial defaults to4 Hz state frames; CLI allows4/15 Hz.
Frame NPZ is accumulated in memory and written at close, so it is insufficient
for high fidelity motion and crash-safe recording. Event samples include qvel,
but regular frames do not. The run copy does not currently preserve the full
workstation manifest as an integrated replay package.

Required shared recorder, used by both Oracle and GPT execution:
- Bind exact source XML, assets, named state layout and private hashes to a run.
- Record every committed physics-step qpos/time for lossless discrete-state
  motion replay, including all object, rope, passive board and dial joints.
  Record display/state visual changes on the same simulation timeline.
- Use bounded streaming, lossless compressed chunks and index/checksums; do not
  retain a long whole-run array only in process memory. Detect gaps and retain
  completed chunks on interruption. Avoid redundant full body/geom pose streams;
  reconstruct them with source-model FK. qvel/forces/control are optional audit
  channels, not necessary to render the same geometry.
- Full dense qpos64 at500 Hz in the largest mosaic is28.0 GiB per simulated
  hour before compression; qpos+qvel is52.14 GiB/hour. Unchanged-value/delta
  encoding can be lossless; actual compression/performance must be measured.
- Apply only a fixed SE(3) placement to reconstructed poses. Use recorded object
  motion; do not rerun IK, commands, contacts or factory physics.
- Namespace each case, remove duplicate task background floor/sky, preserve
  actual task tables and visual bindings. Do not import background solver options.
- Test a synthetic saved-state clip and key timestamps in simple/factory views
  before expensive Oracle runs. This is replay integration testing, not a task
  Oracle or a physical success demonstration.

Private recorder truth and Oracle plans/codes must not enter GPT observations,
work notes or skills. Both execution paths use the same physical source revision,
controller contract and recording format.

## Physics findings

Cube168.75 g and cylinder132.536 g are consistent with60 mm geometry at
rho781.25 kg/m3. Two2.5×12×24 mm steel blades add11.232 g. Socket clear size
4.5×15 mm, depth26 mm leaves total2/3 mm horizontal and2 mm axial allowance.
Dimensions and masses are internally consistent, not measured material evidence.

The L3 board mass is725.3155 g. With three179.982 g cube/plug objects the
linear static z-load estimate is8.866 mm sag atKz1400 N/m, within18 mm travel.
One outer object at100 mm produces about0.1766 Nm and the small-angle estimate
is16.86° atKangle0.6 Nm/rad, close to the20.05° per-axis stop. This is a
purposeful compliant stage, not a nearly rigid board. Dynamic tilt, damping,
contact penetration, insertion and load distribution remain untested.

L3 floating dial module body mass625.31 g, rotor159.59 g; unilateral spring
K0.12 Nm/rad. Enlarged handles and removed counterweight change gravity torque.
Reliable return, achievable grip torque and all-angle access remain unverified.
NoSlip10 is retained consistently; it is a solver choice suppressing numerical
friction drift, not a material measurement or object attachment.

## Cable fidelity

Current cable:200 mm long,4 mm diameter,8×25 mm links,1 g/link; linear mass
40 g/m. Ball joints have damping0.00015 and armature1e-6 kg m2. Compiled link
transverse inertia is6.499e-8 kg m2; artificial joint inertia is about15.4 times
that value (and larger relative to axial inertia). It can affect bending/twist
response; it cannot be ignored as merely cosmetic stabilization.

The model is a discrete flexible, fixed-link-length approximation with soft end
closure. It has no calibrated axial stretch or bending/twist material law. Same
and different cables currently do not collide, so they can pass through. It can
represent approximate low-speed suspension, but is not validated physical rope.

Before Oracle: specify cord/cable material and linear mass, audit artificial
inertia/damping, and compare8/16-link discretizations under equivalent total
length, mass and material parameters. Doubling links must not double total mass.
Test sag/tension, swing period/damping, end residual, table/rod/tray contact and
nonadjacent rope crossing. Exclude only physically adjacent overlap or deliberate
junction contact. Keep current chain if it meets this task's accuracy needs;
use the official elasticity cable plugin only if needed for material bend/twist,
not as an automatic full-model rewrite.

## Additional execution blockers found

The current production verification script still loads frontier-v1. Its Monitor
has40 mm cube and50 mm pocket constants, old blade dimensions and single-payload
fixed-pair references. Current R5 uses60 mm cubes,80 mm pockets, longer blades,
multiple named payloads and flexible teams. Scene preview tests do not validate
this old task evaluator. Port runner, evaluator and reference executors to v2
and read dimensions/assemblies from the task before calling their results valid.
Do not bypass environment_preview_only or label an old-version pass as R5.

## Proposed order

1. Adopt/review factory placement and render actual static composed views.
2. Complete shared streaming recorder and factory recorded-state replay checks.
3. Calibrate/freeze cable and mechanism assumptions; port v2 evaluation.
4. Then run mechanism commissioning and selected full-task Oracles; iterate on
   explicit new source revisions. Keep failed attempts and immutable bundles.
5. Run GPT using Adaptive with matching physical inputs and the same recorder.

Sources: local files robofrontier/workstation.py, recording.py,
robofrontier/suite/{v2,oracle,evaluate}.py, scripts/verify_frontier_suite.py,
configs/backgrounds/robo_factory.json and outputs/robo-factory/reports/scene-check.json.
Official references: https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-joint
(armature is added inertia), https://mujoco.readthedocs.io/en/latest/modeling.html#cable
(elastic cable representation), https://mujoco.readthedocs.io/en/latest/XMLreference.html#option
(NoSlip solver settings).

## Later environment update — R6 dial readability

The dial-only revision now has300 mm disks,40 mm digits and no L3 orange
marker. R5 is preserved. The prior625.31 g module body mass above describesR5,
not R6. Factory placement and mechanism-load assumptions must be refreshed
against the selected frozen revision before commissioning. No Oracle ran.

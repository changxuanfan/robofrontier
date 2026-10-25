# R10 Oracle engineering findings

This records engineering evidence, not GPT policy performance or an optimal-time
reference. Full case status is authoritative in each batch's `summary.json`.
Legacy and `pd1000-v1` successes must remain separate.

## Shared execution

The legacy hand aperture drive had insufficient gain under a real two-hand load:
waiting to its original timeout did not reach80 mm. The new opt-in position
profile uses K=1000 N/m, robot-mass-based damping and a70 N cap. It preserves
force-grasp damping and original passive properties. Mode-switch contact tests
and52/80 mm empty tracking tests passed. An aperture failure under an external
contact constraint is still a real failure, not something to label completed.
References may recover through a separately recorded safe move and release.

Both Transport and Dial exposed a semantic reference error: force mode ending
does not establish physical release. A known failed opening must not be followed
by a long withdrawal that drags the payload. Their private references now check
actual release and choose bounded legal recovery; the public executor does not
automatically choose a new grasp or retry. The shared Skill states this evidence
distinction, without task coordinates, fixed apertures or mandatory tool calls.

Mosaic's scalar O(goals×objects) acceptance loop was a runtime bottleneck.
Vectorization preserved predicates, floating-point boundaries and physical
trajectories; it did not weaken acceptance or change the integrator. Independent
state-prefix comparisons and boundary tests cover that equivalence.

Planning scratch states cannot publish live status/images. Family references
must never execute in scratch data or write object poses into the live world.
The full physical model, actuator commands and process monitor remain the
execution evidence. A collision screen is not a grip-stability certificate.

Each new execution freezes package code, lazily imported script helpers, the
catalog and harness before starting. Independent changes to the live tree do
not change an already running process. Canonical assets are separately hashed.
This is source consistency, not an OS sandbox. A complete4-dial replay using
the frozen launcher reproduced all14,157 recorded states bit for bit.

Oracle process logs now record state changes and a1-simulation-second heartbeat
when the shared motion stream is active, instead of duplicating every object's
pose in JSON. A fresh complete4-dial test passed with all14,157 motion states
bitwise identical: process JSON shrank from2,406,445 to52,760 bytes (45.6×).
That measurement concerns the process log only; full motion was not reduced.
Evidence: `runs/engineering/oracle-compact-log-20261025/verification.json`.

The Oracle advances simulation during explicit execution/waits. Wall-clock
planning does not itself extend a physical hold test. Only simulation time is
evidence of sustained grasp stability; the continuously running policy world
can have different wall/simulation clock behaviour.

## Transport

A successful pickup can prevent the next robot's hand from approaching. Relative
hand/payload screening before arm IK removed thousands of irrelevant solutions.
Choosing a different actual initial grasp, then proving the physical regrasp,
was necessary; adding IK seeds to incompatible hand geometry was not useful.
The tested millimetre values are private reference choices, not general Skills.

A one-time supply reachability map becomes stale when a cylinder physically
rolls. One observed supply moved about117 mm. Candidate pickup and handoff plans
must use the current pose, with the same actual grasp-frame definition.

For a cylinder, arbitrary roll phase is not a meaningful upward face. Project
world vertical into the plane normal to its cylinder axis, and plan radial
capture. Grasp height changes the real contact span of curved surfaces; an
incorrect expected aperture should fail honestly. Do not fix it by increasing
force or silently widening the width tolerance.

## Mosaic

A two-target speed probe did not generalize to all loaded paths. The same
4-arm L2 target passed at0.45 rad/s but lost the cube at0.65 rad/s with the same
50 N grasp, source and controller. The reference now checks contact and tool/
payload relative motion before and after transport. The conservative complete
72-target L2 run subsequently passed with zero failed robot actions.

Large cases need real tabletop supply relays; static reachability alone is not
a physical witness. The longest8-arm chain has been physically executed across
four arms in a single-target commission. It does not count as288 completed
cells. Full long runs have finite time/action budgets and stop on new failures.

One full8-arm attempt stopped at an IK admission rejection near a buffer release
endpoint after its high hover had been feasible. End-height, refinement
neighbourhood and withdrawal need their own margin checks. Do not enlarge IK
or acceptance tolerances to hide this reference planning gap.

One8-arm empty-arm route passed a0.025-rad sampled joint screen but collided
between samples. A201-point reconstruction found0.626 mm nominal penetration
at82.5% of the segment; the actual violation had only0.000373 rad tracking
deviation. This was a sampling miss, not evidence that slowing alone solves it.
The private reference can request a finer0.005-rad screen and still retains
the physical process audit. Neither resolution is a continuous certificate.
Irreversible task violations now end canonical Oracle attempts immediately,
preserving the exact event state and unresolved in-flight action IDs.

An apparent32 mm in-grasp displacement in the6-arm case was not a drop:
the actual isolated continuation remained suspended with bilateral contacts
for2 simulation seconds, with65.6 micrometres further translation. A private
rigidity heuristic must distinguish stable in-grasp repositioning from ongoing
slip. Updating a measured grasp transform requires a bounded stability check
and renewed downstream geometry screening; it is not permission to teleport
the object or waive task acceptance.

## Air relay

The receiving grasp must have an accessible arrival route, a donor opening/
withdrawal route, room for the next transfer, and a valid final lowering path.
A sampled endpoint without these continuations caused repeated dead ends.
A corrected reference physically completed a16-arm rod's entire nine-transfer
chain plus stable landing in a legacy diagnostic run; that is1/4 rods, not a
complete16-arm task and not a new-driver success.

Adding complex side-grasp candidates regressed a previously successful straight
rod baseline when ranked ahead of its proven downward, staggered grasps. Restore
mechanism-appropriate priority; preserve broader candidates as bounded fallback.
Do not turn an engineering reference preference into an Agent permission rule.

For weighted rods,70 N kept one receiver stable longer than50 N but did not
remove a gravity moment caused by the grasp line. A later slow roll still
failed. Force alone is not the solution: bounded asymmetric grasp-point choices
must consider actual assembly COM, all arm geometry and future continuations.

## Dials and factory presentation

In a new-driver independent4-dial L3 test, all rotors were released at90,180 and
-90 degrees without the final controlled-unload procedure. All returned within
the4-degree zero band; the largest residual was about2.84 degrees. Evidence is
in `runs/engineering/dial-passive-release-pd1000-20261025/`. This is stronger
than the old15-degree probe, but does not certify every scale automatically.

Larger L3 cranks exposed two distinct release cases. A centered narrow capture
remained near26 mm despite an80 mm request; an independent higher-gain fixture
also jammed at the same legal70 N ceiling, so increasing gain was not adopted.
A4 mm private capture offset with50 N allowed all six hands to reach80 mm,
but the returning crank then touched an open finger and stalled near67°.
That fixture did not pass passive return. Full opening, absence of bilateral
capture, and complete separation are different facts; a safe axial exit is
being independently tested before changing the reference's release guard.

The fresh six-dial90-degree offset fixture subsequently passed real full
opening, small axial retreat and passive return (maximum residual3.59°), with
all blue holds maintained. It is still a single-angle commissioning result.
Full four-/six-dial repeats then exposed genuine trapped openings at other
angles; the90-degree fixture was not promoted as all-angle proof.

A separate point is the module-displacement guard used during diagnosis:3 mm
was a conservative private test limit, not a game requirement. Moving a module
while the opposite hand continuously supports it is allowed. A new bounded
supported-freeing reference therefore preserves failed aperture feedback,
disables force grasp, checks continuing qualified support, attempts small axial
motions, and requires actual separation plus a separate successful opening.
The unchanged physical/process evaluator remains authoritative. A fresh full
four-dial L3 run using this sequence passed (attempt005); larger scales remain
under test. Neither an opening failure nor force-mode exit alone is relabelled
physical release, and no new public executor recovery policy was added.

The subsequent six-dial full attempt005 failed honestly during the half-turn
unload: D02 lost qualified blue support while C was still driving its rotor.
The process audit stopped at269.102 simulation seconds and preserved C as an
unresolved in-flight action; the world was closed, not resumed or silently
replayed. This is a coupled support/capture failure, not the already-solved
open-finger sweep case. No eight-dial rollout of that candidate was started.
Current new-profile Dial completion remains7/9. Detailed reports and failed
candidate executor bytes are retained; the successful four-dial witness is not
generalized to all scales.

Factory replay preserves source identities and uses FK plus a rigid placement;
it never reruns task commands. A Cycles/Metal adapter preserves visual geometry
and avoids the inspected Filament shadow artefact. Static USD export must use
matrix transforms: default Euler export introduced near-gimbal rotation errors.
The matrix export's31,517 geom IDs/names, Z-up metre scale and poses passed an
independent USD readback. Isaac dynamics and full USD animation are unverified.

## Durable results

Keep source revisions under `configs/environments/`, catalog snapshots under
`configs/suites/`, actual run data under `runs/oracle/<batch>/<family>/<case>/`,
engineering fixtures outside full-case attempt patterns, and derived videos/
factory images under `outputs/`. Motion is60 Hz float32 plus timed appearance
events, streamed with bit-exact compression after quantization. It is visual
reconstruction data, not a resumable2 ms physics checkpoint.

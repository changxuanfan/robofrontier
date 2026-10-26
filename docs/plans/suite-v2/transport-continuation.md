# Transport continuation — 2026-10-26

This checkout resumes archived Transport source snapshot `0a219f2` (parent
`41e9979`) on `codex/transport-continuation`. The original worktree was archived
during takeover; the snapshot retained the new candidate regression and edits.
The old PID3432 was already absent. Its 8/12 frontier remains interrupted evidence,
not a restart checkpoint or a full pass. The takeover interruption record is in
`runs/oracle/transport-r13-full-20261025-wt-r01/`.

## Changes and qualification

- Broaden finite receiver angles/depths, limit each candidate family search,
  and screen the selected grasp's successor. L1 compares low-to-high release
  clearances; L2/L3 retain their socket checks and never inherit free-drop release.
- Carry the selected L1 release height into actual placement commands, preserve
  the original goal, and request precise TCP motion independently of socket seating.
- Preserve normal regrasp correction when a released L1 object misses the original
  goal; no scoring relaxation, live object state writes, resets or failure erasure.
- Explicit engineering pauses use the attempt's `control.json` containing
  `{"paused":true,"reason":"..."}` or `{"paused":false}`. Wait for
  `pause-status.json` acknowledgement before ending the supervising turn. The
  engineering world freezes while acknowledged paused. Only acknowledged pause
  time is excluded from the active wall budget; planning, recovery, unmarked
  process suspension and waits count. Reports retain total/paused/active time.

Recorded kinematics reproduce the old candidate rejection in under one second;
the expanded coverage regression passes. This proves geometry coverage only.
The Transport/source/pause regression group has70 passing tests.

Actual original-order units in `runs/oracle/transport-continuation-20261026/units/`:

| Unit | Outcome |
|---|---|
| `ac-001` | Complete handover/release; final target missed |
| `ac-002` | Precise TCP motion; release still displaced payload |
| `ac-003` | Slower jaw opening did not remove the miss; option not retained |
| `ac-004` | A_item2→C_slot2 accepted after one ordinary regrasp correction |
| `db-001` | D_item2→B_slot2 accepted after one ordinary regrasp correction |

Both successful units have zero violations/unknown actions and65 robot commands.
They are frozen-program unit evidence, not full orders or efficient direct drops.
All unsuccessful source snapshots/traces/motion streams remain available.

## Next gate

Run the original12-goal four-arm L1 order in one new world, then inspect full
acceptance, physical overlap, retained resources and final stability. Preserve a
blocked world for forward recovery. Extend to other scales/levels only after the
representative gates, and keep Oracle recipes out of GPT policy inputs.

## Full-order attempt001: asynchronous phase regression

The first full order stopped at5/12 with C_hand/D_link5 contact at96.92 simulation
seconds. The failure and all unresolved actions are retained; it is not resumed.
The nominal admission screen moved all arms through the same normalized path
fraction. Actual independent speeds let D finish while C was midway, a combination
the screen never examined. A recorded92.20-second FK fixture reproduces the bad
admission in0.43s; the fixed screen rejects it in1.21s. Pairwise independent phase
samples now include finished/idle peer poses. This is still sampled nominal
geometry, not a dynamics or continuous-clearance certificate. Process monitoring
and unchanged original acceptance remain authoritative.

## Full-order attempt002: assisted12/12

All12 original goals accepted, zero process violations and zero unknown actions.
Total503.22 simulation seconds,309.93 wall seconds,336 robot commands. Actual
multi-arm motion overlap124.86 simulation seconds; peak4 moving arms. These are
whole-episode metrics, not a serial/parallel speedup measurement.

The program reached11/12 by itself. The final failed scope had been retried when
only B/C were available, and the once-per-goal automatic retry rule prevented
reconsideration after later work freed all arms. One owner-authored empty-step
request expanded the final replanning scope to the four free arms. It performed
ordinary commands from the actual world and completed the final goal. The report
correctly marks `engineering_assisted=true`, `llm_used=true`.

New code admits a fresh current-state retry after genuinely new accepted progress,
at most once per goal/progress count. Static blocked frontiers cannot spin retries,
and unknown/active actions remain ineligible. The regression reproduces the old
second-progress failure before the fix. Next run must establish a frozen-program
full pass without the owner's recovery request.

## Frozen full-order attempt003

Source `c152ef3` completed all12 goals without owner recovery input: zero process
violations, zero failed or unknown robot actions,335 robot commands,453.66sim s,
259.42wall s. Actual multi-arm movement overlap131.84sim s,390.16sim s with any
arm moving, peak4 moving arms. All27221 source frames and visual events audited.
Two-view4x replay and complete-decode manifests are under
`outputs/oracle/transport-continuation-20261026/l1-frozen-pass/`.
This qualifies four-arm L1 on that source, not all nine cases or a global optimum.

## L2 pickup capacity

The first full L2 run accepted3 goals but retained three pins-up loads whose
current grasps could not finish insertion. The only free arm could not provide
the required final continuation. The frozen runner was stopped with original
state/motion/failure records preserved. This is a planning/capacity block, not
proof of physical impossibility.

Before a one-arm socket pickup the new Oracle screens a direct insertion grasp.
If none is found, it reserves a free receiver that can reach the final board;
an unrelated free arm is insufficient. Screened direct jobs retain single-arm
resources. There is no fixed two-arm cap or added task ownership rule. Checks
are cached until action/resource boundaries change, avoiding per-physics-tick
searches. Real pickup, concurrent motion and insertion remain independently
checked. The regression includes unavailable relevant receiver, unrelated free
capacity, direct-grip independence and bounded repeated screening.

## Nominal screen overhead

Profiling the recorded asynchronous collision showed most candidate time in
`mj_forward`, which solves dynamics/constraints not consumed by this FK screen.
Candidate updates now use kinematics/comPos; pose_clear retains its own unchanged
collision buffer and all phase samples. Three same-fixture comparisons retain
the rejection verdict; median0.980→0.423s. This is screen overhead, not claimed
whole-task speedup.73 relevant regressions pass. Already-running frozen worlds
retain their original source; new runs receive the optimization.

## Multi-hop and Cartesian approach gaps

The first eight-arm L1 attempt reached13/24 and stopped at a known-idle frontier.
Its E→D→C route wrongly required middle carrier D to reach C's final board.
Intermediate grasps now screen carrying to the next transfer neighborhood;
only the final receiver screens final placement/insertion. Actual subsequent
handover routes/support remain separately checked. A kinematic regression proves
an intermediate grasp can reach its next transfer despite not reaching the final
board, and checks scratch-state preservation.

Four-arm L2 attempt002 reached8/12 (with one current-state owner retry) before
B_link7/D_link7 contact during the receiver's Cartesian approach. This stopped
and retained the failure. The recorded506.05s fixture reproduces unsafe admission.
TCP paths now follow sequential Cartesian/rotation IK rather than a straight
joint endpoint chord. The wrist pair's nominal minimum was4.70mm despite actual
contact, exposing tracking reserve as well. New full Oracle planning/admission
uses6mm inter-robot clearance while preserving existing socket/payload tolerances.
These are planning margins only; physical geometry, controller and scoring stay
unchanged. The old unsafe command is now rejected. This remains nominal sampling,
not a guarantee against every dynamic or between-sample collision.

## Eight-arm L1 qualification

Source `c5ae460` completed24/24 without owner recovery input:964 robot commands,
zero failed/unknown actions and zero violations,1080.36sim s and1039.76wall s.
Actual multi-arm motion overlap470.00sim s; peak8 moving arms. All64823 source
frames and visual events audited. This is a separate full witness from four-arm
L1 and does not qualify the socket levels or twelve-arm scale.

Pairwise phase screens now recheck the two arms whose poses changed against the
entire scene; unchanged peers need not each repeat their own full-scene scan.
Synchronized samples still check all moving arms. The collision regressions and
a positive retained-load move/state-preservation case pass (76 tests total).
This reduces repeated checks without pruning the selected pair's scene coverage.

## User execution scope and episode limits

The user clarified: finish the nine Transport cases (4/8/12 arms x L1/L2/L3),
then end this task. Do not proceed to GPT, other families or unrelated work.
The user also removed the default whole-episode deadline: a progressing Oracle
must not stop merely because30 minutes elapsed. New full runners default to no
wall/simulation/action-count cap; explicit optional caps remain available. Per-action
timeouts, unknown-action protection, process-violation handling and explicit STOP
remain. This change is prospective; the old twelve-arm attempt ended at22/36
on its recorded1800s limit and is not relabeled as a continuation/full pass.

## Sender release preload

An independent zero-velocity/controller-hold mechanical fixture reproduced the
twelve-arm blocked G opening (target80mm, actual59.61mm). It is explicitly not
an exact physics restart or original-order qualification. Reducing receiver F's
force setpoint70→50N through the ordinary grasp API allowed full80mm sender opening,
with real force-bearing receiver contact, preserved height and zero violations.
The new helper reproduces this result and restores70N after release/withdrawal.
All values stay within the frozen30–70N interface bounds; no servo, friction,
material or force limit changes. Unknown openings are not retried. Resume also
finishes the released sender's withdrawal instead of leaving a stale carrier hint.
Evidence: `engineering/release-preload-002/` and `release-preload-003/` within
the continuation run. The first20N probe was rejected by existing run limits and
is retained as an unsuccessful diagnostic, not counted as qualification.

Pair-dependent geometry checks use exact signed distances plus bounding-sphere
rejection; synchronized/static/self sweeps are retained. On a reconstructed
twelve-arm state both old/new admitted the same small candidate,1.171→0.230s.
This is read-only screen timing, not whole-task speedup.80 regressions pass.

## Readiness cost, stock posture and recoverable preparation

The eight-arm L2 frontier performed156 repeated F_slot2 probes in627wall seconds
(518s in that probe), because unrelated action IDs invalidated the negative cache.
Cache invalidation now follows lease ownership and accepted placements; actual
admitted movement still receives current-scene checks. Matching also charges a
private grip-change heuristic for pins-up stock assigned to a direct socket job,
so existing transfer work can reorient that stock. It is not a feasibility proof.

A twelve-arm retained-load frontier reproduced a recovery deadlock: historical
J/K/L scope demanded K again, although K was released and belonged to another
retained-load job. Explicit recovery scopes now combine requested actors with
all currently retained actors, rather than all historical peers. Default retries
keep only available old peers plus retained actors. Runtime still rejects busy,
unknown or omitted retained actors. New `resume_goal:false` inputs execute bounded
ordinary preparation commands and return a distinct `prepared` frontier. This is
pending work, never task success; retained loads keep their leases and can be
observed/recovered again. No live executor is hotpatched.84 regressions pass.

Additional full qualification: four-arm L3 attempt001 (source7a1cbf4),12/12,
345 robot commands, one known recoverable failed command, zero process violations
or unresolved actions,719.92sim/1015.54wall seconds. All43197 motion frames audited.
Two complete, decoded4x videos are included in the continuation gallery.

## Empty receiver withdrawal at an already close frontier

The live eight-arm L2 E/F frontier has3.46mm positive robot clearance. The6mm
planning reserve rejected even an empty F retreat that increased every initially
close geom-pair distance. A recorded regression reproduces the rejection.
Admission now allows a single empty, unassigned-to-a-load arm in an otherwise
idle runtime to withdraw from a gap of at least1mm: each initially close pair
must monotonically separate (50micron numerical tolerance against its best
sample), every other robot pair retains6mm, and the endpoint must restore6mm.
Loaded, multi-arm and concurrent requests retain the normal6mm reserve. This is
finite sampled engineering screening; native physical collision monitoring stays
unchanged. It does not hotpatch the old executor.

Independent known-pose/zero-velocity mechanical fixture empty-retreat-004 passed:
actual robot gap3.465→27.058mm, ordinary F joint move completed, actual positive-force
E support retained with+0.163mm load-height change, zero violations. It is not
an exact continuation/checkpoint or an original-order qualification. Fixtures001
and002 preserve rejected setup/admission diagnostics;003/004 pass with explicit
correct target metadata.86 regressions pass, including reverse-direction/loaded
rejections and complete live-state preservation.

Only current lease owners now supply runtime target/contact permissions; old
released scopes in failure history cannot overwrite them. An additional guard
rejects empty recovery scopes before admission instead of raising path[0] later.

## Completed twelve-arm L1 and efficiency measurement

Twelve-arm L1 attempt003 (218c862) passed36/36 in one original-order world:
1882 robot commands,6 known failed commands recovered, zero violations/unknown
or unresolved actions;3143.88simulation/4080.84wall seconds. Four owner inputs
make this engineering-assisted.188634 motion frames/checksums/events audited.
Both full4x views are rendered and fully decoded; the gallery now has10 videos.

The user added efficiency goals: reduce total makespan and primitive commands,
increase useful simultaneous arm motion, prefer simple direct pickup/insertion
and air transfers, and quantify remaining optimization room without claiming a
global optimum. `scripts/analyze_transport_efficiency.py` audits the selected
complete witnesses only. It preserves official wall/simulation timings, reports
recorded engineering handback separately, counts real move/gripper/park/refine
commands, and provides explicitly conditional fixed-command scheduling bounds.
The JSON/Markdown results live at the continuation batch root.

For twelve-arm L1, mean moving arms while any arm moves is1.62 (peak12), with
132 explicitly labeled parking commands and707 precision-refine commands.
These categories are optimization candidates, not proof they are all redundant.
The current handover code waits for sender parking before receiver carrying;
whole future routes are leased at pickup. Both deserve measured scheduling
comparison. No near-optimality claim is supported by peak concurrency alone.

Owner recovery now tests compact nearby retreats against the receiver's dense
pickup/insertion and approach route, omitting a full park when proven clear.
The eight-arm L2 A→H supported recovery reduces its owner prefix5→4 robot
commands while retaining the same physical aperture and original acceptance.
These owner recipes remain explicitly assisted, separate from frozen policy code.


## Engineering wait, inventory loss and recovery ownership

Two twelve-arm configurations lost required sideways cylinder stock during long
engineering waits: history records slow passive motion followed by leaving the
admitted workspace. The affected L2 attempt001 and L3 attempts001/002 are sealed
failed; their partial counts are never combined with a restart. Five qualified
full witnesses remain unchanged. Inventory loss detection had been too late.

New runs offer explicit `--pause-on-handback` at a fully idle, known-action
engineering boundary. The physics owner records and acknowledges this pause,
including wall time; owner input resumes the same live controller/world memory.
This is an engineering-assisted protocol, never a real-time policy result or a
checkpoint restoration. Coherent private float64 qpos/qvel observations permit
FK diagnosis without waiting for delayed video chunks. A read-only stock guard
ends an unreachable attempt instead of repeatedly trying impossible pickups.
No geometry, dynamics, force limits, workspace or acceptance rules are changed.

Planning prioritizes sideways rolling cylinders before stable stock and longer
rolling-stock chains first; identity matching also charges measured base-edge
route length. These are risk/critical-path heuristics, not optimal scheduling.
Prepared releases clear bookkeeping only when force is off and actual payload
contact is absent. Automatic retry now requires an available carrier-to-goal
route before replacing the old scope, preserving needed busy peers for later.
96 regression tests pass, including pause-without-physics, inventory detection,
coherent observations, contact-aware bookkeeping and deferred retry ownership.

Mac process sampling (three-second samples, not whole-episode attribution)
placed 71% and 79% of main-thread samples in native MuJoCo `mj_step`, primarily
constraint/sparse-matrix work. Four running worlds used approximately one core
each; single-world physics ownership and Python scheduling do not saturate all
15 host cores. Engineering idle stepping is avoidable cost, now paused explicitly.


## Known grasp faults preserve contact ownership and controller history

Twelve-arm L3 attempt005 ended at530.508sim with K_item3 below the workspace,
13/36 goals and an unresolved F action explicitly reported. The failed receiver
was force-off but still mechanically in contact and had been released for a
different job. Later width-based owner repairs failed and the source no longer
had verified support; the paused pose represented a moving object, not a stable
hold. Resuming unrelated movement advanced that failed physical situation.
A zero-velocity mechanical fixture is not a prediction for that moving state.
Failed attempts and fixture-only evidence remain separate from full successes.

The next source keeps a force-off actor leased while it still touches its own
job's payload. Known close/open/support faults require measured owner recovery
rather than automatic retry on unrelated progress. A terminated affected scope
can trigger immediate engineering review while other known in-flight controller
jobs remain preserved in memory. Coherent qpos/qvel, actual geometric and positive
force support, and immutable review snapshots are saved before the pause. No
active or uncertain action is adopted, reset or replayed by this protocol.

Owner preparations can explicitly retain empty robots with `retain_robots` and
request `pause_after_prepare:true` when `resume_goal:false`. This supports measured
multi-stage recovery without another job taking a receiver during diagnosis.
Explicit owner inputs acquire resources before automatic heuristic retries.
Normal preparations continue to release empty, contact-free actors by default.

The full Transport Oracle now selects50N grasps within the original30..70N bounds
and the already-supported8mm inner grasp tolerance. A2cm lift probe verifies both
positive force-bearing opposed contact and real object rise before continuing
to the original14cm total lift. These are command/planning choices; robot driver,
MuJoCo integration, masses, meshes, friction, task geometry, force limits and
final acceptance are unchanged. The physical-control fingerprint matches the
previous source: ac85b4061b6285cfe66163ad9f2945a5497b72295ca5896dcc88b8228dc13479.
New regression cases reproduce passive-contact lease loss, explicit reservations,
known-fault review preserving a peer action, owner-priority admission and an
unsupported short lift. Prior frozen live worlds are never hotpatched.


## Half-size central opening as a separate rendering default

The user requested half the linear size of the tabletop's central opening,
without changing running experiments. New offline renders now default to
`--table-opening-scale .5`; scale1 retains the original presentation. Only the
four tabletop boxes are changed on an independent FK render model. Outer bounds,
top height, legs, cameras, robot/object/goal poses and registry counts stay fixed.
The four-arm top is solid and stays unchanged. No frozen R13 scene file or live
MuJoCo world is modified. Standalone derived XMLs are saved separately.

Eight-arm opening width/height changes0.952→0.476m; twelve-arm1.666→0.833m.
A side-by-side4x video uses the exact same300 saved states from successful8L1.
The complete recorded streams of8L1,8L2,12L1 were independently checked at60Hz
against the newly filled tabletop volume:1,047,114 states, zero new contacts.
This is recorded-state geometric compatibility, not a continuous collision proof
or a new physical execution/pass. New video overlays say SOURCE PASS and render
only; manifests distinguish presentation geometry from the source task result.
Three regression cases and a compiled-XML parity check verify the tabletop-only
change and preservation of moving coordinates. Existing videos are not overwritten.

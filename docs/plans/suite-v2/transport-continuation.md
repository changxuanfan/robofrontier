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

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

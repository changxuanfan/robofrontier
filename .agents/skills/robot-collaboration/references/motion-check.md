# Robot geometry and candidate motion screening

These optional tools compare robot motion candidates without moving the live world. They can inform whether overlapping paths justify a wait, cancellation, different path or timing. Choose the call when that information could improve the decision; existing evidence may suffice. Role authority and reservations remain unchanged. Coordinator/Workers inspect, and physical execution stays with the robot's owner.

`robot_model {}` returns a compact summary and full static JSON reference; `{"robot":"A"}` selects one robot's transforms, ranges, TCP and collision geoms. READY already supplies capabilities/bounds, and check_motion uses its internal geometry without requiring this description first. Current joints/velocities come through observe; these tools supply robot facts, not object observations.

`check_motion` takes one candidate move per selected idle arm:

```json
{
  "motions": [
    {"actor":"A", "delta_m":[0.01,0,0], "speed_m_s":0.06, "start_after_s":0},
    {"actor":"B", "target_m":[0.40,0,0.68], "speed_m_s":0.06, "start_after_s":0.4}
  ],
  "horizon_s":2,
  "sample_dt_s":0.1
}
```

Targets/deltas use the same world frame and limits as move. Horizon and sample spacing use controller-tick multiples within returned limits. Delays are relative to the source snapshot, rounded up to controller ticks; the report lists actual nominal starts. They are hypothetical, not queued commands. Existing active move goals are predicted alongside candidates; other arms hold. A busy arm cannot receive a candidate. An active gripper returns unknown because fixed-finger prediction does not cover it. One check computes at a time; physical actions and mail continue. Any active gripper currently makes the whole check unknown, including an unrelated gripper. This computational limitation does not establish a team-wide physical conflict.

Returned source_robot contains compact robot facts, without controller buffers. The tool clones robot/controller state, reuses control and IK, and follows joint references with perfect servo tracking. It checks all cross-robot collision geoms, including idle arms, at the specified samples through the horizon. No live object coordinates, contacts, grasp truth or evaluator data enter. Horizon may end before a move finishes; inspect unfinished_at_horizon. Distance may be capped at the report's probe value; a capped value is a lower-bound statement, not an exact nearest distance.

Results are `conflict` (predicted sampled clearance violation), `no_conflict_found` (none at those nominal samples), or `unknown` (unavailable/incomplete prediction). `advisory_only:true` means the result neither grants nor denies action authority; existing role and execution constraints still apply. Inspect closest pair/time, sample spacing, source state/time, live_state_changed and not_checked. Unknown is not clearance; no_conflict_found is not a collision-free trajectory certificate. An unknown result calls for a decision about the missing information or other supported work, not an automatic identical retry or a team-wide wait.

Between-sample collisions, tracking/dynamics error, self-collision, table/obstacles, payloads and future commands are not covered. Repeating this check cannot resolve a question outside that coverage; choose other evidence or a different candidate as needed. Keep relevant constraints for those. The configured clearance threshold is a screening parameter, not a calibrated physical error allowance. A fresh check still needs interpretation; an old cached result retains its original snapshot. Changed starting state, path, timing or commitments require reassessing applicability. Choose candidates, alternatives and useful independent work yourself; a report never automatically enables motion or substitutes for observed handoff clearance.

Candidate moves accept the same position, quaternion/angular-speed or mutually exclusive joint-target parameters as move, plus actor and optional start_after_s. The checker clones the actual orientation/joint reference controller; both targets participate in state freshness. Its short screening horizon is separate from the action execution deadline.

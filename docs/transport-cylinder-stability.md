# Transport R14: side-lying cylinder stability

**Status: confirmed issue; the published scenes and asset archive are not fixed.**

In an engineering reproduction using the published R14 inputs, a side-lying
cylinder in `transport-a12-d2` moved approximately 173 mm in 30 simulated
seconds with the robot controllers holding position and no robot action
requests. Disabling robot collisions did not eliminate the motion. The same
investigation found substantial initial motion in the 8-arm level 2/3 and
12-arm level 3 scenes. These are physics observations, not model scores.

Interpret affected results with care: some objects may move before an agent
touches them. Preserve the exact scene/release hashes and the run recordings;
do not compare results from a future physics revision as if the environment
were identical.

An in-memory candidate that enables cylinder rolling friction greatly reduced
the observed motion, but still showed millimetre-scale movement over 600
simulated seconds. One engineering pickup succeeded. Placement, handover,
full-task feasibility and long-duration stability are not yet validated.
Do not treat this candidate as a published fix or locally alter benchmark
physics while claiming the original frozen release.

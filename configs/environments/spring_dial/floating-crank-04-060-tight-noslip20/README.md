# Airborne Crank Codebreaker

Independent eight-arm / four-module environment. The fixed `crank-04` and floating gear environments remain unchanged. Each free module has a blue rear support handle, a white marked disk, and a red eccentric crank with an orange arrow. Nominal pairs A/B, C/D, E/F, G/H describe layout, not required agent allocations.

The tabletop is 2.60 × 2.06 m with 0.60 m group spacing. The feedback panel remains within the front table edge. Crank arm length is 75 mm; its rigid cylindrical grip is 22 mm in diameter and 44 mm tall, centered 69 mm from the axis. Complete module mass is about 1.190 kg. NoSlip is 20, matching the preserved source apparatus; it is a simulation solver choice, not a policy-adjustable device parameter or a guarantee of hardware equivalence.

One robot must establish a two-finger blue-handle hold and lift the whole module at least 20 mm clear for 0.25 simulated seconds before another robot contacts the red crank. The crank grip follows a circular path and does not have a free-spinning bearing. A support-pose change changes the disk reference and gravity load; command targets alone do not establish the dial angle or retained support. All four modules must remain supported and airborne for feedback. Symbols 1/2/3 correspond to relative 90°/180°/270°, with ±4° feedback tolerance. The passive hinge permits −180°..180°.

`task.json` is the validated source task; policy runs receive its public projection with static device specifications, RGB and robot state. Exact live module poses, hinge states, contact audit and the generated code remain private. The common action tools and role lifecycles are reused; there is no automatic lifting, dial-turning, fixation or oracle-grasp tool. The task may be prepared in single, hierarchical, peer or adaptive mode.

Engineering Oracle records are stored separately in `runs/floating-crank-oracle-20261024`. They are not model experience or a policy recipe. `validation.json` records the final validated scope and limitations. The Oracle captures simultaneous front/rear views plus full per-frame joint state; it does not start a GPT experiment.

The continuous `r3` Oracle passed lifting, concurrent rotation to all three nonzero marks (maximum mark error 2.69°), withdrawal and passive return (maximum residual 0.12°). It recorded no inter-robot contacts, no premature red contact and no sampled blue-handle contact gap. Minimum clearance during supported manipulation was 44.6 mm. Both videos decoded completely.

Earlier failures remain recorded: `r1` exceeded the support arm's verified IK range; `r2` reached all marks but could not fully open in place. `r3` stops force-held grasping, withdraws axially, and then verifies full opening. This validates that release path; it does not establish that arbitrary in-place opening or every support orientation works. Physical dimensions, friction, NoSlip, spring and controller gains were unchanged between these trials.

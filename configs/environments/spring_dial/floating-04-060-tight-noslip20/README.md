# Airborne dials · compact table · NoSlip 20

This is a new physics variant, not an in-place modification of the earlier trial. Its source is the 0.60 m group-spacing / 2.60 m symmetric table scene, with the CORRECT panel inside the front edge between front-row robots C and E.

`variant-provenance.json` and the structural regression establish that the only scene XML change is `option.noslip_iterations: 0 → 20`. Initial keyframe, robot and object geometry, masses, spring, contact friction, actuator limits and gripper controller are unchanged. The task's strict lift-before-knob-contact contract and all policy observation boundaries remain unchanged. NoSlip is a frozen engineering solver configuration; no Agent action can change it.

The independent 60-second hold and low-friction failure controls are in `runs/floating-long-hold-diagnosis-20261024`. They are engineering evidence, not GPT experience. NoSlip suppresses soft-contact drift but does not guarantee hardware equivalence or stable contact for every trajectory. The first full Oracle retained the older support-roll trajectory and lost grasp during that roll; it is preserved as a failed reference, not hidden or scored as model behavior. A second reference tests a friction-supported airborne grasp without that additional roll. Full turning and release must pass before the new policy run starts.

Physical validation and policy trials are stored separately. No private grasp coordinates, reference trajectories, contact measurements or hidden code are copied into policy inputs. All four agent architectures use the same prepared task/physics when selected; this pilot uses Adaptive gpt-6.1-sol/high.

Full reference r2 passed: airborne pinch support, all three nonzero symbols within tolerance, passive release, no inter-robot contacts or premature knob contacts. The earlier failed roll reference is retained. This route is not imported into policy instructions.

# Floating dials · compact engineering scene

Eight arms, four movable modules. Horizontal pair pitch is 1.10 m (previously 1.60 m); table length is 5.40 m (previously 6.90 m), depth remains 2.06 m. Each facing pair keeps its proven relative geometry. The wide environment and sealed policy trial remain preserved. Only layout and static overview framing changed; device mass, friction, spring, gripper drive and gravity are unchanged.

`preview-global.png` shows the new layout. `oracle-path-screen.json` records a 4 Hz geometric replay of the earlier contact-based Oracle: 373 saved configurations, no inter-robot contacts. This is not a new full dynamic Oracle or a policy result, and does not certify the intervals between samples.

No new GPT experiment has started. Before the next formal run, its task contract must state that a support arm lifts and stabilizes the whole module before the partner touches, grasps or turns the red knob; an apparent gripper success is not stable payload support. Adjusting the blue handle reorients the complete module and does not set the code digit. Relative red-to-white angle defines the digit. A loss of support requires re-establishing the permitted phase before continuing.

This is a task-specific physical sequence, not a universal per-robot schedule. Generic Skill guidance should cover load moment, support orientation, observation and recovery; exact grasp coordinates and the Oracle procedure remain engineering-only. A private process monitor is needed to distinguish premature knob contact from merely invalid code feedback. The existing feedback gate alone does not enforce that sequence; this scene is not a newly validated formal task package yet.

# Airborne Dial Codebreaker · 0.60 m pair spacing

A separate eight-arm, four-free-module scene. Horizontal group pitch 0.60 m; table length 3.90 m, depth 2.06 m, top height 0.40 m. Opposed robots within each group retain their earlier relative placement. Earlier fixed, wide and 1.10 m environments and sealed runs remain unchanged.

## Public task

The digits are relative red-rotor/white-disk symbols 1/2/3; zero is reset. A separate robot must maintain a two-finger grip on the blue handle and lift the whole module at least 20 mm clear of the table continuously for 0.25 simulated seconds before a partner initiates red-knob contact. All four supports and airborne prerequisites remain required for a valid count. The blue handle changes the whole disk frame rather than the password digit. The task offers qualitative load-moment and access hints; exact grasp coordinates, wrist angles and Oracle trajectories are not policy input.

The public task explicitly declares the process rule: premature knob-contact initiation makes this episode process-noncompliant even if the puzzle is subsequently solved. Physical execution continues and records evidence; no hidden reset or automatic retry occurs. Normal contact physics is unchanged. Private final scoring reports puzzle solution and process compliance separately. Post-win cleanup is excluded. Losing support invalidates code feedback, and a newly initiated red contact must re-establish the prerequisite. Agent-directed recovery retains the task's preconditions.

## Evidence boundary

Private sampling records apparatus pose, support contacts and premature contact onsets at the existing physics boundary. It returns none of those measurements or exact live object states to the policy. The display remains aggregate count only. Robot-only IK/CAD, robot proprioception and calibrated fixed RGB views remain available. A pose/gripper command completing does not certify payload support. All four agent modes use this same physical/evidence contract; Adaptive splitting remains a policy decision.

## Engineering verification

The contact-based reference at `runs/floating-puzzle-060-oracle-20261023-r1` advances the actual dynamics, without LLM or module/dial pose writes. It tests four groups concurrently, all nonzero marks within the ±4° task tolerance, passive return, support retention and inter-robot contacts. This is engineering feasibility evidence, not a model result or proof every policy trajectory is safe.

# Airborne Dial Codebreaker · 4 modules / 8 arms

A separate eight-arm scene; the fixed `facing-04` and two-arm `floating-module-01` assets remain intact. Robots A/B, C/D, E/F, G/H initially face modules A, B, C, D respectively. Associations identify the layout and do not prescribe allocation or agent count.

Each module is a free rigid disk/blue handle with a passive red spring rotor. Real gripper contact provides lifting, stabilization and turning. There are no dial actuators, welds, pose resets during execution, or task-specific manipulation macros. Disk diameter 230 mm, thickness 8 mm; eccentric handle 300 × 16 × 20 mm; red knob core diameter 54 mm and height 40 mm. Nominal module mass 1.17 kg. Gravity and contact remain active.

Symbols 0/1/2/3 are attached to the white disk. The rotor's relative hinge is clockwise-positive viewed from the printed front, with mechanical travel ±180°. Hidden code positions are independently sampled from 1/2/3. Four stable nonzero settings and all modules completely at least 20 mm above the table are required for aggregate feedback. Angle tolerance ±4°, maximum rotor speed 5°/s, stable duration 0.25 sim seconds. Win 4 latches; physics keeps running. Grounded/transient inputs show a dash and do not create a code trial. Grounded dial travel is retained privately as a diagnostic; the evaluator does not secretly impose a whole-run failure for an incidental contact.

`task.json` is the engineering source; policy runs receive an explicit public projection with dimensions/rules, fixed robot/camera calibration and RGB, without live apparatus poses, hinge angles, contact evidence, individual correctness or the code. Exact initial module centers, source XML, physics settings and Oracle reports remain on the engineering side. Shared filesystem access is a protocol boundary rather than an OS sandbox.

Fixed cameras: global/global_side/top, board (aggregate display), front/side for each robot, four detail angles for each module. Camera selection, grasping, approach, support orientation, parallelism and Adaptive scope are Agent decisions.

Oracle evidence is separate in `runs/floating-puzzle-oracle-20261023-r1`. It tests physical contact, all three nonzero marks, airborne holding and passive return. It is engineering validation, not a model score or a policy recipe.

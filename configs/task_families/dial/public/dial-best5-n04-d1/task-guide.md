# dial-best5-n04-d1

Unlock the spring dial code by finding and simultaneously holding one correct nonzero input. Minimize execution time and valid attempts using only historical BEST feedback; preserve real grasps and required airborne support.

Read task.json for this case’s static geometry, mechanism, cameras and completion criteria.
Read family-guidance.md before planning manipulation or interpreting BEST/READY.

## Task rules

1. The face has0,1,2,3,4,5 at60-degree intervals. Each private password digit is1..5;0 is an invalid initial position. Clockwise labels4 and5 may be reached as-120 and-60degrees; +180 and-180 are the same mark3.
2. BEST starts at0 and shows the largest correct-count achieved by any one complete valid input in this episode. It is not the current correct-count. An unchanged record can mean equal or worse; never combine per-dial results from different attempts.
3. READY is grey when the current input is invalid, amber while qualifying and green after every dial is within4degrees of a nonzero mark, actually grasped, and stable for0.25seconds. Green means a valid input, not a correct password.
4. The BEST digit is bright only when READY is green. Otherwise its retained value stays visible in dim grey-brown, including while the amber lamp is qualifying. Brightness does not change the record or disclose current correctness.
5. Only a complete valid input updates BEST. Off-mark, zero or lost grasp makes READY invalid while BEST remains. New stable configurations are evaluated automatically, including intermediate configurations during movement; bind a record change to an actually observed configuration, not an assumed target.
6. L2/L3 require a separate robot to hold each blue handle with the whole module at least20mm clear of external supports. Maintain that support during red operation. A crank grip moves around the axle and may require coordinated translation.
7. Unlock occurs only when all digits of one simultaneous valid input are correct. Success latches, while physical springs and support remain active. Inputs and actuation choices are Agent decisions.
8. Only robot actuators and real contacts may affect the live world. Use RGB and permitted robot feedback; hidden passwords, per-dial correctness and live object truth are not policy inputs.

# Fixed spring dials with red crank handles

This is a new copy of the fixed four-dial table scene. The original gear-knob scene remains unchanged. Each white dial is fixed to the table; its passive red rotor carries a radial arm and a vertical grasp cylinder. The grip is 22 mm in diameter and 44 mm high at a 75 mm radius. Hub, arm and grip retain the original 0.12 kg rotating-assembly mass. Dial hinges, springs, markings, cameras, table and robot placement are otherwise unchanged. There is no dial-angle actuator.

## Oracle evidence

`runs/spring-crank-oracle-20261023/r13/report.json` records a four-arm contact test. Each Panda grasped its crank with both fingers and turned the relative dial angle to 90° within 0.83°; the longest sampled two-finger contact gap was 2 ms and the Oracle detected no inter-robot contacts. The video is `runs/spring-crank-oracle-20261023/r13/oracle-crank-four-dials.mp4`.

Opening the grippers fully after the turn was not validated. The first open request stopped at 74.1 mm against an 80 mm target, so spring return after release remains unproven. This does not invalidate the confirmed grasp-and-turn result.

The GPT task is a separate new run, recorded under `runs/spring-crank-adaptive-sol-high-20261024-r1`; the prior gear task and its runs were preserved.

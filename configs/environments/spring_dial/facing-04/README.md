# Spring Dial Codebreaker · four arms / four dials

Environment and policy adapter, built 2026-10-23. No LLM trial has started.
Existing beam and mosaic environments are unchanged.

## Inspect

From the repository root:

```sh
.venv/bin/python -m scripts.build_spring_dial
.venv/bin/mjpython -m scripts.view_spring_dial
```

F6 cycles board / global / top / side / individual-dial views. F7 resets the whole
preview and samples a new private code. Closing the window exits normally. Mouse
perturbation in the native viewer is an engineering aid, not an agent action.
No keyboard arm teleoperation or policy sessions are launched by this preview.

`preview-board.png` emphasizes readable game state; `preview-global.png` shows the
four-arm arrangement. A and B face each other on the left; C and D face each
other on the right. All knobs are red with orange indicators, on the original
brown table. Small letter plates identify the devices; there are no black title
or instruction boards. `warm`, `silver` and `panel` are cosmetic alternatives.
The user selected `panel` (03): its default now has a 290 mm visual white face,
bold black A/B/C/D labels, a 128 mm orange indicator and English `CORRECT / 4`.
The degree numerals are printed in bold black outside the white circular faces;
the pointer can extend to the rim without covering the numbers.
The fixed collision base and graspable knob retain their original dimensions;
the face and indicator are noncontact visual readouts.

Generate the three actual MuJoCo renderings and compare their physical models:

```sh
.venv/bin/python -m scripts.render_spring_dial_designs
```

The images/scenes are in `runs/spring-dial-designs-20261023/`. Native previews can
load any candidate with `--directory runs/spring-dial-designs-20261023/silver`.

## Apparatus

One 1.90 × 2.06 m table, four imported Panda robots and four fixed-base passive
hinges. Each knob has a 48 mm core, shallow physical scallops (approximately
54 mm overall), 44 mm height and 120 g rotating mass. A larger stationary face
provides legible degree markings without exceeding the gripper's 80 mm opening.
The orange indicator is a non-colliding optical marker tied to the hinge, not a
graspable lever. Metallic appearance does not determine contact friction.

The dial labels are symbols: 0 north, 1 east, 2 south, 3 west, corresponding to
0/90/180/270 degrees. Clockwise is positive from the top view.
The physical range is −180..+180 degrees. Left 90 is displayed as 270; either
180-degree endpoint has the same code meaning. There is no continuous winding
through the endpoints. A torsional spring of 0.03 N·m/rad and damping of
0.003 N·m·s/rad return the knob continuously toward zero. There are no dial motors,
nonzero detents, welds, grasp attachments, resets on release or angle teleportation.
The existing arm and force-grasp controller supplies all manipulation.

## Game and information boundary

Each private code contains four independently sampled entries from {1, 2, 3}:
81 candidates, with repeated symbols allowed and zero excluded.
All four actual dial angles must lie within 4 degrees of a nonzero mark, move no
faster than 5 degrees/s and remain stable for 0.25 simulated seconds before the
apparatus displays the number of correct entries. Invalid/transient inputs show
a dash, not a zero score. Continued holding does not repeatedly count queries;
leaving and re-entering a valid stable input creates another trial. Four correct
is the winning score; this does not lock the hinges or freeze physics.

The secret is sampled in the preview's private apparatus and is absent from XML,
textures, parameters, environment and public task. The display reads actual
hinge state internally, like a physical game controller with encoders. Policy
observations are intended to remain RGB, robot proprioception and action status;
there is no public exact-angle/secret/per-dial-correctness endpoint. The shared
filesystem is not an OS security boundary. Formal runs generate the code once in
`private/inputs/spring-code.json`; it is covered by the private integrity manifest.
Code, per-dial correctness and dynamic angles are never returned by policy tools.

The scene builder and private engineering validation are distinct from policy
experience. `task-draft.json` is descriptive; generated `task.json` is the canonical
registered policy task. The current task withholds exact fixture-axis coordinates;
Agents localize the dials from RGB. Static robot configuration, camera calibration
and declared fixture dimensions remain public. Dynamic angles are always private.

## Prepare a policy trial

Use a new run directory and the chosen existing topology configuration:

```sh
.venv/bin/python -m scripts.create_pipeline_run runs/NEW-RUN \
  --task configs/environments/spring_dial/facing-04/task.json \
  --harness PATH-TO-HARNESS.json
```

Preparation creates no model sessions or robot actions. An engineering-only
`--private-puzzle PATH` may supply `{"code_symbols":[1,2,3,1]}` to pair trials;
never hand that file to a policy role. Omission samples a private code uniformly.
The generic dispatcher, permissions, mailbox and adaptive lifecycle are reused.

Physics updates the apparatus every 2 ms after START; count/display state derives
from measured joints, not commanded angles. Sparse private audit samples and exact
trial-entry events are written to `private/outputs/spring-dial-process.jsonl`.
Unobserved intermediate stable configurations still count. A sampling gap marks
the evidence incomplete, rather than silently inventing an extra trial. An
incomplete/interrupted run must be separated from model failures in analysis.

First valid score 4 latches the visible success digit and the private result.
Normal finish freezes scoring/metrics, not the physical springs. Native view uses
`global`; the main recording uses `board` so the score is legible, while Agents
choose allowed camera observations themselves. Video is named `board-realtime.mp4`.
Observation JSON contains robot state and timestamped images, not a structured
score/angle endpoint. Run-local experience remains mutable; frozen instructions
and engineering reference corrections are not policy experience.

## Validation and limits

`validation.json`: stable zero initialization, finite state, 32 robot actuators,
zero dial actuators and no initial penetration exceeding 1 mm.

`runs/spring-dial-engineering-20261023/attempt-02/report.json`: fixed-program
contact witness using the existing PandaArm controller. All four robots together
grasped and held 90, −90 (270 display) and 180 within the game's tolerance, then
released and returned to zero. No inter-robot penetration above 0.5 mm was found
in that witness. It uses private angle measurements to correct engineering
motions; it is neither an RGB agent demonstration nor an exhaustive collision
certificate. Arbitrary held-angle transitions and alternate grasps remain untested.

The earlier attempt and its snapshots are retained in the parent engineering
directory. An initial open-loop wrist rotation of 90 degrees produced about 83
degrees at the dial, demonstrating why command completion cannot certify dial
alignment. Its report-writing error was corrected before the second witness.

Seven focused feedback tests cover invalid inputs, zero score, signed angle
equivalence, stable-entry counting, partial changes, motion and sampling gaps:

```sh
.venv/bin/python -m unittest discover -s tests -p test_spring_dial.py
```

The manipulation witness is separately repeatable with a new output directory:

```sh
.venv/bin/python -m scripts.verify_spring_dial --output runs/spring-dial-check-new
```

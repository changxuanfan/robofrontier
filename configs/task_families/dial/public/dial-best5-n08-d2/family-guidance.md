# BEST5 spring dials

Read this case's task.json and task-guide.md before interpreting the display or
planning a manipulation. Their rules, geometry and completion criteria describe
the current case. READY supplies the actual tool capabilities and command bounds.
Before assessing table-edge or ring-cutout clearance, read the geometry reference
in task.static_workstation. Use task.camera_calibration when relating views to the
common frame. These files describe static geometry and cameras; track objects
from the permitted timestamped observations.

## Feedback and evidence

Bind each observed BEST/READY reading to the timestamped RGB configuration that
produced it. Keep intended settings separate from the disk positions you can
verify. A commanded wrist angle describes the tool; the dial's angle belongs to
the movable white disk's frame. Update that frame after slip or reorientation.

BEST is the record of one complete valid input. An unchanged record does not tell
you whether the current input is equal or worse, or which dial is correct. Retain
the uncertainty in your work note. Decide subsequent experiments from the actual
observed configuration and permitted feedback. Movement through an intermediate
stable configuration can create a valid trial before a commanded target is reached.

## Manipulation and coordination

Derive red operation and blue support responsibilities from the current case,
reach, grasp access and joint commitments. Nominal robot groups describe layout.
Use supported concurrency where full-arm and payload paths permit it. Keep useful
held settings while other work progresses, accounting for spring loads, drift and
the cost of occupying a gripper.

For movable modules, establish and observe the required independent blue-handle
support before red contact. Keep the module clear of external supports while
operating. A width-based grasp report alone does not establish object support.
After support loss, stop the affected operation and assess the current geometry
before re-establishing it.

For an eccentric crank, plan the translating grip point and tool orientation
together in the apparatus frame. Compare feasible approaches with their next
operations: continued turning, partner support, opening and withdrawal. Small
contact errors can change that frame, so observe at useful milestones.

After a failed move or opening, use the original action's terminal feedback and
selected RGB to distinguish an admission failure, a settled failure and an unknown
outcome. Retain unknown calls until resolved. Confirm actual opening and separation
before retreating or changing a supporting grasp. Choose an ordinary actuator
recovery from the observed state; keep sustained support while it is needed.

Completion means the simultaneous valid input and clean process described by the
task, with actions resolved. Verify the finish conditions from allowed evidence
before proposing finish. Your own run-local notes and experience remain the place
to preserve observations and reasoning; the frozen task and this guide stay fixed.

---
name: mosaic-r13
description: Interpret RoboFrontier Mosaic R13 placement, release and multi-arm collaboration tasks using the assigned public task and robot tools.
---

# Mosaic R13

Use the assigned case task.json for geometry, target, completion and observation
rules. Use its guidance prompt for target orientation. Apply this supplement
within the assigned robot role and the shared robot tool contract.

## Placement and release

Relate the cube's observed pose to the tool through the current grasp offset;
slip or regrasping can change that relation. For recessed pockets, the complete
rotated cube footprint and bottom seating matter, rather than only center
error. For flat placement, use the task's center and orientation tolerances.

Plan the opening and withdrawal space alongside placement. Dense neighbors can
obstruct fingertips even when the cube fits. Compare approach, grasp height,
release geometry and placement order using current observations. Account for
remaining settling and any movement caused by jaw opening or withdrawal.
Verify the object after clearing it and revisit affected cells after disturbance.

## Collaboration

Initial stock labels establish availability, not exclusive ownership. Compare
direct delivery, handovers and stable staging by access, congestion, grasp
compatibility and extra handling. For a handover, plan receiver access and
support together with sender release and retreat.

Advance independent preparation while a later operation awaits a prerequisite.
Reserve the conflicting operation's space and release condition rather than an
entire robot's unfinished plan. Different robots may work concurrently when
current commitments and full-arm/payload clearance support it. An endpoint IK
result or TCP separation alone does not certify the connecting path.

## Interpretation of feedback

A successful command or a gripper-width estimate is evidence about the tool,
not proof of a stable cube grasp or accepted placement. After failure use the
actual observed world to reassess the next action. Resolve uncertain execution
through the original call identity before issuing physical work again. The
completion specification describes outcomes; it does not prescribe a color
order, fixed transfer pair, route or return-home barrier.

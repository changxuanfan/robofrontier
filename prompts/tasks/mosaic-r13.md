# Mosaic R13 task supplement

Read the assigned case's task.json and target-pattern.png. Numeric geometry and
completion conditions live in that task file; use its actual robot bases,
workspace, cameras and target dimensions rather than dimensions from another
case. The task file names the Mosaic skill supplement for manipulation-specific
interpretation. The assigned role and shared robot tool contract govern execution.

The supplied pattern matrix and target image already have their final world
orientation: row 1 is north (+Y), column 1 is west (−X). Historical rotation or
mirroring descriptions explain how that reference was made; use the supplied
reference as-is. Background-colored cells are part of the artwork.

Every target cell needs a distinct released cube of its required color. Source
labels describe initial inventory, with shared access to supplies. Select the
allocation, order, temporary staging and handovers from current observations,
reach, access and interactions. L1 uses real recessed pockets; L2 printed lines
are visual marks; L3 has no visible grid, with the same declared target frame.

Use the public completion specification to judge the final result. Placement,
release and withdrawing the gripper are separate events. After contact or later
work can disturb a completed cell, inspect the affected result again. Preserve
verified work while choosing useful concurrent prefixes and avoiding conflicting
swept space, including the payload and opening fingers.

Track objects through permitted timestamped RGB and robot/action feedback.
Public target coordinates and a commanded TCP do not measure an object's actual
pose. Task geometry and this supplement contain no precomputed action sequence;
choose and revise motion from the current allowed evidence.

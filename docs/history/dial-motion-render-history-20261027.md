> 历史记录：本文保留当时过程，不作为当前操作指令。当前规则、默认值和入口见 [旋钮家族说明](../tasks/dial-best5.md)。

# Recorded motion editing and factory replay

Original motion is immutable 60 Hz float32 `qpos`, including robot joints,
individual fingers, free objects and passive/articulated joints. Timestamped
appearance events preserve the score and READY display. An idle edit is a
derived JSON index into that original recording, shared by all renderers and
cameras. It does not alter Oracle outcomes or physical completion times.

`robofrontier.motion_edit` measures one-second excursions of robot joints,
gripper aperture and object translation/rotation. Defaults treat an interval as
idle only when all channels remain within these bounds for at least two seconds:

| Channel | Excursion bound |
| --- | --- |
| Each arm joint | 0.3 degrees |
| Each finger slide | 0.25 mm |
| Object position | 1 mm |
| Object rotation / passive hinge | 0.75 degrees |

Quaternion signs are normalized. Coordinate bounding-box diagonals conservatively
bound multi-axis excursions. A whole-interval excursion check retains slow real
drift even if each short window alone is below threshold. Short pauses are kept;
long idle intervals retain 0.2 seconds of context at their edges. Appearance
updates retain their source samples and 0.25 seconds of context. Actual arm
motion, finger closure, appreciable spring return and drops remain visible even
when other parts are still. These are explicit, adjustable geometric thresholds,
not a claim that an algorithm can infer every action's intent.

The edit contains original source frame intervals/timestamps, their concatenated
edited times, removed intervals, all settings and hashes of the source scene,
recording index and events. Readers reject an edit bound to another source.
`EditedMotion.frames()` exposes original frame index, original time, edited
time and the original float32 state. No second copy of all motion data is needed.

`EditedMotion.video_indices(fps, speed)` creates the common output frame schedule.
It samples retained source frames only, preserving interval boundaries and
appearance changes explicitly. No interpolation bridges an edited-out interval.
Both paired videos render this identical schedule. At accelerated playback,
extra boundary/feedback frames slightly extend duration relative to a strict
constant-speed video; the speed label is the nominal motion speed. Each video
frame's original index and timestamp is listed in the render report. The source
simulation time remains visible in the caption, including jumps over removed time.

Dial-family presentations default to **10x in both backgrounds**, per the user
on 2026-10-27. Other families retain their existing caller defaults (4x in the
generic simple Oracle renderer, 1x in generic factory replay). Explicit speed
arguments remain available. The dial-only batch defaults to 10x/30 fps.

Create an edit with:

```bash
.venv/bin/python -m scripts.edit_recorded_motion /absolute/trial/motion \
  --output /absolute/edits/case.json
```

Render paired simple/factory videos with:

```bash
.venv/bin/python -m scripts.replay_motion_in_factory /absolute/trial/motion \
  --factory /absolute/factory/scene.xml --edit /absolute/edits/case.json \
  --output /absolute/new-output --render --fps 30 --speed 10
```

`scripts.render_frontier_oracle --edit` uses the same schedule for additional
simple-scene camera views. `scripts.render_edited_dial_batch` renders a nine-case
selection, verifies schedule identity and full video decoding, and publishes
flat `simple-videos/` and `factory-videos/` folders.

Factory replay uses the exact frozen recorded workstation revision. It restores
source FK poses and transforms the complete station by one translation and yaw,
with scale fixed at one. Fixed tables, robot bases, moving objects, links,
fingers, display and the camera share that transform. Different factory
materials, lighting or surrounding decoration do not change the edit. Composer
`--placements` accepts a JSON mapping from case ID to `translation_m` and
`yaw_rad`; geometry/scale changes are rejected. The operator must choose a
placement with adequate clearance; arbitrary relocation is not a collision or
floor-support certificate. Source geometry/layout changes require a new source
revision, not silent reuse of old motion. Replay invokes no robot command, IK,
contact integration or `mj_step`.

## Nine-case output, 2026-10-27

The selected closed episodes are from
`runs/oracle/dial-best5-recovery-20261026/batch-status.json`.
Edits are under `outputs/oracle/dial-motion-edits-20261027/`.
The factory presentation uses those recordings' own frozen `motion/source/`
directories, not the older nine-dial factory revision.
Paired videos and their time mappings are under
`outputs/robo-factory/dial-best5-edited-10x-20261027/`.
The source-matched factory composition is under
`outputs/robo-factory/dial-best5-edited-20261027/scene/`.
The preceding 12x presentation batch was superseded, with eight completed
pairs retained and its final video rendering interrupted after the speed change.

Default settings removed 9.617 seconds from four-dial L3, 2.267 seconds from
six-dial L3 and 6.2 seconds from eight-dial L3 (18.083 source seconds total).
The other six cases have no sustained idle interval under these settings.
This differs from the earlier video-frame-only trim: detection now runs on the
full 60 Hz motion before accelerated 30 fps rendering. The prior videos remain
in their original output directories. Eight-dial L3 remains labelled assisted.

All nine final 10x pairs completed. Every video was fully decoded and its actual
decoded frame count matched the common source schedule; all factory geometry
positions matched source FK plus placement within 5.03e-15 m. Seventeen relevant
tests passed. `verification.json` and `edit-audit.json` in the final output root
record the checks. The original flat nine-video delivery folder now contains
the 10x simple clips; its previous12x contents were retained in the sibling
`dial-nine-videos-12x-previous-20261027/` folder.

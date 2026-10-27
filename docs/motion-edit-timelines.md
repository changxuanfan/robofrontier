# Recorded motion edits shared across backgrounds

Keep the original `robofrontier.motion.v1` recording immutable. A
`robofrontier.motion-edit.v1` JSON file references that recording by case, frozen
scene SHA-256, and motion-index SHA-256. It contains two related selections:

- `render_frames`: exact reviewed output frame → original source frame/time.
  The output clock is `output_frame / fps`. The original clock retains jumps
  across removed waits. This selection reproduces the reviewed 30fps, nominal
  60× video in any background without running motion detection again.
- `keep_source_ranges`: inclusive original-frame windows. These retain the
  full recorded cadence within each window. `MotionEdit.kept_source_frames`
  yields `(edited_time_s, original_time_s, qpos)` without copying the dataset.
  This is the input for future alternate-rate resampling; existing factory
  video replay uses the exact `render_frames`, not a new resampling policy.

Range boundaries follow the original video's sampling grid (normally 120
source samples at 60Hz for a 30fps/60× presentation), including the forced final
sample. They are not precise subframe estimates of the start/end of an action.
The logical full-rate view uses its own nominal-rate edited clock; original
sample times remain available, including physics-step quantization and a
possibly short forced final sampling interval.

The current edits derive from the reviewed simple-scene videos, including the
second image-based pass that ignores the time overlay and invisible gripper
jitter. These are fixed presentation decisions, independent of the destination
background. They are not an assertion that every omitted physical state was
strictly motionless: tiny or occluded motion may be omitted. The original
recording is the authoritative complete trajectory and is never modified.

## Export and reuse

```sh
python -m scripts.export_mosaic_edit_timelines \
  --folder outputs/oracle/mosaic-continuation-20261026/videos \
  --output outputs/oracle/mosaic-continuation-20261026/videos/轨迹剪辑时间表

python -m scripts.replay_motion_in_factory RECORDING_DIRECTORY \
  --factory MATCHING_FACTORY_DIRECTORY/scene.xml \
  --edit-timeline CASE_TIMELINE.json \
  --output NEW_VIDEO_DIRECTORY --render

python -m scripts.export_factory_recorded_pose RECORDING_DIRECTORY \
  --factory MATCHING_FACTORY_DIRECTORY/scene.xml \
  --edit-timeline CASE_TIMELINE.json --output-frame 100 \
  --output NEW_POSE.json
```

The replay renders simple and factory views from the same ordered source states
and exact output cadence. Appearance events crossed by a cut are applied up to
the selected original timestamp. The pose export supports downstream Blender
presentation using the same output-frame numbering. Neither path steps physics,
solves IK, smooths states, or fabricates a restart checkpoint.

A factory must be composed from the recording's frozen workstation revision.
The older v6 factory contains r10 Mosaic base layouts; these do not match the
successful recordings and are deliberately rejected. The generated
`outputs/robo-factory/v6-mosaic-recorded-20261027/scene.xml` uses all nine actual
frozen recording sources instead. Other factory workstations remain from the
existing catalog. The catalog used to compose it is retained under
`runs/engineering/mosaic-edit-transfer-20261027/factory-recorded-catalog.json`.

Factory placement applies the same translation and horizontal rotation to the
entire workstation: table, fixed arm bases, animated links, and objects. No
hardware is scaled. Background, lighting, and materials may differ. Changing
relative base/object positions, table dimensions, or task geometry requires a
new validation; a rigid replay cannot make an old trajectory valid in a newly
arranged physical task. This is visual transfer, not a new factory dynamics or
collision test.

## Verification

`python -m scripts.check_mosaic_edit_transfer` verifies every original recording
chunk and every selected timestamp for all nine cases. It checks source hashes
against factory bindings and tests factory geometry poses at the initial,
middle, and final selected states, plus both sides of the previously reported
8-arm L1 static wait. Optional `--render-case mosaic-a08-12x24-d1` produces four
simple/factory comparison images around that cut. These checks do not claim
that nine full factory videos have been exported.

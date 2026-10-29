# Transport R14: cylinder rolling-friction candidate (2026-10-29)

This version changes Transport cylinder collision parameters to `condim=6`
and rolling friction `0.01`. It is a new physics revision, not the original
R14 physical environment. Identify runs by their frozen task release and scene
hashes; do not pool or directly compare results across these physics revisions.
The original GitHub release asset has been removed; old commits require an independently retained matching backup.

In the original `transport-a12-d2` scene, an untouched side-lying cylinder
moved about 173 mm over 30 simulated seconds. In the updated scenes, a
30-second passive test across the six cases containing cylinders measured at
most 0.217 mm horizontal cylinder movement; the three remaining Transport
cases have no cylinders. The 12-arm Level 2 candidate showed up to 4.362 mm
displacement across all movable bodies in a separate 600-second test. A single
engineering pickup of one cylinder also succeeded. These measurements do not
certify full-task success, grasping all objects, handover, insertion, or
long-duration stability. No original Oracle pass is inherited for this physics.

For the passive engineering check, prepare fresh Transport jobs and run:

```sh
python -m scripts.tasks prepare-batch --output runs/engineering/transport-stability --families transport
python -m scripts.check_transport_cylinder_stability --batch runs/engineering/transport-stability/batch.json
```

This check uses simulation time, starts no model, and does not execute a task.
If another experiment has already prepared jobs, do not modify its frozen
inputs; prepare a new batch for this release.

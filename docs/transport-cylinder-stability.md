# Rolling-friction 0.03 engineering candidate (2026-10-29)

This policy physics revision sets Transport cylinder collision geoms to
`condim=6`, rolling friction `0.03`, and Dial module disk/stem collision geoms
to `condim=6`, rolling friction `0.03`. Dial modules without a stem modify the
disk only. Table friction, Mosaic blocks, Relay objects and other contact
parameters are unchanged. This changes the physical environment: identify runs
by their frozen scene and task-release hashes. Do not combine scores with
previous physical revisions as if they were the same benchmark. Historical
Oracle success and prior compiled-physics parity are not inherited.

Earlier in-memory engineering probes on `transport-a12-d2` H_item3 found that
0.03 reduced post-release movement relative to 0.01 in some force/direction
tests; 0.03 did not prevent large movements in all tests. One side-cylinder
pickup and lift passed. In `dial-best5-n04-d2`, one 0.03 module force probe
and a lift/knob-capture probe were favorable, and a hinge spring-return probe
still worked. These are limited, case-specific probes, not complete model runs.
Transport insertion, handovers, full Dial turns, other forces and long-duration
cross-platform behavior remain unqualified. This release is for further
engineering tests; it does not establish a formally validated benchmark.

For an independent finite passive Transport check, prepare fresh jobs and run:

```sh
python -m scripts.tasks prepare-batch --output runs/engineering/transport-stability --families transport
python -m scripts.check_transport_cylinder_stability --batch runs/engineering/transport-stability/batch.json
```

This check uses simulation time, starts no model, and does not execute a task.
Do not modify existing frozen experiment inputs; prepare a fresh batch.

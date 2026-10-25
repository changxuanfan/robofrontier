# R10 engineering and bounded Oracle batch

Date: 2026-10-25. This is an implementation record. The earlier
`factory-oracle-next.md` remains the historical launch plan.

All 36 sources now have independent R10 environment directories, selected by
`configs/suites/frontier-v2-r10.json` and the current suite index. Old revisions
and records are retained. The common tabletop is 0.80 m; world-space geometry,
free bodies, initial states, cameras, fixtures, task bounds and scoring height
references moved together. Legs lengthened; floor stayed at Z=0. Sizes, masses,
inertias, friction, joint axes and 2 ms integration settings are unchanged.
The ground drop distance therefore differs from the old 0.40 m revision; these
are new source hashes, not relabelled old experiments.

## Factory and replay

V6 uses a 55 by 50 m building and four family regions. It contains the actual
36 R10 layouts, brown textured tabletops, underbench drawers/control units,
shelves, bins, cartons and limited floor pallet islands. The 91 dressing groups
are display-only, outside tabletop holes and source planning envelopes. Source
motion still needs swept-volume inspection before a final long factory video;
a static non-intersection check is not a dynamic certificate.

Lighting/exposure was compared at fixed cameras; the selected exposure is 3.7.
This improves the earlier pale appearance without claiming photographic
realism. Aesthetic approval remains a visual judgement. Current images:
`outputs/robo-factory/v6-final-previews/`. Intermediate look variants remain
under `outputs/robo-factory/engineering/` and `reports/v6-look-probe/`.

All36 static source/FK placement checks passed. The final V6 short replay checked
121 stored states including articulated relay parts, with maximum geometry
position discrepancy below 1e-6 m. Both videos decoded, with 61 selected images
and zero dropped frames. Evidence: `outputs/robo-factory/v6-final-replay/`.
This passive two-second probe proves the recording/placement chain, not a
complete grasp, relay or puzzle solution.

The source-bound recording remains 60 Hz float32 joint/position data plus
appearance events in streamed chunks, using the immutable content cache. It
includes passive bodies/joints, not just commands, but is not a 2 ms solver
checkpoint. Factory playback applies FK and one rigid workstation transform;
it never reruns commands, IK or task physics. Images are GPU rendered on Apple
M5 Pro; ordinary task dynamics and IK run on CPU. Future USD/Isaac conversion
remains unverified; portable meshes, materials, identifiers and frame conventions
are retained rather than claiming an untested target works.

## Parallel ownership and limits

The native interface accepted three full-history child forks. The fourth request
returned `agent thread limit reached`; the parent therefore owns relay and shared
engineering, with children owning transport, mosaic and dial respectively.
Parent settings were verified as gpt-6.1-sol/high, full filesystem access and no
approval prompts. Forks inherit those settings. No blank replacement chats or
nested agent pool were used. Children finish their bounded assignments and end;
they are not persistent robot personas.

Each family has its own physical model, process, output directory and finite
wall/simulation/action budgets. Independent physical worlds can run concurrently.
A loopback read-only viewer shows actual simple-background frames and progress:
`http://127.0.0.1:54533/`. The parent monitors native agents, heartbeats, progress
and failures while handling relay; process-alive alone is not completion.

## Oracle status and evidence

The batch root is `runs/oracle/suite-v2-r10-20261025/`. `summary.json` lists every
selected case, full successes, latest reports and partial commissioning. Refresh
with `scripts/summarize_oracle_batch.py`. Per-case attempts never overwrite old
attempts. Partial goal samples, budget exhaustion, interrupted unknown actions,
reference search failures and full physical success are distinct outcomes.

References use private geometry for engineering planning but execute normal
actuator commands in the live world. Candidate object poses occur only in scratch
data, with real contacts/support and acceptance checked during execution. No
live object teleportation, hand welds, scoring relaxation or changed friction
is allowed to manufacture success. No GPT policy experiment has been launched.
The environment remains preview-only for policy readiness until the relevant
complete physical acceptance is actually established.

Current nontrivial reference work includes whole-arm receiving posture/approach
and future-carry clearance for relay; real tabletop relay for puzzle supplies;
contact-aware regrasp for side-facing transport plugs. A finite failed search
is not proof the environment is impossible. Stop on a repeated identified defect,
diagnose and validate the new strategy in isolation before expanding.

## Files and retention

- `configs/environments/`: immutable source revisions and asset manifests.
- `configs/suites/`: selected 36-case catalog snapshots.
- `robofrontier/suite/`: shared execution plus separate family-owned references.
- `runs/oracle/<batch>/<family>/<case>/seed-0/attempt-NNN/`: physical states,
  source/executor hashes, action trace, process checks, stills and final report.
- `runs/state-assets/`: checksum-bound immutable shared assets; portable export
  must include referenced content.
- `outputs/robo-factory/`: generated factory models, visual checks and videos.
- `docs/generated/`: small indexes and selected previews, not another full copy
  of every run or frame.

No historical mass renaming, deletion or new database/memory framework was
introduced. The existing record reader filenames were preserved. Factory renders
are derived products; they do not replace source motion records.

## Later commissioning updates

After Dial completed all nine configurations, its fork ended. A newly forked
Relay agent took over the parent's remaining relay engineering; this uses a
released slot, not four simultaneous children. The parent now supervises and
owns only shared engineering, rendering and aggregation. Distinct long cases
and small validation fixtures may use up to two isolated worlds in a family,
with separate runner summary filenames. Each real world still has one writer.

Shared preview timing now has an independent clock, so frequent short actions
cannot starve the image. Scratch data is prohibited from publishing live images
or status; regression checks cover this identity boundary. The viewer labels
image age/source, prefers final/failure images after closure, and lists parallel
active worlds. Recorded physics streams remain the primary motion evidence.

Mosaic acceptance had a measured quadratic Python hot loop. Its predicates are
now evaluated in arrays, with floor contact checked only for geometrically
eligible objects. Requirement equality, inclusive boundaries, colour/shape,
seating and contact thresholds did not change. Differential checks cover all
nine scenes, disturbed poses, contact/floating-point boundaries and25 actual
saved states from the63/72 interrupted trial. A new physical attempt also
matches the earlier trajectory prefix exactly. Function timings and evidence:
`runs/engineering/evaluator-v2-vectorization-20261025/report.json`. Function-only
speedup must not be presented as full-task speedup.

New environment logic fingerprints include both `evaluate.py` and
`evaluate_v2.py`, with an explicit provenance schema. Older report fields are
not rewritten; their complete frozen executor bundles retain the older code.
The physical controller and source models have not changed in this optimization.

The simple-video renderer now streams the same chunked motion format, restores
appearance events and uses FK only. Example complete front/rear8x replay:
`outputs/oracle/suite-v2-r10-20261025/relay-a08-d1/`. Display downsampling is
explicit in its manifest; the full60 Hz source is retained.

The wide factory view still shows a shadow artefact on small robot details.
Exposure, disabled-shadow,8192-map and interior-key-light probes are retained
under `outputs/robo-factory/reports/v6-shadow-probe/`; simply enlarging the map
did not resolve it. Do not call the current visual a photorealism acceptance.
Physical/source placement and replay checks are separate and remain passed.

## Opt-in hand position driver commissioning

The loaded-opening fixture exposed insufficient position-servo drive, not an
80 mm mechanical aperture restriction. The original K=100 N/m drive remained
about59 mm even after waiting to its original deadline. Same-state fixtures
with K=1000 and4000 N/m and critical robot-only damping both opened reliably;
1000 was selected with the lower measured peak force. No mass, passive damping,
friction, geometry or timestep changed. Engineering fixture evidence is separate
from complete-task success.

The shared driver now has opt-in `pd1000-v1` position tuning with a70 N cap.
Legacy inputs retain their old driver. Force-grasp hold retains its original
velocity damping, and switching back to aperture move restores the selected
position profile. Physical-control fingerprints include the profile field.
Regression checks cover52/80 mm tracking, real pickup/release, mode restoration,
legacy behaviour, bounded stopping and read-only checks. This is a benchmark
motor implementation, not a claim about Franka firmware calibration.

The original batch is bound to its frozen legacy harness. New verification uses
`runs/oracle/suite-v2-r10-servo2-20261025/` and
`configs/harness-frontier-v2-servo2.json`; old successes are not relabelled as
new-driver successes. The latter config also declares Adaptive/gpt-6.1-sol/high
for later policy experiments, which have not started. The parent now owns Dial
reverification after its completed disposable fork, while the other three forks
continue their families. Both batches can be inspected in the same read-only
viewer, with batch labels.

## Offline high-quality factory presentation

An additional Cycles/Metal path now imports the same compiled visual geometry,
shared meshes, authored textures and stable geom identifiers without physics.
The neutral exposure variant is in `outputs/robo-factory/v6-cycles-final/`;
three1280x800 images used48 samples, about20–23 seconds each after scene setup.
This resolves the observed Filament small-detail shadow acne in the inspected
views. Transform conversion is explicitly checked: position error below1e-6m
and rotation-matrix component error below5e-7. Geometry is not artistically
moved to make the picture work. Lighting/exposure are render-only choices.

The compiled asset bundle is `outputs/robo-factory/v6-blender-assets/`.
`export_factory_blender.py` exports it; `render_factory_blender.py` is run inside
Blender. Shared meshes are instanced rather than copied per robot part. The
saved `.blend` references the asset textures, so retain both folders when moving
it. A visual `.usdc` with copied textures and original geom ID/name custom
properties is also exported. A USD visual export is not proof of Isaac dynamics
compatibility; task control and all acceptance still run only in source MuJoCo.

The static USD audit checks all31,517 original geom IDs/names, metre scale,
Z-up axis and rigid poses. Blender's default Euler USD export produced a
measurable near-gimbal rotation error, so the adapter explicitly exports matrix
transforms. The matrix export passed with position error<1e-6m and rotation
component error<5e-7. The failed Euler diagnostic is retained under
`outputs/robo-factory/reports/v6-usd-trs-defect/`. Authored text UVs are converted
from the MuJoCo image-origin convention to Blender's, avoiding inverted signs.
This is still static visual interchange acceptance, not Isaac dynamics testing.

Successful motion streams are independently audited with
`scripts/check_oracle_motion.py`. The legacy16 completed cases contain1,053,941
recorded states and347.4MiB of compressed state chunks, excluding models,
metadata, events and failed attempts. This is measured data, not a storage
estimate based on full-rate float64 solver state. Keep that distinction explicit.

A secondary EEVEE presentation probe did not show a useful render-speed gain on
this scene:1280x800 close-up frames remained about20–21 seconds after startup.
It is retained as a probe only, not adopted as a claimed fast renderer. Source
MuJoCo preview and video paths remain available independently of Cycles output.

## Frozen execution startup

New Oracle CLI launches now copy Python packages plus script helpers and the
selected harness/catalog, then exec from that snapshot using the same PID.
This closes the interval in which source could be saved after provenance
capture but before lazy family import. Run-local Mosaic commissioning uses the
same entrypoint mechanism. Data-directory aliases still point to the canonical
versioned assets, which retain independent source checksums. This is reproducible
execution, not an OS sandbox or a new task scheduler.

A complete four-dial physical run using the frozen launcher passed. All14,157
recorded timestamps/qpos frames are bitwise identical to the original same-input
run; physical-control SHA remainsac85b406... . Evidence:
`runs/engineering/oracle-frozen-launch-20261025/verification.json`.
Snapshot metadata is stored separately from the code/config executor digest so
launch timestamps do not masquerade as code changes. Existing loaded worlds
were not hot modified; older reports are retained without retroactive rewriting.

# Factory realism and Oracle launch plan — 2026-10-25

Historical planning snapshot, superseded by `docs/robo-factory.md` and
`docs/gpt-policy.md`. Its Filament path has been retired: current simple-task
observations use MuJoCo OpenGL Classic; factory output uses Cycles/Eevee.

Status: planning/audit only. No scene changes, Oracle execution, agent spawn,
asset deletion or historical path migration in this phase. Evidence:
`outputs/robo-factory/reports/planning-audit-20261025.json`.

## Established boundaries

Oracle and later GPT execute the same frozen simple-background physical models.
Factory output restores recorded geometry motion and appearance events offline,
using source-model FK and one fixed workstation placement. It never reruns
commands, IK, contacts or task physics. Separately recorded tasks shown together
are a presentation, not evidence of a simultaneous36-task physical experiment.
Future GPT topology remains Adaptive. Oracle state/solutions remain private.

Current sources are threeR4, eighteenR5, nineR6, threeR8 and threeR9 cases.
The existing55x50 m V5 factory composition still uses the older R7 relay inputs.
It must be rebuilt from the selected current snapshot before new motion replay.
Current source tables are0.4 m high; source integration steps remain2 ms.

## Realism work, in order

1. Compare fixed interior and bench-close cameras against the retained approved
   industrial reference. Make one fully dressed workstation sample before
   replicating to36 cases. Reference table counts and silhouettes are style,
   not instructions to replace the actual benchmark hardware/layouts.
2. Diagnose pale/flat appearance with separate lighting/exposure/material tests.
   Inspect environment fill, sun/ceiling balance, colour space, white plastic
   and painted-metal albedo, normal/roughness maps, imported material bindings,
   contact shadows and ambient occlusion. Existing exposure4.3 alone does not
   establish the cause. Avoid baking static robot shadows into the floor.
3. Use physical material scale and visible edge detail: real board edges and
   modest bevels, differentiated painted/galvanised metal and rubber, concrete
   roughness variation, restrained floor repairs and wear, readable labels,
   equipment connectors/supports and believable window light. Do not turn the
   floor into a mirror or create uniform dirt/noise everywhere.
4. Candidate bench height0.80 m, to be checked on the sample. Keep current XY
   table shapes, object sizes, robot geometry and relative task layouts. A height
   change needs one shared tabletop/frame parameter: update bases, fixtures,
   supplies, goals, home/parking poses, motion bounds, cameras and evaluators.
   Current absolute0.4/0.76/1.08 m assumptions occur in builders, references and
   evaluation; changing just TABLE or stretching legs is insufficient. Preserve
   old models/records and create a new source revision. Ground height is physical:
   altered drop distance is not silently declared the same old experiment.
5. Factory-only dressing: modular drawer/control units, shelves and plastic bins
   below solid tabletop areas; local cable/air connections and believable frame
   supports; carts, pallets, packing/crate stations, waste bins and maintenance
   equipment in appropriate floor service areas. Keep exits, aisles, ring cutouts,
   maintenance access and robot/load motion clear. Place objects purposefully,
   not uniformly beneath every square metre. Dressing is visual-only, but must
   not visibly intersect robots, carried objects or the floor; inspect recorded
   swept bounds afterwards rather than altering motions to fit decoration.
6. Keep completeness from all sides: full roof/walls, configurable inspection
   visibility and cameras, close and wide verification. Quality comes first;
   share meshes/materials and batch static details without reducing close-view
   fidelity. The previous lightweight triangle budget is not a final-quality gate.

## Two rendering targets

- MuJoCo Filament remains the currently verified GPU renderer. Actual current
  V5 full-scene timing at1280x720: interior diagonal118.90 ms/frame, storage
  close10.96 ms/frame,8 warm measured frames each. This includes readback/copy,
  excludes physics/startup/encoding. The earlier empty-shell3-12 ms measurements
  do not describe the current36-workstation scene.
- The graphics device was verified as Apple M5 Pro. Factory rendering uses
  OpenGL via the Apple driver; ordinary MuJoCo task dynamics currently run on
  CPU. GPU rendering does not automatically move mj_step or IK to GPU.
- Source meshes, UVs, PBR texture roles, material identifiers and rigid local
  transforms stay reusable. Maintain portable asset paths, metres/Z-up and
  explicit quaternion conversion. Use a small material/placement adapter for
  MJCF/Filament and USD/UsdPreviewSurface; renderer lighting units and outputs
  need separate verification, not assumed identical numeric settings.
- MuJoCo provides a USD scene/trajectory exporter; this does not certify the
  current factory's complete material/animation conversion. Validate one short
  clip including a passive joint and visible display changes before bulk export.
  USD animation is kinematic playback, with task dynamics disabled in the target.
  Future Isaac/RTX rendering needs a supported machine; the current Mac test is
  not an Isaac validation. No requirement to run another physics simulator merely
  to produce factory images.

## Gates before Oracle delegation

1. Settle the shared tabletop/frame and physical source revision; rebuild the
   factory placements from it; verify short simple/factory pose reconstruction.
   Preserve the generic60 Hz float32 position/joint recording plus timestamped
   appearance events, streamed chunks and source hashes. This captures all
   visible moving parts for replay, not every2 ms solver state or a restart
   checkpoint. Do not reconstruct object dynamics from commands.
2. Resolve or explicitly reject unmet mechanical assumptions. Current disturbed
   closed-ring fixtures show about1.2 mm overlap at2 ms; full dynamic contact
   quality and manipulator handover are unverified. A capability gate rejects
   unsupported references before creating a long physical run: the existing
   rigid-beam relay reference explicitly refuses elastic-hinge L3. That is a
   missing reference, not proof that the task itself is impossible.
3. Adapt references to the chosen geometry/height and validate representative
   grasp/release, insertion/compliant board, complete module support/rotation,
   and articulated relay support/receiving operations. Private Oracle truth is
   permitted for engineering planning; live state teleportation, hand welds,
   altered scoring or changed physics to force success are not permitted.
4. Freeze benchmark scene/physics/controller/evaluator and executable source for
   a batch. Workers read shared inputs and code; do not change them mid-batch.
   Transport and mosaic currently share placement_oracle.py, so no concurrent
   edits there. Workers submit bounded diagnosis/candidate patches in their own
   output area; integrate shared fixes only for a new validated executor version.
5. Start with each family's smallest representative, expand after it succeeds.
   Physical failure, unsupported reference and exhausted search/time budget are
   distinct outcomes. An unsuccessful finite candidate search is not a global
   unsolvability proof. Stop a family on repeated common failure; do not run nine
   expensive identical failures or relax the task to produce a pass.

## Fork boundaries and supervision

Parent model/effort were verified locally as gpt-6.1-sol/high with full filesystem
access and no approval prompts. A full-history fork uses fork_turns=all and
inherits those settings; the current spawn interface does not permit a separate
model/effort override on such a fork. Verify effective settings at launch.

The present tool advertises4 total active slots including the parent. Four
concurrent children therefore require at least5 total slots; do not claim4 are
running under the current cap. If the cap cannot change, keep four logical
assignments and execute3+1. Do not substitute new blank app sessions for forks.

| Child | Owns | Output / completion |
| --- | --- | --- |
| Oracle-transport | Frozen nine transport cases, one active world at a time | Own family reports, recordings and bounded failure diagnosis |
| Oracle-mosaic | Frozen nine mosaic cases, one active world at a time | Same; no transport/shared helper edits |
| Oracle-relay | Frozen nine relay cases after articulated-reference readiness | Same; stop/report unsupported mechanics before long execution |
| Oracle-dial | Frozen nine dial cases, aggregate-only code inference | Same; no private digit leakage or other family changes |

Each child has its own process/model/data, run ID, output paths and any viewing
port; no shared physical world, global bridge file or aggregate-status writes.
No nested agent pool. Main session remains responsible for waiting, native-agent
state, process liveness, simulation progress, heartbeats, deadlines, disk space
and reporting; it does not issue robot actions during child execution. Four live
views may share an inspection dashboard, not a single mutable simulation.

Add per-case wall/simulation/action/refinement budgets to existing reference
execution and use a small outer process supervisor for blocked native calls.
Record whether a period is planning, physics or recording before calling it
stalled. Graceful stop finalises chunks/reports; hard exit retains completed
chunks and marks unresolved outcomes unknown. No automatic action replay or
same-trial reset. The old605 simulated-second action loop guard is not a batch
wall-time safeguard. Parent progress monitoring cannot be replaced by an alive
background-process label. A finished assignment returns report paths and ends;
it is not a persistent robot role and is not automatically woken for new work.

## File organisation: minimal changes, no mass move now

Keep code in robofrontier/, launchers in scripts/, immutable environment bundles
and the selected suite in configs/, reusable factory assets in assets/robo_factory/.
The current footprint is approximately runs33 GiB (historical oracle15 GiB),
configs7.1 GiB, docs1.5 GiB and outputs0.7 GiB. Historical Oracle root has264
loose files; this is not a clean batch layout. The new36-case v2 Oracle root does
not exist.201 workstation manifests reference6.44 GiB of assets but only131
unique declared content hashes (~33 MiB); this does not mean all configs can
shrink to33 MiB or that old portable bundles can simply be deleted.

For new output use:

```
runs/oracle/<batch-id>/
  batch.json                         # frozen snapshot, workers, budgets, status
  <family>/<case-id>/seed-0/attempt-001/
    status.json                      # incremental, atomic; owned by one worker
    report.json                      # final result and evidence references
    events.jsonl                     # existing ordered action audit
    motion/                          # existing chunked source-bound recording
    snapshots/                       # initial / milestones / failure / final
    stdout.log
    stderr.log
```

Reuse existing runs/state-assets content cache/hardlinks for new recordings;
cache entries must be immutable and checksum-verified. Frozen code/assets can be
shared by checksum, with portable export bundles assembled when needed. Keep
current Trial-required filenames internally until the launcher/reader is updated
together; this tree is the target, not a claim that it is already implemented.
Source records are not display products: factory videos/images belong under
outputs/robo-factory/<render-version>/<batch-id>/, while docs/generated holds
indexes and selected previews, not another copy of every frame/video.

Inventory and index history first. Do not mass rename/delete archived inputs,
recordings or images whose paths/hashes are referenced by manifests. Introduce
batch identity and ownership for new work first; deduplicate old bundles only
with manifest validation and replay checks. Keep one concise current-state index
and revision-labelled historical evidence; no extra memory/agent framework or
database is needed for this organisation.

## Primary references checked

- MuJoCo rendering: https://mujoco.readthedocs.io/en/latest/programming/rendering.html
- USD trajectory export: https://mujoco.readthedocs.io/en/stable/python.html#usd-exporter
- USD materials: https://docs.omniverse.nvidia.com/materials-and-rendering/latest/templates/UsdPreviewSurface.html
- Isaac platform requirements: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html
- Codex subagents: https://learn.chatgpt.com/docs/agent-configuration/subagents

Next implementation order: sample/height/frame contract, factory dressing and
look validation, selected-source composition/replay verification, Oracle
capability/budget/output gates, then launch the approved bounded fork batch.

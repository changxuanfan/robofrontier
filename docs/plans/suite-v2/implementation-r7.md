# Pre-Oracle preparation — 2026-10-25

This revision prepares the four families without running a full-task Oracle,
GPT, or Codex subagent. Passive fixtures, 0.2–2 second recording/capacity probes
and static FK tests are engineering checks, not task completion evidence.

## Current artifacts

- Candidate catalogue: configs/suites/frontier-v2-r7.json. Three R4, eighteen R5,
  nine R6 and six R7 cases; all36 physical inputs frozen against in-place rebuild.
- Factory config: configs/backgrounds/robo_factory.json and versioned
  robo_factory_55.json. Previous48×42 config retained as robo_factory_v3.json.
- Empty55×55 shell: outputs/robo-factory/v4-shell/scene.xml.
-36 composed workstations: outputs/robo-factory/v4-workstations/scene.xml.
  No physical task DOFs or actuators; all task contacts disabled for display.
  Old factory scene and old task model revisions are retained.
- Default future policy harness: configs/harness-frontier-v2.json, Adaptive.
  The global legacy harness remains unchanged. Policy trials remain gated as
  environment_preview_only until full-task commissioning.

## Placement

Four23×23 m regions,3 m central cross aisles,3 m between the building edge and
region boundary. Coordinates: transport[-24.5,1.5,-1.5,24.5],
mosaic[1.5,1.5,24.5,24.5], relay[-24.5,-24.5,-1.5,-1.5],
dial[1.5,-24.5,24.5,-1.5]. Each family has nine cells with variable size.
Transport groups scale by row and difficulty by column; other regions group
difficulty by row and scale by column. Workstations are rigidly translated,
not rescaled. Layout uses initial physical bounds plus an indicative1 m base
radius and1.2 m inter-cell gaps. Long loaded-motion swept bounds remain to be
checked after actual executions. Doors, columns and perimeter equipment remain
separate from task mechanics. Floor labels identify regions and individual cases.

The composer namespaces bodies and appearance geometries, shares identical
assets, preserves source mesh transforms, and omits collision-only robot skins
from rendering. All36 source-to-factory initial visible geometry positions and
orientations were checked. Maximum position discrepancy is about5.1e-15 m.
This is a coordinate-transform test, not physical accuracy evidence.

## Cable candidate

Official mujoco.elasticity.cable,200 mm length,4 mm diameter,16 segments,
10 g/m linear mass (2 g total), effective bend1e6 Pa and twist3e5 Pa,
straight stress-free shape, no added armature. Damping scales from a5 ms
relaxation-time assumption and the segment bending stiffness. End attachment
solref2 ms and stronger constraint impedance are fixed numerical settings.
Nonadjacent segments/different cords collide. Adjacent links and the intentional
shared top junction are excluded; this is not a detailed knot model.

Material properties are engineering assumptions, not measured braided-cord
values. They must not be described as physically calibrated.

The original2 ms candidate failed numerically. A stricter end connection also
made16 segments unstable at1 ms. The retained16-segment candidate uses0.5 ms;
0.25 ms and8-segment comparisons gave a similar~0.957 s pendulum period versus
a~0.952 s point-pendulum reference. Max end error in the retained small fixture
was~0.035 mm. Earlier failed reports are preserved. Numeric warning/time-reset
checks now abort a run rather than silently continuing an automatically reset
world. Six complete relay initial scenes also passed0.25 s passive settling.

Independent spring test: actual board with three initialized plug/cube loads
settled8.866 mm, agreeing with the linear load estimate. Isolated vertically
supported crank returned from90/180/-90 degrees to within4 degrees after8 s.
Neither test establishes actual robot grasping, all task trajectories or full
puzzle success.

## Shared recording and replay

robofrontier.motion_recording is used by v2 Oracle Trial and policy World.
60 Hz sampling, float32 positional state, bitwise XOR delta + ZIP compression,
2-second bounded chunks. Exact unchanged values compress; no motion threshold
is applied. Quantization tofloat32 is explicitly lossy; subsequent storage and
reconstruction preserve those float32 bits exactly. All moving objects, rope
joints, passive board joints and robot joints are in qpos. Device geometry-RGBA
changes are logged on the same physics clock. Assets/source layout and version
hashes are frozen; a shared content-addressed cache avoids duplicate asset bytes.

Completed chunks and their index are atomic. Reader validates source and chunk
checksums. Recovery can reindex finalized orphan chunks, excluding temporary
incomplete tails. This is a visual recording, not a full physics restart state.
The implementation uses synchronous bounded chunk writes; their wall time is
reported rather than hidden or silently dropping frames.

The private two-second passive relay probe recorded121 states:241032 raw bytes,
160160 compressed chunk bytes (0.664 ratio). Compression is scene-dependent;
this is not an estimate for every task. Both simple and factory videos contain
61 output frames at30 fps including the final endpoint, with zero encoder drops.
All121 source states were reconstructed in the factory with~4e-15 m transform
error after the declared float32 conversion. A separate synthetic test moves a
robot and floating module and changes the display, verifying those channels
without any physics steps. The factory never re-executes commands or contacts.

Recording/source truth remains private engineering data, not agent observations
or skill content. This is tool/context separation, not OS-level file isolation.
Isaac/USD conversion and actual replay there remain unverified.

## V2 execution and acceptance adapters

scripts/verify_frontier_v2.py defaults to no-physics preflight. All36 cases load
with the v2 evaluator; no initial task success is reported. Full execution needs
explicit --execute plus a family/case selection. Do not run it during preparation.

Placement acceptance reads actual cube, pocket, blade and socket dimensions.
Pocket completion checks full footprint/seating, not old40/50 mm assumptions.
Relay evaluation tracks each assembly independently, real beam support,
receiving overlap, custody, drops/recovery, released stable final landing and
tray spills, without fixed pairs. Reference transport/mosaic apertures derive
from current object size. V2 relay has a separate finite-candidate multi-payload
reference; it is not a successful witness until it is actually executed.
Landing packing, contact-rich motion, long-run robustness and reference failure
recovery remain subjects for future Oracle work. No success has been inferred
from static checks or reference-code existence.

## Four-worker capacity and cost

Four independent passive physics/recording processes, one largest scene per
family, completed0.2 simulated seconds each without task commands or agents.
Peak process RSS was~331–382 MiB each. Sample realtime factors: transport4.62,
mosaic1.08, dial8.43, relay0.0945. The Cable case is substantially slower.
This is a short, load-specific initial-contact sample, not a long-run guarantee.

A scratch8-segment/1 ms candidate kept the same material/total mass and completed
0.5 simulated seconds in~1.53 wall seconds (~0.328 realtime). Its observed throughput is higher in a different short standalone workload;
this is not a controlled speedup comparison. It has NOT replaced the retained16-segment scenes; contact/shape fidelity needs
more evidence before adopting it. Keep this cost visible before full Oracles.

## Future Oracle operation, not started here

Create four external Codex subagents only when requested to start Oracle:
Transport / Mosaic / Relay / Dial, nine tasks each. Each owns its independent
MuJoCo processes, run directories, captures and reports; no shared MjData or
nested task pool. Use one family per worker and the explicit v2 execution entry.
Keep foreground physics/recording separate from later heavy factory rendering.
Do not let individual workers silently change shared parameters/acceptance.
Preserve all failures and revisions. Research handles engineering health and
summaries, not the workers' robot planning. Subsequent GPT defaults to Adaptive
with the same frozen physical inputs, controller contract and recorder format.

## Regression evidence

123 affected regression cases passed together; the final interrupted-event
corruption regression also passed with the motion-recording group (124 unique
checks). Input candidates remain preview-only for policy execution. No full
Oracle, GPT or Codex subagent was started.

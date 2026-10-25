# Current: R9 mixed-axis relay L3

Next-phase planning only:
[factory realism and Oracle launch](factory-oracle-next.md). No launch yet.

See [relay-r9.md](relay-r9.md). Three new L3 scenes mix in-plane and
out-of-plane spring hinges; all R8 inputs retained. No Oracle/GPT run.

# R8 cable-free relay environments

See [relay-r8.md](relay-r8.md). Six new simple-background relay scenes, retained
R7 Cable inputs, no full Oracle/GPT or new factory composition.

# Archived: R7 pre-Oracle preparation

See [pre-oracle-preparation.md](pre-oracle-preparation.md) for current artifacts, evidence, performance and limitations. No full Oracle/GPT run.

# Frontier v2 R6 — dial readability update

Nine dial cases use R6; twenty-four cases retain R5, three retain R4. R5 is
preserved. No Oracle or GPT run.

- Double dial digit height20→40 mm; use122 mm numeral radius and8 mm outer
  margin. Enlarge actual white disk243.8→300 mm to avoid grip/label overlap.
  Density unchanged; disk mass510.397 g, inertia recomputed.
- Remove L3 orange stripe/arrow, retaining crank direction as indicator.
  L1/L2 gear knobs retain a triangular orange arrow.
- Recompute floating tabletop rest and save camera, asset and pose manifests.
- 36 static checks and73 regression tests passed; no dynamic validation.

Previous R5 implementation follows; its dimensions describe the preserved R5.

# Frontier v2 R5 — environment revision

Specification: docs/frontier-v2-environments.md.
Current catalogue: configs/suites/frontier-v2.json.
33 changed R5 cases plus three unchanged R4 transport L1 cases; old revisions
are preserved. No Oracle, GPT run, factory placement or Isaac conversion.

Implemented:
- 2/3/4 independent relay assemblies for8/12/16 arms, spread along the route.
  Unique IDs, full copied cable/free-body assemblies and close cameras.
  L3 uses a135° mitered bent rod instead of the arc, maintaining1.2 m chord.
- Correct three asymmetric9×18 petal cells. Apply user-selected Bright Tulip.
  Four/eight-arm mosaic table Y length contracts10% fromR4, X unchanged.
  Screen empty-wrist rotation and approach samples for all supplies/goals.
- Transport source identity mapping stays fixed A–L. L2/L3 use2.5×12×24 mm
  blades,4.5×15 mm socket openings,26 mm depth and32 mm board thickness.
  Recompute masses, inertias, initial support, spring presentation and cameras.
- All dial disks grow6% to243.8 mm; numerals20 mm tall and at the edge.
  Single round L3 crank length83 mm, shaft/grip thickness+25%; remove opposite
  balance arm. Recompute disk/rotor masses and initial tabletop rest.
  Spring0.12 Nm/rad is a candidate; dynamic return is not yet verified.
- Refresh static MuJoCo gallery, per-rod and plug/socket close views.

Validation scope:
- 36 static layout/access checks; no simulation time advanced.
- 73 targeted regressions: suite geometry/public projection, controller/gripper,
  robot-only motion checks, static access and portable saved-pose reconstruction.
- These do not prove continuous collision-free motion, grip dynamics, reliable
  crank return, full relay delivery, final packing or task success.
- Rope-to-rope collision remains disabled in the inherited cable model.
- Runtime policy/harness and factory sources unchanged.

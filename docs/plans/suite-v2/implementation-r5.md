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

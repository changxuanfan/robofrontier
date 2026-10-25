# R8 cable-free relay environments — 2026-10-25

Six revised simple-background scenes; no task Oracle, GPT run or factory
recomposition. L1 and the other thirty suite scenes remain unchanged. Frozen
R7 Cable inputs and their measurements are retained, not overwritten.

## Apparatus

- L2 keeps the 90-degree red curved beam and replaces the cord with two
  perpendicular, closed steel eyes. The suspended body is free, supported by
  real ring-to-ring collision forces. No equality, weld, robot attachment or
  active stabiliser connects the weight to the beam.
- L3 keeps the 135-degree elbow outline, but splits the timber at a passive
  flexion hinge. Spring stiffness18 Nm/rad, damping0.55 Nms/rad, zero rest angle,
  nominal limits +/-25 degrees. This is a manufactured spring-loaded joint,
  not a claim to model distributed elastic timber. A cosmetic coil shows its
  location; the physical joint supplies the restoring torque.
- L3 uses the same closed-eye contact suspension, followed by an approximately
  120 mm rigid metal shaft and a weight. It swings as one free rigid assembly.
  No tray, rope, Cable plugin, direct weight grasp or automatic levelling.
- Each metal weight itself is600 g (previous250 g), with a base, shoulders and
  visible600 g marking. Total suspended mass includes its eye and stem/shaft:
  approximately627.3 g forL2 and652.3 g forL3. Ring stock diameter5 mm; beam-side
  ring centreline radius22 mm, weight-eye radius18 mm;20 capsule segments/ring.
- Rings use declared4 ms contact time constant and explicit impedance. Both
  revised levels use2 ms integration. Parameters are engineering assumptions,
  not real-material calibration or proof against high-speed tunnelling.

8/12/16 arms retain2/3/4 separate numbered rods distributed along the route.
Upper-half-ring layouts, destinations and flexible team choice remain intact.
Every component starts naturally table-supported; no extra parking pedestals.
Task hints describe the apparatus and future support/clearance considerations,
not a sequence of grasps, assigned teams or a solved handover plan.

## Checks and limits

`docs/generated/frontier-v2/relay-r8/static-checks.json` checks all six initial
scenes: robot mounting footprints, pickup/destination patches, full assembly
footprints (including the articulated half and independent weight), inter-rod
contacts, task/schema validity and finite state. All six pass. No initial
payload penetration greater than0.2 mm was found by the separate contact scan.

`checks.json` contains elevated, passive suspension fixtures at2 and1 ms, each
five simulated seconds. The beam root is clamped to an engineering test stand;
there is no robot and this is not a grasp/manipulation witness. The fixture
preserves the live articulated-bearing parent/child collision exclusion after
making its root static. Both weights remain linked, without numerical warnings.
Maximum ring penetration is approximately0.45 mm in these fixtures. Soft joint
limits allow slight transient overshoot. None of this certifies aggressive
robot motion, continuous multi-arm paths, full-task solvability or exact real
material behaviour.

Matched-duration whole-scene timing: each of six scenes advances0.1 simulated
seconds, median of3 repetitions, standard passive robot pose holding. Loading,
recording, rendering and model inference are excluded. R7 sum6.257 s; R8 sum
1.312 s; approximately4.77x faster in this workload. Apparatus, contacts and
timesteps all differ, so this is a design comparison, NOT an isolated Cable
overhead measurement or a complete-task speedup prediction. The report also
retains detected same-assembly initial overlaps in the old Cable baseline;
those frozen inputs are historical, not corrected by this revision. Curved L2
still has many tabletop contact facets and is substantially slower than L3 in
the passive workload; removing Cable does not eliminate all solver cost.

The private evaluator now recognises both timber halves as possible supporting
grasps, excluding rings, shafts and weights. Existing custody/drop/receiving
overlap requirements remain. Removed trays no longer trigger tray acceptance
or camera assumptions. The old rigid-body Oracle explicitly refuses articulated
L3: its multi-body reference requires adaptation before later commissioning.
All scenes remain`environment_preview_only` and`oracle_verified=false`.

## Records and previews

Stable names, complete physical assets, body/joint maps and source hashes remain
in each workstation manifest. The hinge coordinate and independent weight pose
are included by the same generic60 Hz recording/FK contract. The current V5
factory composition still contains R7 relay assets; it must be recomposed from
R8 before replaying new relay recordings. No factory or Isaac compatibility
claim is inferred from static preview success.

- Current immutable mixed-version snapshot: `configs/suites/frontier-v2-r8.json`.
- Previous Cable snapshot: `configs/suites/frontier-v2-r7.json`.
- Gallery: `docs/generated/frontier-v2/relay-r8/index.html`.
- Top views: `relay-r8/top-all.png`; close views:`weights-all.png` and
  `l2-l3-detail.png`; each scene also has front, rear, rod, weight and L3 hinge
  close views. All images are actual tabletop initial-state MuJoCo renders.
- Generator: `robofrontier/suite/rigid_relay.py`, called by`v2.relay` forR8 only.
- Engineering fixtures:`scripts/check_rigid_relay.py` (never task Oracle).
- Preview renderer:`scripts/preview_rigid_relay.py` (no physics steps).

# Frontier v2 R4 — table scaling and upper-arc layouts

Current specification: docs/frontier-v2-environments.md.
R1/R2/R3 catalogue, model folders, builder snapshots and galleries preserved.
Current scene_revision=4. No full Oracle, GPT run or factory/Isaac import.

Implemented:
- Transport external table sides and mount/inner-hole X/Y positions contract
  .85 relative to R3. New external sides1.938/2.720/3.536 m for4/8/12 arms.
  Robot hardware,60 mm stock and boards are not scaled; four-arm table is solid.
- Twelve-arm relay: six pairs on an upper semicircle, route radius1.73 m,
  inner/outer bases1.08/2.38 m; route X increases from left to right. Shared
  upper-arc table/endcaps reused with sixteen-arm relay and floating six disks.
- Twelve-arm floating gear/crank levels: six upper-arc modules, same radius,
  real tabletop initial rest and unobstructed outer-top score panel.
- Mosaic same-side X positions/pitch contract .95; table X/Y sizes expand1.25.
  Artwork,60 mm cubes,80 mm pockets,81 mm pitch, depth and side clearances
  stay fixed. Every supplied cube stays in a reachable region.
- Static stock candidate screening rejects self/table penetration during a
  three-pose downward approach. Final models also screen all neighbor-stock
  and robot contacts, using a70 mm preshape (runtime choice remains free).
  Three otherwise IK-reachable but self-colliding supplies were excluded.

Evidence:
-36/36 static layout checks passed, including actual table support for mount
 corners, relay start/end region edges and initial beam geometry.
-1782 sources and1782 goals have IK candidates across transport/mosaic levels.
 Mosaic1566 sources and1566 targets additionally pass the sampled contact screen.
-70 regression tests passed, including .85/.95/1.25 geometric ratios, arc
 orientation/order, self-collision rejection and scratch-only checks, unchanged
 legacy controller behavior and portable saved-pose reconstruction.
-No simulation time advanced in engineering layout checks. No continuous path,
 dynamic grasp, full assembly or full relay proof is claimed.
-Four top-only family sheets refreshed,5976x4276 each; versioned copies -top-r4.

# Frontier v2 R3 — supplied Tulip and compact layout iteration

Current specification: docs/frontier-v2-environments.md.
R1/R2 catalogues, generated assets, previews and builders remain preserved.
Source builder: robofrontier/suite/v2.py. Current scene_revision=3.

Requested changes implemented:
- Smaller solid four-arm transport table; smaller eight-arm table/opening;
  reduced center hole on twelve-arm table.
- Mosaic60 mm cubes,80 mm clear pockets,81 mm pitch; scales6x12,9x18,12x24.
  Exact user Tulip JSON/palette imported and whole image rotated clockwise90
  for the horizontal table. No cell/quantity optimization or recoloring.
- Opposed mosaic arm rows move out; outer footprints align with grid ends,
  six/eight-arm within-row spacing tightens. Stock moves away; tables widen.
  Eight-arm side clearance305 mm selected after340/320 mm candidates lost
  downward IK coverage at some center cells. Relative to R2 bases still move
 111 mm outward; no robot limits changed.
- Highest relay outer/inner radii shrink to2.43/1.03 m. Semicircle endcaps
  physically support both start/end rectangle corners and initial beam.
- Fixed dials use solid rectangular tables again, four/six disks in two opposed
  rows with460 mm disk-row spacing. Floating levels retain existing layouts.
- Output only top views: four family sheets, each nine cases.

Evidence: all36 compile and pass static collision/table-support/endpoint checks.
All1782 source instances and1782 targets across transport/mosaic have approach
and endpoint IK witnesses; these are not full-path or manipulation proofs.
All relay zone edge/corner samples and initial rod collision-box vertices have
physical table support. No simulation time advanced in these checks.
68 regression tests passed. No full Oracle/GPT/factory migration was started.
Stable assets/coordinates/IDs and read-only pose-export boundary remain intact.

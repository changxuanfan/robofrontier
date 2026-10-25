# Relay r11 — static environment revision

Nine cases: 8/12/16 arms × L1/L2/L3. Old r10 source scenes and run evidence are retained byte for byte.

- Two equal1.40m square paint regions: START/GOAL. Eight-arm centres are(-1.30,0) and(1.30,0); arc-table centres are(-1.70,-0.10) and(1.70,-0.10). Region orientation is world-aligned, and actual final acceptance uses the displayed square. This is a new task boundary, not inherited r10 qualification.
- Remove intermediate route outlines, pickup outlines and repeated region labels. Keep two straight-table arrows or five clockwise upper-arc arrows. Rod IDs and robot IDs stay available.
- L2/L3 apply a common180degree yaw about each beam root to every free root of the complete assembly, including the independent weight. Nested hinge and ring geometry follow the same rigid transform. Beam root centres, robot mounts, tables, mass/inertia, material/contact parameters, solver/timestep and controllers remain unchanged.
- Payloads remain distributed along the route. START is the route origin, not a requirement that all payloads begin there. Intermediate unpainted anchors do not prescribe pairings or handover locations.
- Workstation body/geometry registry and hashes are regenerated for offline reconstruction from this revision. Old recorded states continue to use their own frozen source model; old successful Oracle results are not credited to the new initial orientation or target boundary.

## Validation

Nine static checks compare original mechanics and complete-assembly rigid transforms. Each case checks1250 table-supported sample points over the endpoint regions, complete initial apparatus footprints over the real tabletop, full first apparatus within START, and no initial robot/apparatus or inter-apparatus penetration deeper than1mm. Static checks never callmj_step. These do not establish dynamic manipulation feasibility.

Four focused static regressions pass. Separate existing Relay acceptance regressions check independent payload completion and original overlap/drop-custody semantics. No changes to scheduling, control, handover evaluation, prompts/skills, or an Oracle/policy run are part of this revision.

## Outputs

Gallery:docs/generated/frontier-v2/relay-r11/index.html. Independent1920×1200 top/front images per case and two nine-case high-resolution sheets. Version catalog and original-file checksums are stored beside the gallery.

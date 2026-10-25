# Airborne dials · 0.60 m spacing, symmetric trimmed table

This separate scene preserves the earlier environments. Robot and module positions are unchanged from the first 0.60 m version. Only the table edges, score-panel location and overview framing changed.

Table: 2.60 m long × 2.06 m deep, top height 0.40 m, centered at X=-0.125 m to match the complete row of robot bases. Base-to-edge horizontal margins are equal. Initial robot geometry retains approximately 12 cm of table margin on each outer side.

The CORRECT panel is centered at X=-0.25 m, midway between front-row robots C and E (the second and third front robots). Its front edge is 2 cm inside the front table edge; the entire panel is on the tabletop. The board camera is centered on that panel. The overview camera frames the full robot envelope rather than zooming with the trimmed table, preventing cropped arms.

The strict lift-before-knob-contact rules, relative dial symbols, qualitative load-moment hints, observation boundary and private contact-sequence audit are identical to the preceding 0.60 m task. Grasp points, orientations, ordering, coordination and Adaptive scopes remain Agent choices. No Oracle coordinates or trajectories are policy inputs.

The full contact-dynamics Oracle is recorded separately in `runs/floating-puzzle-060-tight-oracle-20261023-r1`. The camera-only framing adjustment is documented in `camera-only-change.json`; its before/after hashes verify all non-camera XML elements were identical. A saved-state replay may use the corrected framing without advancing physics.

# PRD: Procedural Traffic Flow Intersection Curves — Houdini 21

## What's Implemented (Feb 2026 - v5)

### Road System
- 5 lines per road x 3.5m spacing = 14m road width (4 lanes, 2 per direction)
- Right-hand traffic (drive on right)
- ConvertLine SOP (not Convert)

### Turn Rules
- Inner lane (left): LEFT TURN ONLY — Bezier arc through intersection centre
- Outer lane (right): RIGHT TURN ONLY — Bezier arc at corners
- At T-junctions/corners: impossible turns replaced with straight-ahead
- Bus-compatible turning radii (entry_dist=20, ~14.75m right / ~21.75m left)

### Direction Indicators
- V-chevron arrows placed BETWEEN lane lines (in the road surface gap)
- Left arrow in inner lane, right arrow in outer lane
- Straight arrows added where lane also allows straight-through
- Amber coloured for visibility

### Animated Vehicle Flow (v5 — Feb 2026)
- Complete route polylines generated at LANE CENTRES (between markings, not on them)
- Two route types:
  - Straight-through: edge-to-edge lane centre paths (40 total: 5 roads x 4 lanes x 2 dirs)
  - Turning: edge → straight → Bezier arc → straight → edge (seamless transitions)
- Vehicles wrap from path end to start (appears as new vehicle entering grid)
- Box geometry (4m x 2m x 1.5m) oriented by tangent via Copy To Points
- Animated with @Time + primuv for smooth curve-following
- Random phase offset per route prevents lockstep movement
- Adjustable parameters on gen_vehicle_routes and gen_vehicle_points nodes

### Colours
- White: road lane lines
- Red: turn arcs (left + right)
- Amber: direction indicators
- Green: original grid reference
- Blue: animated vehicles

## Backlog
- P1: Bidirectional traffic density balancing
- P2: Add dedicated straight-through lane markings (dashed center lines)
- P3: Varied vehicle sizes (cars, buses, trucks)
- P4: Random vehicle colors per route

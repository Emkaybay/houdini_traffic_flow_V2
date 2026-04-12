# PRD: Procedural Traffic Flow Intersection Curves — Houdini 21

## What's Implemented (Feb 2026 - v4)

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

### Animated Vehicle Flow (NEW - Feb 2026)
- Box geometry (4m x 2m x 1.5m) representing cars
- Animated along ALL driveable curves (straight lanes + turn arcs)
- Uses @Time + primuv for smooth curve-following animation
- Random phase offset per primitive prevents lockstep movement
- Oriented by tangent (N + up attributes) via Copy To Points
- Adjustable parameters: speed (15 m/s), spacing (30m), Y offset (0.75m)
- Blue colored vehicles on top of existing road visualization

### Colours
- White: road lane lines
- Red: turn arcs (left + right)
- Amber: direction indicators
- Green: original grid reference
- Blue: animated vehicles

## Backlog
- P1: Bidirectional traffic (opposite lanes go opposite directions)
- P2: Add dedicated straight-through lane markings
- P3: Varied vehicle sizes (cars, buses, trucks)
- P4: Random vehicle colors

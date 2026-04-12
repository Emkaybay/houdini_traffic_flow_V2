# PRD: Procedural Traffic Flow Intersection Curves — Houdini 21

## What's Implemented (Jan 2026 - v3)

### Road System
- 5 lines per road × 3.5m spacing = 14m road width (4 lanes, 2 per direction)
- Right-hand traffic (drive on right)
- ConvertLine SOP (not Convert)

### Turn Rules
- Inner lane (left): LEFT TURN ONLY — Bezier arc through intersection centre
- Outer lane (right): RIGHT TURN ONLY — Bezier arc at corners
- At T-junctions/corners: impossible turns replaced with straight-ahead

### Direction Indicators
- V-chevron arrows placed BETWEEN lane lines (in the road surface gap)
- Left arrow in inner lane, right arrow in outer lane
- Straight arrows added where lane also allows straight-through
- Amber coloured for visibility

### Colours
- White: road lane lines
- Red: turn arcs (left + right)
- Amber: direction indicators
- Green: original grid reference

## Backlog
- P1: Fine-tune handle factors for more realistic turn radii
- P2: Add dedicated straight-through lane markings
- P3: Animated flow along arcs

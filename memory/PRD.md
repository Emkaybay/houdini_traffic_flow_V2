# PRD: Procedural Traffic Flow Intersection Curves — Houdini 21

## Problem Statement
Generate procedural traffic flow curves at every intersection of a 4x4 grid road system in Houdini 21, matching reference image: multi-lane white roads + red quarter-circle arcs at corners + green grid.

## What's Implemented (Jan 2026)

### v2 (Current) — Matches reference image
- `setup_traffic_flow.py` — One-click Python script, uses **ConvertLine SOP** (not Convert)
- `vex/classify_intersections.vfl` — Intersection type detection (4-way, 3-way, corner)
- `vex/gen_road_lanes.vfl` — **NEW**: Multi-lane parallel road lines (Detail wrangle)
- `vex/gen_intersection_arcs.vfl` — **NEW**: Quarter-circle corner arcs at all intersection types (single unified wrangle, no Blast SOPs needed)
- `vex/color_visualization.vfl` — White=roads, Red=arcs, Green=grid
- Directional chevron indicators on arcs
- All parameters exposed as spare parameters with sensible defaults

### v1 (Replaced)
- Used Convert SOP instead of ConvertLine
- Generated Bezier curves (not matching reference)
- Required Blast SOPs to separate intersection types

## User Choices
- No U-turns
- Chevron indicators: yes
- Multi-lane roads matching reference image
- ConvertLine SOP (not Convert)

## Backlog
- P1: Tune defaults to perfectly match user's specific grid dimensions
- P2: Animated flow pulse along arcs
- P3: Width variation for rendered curves

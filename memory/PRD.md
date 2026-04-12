# PRD: Procedural Traffic Flow Intersection Curves — Houdini 21

## Problem Statement
Generate procedural traffic flow curves at every intersection of a 4x4 grid road system in Houdini 21, showing straight-through and turning paths (no U-turns), with directional chevron indicators.

## Architecture
- **Deliverable**: Python setup script + standalone VEX code files
- **Target**: Houdini 21 (VEX functions compatible since H18+)
- **Grid**: 348x348, 5x5 points (4 divisions), 87-unit cell spacing, XZ plane

## What's Implemented (Jan 2026)
- `setup_traffic_flow.py` — One-click Python script that builds the full node network in Houdini
- `vex/classify_intersections.vfl` — Intersection type detection (4-way, 3-way, corner) with point groups
- `vex/gen_4way_curves.vfl` — 6 Bezier curves per 4-way intersection (2 straight + 4 turns)
- `vex/gen_3way_curves.vfl` — 3 Bezier curves per T-junction (1 straight + 2 turns)
- `vex/gen_corner_curves.vfl` — 1 Bezier curve per corner (1 turn)
- `vex/color_visualization.vfl` — Color coding (green=straight, red=turn)
- Directional chevron indicators at curve midpoints
- Spare parameters: road_width, turn_radius, arc_segments, chevron_size
- Total: 94 curves across 25 intersections
- `README.md` with full manual setup instructions

## User Choices
- No U-turns
- Chevron indicators: yes
- Animation/pulse: skipped
- No web UI — scripts only

## Backlog / Future
- P1: Animated flow pulse along curves (`@flow_t` + `@Time`)
- P1: Multi-lane offset (parallel curves for multi-lane roads)
- P2: Width variation (thicker for through-routes, thinner for turns)
- P2: Particle (car) advection along curves with `@curveu` attributes
- P3: Radial construction lines from reference image
- P3: `.hipnc` file export (requires Houdini API)

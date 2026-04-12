# PRD: Procedural Traffic Flow Intersection Curves — Houdini 21

## Original Problem Statement
Modify Houdini traffic flow project to reduce cars by 90% and make them aware of each other.

## Architecture
- **Platform**: Houdini 21 (SideFX) — Python Shell + VEX wrangles
- **Node graph**: Grid → ConvertLine → Fuse → (Lanes | Arcs | Vehicle Routes → Vehicles → Awareness → Copy Cars) → Merge → Display
- **Vehicle system**: Parametric curve animation with `@Time`, per-frame position computation

## User Persona
- Houdini artist creating procedural traffic simulations
- Needs realistic vehicle density and inter-vehicle awareness

## Core Requirements (Static)
- Right-hand traffic, 2 lanes per direction, 14m roads
- Inner lane = left turn, Outer lane = right turn
- Animated vehicles following route polylines
- Direction indicators (chevron arrows)

## What's Implemented

### v5 (Previous — Feb 2026)
- Road system, turn rules, direction indicators
- Animated vehicles with random phase offset
- No vehicle awareness, full density

### v6 (Current — Apr 2026)
- **90% car reduction**: `vehicle_density` parameter (default 0.1)
- **Same-route awareness**: Vehicles sorted by curve parameter, minimum following distance enforced (`min_follow_dist` = 15m), cascading backward from leader
- **Cross-route awareness**: New `vehicle_awareness` wrangle node using `pcfind` spatial queries — vehicles from different routes slow/stop when too close ahead at intersections
- **New parameters**: `vehicle_density`, `min_follow_dist`, `awareness_radius`, `stop_distance`, `slow_distance`
- **New file**: `vex/vehicle_awareness.vfl`

### Files Modified
- `setup_traffic_flow.py` — Updated GEN_VEHICLES_VEX, added VEHICLE_AWARENESS_VEX, new node + params
- `vex/gen_vehicle_points.vfl` — Density reduction + same-route following distance
- `vex/vehicle_awareness.vfl` — NEW: cross-route pcfind avoidance
- `README.md` — Updated documentation with v6 parameters

## Backlog
- P1: Bidirectional traffic density balancing
- P2: Add dedicated straight-through lane markings (dashed center lines)
- P2: Traffic light system at intersections (timed stop/go cycles)
- P3: Varied vehicle sizes (cars, buses, trucks)
- P4: Random vehicle colors per route
- P4: Queue formation at red lights

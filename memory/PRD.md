# PRD: Procedural Traffic Flow Intersection Curves — Houdini 21

## Original Problem Statement
Modify Houdini traffic flow project to reduce cars by 90% and make them aware of each other. Fix bouncing/glitching — make cars behave realistically, decreasing speed when necessary.

## Architecture
- **Platform**: Houdini 21 (SideFX) — Python Shell + VEX wrangles
- **Node graph**: Grid > ConvertLine > Fuse > (Lanes | Arcs | Vehicle Routes > Vehicles > Awareness > Copy Cars) > Merge > Display
- **Vehicle system**: Parametric curve animation with @Time, per-frame position computation

## What's Implemented

### v5 (Previous)
- Road system, turn rules, direction indicators
- Animated vehicles with random phase offset
- No vehicle awareness, full density

### v6b (Current — Apr 2026)
- **90% car reduction**: `vehicle_density` param (default 0.1), probabilistic spawning for short routes
- **Cross-route awareness**: `vehicle_awareness` wrangle using pcfind spatial queries
- **Safe-position targeting**: Vehicles lerp toward safe position behind blocker (NOT pushback)
- **Priority system**: Deterministic XOR hash per route pair prevents mutual yielding/oscillation
- **Smooth easing**: Hermite smoothstep for natural deceleration curve
- **Capped displacement**: Max 4m adjustment per evaluation prevents jumps
- **Tight forward cone**: 0.85 threshold (~30 degrees) prevents side/perpendicular false detections

### Bug Fix (v6 > v6b)
- REMOVED: sort + gap enforcement in gen_vehicle_points (caused frame-to-frame position swapping)
- REMOVED: tangent pushback in vehicle_awareness (caused back-and-forth oscillation)
- ADDED: deterministic priority hash (prevents mutual yielding at intersections)
- ADDED: safe-position lerp targeting (smooth, continuous, non-oscillating)

### Files Modified
- `setup_traffic_flow.py` — Updated VEX inline + node graph + parameters
- `vex/gen_vehicle_points.vfl` — Clean density-only generation
- `vex/vehicle_awareness.vfl` — Safe-position targeting with priority system

## Backlog
- P1: Traffic light system at intersections
- P2: Bidirectional traffic density balancing
- P2: Dashed center lane markings
- P3: Varied vehicle sizes (cars, buses, trucks)
- P4: Random vehicle colors

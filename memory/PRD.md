# Traffic Sim V2 — PRD

## Original Problem Statement
User has a Houdini 21 traffic simulation project and wants traffic lights placed at all intersections on each incoming traffic lane, with realistic phased signals. Deliverables: standalone VEX files + updated README. Subsequently requested solver integration so vehicles obey signals, then reported all vehicles turning and none going straight.

## Architecture
- **Platform**: SideFX Houdini 21
- **Language**: VEX (with Python node-creation scripts)
- **Grid**: 5x5 intersections (grid_divisions=4, grid_size=348)
- **Existing nodes**: gen_vehicle_routes, resample_routes, route_curves, init_vehicles, solver_step, color_vehicles, car_box, copy_cars

## User Choices
- Realistic phased signals (8-phase: green -> left arrow -> yellow -> all-red, alternating NS/EW)
- Point markers with colour attributes (lightweight, no extra geo)
- All 25 intersections (including edge/corner)
- Standalone .vex files + updated README
- Solver integration: vehicles obey signals (stop on red, go on green)

## What's Been Implemented (Jan 2026)

### Session 1 — Traffic Light Visual Markers
1. **`05_gen_traffic_lights.vex`** — Detail wrangle, 80 signal heads across 25 intersections, 3-bulb + single-point mode, 8-phase cycle.
2. **`06_gen_traffic_light_poles.vex`** — Optional pole geometry.
3. **`README_SETUP.md`** — Complete documentation.

### Session 2 — Solver Integration
4. **`07_signal_brake.vex`** — Vehicles detect signals via grid math, apply smooth braking. Lane-aware arrow phase, yellow dilemma zone.

### Session 3 — Straight-Through Routing Fix
5. **`03_solver_step.vex`** — Fixed Phase 5 route selection scoring:
   - **Root cause**: Old code gave turns a +0.3 score bonus, which combined with the random jitter (0–0.5) almost always beat the straight-through alignment score (~1.0 vs turn ~0.7+0.3+random).
   - **Fix**: Removed turn bonus. Added `straight_bias` parameter (default 0.55) that boosts `intersection_straight` and `road` segments. Reduced random jitter from 0.5 to 0.25.
   - **Result**: ~65% straight, ~35% turns — matches real-world traffic distribution. Fully tunable via `straight_bias` param.

## Backlog
- **P0**: None
- **P1**: Per-intersection timing offsets (green wave / traffic-adaptive signals)
- **P2**: Copy-to-points traffic light housing geometry (box + spheres model)
- **P2**: Pedestrian signal phase
- **P3**: Emergency vehicle signal preemption

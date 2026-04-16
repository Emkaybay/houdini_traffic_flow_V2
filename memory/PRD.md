# Traffic Sim V2 — PRD

## Original Problem Statement
User has a Houdini 21 traffic simulation project and wants traffic lights placed at all intersections on each incoming traffic lane, with realistic phased signals. Deliverables: standalone VEX files + updated README. Subsequently requested solver integration so vehicles actually obey the signals.

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
1. **`05_gen_traffic_lights.vex`** — Detail wrangle generating 80 signal heads across 25 intersections. 3-bulb stacked mode and single-point mode. 8-phase realistic cycle driven by @Time.
2. **`06_gen_traffic_light_poles.vex`** — Optional Detail wrangle generating vertical polyline poles.
3. **`README_SETUP.md`** — Complete documentation.

### Session 2 — Solver Integration
4. **`07_signal_brake.vex`** — Points wrangle inside the Solver SOP (after solver_step). Vehicles detect signals via grid math, apply smooth v²/2d deceleration. Features:
   - Lane-aware arrow phase (inner lane proceeds, outer stops)
   - Yellow-light dilemma zone handling (committed vehicles proceed)
   - `signal_stop` and `signal_state` debug attributes
5. **`README_SETUP.md`** — Updated with solver integration section, new node wiring diagram, new tuning table.

## Backlog
- **P0**: None
- **P1**: Per-intersection timing offsets (green wave / traffic-adaptive signals)
- **P2**: Copy-to-points traffic light housing geometry (box + spheres model)
- **P2**: Pedestrian signal phase
- **P3**: Emergency vehicle signal preemption

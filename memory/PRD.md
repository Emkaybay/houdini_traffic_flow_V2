# Traffic Sim V2 — PRD

## Original Problem Statement
User has a Houdini 21 traffic simulation project and wants traffic lights placed at all intersections on each incoming traffic lane, with realistic phased signals. Deliverables: standalone VEX files + updated README.

## Architecture
- **Platform**: SideFX Houdini 21
- **Language**: VEX (with Python node-creation scripts)
- **Grid**: 5x5 intersections (grid_divisions=4, grid_size=348)
- **Existing nodes**: gen_vehicle_routes, resample_routes, route_curves, init_vehicles, solver_step, color_vehicles, car_box, copy_cars

## User Choices
- Realistic phased signals (8-phase: green → left arrow → yellow → all-red, alternating NS/EW)
- Point markers with colour attributes (lightweight, no extra geo)
- Visual only (no solver integration — vehicles ignore lights)
- All 25 intersections (including edge/corner)
- Standalone .vex files + updated README

## What's Been Implemented (Jan 2026)
1. **`05_gen_traffic_lights.vex`** — Detail wrangle generating 80 signal heads across 25 intersections. Supports 3-bulb stacked mode and single-point mode. 8-phase realistic cycle driven by @Time.
2. **`06_gen_traffic_light_poles.vex`** — Optional Detail wrangle generating vertical polyline poles at each signal location.
3. **`README_SETUP.md`** — Updated with complete traffic light documentation: setup steps, parameter tables, phase cycle diagram, attribute reference, tuning guide.

## Backlog
- **P0**: None
- **P1**: Solver integration — vehicles stop on red, go on green (user deferred)
- **P2**: Per-intersection timing offsets (green wave / traffic-adaptive signals)
- **P2**: Copy-to-points traffic light housing geometry (box + spheres model)
- **P3**: Pedestrian signal phase

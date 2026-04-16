# PRD — Traffic Sim V2 (Houdini 21)

## Original Problem Statement
Houdini 21 road traffic simulation where cars stop in the middle of intersections without cars in front of them, and some just stop on the middle of the road for no reason. User requested VEX code fixes and README update.

## Architecture
- **Platform**: SideFX Houdini 21
- **Language**: VEX (inside AttribWrangle SOPs)
- **Structure**: Solver SOP (`car_anim`) with two internal wrangles: `solver_step` + `signal_brake`
- **Grid**: 348 units, 4 divisions, 4 lanes (5 lane lines), 3.5m spacing

## Core Requirements
1. Vehicles must follow route curves smoothly without stopping randomly
2. Vehicles must brake for other vehicles ahead (any route, not just same route)
3. Vehicles must obey traffic lights (red/yellow = stop, green = go)
4. Vehicles must chain between road ↔ intersection segments at curve ends
5. Vehicles must respawn when no connecting segment exists

## What's Been Implemented (Jan 2026)
- **03_solver_step.vex**: Complete rewrite fixing 6 root-cause bugs (wrong input reference, straight-line movement, route-only detection, tight braking thresholds, zero tangent, segment chaining)
- **04_signal_brake.vex**: Complete rewrite fixing traffic light matching and braking ramp
- **README.md**: Updated with bug fix documentation, node wiring diagram, parameter/attribute reference, troubleshooting guide

## Root Causes Fixed
1. `primuv(0,…)` → `primuv(1,…)` (wrong input for route curves)
2. Straight-line `@P +=` → curve-following `u_param` advance + `primuv` sample
3. Route-based vehicle detection → `nearpoints()` spatial search with cone+lateral filter
4. Linear lerp with tight thresholds → quadratic ease with wider range
5. Hard-coded direction thresholds → dot-product approach matching
6. Zero tangent guard added for first-frame safety

## Backlog
- P1: Fine-tune `follow_dist`/`stop_dist` for specific traffic density
- P2: Lane-change logic (vehicles currently stay in their lane forever)
- P2: Variable speed limits per segment type (slower in turns)
- P3: Yielding / priority at uncontrolled intersections

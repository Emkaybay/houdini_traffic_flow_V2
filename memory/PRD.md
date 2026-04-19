# Traffic Sim V2 — PRD

## Original Problem Statement
Houdini 21 traffic simulation project. User wants vehicles to detect each other by bounding box so they never collide or overlap. Using the Bound SOP with bounding type Box connected to car_box2. VEX code + updated READMEs requested.

## Architecture
- **Platform:** Houdini 21 (SideFX)
- **Language:** VEX (for SOP wrangles), Python (for node scripting)
- **Two geometry networks:**
  - `/obj/traffic_flow_curves` — road geometry (grid lanes, intersection arcs, indicators)
  - `/obj/traffic_sim_v2` — simulation (vehicles, solver, traffic lights, collision)

## Core Requirements
1. Lane-based route generation (inner/outer lanes, straights, turns)
2. Vehicle spawning & movement along curves
3. Traffic light system (8-phase cycle)
4. Signal braking (vehicles obey lights)
5. **Bounding box collision avoidance (v2.2)** — OBB-based, trailing vehicle brakes
6. Cross-lane detection at intersections

## What's Been Implemented
- **v2.2**: Bounding box collision avoidance (reactive, replaced)
- **v2.3**: Predictive collision with TCA (fixed opposite-direction on straights)
- **v2.4**: Signal-aware predictive (fixed cross-traffic at red lights)
- **v2.5**: Straight-passing lateral clearance (fixed passing vehicles on own lanes)
- **v2.7**: Centralized collision architecture
  - Removed ALL collision detection from solver_step (Phase 1 + Phase 2)
  - solver_step now only: accelerate, advance, route-switch, position update
  - bbox_collision is the SOLE collision handler: following, cross-lane, left-turn yield, emergency
  - Added position correction: pulls vehicle back along route when braking (prevents overshoot)
  - Left-turn yield uses geometric check (oncoming + in intersection zone) — works for multi-vehicle scenarios
  - New parameter: `vehicle_offset` on bbox_collision (for position correction)

## Prioritized Backlog
- **P0**: (none — core collision detection complete)
- **P1**: Add different vehicle types/models with varying bounding box sizes
- **P1**: Per-vehicle-type speed limits and acceleration profiles
- **P2**: Lane changing logic (vehicles can switch inner↔outer)
- **P2**: Pedestrian crossings at intersections
- **P3**: Emergency vehicle priority (override traffic signals)

## Next Tasks
1. User to test `08_bbox_collision.vex` in Houdini and tune parameters
2. Add additional vehicle models (user mentioned "different vehicles along the way")
3. Per-model Bound SOP with unique half-extents passed to solver

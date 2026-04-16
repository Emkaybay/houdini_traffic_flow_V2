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
- **2026-01-XX — v2.2**: Bounding box collision avoidance
  - `08_bbox_collision.vex` — OBB collision detection using SAT, inside solver
  - Bound SOP setup instructions for car_box2
  - Cross-lane intersection detection (angular filter > 45°)
  - Updated README.md and README_SETUP.md with full documentation

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

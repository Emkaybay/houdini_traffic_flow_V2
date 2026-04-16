# PRD — Traffic Sim V2 (Houdini 21)

## Original Problem Statement
Houdini 21 road traffic simulation where some cars just stop on the middle of the intersections without cars in front of them, and some just stop on the middle of the road for no reason. User requested VEX code fix and README update.

## Architecture
- **Platform**: SideFX Houdini 21
- **Language**: VEX (inside AttribWrangle SOPs)
- **Solver**: `car_anim` (Solver SOP) containing `solver_step` + `signal_brake`
- **Inputs**: solver_step Input 0 = prev_frame, Input 1 = route_curves (Object Merge)
- **Grid**: 348 units, 4 divisions, 4 lanes (5 lane lines), 3.5m spacing

## Core Requirements
1. Fix vehicles stopping in middle of intersections for no reason
2. Fix vehicles stopping on open road for no reason
3. Preserve all existing behaviour (route switching, respawn, signal braking)
4. Provide updated VEX file + README

## What's Been Implemented (Jan 2026)

### v2.1 — Three surgical fixes in `03_solver_step.vex`

**Fix A** (Phase 1 — lateral filter):
- Changed `safe_dist * 0.6` → `safe_dist * 0.4`
- Adjacent-lane vehicles (3.5m apart) no longer falsely trigger same-lane following

**Fix B** (Phase 2 — intersection skip):
- Added `segment_type` check: if vehicle is on `intersection_straight`, `right_turn`, or `left_turn`, skip entire cross-traffic phase
- Prevents vehicles from yielding mid-intersection

**Fix C** (Phase 2 — same-direction filter):
- Added `if(dot(my_dir, nb_dir) > 0.5) continue;`
- Vehicles on parallel routes going the same direction were falsely triggering cross-traffic yields

### Files modified
- `03_solver_step.vex` — 3 changes (Phase 1 lateral, Phase 2 skip, Phase 2 filter)
- `README.md` — updated with fix documentation

### Files unchanged
- `01_gen_vehicle_routes.vex`
- `02_init_vehicles.vex`
- `05_color_vehicles.vex`
- `07_signal_brake.vex`
- All node wiring and parameters

## Backlog
- P1: Test with different traffic densities to validate fix
- P2: Variable speed limits per segment type (slower in turns)
- P2: Lane-change logic
- P3: Yielding / priority at uncontrolled intersections

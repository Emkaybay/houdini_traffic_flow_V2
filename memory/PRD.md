# PRD: Procedural Traffic Flow Intersection Curves — Houdini 21

## Original Problem Statement
Modify Houdini traffic flow project to:
1. Reduce cars by 90%
2. Make cars aware of each other
3. Fix bouncing/jittering — realistic deceleration
4. Compute 24 frames in advance per car
5. Add debug prediction line that bends on turns
6. Decelerate and stop realistically when about to collide

## Architecture
- **Platform**: Houdini 21 (SideFX) — Python Shell + VEX wrangles
- **Node graph**: Routes > gen_vehicle_points (vehicles + predictions) > vehicle_awareness > blast_split > display
- **Vehicle system**: Parametric curve animation with @Time, prediction polylines, pcfind collision detection

## What's Implemented

### v6c (Current — Apr 2026)
- **90% car reduction**: vehicle_density=0.1, probabilistic spawning
- **24-frame prediction polylines**: Each vehicle generates a polyline along its actual route curve showing where it will be for the next 24 frames. Bends on turns.
- **Frame-by-frame collision detection**: Awareness wrangle collects own predictions + others' predictions, compares at matching frames. Detects collisions up to 1 second before they happen.
- **Distance-based braking**: Computes room-to-collision, converts to frames-of-room, applies Hermite S-curve braking. Pullback capped at one frame of travel (~0.6m).
- **Priority system**: XOR hash per route pair prevents mutual yielding
- **Debug visualization**: Cyan prediction lines visible above each vehicle
- **New node graph**: awareness runs BEFORE blast (needs prediction data), two blast nodes split vehicles from predictions

### Version History
- v5: Initial animated vehicles
- v6: Added density + awareness (bounced/glitched)
- v6b: Fixed bouncing with safe-position targeting + oncoming filter
- v6c: Prediction-based collision with debug lines, realistic braking

## Backlog
- P1: Traffic light system at intersections
- P2: Bidirectional traffic density balancing
- P3: Varied vehicle sizes (cars, buses, trucks)
- P4: Random vehicle colors

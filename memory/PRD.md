# PRD: Procedural Traffic Flow Intersection Curves — Houdini 21

## Original Problem Statement
Houdini traffic flow project modifications:
1. Reduce cars by 90%
2. Make cars aware of each other
3. Fix bouncing/jittering — realistic deceleration
4. Compute 24 frames in advance per car
5. Debug prediction line that bends on turns
6. Decelerate and stop realistically
7. Line length based on vehicle speed
8. Straight-through vehicles must detect each other at intersections
9. No overlapping vehicles

## What's Implemented (v6d — Apr 2026)

### Core System
- 90% car reduction (vehicle_density=0.1)
- 24-frame prediction polylines per vehicle (follows actual curve, bends on turns)
- Frame-by-frame collision detection comparing prediction paths
- Distance-based braking with Hermite S-curve easing
- Priority system (XOR hash) prevents mutual yielding

### v6d Fixes (latest)
- **Search radius doubled** (22m > 37m): pcfind now searches from midpoint of prediction path with v_speed*2.5 radius. Straight-through vehicles at intersections now properly detected.
- **Speed-proportional debug lines**: pred_truncate node reads vehicle's brake factor and removes excess prediction points. Fast = long line, braking = short line.
- **New file**: vex/pred_truncate.vfl
- **New node**: pred_truncate wrangle between awareness and blast

### Node Graph
```
gen_vehicle_routes > resample > gen_vehicle_points > vehicle_awareness > pred_truncate
                                                                            |
                                                          blast_vehicles > copy_cars > color_veh
                                                          blast_predictions > color_pred
                                                          merge(roads, veh, pred) > display
```

## Files
- setup_traffic_flow.py — Main setup script
- vex/gen_vehicle_points.vfl — Vehicles + prediction polylines
- vex/vehicle_awareness.vfl — Predictive collision avoidance
- vex/pred_truncate.vfl — Speed-proportional line truncation
- vex/gen_vehicle_routes.vfl, gen_road_lanes.vfl, gen_intersection_arcs.vfl, classify_intersections.vfl, color_visualization.vfl — Unchanged

## Backlog
- P1: Traffic light system at intersections
- P2: Varied vehicle sizes
- P3: Random vehicle colors

# Houdini Road System V2 - Vehicle Intersection Fix

## Problem
Vehicles were drifting outside lane markings during turns at intersections.
The Bezier curve handles in `gen_vehicle_routes` didn't match the lane boundary
curves drawn by `traffic_flow_curves`, causing the vehicle arcs to cut across
lane lines.

## Root Cause Analysis

### Lane Boundary Curves (traffic_flow_curves)

**Right Turn Boundaries:**
| Curve | Offset from center | Handle calculation |
|-------|-------------------|-------------------|
| Outer boundary (`gen_right_intersection_outer`) | `2.0 * lane_spacing = 7.0` | `entry_dist * 0.4 = 8.0` |
| Inner boundary (`gen_right_intersection_inner`) | `1.0 * lane_spacing = 3.5` | `entry_dist * 0.5 = 10.0` |

**Left Turn Boundaries:**
| Curve | Offset from center | Handle calculation |
|-------|-------------------|-------------------|
| Outer boundary (`gen_left_intersection_outer`) | `1.0 * lane_spacing = 3.5` | `entry_dist * 1.2 * 0.6 = 14.4` |
| Inner boundary (`gen_left_intersection_inner`) | `1.0 * lane_spacing = 3.5` | `entry_dist * 1.2 * 0.6 = 14.4` |

### Vehicle Routes (traffic_sim) - BEFORE FIX

| Turn Type | Vehicle Offset | Handle (OLD) | Issue |
|-----------|---------------|--------------|-------|
| Right turn | `1.5 * ls = 5.25` | `(entry_dist - outer_off) * 0.5523 = 8.15` | Doesn't follow boundary curvature |
| Left turn | `0.5 * ls = 1.75` | `(entry_dist + inner_off) * 0.5523 = 12.01` | Doesn't follow boundary curvature |

### Vehicle Routes - AFTER FIX

| Turn Type | Vehicle Offset | Handle (NEW) | How calculated |
|-----------|---------------|--------------|----------------|
| Right turn | `1.5 * ls = 5.25` | `entry_dist * 0.45 = 9.0` | Average of boundary handles (0.4 + 0.5)/2 |
| Left turn | `0.5 * ls = 1.75` | `entry_dist * 0.72 = 14.4` | Matches boundary pattern (1.2 * 0.6) |

## Files

### gen_vehicle_routes_FIXED.vex
**Replace the VEX snippet in:** `/obj/traffic_sim/gen_vehicle_routes`

Changes:
- Right turn handle: `entry_dist * 0.45` (was `(entry_dist - outer_off) * 0.5523`)
- Left turn handle: `entry_dist * 0.72` (was `(entry_dist + inner_off) * 0.5523`)
- All other code unchanged (straight routes, edge points, route_type attributes)

## How to Apply in Houdini 21

1. Open your `Road_SystemV2.hip` file
2. Navigate to `/obj/traffic_sim/gen_vehicle_routes`
3. Open the VEX Expression Editor (click the code icon on the wrangle)
4. Select all existing code and replace with contents of `gen_vehicle_routes_FIXED.vex`
5. Press Alt+E or click Apply to compile
6. Scrub the timeline to verify vehicles follow between lanes

## Parameter Requirements (unchanged)
These parameters must exist on the `gen_vehicle_routes` wrangle node:
- `grid_size`: 348
- `grid_divisions`: 4
- `lane_spacing`: 3.5
- `entry_dist`: 20
- `arc_segments`: 16

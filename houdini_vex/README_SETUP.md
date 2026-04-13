# Traffic Sim V2 — Realistic Lane-Based Intersections

## How It Works

Routes are **short segments** between intersections (not edge-to-edge).
At each intersection boundary, the vehicle picks a new segment based on **lane rules**:

```
OUTER LANE (right side of road):
  → Can go STRAIGHT through
  → Can turn RIGHT
  → Cannot turn left

INNER LANE (left side of road):
  → Can go STRAIGHT through
  → Can turn LEFT
  → Cannot turn right
```

This matches real-world traffic rules.

## Segment Types

```
       intersection
        boundary
           |
  ─────────┤ ├─────────     "road" segments (between intersections)
           |X|
           |X|               "intersection_straight" (through intersection)
           |X|
  ─────────┤ ├─────────
           |
       intersection
        boundary

  Right turn arc:  outer lane → perpendicular outer lane
  Left turn arc:   inner lane → perpendicular inner lane
```

## Node Network

```
gen_vehicle_routes (Detail) ─→ resample_routes ─→ route_curves (Null)
                                                        │
                                    ┌───────────────────┘
                                    │
init_vehicles (Detail) ─────→ traffic_solver (Solver SOP)
                                    │
                              Inside Solver:
                                prev_frame ──→ solver_step (input 0)
                                Object Merge → solver_step (input 1)
                                solver_step ──→ Output
                                    │
                              color_vehicles ──→ copy_cars ──→ DISPLAY
                                                    ↑
                                               car_box (Box)
```

## Setup

### 1. gen_vehicle_routes
- Wrangle, **Detail**, paste `01_gen_vehicle_routes.vex`
- Parameters: grid_size=348, grid_divisions=4, lane_spacing=3.5, entry_dist=20, arc_segments=16

### 2. resample_routes
- Resample SOP, length=2

### 3. route_curves
- Null SOP

### 4. init_vehicles
- Wrangle, **Detail**, paste `02_init_vehicles.vex`
- Wire route_curves → input 0
- Parameters: vehicle_speed=15, vehicle_spacing=25, vehicle_density=0.3, vehicle_offset=0.5, random_seed=42

### 5. Solver
- Solver SOP, wire init_vehicles → input
- Inside: prev_frame → solver_step input 0
- Inside: Object Merge (path: /obj/traffic_sim_v2/route_curves) → solver_step input 1
- solver_step: Wrangle, **Points**, paste `03_solver_step.vex`
- Parameters: max_speed=15, acceleration=8, deceleration=25, look_ahead_dist=25, cross_detect_dist=30, min_safe_dist=10, cross_safe_time=1.5, search_count=150, vehicle_offset=0.5, route_match_dist=2.0

### 6. color_vehicles
- Wrangle, **Points**, paste `04_color_vehicles.vex`

### 7. car_box + copy_cars
- Box: 4 x 1.5 x 2, center Y = 0.75
- Copy to Points: car_box → input 1, color → input 2

## Parameters

### solver_step
| Param | Default | Effect |
|-------|---------|--------|
| max_speed | 15 | Cruising speed |
| acceleration | 8 | Speed up (units/sec²) |
| deceleration | 25 | Brake force (units/sec²) |
| look_ahead_dist | 25 | Following detection range |
| cross_detect_dist | 30 | Intersection detection range |
| min_safe_dist | 10 | Gap to maintain |
| cross_safe_time | 1.5 | Crossing clearance (seconds) |
| search_count | 150 | pcfind max neighbors |
| vehicle_offset | 0.5 | Y height |
| route_match_dist | 2.0 | Segment connection tolerance |

## Tuning

| Problem | Fix |
|---------|-----|
| Vehicles don't switch segments | Increase route_match_dist to 3.0 |
| Never turn right/left | Check lane_type attribute exists on routes |
| Too few vehicles | Increase vehicle_density to 0.5 |
| Collisions at intersections | Increase min_safe_dist to 14 |
| Vehicles stop and don't restart | Check solver Object Merge resolves |

**After changing init_vehicles params → go to frame 1!**

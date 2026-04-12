# Procedural Traffic Flow Intersection Curves — Houdini 21

Right-hand traffic, 2 lanes per direction, 14m road (3.5m lane spacing).
Inner lane = left turn only. Outer lane = right turn only.
Direction arrows placed between the lane lines.

**v6c: Predictive collision avoidance + debug prediction lines**

---

## Quick Start

```python
exec(open(r"C:\Users\KABELO\Downloads\houdini_traffic_flow\setup_traffic_flow.py").read())
```

---

## What You Get

| Element | Color | Description |
|---------|-------|-------------|
| Road lanes | White | 5 parallel lines per road, 3.5m apart = 14m road |
| Right turn arcs | Red | Bezier curves at corners (outer lane) |
| Left turn arcs | Red | Bezier curves through centre (inner lane) |
| Direction arrows | Amber | V-chevrons in each lane gap before intersection |
| Grid lines | Green | Original grid reference |
| Vehicles | Blue | Animated boxes flowing along all lanes + turns |
| Prediction lines | Cyan | Debug: 24-frame lookahead per vehicle (bends on turns) |

## v6c — Predictive Vehicle Awareness

### How It Works

1. **24-Frame Prediction Paths**: Each vehicle generates a polyline showing exactly where it will be for the next 24 frames (~1 second). The path follows the actual route curve, so it **bends on turns**.

2. **Frame-by-Frame Collision Detection**: The awareness wrangle collects its own prediction path AND nearby vehicles' prediction paths. It compares them frame-by-frame: "At frame 12, will I be within 5m of another vehicle's frame-12 position?"

3. **Distance-Based Braking**: When a collision is predicted:
   - Computes distance along travel direction to the collision point
   - Subtracts safe stop distance (8m) to get "room"
   - Converts room to "frames of room" at current speed
   - Applies Hermite S-curve braking: gentle onset, firm finish
   - Pullback = brake_factor * one_frame_of_travel (~0.6m max)

4. **Priority System**: Deterministic XOR hash per route pair. For each encounter, exactly one vehicle yields. No mutual yielding, no oscillation.

### Debug Prediction Lines (Cyan)
- Visible above each vehicle as a cyan polyline
- Shows the exact path the vehicle will follow for the next second
- **Bends on turns** — follows the actual route curve, not a straight line
- Useful for understanding collision detection behavior

### Node Graph (v6c)

```
gen_vehicle_routes (Detail)
  |
resample_routes
  |
gen_vehicle_points (Primitives — creates vehicles + prediction polylines)
  |
vehicle_awareness  (Points — prediction-based collision check)
  |
  +-- blast_vehicles     --> copy_cars --> color_vehicles (blue)
  |
  +-- blast_predictions  --> color_predictions (cyan debug lines)
  |
merge(roads, vehicles, predictions) --> display
```

## Parameters

### Vehicle Parameters (`gen_vehicle_points`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `vehicle_speed` | 15.0 | Speed in m/s (~54 km/h) |
| `vehicle_spacing` | 30.0 | Base gap between vehicles (metres) |
| `vehicle_offset` | 0.75 | Y lift above road surface |
| `vehicle_density` | 0.1 | Fraction of full count (0.1 = 90% reduction) |
| `pred_frames` | **24** | **Lookahead frames for prediction path** |

### Awareness Parameters (`vehicle_awareness`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stop_distance` | 8.0 | Safe gap behind blocker (car length + buffer) |
| `collision_threshold` | **5.0** | **Max distance between predictions to count as collision** |
| `vehicle_speed` | 15.0 | Must match gen_vehicle_points speed |

### Lane Parameters (`gen_road_lanes`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_size` | 348 | Total grid dimension |
| `grid_divisions` | 4 | Cells per axis |
| `num_lane_lines` | 5 | Lines per road (5 lines = 4 lanes) |
| `lane_spacing` | 3.5 | Gap between lines (metres) |

### Arc & Indicator Parameters (`gen_intersection_arcs`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lane_spacing` | 3.5 | Must match lane generation |
| `entry_dist` | 20.0 | Where arcs start/end from intersection centre |
| `arc_segments` | 16 | Smoothness of turn curves |
| `chevron_size` | 3.0 | Size of direction arrow indicators |
| `indicator_dist` | 32.0 | How far before intersection the arrows sit |

**Animation:** Press **Play** on the Houdini timeline to see vehicles move.
Set frame range to **1-240** for 10 seconds of animation at 24fps.

## Files

```
houdini_traffic_flow/
  setup_traffic_flow.py              <- Run in Python Shell
  README.md                          <- This file
  vex/
    classify_intersections.vfl       <- Detect intersection types
    gen_road_lanes.vfl               <- Multi-lane roads (3.5m spacing)
    gen_intersection_arcs.vfl        <- Turn arcs + direction arrows
    gen_vehicle_routes.vfl           <- Complete route polylines
    gen_vehicle_points.vfl           <- Vehicles + prediction paths (v6c)
    vehicle_awareness.vfl            <- Predictive collision avoidance (v6c)
    color_visualization.vfl          <- White/red/amber/green colours
```

# Procedural Traffic Flow Intersection Curves — Houdini 21

Right-hand traffic, 2 lanes per direction, 14m road (3.5m lane spacing).
Inner lane = left turn only. Outer lane = right turn only.
Direction arrows placed between the lane lines.

**v6: 90% car reduction + vehicle awareness system**

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

## Lane Rules (Right-Hand Traffic)

```
             Travel direction
                  ^
    ----------+----+----+---------
    Oncoming  | L  | R  | Your side
    traffic   |turn|turn| of road
    ----------+----+----+---------
             inner outer
             lane  lane
```

- **Inner lane** (closer to centre line): Left turn only
- **Outer lane** (closer to curb): Right turn only
- At 3-way/corner intersections: if a turn is impossible, that lane shows straight-ahead arrow instead

## v6 Changes: Vehicle Awareness System

### 90% Car Reduction
- `vehicle_density` parameter (default **0.1** = 10% of original count)
- Adjustable from 0.01 (1%) to 1.0 (100%)
- Fewer vehicles = clearer simulation, better performance

### Same-Route Awareness (Following Distance)
- Vehicles on the **same route** maintain minimum following distance
- `min_follow_dist` (default **15m**) — vehicles won't tailgate
- Trailing vehicles are pushed back along the curve if too close to the one ahead
- Cascading enforcement: entire platoon spaces out from the leader

### Cross-Route Awareness (Intersection Avoidance)
- New `vehicle_awareness` node uses **pcfind** spatial queries
- Detects vehicles from **different routes** that are close and ahead
- Vehicles slow/stop when a cross-route vehicle is within their forward cone (~60 degrees)
- `awareness_radius` (default **20m**) — how far to look for other vehicles
- `stop_distance` (default **6m**) — hard stop distance from another vehicle
- `slow_distance` (default **15m**) — start decelerating at this range
- Smooth deceleration via tangent-direction pushback

### Node Graph (v6)

```
route_null
  |
gen_vehicle_routes  (Detail wrangle — creates route polylines)
  |
resample_routes     (Smooth interpolation)
  |
gen_vehicle_points  (Prim wrangle — density + same-route following dist)
  |
keep_vehicles_only  (Blast — discard route geometry)
  |
vehicle_awareness   (Point wrangle — cross-route pcfind avoidance)  <-- NEW
  |
copy_cars           (Copy box to points)
  |
color_vehicles      (Blue)
```

## Parameters

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

### Vehicle Parameters (`gen_vehicle_points`) — UPDATED v6

| Parameter | Default | Description |
|-----------|---------|-------------|
| `vehicle_speed` | 15.0 | Speed in m/s (~54 km/h) |
| `vehicle_spacing` | 30.0 | Base gap between vehicles (metres) |
| `vehicle_offset` | 0.75 | Y lift above road surface |
| `vehicle_density` | **0.1** | **Fraction of full count (0.1 = 10%, 90% reduction)** |
| `min_follow_dist` | **15.0** | **Minimum following distance on same route (metres)** |

### Awareness Parameters (`vehicle_awareness`) — NEW v6

| Parameter | Default | Description |
|-----------|---------|-------------|
| `awareness_radius` | 20.0 | Search radius for nearby vehicles from other routes |
| `stop_distance` | 6.0 | Hard stop distance (vehicle length + buffer) |
| `slow_distance` | 15.0 | Start decelerating at this range |

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
    gen_vehicle_points.vfl           <- Animated vehicles (v6: density + following dist)
    vehicle_awareness.vfl            <- Cross-route avoidance (v6: pcfind)
    color_visualization.vfl          <- White/red/amber/green colours
```

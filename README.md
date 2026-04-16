# Traffic Sim V2 — Houdini 21 VEX Traffic Simulation

## Overview

VEX-driven traffic simulation with lane rules, traffic lights, curve-following
movement, and **bounding-box collision avoidance**.  Vehicles navigate
lane-based routes through a grid city, obey traffic signals, and use Oriented
Bounding Boxes (OBB) to prevent overlap.

### Key Features
- Lane rules (outer → right turn / straight, inner → left turn / straight)
- Curve-following movement along resampled polylines
- Proximity-based following distance
- **OBB bounding-box collision detection (NEW in v2.2)**
- Cross-traffic yielding at intersections
- Probability-based route switching (`straight_bias`)
- Edge respawn for continuous traffic flow
- 8-phase traffic light system with signal braking
- Brake-intensity colour feedback (green → yellow → red)

### Segment Types

| Type                    | Description                               |
|:------------------------|:------------------------------------------|
| `road`                  | Straight segment between two intersections|
| `intersection_straight` | Straight through an intersection          |
| `right_turn`            | Outer-lane right-turn arc (Bézier)        |
| `left_turn`             | Inner-lane left-turn arc (Bézier)         |

---

## Bug Fixes (v2.1)

Only `03_solver_step.vex` was changed.  Fixed vehicles stopping randomly due
to route-matching issues and edge-respawn bugs.

---

## What's New in v2.2 — Bounding Box Collision Avoidance

### Problem
Vehicles could overlap or pass through each other when basic following-distance
logic was insufficient — especially at intersections where paths cross at
angles.

### Solution
A new solver wrangle (`08_bbox_collision.vex`) computes **Oriented Bounding
Boxes** for every vehicle using the actual car geometry dimensions captured by
a **Bound SOP** on `car_box2`.  It performs a per-frame **Separating Axis
Theorem (SAT)** test in 2D (ground plane) across 4 axes to detect overlap or
imminent collision.

### Behaviour
- **Trailing vehicle decelerates** proportionally to the overlap urgency
  (smooth braking, not a binary stop).
- **Hard stop** on actual penetration (`worst_gap < 0`).
- **Cross-lane detection at intersections**: when a vehicle is within
  `intersection_radius` of a grid intersection, it also evaluates vehicles on
  other lanes — but only if their travel directions actually cross (> ~45°
  angle difference), to avoid false brakes from parallel traffic.

### Solver Chain (Updated)

```
prev_frame → solver_step → bbox_collision → signal_brake → Output
```

> `solver_step` moves vehicles and does basic AI, `bbox_collision` enforces
> bounding-box separation, then `signal_brake` enforces traffic signals.
> Each wrangle reads and writes `f@speed` / `f@brake` cumulatively.

---

## Node Network

### Object Level
- `/obj/traffic_flow_curves` — road geometry display
- `/obj/traffic_sim_v2` — simulation (vehicles + lights)

### Inside `/obj/traffic_sim_v2`

```
gen_vehicle_routes ──► resample_routes ──► route_curves (Null)
                                               │
                                               ▼
                                         init_vehicles
                                               │
                                               ▼
                                       traffic_solver (Solver SOP)
                                       ┌───────────────────────────┐
                                       │ prev_frame                │
                                       │     │                     │
                                       │     ▼                     │
                                       │ solver_step               │
                                       │     │                     │
                                       │     ▼                     │
                                       │ bbox_collision  ◄── NEW   │
                                       │     │                     │
                                       │     ▼                     │
                                       │ signal_brake              │
                                       │     │                     │
                                       │     ▼                     │
                                       │   Output                  │
                                       └───────────────────────────┘
                                               │
                                               ▼
                                         color_vehicles
                                               │
                                               ▼
car_box2 ──► bound_car (Bound SOP, NEW)   copy_cars ◄── edit1
                                               │
                                               ▼
gen_traffic_lights ─┐                      merge_all
gen_light_poles ────┤                          │
                    ▼                          ▼
              merge_lights ──────────────► OUTPUT
```

---

## VEX Files Reference

| File                             | Node               | Run Over | Purpose                             |
|:---------------------------------|:-------------------|:---------|:------------------------------------|
| `01_gen_vehicle_routes.vex`      | gen_vehicle_routes | Detail   | Road network geometry               |
| `02_init_vehicles.vex`           | init_vehicles      | Detail   | Spawn vehicles on roads             |
| `03_solver_step.vex`             | solver_step        | Points   | Vehicle movement & AI               |
| `04_color_vehicles.vex`          | color_vehicles     | Points   | Brake-based vehicle colouring       |
| `05_gen_traffic_lights.vex`      | gen_traffic_lights | Detail   | Traffic signal points               |
| `06_gen_traffic_light_poles.vex` | gen_light_poles    | Detail   | Signal pole geometry (optional)     |
| `07_signal_brake.vex`            | signal_brake       | Points   | Vehicles obey traffic signals       |
| **`08_bbox_collision.vex`**      | **bbox_collision** | **Points** | **OBB collision avoidance — NEW** |

---

## Setup — Quick Reference

> Full step-by-step is in **README_SETUP.md**.

### v2.2 additions only:

1. **Bound SOP** (`bound_car`): create at `/obj/traffic_sim_v2`, connect input
   to `car_box2` output, set Bounding Type = **Bounding Box**.
2. **`bbox_collision` wrangle**: create inside `traffic_solver`, wire after
   `solver_step` before `signal_brake`, run over Points, paste
   `08_bbox_collision.vex`.
3. **Spare parameters** on `bbox_collision`:
   - `bbox_half_length` = 2.0 (half Bound Z)
   - `bbox_half_width` = 1.0 (half Bound X)
   - `bbox_padding` = 1.5
   - `search_radius` = 30
   - `max_neighbors` = 50
   - `brake_force` = 25
   - `intersection_radius` = 25
   - `grid_size` = 348
   - `grid_divisions` = 4

---

## Parameter Reference

### `solver_step` Parameters

| Param              | Type  | Default | Effect                           |
|:-------------------|:------|:--------|:---------------------------------|
| `max_speed`        | float | 15      | Cruising speed                   |
| `acceleration`     | float | 8       | Speed up (units/s^2)             |
| `deceleration`     | float | 25      | Brake force (units/s^2)          |
| `look_ahead_dist`  | float | 25      | Following detection range        |
| `cross_detect_dist`| float | 30      | Intersection detection range     |
| `min_safe_dist`    | float | 10      | Minimum gap to maintain          |
| `cross_safe_time`  | float | 1.5     | Crossing clearance (seconds)     |
| `search_count`     | int   | 150     | Max neighbours for searches      |
| `vehicle_offset`   | float | 0.5     | Y height for vehicles            |
| `route_match_dist` | float | 2.0     | Segment connection tolerance     |
| `straight_bias`    | float | 0.65    | Probability of going straight    |
| `grid_size`        | float | 348     | Edge respawn (match route gen)   |
| `entry_dist`       | float | 20      | Edge respawn (match route gen)   |

### `bbox_collision` Parameters (NEW)

| Param                 | Type  | Default | Effect                                          |
|:----------------------|:------|:--------|:------------------------------------------------|
| `bbox_half_length`    | float | 2.0     | Half car length along forward (from Bound SOP Z)|
| `bbox_half_width`     | float | 1.0     | Half car width lateral (from Bound SOP X)       |
| `bbox_padding`        | float | 1.5     | Extra safety margin around each bounding box    |
| `search_radius`       | float | 30.0    | pcfind neighbour search radius                  |
| `max_neighbors`       | int   | 50      | Max neighbours evaluated per vehicle per frame  |
| `brake_force`         | float | 25.0    | Deceleration for collision avoidance (units/s^2)|
| `intersection_radius` | float | 25.0    | Cross-lane detection zone around intersections  |
| `grid_size`           | float | 348     | Match `gen_vehicle_routes`                      |
| `grid_divisions`      | int   | 4       | Match `gen_vehicle_routes`                      |

### `signal_brake` Parameters

| Param                | Type  | Default | Effect                            |
|:---------------------|:------|:--------|:----------------------------------|
| `grid_size`          | float | 348     | Match route gen                   |
| `grid_divisions`     | int   | 4       | Match route gen                   |
| `entry_dist`         | float | 20      | Match route gen                   |
| `green_time`         | float | 10      | Match traffic lights              |
| `arrow_time`         | float | 4       | Match traffic lights              |
| `yellow_time`        | float | 3       | Match traffic lights              |
| `clearance_time`     | float | 1       | Match traffic lights              |
| `signal_detect_dist` | float | 45      | How far vehicles see signals      |
| `signal_deceleration`| float | 25      | Max braking for signal stops      |
| `stop_margin`        | float | 1.5     | Distance from stop line for stop  |

---

## Attribute Reference

### Vehicle Point Attributes

| Attribute        | Type   | Set By           | Description                              |
|:-----------------|:-------|:-----------------|:-----------------------------------------|
| `P`              | vector | solver_step      | World position                           |
| `N`              | vector | solver_step      | Forward direction (tangent)              |
| `up`             | vector | init_vehicles    | Up vector (0,1,0)                        |
| `speed`          | float  | solver_step / bbox_collision / signal_brake | Current speed |
| `target_speed`   | float  | solver_step      | Desired speed                            |
| `brake`          | float  | solver_step / bbox_collision / signal_brake | Brake intensity 0–1 |
| `u_param`        | float  | solver_step      | Position along current prim (0–1)        |
| `route_id`       | int    | solver_step      | Current route primitive index            |
| `vehicle_id`     | int    | init_vehicles    | Unique vehicle identifier                |
| `prim_length`    | float  | solver_step      | Length of current route prim             |
| `curve_type`     | string | init_vehicles    | Always `"vehicle"` for vehicles          |
| `lane_type`      | string | init_vehicles    | `"inner"` or `"outer"`                   |
| `signal_stop`    | int    | signal_brake     | 1 if stopped by a traffic signal         |
| `signal_state`   | string | signal_brake     | Signal state seen by vehicle             |
| **`bbox_stopped`** | **int** | **bbox_collision** | **1 if braking due to OBB proximity — NEW** |

### Traffic Light Attributes

| Attribute         | Type   | Description                    |
|:------------------|:-------|:-------------------------------|
| `Cd`              | vector | Bulb colour                    |
| `Alpha`           | float  | 1.0 active / 0.25 dim         |
| `N`               | vector | Approach direction             |
| `pscale`          | float  | Bulb display size              |
| `light_state`     | string | green / left_arrow / yellow / red |
| `bulb_name`       | string | green / yellow / red (3-bulb)  |
| `is_active`       | int    | 1 if this bulb is lit          |
| `intersection_id` | int    | Unique intersection ID         |
| `approach_dir`    | int    | 0=+X, 1=-X, 2=+Z, 3=-Z       |
| `is_ns`           | int    | 1 for NS, 0 for EW            |
| `phase`           | int    | Current global phase (0–7)     |

---

## Traffic Light Phase Cycle

| Time (s) | Phase | NS Approaches     | EW Approaches     |
|:---------|:------|:-------------------|:-------------------|
| 0–10     | 0     | GREEN              | RED                |
| 10–14    | 1     | LEFT-TURN ARROW    | RED                |
| 14–17    | 2     | YELLOW             | RED                |
| 17–18    | 3     | RED (clearance)    | RED (clearance)    |
| 18–28    | 4     | RED                | GREEN              |
| 28–32    | 5     | RED                | LEFT-TURN ARROW    |
| 32–35    | 6     | RED                | YELLOW             |
| 35–36    | 7     | RED (clearance)    | RED (clearance)    |

---

## Bounding Box Collision — Technical Details

### Algorithm: Separating Axis Theorem (SAT) in 2D

Each vehicle's bounding box is an **Oriented Bounding Box (OBB)** — a
rectangle on the ground plane, rotated to align with the vehicle's `@N`
(forward direction).

The OBB is defined by:
- **Centre:** vehicle's `@P` (x, z)
- **Forward axis:** `normalize(@N.xz)`
- **Right axis:** perpendicular to forward on ground plane
- **Half-extents:** `bbox_half_length` (forward) and `bbox_half_width` (lateral), both from the Bound SOP on `car_box2`, plus `bbox_padding`

**SAT tests 4 separating axes** (2 per box):
1. Vehicle A's forward direction
2. Vehicle A's right direction
3. Vehicle B's forward direction
4. Vehicle B's right direction

For each axis, if the projected gap between the two OBBs is positive, the
boxes are separated on that axis.  If ALL 4 gaps are ≤ 0, the boxes overlap.

The **maximum gap** across all axes represents the "most separating" axis.
This value drives the braking response:
- `max_gap > bbox_padding` → no action
- `0 < max_gap ≤ bbox_padding` → proportional braking (approaching)
- `max_gap ≤ 0` → hard stop (overlapping)

### Cross-Lane Logic at Intersections

The wrangle computes the nearest grid intersection via a fast snap:

```vex
float near_ix = rint((@P.x + half_grid) / cell) * cell - half_grid;
float near_iz = rint((@P.z + half_grid) / cell) * cell - half_grid;
```

If the vehicle is within `intersection_radius` of this point, **cross-lane
detection activates**.  However, vehicles on different lanes are only
considered threats if their travel directions differ by more than ~45°
(`abs(dot) < 0.7`), filtering out parallel same-direction traffic that
happens to be on another lane.

### Bound SOP → Half-Extents Mapping

| Bound SOP Axis | car_box2 (after Y=90°) | Copy-to-Points maps to | Parameter           |
|:---------------|:-----------------------|:-----------------------|:--------------------|
| X size = 2     | car width              | lateral (right of N)   | `bbox_half_width`  = 1.0 |
| Y size = 1.5   | car height             | vertical (ignored 2D)  | —                   |
| Z size = 4     | car length             | forward (along N)      | `bbox_half_length` = 2.0 |

---

## Grid Parameters

| Parameter        | Value | Notes              |
|:-----------------|:------|:-------------------|
| `grid_size`      | 348   | World units        |
| `grid_divisions` | 4     | 5×5 intersections  |
| `lane_spacing`   | 3.5   | Lane-to-lane gap   |
| `entry_dist`     | 20    | Intersection boundary |
| `arc_segments`   | 16–32 | Turn arc resolution|

---

## Tuning & Troubleshooting

### Bounding Box Collision (NEW)

| Symptom                          | Solution                                         |
|:---------------------------------|:-------------------------------------------------|
| Vehicles still clip through      | Increase `bbox_padding` to 2.0–3.0               |
| Braking too aggressively         | Decrease `bbox_padding` or `brake_force`          |
| False brakes on parallel lanes   | Decrease `intersection_radius` to 15–20           |
| Missing intersection collisions  | Increase `intersection_radius` to 30–35           |
| Vehicles deadlocked at crossing  | Reduce `bbox_padding`; traffic lights should clear|
| Jerky stop-start behaviour       | Lower `brake_force` to 15–18 for smoother braking |
| Slow with many vehicles          | Reduce `max_neighbors` to 30, `search_radius` to 20 |
| Changed car model                | Re-check Bound SOP, update half_length/half_width |

### Existing Issues

| Symptom              | Solution                                    |
|:---------------------|:--------------------------------------------|
| Segment switching    | Increase `route_match_dist`                 |
| Too many/few turns   | Adjust `straight_bias`                      |
| Edge looping         | Add `grid_size`/`entry_dist` to solver_step |
| Traffic light timing | Adjust phase durations on gen_traffic_lights|
| Ignoring red lights  | Ensure signal_brake wired after bbox_collision |

### Performance Tips

- With 100+ vehicles, reduce `max_neighbors` to 30 and `search_radius` to 20.
- The `pcfind` call is the most expensive operation — smaller radii help significantly.
- If cross-lane detection isn't needed, set `intersection_radius` to 0 to skip it entirely.

---

## Quick Diff: v2.2

| Change                  | Files affected      |
|:------------------------|:--------------------|
| Bound SOP added         | Network only        |
| bbox_collision wrangle  | `08_bbox_collision.vex` (NEW) |
| Solver wiring updated   | solver_step → bbox_collision → signal_brake |
| READMEs updated         | README.md, README_SETUP.md |
| No changes to existing VEX files | 01–07 unchanged |

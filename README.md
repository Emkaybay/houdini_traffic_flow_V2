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
- **OBB predictive collision detection (NEW in v2.3)**
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

## What's New in v2.3 — Predictive Bounding Box Collision

### Problem (v2.2)
The v2.2 reactive collision check tested bounding box overlap at **current
positions only**.  Vehicles passing each other on parallel lanes going opposite
directions would falsely brake — the search radius caught them, and the OBB
test saw proximity, even though the cars would safely pass each other.

### Solution (v2.3) — Predictive Collision with Time-to-Closest-Approach

Instead of asking "are these boxes close now?", the new code asks **"will
these boxes overlap in the near future?"** using the vehicles' velocities.

**Key concept — Time to Closest Approach (TCA):**

```
relative_velocity = neighbour_velocity − my_velocity
TCA = −dot(separation, relative_velocity) / dot(relative_velocity, relative_velocity)
```

- **TCA ≤ 0** → vehicles are moving **apart** → skip entirely (this is the
  fix for opposite-direction traffic)
- **TCA > 0** → project both vehicles to their future positions at TCA →
  run the OBB overlap test **there**

Two cars going opposite directions on offset lanes:
- TCA puts them at the passing moment
- At that moment their lateral offset (lane spacing) keeps the predicted OBBs
  separated → **no braking**

Two cars converging on the same space (merging, turning into each other):
- TCA puts them at the collision moment
- Predicted OBBs overlap → **braking proportional to urgency**

### Two-Phase Detection

| Phase     | Condition                              | Response                        |
|:----------|:---------------------------------------|:--------------------------------|
| Emergency | OBBs overlapping NOW + still converging| Hard stop (prevents penetration)|
| Predictive| OBBs predicted to overlap within `look_ahead_time` | Proportional brake (close = harder, distant = lighter) |

> If two vehicles are already overlapping but moving apart (resolving), the
> emergency phase does NOT trigger — the penetration is clearing on its own.

### Solver Chain (Updated)

```
prev_frame → solver_step → signal_brake → bbox_collision → Output
```

> `solver_step` moves vehicles, `signal_brake` stops red-light vehicles
> (setting `speed=0`, `signal_stop=1`), then `bbox_collision` runs predictive
> collision — it reads `signal_stop` and skips stopped cross-traffic,
> eliminating false brakes at intersections.

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
                                       │ signal_brake              │
                                       │     │                     │
                                       │     ▼                     │
                                       │ bbox_collision  ◄── NEW   │
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

### `bbox_collision` Parameters (NEW — v2.3 Predictive)

| Param                 | Type  | Default | Effect                                               |
|:----------------------|:------|:--------|:-----------------------------------------------------|
| `bbox_half_length`    | float | 2.0     | Half car length along forward (from Bound SOP Z)     |
| `bbox_half_width`     | float | 1.0     | Half car width lateral (from Bound SOP X)            |
| `bbox_padding`        | float | 1.0     | Extra safety margin around each bounding box         |
| `search_radius`       | float | 35.0    | pcfind neighbour search radius                       |
| `max_neighbors`       | int   | 50      | Max neighbours evaluated per vehicle per frame       |
| `brake_force`         | float | 25.0    | Deceleration for collision avoidance (units/s^2)     |
| `look_ahead_time`     | float | 2.0     | How far into the future to predict (seconds)         |
| `emergency_gap`       | float | 0.5     | Gap threshold for immediate emergency braking        |
| `stopped_speed_thresh`| float | 0.5     | Cross-lane vehicles below this speed are skipped     |
| `intersection_radius` | float | 25.0    | Cross-lane detection zone around intersections       |
| `grid_size`           | float | 348     | Match `gen_vehicle_routes`                           |
| `grid_divisions`      | int   | 4       | Match `gen_vehicle_routes`                           |

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

## Bounding Box Collision — Technical Details (v2.3)

### Algorithm: Two-Phase Predictive OBB Detection

#### Phase 1 — Emergency (Current Frame)

Checks if two OBBs are **already overlapping AND still converging** (closing
speed > 0).  If both conditions are true → emergency brake.  If the vehicles
are overlapping but moving apart, the penetration is self-resolving → no action.

#### Phase 2 — Predictive (Future Frame via TCA)

1. **Compute velocities:** `my_vel = fwd * speed`, `nb_vel = nb_fwd * nb_speed`
2. **Relative velocity:** `rel_vel = nb_vel - my_vel`
3. **Time to Closest Approach:**
   ```
   TCA = -dot(separation, rel_vel) / dot(rel_vel, rel_vel)
   ```
4. **TCA ≤ 0 → diverging → skip** (this eliminates opposite-direction traffic)
5. **Project forward:** `my_future = P + my_vel * TCA`, same for neighbour
6. **OBB SAT test** at predicted positions using current orientations
7. **If overlap predicted:** brake proportionally — imminent (low TCA) = hard
   brake, distant (high TCA) = gentle brake

### Why This Fixes Opposite-Direction False Brakes

Two vehicles on parallel lanes going opposite directions:
- They approach each other (closing speed > 0)
- TCA = the moment they're side-by-side
- At TCA, predicted positions have full **lateral offset** (lane spacing ≈ 3.5 units)
- OBB half-widths + padding ≈ 2.0 each → total = 4.0
- Lane spacing (3.5) + lateral offset > 0 → **no overlap** → no braking

Two vehicles merging into the same lane:
- TCA = moment they'd share the same space
- At TCA, predicted positions **converge** (lateral offset → 0)
- OBBs overlap → **braking triggers**

### OBB Gap Function (Separating Axis Theorem)

```vex
function float obb_gap(
    vector sep;
    vector a_fwd; vector a_rgt; float a_hl; float a_hw;
    vector b_fwd; vector b_rgt; float b_hl; float b_hw)
```

Tests 4 separating axes (2 per box).  Returns the largest gap:
- `gap > 0` → separated (no collision)
- `gap ≤ 0` → overlapping

### Cross-Lane Logic at Intersections

Same as v2.2: activates within `intersection_radius` of the nearest grid
intersection.  Only considers vehicles on other lanes if their travel
directions differ by > ~45° (`abs(dot) < 0.7`), filtering parallel traffic.

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

### Bounding Box Collision (v2.3 Predictive)

| Symptom                          | Solution                                         |
|:---------------------------------|:-------------------------------------------------|
| Vehicles still clip through      | Increase `bbox_padding` to 1.5–2.0               |
| Braking too aggressively         | Decrease `look_ahead_time` to 1.0–1.5 seconds    |
| Opposite-direction false brakes  | Should not happen (TCA filters diverging). Check `lane_type` attribs |
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
| Ignoring red lights  | Ensure signal_brake wired before bbox_collision |

### Performance Tips

- With 100+ vehicles, reduce `max_neighbors` to 30 and `search_radius` to 20.
- The `pcfind` call is the most expensive operation — smaller radii help significantly.
- If cross-lane detection isn't needed, set `intersection_radius` to 0 to skip it entirely.

---

## Quick Diff: v2.4

| Change                  | Files affected      |
|:------------------------|:--------------------|
| Bound SOP added         | Network only        |
| bbox_collision wrangle  | `08_bbox_collision.vex` (predictive + signal-aware) |
| **Solver wiring changed** | **solver_step → signal_brake → bbox_collision → Output** |
| READMEs updated         | README.md, README_SETUP.md |
| No changes to existing VEX files | 01–07 unchanged |

### v2.3 → v2.4 Changes (bbox_collision only)

| v2.3                                     | v2.4                                            |
|:-----------------------------------------|:------------------------------------------------|
| Ran BEFORE signal_brake                  | Runs AFTER signal_brake                         |
| Cross-traffic still moving when checked  | Cross-traffic already stopped (speed=0)         |
| No signal awareness                      | Reads `signal_stop` — skips red-light vehicles  |
| False brakes at green intersections      | Only brakes for genuinely moving threats         |
| New param: —                             | New param: `stopped_speed_thresh` (0.5)         |

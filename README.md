# Traffic Sim V2 — Realistic Lane-Based Intersections

> **Houdini 21** | VEX-driven traffic simulation with lane rules, traffic lights, and smooth curve-following movement.

---

## Table of Contents

1. [Overview](#overview)
2. [Bug Fixes (v2.1)](#bug-fixes-v21)
3. [Node Network](#node-network)
4. [VEX Files Reference](#vex-files-reference)
5. [Setup Step-by-Step](#setup-step-by-step)
6. [Parameter Reference](#parameter-reference)
7. [Attribute Reference](#attribute-reference)
8. [Traffic Light System](#traffic-light-system)
9. [Tuning & Troubleshooting](#tuning--troubleshooting)

---

## Overview

A procedural traffic simulation built entirely in Houdini VEX. Vehicles follow lane-based route curves through a grid city, obey traffic lights, brake for vehicles ahead, and chain between road and intersection segments automatically.

### Core Features

- **Lane rules**: outer lane → straight/right, inner lane → straight/left
- **Curve-following movement**: vehicles slide along `primuv` samples on Input 1
- **Proximity-based collision avoidance**: `pcfind` search for vehicles ahead
- **Cross-traffic yielding**: deterministic priority at intersections
- **Probability-based route switching**: `straight_bias` controls straight vs turn ratio
- **Edge respawn**: vehicles reaching grid boundary respawn on inward-facing edge roads
- **Traffic lights**: 8-phase signal cycle with smooth physics-based braking
- **Visual feedback**: brake intensity drives green → yellow → red color per vehicle

### Segment Types

| Segment | Description |
|---|---|
| `road` | Straight stretches between intersections |
| `intersection_straight` | Through-lanes inside the intersection box |
| `right_turn` | Outer-lane right-turn arcs |
| `left_turn` | Inner-lane left-turn arcs |

### Lane Rules at Intersections

| Lane | Allowed Turns |
|---|---|
| `outer` | Straight, Right |
| `inner` | Straight, Left |

---

## Bug Fixes (v2.1)

This version fixes vehicles randomly stopping in the middle of intersections or on open roads with nothing in front of them.

**Only `03_solver_step.vex` was changed.** `signal_brake` and all other VEX files are unchanged.

### Root Causes Found & Fixed

| Fix | Bug | Cause | Change |
|---|---|---|---|
| **A** | Cars stop on open road with nothing ahead | **Phase 2 (cross-traffic) triggered on parallel same-direction traffic.** The direction filter `dot(my_dir, nb_dir) < -0.7` only rejected near-opposite vehicles. Two vehicles on parallel routes going the same way passed all checks. The yield logic then forced one to hard-stop. | Added `if(dot(my_dir, nb_dir) > 0.5) continue;` — skips vehicles going in roughly the same direction (within ~60° of your heading). Only true perpendicular cross-traffic remains. |
| **B** | Cars stop in the middle of intersections | **Phase 2 ran even when the vehicle was already inside an intersection segment.** A vehicle on `intersection_straight` or a turn arc would detect cross-traffic from the perpendicular road and yield — but it had already committed to crossing. Stopping mid-intersection caused gridlock. | Added `string my_seg_type = prim(1, "segment_type", my_route)` check. If the vehicle is on any intersection segment (`intersection_straight`, `right_turn`, `left_turn`), Phase 2 is skipped entirely. Traffic lights guarantee cross-traffic is held during your green. |
| **C** | Occasional braking behind vehicles in adjacent lanes | **Phase 1 (same-lane following) lateral filter was too wide.** `safe_dist * 0.6 = 6.0` units — with `lane_spacing = 3.5`, this caught vehicles nearly 2 lanes away as if they were in your lane. | Changed `safe_dist * 0.6` to `safe_dist * 0.4` (= 4.0 units). Only true same-lane vehicles trigger following brakes. |
| **D** | Inner/outer lane vehicles queue behind each other at red lights instead of stopping independently at stop line | **Phase 1 had no `lane_type` check.** It used only lateral distance to decide "same lane." An outer-lane car stopped at a red light was close enough laterally for the inner-lane car to treat it as a same-lane leader and queue behind it. | Added `if(point(0, "lane_type", nb) != my_lane) continue;` in Phase 1 — vehicles only follow others in the exact same lane type. Each lane now stops independently at its own stop line. |
| **E** | Vehicles stop on green light before it turns yellow | **Phase 2 detected perpendicular vehicles stopped at their own red light as cross-traffic threats.** The distance check (`dist < 15`) triggered, the yield logic forced the green-light vehicle to hard-stop. | Added `if(point(0, "signal_stop", nb) == 1) continue;` in Phase 2 — vehicles held by their own signal won't enter the intersection, so they're not threats. |
| **F** | Vehicles stop too aggressively when yellow just starts (close vehicles should pass through) | **Yellow dilemma zone used emergency deceleration** (`decel = 25`), giving a braking distance of only 4.5 units at speed 15. Only vehicles within 4.5 units were considered "committed." | Changed to comfortable deceleration (`decel * 0.5`) plus reaction-time distance (`speed * 0.3`). At speed 15 the committed zone is now ~13.5 units — vehicles close to the line proceed through yellow naturally. |

### What was NOT changed

- Phase 1 logic (same-lane following) — only the lateral threshold
- Phase 3 (speed adjustment) — untouched
- Phase 4 (advance) — untouched
- Phase 5 (lane-based route switching) — untouched
- Phase 6 (position update) — untouched
- `signal_brake` — untouched
- `init_vehicles` — untouched
- `gen_vehicle_routes` — untouched
- `color_vehicles` — untouched
- `gen_traffic_lights` / `gen_light_poles` — untouched

---

## Node Network

### Object-Level

```
/obj/
├── traffic_flow_curves     ← road geometry (display)
└── traffic_sim_v2          ← simulation (vehicles + lights)
```

### Inside `/obj/traffic_sim_v2`

```
gen_vehicle_routes (AttribWrangle)     ← 01_gen_vehicle_routes.vex
    │
    ▼
resample_routes (Resample)
    │
    ▼
route_curves (Null)                    ← reference point for Object Merges
    │
    ▼
init_vehicles (AttribWrangle)          ← 02_init_vehicles.vex
    │
    ▼
car_anim (Solver SOP) ──────────────────────────────────────────┐
│                                                                 │
│  ┌── prev_frame ──────────┐  ┌── route_curves_ref ──────────┐ │
│  │   (auto, Input 0)      │  │   Object Merge               │ │
│  │                        │  │   → /obj/traffic_sim_v2/      │ │
│  │                        │  │     route_curves              │ │
│  │                        ▼  ▼                               │ │
│  │            solver_step (AttribWrangle)                     │ │
│  │            03_solver_step.vex  ◄── FIXED in v2.1          │ │
│  │                        │                                  │ │
│  │                        ▼                                  │ │
│  │            signal_brake (AttribWrangle)                    │ │
│  │            07_signal_brake.vex                             │ │
│  │                        │                                  │ │
│  │                        ▼                                  │ │
│  │                    output ────────────────────────────►    │ │
│  └───────────────────────────────────────────────────────────┘ │
├────────────────────────────────────────────────────────────────┘
    │
    ▼
color_vehicles (AttribWrangle)         ← 05_color_vehicles.vex
    │
    ▼
copy_cars (Copy to Points)
    │
    ▼
merge_all (Merge)  ← also receives gen_traffic_lights + gen_light_poles
    │
    ▼
OUTPUT
```

---

## VEX Files Reference

| File | Node | Run Over | Purpose |
|---|---|---|---|
| `01_gen_vehicle_routes.vex` | `gen_vehicle_routes` | Detail | Procedurally builds the route curve network |
| `02_init_vehicles.vex` | `init_vehicles` | Detail | Scatters vehicle points on road segments |
| **`03_solver_step.vex`** | `solver_step` (inside `car_anim`) | Points | **Moves vehicles, brakes, route switching — FIXED in v2.1** |
| `05_color_vehicles.vex` | `color_vehicles` | Points | Colors vehicles by brake intensity |
| `07_signal_brake.vex` | `signal_brake` (inside `car_anim`) | Points | Brakes vehicles for red/yellow traffic lights |

---

## Setup Step-by-Step

### Applying the v2.1 Fix

1. Open your `traffic_sim_v2` geo node.
2. Dive into the **`car_anim`** Solver SOP.
3. Select the **`solver_step`** AttribWrangle.
4. Replace the VEX snippet with the contents of **`03_solver_step.vex`**.
5. That's it. No new nodes, no new parameters, no rewiring needed.

### Parameters on `solver_step` (unchanged)

These should already exist on your node. Defaults for reference:

| Parameter | Type | Default |
|---|---|---|
| `max_speed` | float | 15 |
| `acceleration` | float | 8 |
| `deceleration` | float | 25 |
| `look_ahead_dist` | float | 25 |
| `cross_detect_dist` | float | 30 |
| `min_safe_dist` | float | 10 |
| `cross_safe_time` | float | 1.5 |
| `search_count` | int | 150 |
| `vehicle_offset` | float | 0.5 |
| `route_match_dist` | float | 2.0 |
| `straight_bias` | float | 0.65 |
| `grid_size` | float | 348 |
| `entry_dist` | float | 20 |

### Parameters on `signal_brake` (unchanged)

| Parameter | Type | Default |
|---|---|---|
| `grid_size` | float | 348 |
| `grid_divisions` | int | 4 |
| `entry_dist` | float | 20 |
| `green_time` | float | 10.0 |
| `arrow_time` | float | 4.0 |
| `yellow_time` | float | 3.0 |
| `clearance_time` | float | 1.0 |
| `signal_detect_dist` | float | 45.0 |
| `signal_deceleration` | float | 25.0 |
| `stop_margin` | float | 1.5 |

---

## Attribute Reference

### Vehicle Point Attributes

| Attribute | Type | Set By | Description |
|---|---|---|---|
| `speed` | float | solver_step / signal_brake | Current speed |
| `target_speed` | float | solver_step / signal_brake | Desired speed this frame |
| `u_param` | float | solver_step | Normalized position along current curve (0–1) |
| `route_id` | int | solver_step | Primitive index of current route curve (on Input 1) |
| `prim_length` | float | solver_step | Arc length of the current route curve |
| `vehicle_id` | int | init_vehicles | Unique vehicle identifier |
| `lane_type` | string | solver_step | `"inner"` or `"outer"` |
| `brake` | float | solver_step / signal_brake | 0 = cruising, 1 = full braking (drives color) |
| `signal_stop` | int | signal_brake | 1 = currently stopped for a traffic signal |
| `signal_state` | string | signal_brake | `"green"`, `"left_arrow"`, `"yellow"`, `"red"`, `"none"` |
| `N` | vector | solver_step | Forward direction (curve tangent) |
| `up` | vector | init_vehicles | Up vector `{0,1,0}` |
| `curve_type` | string | init_vehicles | `"vehicle"` |

---

## Traffic Light System

### Signal Phase Cycle (8 phases)

```
Phase 0 ──► NS Green         (0s  – 10s)
Phase 1 ──► NS Left Arrow    (10s – 14s)
Phase 2 ──► NS Yellow        (14s – 17s)
Phase 3 ──► All Red (clear)  (17s – 18s)
Phase 4 ──► EW Green         (18s – 28s)
Phase 5 ──► EW Left Arrow    (28s – 32s)
Phase 6 ──► EW Yellow        (32s – 35s)
Phase 7 ──► All Red (clear)  (35s – 36s)
                              └─ cycle repeats
```

### How signal_brake Works

1. Determines vehicle's primary travel axis (X or Z).
2. Finds the next intersection ahead using grid math.
3. Computes the signal phase from `@Time` using the same formula as `gen_traffic_lights`.
4. Decides must-stop based on phase + lane type:
   - **Green** → go
   - **Left arrow** → inner lane goes, outer stops
   - **Yellow** → dilemma zone: stop only if braking distance < distance to stop line
   - **Red / clearance** → stop
5. Applies physics-based braking: `a = v² / (2d)`, clamped to `signal_deceleration`.

---

## Tuning & Troubleshooting

### If cars still stop on road (after v2.1 fix)

| Symptom | Likely Cause | Fix |
|---|---|---|
| Cars slow behind adjacent-lane traffic | `min_safe_dist * 0.4` still too wide for your lane spacing | Lower `min_safe_dist` (try 8) or adjust lane_spacing |
| Cars stop far from intersections | `signal_detect_dist` too high | Lower to 30–35 |
| Cars run red lights | `signal_detect_dist` too low | Raise to 50+ |
| All cars at one intersection stop | Traffic light timing mismatch | Ensure `green_time`, `arrow_time`, `yellow_time`, `clearance_time` match between `signal_brake` and `gen_traffic_lights` |

### If cars still stop at intersections (after v2.1 fix)

| Symptom | Likely Cause | Fix |
|---|---|---|
| Car stops right at entry to intersection | Signal brake is correctly holding it at the stop line | This is correct behaviour — check `signal_state` attribute |
| Car stops inside intersection with `signal_stop = 0` | Another issue in Phase 1 (following) | Check if a car ahead on the same intersection segment is stopped — the following logic still runs inside intersections |
| Gridlock (all cars in intersection stopped) | Timing issue — clearance_time too short | Increase `clearance_time` to 2.0 |

### Performance tips

- `search_count` is the biggest performance knob — lower it if your scene is slow (try 80)
- `cross_detect_dist` can be reduced to 20 for tighter intersections
- For large grids (6+ divisions), consider lowering `vehicle_density` in `init_vehicles`

---

## Grid Parameters

| Parameter | Value |
|---|---|
| `grid_size` | 348 |
| `grid_divisions` | 4 |
| `lane_spacing` | 3.5 |
| `num_lane_lines` | 5 (= 4 lanes) |
| `entry_dist` | 20 |

---

## Quick Diff: What Changed in v2.1

Only **`03_solver_step.vex`** and **`04_signal_brake.vex`** were modified. Six surgical changes total:

### Change A — Phase 1, line with lateral filter
```
BEFORE:  if(lat > safe_dist * 0.6) continue;
AFTER:   if(lat > safe_dist * 0.4) continue;
```

### Change B — Phase 2, new block wrapping the entire cross-traffic section
```vex
// NEW: added before the cross-traffic foreach loop
string my_seg_type = prim(1, "segment_type", my_route);
int in_intersection = (my_seg_type == "intersection_straight" ||
                       my_seg_type == "right_turn" ||
                       my_seg_type == "left_turn");
if(!in_intersection) {
    // ... entire Phase 2 cross-traffic logic ...
}
```

### Change C — Phase 2, new line inside the foreach loop
```vex
// NEW: added right after the existing  if(dot(my_dir, nb_dir) < -0.7) continue;
if(dot(my_dir, nb_dir) > 0.5) continue;   // skip same-direction parallel traffic
```

### Change D — Phase 1, new line inside the foreach loop
```vex
// NEW: added right after the vehicle_id check
if(point(0, "lane_type", nb) != my_lane) continue;   // only follow same-lane vehicles
```

### Change E — Phase 2, new line inside the foreach loop (solver_step)
```vex
// NEW: added right after the route_id check
if(point(0, "signal_stop", nb) == 1) continue;   // skip signal-held vehicles
```

### Change F — Yellow dilemma zone (signal_brake)
```vex
BEFORE:
    float braking_dist = (speed > 0.1)
                       ? (speed * speed) / (2.0 * decel)
                       : 0;

AFTER:
    float comfort_decel = decel * 0.5;
    float react_dist = speed * 0.3;
    float braking_dist = (speed > 0.1)
                       ? react_dist + (speed * speed) / (2.0 * comfort_decel)
                       : 0;
```

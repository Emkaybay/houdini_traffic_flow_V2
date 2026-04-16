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
- **Curve-following movement**: vehicles slide along `primuv` samples (no straight-line drift)
- **Proximity-based collision avoidance**: spatial `nearpoints()` search catches vehicles from any route
- **Segment chaining**: at end-of-curve, vehicles pick the best connecting segment
- **Edge respawn**: if no connecting segment exists, vehicle respawns on a random road
- **Traffic lights**: 8-phase signal cycle with smooth quadratic braking ramps
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

This version fixes vehicles randomly stopping in the middle of intersections or roads.

### Root Causes Found & Fixed

| # | Bug | Cause | Fix |
|---|---|---|---|
| 1 | **Cars freeze mid-road** | `primuv(0, …)` sampled the vehicle-point cloud (Input 0) instead of route curves (Input 1). Since `init_vehicles` removes all primitives, `primuv` returned `{0,0,0}` → `@N` became a zero vector → displacement was zero. | Changed every `primuv(0, …)` to `primuv(1, …)` to sample the route curves on Input 1. |
| 2 | **Cars drift off-road at turns** | Position was updated by straight-line displacement (`@P += vel * speed * dt`) instead of following the curve. | Position is now computed by advancing `u_param` first, then sampling the new position from the curve with `primuv(1, "P", rid, ...)`. |
| 3 | **Cars stop at intersections with nobody ahead** | Forward-vehicle detection only checked vehicles on the same `route_id` + `lane_type`. At intersections, vehicles from different routes converge and were invisible to each other. | Replaced route-based loop with `nearpoints()` spatial search + forward-cone + lateral-width filter. Any vehicle physically ahead is detected regardless of route. |
| 4 | **Phantom hard-stops on open road** | Braking used `lerp()` between `brake_dist=5` and `lane_change_brake_dist=3` — when distance < 3, the interpolation factor went negative and `max(0,…)` clamped speed to zero instantly. | Replaced with quadratic ease: `desired = tgt * t * t` where `t = (dist - stop_dist) / (follow_dist - stop_dist)`. Wider range (follow=12, stop=3) and smooth ramp eliminate snap-to-zero. |
| 5 | **Traffic light matching failed on non-axis-aligned roads** | Hard-coded `abs(vel.z) > 0.5` didn't work for diagonal directions. Also looped only over `primpoints(1,0)` — just the first prim. | Now loops over ALL light points with a pure dot-product approach test against each light's `approach_dir` vector. Works for any road orientation. |
| 6 | **Zero tangent on first frame** | If `@N` was uninitialized or zeroed out, the vehicle had no forward direction and could never move. | Added a safety rebuild: if `length(@N) < 0.001`, recompute tangent from the curve via `primuv`. |

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
init_vehicles (AttribWrangle)          ← 02_init_vehicles.vex
    │
    ▼
car_anim (Solver SOP)  ─────────────────────────────────────┐
│                                                             │
│  ┌─ prev_frame ──────────┐   ┌─ route_curves_ref ────┐    │
│  │  (Input 0)            │   │  Object Merge          │    │
│  │                       ▼   │  → /obj/traffic_sim_v2 │    │
│  │               solver_step (AttribWrangle)           │    │
│  │               03_solver_step.vex                    │    │
│  │                       │                             │    │
│  │                       ▼                             │    │
│  │               signal_brake (AttribWrangle)          │    │
│  │               04_signal_brake.vex                   │    │
│  │               Input 1 ← traffic_lights_ref          │    │
│  │                       │        (Object Merge        │    │
│  │                       │   → gen_traffic_lights)     │    │
│  │                       ▼                             │    │
│  │               output ──────────────────────────────►│    │
│  └─────────────────────────────────────────────────────┘    │
├──────────────────────────────────────────────────────────────┘
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
| **`03_solver_step.vex`** | `solver_step` (inside `car_anim`) | Points | **Moves vehicles along curves, brakes for traffic** |
| **`04_signal_brake.vex`** | `signal_brake` (inside `car_anim`) | Points | **Brakes vehicles for red/yellow traffic lights** |
| `05_color_vehicles.vex` | `color_vehicles` | Points | Colors vehicles by brake intensity |

---

## Setup Step-by-Step

### A. Route Generation (no changes needed)

1. `gen_vehicle_routes` AttribWrangle runs over Detail, Input 0 = none.
2. Output feeds into a **Resample** SOP (`resample_routes`, length ≈ 2).
3. A **Null** named `route_curves` sits after resample for easy referencing.

### B. Vehicle Initialization (no changes needed)

1. `init_vehicles` AttribWrangle runs over Detail, Input 0 = `route_curves`.
2. Paste `02_init_vehicles.vex` into the snippet.
3. Set spare parms:
   - `vehicle_speed` = 15
   - `vehicle_spacing` = 25
   - `vehicle_density` = 0.3
   - `vehicle_offset` = 0.5
   - `random_seed` = 42

### C. Solver Setup (UPDATED)

1. Rename your Solver SOP to **`car_anim`**.
2. Wire `init_vehicles` output into the Solver's **Input 1**.
3. **Inside the Solver**:

   a. Create an **Object Merge** named `route_curves_ref`:
      - Object path: `/obj/traffic_sim_v2/route_curves`
      - Transform: Into This Object

   b. Create an **AttribWrangle** named `solver_step`:
      - Run Over: **Points**
      - Input 0: `prev_frame`
      - Input 1: `route_curves_ref`
      - Paste **`03_solver_step.vex`** into the snippet

   c. **Add spare parameters on `solver_step`** (right-click → Edit Parameter Interface):

      | Parameter | Type | Default | Description |
      |---|---|---|---|
      | `max_speed` | Float | 15 | Maximum cruising speed |
      | `accel_rate` | Float | 8 | Acceleration (units/s²) |
      | `decel_rate` | Float | 20 | Deceleration for hard braking |
      | `follow_dist` | Float | 12 | Distance to begin slowing for car ahead |
      | `stop_dist` | Float | 3 | Full-stop buffer distance |
      | `search_radius` | Float | 35 | Spatial search radius for neighbours |
      | `vehicle_offset` | Float | 0.5 | Y offset above road surface |
      | `straight_bias` | Float | 0.5 | Preference for straight segments at junctions |

   d. Create another **Object Merge** named `traffic_lights_ref`:
      - Object path: `/obj/traffic_sim_v2/gen_traffic_lights`
      - Transform: Into This Object

   e. Create an **AttribWrangle** named `signal_brake`:
      - Run Over: **Points**
      - Input 0: output of `solver_step`
      - Input 1: `traffic_lights_ref`
      - Paste **`04_signal_brake.vex`** into the snippet

   f. **Add spare parameters on `signal_brake`**:

      | Parameter | Type | Default | Description |
      |---|---|---|---|
      | `signal_search_radius` | Float | 40 | Max distance to detect a traffic light |
      | `signal_stop_dist` | Float | 2 | Hard-stop distance from the light |
      | `signal_slow_dist` | Float | 15 | Begin slowing at this distance |
      | `signal_approach_dot` | Float | 0.5 | Min dot product to count as "approaching" |

   g. Wire `signal_brake` output to the Solver's **output**.

### D. Color & Display (no changes needed)

1. After the Solver, add `color_vehicles` AttribWrangle (run over Points).
2. Paste `05_color_vehicles.vex`.
3. Feed into a **Copy to Points** with your car geometry.

---

## Parameter Reference

### solver_step Parameters

| Parameter | Default | Effect |
|---|---|---|
| `max_speed` | 15 | Global speed cap |
| `accel_rate` | 8 | How quickly cars reach cruising speed |
| `decel_rate` | 20 | How quickly cars can emergency-brake |
| `follow_dist` | 12 | Start slowing when a car is this close ahead |
| `stop_dist` | 3 | Full stop buffer behind the car ahead |
| `search_radius` | 35 | How far to scan for neighbouring vehicles |
| `vehicle_offset` | 0.5 | Height above road surface |
| `straight_bias` | 0.5 | Higher = prefer going straight at junctions |

### signal_brake Parameters

| Parameter | Default | Effect |
|---|---|---|
| `signal_search_radius` | 40 | Max distance to scan for traffic lights |
| `signal_stop_dist` | 2 | Distance from light where car fully stops |
| `signal_slow_dist` | 15 | Distance from light where braking begins |
| `signal_approach_dot` | 0.5 | Direction threshold (0.5 ≈ 60-degree cone) |

---

## Attribute Reference

### Vehicle Point Attributes

| Attribute | Type | Set By | Description |
|---|---|---|---|
| `speed` | float | solver_step | Current speed |
| `target_speed` | float | init_vehicles | Desired cruising speed |
| `u_param` | float | solver_step | Normalized position along current curve (0–1) |
| `route_id` | int | solver_step | Primitive index of the current route curve |
| `prim_length` | float | solver_step | Arc length of the current route curve |
| `vehicle_id` | int | init_vehicles | Unique vehicle identifier |
| `lane_type` | string | init_vehicles | `"inner"` or `"outer"` |
| `segment_type` | string | solver_step | Current segment: `road`, `intersection_straight`, `right_turn`, `left_turn` |
| `brake` | float | solver_step / signal_brake | 0 = cruising, 1 = full braking (drives color) |
| `signal_stop` | int | signal_brake | 1 = currently stopped for a traffic signal |
| `signal_state` | string | signal_brake | `"red"`, `"yellow"`, `"green"`, or `"none"` |
| `N` | vector | solver_step | Forward direction (tangent to curve) |
| `up` | vector | solver_step | Up vector `{0,1,0}` |
| `curve_type` | string | init_vehicles | Always `"vehicle"` |

### Traffic Light Point Attributes

| Attribute | Type | Description |
|---|---|---|
| `light_state` | string | `"red"`, `"yellow"`, `"green"`, `"arrow"` |
| `approach_dir` | int | 0=East, 1=West, 2=North, 3=South |
| `is_ns` | int | 1 if this light controls North-South traffic |
| `phase` | float | Current phase position in the signal cycle |
| `Cd` | vector | Display color for the light bulb |

---

## Traffic Light System

### Signal Phase Cycle (8 phases)

```
Phase 0 ──► NS Green         (0.00 – 0.30)
Phase 1 ──► NS Arrow         (0.30 – 0.40)
Phase 2 ──► NS Yellow        (0.40 – 0.45)
Phase 3 ──► All Red (clear)  (0.45 – 0.50)
Phase 4 ──► EW Green         (0.50 – 0.80)
Phase 5 ──► EW Arrow         (0.80 – 0.90)
Phase 6 ──► EW Yellow        (0.90 – 0.95)
Phase 7 ──► All Red (clear)  (0.95 – 1.00)
```

### How Vehicles Respond

1. `signal_brake` scans all traffic light points within `signal_search_radius`.
2. For each light, it checks:
   - Is the light ahead of the vehicle? (dot product > `signal_approach_dot`)
   - Does the vehicle's travel direction match the light's `approach_dir`?
3. If a matching red/yellow light is found, a **quadratic braking ramp** is applied:
   - At `signal_slow_dist`: begin gentle deceleration
   - At `signal_stop_dist`: full stop
   - Ramp: `desired = target_speed * t²` where `t = (dist - stop) / (slow - stop)`
4. When the light turns green, `signal_stop` clears and the vehicle accelerates normally.

---

## Tuning & Troubleshooting

### Cars still stopping randomly?

| Symptom | Check | Fix |
|---|---|---|
| Cars freeze and never move | `@N` is `{0,0,0}` on the vehicle points | Ensure `route_curves_ref` Object Merge path is correct and points to the Null after Resample |
| Cars teleport or jump | `prim_length` is very small or zero | Check that Resample isn't producing degenerate tiny prims |
| Cars pile up at intersections | `follow_dist` too large or `stop_dist` too large | Lower `follow_dist` to 8–10, `stop_dist` to 2 |
| Cars ignore vehicles in adjacent lanes | `search_radius` too small | Increase to 40–50 |
| Cars brake for vehicles in other lanes | Lateral filter too wide | In `solver_step.vex`, reduce the lateral threshold (currently 4.0) |
| Cars don't chain through intersections | No connecting segment found within 3 units | Check that intersection segments connect cleanly to road segments (gap < 3 units) |
| Cars respawn too often | Grid gaps between segments | In `gen_vehicle_routes`, check `entry_dist` and `lane_spacing` produce connected geometry |

### Traffic lights not working?

| Symptom | Check | Fix |
|---|---|---|
| Cars ignore red lights | `signal_brake` Input 1 not wired | Create Object Merge → `gen_traffic_lights` and wire to Input 1 of `signal_brake` |
| Cars brake at green lights | `light_state` attribute missing or wrong | Verify `gen_traffic_lights` sets `light_state` string attribute |
| Cars brake too early | `signal_slow_dist` too high | Lower to 10–12 |
| Cars run red lights | `signal_approach_dot` too high | Lower to 0.3–0.4 to widen detection cone |

### Performance tips

- `search_radius` is the biggest performance knob — keep it as low as possible while still detecting ahead vehicles
- For very large grids (8+ divisions), consider lowering `vehicle_density` in `init_vehicles`
- The `nearpoints()` call in `solver_step` is O(n log n) per vehicle — much faster than the old O(n²) brute-force loop

---

## Quick Reference: What Changed in v2.1

**Files modified:**
- `03_solver_step.vex` — complete rewrite
- `04_signal_brake.vex` — complete rewrite

**Files unchanged:**
- `01_gen_vehicle_routes.vex`
- `02_init_vehicles.vex`
- `05_color_vehicles.vex`

**Node changes:**
- Solver SOP renamed to `car_anim` (optional, cosmetic)
- No new nodes required — same wiring, just updated VEX snippets

---

## Grid Parameters (Reference)

| Parameter | Value |
|---|---|
| `grid_size` | 348 |
| `grid_divisions` | 4 |
| `lane_spacing` | 3.5 |
| `num_lane_lines` | 5 (= 4 lanes) |
| `entry_dist` | (from gen_vehicle_routes) |
| `arc_segments` | (from gen_vehicle_routes) |

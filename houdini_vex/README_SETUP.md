# Traffic Sim V2 — Solver-Based with Route Switching

## Overview

Solver-based traffic simulation where vehicles have persistent state (speed, position).
Vehicles follow route curves, brake for other vehicles, and **switch routes at endpoints
to turn at intersections**. Designed to transition into an RBD car rig later.

---

## Node Network

```
gen_vehicle_routes (Detail Wrangle)
        |
resample_routes (Resample SOP, length=2)
        |
route_curves (Null)
        |
init_vehicles (Detail Wrangle)
        |
traffic_solver (Solver SOP)
  |  INSIDE:
  |    prev_frame -----------> solver_step (input 0)
  |    Object Merge ---------> solver_step (input 1)
  |    (path: /obj/traffic_sim_v2/route_curves)
  |    solver_step ----------> Output
        |
color_vehicles (Point Wrangle)
        |
copy_cars (Copy to Points) <--- car_box (Box: 4 x 1.5 x 2)
        |
      DISPLAY
```

---

## VEX Files

| File | Node | Run Over | What it does |
|------|------|----------|-------------|
| `01_gen_vehicle_routes.vex` | gen_vehicle_routes | Detail | Creates straight + turning route polylines |
| `02_init_vehicles.vex` | init_vehicles | Detail | Scatters vehicle points on routes randomly |
| `03_solver_step.vex` | solver_step (inside Solver) | Points | Detect → Brake → Advance → Route Switch |
| `04_color_vehicles.vex` | color_vehicles | Points | Green/Yellow/Red based on brake |

---

## Setup Step-by-Step

### 1. Create Geo Node
- `/obj/` level → Create Geometry: `traffic_sim_v2`
- Dive inside, delete any default nodes

### 2. gen_vehicle_routes
- Drop **Attrib Wrangle**, name: `gen_vehicle_routes`
- Run Over: **Detail**
- Paste `01_gen_vehicle_routes.vex`
- Create Spare Parameters, set:
  - grid_size = `348`
  - grid_divisions = `4`
  - lane_spacing = `3.5`
  - entry_dist = `20`
  - arc_segments = `16`

### 3. resample_routes
- Drop **Resample** SOP after gen_vehicle_routes
- Set Length to `2`

### 4. route_curves
- Drop **Null** SOP after resample_routes, name: `route_curves`

### 5. init_vehicles
- Drop **Attrib Wrangle**, name: `init_vehicles`
- Wire `route_curves` → input 0
- Run Over: **Detail**
- Paste `02_init_vehicles.vex`
- Create Spare Parameters, set:
  - vehicle_speed = `15`
  - vehicle_spacing = `30`
  - straight_density = `0.35`
  - turn_density = `0.0`
  - vehicle_offset = `0.5`
  - random_seed = `42`

### 6. Solver (IMPORTANT)

- Drop **Solver** SOP, name: `traffic_solver`
- Wire `init_vehicles` → Solver input 1
- **Dive inside** the Solver

#### Inside the Solver:

a) You see `prev_frame` (previous frame's vehicle points)

b) Drop **Object Merge** node, name: `route_curves_ref`
   - Set Object 1 path to the **full path** of your route_curves null
   - Example: `/obj/traffic_sim_v2/route_curves`

c) Drop **Attrib Wrangle**, name: `solver_step`
   - Run Over: **Points**
   - Wire `prev_frame` → `solver_step` **input 0**
   - Wire `route_curves_ref` → `solver_step` **input 1**
   - Paste `03_solver_step.vex`
   - Create Spare Parameters, set:

     | Parameter | Value |
     |-----------|-------|
     | max_speed | 15 |
     | acceleration | 8 |
     | deceleration | 25 |
     | look_ahead_dist | 30 |
     | cross_detect_dist | 35 |
     | min_safe_dist | 10 |
     | cross_safe_time | 1.5 |
     | search_count | 150 |
     | vehicle_offset | 0.5 |
     | route_match_dist | 3.0 |

d) Wire `solver_step` → **Output** node

e) Go back up (out of the Solver)

### 7. color_vehicles
- Drop **Attrib Wrangle** after Solver, name: `color_vehicles`
- Run Over: **Points**
- Paste `04_color_vehicles.vex`

### 8. car_box + copy_cars
- Drop **Box** SOP, name: `car_box`
  - Size: `4, 1.5, 2`
  - Center Y: `0.75`
- Drop **Copy to Points** SOP, name: `copy_cars`
  - Wire `car_box` → input 1 (source)
  - Wire `color_vehicles` → input 2 (target points)
  - Enable **Use Implicit N**
- Set **display flag** on `copy_cars`

### 9. Run
- Go to **frame 1**
- Press **Play**

---

## How It Works

### Vehicle Movement
1. Solver reads previous frame's vehicle points
2. Each vehicle has persistent `speed` and `u_param` (position along route)
3. Speed adjusts toward `target_speed` via acceleration/deceleration
4. `u_param` advances by `speed * dt / prim_length`
5. Position is read from route curve via `primuv(1, "P", route_id, u)`

### Route Switching (How Vehicles Turn)
When `u_param >= 0.97` (near end of route):
1. Get the **endpoint** of the current route
2. Search all routes for one whose **start point** is within `route_match_dist`
3. Score candidates by direction compatibility (prefers forward, allows turns)
4. Pick the best-scoring route (with randomness for variety)
5. Switch: `route_id` updates, `u_param` resets to `0.02`

**Why route_match_dist = 12.0?**
Roads have separated lanes: +X traffic uses the +Z side, -X uses -Z side.
When a +X vehicle reaches the east edge, the nearest route starting there
is a -X route on the -Z side — up to 10.5 units away (outer_off * 2).
So `route_match_dist` must be > 10.5 to bridge the lane gap.
Value of 12.0 catches all same-road transitions but won't accidentally
match routes on adjacent roads (87 units apart).

### Anti-Collision
- **Same-lane following**: pcfind ahead, brake proportionally to gap
- **Cross-traffic**: detect crossing vehicles, yield based on route priority
- **Speed is persistent**: `speed = 0` means the vehicle genuinely stops
  (unlike the old @Time system where pullback couldn't actually stop movement)

---

## Parameters Quick Reference

### init_vehicles
| Param | Default | Effect |
|-------|---------|--------|
| vehicle_speed | 15 | Initial speed for all vehicles |
| vehicle_spacing | 30 | Minimum distance between vehicles on same route |
| straight_density | 0.35 | How many vehicles per straight route (0-1) |
| turn_density | 0.0 | How many vehicles per turn route (0 = none, use route switching instead) |
| vehicle_offset | 0.5 | Y height above road |
| random_seed | 42 | Change for different layouts |

### solver_step
| Param | Default | Effect |
|-------|---------|--------|
| max_speed | 15 | Cruising speed (units/sec) |
| acceleration | 8 | Speed up rate (units/sec²) |
| deceleration | 25 | Brake force (units/sec²) |
| look_ahead_dist | 30 | Same-lane detection range |
| cross_detect_dist | 35 | Intersection detection range |
| min_safe_dist | 10 | Gap to maintain from threats |
| cross_safe_time | 1.5 | Seconds of clearance at crossings |
| search_count | 150 | Max pcfind neighbors |
| vehicle_offset | 0.5 | Y height above road |
| route_match_dist | 12.0 | How close route start/end must be to connect (must bridge lane gap) |

---

## Tuning Guide

| Problem | Fix |
|---------|-----|
| Vehicles don't turn | Increase `route_match_dist` (default 12.0, try 15.0) |
| Too many collisions | Increase `min_safe_dist` to 14 |
| Vehicles stop too late | Increase `deceleration` to 40 |
| Vehicles stop too early | Decrease `cross_detect_dist` to 25 |
| Too few vehicles | Increase `straight_density` to 0.5 |
| Jerky stops | Decrease `deceleration` to 15 |
| Want different layout | Change `random_seed` in init_vehicles |
| Vehicles teleport at turns | Decrease `route_match_dist` to 8.0 |

**After changing init_vehicles params, always go back to frame 1!**
The solver resets from initial state at frame 1.

---

## Troubleshooting

### Vehicles stack at origin
- Object Merge path inside solver is wrong
- Check it resolves to the route_curves null
- Use full path: `/obj/traffic_sim_v2/route_curves`

### Vehicles don't move
- solver_step must have **2 inputs** connected:
  - Input 0 = prev_frame
  - Input 1 = Object Merge (route curves)
- Check `nprimitives(1)` is not 0 (add a printf to debug)

### All vehicles are red (brake=1)
- `num_routes == 0` — Object Merge not connected
- `my_route >= num_routes` — route_id doesn't match

### Vehicles don't switch routes
- `route_match_dist` too small (try 5.0)
- Routes don't connect: check route_curves, endpoints should overlap

### Solver node is locked
- Right-click solver → Allow Editing of Contents
- Or in Python: `solver.allowEditingOfContents()`

---

## RBD Transition Path

```
PHASE 1 (NOW):
  SOP Solver → vehicle points → Copy to Points (box)

PHASE 2 (LATER):
  SOP Solver → "target" points (brain: where vehicle WANTS to go)
                    |
  DOP Network → RBD car rig (body: where vehicle ACTUALLY goes)
                    |
  Constraints (Pin/Spring) connect RBD car to solver target point
  RBD handles: collisions, suspension, wheel spin, weight transfer
  Solver handles: navigation, lane logic, intersection stops, route switching
```

The solver vehicle points become **goal positions** for RBD vehicles.
The RBD car rig chases the goal via constraints.
Physics handles the rest.

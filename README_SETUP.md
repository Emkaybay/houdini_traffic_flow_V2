# Traffic Sim V2 — Realistic Lane-Based Intersections

## How It Works

Routes are **short segments** between intersections (not edge-to-edge).
At each intersection boundary, the vehicle picks a new segment based on **lane rules**:

```
OUTER LANE (right side of road):
  -> Can go STRAIGHT through
  -> Can turn RIGHT
  -> Cannot turn left

INNER LANE (left side of road):
  -> Can go STRAIGHT through
  -> Can turn LEFT
  -> Cannot turn right
```

This matches real-world traffic rules.

## Segment Types

```
       intersection
        boundary
           |
  ---------| |----------     "road" segments (between intersections)
           |X|
           |X|               "intersection_straight" (through intersection)
           |X|
  ---------| |----------
           |
       intersection
        boundary

  Right turn arc:  outer lane -> perpendicular outer lane
  Left turn arc:   inner lane -> perpendicular inner lane
```

---

## Node Network

```
gen_vehicle_routes (Detail) --> resample_routes --> route_curves (Null)
                                                        |
                                    +-------------------+
                                    |
init_vehicles (Detail) -------> traffic_solver (Solver SOP)
                                    |
                              Inside Solver:
                                prev_frame ---------> solver_step (input 0)
                                Object Merge -------> solver_step (input 1)
                                solver_step --------> signal_brake ------> Output
                                    |                     ^
                              color_vehicles --> copy_cars |  --> DISPLAY
                                                    ^     |
                                               car_box    |
                                                          |
  ===== TRAFFIC LIGHTS (visual) =====                     |
                                                          |
  gen_traffic_lights (Detail) ----+                       |
                                  |--> merge_lights ------+---> DISPLAY
  gen_light_poles    (Detail) ----+     (Merge SOP)
```

---

## Setup (Existing Nodes)

### 1. gen_vehicle_routes
- Wrangle, **Detail**, paste `01_gen_vehicle_routes.vex`
- Parameters: grid_size=348, grid_divisions=4, lane_spacing=3.5, entry_dist=20, arc_segments=16

### 2. resample_routes
- Resample SOP, length=2

### 3. route_curves
- Null SOP

### 4. init_vehicles
- Wrangle, **Detail**, paste `02_init_vehicles.vex`
- Wire route_curves -> input 0
- Parameters: vehicle_speed=15, vehicle_spacing=25, vehicle_density=0.3, vehicle_offset=0.5, random_seed=42

### 5. Solver
- Solver SOP, wire init_vehicles -> input
- Inside: prev_frame -> solver_step input 0
- Inside: Object Merge (path: /obj/traffic_sim_v2/route_curves) -> solver_step input 1
- solver_step: Wrangle, **Points**, paste `03_solver_step.vex`
- Parameters: max_speed=15, acceleration=8, deceleration=25, look_ahead_dist=25, cross_detect_dist=30, min_safe_dist=10, cross_safe_time=1.5, search_count=150, vehicle_offset=0.5, route_match_dist=2.0

### 6. color_vehicles
- Wrangle, **Points**, paste `04_color_vehicles.vex`

### 7. car_box + copy_cars
- Box: 4 x 1.5 x 2, center Y = 0.75
- Copy to Points: car_box -> input 1, color -> input 2

---

## Traffic Lights Setup (NEW)

### 8. gen_traffic_lights

1. Create an **AttribWrangle** node named `gen_traffic_lights`.
2. Set **Run Over** to **Detail**.
3. Paste the contents of `05_gen_traffic_lights.vex` into the **VEXpression** snippet.
4. Add these **Spare Parameters** on the wrangle node (click the gear icon -> Edit Parameter Interface):

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `grid_size` | Float | 348 | Must match gen_vehicle_routes |
| `grid_divisions` | Int | 4 | Must match gen_vehicle_routes |
| `lane_spacing` | Float | 3.5 | Must match gen_vehicle_routes |
| `entry_dist` | Float | 20 | Must match gen_vehicle_routes |
| `light_height` | Float | 6.0 | Height of the bottom bulb above ground |
| `pole_offset` | Float | 2.0 | Lateral gap between road edge and pole |
| `bulb_spacing` | Float | 1.8 | Vertical gap between stacked bulbs |
| `green_time` | Float | 10.0 | Green phase duration (seconds) |
| `arrow_time` | Float | 4.0 | Left-turn arrow duration (seconds) |
| `yellow_time` | Float | 3.0 | Yellow phase duration (seconds) |
| `clearance_time` | Float | 1.0 | All-red clearance interval (seconds) |
| `three_bulb` | Int | 1 | 1 = three stacked bulbs, 0 = single point |

> **Tip:** You can channel-reference `grid_size`, `grid_divisions`, `lane_spacing`, and `entry_dist` from `gen_vehicle_routes` so they stay in sync:
> ```
> ch("../gen_vehicle_routes/grid_size")
> ```

### 9. gen_light_poles (Optional)

1. Create an **AttribWrangle** node named `gen_light_poles`.
2. Set **Run Over** to **Detail**.
3. Paste `06_gen_traffic_light_poles.vex`.
4. Add spare parameters (same grid values as step 8), plus:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pole_color_r` | Float | 0.25 | Pole colour R |
| `pole_color_g` | Float | 0.25 | Pole colour G |
| `pole_color_b` | Float | 0.25 | Pole colour B |

5. **Merge** `gen_traffic_lights` and `gen_light_poles` with a **Merge SOP** (`merge_lights`).

### 10. Display

Option A — **Standalone display**: Set the display flag on `merge_lights` (or `gen_traffic_lights` alone) to see the lights in isolation.

Option B — **Combined with vehicles**: Create a **Merge SOP** that combines `copy_cars` and `merge_lights`, then set the display flag on that merge.

> **Important:** For the point markers to show colour in the viewport, enable:
> **Display Options (D key) -> Markers -> Point Markers** and set a visible size,
> or enable **Particles -> Display As: Discs/Spheres**.

---

## Solver Integration — Vehicles Obey Signals (NEW)

### 11. signal_brake (Inside the Solver)

This wrangle makes vehicles actually stop at red lights and proceed on green.

1. **Open** the Solver SOP (double-click `traffic_solver`).
2. Create an **AttribWrangle** named `signal_brake`.
3. Set **Run Over** to **Points**.
4. Paste the contents of `07_signal_brake.vex`.
5. **Wire it AFTER `solver_step`**:
   ```
   solver_step → signal_brake → Output
   ```
   (Disconnect solver_step from Output, insert signal_brake between them.)

6. Add these **Spare Parameters**:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `grid_size` | Float | 348 | Must match gen_vehicle_routes |
| `grid_divisions` | Int | 4 | Must match gen_vehicle_routes |
| `entry_dist` | Float | 20 | Must match gen_vehicle_routes |
| `green_time` | Float | 10.0 | Must match gen_traffic_lights |
| `arrow_time` | Float | 4.0 | Must match gen_traffic_lights |
| `yellow_time` | Float | 3.0 | Must match gen_traffic_lights |
| `clearance_time` | Float | 1.0 | Must match gen_traffic_lights |
| `signal_detect_dist` | Float | 45.0 | How far ahead vehicles "see" the signal |
| `signal_deceleration` | Float | 25.0 | Max braking force for signal stops |
| `stop_margin` | Float | 1.5 | Distance from stop line for hard stop |

> **Tip:** Channel-reference timing values from `gen_traffic_lights` to keep everything in sync:
> ```
> ch("../../../gen_traffic_lights/green_time")
> ```

### How It Works

Each frame, for every vehicle point:

1. **Detect travel axis** — is the vehicle going primarily along X (east-west) or Z (north-south)?
2. **Find next intersection** — using grid math, locate the next intersection center ahead.
3. **Calculate stop-line distance** — the stop line is `entry_dist` before the intersection center.
4. **Read the signal phase** — same `@Time`-based formula as `05_gen_traffic_lights.vex`.
5. **Decision logic**:
   - **Green** → proceed (all lanes)
   - **Left-turn arrow** → inner lane proceeds, outer lane stops
   - **Yellow** → dilemma zone check: if the vehicle can't physically stop before the line at its current speed, it proceeds ("committed"); otherwise it stops
   - **Red / clearance** → stop
6. **Apply braking** — smooth `v² / 2d` deceleration from far away; hard stop when very close to the line.

### New Vehicle Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `signal_stop` | int | 1 if the vehicle is currently being held by a red/yellow signal |
| `signal_state` | string | The signal this vehicle sees: "green", "left_arrow", "yellow", "red", or "none" |

These are useful for debugging and for custom colouring (e.g. showing signal-stopped vehicles in blue).

### What You Should See

- **Red phase**: Vehicles queue up behind the stop line, evenly braking to a halt.
- **Green phase**: Queued vehicles accelerate away from the line.
- **Left-arrow phase**: Inner-lane vehicles proceed; outer-lane vehicles remain stopped.
- **Yellow phase**: Vehicles far from the line stop; vehicles close to the line proceed through.
- **Colour feedback**: Signal-stopped vehicles show high `@brake` values (red in the existing `color_vehicles` wrangle).

---

## Signal Phase Cycle

The traffic lights cycle through 8 phases. One full cycle takes
`2 x (green_time + arrow_time + yellow_time + clearance_time)` seconds
(default = 36 s).

```
Time (s)   Phase   NS Approaches        EW Approaches
-------------------------------------------------------
 0 - 10    0       GREEN                RED
10 - 14    1       LEFT-TURN ARROW      RED
14 - 17    2       YELLOW               RED
17 - 18    3       RED (clearance)      RED (clearance)
18 - 28    4       RED                  GREEN
28 - 32    5       RED                  LEFT-TURN ARROW
32 - 35    6       RED                  YELLOW
35 - 36    7       RED (clearance)      RED (clearance)
```

### Direction Mapping

```
NS approaches = traffic arriving from +Z or -Z  (indices 2, 3)
EW approaches = traffic arriving from +X or -X  (indices 0, 1)
```

### Colour Key

| State | Colour (Cd) | Meaning |
|-------|-------------|---------|
| Green | (0.08, 0.95, 0.18) | Go — through and right turns |
| Left Arrow | (0.08, 0.92, 0.55) | Protected left turn only |
| Yellow | (1.0, 0.88, 0.08) | Caution — signal about to turn red |
| Red | (0.95, 0.08, 0.08) | Stop |
| Dim (inactive bulb) | (0.12, 0.12, 0.12) | Bulb is off (three-bulb mode) |

---

## Point Attributes on Traffic Lights

| Attribute | Type | Description |
|-----------|------|-------------|
| `Cd` | vector | Colour of this bulb / point |
| `Alpha` | float | 1.0 if active, 0.25 if dim |
| `N` | vector | Faces approaching traffic (= approach direction) |
| `up` | vector | Always (0, 1, 0) |
| `pscale` | float | 2.2 active / 1.2 dim (3-bulb) or 2.5 (single) |
| `light_state` | string | "green", "left_arrow", "yellow", or "red" |
| `bulb_name` | string | "green", "yellow", or "red" (3-bulb only) |
| `is_active` | int | 1 if this bulb is the lit one (3-bulb only) |
| `intersection_id` | int | Unique ID for the intersection (row * cols + col) |
| `approach_dir` | int | 0=+X, 1=-X, 2=+Z, 3=-Z |
| `is_ns` | int | 1 = north-south approach, 0 = east-west |
| `phase` | int | Current global phase (0-7) |
| `curve_type` | string | "traffic_light" |

---

## Traffic Light Counts

With default `grid_divisions = 4` (5 x 5 = 25 intersections):

| Location | Count | Approaches each | Signal heads |
|----------|-------|-----------------|--------------|
| Corners | 4 | 2 | 8 |
| Edges | 12 | 3 | 36 |
| Interior | 9 | 4 | 36 |
| **Total** | **25** | | **80** |

Three-bulb mode: 80 x 3 = **240 points** (very lightweight).

---

## Parameters (Existing)

### solver_step
| Param | Default | Effect |
|-------|---------|--------|
| max_speed | 15 | Cruising speed |
| acceleration | 8 | Speed up (units/sec^2) |
| deceleration | 25 | Brake force (units/sec^2) |
| look_ahead_dist | 25 | Following detection range |
| cross_detect_dist | 30 | Intersection detection range |
| min_safe_dist | 10 | Gap to maintain |
| cross_safe_time | 1.5 | Crossing clearance (seconds) |
| search_count | 150 | pcfind max neighbors |
| vehicle_offset | 0.5 | Y height |
| route_match_dist | 2.0 | Segment connection tolerance |
| straight_bias | 0.65 | **(NEW)** Probability of going straight at intersections. 0.65 = 65% straight / 35% turn. Tunable: 0 = always turn, 1 = always straight. |

---

## Tuning

### Vehicle Issues
| Problem | Fix |
|---------|-----|
| Vehicles don't switch segments | Increase route_match_dist to 3.0 |
| Never turn right/left | Check lane_type attribute exists on routes |
| **All vehicles turn, none go straight** | **Fixed in updated 03_solver_step.vex — uses probability-based selection. `straight_bias` (default 0.65) = 65% straight / 35% turn.** |
| Want more turns | Lower `straight_bias` to 0.3–0.4 |
| Want almost all straight | Raise `straight_bias` to 0.85–0.95 |
| **Edge vehicles bounce in a loop** | **Fixed — vehicles at grid edges with no next segment now respawn on a random interior road segment instead of looping.** |
| Too few vehicles | Increase vehicle_density to 0.5 |
| Collisions at intersections | Increase min_safe_dist to 14 |
| Vehicles stop and don't restart | Check solver Object Merge resolves |

### Traffic Light Issues
| Problem | Fix |
|---------|-----|
| Lights don't change colour | Make sure @Time is advancing (press Play) |
| Lights are invisible in viewport | Enable Point Markers or Disc display mode |
| Lights are inside the road | Increase `pole_offset` to 3-4 |
| Cycle is too fast / slow | Adjust `green_time`, `arrow_time`, `yellow_time` |
| Lights on edges look wrong | This is correct — edge intersections have fewer approaches |
| Poles don't appear | Check gen_light_poles is merged and display flag is set |
| All lights are red | You may be paused on a clearance frame — scrub forward |
| Arrow phase hard to see | Increase `arrow_time` or adjust `c_arrow` in the VEX |

### Signal Brake / Solver Integration Issues
| Problem | Fix |
|---------|-----|
| Vehicles ignore red lights | Check signal_brake is wired AFTER solver_step inside the solver |
| Vehicles stop in the middle of the road | Increase `signal_detect_dist` to 50-60 |
| Vehicles overshoot the stop line | Decrease `stop_margin` to 1.0 or increase `signal_deceleration` |
| Vehicles never go on green | Verify timing params match between signal_brake and gen_traffic_lights |
| Outer lane moves during arrow | Check `lane_type` attribute exists on vehicles (set by init_vehicles) |
| Jerky braking | Reduce `signal_deceleration` to 15-20 for softer stops |
| Vehicles don't accelerate after red | Existing solver_step restores target_speed — check it's wired before signal_brake |
| signal_stop attribute is always 0 | Verify the wrangle is actually executing (check cook count) |

**After changing gen_vehicle_routes or traffic light params -> go to frame 1!**

---

## File Reference

| File | Node | Run Over | Purpose |
|------|------|----------|---------|
| `01_gen_vehicle_routes.vex` | gen_vehicle_routes | Detail | Road network geometry |
| `02_init_vehicles.vex` | init_vehicles | Detail | Spawn vehicles on roads |
| `03_solver_step.vex` | solver_step | Points | Vehicle movement & AI **(updated — straight/turn fix, edge respawn)** |
| `04_color_vehicles.vex` | color_vehicles | Points | Brake-based vehicle colouring |
| `05_gen_traffic_lights.vex` | gen_traffic_lights | Detail | **Traffic signal point markers** |
| `06_gen_traffic_light_poles.vex` | gen_light_poles | Detail | **Signal pole geometry (optional)** |
| `07_signal_brake.vex` | signal_brake | Points | **Vehicles obey traffic signals (inside solver)** |

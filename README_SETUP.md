# Traffic Sim V2 — Setup Guide

## Realistic Lane-Based Intersections + Predictive Bounding Box Collision

---

## How It Works

**Route Structure:**  Short segments between intersections, not full road edges.

**Lane Rules at each intersection boundary:**

| Lane           | Straight | Right Turn | Left Turn |
|:---------------|:---------|:-----------|:----------|
| **Outer** (right) | YES      | YES        | NO        |
| **Inner** (left)  | YES      | NO         | YES       |

**Segment Types:** `road`, `intersection_straight`, `right_turn`, `left_turn`

---

## Network Structure

### Route Generation
- `gen_vehicle_routes` (Wrangle) → `resample_routes` (Resample) → `route_curves` (Null)

### Vehicle Initialization & Simulation
- `init_vehicles` (Wrangle)
- `traffic_solver` (Solver SOP):
  - Inside Solver:
    - `prev_frame`
    - `solver_step` (Wrangle)
    - **`bbox_collision` (Wrangle) — NEW**
    - `signal_brake` (Wrangle)
  - `color_vehicles` (Wrangle)
  - `copy_cars` (Copy to Points)
  - `car_box` geometry

### Bounding Box (NEW)
- **`bound_car` (Bound SOP)** → connected to `car_box2` output

### Traffic Lights
- `gen_traffic_lights` (Wrangle)
- `gen_light_poles` (Wrangle)
- `merge_lights` (Merge SOP)

### Display
- `DISPLAY` / `OUTPUT`

---

## Setup Instructions

### Existing Nodes Setup

#### 1. `gen_vehicle_routes` (Wrangle)
- **Run Over:** Detail
- Paste `01_gen_vehicle_routes.vex`
- **Parameters:**
  - `grid_size`: 348
  - `grid_divisions`: 4
  - `lane_spacing`: 3.5
  - `entry_dist`: 20
  - `arc_segments`: 16

#### 2. `resample_routes` (Resample SOP)
- **Length:** 2

#### 3. `route_curves` (Null SOP)
- No specific setup.

#### 4. `init_vehicles` (Wrangle)
- **Run Over:** Detail
- Paste `02_init_vehicles.vex`
- Wire `route_curves` into **Input 0**
- **Parameters:**
  - `vehicle_speed`: 15
  - `vehicle_spacing`: 25
  - `vehicle_density`: 0.3
  - `vehicle_offset`: 0.5
  - `random_seed`: 42

#### 5. `traffic_solver` (Solver SOP)
- Wire `init_vehicles` into **Input 0**
- **Inside the Solver:**
  - Wire `prev_frame` → `solver_step` Input 0
  - Wire Object Merge (pointing to `/obj/traffic_sim_v2/route_curves`) → `solver_step` Input 1
  - **`solver_step` (Wrangle):**
    - **Run Over:** Points
    - Paste `03_solver_step.vex`
    - **v2.7: Collision detection removed** — solver_step now only handles
      movement, acceleration, route switching, and position updates.
      All collision avoidance is in `bbox_collision`.
    - **v2.8: Brake-aware acceleration** — solver_step checks the previous
      frame's `brake` value. If braking was active (`brake > 0.3`), it
      won't accelerate — preventing creep into stopped vehicles.
    - **Parameters:**
      - `max_speed`: 15
      - `acceleration`: 8
      - `deceleration`: 25
      - `vehicle_offset`: 0.5
      - `route_match_dist`: 2.0
      - `straight_bias`: 0.65
      - `grid_size`: 348
      - `entry_dist`: 20

#### 6. `color_vehicles` (Wrangle)
- **Run Over:** Points
- Paste `04_color_vehicles.vex`

#### 7. Car Geometry & Copying
- **`car_box2` (Box SOP):**
  - Size: 4 x 1.5 x 2
  - Center Y: 0.75
  - Rotation Y: 90
- **`copy_cars` (Copy to Points):**
  - Wire `car_box2` (via edgedivide/edit) into Input 1
  - Wire `color_vehicles` into Input 2

---

### Predictive Bounding Box Collision Setup (NEW — v2.3)

This section adds **predictive** OBB collision avoidance. Instead of checking
overlap at current positions (which falsely brakes opposite-direction traffic),
it predicts where vehicles will be using their velocity and only brakes when
predicted future positions actually overlap.

#### 8. `bound_car` (Bound SOP) — Capture Car Dimensions

1. **Create** a `Bound` SOP at the same level as `car_box2` inside `/obj/traffic_sim_v2`.
2. **Connect** the output of `car_box2` into the input of `bound_car`.
3. Set **Bounding Type** to **Bounding Box**.
4. **Note the output dimensions** from the Bound SOP geometry details:

   | Bound Axis | Size | Half-Extent | Maps To           |
   |:-----------|:-----|:------------|:------------------|
   | X          | 2    | 1.0         | `bbox_half_width` (lateral) |
   | Y          | 1.5  | 0.75        | height (not used in 2D)     |
   | Z          | 4    | 2.0         | `bbox_half_length` (forward)|

   > When Copy to Points aligns the car geometry, the Z axis becomes the
   > vehicle's forward direction (aligned with `@N`).

   **If you change the car geometry** (different box, imported model, etc.),
   re-check the Bound SOP's output and update the parameters below.

   **Optional expression references** (instead of hard-coding):
   ```
   bbox_half_length:  bbox("/obj/traffic_sim_v2/bound_car", D_ZSIZE) / 2
   bbox_half_width:   bbox("/obj/traffic_sim_v2/bound_car", D_XSIZE) / 2
   ```

#### 9. `bbox_collision` (AttribWrangle) — Inside the Solver

1. **Open the Solver SOP** (double-click `traffic_solver`).
2. **Create** an AttribWrangle node named `bbox_collision`.
3. Set **Run Over** to **Points**.
4. Paste `08_bbox_collision.vex` into the VEXpression.
5. **Wire it AFTER `solver_step` and BEFORE `signal_brake`**:

   ```
   prev_frame → solver_step → bbox_collision → signal_brake → Output
   ```

   > **Why before signal_brake?** bbox_collision handles all vehicle-to-vehicle
   > spacing first (bounding-box-based, supports different vehicle sizes).
   > signal_brake then adds traffic light stops on top.  `signal_stop` from
   > the previous frame is still on the points, so intersection awareness
   > works correctly.

6. **Wire Input 1** to the **same Object Merge** that `solver_step` uses
   (pointing to `/obj/traffic_sim_v2/route_curves`).

   > **Why Input 1?** The left-turn yield rule needs to read `segment_type`
   > from the route curve primitives (via `route_id`) to know if a vehicle
   > is going straight or turning left.

7. **Add Spare Parameters** (Gear icon → Edit Parameter Interface):

   | Parameter             | Type  | Default | Description                                                    |
   |:----------------------|:------|:--------|:---------------------------------------------------------------|
   | `bbox_half_length`    | Float | 2.0     | Half car length along forward (Z from Bound SOP)               |
   | `bbox_half_width`     | Float | 1.0     | Half car width lateral (X from Bound SOP)                      |
   | `bbox_padding`        | Float | 1.0     | Extra safety margin around each bbox                           |
   | `search_radius`       | Float | 35.0    | pcfind neighbour search radius                                 |
   | `max_neighbors`       | Int   | 50      | Maximum neighbours to evaluate per vehicle                     |
   | `brake_force`         | Float | 25.0    | Deceleration for collision avoidance (units/s^2)               |
   | `look_ahead_time`     | Float | 2.0     | How far into the future to predict (seconds)                   |
   | `emergency_gap`       | Float | 0.5     | Gap threshold for immediate emergency braking                  |
   | `stopped_speed_thresh`| Float | 0.5     | Below this speed, cross-lane vehicles are considered stopped   |
   | `left_turn_yield_dist`| Float | 25.0    | How far a straight vehicle detects a left-turner to yield      |
   | `follow_gap`          | Float | 6.0     | Minimum bumper-to-bumper distance for same-lane following      |
   | `intersection_radius` | Float | 25.0    | Distance from intersection to enable cross-lane detection      |
   | `grid_size`           | Float | 348     | Match `gen_vehicle_routes`                                     |
   | `grid_divisions`      | Int   | 4       | Match `gen_vehicle_routes`                                     |

8. **Channel reference** `grid_size` and `grid_divisions` from `gen_vehicle_routes`:
   ```
   grid_size:      ch("../../gen_vehicle_routes/grid_size")
   grid_divisions: ch("../../gen_vehicle_routes/grid_divisions")
   ```
   (Path may vary depending on your network depth — adjust the `../` count.)

---

### Traffic Lights Setup

#### 10. `gen_traffic_lights` (AttribWrangle)
- **Run Over:** Detail
- Paste `05_gen_traffic_lights.vex`
- **Spare Parameters:**
  - `grid_size` (Float, Default: 348)
  - `grid_divisions` (Int, Default: 4)
  - `lane_spacing` (Float, Default: 3.5)
  - `entry_dist` (Float, Default: 20)
  - `light_height` (Float, Default: 6.0)
  - `pole_offset` (Float, Default: 2.0)
  - `bulb_spacing` (Float, Default: 1.8)
  - `green_time` (Float, Default: 10.0)
  - `arrow_time` (Float, Default: 4.0)
  - `yellow_time` (Float, Default: 3.0)
  - `clearance_time` (Float, Default: 1.0)
  - `three_bulb` (Int, Default: 1)

#### 11. `gen_light_poles` (AttribWrangle) (Optional)
- **Run Over:** Detail
- Paste `06_gen_traffic_light_poles.vex`
- Same grid params as step 10, plus:
  - `pole_color_r` (Float, 0.25)
  - `pole_color_g` (Float, 0.25)
  - `pole_color_b` (Float, 0.25)
- **`merge_lights` (Merge):** Merge `gen_traffic_lights` + `gen_light_poles`

#### 12. Solver Integration — Vehicles Obey Signals

- **`signal_brake`** (inside Solver, AFTER `bbox_collision`, BEFORE `Output`):
  - **Run Over:** Points
  - Paste `07_signal_brake.vex`
  - Wire AFTER `bbox_collision` and BEFORE `Output`
  - **Spare Parameters:**
    - `grid_size`, `grid_divisions`, `entry_dist` (match route gen)
    - `green_time`, `arrow_time`, `yellow_time`, `clearance_time` (match traffic lights)
    - `signal_detect_dist`: 45.0
    - `signal_deceleration`: 25.0
    - `stop_margin`: 1.5

---

### Display

- **Option A:** Display flag on `merge_lights` (or `gen_traffic_lights`).
- **Option B:** Merge `copy_cars` + `merge_lights` → set display flag.
- **Viewport:** Enable **Display Options (D) → Markers → Point Markers** or **Particles → Display As: Discs/Spheres**.

---

## Solver Node Order (Complete)

Inside `traffic_solver`:

```
prev_frame
    │
    ▼
solver_step          ← movement + route switching ONLY (no collision)
    │                   Input 1: Object Merge → route_curves
    ▼
bbox_collision       ← ALL vehicle-to-vehicle spacing + collision
    │                   + left-turn yield + position correction
    │                   Input 1: Object Merge → route_curves (same)
    ▼
signal_brake         ← traffic light obedience (stops at red)
    │
    ▼
Output
```

> **v2.8 architecture:** bbox_collision runs BEFORE signal_brake.
> It handles all vehicle-to-vehicle interactions using bounding boxes
> (supports future multi-size vehicles).  signal_brake then adds
> traffic light stops independently.  `signal_stop` from the previous
> frame is read for intersection cross-traffic awareness.

---

## File Reference

| File                               | Node               | Run Over | Purpose                                      |
|:-----------------------------------|:-------------------|:---------|:---------------------------------------------|
| `01_gen_vehicle_routes.vex`        | gen_vehicle_routes | Detail   | Road network geometry                        |
| `02_init_vehicles.vex`             | init_vehicles      | Detail   | Spawn vehicles on roads                      |
| `03_solver_step.vex`               | solver_step        | Points   | Vehicle movement & AI                        |
| `04_color_vehicles.vex`            | color_vehicles     | Points   | Brake-based vehicle colouring                |
| `05_gen_traffic_lights.vex`        | gen_traffic_lights | Detail   | Traffic signal point markers                 |
| `06_gen_traffic_light_poles.vex`   | gen_light_poles    | Detail   | Signal pole geometry (optional)              |
| `07_signal_brake.vex`              | signal_brake       | Points   | Vehicles obey traffic signals (solver)       |
| **`08_bbox_collision.vex`**        | **bbox_collision** | **Points** | **OBB collision avoidance (solver) — NEW** |

---

## Tuning Guide

### Bounding Box Collision + Left-Turn Yield (v2.6)

| Issue                                 | Fix                                                        |
|:--------------------------------------|:-----------------------------------------------------------|
| Vehicles still overlapping            | Increase `bbox_padding` (try 1.5–2.0)                     |
| Vehicles braking too early / too far  | Decrease `look_ahead_time` (try 1.0–1.5)                  |
| Opposite-direction false brakes       | Handled by lateral clearance + TCA divergence filter       |
| Cross-lane false brakes on straights  | Decrease `intersection_radius` (try 15–20)                 |
| Missing cross-lane collisions         | Increase `intersection_radius` (try 30–35)                 |
| Performance (many vehicles)           | Reduce `max_neighbors` (30) or `search_radius` (20)       |
| Vehicles stuck at intersections       | Reduce `bbox_padding`; ensure `signal_brake` timing is correct |
| Jerky braking                         | Reduce `brake_force` (try 15–18)                           |
| Adding a new vehicle model            | Re-check Bound SOP, update `bbox_half_length/width`       |
| Straight not yielding to left-turner  | Check `segment_type` and `approach_dir` attribs on vehicles |
| Straight not yielding to right-turner | Same — verify `segment_type` = `"right_turn"` on that prim |
| Yielding too early / from too far     | Decrease `left_turn_yield_dist` (try 15–20)                |
| Not yielding soon enough              | Increase `left_turn_yield_dist` (try 30–35)                |
| Left-turner slowing for straight      | Check approach_dir — they should be conflicting directions  |
| 2-vs-1 collision (multiple straights) | Fixed: geometric check, not TCA for curved arcs            |
| Vehicle stuck after phase change      | Vehicle-in-intersection awareness now handles this          |
| Turn vs turn collision                | Left yields to right; same type falls through to OBB       |
| Vehicles too close when queuing       | Increase `follow_gap` (try 8–10)                           |
| Vehicles too far apart in queue       | Decrease `follow_gap` (try 3–4)                            |
| Different-sized vehicles overlapping  | Future: set per-point `bbox_hl`/`bbox_hw` in init_vehicles |

### Vehicle Issues (Existing)

| Issue              | Fix                              |
|:-------------------|:---------------------------------|
| Segment switching  | Increase `route_match_dist`      |
| Turning frequency  | Adjust `straight_bias`           |
| Edge vehicle loops | Add `grid_size`/`entry_dist` to `solver_step` |
| General collisions | Increase `min_safe_dist`         |

### Traffic Light Issues (Existing)

| Issue           | Fix                                              |
|:----------------|:-------------------------------------------------|
| Visibility      | Enable Point Markers or Disc display             |
| Timing          | Adjust `green_time`, `arrow_time`, `yellow_time` |
| Ignoring reds   | Ensure `signal_brake` is wired before `bbox_collision` |
| Stopping mid-road | Increase `signal_detect_dist`                  |

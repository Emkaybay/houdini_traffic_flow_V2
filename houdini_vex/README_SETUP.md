# Traffic Sim V2 — Solver-Based (RBD-Ready)

## Why Rebuild?

The old `gen_vehicle_points` recalculated positions from `@Time` every frame.
This means vehicles had **no persistent state** — braking could only offset
the position slightly, never truly stop. The new system uses a **SOP Solver**
where speed and position persist between frames, so `speed = 0` means the
vehicle genuinely stops.

---

## Architecture

```
gen_vehicle_routes ──→ route_curves (Null)
                            │
                            ├──→ [Solver Input 2]
                            │
init_vehicles ──────→ [Solver Input 1]
                            │
                      ┌─────▼──────┐
                      │ SOP Solver  │
                      │             │
                      │  prev_frame─┼──→ solver_step (input 0)
                      │  Input_2  ──┼──→ solver_step (input 1)
                      │             │
                      └─────┬──────┘
                            │
                      color_vehicles
                            │
                    ┌───────┴────────┐
                    │                │
              keep_vehicles    (prediction viz)
                    │
              ┌─────┴──────┐
              │ car_box     │
              │ (Box SOP)   │
              └─────┬──────┘
                    │
              copy_to_points ──→ DISPLAY
```

---

## Step-by-Step Setup in Houdini 21

### 1. Create the Geo Node
- At `/obj/` level, create a new Geometry node: `traffic_sim_v2`
- Dive inside

### 2. Route Generation
- Drop an **Attrib Wrangle**, name it `gen_vehicle_routes`
- Set **Run Over: Detail**
- Paste code from `01_gen_vehicle_routes.vex`
- Click the slider icon (**Create Spare Parameters**) to auto-create all `ch()` params
- Set defaults:
  | Param | Value |
  |-------|-------|
  | grid_size | 348 |
  | grid_divisions | 4 |
  | lane_spacing | 3.5 |
  | entry_dist | 20 |
  | arc_segments | 16 |

- Add a **Null** after it, name it `route_curves`

### 3. Vehicle Initialization
- Drop an **Attrib Wrangle**, name it `init_vehicles`
- Wire `route_curves` into its **Input 0**
- Set **Run Over: Primitives**
- Paste code from `02_init_vehicles.vex`
- Create Spare Parameters, set:
  | Param | Value |
  |-------|-------|
  | vehicle_speed | 15 |
  | vehicle_spacing | 30 |
  | vehicle_density | 0.4 |
  | vehicle_offset | 0.5 |

### 4. SOP Solver (the core)
- Drop a **Solver** SOP
- Wire `init_vehicles` into **Input 1** (left input)
- Wire `route_curves` into **Input 2** (second input)
- **Dive inside** the Solver

#### Inside the Solver:
- You'll see `prev_frame` node (provides previous frame's vehicle points)
- You'll see `Input_2` node (provides route curves, live)
- Drop an **Attrib Wrangle**, name it `solver_step`
- Wire `prev_frame` → `solver_step` **Input 0**
- Wire `Input_2` → `solver_step` **Input 1**
- Set **Run Over: Points**
- Paste code from `03_solver_step.vex`
- Create Spare Parameters, set:
  | Param | Value |
  |-------|-------|
  | max_speed | 15 |
  | acceleration | 8 |
  | deceleration | 25 |
  | look_ahead_dist | 30 |
  | cross_detect_dist | 35 |
  | min_safe_dist | 10 |
  | cross_safe_time | 1.5 |
  | search_count | 150 |
  | vehicle_offset | 0.5 |

- Wire `solver_step` → the **Output** node
- **Go back up** (out of the Solver)

### 5. Color Visualization
- Drop an **Attrib Wrangle** after the Solver, name it `color_vehicles`
- Set **Run Over: Points**
- Paste code from `04_color_vehicles.vex`
- No parameters needed

### 6. Vehicle Geometry (Simple Box)
- Drop a **Box** SOP, name it `car_box`
- Set size: `(4, 1.5, 2)` (length x height x width)
- Set center: `(0, 0.75, 0)` (so bottom sits on ground)

### 7. Copy to Points
- Drop a **Copy to Points** SOP, name it `copy_cars`
- Wire `car_box` → Input 1 (left, source geometry)
- Wire `color_vehicles` → Input 2 (right, target points)
- Set **Transform Using Target Point Attributes** to enabled
- Set **Use Implicit N** to enabled

### 8. Display
- Set display flag on `copy_cars`
- Press Play — vehicles should move along routes and **actually stop** at intersections

---

## How It Works

| Frame | What happens |
|-------|-------------|
| 1 | Solver takes `init_vehicles` output as initial state |
| 2+ | Solver reads **previous frame** vehicle points |
| | → Detects threats via pcfind (following + cross-traffic) |
| | → Sets target_speed (0 = stop, 15 = cruise) |
| | → Accelerates/decelerates toward target (persistent!) |
| | → Advances u_param by speed * dt / curve_length |
| | → Reads new position from route curves via primuv |

**Why braking works now:**
- Frame 10: threat detected → target_speed = 0
- Frame 11: speed = max(15 - 25*dt, 0) = 13.96 → vehicle slows
- Frame 12: speed = max(13.96 - 25*dt, 0) = 12.92 → keeps slowing
- ...
- Frame 24: speed = 0 → vehicle STOPPED (u_param doesn't advance)
- Threat clears → target_speed = 15 → speed gradually increases

---

## Parameters Quick Reference

| Param | Where | Default | Effect |
|-------|-------|---------|--------|
| max_speed | solver_step | 15 | Cruising speed (units/sec) |
| acceleration | solver_step | 8 | How fast vehicles speed up (units/sec²) |
| deceleration | solver_step | 25 | How fast vehicles brake (units/sec²) |
| look_ahead_dist | solver_step | 30 | Same-lane detection range |
| cross_detect_dist | solver_step | 35 | Intersection detection range |
| min_safe_dist | solver_step | 10 | Gap to maintain from threats |
| cross_safe_time | solver_step | 1.5 | Time clearance at crossings (seconds) |
| vehicle_density | init_vehicles | 0.4 | How many vehicles per route (0-1) |

**Tuning tips:**
- Vehicles stop too late? → Increase `deceleration` (try 40)
- Vehicles stop too early? → Decrease `cross_detect_dist` or `cross_safe_time`
- Too many collisions? → Increase `min_safe_dist` (try 14)
- Too few vehicles? → Increase `vehicle_density` (try 0.6)
- Jerky stops? → Decrease `deceleration` (try 15)

---

## RBD Car Rig Transition Path

This solver-based system is designed to plug directly into RBD:

```
CURRENT (Phase 1):
  SOP Solver → vehicle points → Copy to Points (box)

FUTURE (Phase 2):
  SOP Solver → vehicle "target" points
                    ↓
  DOP Network → RBD car rig
                    ↓
  Constraints connect RBD car to target points
  RBD handles: collisions, suspension, wheel spin, weight transfer
  Solver handles: navigation, lane following, intersection logic
```

**How to transition:**
1. Keep the SOP Solver as your "AI brain"
2. Create an RBD car rig (wheels, chassis, constraints)
3. Add a **Position** or **Spring** constraint from each RBD car to its matching solver point
4. The RBD car "chases" the solver point — physics handles the rest
5. The solver point = where the car WANTS to be
6. The RBD position = where the car ACTUALLY is (with physics)

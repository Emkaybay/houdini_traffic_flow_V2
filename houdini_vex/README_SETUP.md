# Traffic Sim V2 — Solver-Based (RBD-Ready)

## Architecture

```
gen_vehicle_routes ──→ route_curves (Null)
                            │
                            │
init_vehicles ──────→ [Solver Input 1]
                            │
                      ┌─────▼──────────────────────────────────┐
                      │            SOP Solver                   │
                      │                                         │
                      │  prev_frame ──→ solver_step (input 0)   │
                      │                                         │
                      │  Object Merge ─→ solver_step (input 1)  │
                      │  (path: ../../route_curves)             │
                      │                                         │
                      │  solver_step ──→ Output                 │
                      └─────┬──────────────────────────────────┘
                            │
                      color_vehicles
                            │
                      copy_to_points ← car_box (Box: 4 x 1.5 x 2)
                            │
                          DISPLAY
```

---

## Step-by-Step Setup

### 1. Geo Node
- Create Geometry node: `traffic_sim_v2`
- Dive inside

### 2. Route Generation
- **Attrib Wrangle**: `gen_vehicle_routes`
- Run Over: **Detail**
- Paste: `01_gen_vehicle_routes.vex`
- Create Spare Parameters, set:
  - grid_size = 348, grid_divisions = 4
  - lane_spacing = 3.5, entry_dist = 20, arc_segments = 16
- Add a **Null** after it → name: `route_curves`

### 3. Vehicle Init
- **Attrib Wrangle**: `init_vehicles`
- Wire `route_curves` → input 0
- Run Over: **Primitives**
- Paste: `02_init_vehicles.vex`
- Create Spare Parameters:
  - vehicle_speed = 15, vehicle_spacing = 30
  - vehicle_density = 0.4, vehicle_offset = 0.5
  - random_seed = 42

### 4. SOP Solver (IMPORTANT — read carefully)

- Drop a **Solver** SOP
- Wire `init_vehicles` → **Solver Input 1** (first/left input)
- **Dive inside** the Solver

#### Inside the Solver:

a) You see `prev_frame` — this gives previous frame's vehicle points

b) **Create an Object Merge** node:
   - Drop **Object Merge** node
   - Set **Object 1** path to: `../../route_curves`
   - (This reaches UP out of the solver, to the route_curves null)

c) Drop an **Attrib Wrangle**, name it `solver_step`:
   - Wire `prev_frame` → **solver_step input 0** (first input)
   - Wire `Object Merge` → **solver_step input 1** (second input)
   - Run Over: **Points**
   - Paste: `03_solver_step.vex`
   - Create Spare Parameters:
     - max_speed = 15
     - acceleration = 8
     - deceleration = 25
     - look_ahead_dist = 30
     - cross_detect_dist = 35
     - min_safe_dist = 10
     - cross_safe_time = 1.5
     - search_count = 150
     - vehicle_offset = 0.5

d) Wire `solver_step` → **Output** node

e) **Go back up** (out of the Solver)

### 5. Color
- **Attrib Wrangle**: `color_vehicles`
- Wire Solver output → input 0
- Run Over: **Points**
- Paste: `04_color_vehicles.vex`

### 6. Box + Copy
- **Box** SOP: `car_box`, size (4, 1.5, 2), center (0, 0.75, 0)
- **Copy to Points**: wire car_box → input 1, color_vehicles → input 2
- Enable: Transform Using Target Point Attributes, Use Implicit N

### 7. Play
- Set display flag on Copy to Points
- Press Play

---

## Troubleshooting

**Vehicles stack at origin?**
→ Object Merge path is wrong. Inside solver, click the Object Merge,
  check the path resolves to the route_curves null.
  Try: `../../route_curves` or `/obj/traffic_sim_v2/route_curves`

**Vehicles don't move?**
→ Check solver_step has two inputs connected:
  Input 0 = prev_frame, Input 1 = Object Merge

**Vehicles jitter/teleport?**
→ Lower deceleration (try 15), increase min_safe_dist (try 14)

**Too few vehicles?**
→ Increase vehicle_density in init_vehicles (try 0.6)

**Want different random layout?**
→ Change random_seed in init_vehicles

---

## RBD Transition

```
PHASE 1 (NOW):
  SOP Solver → vehicle points → Copy to Points (box)

PHASE 2 (LATER):
  SOP Solver → "target" points
                    ↓
  DOP Network → RBD car rig follows targets via constraints
  Physics handles: collisions, suspension, wheels
  Solver handles: navigation, lane logic, intersection stops
```

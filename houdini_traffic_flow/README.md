# Procedural Traffic Flow Intersection Curves — Houdini 21

Generates ornamental traffic flow curves at every intersection of a 4x4 grid road system.
Shows all possible vehicle paths: straight-through and left/right turns (no U-turns).
Includes directional chevron indicators at curve midpoints.

---

## Quick Start (Automated)

1. Open **Houdini 21**
2. Open **Windows > Python Shell** (or Python Source Editor)
3. Paste the contents of `setup_traffic_flow.py` and press Enter
4. The full node network is created under `/obj/traffic_flow_curves`

The script creates all nodes, wires them, sets VEX snippets, and adds spare parameters automatically.

---

## Manual Setup (Using .vfl files)

If you prefer to build the network by hand:

### Node Network (Top to Bottom)

```
Grid (348x348, Rows=5, Cols=5, Orient=ZX Plane)
  |
ConvertLine (or set Grid surftype to Rows & Columns)
  |
Fuse (snap dist: 0.001)
  |
AttribWrangle: "classify_intersections"    [vex/classify_intersections.vfl]
  |               |               |
Blast            Blast            Blast
"type_4way"      "type_3way"      "type_corner"
(negate=ON)      (negate=ON)      (negate=ON)
  |               |               |
AttribWrangle    AttribWrangle    AttribWrangle
"gen_4way"       "gen_3way"       "gen_corner"
  |               |               |
  +-------+-------+
          |
       Merge
          |
    Resample (length=2)
          |
    AttribWrangle: "color_visualization"    [vex/color_visualization.vfl]
          |
       Merge (input 0: original grid lines, input 1: colored curves)
```

### Steps

1. **Grid SOP**: Size 348x348, Rows 5, Cols 5, Orient ZX Plane (Y up)
2. **Convert SOP** (or use Grid with Surface Type = Rows & Columns)
3. **Fuse SOP**: Snap Distance 0.001
4. **Attribute Wrangle** "classify_intersections": Run Over Points, paste `vex/classify_intersections.vfl`
5. **Three Blast SOPs**: Each uses a point group from step 4:
   - Group: `type_4way`, Negate (Delete Non Selected) ON
   - Group: `type_3way`, Negate ON
   - Group: `type_corner`, Negate ON
6. **Three Attribute Wrangles** (each Run Over Points):
   - `gen_4way_curves` → paste `vex/gen_4way_curves.vfl` → add spare params (see below)
   - `gen_3way_curves` → paste `vex/gen_3way_curves.vfl` → add spare params
   - `gen_corner_curves` → paste `vex/gen_corner_curves.vfl` → add spare params
7. **Merge SOP**: Connect all three curve wrangle outputs
8. **Resample SOP**: Length 2, check "Maintain Last Vertex"
9. **Attribute Wrangle** "color_visualization": Run Over Points, paste `vex/color_visualization.vfl`
10. **Merge SOP**: Input 0 = ConvertLine output, Input 1 = colored curves

### Spare Parameters (add to each curve wrangle)

| Parameter | Channel | Type | Default | Range |
|-----------|---------|------|---------|-------|
| Road Width | `road_width` | Float | 35 | 10 – 43 |
| Turn Radius | `turn_radius` | Float | 30 | 10 – 43 |
| Arc Segments | `arc_segments` | Int | 16 | 4 – 32 |
| Chevron Size | `chevron_size` | Float | 8 | 2 – 20 |

To add spare parameters: RMB on wrangle node > Edit Parameter Interface > drag Float/Int from left panel.

---

## Curve Colors

| Type | Color | Description |
|------|-------|-------------|
| Straight-through | Green `{0.13, 1.0, 0.4}` | 180-degree opposing directions (N-S, E-W) |
| Turn | Red `{1.0, 0.23, 0.19}` | 90-degree perpendicular directions |
| Default | Blue `{0.5, 0.5, 1.0}` | Fallback for unclassified curves |

---

## Intersection Counts (4x4 grid = 5x5 points)

| Type | Count | Curves per intersection | Total curves |
|------|-------|------------------------|--------------|
| 4-way (interior) | 9 | 6 (2 straight + 4 turns) | 54 |
| 3-way (edge) | 12 | 3 (1 straight + 2 turns) | 36 |
| Corner | 4 | 1 (1 turn) | 4 |
| **Total** | **25** | | **94 curves** |

---

## Assumptions

- Grid on XZ plane (Y up) — Houdini's default ground plane
- Grid spacing: 348 / 4 = 87 units per cell
- All intersection types get flow curves
- Chevrons indicate traffic direction at curve midpoints
- No U-turns generated
- Compatible with Houdini 21 (standard VEX functions since H18+)

---

## File Listing

```
houdini_traffic_flow/
  README.md                          # This file
  setup_traffic_flow.py              # Auto-builds full node network (run in Python Shell)
  vex/
    classify_intersections.vfl       # Phase 1: Detect intersection types
    gen_4way_curves.vfl              # Phase 2: Curves at 4-way crossings
    gen_3way_curves.vfl              # Phase 3: Curves at T-junctions
    gen_corner_curves.vfl            # Phase 4: Curves at corners
    color_visualization.vfl          # Phase 5: Color by curve type
```

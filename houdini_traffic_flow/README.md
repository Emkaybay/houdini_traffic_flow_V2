# Procedural Traffic Flow Intersection Curves — Houdini 21

Multi-lane road system with quarter-circle turning arcs at every intersection
of a 4x4 grid. Matches the reference image: white lanes, red arcs, green grid.

---

## Quick Start

1. Open **Houdini 21**
2. Open **Windows > Python Shell**
3. Paste:
   ```python
   exec(open(r"C:\Users\KABELO\Downloads\houdini_traffic_flow\setup_traffic_flow.py").read())
   ```
4. Done — network is at `/obj/traffic_flow_curves`

---

## What It Creates

### Node Network

```
Grid (348x348, Rows=5, Cols=5, ZX Plane)
  |
ConvertLine  ──────────────────────────────────────────┐
  |                                                     |
Fuse (snap: 0.001)                                     |
  |                                                     |
  ├── gen_road_lanes (Detail wrangle)                  |
  |     |                                               |
  |   Blast "keep_only_lanes" (@is_lane==1, negate)    |
  |     → [WHITE ROAD LANES]                           |
  |                                                     |
  └── classify_intersections (Points wrangle)           |
        |                                               |
      gen_intersection_arcs (Points wrangle)            |
        |                                               |
      Resample (length=2)                               |
        → [RED TURNING ARCS]                            |
                                                        |
Final Merge ← [LANES] + [ARCS] + [GRID LINES] ────────┘
  |
color_visualization (Points wrangle)
  |
Display
```

### Visual Output

| Element | Color | Source |
|---------|-------|--------|
| Road lanes | White | gen_road_lanes — multi-lane parallel lines |
| Turning arcs | Red | gen_intersection_arcs — quarter-circle arcs at corners |
| Grid lines | Green | Original ConvertLine output |

---

## Parameters

### Lane Parameters (on `gen_road_lanes` node)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_size` | 348 | Total grid dimension |
| `grid_divisions` | 4 | Cells per axis |
| `num_lanes` | 5 | Number of parallel lane lines per road |
| `lane_spacing` | 3.0 | Distance between adjacent lanes |

### Arc Parameters (on `gen_intersection_arcs` node)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `road_half_width` | 15 | Distance from intersection centre to arc corner |
| `num_arcs` | 3 | Concentric arcs per corner |
| `inner_radius` | 6 | Radius of tightest (innermost) arc |
| `arc_spacing` | 5 | Gap between concentric arcs |
| `arc_segments` | 12 | Points per arc (smoothness) |
| `chevron_size` | 4 | Size of direction indicator (0 = disabled) |

---

## Intersection Coverage

| Type | Count | Corners per intersection | Total corner sets |
|------|-------|--------------------------|-------------------|
| 4-way (interior) | 9 | 4 | 36 |
| 3-way (edge) | 12 | 2 | 24 |
| Corner | 4 | 1 | 4 |
| **Total** | **25** | | **64 corner sets** |

Each corner set contains `num_arcs` concentric quarter-circle arcs.

---

## File Listing

```
houdini_traffic_flow/
  README.md                              # This file
  setup_traffic_flow.py                  # One-click Python setup script
  vex/
    classify_intersections.vfl           # Detect intersection types
    gen_road_lanes.vfl                   # Multi-lane parallel road lines
    gen_intersection_arcs.vfl            # Quarter-circle corner arcs + chevrons
    color_visualization.vfl              # White/red/green color coding
```

---

## Troubleshooting

- **No curves visible**: Check that `road_half_width` is less than half the cell size (348/4/2 = 43.5)
- **Arcs look angular**: Increase `arc_segments` (try 20-24)
- **Lanes too dense/sparse**: Adjust `num_lanes` and `lane_spacing`
- **ConvertLine error**: Make sure you're using Houdini 18+ (the `convertline` SOP)

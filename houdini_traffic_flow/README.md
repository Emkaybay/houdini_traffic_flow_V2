# Procedural Traffic Flow Intersection Curves — Houdini 21

Right-hand traffic, 2 lanes per direction, 14m road (3.5m lane spacing).
Inner lane = left turn only. Outer lane = right turn only.
Direction arrows placed between the lane lines.

---

## Quick Start

```python
exec(open(r"C:\Users\KABELO\Downloads\houdini_traffic_flow\setup_traffic_flow.py").read())
```

---

## What You Get

| Element | Color | Description |
|---------|-------|-------------|
| Road lanes | White | 5 parallel lines per road, 3.5m apart = 14m road |
| Right turn arcs | Red | Bezier curves at corners (outer lane) |
| Left turn arcs | Red | Bezier curves through centre (inner lane) |
| Direction arrows | Amber | V-chevrons in each lane gap before intersection |
| Grid lines | Green | Original grid reference |

## Lane Rules (Right-Hand Traffic)

```
             Travel direction
                  ↑
    ─────────┬────┬────┬─────────
    Oncoming │ L  │ R  │ Your side
    traffic  │turn│turn│ of road
    ─────────┴────┴────┴─────────
             inner outer
             lane  lane
```

- **Inner lane** (closer to centre line): Left turn only
- **Outer lane** (closer to curb): Right turn only
- At 3-way/corner intersections: if a turn is impossible, that lane shows straight-ahead arrow instead

## Parameters

### Lane Parameters (`gen_road_lanes`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_size` | 348 | Total grid dimension |
| `grid_divisions` | 4 | Cells per axis |
| `num_lane_lines` | 5 | Lines per road (5 lines = 4 lanes) |
| `lane_spacing` | 3.5 | Gap between lines (metres) |

### Arc & Indicator Parameters (`gen_intersection_arcs`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lane_spacing` | 3.5 | Must match lane generation |
| `entry_dist` | 8.0 | Where arcs start/end from intersection centre |
| `arc_segments` | 16 | Smoothness of turn curves |
| `chevron_size` | 2.0 | Size of direction arrow indicators |
| `indicator_dist` | 20.0 | How far before intersection the arrows sit |

## Files

```
houdini_traffic_flow/
  setup_traffic_flow.py              ← Run in Python Shell
  README.md                          ← This file
  vex/
    classify_intersections.vfl       ← Detect intersection types
    gen_road_lanes.vfl               ← Multi-lane roads (3.5m spacing)
    gen_intersection_arcs.vfl        ← Turn arcs + direction arrows
    color_visualization.vfl          ← White/red/amber/green colours
```

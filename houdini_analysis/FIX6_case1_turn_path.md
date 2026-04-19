# FIX 6 — CASE 1: Straight Yields to Turner Only If In Path

## The Problem

CASE 1 in the intersection handler:
```vex
if (!i_am_left_turn && !i_am_right_turn && nb_is_turning)
{
    // Unconditional yield — no check if turn arc crosses my path!
    if (nb_ahead_in_zone && nb_speed >= stop_thresh)
    {
        float yield_urg = fit(dist, 2.0, yield_dist, 0.85, 0.2);
        ...
    }
    continue;
}
```

Yields to ANY turning vehicle from a conflicting direction, regardless
of whether the turn arc actually crosses the straight vehicle's lane.

## Why Right Turns Don't Cross Straight Paths

Intersection geometry (from gen_vehicle_routes):
```
lane_spacing = 3.5
inner_off    = 1.75  (inner lane offset from road center)
outer_off    = 5.25  (outer lane offset from road center)
```

Right turns are on the OUTER lane (5.25 offset). They curve through
the outer corner of the intersection.

Straight-through vehicles:
  Inner lane: 1.75 from center -> 7+ units from right turn arc
  Outer lane: 5.25 from center -> 10+ units from right turn arc (perpendicular road)

Neither lane is anywhere near the right turn arc.

## When Turns DO Cross Straight Paths

Left turns are on the INNER lane (1.75 offset). They curve through
the inner part of the intersection, crossing inner-lane straight paths:

  Left turn from +Z: arc passes through Z ~ iz + 1.75
  Straight going +X inner: path at Z = iz + 1.75
  Lateral distance: 0-2 units -> GENUINE collision risk

## The Fix

Added two early-out checks before the yield:

```vex
// 1. Lateral clearance: is the turner in my forward corridor?
float lat_to_turner = abs(dot(sep, rgt));
float path_thresh = half_wid * 3.5;  // = 3.5 units

if (lat_to_turner > path_thresh)
{
    continue;  // Turner is far to my side, not in my path
}

// 2. Forward check: is the turner behind me?
float fwd_to_turner = dot(sep, fwd);

if (fwd_to_turner < -half_len * 2.0)
{
    continue;  // Turner is behind me, not a forward threat
}
```

### Threshold Analysis

| Scenario | Lateral distance | > 3.5? | Action |
|:---------|:-----------------|:-------|:-------|
| Right turn from perp direction, I'm inner lane | ~7+ units | YES | Skip (correct) |
| Right turn from perp direction, I'm outer lane | ~10+ units | YES | Skip (correct) |
| Left turn crossing my inner lane | 0-2 units | NO | Yield (correct) |
| Left turn far from my outer lane | ~3.5+ units | YES | Skip (correct) |
| Turn merging ahead into my lane | 0-1 units | NO | Yield (correct) |

### Tuning

| Symptom | Adjustment |
|:--------|:-----------|
| Still false-braking for distant right turns | Increase path_thresh multiplier (3.5 -> 4.0) |
| Missing yield for a turn that clips my lane | Decrease path_thresh multiplier (3.5 -> 3.0) |
| Vehicle ignoring turn that merges ahead | Should work: lat < 3.5 + fwd > 0 -> yields |

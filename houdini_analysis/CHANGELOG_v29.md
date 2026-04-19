# bbox_collision v2.8 -> v2.9 — False Braking Fix

## Problem
Straight vehicles (`intersection_straight`) falsely stop when passing
through intersections from perpendicular or opposing directions, even
though they're far apart and won't collide.

## Root Cause

Two `intersection_straight` vehicles from conflicting directions had
**no CASE** in the intersection handler. They fell through to the
**OBB safety net**, which uses **PADDED** bounding box dimensions:

```
Actual car:     4.0 x 2.0  (half_len=2.0, half_wid=1.0)
Padded OBB:     6.0 x 4.0  (hl=3.0, hw=2.0)  <-- used by safety net
```

Two perpendicular 6x4 boxes overlap when vehicles are ~7 units apart.
The SAT test returns gap = -0.25 -> hard stop triggered on cars that
are nearly 7 units apart and would safely pass.

Without padding (actual car size), gap = +1.75 -> correctly separated.

## Code Path (before fix)

```
Intersection handler:
  CASE 1: straight vs turning    -> handled
  (gap)                          -> straight vs straight: NO CASE!
  CASE 2: turning vs straight    -> handled
  ...

Falls through to Direction + Lane Filtering:
  CASE B (perpendicular): lat < body_width(2.0)?
    Inner-inner: lat=1.75 < 2.0 -> NOT skipped

Falls through to OBB Safety Net:
  Uses PADDED dimensions -> false overlap -> HARD STOP
```

## Fix: CASE 1b (inserted after CASE 1, before CASE 2)

### Three sub-cases:

| Heading relationship | Lateral clearance | Action | Why safe |
|:---------------------|:------------------|:-------|:---------|
| Opposing (dot < -0.3) | N/A | `continue` | Parallel offset lanes, spacing >= 3.5 > body 2.0 |
| Perpendicular, clear | Both > 2.5 | `continue` | Non-crossing paths (outer-outer, mixed) |
| Perpendicular, cross | One or both < 2.5 | TCA + **unpadded** OBB -> `continue` | Only brakes on genuine predicted overlap |

### Key design decision: `continue` at the end

Every branch of CASE 1b ends with `continue`, which means:
- The **padded** OBB safety net is **never reached** for straight-vs-straight
- The **padded** emergency check is **never reached**
- The **padded** predictive Phase 2 is **never reached**

For crossing paths, the custom TCA inside CASE 1b uses **unpadded** OBB
(actual car dimensions). This means:
- Cars that miss each other by 1+ unit of actual body clearance -> no brake
- Cars genuinely on collision course -> proportional brake based on TCA

## How to apply

**Option A (full replace):**
Paste the entire `08_bbox_collision_v29_fixed.vex` into your `bbox_collision` VEXpression.

**Option B (surgical insert):**
Insert the code from `CASE_1b_insert.vex` between CASE 1's closing `continue;` + `}`
and the `// ---------- CASE 2:` comment.

**No parameter changes** needed — same spare parms as v2.8.

## Tuning (if needed after testing)

| Symptom | Adjustment |
|:--------|:-----------|
| Perpendicular vehicles still false-braking | Increase `clear_thresh` multiplier (2.5 -> 3.0) |
| Perpendicular vehicles clipping through | Decrease `clear_thresh` multiplier (2.5 -> 2.0) or add small padding to TCA OBB check |
| Opposing vehicles on tight lanes clipping | Shouldn't happen (lane_spacing=3.5 > body=2.0). If custom lane_spacing < 2.0, remove the opposing `continue` |
| TCA braking too aggressive for crossing | Reduce urgency range: change `fit(tca, 0, look_time, 0.85, 0.15)` to `0.7, 0.1` |
| TCA braking too soft for crossing | Increase urgency range: change to `0.95, 0.2` |

## Updated Intersection Rules Table

| My segment              | Neighbour segment         | Action                          |
|:------------------------|:--------------------------|:--------------------------------|
| straight/road           | left_turn or right_turn   | I yield (turning priority)      |
| **intersection_straight** | **intersection_straight** | **CASE 1b: skip or TCA-only** (NEW) |
| left/right turn         | intersection_straight     | I skip (they yield)             |
| left_turn               | right_turn                | I yield (left yields to right)  |
| right_turn              | left_turn                 | I skip (they yield)             |
| road (approaching)      | Any in intersection       | I yield (they're committed)     |
| In intersection         | road (approaching)        | I skip (they yield)             |
| Same turn type          | Same turn type            | Fall through to OBB             |

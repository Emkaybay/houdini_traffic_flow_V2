# bbox_collision v2.8 -> v2.9 — Straight-vs-Straight Fix

## The Bug

In the Intersection Collision Handler inside `if (conflicting)`, there were 5 CASEs
covering all turn combinations, but **no CASE for two `intersection_straight` vehicles
from conflicting (perpendicular or opposing) directions**.

### Code path trace for two perpendicular straight vehicles:

```
Vehicle A: going +X, inner lane, on intersection_straight
Vehicle B: going +Z, inner lane, on intersection_straight
Both at the same intersection
```

1. `near_isect = 1`, `same_isect = 1` -> enters handler
2. `app_dot = dot(+X, +Z) = 0.0` -> `conflicting = 1`
3. CASE 1: `!i_am_turn && nb_is_turning` -> **NO** (B is straight, not turning)
4. CASE 2: `i_am_turning && nb_is_straight` -> **NO** (A is not turning)
5. CASE 3: `left vs right` -> **NO** (neither is turning)
6. CASE 4: `!i_in_isect && nb_in_isect` -> **NO** (both are `i_in_isect = 1`)
7. Falls through `if(conflicting)` block **with no yield decision**

Result: Both vehicles hit the OBB safety net -> **both brake -> potential deadlock**

### When this happens in practice:

During a traffic light phase transition:
- Frame N: Vehicle A has green, enters `intersection_straight`. B has red, stopped.
- Frame N+1: Phase changes. B gets green, starts entering `intersection_straight`.
- A is still crossing (signal_stop=0), B just released (signal_stop was 1).
- Both are now on `intersection_straight`, conflicting direction, NO yield rule.
- Both brake via OBB. Neither knows to proceed first.

## The Fix (CASE 1b)

Added between CASE 1 and CASE 2. The new code goes right after the
`continue;` at the end of CASE 1.

### Priority logic:

1. **I was signal-held last frame, they weren't** -> I yield (they're committed, crossing)
2. **They were signal-held last frame, I wasn't** -> they yield (I continue)
3. **Both same state** -> tiebreaker:
   - Vehicle with higher `u_param` (further along segment = closer to exit) has priority
   - If u_param is tied, lower `vehicle_id` yields

### Lateral clearance guard:

Before applying any yield, checks `lat_clear > half_wid * 3.0`.
Opposite-direction straights on offset lanes (e.g., +X outer and -X outer,
separated by ~10.5 units) skip this yield entirely — they pass safely side
by side. Only vehicles on genuinely crossing paths (inner lanes crossing
at ~1.75 unit offset) get the yield treatment.

## What to paste

Replace the entire VEXpression in `bbox_collision` with the contents of:
`08_bbox_collision_v29_fixed.vex`

No parameter changes needed — same spare parms as v2.8.

## Updated Intersection Collision Rules Table

| My segment                    | Neighbour segment          | Action                              |
|:------------------------------|:---------------------------|:------------------------------------|
| `intersection_straight`/`road`| `left_turn` or `right_turn`| **I yield** (turning has priority)  |
| **`intersection_straight`**   | **`intersection_straight`**| **signal_stop priority** (NEW v2.9) |
| `left_turn` or `right_turn`  | `intersection_straight`    | **I skip** (they yield to me)       |
| `left_turn`                   | `right_turn`               | **I yield** (left yields to right)  |
| `right_turn`                  | `left_turn`                | **I skip** (they yield to me)       |
| `road` (approaching)          | Any vehicle IN intersection| **I yield** (they're committed)     |
| Any IN intersection           | `road` (approaching)       | **I skip** (they yield to me)       |
| Same turn type                | Same turn type             | **Fall through to OBB safety net**  |

## Tuning (new for v2.9)

| Symptom                                     | Solution                                       |
|:--------------------------------------------|:-----------------------------------------------|
| Straight vehicles still deadlocking          | Verify `signal_stop` is being set by signal_brake |
| Yield triggering on parallel opposing lanes  | Should be filtered by `lat_clear > half_wid*3.0`; increase multiplier if needed |
| Wrong vehicle yielding during phase change   | Check that `signal_brake` sets `signal_stop = 1` correctly for red-held vehicles |
| Both vehicles have signal_stop = 0           | Tiebreaker uses u_param; verify solver_step updates u_param correctly |

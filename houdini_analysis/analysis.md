# bbox_collision Analysis: Straight-vs-Straight at Intersections

## The Gap Found

In the **Intersection Collision Handler** (lines inside `if (conflicting)`), there are
5 explicit CASEs:

| CASE | My segment           | Neighbour segment        | Action          |
|------|----------------------|--------------------------|-----------------|
| 1    | straight/road        | left_turn OR right_turn  | I yield         |
| 2    | left/right turn      | intersection_straight    | I skip (they yield) |
| 3    | left_turn            | right_turn               | I yield         |
| 3b   | right_turn           | left_turn                | I skip          |
| 4    | NOT in intersection  | IN intersection          | I yield         |
| 4b   | IN intersection      | NOT in intersection      | I skip          |
| 5    | Same turn type       | Same turn type           | Fall through to OBB |

**MISSING**: `intersection_straight` vs `intersection_straight` from conflicting directions.

## What Happens Now (the bug)

Two vehicles both on `intersection_straight` from perpendicular approaches:

1. Both enter intersection handler (`near_isect = 1`, `same_isect = 1`)
2. `conflicting = 1` (perpendicular approach, `app_dot ≈ 0`)
3. CASE 1: NO — neighbour is NOT turning
4. CASE 2: NO — I'm NOT turning
5. CASE 3: NO — neither is turning
6. CASE 4: NO — BOTH are `i_in_isect = 1` (both on intersection_straight)
7. Falls through without any yield decision or `continue`

Then hits Direction + Lane Filtering → OBB checks → **BOTH vehicles brake simultaneously**

## The Problem

- No clear priority: Vehicle A brakes for B, Vehicle B brakes for A
- Potential deadlock: both stop, neither knows to yield
- During phase transitions: Vehicle A entered on previous green (still crossing),
  Vehicle B enters on new green → both on intersection_straight, conflicting,
  no yield rule → both brake

## The Fix

Add a CASE between CASE 2 and CASE 3 for straight-vs-straight:

**Priority rule**: Use previous frame's `signal_stop` to determine who was just released:
- I was signal-stopped last frame (just got green) → I yield to them (they're committed)
- They were signal-stopped last frame → they yield (I'm committed)
- Both same state → tiebreaker by vehicle_id (consistent, no oscillation)

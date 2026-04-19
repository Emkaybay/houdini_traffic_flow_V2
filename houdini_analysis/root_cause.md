# Root Cause Analysis: False Braking at Intersections

## The Problem
Two `intersection_straight` vehicles from perpendicular or opposing directions
FALSELY BRAKE even though they'll safely pass each other.

## Root Cause: Padded OBB Safety Net

The code path:

1. Intersection handler: both `intersection_straight`, conflicting direction
   → NO CASE matches (1-5 don't cover straight-vs-straight)
   → falls through WITHOUT `continue`

2. Direction + Lane Filtering (CASE B: perpendicular at intersection):
   - Checks `my_lat > body_width(2.0) && nb_lat > body_width(2.0)`
   - For inner-inner perpendicular: lat distances ≈ 1.75 < 2.0 → NOT skipped
   - Falls through to collision checks

3. **OBB SAFETY NET** fires with **PADDED** dimensions:
   - Padded: hl = 2.0 + 1.0 = 3.0, hw = 1.0 + 1.0 = 2.0
   - Each vehicle's padded footprint: 6.0 x 4.0 (vs actual 4.0 x 2.0)
   - Two perpendicular 6x4 boxes overlap when vehicles are ~7 units apart!

## Proof (SAT calculation)

Vehicle A at (-3, 0, 1.75) going +X, Vehicle B at (1.75, 0, -3) going +Z:
- Actual separation: 6.7 units
- Padded OBB gap on all 4 axes: -0.25 → OVERLAP (false!)
- UNPADDED OBB gap: +1.75 → SEPARATED (correct!)

## The Fix

Add CASE 1b for straight-vs-straight in the intersection handler that:

1. **Opposing (heading_dot < -0.3)**: `continue`
   → Parallel offset lanes, lane_spacing(3.5) > body_width(2.0), always safe

2. **Perpendicular, large lateral clearance**: `continue`
   → Different path offsets (outer-outer, inner-outer), won't cross

3. **Perpendicular, crossing paths**: Custom TCA with UNPADDED OBB → `continue`
   → Uses actual vehicle dimensions, not inflated padding
   → Only brakes if vehicles genuinely predicted to overlap at meeting point
   → Ends with `continue` so padded safety net is NEVER reached for this pair

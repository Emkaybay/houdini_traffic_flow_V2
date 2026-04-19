# bbox_collision Full Audit — v2.8 → v3.0

## 5 Issues Found (2 Medium, 3 Low)

---

### FIX 1 — CASE 1b: Straight-vs-Straight False Braking [MEDIUM]

**Already discussed.** Two `intersection_straight` vehicles from conflicting
directions had no CASE. Fell through to padded OBB safety net (6x4 boxes
overlap at ~7 units). Fixed with opposing skip + unpadded TCA for crossing paths.

---

### FIX 2 — Non-Threat Skip Bypasses Same-Lane Following [MEDIUM]

**Location:** Top of intersection handler, lines:
```vex
if (nb_signal_stop == 1 && !nb_is_turning) { continue; }
if (nb_speed < stop_thresh && !nb_is_turning) { continue; }
```

**Bug:** These skip ALL non-turning vehicles at the same intersection that are
signal-stopped or slow — including same-direction, same-lane vehicles AHEAD.

**Scenario:**
```
Vehicle A: going +X, outer lane, approaching intersection
Vehicle B: going +X, outer lane, stopped at red light (directly ahead of A)
Both at same intersection → same_isect = 1

→ nb_signal_stop == 1 && !nb_is_turning → CONTINUE
→ B is SKIPPED entirely: no following-distance check, no OBB, nothing
→ A drives into B's rear for 1 frame before signal_brake stops A
```

**Why it matters:** The `continue` skips the ENTIRE foreach iteration.
Following-distance, OBB safety net, emergency, TCA — all bypassed.
signal_brake mitigates (stops A too), but there's a 1-frame window
where A advances ~0.6 units into B's space.

**Fix (v3.0):** Check approach direction before skipping. Only skip
vehicles from a DIFFERENT approach:
```vex
vector nb_app_2d = normalize(set(nb_approach.x, 0, nb_approach.z));
float approach_dot = dot(my_app_2d, nb_app_2d);
int diff_approach = (approach_dot < 0.5) ? 1 : 0;

if (diff_approach) {
    if (nb_signal_stop == 1 && !nb_is_turning) { continue; }
    if (nb_speed < stop_thresh && !nb_is_turning) { continue; }
}
```

Same-approach vehicles (ahead on same lane) always fall through to
following-distance and OBB checks, even if signal-stopped.

---

### FIX 3 — bbox_stopped Blocks Acceleration on Mild Braking [MEDIUM]

**Location:** End of bbox_collision:
```vex
if (do_brake) {
    ...
    i@bbox_stopped = 1;   // ← set for ANY urgency, even 0.1
}
```

**In solver_step:**
```vex
if (i@bbox_stopped == 1 || i@signal_stop == 1)
    target = min(target, cur_speed);  // blocks acceleration
```

**Bug:** A distant TCA prediction (urg = 0.1, barely any speed reduction)
sets `bbox_stopped = 1`. On the next frame, solver_step reads this and
prevents acceleration. If the mild TCA keeps triggering (distant crossing
vehicle at the edge of search_radius), the vehicle gradually decelerates
for no visible reason. The driver appears to slow down in open road.

**Fix (v3.0):** Threshold on urgency:
```vex
i@bbox_stopped = (worst_urg > 0.3) ? 1 : 0;
```

Mild braking (< 0.3 urgency) no longer blocks acceleration on the
next frame. Only significant braking events prevent speed recovery.

---

### FIX 4 — Different-Lane Perpendicular Filter Too Permissive [LOW]

**Location:** Direction + Lane Filtering, different lane type:
```vex
float lat_dist = abs(dot(sep, rgt));
if (lat_dist > half_wid * 2.0) continue;
```

**Bug:** Only checks lateral distance from MY perspective. Two vehicles
may be clear of each other from the NEIGHBOUR's perspective but not
from mine (or vice versa).

**Scenario:**
```
A: going +X inner, at (ix+2, 0, iz+1.75)
B: going +Z outer, at (ix+5.25, 0, iz)
sep = (3.25, 0, -1.75)

My perspective:  lat_dist = |dot(sep, my_rgt)| = 1.75  (< 2.0 → NOT skipped)
Their perspective: nb_lat = |dot(-sep, nb_rgt)| = 3.25  (> 2.0 → clear!)

B is clearly outside A's forward path (3.25 > 2.0), but the one-sided
check doesn't see this. Falls through to padded OBB → false hard stop.
```

**Fix (v3.0):** Check from BOTH perspectives, skip if EITHER is clear:
```vex
float my_lat_dist = abs(dot(sep, rgt));
vector nb_rgt_chk = set(-nb_fwd_chk.z, 0, nb_fwd_chk.x);
float nb_lat_dist = abs(dot(-sep, nb_rgt_chk));
float cross_body = half_wid * 2.0;
if (my_lat_dist > cross_body || nb_lat_dist > cross_body) continue;
```

If either vehicle is clear of the other's forward path → safe to skip.

---

### FIX 5 — CASE B Threshold Too Tight [LOW]

**Location:** CASE B (perpendicular at intersection, same lane type):
```vex
float body_width = half_wid * 2.0;  // = 2.0
if (my_lat > body_width && nb_lat > body_width) continue;
```

**Bug:** Threshold = 2.0 = sum of two unpadded half-widths. Vehicles
at exactly body-touching distance (2.0 lateral clearance, zero gap)
fall through to the padded OBB safety net, which sees overlap due
to the 1.0 padding per side.

**Fix (v3.0):** Increase multiplier to 2.5:
```vex
float body_width = half_wid * 2.5;  // = 2.5
```

Adds a 0.25-unit gap per side before falling through to padded checks.

---

## Non-Issues (Investigated, No Fix Needed)

| Item | Status | Reason |
|:-----|:-------|:-------|
| `road` vs `road` at intersection | OK | Rare; signal_brake handles timing; vehicles switch to intersection_straight before getting close |
| Following-gap lateral (3.0) vs CASE A (2.0) | OK | Following requires `dir_dot > 0.5` + same lane_type; opposing vehicles never match |
| Empty `segment_type` first frame | OK | Conservative behavior (no CASE matches → falls to OBB) |
| One-frame green release delay | OK | Known; signal_brake GREEN RELEASE handler mitigates |
| CASE 1 yields to turns that don't cross path | OK | Conservative (safe); turns complete quickly |

---

## Files

| File | Contents |
|:-----|:---------|
| `08_bbox_collision_v30_all_fixes.vex` | Complete fixed VEX — all 5 fixes applied |
| `08_bbox_collision_v29_fixed.vex` | Previous version — only FIX 1 (CASE 1b) |
| `CASE_1b_insert.vex` | Surgical insert for FIX 1 only |
| `full_audit_notes.md` | Detailed code path analysis |

**Recommendation:** Use `v30_all_fixes` for the complete fix set.
No parameter changes needed — same spare parms as v2.8.

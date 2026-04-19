# bbox_collision Full Audit — Code Path Analysis
# Tracing every path to find false brakes, missed collisions, and edge cases

## METHODOLOGY
# For each pair of vehicles, trace:
#   1. Intersection handler (same_isect? which CASE?)
#   2. Direction + Lane filtering (which branch?)
#   3. Which collision check fires? (safety net / emergency / TCA)
#   4. Expected behavior vs actual

## ====================================================================
## ISSUE 1: Non-Threat Skip Too Broad
## ====================================================================
## SEVERITY: MEDIUM
##
## Code:
##   if (nb_signal_stop == 1 && !nb_is_turning) { continue; }
##   if (nb_speed < stop_thresh && !nb_is_turning) { continue; }
##
## This skips ALL non-turning signal-stopped/slow vehicles at the same
## intersection — including same-direction same-lane vehicles AHEAD.
##
## Scenario:
##   Vehicle A: going +X, outer lane, approaching intersection
##   Vehicle B: going +X, outer lane, stopped at red light (ahead of A)
##   Both at same intersection, same_isect = 1
##
##   → nb_signal_stop = 1 && !nb_is_turning(1) → CONTINUE
##   → B is SKIPPED ENTIRELY — no following distance check!
##   → A would drive into the back of B
##
## Mitigation: signal_brake also stops A (same red light). But there's
## a 1-frame window where bbox_collision doesn't slow A down. With
## dt=1/24, A advances ~0.625 units before signal_brake stops it.
##
## FIX: Only skip if the neighbour is from a DIFFERENT approach:
##   if (nb_signal_stop == 1 && !nb_is_turning) {
##       vector nb_app_2d = normalize(set(nb_approach.x, 0, nb_approach.z));
##       vector my_app_2d = normalize(set(my_approach.x, 0, my_approach.z));
##       if (dot(my_app_2d, nb_app_2d) < 0.5) continue;  // different approach → skip
##       // same approach → fall through (still need following distance)
##   }

## ====================================================================
## ISSUE 2: bbox_stopped Set on Any Braking (Prevents Acceleration)
## ====================================================================
## SEVERITY: MEDIUM
##
## Code (end of bbox_collision):
##   if (do_brake) {
##       ...
##       i@bbox_stopped = 1;
##   }
##
## solver_step reads:
##   if (i@bbox_stopped == 1 || i@signal_stop == 1)
##       target = min(target, cur_speed);  // DON'T ACCELERATE
##
## Problem: Even a mild brake (urg = 0.1 from a distant TCA prediction)
## sets bbox_stopped = 1 → solver_step prevents acceleration next frame.
## If the mild TCA keeps triggering each frame (distant crossing vehicle),
## the vehicle gradually decelerates without clear reason.
##
## FIX: Only set bbox_stopped when urgency is significant:
##   i@bbox_stopped = (worst_urg > 0.3) ? 1 : 0;

## ====================================================================
## ISSUE 3: Different-Lane Perpendicular False Brake at Intersection
## ====================================================================
## SEVERITY: LOW-MEDIUM (less common than inner-inner, but possible)
##
## Scenario:
##   Vehicle A: going +X, inner lane, at (ix+2, 0, iz+1.75)
##   Vehicle B: going +Z, outer lane, at (ix+5.25, 0, iz)
##   Different lane types: inner vs outer
##
## Path through filtering:
##   Different lane → near_isect(yes) → signal_stop(no) → speed(ok)
##   → abs(dir_dot) > 0.7? abs(0)=0, NO → doesn't skip
##   → lat_dist = 1.75, > half_wid*2.0(2.0)? NO → doesn't skip
##   → Falls through to OBB safety net
##
## OBB with padded dimensions (sep = (3.25, 0, -1.75)):
##   gap = -1.75 → OVERLAP → HARD STOP (false positive!)
##
## OBB WITHOUT padding:
##   gap = +0.25 → SEPARATED (correct!)
##
## FIX: Increase different-lane lateral threshold or add intersection
## straight-through awareness to the different-lane filter:
##   float lat_dist = abs(dot(sep, rgt));
##   if (lat_dist > half_wid * 2.5) continue;  // was 2.0
##
## Or: check both lateral directions (like CASE B does):
##   vector nb_rgt_tmp = set(-nb_fwd_chk.z, 0, nb_fwd_chk.x);
##   float nb_lat = abs(dot(-sep, nb_rgt_tmp));
##   if (lat_dist > half_wid * 2.0 || nb_lat > half_wid * 2.0) continue;
##   // (OR instead of AND — if EITHER vehicle is clear of the other's path)

## ====================================================================
## ISSUE 4: CASE B Threshold Slightly Too Tight
## ====================================================================
## SEVERITY: LOW
##
## Code:
##   float body_width = half_wid * 2.0;  // = 2.0
##   if (my_lat > body_width && nb_lat > body_width) continue;
##
## Vehicles at exactly 2.0 lateral clearance (bodies touching, no gap)
## fall through to padded OBB. The padded OBB then triggers false stops.
##
## FIX: Increase to 2.5x for safety margin:
##   float body_width = half_wid * 2.5;  // = 2.5

## ====================================================================
## ISSUE 5: road vs road at Same Intersection (No Explicit Handling)
## ====================================================================
## SEVERITY: LOW (rare scenario, traffic lights mitigate)
##
## Two "road" vehicles approaching the same intersection from
## perpendicular directions. Both have i_in_isect = 0, nb_in_isect = 0.
## No CASE matches in intersection handler.
##
## Falls through to filtering → CASE B → OBB checks.
## Usually handled by signal_brake (one has red). But during clearance
## overlap, both might be moving.
##
## Not a major issue — by the time they're close enough for OBB to
## trigger, at least one should have switched to intersection_straight
## (handled by CASE 1b).

## ====================================================================
## ISSUE 6: Following Gap Lateral Check Asymmetry
## ====================================================================
## SEVERITY: LOW (edge case only)
##
## Following gap lateral check: lat < half_wid * 3.0 = 3.0
## But CASE A (opposite direction) skip: lat_dist > half_wid * 2.0 = 2.0
##
## A vehicle at 2.5 units lateral distance, same direction:
##   - Enters following gap (2.5 < 3.0)
##   - Gets following-distance braking
##
## A vehicle at 2.5 units lateral distance, opposite direction:
##   - CASE A: 2.5 > 2.0 → continue → skipped
##
## Both thresholds use different multipliers (3.0 vs 2.0).
## The following gap at 3.0 is slightly generous — could catch
## vehicles on adjacent lanes that happen to be same-direction.
## But the dir_dot > 0.5 + same lane_type check limits this.
##
## Not a real issue — just noting the asymmetry.

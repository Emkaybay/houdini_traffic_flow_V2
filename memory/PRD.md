# PRD: Procedural Traffic Flow Intersection Curves — Houdini 21

## What's Implemented (v6f — Apr 2026)

### Vehicle Awareness System
- 90% car reduction (density=0.1)
- 24-frame prediction polylines (follow actual route curves)
- Time-window collision detection (±4 frames tolerance)
- Oncoming traffic filter via `pred_dir` attribute (dot < -0.5 = skip)
- Distance-based braking with Hermite S-curve (brake_horizon=20 frames)
- Priority system (XOR hash) prevents mutual yielding

### Speed-Based Vehicle Colors
- GREEN = full speed / accelerating
- YELLOW = decelerating
- RED = stopped / near-stopped
- Smooth gradient interpolation based on f@brake value

### Debug Visualization
- Cyan prediction lines (bends on turns, truncates when braking)

### Version History
- v5: Initial animated vehicles
- v6: Density + awareness (bounced)
- v6b: Fixed bouncing, added oncoming filter
- v6c: Prediction-based collision, debug lines
- v6d: Fixed search radius for intersection detection
- v6e: Time-window comparison for near-miss crossings
- v6f: Oncoming filter via pred_dir, speed colors, stronger braking

## Backlog
- P1: Traffic light system
- P2: Varied vehicle sizes
- P3: Solver SOP for persistent state (true speed accumulation)

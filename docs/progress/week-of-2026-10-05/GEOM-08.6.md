# GEOM-08.6 — tool_tip trace/approach/retract waypoints from lifted segments (crackvision.tool_waypoints)

**Accepted:** 2026-10-09 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`1a9d5f0`](https://github.com/Janga786/rebot-crack-vision/commit/1a9d5f04d1336a32674cf5fa9853f13f5bc35cf9)

## What was done

Added the module that turns a lifted 3D crack-surface segment into a sequence of robot tool-tip waypoints: evenly spaced trace points 1cm off the surface along the surface normal, bracketed by approach/retract points 4cm off, with a smooth roll (no sudden twisting) and a per-point uncertainty check against the 1cm clearance budget. All 8 new tests and the full 332-test project suite pass.

Files changed:
- [src/crackvision/tool_waypoints.py](https://github.com/Janga786/rebot-crack-vision/blob/1a9d5f04d1336a32674cf5fa9853f13f5bc35cf9/src/crackvision/tool_waypoints.py) (+183 / −0)
- [tests/test_tool_waypoints.py](https://github.com/Janga786/rebot-crack-vision/blob/1a9d5f04d1336a32674cf5fa9853f13f5bc35cf9/tests/test_tool_waypoints.py) (+268 / −0)

Commits: [`1a9d5f0`](https://github.com/Janga786/rebot-crack-vision/commit/1a9d5f04d1336a32674cf5fa9853f13f5bc35cf9)

## Why it was done

`crackvision.tool_waypoints.segment_waypoints` converts one lifted segment into §10 tool_tip poses in base_link. Positions sit at the 0.01 m trace clearance along the outward normal, resampled every 2 mm of arc length. +X runs along the anti-normal with a deterministic parallel-transport roll. Approach and retract poses sit at 0.04 m. Each waypoint carries σ_along_normal and a within_budget flag.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty

## How it moves the project forward

- Geometry & calibration: **11/24** tasks accepted; whole project: **44/92**.
- REQ-GEOM-1: 5/9 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, suite ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this change. tool_waypoints.py does what card GEOM-08.6 asks: §10.5 arc-length resampling with both ends included, +X = −n_out, parallel-transport roll with both fallbacks, 0.01 m trace and 0.04 m approach/retract clearance, and the §10.4 σ_along_normal / within_budget check. The 8 unit tests pass. I also ran my own probe on a helix over a 0.1 m cylinder, using a tilted capture Z (written to /tmp; the working tree is unchanged). On every waypoint, the quaternion is unit length, R is a proper rotation, +X = −n to 1e-9, and position − surface = 0.01·n. Each step
…[394 chars clipped]”

## What it unlocks next

- **Ready to start:** GEOM-08.7 — crackvision.path3d CLI: paths.json + mask + capture record → data/paths3d/{case}_paths3d.json with eligibility

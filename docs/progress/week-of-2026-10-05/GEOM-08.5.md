# GEOM-08.5 — Lift ordered pixel polylines to base_link surface points with normals, gap policy and per-point covariance (crackvision.lift3d)

**Accepted:** 2026-10-09 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`44cd7e9`](https://github.com/Janga786/rebot-crack-vision/commit/44cd7e9e8d3c84a2fb502d9656c2e98fcbfe2ce0)

## What was done

Added the crackvision.lift3d module, which converts a 2D crack-path pixel trace plus a depth image into 3D points on the robot's base frame, each with an outward-facing surface normal and an uncertainty estimate. All 7 new tests pass, including checks against a tilted reference plane (normal accurate to better than 0.5 degrees) and against a numerically computed derivative to confirm the uncertainty formula is correct; the full project test suite (316 tests) still passes with no regressions.

After the first audit, a repair round (GEOM-08.5.R1) fixed the reported issues: Fixed a bug in the 3D crack-lifting pipeline (GEOM-08.5) where points created by bridging small gaps in a crack path were marked valid but silently carried NaN for several geometry fields (depth, 3D position in camera frame, normal uncertainty) instead of real interpolated values. Added tests proving every valid point, including these interpolated ones, now has fully finite, correctly-computed geometry, and tightened an existing test so it actually checks the camera's true centre pixel rather than a mislabeled off-axis one. The full test suite (319 tests) passes.

Files changed:
- [src/crackvision/lift3d.py](https://github.com/Janga786/rebot-crack-vision/blob/44cd7e9e8d3c84a2fb502d9656c2e98fcbfe2ce0/src/crackvision/lift3d.py) (+328 / −6)
- [tests/test_lift3d.py](https://github.com/Janga786/rebot-crack-vision/blob/44cd7e9e8d3c84a2fb502d9656c2e98fcbfe2ce0/tests/test_lift3d.py) (+346 / −7)

Commits: [`a05d93d`](https://github.com/Janga786/rebot-crack-vision/commit/a05d93dca1d2d00c62617ce40c930a1120e1803c), [`44cd7e9`](https://github.com/Janga786/rebot-crack-vision/commit/44cd7e9e8d3c84a2fb502d9656c2e98fcbfe2ce0)

## Why it was done

`crackvision.lift3d.lift_polyline` turns one paths.json dense polyline, plus depth, mask and a capture record, into per-point base_link surface points and outward normals. Each point gets an invalid reason, the §10 gap interpolation and segmentation, and the §10 first-order covariance. It never produces a point at the camera origin.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty
- Requirement **REQ-GEOM-2**: Camera-to-robot calibration with explicit transform conventions and residuals

## How it moves the project forward

- Geometry & calibration: **10/24** tasks accepted; whole project: **43/92**.
- REQ-GEOM-1: 4/9 contributing tasks done
- REQ-GEOM-2: 7/18 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, suite ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this repair. All three findings from the GEOM-08.5 review are fixed. Interpolated points now have finite geometry in every field: p_optical = T⁻¹·p_base, depth_m = p_optical z, and normal_rms_m, sigma_normal_rad and cov_base come from the bounding neighbour with the larger trace. The gap test now checks the exact lerp and nlerp values and the covariance inheritance, using neighbours at 0.12 m and 0.45 m so the two traces differ. It also asserts finite values over every valid point. The finite-difference Jacobian test now runs at the true principal-point pixel 
…[346 chars clipped]”
- Quality loop: the audit found 3 issue(s) that were fixed before acceptance (repair task GEOM-08.5.R1): Gap test does not verify interpolated values or covariance inheritance; Interpolated (valid=True) points carry NaN depth_m/p_optical/normal_rms_m/sigma_normal_rad; Covariance FD test has no centre (principal-point) pixel.

## What it unlocks next

- **Ready to start:** GEOM-08.6 — tool_tip trace/approach/retract waypoints from lifted segments (crackvision.tool_waypoints)
- Closer: GEOM-08.7 — crackvision.path3d CLI: paths.json + mask + capture record → data/paths3d/{case}_paths3d.json with eligibility (still needs GEOM-08.6)

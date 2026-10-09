# GEOM-08.7 — crackvision.path3d CLI: paths.json + mask + capture record → data/paths3d/{case}_paths3d.json with eligibility

**Accepted:** 2026-10-09 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`7debf79`](https://github.com/Janga786/rebot-crack-vision/commit/7debf792e3853be1e114a4a08e46c6316f6e5af5)

## What was done

Built the crackvision.path3d command-line tool, which takes a crack's 2D pixel path plus a robot camera-capture snapshot and computes the actual 3D positions, surface normals, and robot tool-tip waypoints needed to trace that crack in the robot's base frame, along with uncertainty estimates and a go/no-go eligibility flag. Verified with 17 new automated tests (plus the full 341-test project suite, all passing) covering the full output schema, a sub-millimeter position accuracy check against hand-computed geometry, correct gap-bridging/splitting behavior, and every required refusal case (missing or invalid capture data, mismatched image sizes, downscaled images).

Files changed:
- [src/crackvision/path3d.py](https://github.com/Janga786/rebot-crack-vision/blob/7debf792e3853be1e114a4a08e46c6316f6e5af5/src/crackvision/path3d.py) (+563 / −0)
- [tests/test_path3d_cli.py](https://github.com/Janga786/rebot-crack-vision/blob/7debf792e3853be1e114a4a08e46c6316f6e5af5/tests/test_path3d_cli.py) (+529 / −0)

Commits: [`7debf79`](https://github.com/Janga786/rebot-crack-vision/commit/7debf792e3853be1e114a4a08e46c6316f6e5af5)

## Why it was done

`./env.sh python -m crackvision.path3d` lifts every main path and branch of each case into the exact §10 JSON (points, segments, waypoints, calibration, uncertainty_model, execution_eligible + reasons, source sha256s). It follows INTERFACES §0 and refuses cases without a valid §9 capture record.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty
- Requirement **REQ-GEOM-2**: Camera-to-robot calibration with explicit transform conventions and residuals

## How it moves the project forward

- Geometry & calibration: **12/24** tasks accepted; whole project: **46/92**.
- REQ-GEOM-1: 6/9 contributing tasks done
- REQ-GEOM-2: 8/18 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, cli-help ✔, suite ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “crackvision.path3d correctly implements the §10 JSON schema, exit-code policy, and all required refusals, layered cleanly on the already-accepted capture_record/lift3d/tool_waypoints/kinematics modules without introducing new geometry logic. All three scheduler acceptance checks and the full 341-test suite pass when re-run independently. Scope is exactly the two files the card allows (path3d.py, test_path3d_cli.py), with no ROS imports and no robot-motion code paths.”

## What it unlocks next

- **Ready to start:** GEOM-08.8 — Synthetic ground-truth verification of pixel→base_link 3D paths and tool waypoints (+ evidence note)

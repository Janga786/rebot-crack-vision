# MOT-07.7 — Independent trajectory checker, part 2: path fidelity, constant tool speed (crackvision.kinematics FK) and retime-report consistency

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`39b0010`](https://github.com/Janga786/rebot-crack-vision/commit/39b001071cc368e42b2112cfe8fc14911b3f71b7)

## What was done

Finished the second half of the independent trajectory checker: it now also verifies that a planned robot path stays on its intended route (within 1e-6 rad), that crack-tracing moves hold a steady tool-tip speed computed from real forward kinematics, and that the retiming report's own numbers match what the checker independently recomputes. All 53 tests across both the new and existing test files pass, the full 456-test repository suite still passes with no regressions, and the checker still never depends on the ROS-based motion code it's meant to double-check.

After the first audit, a repair round (MOT-07.7.R1) fixed the reported issues: Fixed three independent-reviewer findings in the MOT-07.7 trajectory checker: it now correctly handles plans whose last leg returns to an earlier waypoint (e.g. a retract leg ending back at an approach point) instead of misassigning the revisited point to the wrong leg; a malformed retime report (missing/wrong-typed fields) now produces a clean failing check result instead of crashing the whole CLI; and the speed-check report now includes the L/v_0/v_max numbers the spec requires. Verified with the full test suite (462 tests passing) plus new targeted tests covering each fix, including a real end-to-end CLI run.</presentation_summary>

Files changed:
- [src/crackvision/trajectory_check.py](https://github.com/Janga786/rebot-crack-vision/blob/39b001071cc368e42b2112cfe8fc14911b3f71b7/src/crackvision/trajectory_check.py) (+575 / −32)
- [tests/test_trajectory_check_path.py](https://github.com/Janga786/rebot-crack-vision/blob/39b001071cc368e42b2112cfe8fc14911b3f71b7/tests/test_trajectory_check_path.py) (+608 / −0)

Commits: [`eea4423`](https://github.com/Janga786/rebot-crack-vision/commit/eea4423d7121ce3bb8006ef472e9399df05d4dd4), [`39b0010`](https://github.com/Janga786/rebot-crack-vision/commit/39b001071cc368e42b2112cfe8fc14911b3f71b7)

## Why it was done

crackvision.trajectory_check additionally verifies C-PATH (every point on the plan polyline, monotone, all plan configurations present), C-TOOLSPEED (trace legs located geometrically, tool_tip speed from crackvision.kinematics within the §16 band/ramp rules) and C-REPORT (the retime report agrees with the recomputed values).

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **41/51** tasks accepted; whole project: **103/138**.
- REQ-MOT-1: 33/40 contributing tasks done
- REQ-MOT-2: 25/32 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, independent ✔, kinematics-regression ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “All three MOT-07.7 findings are fixed, and I checked each one myself rather than relying on the implementer's claims. 1. **Revisited vertex (blocker).** I rebuilt the exact failing case: the MOT-07.5 fixture plan, retimed by the real `retime_plan`. The checker at base 44e8a4f still fails it: C-PATH reports a regression at point 25 and C-REPORT reports a point_range mismatch. At 39b0010 the CLI exits 0 and every check passes, including C-REPORT against the report's ranges [0,18], [18,20], [20,22], [22,25]. When nearest segments tie within `_PATH_TOL_RAD`, `project_polyline` 
…[834 chars clipped]”
- Quality loop: the audit found 3 issue(s) that were fixed before acceptance (repair task MOT-07.7.R1): C-TOOLSPEED numbers omit L, v_0 and v_max per trace leg; Projection breaks ties at a revisited plan configuration by taking the earliest segment, so correct retime_plan output fails C-PATH and C-REPORT; Malformed retime report crashes the checker (exit 1) instead of failing C-REPORT.

## What it unlocks next

- **Ready to start:** MOT-07.8 — Offline cross-validation: retimer vs independent checker vs G-LIMITS on a committed synthetic plan, with fault injection
- Closer: MOT-07.10 — Retime the view-plan transit leg and settle the §11.2 purpose for view moves (still needs MOT-07.9)

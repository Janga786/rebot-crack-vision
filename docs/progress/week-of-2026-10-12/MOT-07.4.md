# MOT-07.4 — Retimer: constant tool speed on trace legs + §11.2 trajectory and crackvision.retime_report/1 assembly with self-check

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`85ccd5e`](https://github.com/Janga786/rebot-crack-vision/commit/85ccd5e5f3c73cd55209b323d7cd0c2abcf099a5)

## What was done

Built the MOT-07.4 retimer: it takes a geometric crack-trace plan and produces a timed robot trajectory that moves at one steady, configurable speed along each trace while respecting every joint's speed/acceleration limit, refusing outright rather than guessing if a trace is too slow or configs don't match. Verified with 15 new automated tests (plus the existing 577 motion-package tests, all still passing) using two synthetic test traces -- a straight one and a curved one -- confirming the output trajectory and its accompanying report are self-consistent and pass every required safety check.

After the first audit, a repair round (MOT-07.4.R1) fixed the reported issues: Fixed a gap in the motion retiming report validator so it now rejects malformed or tampered report files: a bad parameters block, a non-standard timestamp, or an out-of-range point index all now cause a clear refusal instead of silently passing. Added test coverage confirming every reported trajectory boundary, speed statistic, and ramp length matches an independent recomputation from the actual trajectory data. Full test suite (600 tests) passes.</presentation_summary>

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/retime.py](https://github.com/Janga786/rebot-crack-vision/blob/85ccd5e5f3c73cd55209b323d7cd0c2abcf099a5/ros2_ws/src/crackvision_motion/crackvision_motion/retime.py) (+659 / −16)
- [ros2_ws/src/crackvision_motion/test/test_retime.py](https://github.com/Janga786/rebot-crack-vision/blob/85ccd5e5f3c73cd55209b323d7cd0c2abcf099a5/ros2_ws/src/crackvision_motion/test/test_retime.py) (+585 / −0)

Commits: [`d3a5ce9`](https://github.com/Janga786/rebot-crack-vision/commit/d3a5ce91f354bb3156b3ab93c1c9d93bd2121e1a), [`85ccd5e`](https://github.com/Janga786/rebot-crack-vision/commit/85ccd5e5f3c73cd55209b323d7cd0c2abcf099a5)

## Why it was done

crackvision_motion.retime turns a validated cartesian plan into a §11.2 joint_trajectory document plus a §16.4 retime report, holding each trace leg at a single constant tool_tip speed (with bounded ramps) within all joint limits, refusing (typed error) on sha mismatch, too-slow traces or any check_limits violation.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **38/51** tasks accepted; whole project: **100/138**.
- REQ-MOT-1: 31/40 contributing tasks done
- REQ-MOT-2: 23/32 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, regression ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting MOT-07.4.R1. Both findings are fixed and MOT-07.4's criteria still hold. - **Finding fa4ecd83f836fb00 (validator strictness):** `validate_retime_report` now checks `parameters` by passing it through the real `retime_core.load_retime_config`. `created_utc` must match the `YYYY-MM-DDTHH:MM:SSZ` format and be a real date. `totals` is now parsed before `legs`, so any `point_range` whose last index is ≥ `n_points` is rejected. I probed it with 20 malformed reports and it rejected every one. That covers all five cases from the original finding, plus Feb 30, a traili
…[918 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task MOT-07.4.R1): Report point_range and totals peak fractions are not tested against the trajectory; validate_retime_report is not strict for parameters / created_utc / point_range bounds.

## What it unlocks next

- **Ready to start:** MOT-07.5 — retime_plan CLI (ROS-free console script): plan → data/trajectories/{case}_trajectory.json + .retime.json

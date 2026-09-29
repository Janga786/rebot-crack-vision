# MOT-04.1 — Reachability config + boresight-down target/pose generator (pure Python)

**Accepted:** 2026-09-29 · **Work package:** Motion planning (L-MOTION) · **Track:** software · **Verified commit:** [`3abfc0f`](https://github.com/Janga786/rebot-crack-vision/commit/3abfc0f74ddbae9c3d844813e228bceb4dd45f6e)

## What was done

Built the pure-Python geometry and config layer for the arm's reachability sweep: a validated YAML config describing the sweep grid, boresight-down orientation sampling, and IK/placement parameters, plus a small library that turns it into concrete 3D target poses and a provable upper bound on how far the arm can physically reach (derived from the robot's URDF). All 26 unit tests pass, including checks that generated orientations are mathematically exact rotations and that the reach bound is a true worst-case bound. This is geometry/config only -- no robot motion, no ROS graph.

After the first audit, a repair round (MOT-04.1.R1) fixed the reported issues: Fixed a broken unit test in the motion-reachability config module that was checking outdated grid-sample counts left over from a prior step-size tuning change. All 26 tests in the reachability config/geometry test suite now pass, and the committed sweep config still validates cleanly with no ROS dependencies pulled in.

Files changed:
- [config/motion/reachability.yaml](https://github.com/Janga786/rebot-crack-vision/blob/3abfc0f74ddbae9c3d844813e228bceb4dd45f6e/config/motion/reachability.yaml) (+75 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py](https://github.com/Janga786/rebot-crack-vision/blob/3abfc0f74ddbae9c3d844813e228bceb4dd45f6e/ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py) (+525 / −0)
- [ros2_ws/src/crackvision_motion/test/test_reachability_core.py](https://github.com/Janga786/rebot-crack-vision/blob/3abfc0f74ddbae9c3d844813e228bceb4dd45f6e/ros2_ws/src/crackvision_motion/test/test_reachability_core.py) (+400 / −2)

Commits: [`7a3ef4a`](https://github.com/Janga786/rebot-crack-vision/commit/7a3ef4a15aedf753d324fafaced45bcd42e0c54f), [`3abfc0f`](https://github.com/Janga786/rebot-crack-vision/commit/3abfc0f74ddbae9c3d844813e228bceb4dd45f6e)

## Why it was done

config/motion/reachability.yaml (all values tagged nominal) plus a ROS-free module that validates it, enumerates the sweep targets in a fixed order, builds boresight-down TCP poses with sampled roll, and computes a provable reach bound from the URDF. All of it is unit-tested under the system python3.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **4/15** tasks accepted; whole project: **28/74**.
- REQ-MOT-2: 3/9 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, config-valid ✔, no-ros-import ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair correctly updates two hard-coded grid sample-count assertions in test_reachability_core.py to match the MOT-04.5 grid-step change (0.025→0.03) already committed in config/motion/reachability.yaml. The arithmetic is verified correct (16 x-samples, 27 y-samples), the change is minimally scoped to the test file, and all three scheduler acceptance checks plus the full 26-test suite pass, confirming MOT-04.1's original criteria still hold.”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task MOT-04.1.R1): check unit fails after upstream change.

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

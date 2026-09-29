# MOT-04.1 — Reachability config + boresight-down target/pose generator (pure Python)

**Accepted:** 2026-09-29 · **Work package:** Motion planning (L-MOTION) · **Track:** software · **Verified commit:** [`7a3ef4a`](https://github.com/Janga786/rebot-crack-vision/commit/7a3ef4a15aedf753d324fafaced45bcd42e0c54f)

## What was done

Built the pure-Python geometry and config layer for the arm's reachability sweep: a validated YAML config describing the sweep grid, boresight-down orientation sampling, and IK/placement parameters, plus a small library that turns it into concrete 3D target poses and a provable upper bound on how far the arm can physically reach (derived from the robot's URDF). All 26 unit tests pass, including checks that generated orientations are mathematically exact rotations and that the reach bound is a true worst-case bound. This is geometry/config only -- no robot motion, no ROS graph.

Files changed:
- [config/motion/reachability.yaml](https://github.com/Janga786/rebot-crack-vision/blob/7a3ef4a15aedf753d324fafaced45bcd42e0c54f/config/motion/reachability.yaml) (+75 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py](https://github.com/Janga786/rebot-crack-vision/blob/7a3ef4a15aedf753d324fafaced45bcd42e0c54f/ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py) (+525 / −0)
- [ros2_ws/src/crackvision_motion/test/test_reachability_core.py](https://github.com/Janga786/rebot-crack-vision/blob/7a3ef4a15aedf753d324fafaced45bcd42e0c54f/ros2_ws/src/crackvision_motion/test/test_reachability_core.py) (+396 / −0)

Commits: [`7a3ef4a`](https://github.com/Janga786/rebot-crack-vision/commit/7a3ef4a15aedf753d324fafaced45bcd42e0c54f)

## Why it was done

config/motion/reachability.yaml (all values tagged nominal) plus a ROS-free module that validates it, enumerates the sweep targets in a fixed order, builds boresight-down TCP poses with sampled roll, and computes a provable reach bound from the URDF. All of it is unit-tested under the system python3.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **2/15** tasks accepted; whole project: **25/74**.
- REQ-MOT-2: 1/9 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, config-valid ✔, no-ros-import ✔); independent audit accepted it (criteria: 9 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “MOT-04.1 delivers a pure-Python, ROS-free reachability config + boresight-down pose/target generator exactly within its allowed scope (3 files, no scope creep). All three scheduler acceptance commands re-run clean (26 tests pass, config loads, no rclpy/*_msgs imported). I independently re-derived the Rodrigues rotation-matrix formula by hand and confirmed the implementer's claimed sign-bug fix is correct, spot-checked orientation sampling and target_position math, and reviewed the reach-bound triangle-inequality argument (correct: rotations preserve vector norm, so it's a v
…[849 chars clipped]”

## What it unlocks next

- **Ready to start:** MOT-04.2 — Reachability map + placement contract (INTERFACES §7), map I/O, ROS-side CLI common layer

# MOT-05.2 — Joint-trajectory file library: validation, hashing, limit checks, time scaling, densification (pure Python)

**Accepted:** 2026-10-09 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`7203728`](https://github.com/Janga786/rebot-crack-vision/commit/7203728a286567b8588d1f7dcd3e4b5247c11081)

## What was done

Built the joint-trajectory validation library for the robot motion-execution safety gate: it checks a planned trajectory file against the real B601-DM joint limits (position, velocity, acceleration) and a documented near-limit "don't speed up toward a bound" rule, catching both genuinely out-of-range moves and over-fast moves before they could run on hardware. It also implements safe slow-down (uniform time scaling) and path densification for later collision checking. All 43 automated tests pass, including the two worked examples from the written specification.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/7203728a286567b8588d1f7dcd3e4b5247c11081/ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py) (+426 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/trajectory_estop.json](https://github.com/Janga786/rebot-crack-vision/blob/7203728a286567b8588d1f7dcd3e4b5247c11081/ros2_ws/src/crackvision_motion/test/fixtures/trajectory_estop.json) (+17 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/trajectory_out_of_limits.json](https://github.com/Janga786/rebot-crack-vision/blob/7203728a286567b8588d1f7dcd3e4b5247c11081/ros2_ws/src/crackvision_motion/test/fixtures/trajectory_out_of_limits.json) (+17 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/trajectory_overspeed.json](https://github.com/Janga786/rebot-crack-vision/blob/7203728a286567b8588d1f7dcd3e4b5247c11081/ros2_ws/src/crackvision_motion/test/fixtures/trajectory_overspeed.json) (+17 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/trajectory_smoke.json](https://github.com/Janga786/rebot-crack-vision/blob/7203728a286567b8588d1f7dcd3e4b5247c11081/ros2_ws/src/crackvision_motion/test/fixtures/trajectory_smoke.json) (+18 / −0)
- [ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/7203728a286567b8588d1f7dcd3e4b5247c11081/ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py) (+486 / −0)

Commits: [`7203728`](https://github.com/Janga786/rebot-crack-vision/commit/7203728a286567b8588d1f7dcd3e4b5247c11081)

## Why it was done

crackvision_motion/joint_trajectory.py loads and validates crackvision.joint_trajectory/1 and checks it against config/robot/b601_dm_limits.yaml (position with margin policy, explicit and finite-difference velocity/acceleration). It also implements the §11 uniform time scaling and densification for collision checks, with no ROS imports, unit tests and fixtures.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **9/27** tasks accepted; whole project: **56/104**.
- REQ-MOT-1: 7/24 contributing tasks done
- REQ-INT-2: 2/13 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, pure-python ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “joint_trajectory.py implements the §11.2 schema validation, the §11.6 margin policy, G-LIMITS (1) limit checks from both explicit fields and finite differences, uniform time scaling and densification. It imports no ROS modules. I re-ran both acceptance checks: 43 tests passed and the import grep returned exit 0. Every malformed case in the card has its own test with a specific message. The finite-difference margin pass compares the arriving segment speed with the leaving segment speed at each interior point. That reproduces all three §11.6 worked examples, and it flags at l
…[425 chars clipped]”

## What it unlocks next

- **Ready to start:** MOT-05.3 — Offline execution gate: commissioning/approval/execution-config loaders + full gate report (pure Python)

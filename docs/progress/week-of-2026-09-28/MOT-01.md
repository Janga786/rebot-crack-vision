# MOT-01 — Robot model reconciliation (vendor vs presentation URDF) + limits file

**Accepted:** 2026-09-28 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`e8fbba2`](https://github.com/Janga786/rebot-crack-vision/commit/e8fbba2d2828f8b551451178c4de94563328be6e)

## What was done

Settled which robot model file is 'the' B601-DM model for planning, confirmed our simulation copy matches the vendor file exactly, and caught a real mismatch where the physical robot's driver currently publishes a gripper-less model while MoveIt plans with the gripper-equipped one. Produced a sourced limits file (position, velocity, acceleration, effort for all 8 joints) with every number traced to its origin file, plus a small diff tool and tests to keep re-checking this automatically. All new tests pass (7/7), and the full existing test suite (69 tests) still passes with no regressions.

Files changed:
- [config/robot/b601_dm_limits.yaml](https://github.com/Janga786/rebot-crack-vision/blob/e8fbba2d2828f8b551451178c4de94563328be6e/config/robot/b601_dm_limits.yaml) (+149 / −0)
- [docs/motion/ROBOT_MODEL.md](https://github.com/Janga786/rebot-crack-vision/blob/e8fbba2d2828f8b551451178c4de94563328be6e/docs/motion/ROBOT_MODEL.md) (+144 / −0)
- [scripts/motion/compare_urdf.py](https://github.com/Janga786/rebot-crack-vision/blob/e8fbba2d2828f8b551451178c4de94563328be6e/scripts/motion/compare_urdf.py) (+192 / −0)
- [tests/test_compare_urdf.py](https://github.com/Janga786/rebot-crack-vision/blob/e8fbba2d2828f8b551451178c4de94563328be6e/tests/test_compare_urdf.py) (+157 / −0)

Commits: [`e8fbba2`](https://github.com/Janga786/rebot-crack-vision/commit/e8fbba2d2828f8b551451178c4de94563328be6e)

## Why it was done

One canonical B601-DM model and a sourced limits file (position, velocity, acceleration, effort) for all planning.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene

## How it moves the project forward

- Motion planning: **1/10** tasks accepted; whole project: **14/69**.
- REQ-MOT-1: 1/9 contributing tasks done
- Verification: automated checks run by the pipeline itself (limits ✔, pytest ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I checked each claim against ~/rebot_ws, reading it without changing anything. HEAD is 2ebc4b8c, and exactly 2 tracked files are modified: moveit_config's kinematics.yaml and package.xml. Every value in the limits YAML matches the URDF at that commit (position and effort) or joint_limits.yaml (velocity and acceleration). Each value cites its source, and none are invented. compare_urdf.py reproduces both reported results: vendor vs sim is identical, and vendor vs the gripper-less fixend model shows only the gripper links/joints and a joint6 origin offset. The gripper_tcp fra
…[192 chars clipped]”

## What it unlocks next

- **Ready to start:** GEOM-03 — Calibration method + camera mounting decision
- **Ready to start:** HOST-04 — Arm USB adapter access via least-privilege udev rule
- **Ready to start:** MOT-02 — ROS 2 overlay workspace + headless MoveIt mock planning
- Closer: MOT-05 — Commissioning-gated execution interface (mock/dry/real) (still needs MOT-03, MOT-02)
- Closer: MOT-07 — Time parameterization within vel/acc limits + independent checker (still needs MOT-06, MOT-02)

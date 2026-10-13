# MOT-07.2 — Pure-numpy tool_tip FK in crackvision_motion (tool_fk) with parity against crackvision.kinematics

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`ed8401c`](https://github.com/Janga786/rebot-crack-vision/commit/ed8401cbfcff6763e32603a459eaf10100268c1a)

## What was done

Added a pure-numpy, ROS-free forward-kinematics module (tool_fk) that computes the robot's tool-tip position/orientation from joint angles, matching the existing reference implementation to better than a nanometre/nanoradian on 25 test configurations. Verified it correctly refuses malformed inputs (missing files, broken robot models, wrong joint counts) and computes 4000 configurations in about 0.05 seconds, well within the required speed bound.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/tool_fk.py](https://github.com/Janga786/rebot-crack-vision/blob/ed8401cbfcff6763e32603a459eaf10100268c1a/ros2_ws/src/crackvision_motion/crackvision_motion/tool_fk.py) (+206 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/retime/fk_reference.json](https://github.com/Janga786/rebot-crack-vision/blob/ed8401cbfcff6763e32603a459eaf10100268c1a/ros2_ws/src/crackvision_motion/test/fixtures/retime/fk_reference.json) (+549 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/retime/make_fk_reference.py](https://github.com/Janga786/rebot-crack-vision/blob/ed8401cbfcff6763e32603a459eaf10100268c1a/ros2_ws/src/crackvision_motion/test/fixtures/retime/make_fk_reference.py) (+112 / −0)
- [ros2_ws/src/crackvision_motion/test/test_tool_fk.py](https://github.com/Janga786/rebot-crack-vision/blob/ed8401cbfcff6763e32603a459eaf10100268c1a/ros2_ws/src/crackvision_motion/test/test_tool_fk.py) (+128 / −0)

Commits: [`ed8401c`](https://github.com/Janga786/rebot-crack-vision/commit/ed8401cbfcff6763e32603a459eaf10100268c1a)

## Why it was done

crackvision_motion.tool_fk computes base_link→tool_tip poses from the URDF chain plus end_effector.yaml's tool block without ROS, matching crackvision.kinematics to ≤1e-9 m / 1e-9 rad on a committed reference fixture whose freshness is checkable.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **36/51** tasks accepted; whole project: **98/138**.
- REQ-MOT-1: 30/40 contributing tasks done
- REQ-MOT-2: 21/32 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, fixture-fresh ✔, ros-free ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “tool_fk.py is ROS-free. It walks base_link→gripper_link the same way kinematics.load_chain does (Rz·Ry·Rx origins, Rodrigues rotation about the normalised axis) and builds the tool block with crackvision_description.end_effector.load_config/transform. It checks parent_link and exposes both shas. All three acceptance checks pass when I re-run them: 11 tests pass, the fixture is fresh, and the import grep finds nothing. The fixture has the zero state plus 24 seeded configs drawn within the b601_dm_limits.yaml limits, with non-trivial rotations. --check rebuilds the whole dict
…[329 chars clipped]”

## What it unlocks next

- Closer: MOT-07.4 — Retimer: constant tool speed on trace legs + §11.2 trajectory and crackvision.retime_report/1 assembly with self-check (still needs MOT-07.3)

# MOT-08.7 — RViz preview launch + rviz config (no MotionPlanning display) + headless publish smoke on the mock stack

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`7abef52`](https://github.com/Janga786/rebot-crack-vision/commit/7abef52f85126049e91d82d541dd6626fd7f5d4b)

## What was done

Added an RViz preview launch (preview_rviz.launch.py) that brings up the existing headless mock MoveIt stack plus RViz with a new preview.rviz config showing only RobotModel/Trajectory/PlanningScene/MarkerArray — no Plan/Execute buttons — so an operator can look at a planned trajectory in 3D without any motion capability. Since this dev box has no display, a headless smoke test (test_preview_rviz.sh) instead proves the underlying node graph works correctly: it verified the published trajectory preview has the expected 3 points and correct joint names, a non-empty crack/tool-path marker array, and that the robot's joint states never moved (max change 0.0) during the whole preview.

Files changed:
- [ros2_ws/src/crackvision_motion/launch/preview_rviz.launch.py](https://github.com/Janga786/rebot-crack-vision/blob/7abef52f85126049e91d82d541dd6626fd7f5d4b/ros2_ws/src/crackvision_motion/launch/preview_rviz.launch.py) (+91 / −0)
- [ros2_ws/src/crackvision_motion/rviz/preview.rviz](https://github.com/Janga786/rebot-crack-vision/blob/7abef52f85126049e91d82d541dd6626fd7f5d4b/ros2_ws/src/crackvision_motion/rviz/preview.rviz) (+72 / −0)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/7abef52f85126049e91d82d541dd6626fd7f5d4b/ros2_ws/src/crackvision_motion/setup.py) (+1 / −0)
- [scripts/ros/test_preview_rviz.sh](https://github.com/Janga786/rebot-crack-vision/blob/7abef52f85126049e91d82d541dd6626fd7f5d4b/scripts/ros/test_preview_rviz.sh) (+212 / −0)

Commits: [`7abef52`](https://github.com/Janga786/rebot-crack-vision/commit/7abef52f85126049e91d82d541dd6626fd7f5d4b)

## Why it was done

`ros2 launch crackvision_motion preview_rviz.launch.py trajectory:=PATH` brings up the mock planning stack, rviz2 with rviz/preview.rviz and the preview node. scripts/ros/test_preview_rviz.sh proves headlessly that /display_planned_path carries the trajectory.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **44/66** tasks accepted; whole project: **106/156**.
- REQ-MOT-1: 28/45 contributing tasks done
- REQ-OPS-1: 12/19 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, rviz-config ✔, launch-args ✔, smoke ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “MOT-08.7 adds preview_rviz.launch.py, rviz/preview.rviz, the setup.py data_files entry, and scripts/ros/test_preview_rviz.sh exactly as scoped. All four scheduler acceptance checks were independently reproduced (build, rviz-config, launch-args, smoke) with matching output. The launch file includes mock_planning.launch.py rather than duplicating it, declares the required trajectory/paths3d/speed_scale/rviz args, and starts preview_trajectory without --once per the card. preview.rviz has exactly the four required displays (RobotModel, moveit_rviz_plugin/Trajectory on /display
…[511 chars clipped]”

## What it unlocks next

- **Ready to start:** MOT-08.8 — Headless mock run: view plan → retime → check → preview → approve (pty) → execute_trajectory dry with G-APPROVAL pass, tamper refusals

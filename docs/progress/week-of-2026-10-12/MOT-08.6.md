# MOT-08.6 — preview_trajectory ROS node: DisplayTrajectory + tool-path/crack MarkerArray, latched, motion-free

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`77cb883`](https://github.com/Janga786/rebot-crack-vision/commit/77cb883a8210b2999d50295f6478c0528d4c1010)

## What was done

Added a new motion-free ROS node, preview_trajectory, that lets an operator see a planned arm trajectory and its crack-path overlay in RViz before approving a real run, without ever being able to command the arm. All 13 unit tests for its pure message-building logic pass, a static check confirms the code has no motion-capable API calls anywhere in the file, and a live run against the ROS graph successfully published both the trajectory preview and the crack-overlay markers and exited cleanly.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/preview_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/77cb883a8210b2999d50295f6478c0528d4c1010/ros2_ws/src/crackvision_motion/crackvision_motion/preview_trajectory.py) (+291 / −0)
- [ros2_ws/src/crackvision_motion/package.xml](https://github.com/Janga786/rebot-crack-vision/blob/77cb883a8210b2999d50295f6478c0528d4c1010/ros2_ws/src/crackvision_motion/package.xml) (+2 / −0)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/77cb883a8210b2999d50295f6478c0528d4c1010/ros2_ws/src/crackvision_motion/setup.py) (+1 / −0)
- [ros2_ws/src/crackvision_motion/test/test_preview_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/77cb883a8210b2999d50295f6478c0528d4c1010/ros2_ws/src/crackvision_motion/test/test_preview_trajectory.py) (+163 / −0)

Commits: [`77cb883`](https://github.com/Janga786/rebot-crack-vision/commit/77cb883a8210b2999d50295f6478c0528d4c1010)

## Why it was done

`ros2 run crackvision_motion preview_trajectory --trajectory PATH` publishes a latched moveit_msgs/DisplayTrajectory on /display_planned_path and a MarkerArray on /crackvision/preview_markers, built by pure, unit-tested functions. It never creates an action or service client.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **41/66** tasks accepted; whole project: **100/156**.
- REQ-MOT-1: 27/45 contributing tasks done
- REQ-OPS-1: 10/19 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, motion-free ✔, help ✔); independent audit accepted it (criteria: 9 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “preview_trajectory implements a motion-free rclpy node per §18.4: pure builders (build_display_trajectory, build_markers) are correctly specified and unit-tested, publishers use TRANSIENT_LOCAL+RELIABLE QoS on exactly the two required topics, --once/--wait-s/exit-code behavior all verified live against a real ROS graph, and setup.py/package.xml changes are scoped exactly as required. No findings.”

## What it unlocks next

- **Ready to start:** MOT-08.7 — RViz preview launch + rviz config (no MotionPlanning display) + headless publish smoke on the mock stack

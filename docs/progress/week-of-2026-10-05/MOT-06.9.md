# MOT-06.9 — plan_view node/CLI + headless smoke: reachable, collision-free camera view pose with the specimen in FOV

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`3ac7ff5`](https://github.com/Janga786/rebot-crack-vision/commit/3ac7ff5a97689a751cea29aa065bd87d7da768da)

## What was done

Added the plan_view ROS 2 CLI that picks a reachable, collision-free camera view pose over the specimen for MOT-06's camera-positioning phase, verified end-to-end on the mock MoveIt stack. The smoke test's own independent FK call confirmed the chosen pose places camera_link 0.25003 m from the specimen's top centre (within the required ±2 mm) with the optical axis only 0.064° off the ideal straight-down view, and a deliberately out-of-reach specimen centre correctly produced a refusal listing every tried candidate's failure reason.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/plan_view.py](https://github.com/Janga786/rebot-crack-vision/blob/3ac7ff5a97689a751cea29aa065bd87d7da768da/ros2_ws/src/crackvision_motion/crackvision_motion/plan_view.py) (+463 / −0)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/3ac7ff5a97689a751cea29aa065bd87d7da768da/ros2_ws/src/crackvision_motion/setup.py) (+1 / −0)
- [ros2_ws/src/crackvision_motion/test/test_plan_view.py](https://github.com/Janga786/rebot-crack-vision/blob/3ac7ff5a97689a751cea29aa065bd87d7da768da/ros2_ws/src/crackvision_motion/test/test_plan_view.py) (+350 / −0)
- [scripts/ros/test_view_plan.sh](https://github.com/Janga786/rebot-crack-vision/blob/3ac7ff5a97689a751cea29aa065bd87d7da768da/scripts/ros/test_view_plan.sh) (+214 / −0)

Commits: [`3ac7ff5`](https://github.com/Janga786/rebot-crack-vision/commit/3ac7ff5a97689a751cea29aa065bd87d7da768da)

## Why it was done

`ros2 run crackvision_motion plan_view` picks the first view candidate that passes the FOV check, IK for camera_link, independent FK/validity/limits re-check and an OMPL transit from home, writes a validated crackvision.view_plan/1, and scripts/ros/test_view_plan.sh proves it on the mock stack at the recommended placement.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **22/41** tasks accepted; whole project: **75/125**.
- REQ-MOT-1: 18/34 contributing tasks done
- REQ-MOT-2: 12/22 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, smoke ✔, regression ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I reran all four acceptance checks at 3ac7ff5 and all of them pass. The build exits 0, the full regression suite passes (512 tests) and the smoke test prints "OK". The positive case picks tilt=0 yaw=0. My own run of the smoke test's FK check put camera_link 0.25020 m from the specimen top centre, with the optical axis 0.050° off the straight-down direction. The --center 5.0 5.0 case exits 3 and lists the reason for each of the 8 candidates. plan_view.py follows §13.14: --center takes precedence over the scene; FOV is checked first, then IK for camera_link, an independent /c
…[450 chars clipped]”

## What it unlocks next

- **Ready to start:** MOT-06.10 — Synthetic end-to-end planning run (GEOM-08 synthetic case → paths3d → view + crack plan) and CARTESIAN_PLANNING.md

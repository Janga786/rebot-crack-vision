# MOT-02 — ROS 2 overlay workspace + headless MoveIt mock planning

**Accepted:** 2026-09-28 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`20e25c7`](https://github.com/Janga786/rebot-crack-vision/commit/20e25c706af6718d9132cfe72b6d009831995c90)

## What was done

This repo now has its own ROS 2 overlay workspace that builds on top of the robot's existing ROS install and can plan robot arm motions entirely in software, with no hardware attached. Running the two provided scripts builds the workspace and then boots a headless MoveIt planning stack against a mock version of the real 6-DOF arm, sends it a sample arm pose to plan to, and confirms the planner finds a valid, collision-free path -- verified with a real exit-code check, not just visually. The whole cycle runs in under 15 seconds and cleanly shuts down every process it started, and the robot's own separate workspace was left completely untouched.

Files changed:
- [.gitignore](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/.gitignore) (+5 / −0)
- [docs/motion/ROS_WORKSPACE.md](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/docs/motion/ROS_WORKSPACE.md) (+110 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/__init__.py](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/ros2_ws/src/crackvision_motion/crackvision_motion/__init__.py) (+0 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/plan_joint_goal.py](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/ros2_ws/src/crackvision_motion/crackvision_motion/plan_joint_goal.py) (+92 / −0)
- [ros2_ws/src/crackvision_motion/launch/mock_planning.launch.py](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/ros2_ws/src/crackvision_motion/launch/mock_planning.launch.py) (+136 / −0)
- [ros2_ws/src/crackvision_motion/package.xml](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/ros2_ws/src/crackvision_motion/package.xml) (+34 / −0)
- [ros2_ws/src/crackvision_motion/resource/crackvision_motion](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/ros2_ws/src/crackvision_motion/resource/crackvision_motion) (+0 / −0)
- [ros2_ws/src/crackvision_motion/setup.cfg](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/ros2_ws/src/crackvision_motion/setup.cfg) (+4 / −0)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/ros2_ws/src/crackvision_motion/setup.py) (+31 / −0)
- [scripts/ros/build_ws.sh](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/scripts/ros/build_ws.sh) (+15 / −0)
- [scripts/ros/env_ros.sh](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/scripts/ros/env_ros.sh) (+102 / −0)
- [scripts/ros/test_mock_plan.sh](https://github.com/Janga786/rebot-crack-vision/blob/20e25c706af6718d9132cfe72b6d009831995c90/scripts/ros/test_mock_plan.sh) (+86 / −0)

Commits: [`20e25c7`](https://github.com/Janga786/rebot-crack-vision/commit/20e25c706af6718d9132cfe72b6d009831995c90)

## Why it was done

This repo builds a ROS 2 overlay on ~/rebot_ws and can plan with MoveIt against mock hardware headlessly.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene

## How it moves the project forward

- Motion planning: **2/10** tasks accepted; whole project: **18/69**.
- REQ-MOT-1: 2/9 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, mock-plan ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “MOT-02 adds a ROS 2 overlay workspace (crackvision_motion) plus env_ros.sh/build_ws.sh/test_mock_plan.sh that build MoveIt+ros2_control mock planning headlessly on top of the read-only ~/rebot_ws underlay. Independently re-ran both acceptance checks from a clean build/install/log and both pass; verified no stray processes remain after the test and that ~/rebot_ws is untouched (matches the pre-existing baseline exactly).”

## What it unlocks next

- **Ready to start:** MOT-03 — Planning scene from config (nominal until measured)
- **Can now be planned in detail:** MOT-04 — Reachability map + recommended specimen placement
- Closer: MOT-05 — Commissioning-gated execution interface (mock/dry/real) (still needs MOT-03)
- Closer: MOT-06 — Cartesian path planning along crack paths (still needs MOT-03, MOT-04, GEOM-08, GEOM-08)
- Closer: MOT-07 — Time parameterization within vel/acc limits + independent checker (still needs MOT-06)
- Closer: MOT-08 — Trajectory preview + operator approval artefact (still needs MOT-07)

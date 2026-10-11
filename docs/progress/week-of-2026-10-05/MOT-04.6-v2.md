# MOT-04.6 — Reachability on task frames (tool_tip / camera_link) with a workcell-faithful specimen proxy + camera view check

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`b3bbc16`](https://github.com/Janga786/rebot-crack-vision/commit/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d)

## What was done

Implemented interactively by the technical-lead recovery session (Claude Opus 5.5, operator-authorised, 2026-09-30); commit 54e522e. Fixes MOT-04.5's frame error (standoffs meant as tool-tip clearances were applied to gripper_tcp) and the slab-under-base artefact without weakening any collision check. Coarse probe (scratch): 599/910 reachable, feasible 0.20 m placement at (0.29, 0), 200/200 half-step verification.

After the first audit, a repair round (MOT-04.6.R1) fixed the reported issues: The motion package's unit test suite runs again without a sourced ROS workspace: a shared pytest conftest now puts the in-tree description and motion packages on the import path. Verified: 308 unit tests pass, and the reachability sweep smoke tests and the placement CLI smoke test all exit 0.

_This step was a physical procedure performed by the operator; the evidence files below were recorded during it and then independently audited._

Files changed:
- [config/motion/reachability.yaml](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/config/motion/reachability.yaml) (+27 / −17)
- [ros2_ws/src/crackvision_motion/crackvision_motion/placement.py](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/ros2_ws/src/crackvision_motion/crackvision_motion/placement.py) (+85 / −5)
- [ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py) (+93 / −4)
- [ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py) (+71 / −21)
- [ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py) (+14 / −1)
- [ros2_ws/src/crackvision_motion/test/conftest.py](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/ros2_ws/src/crackvision_motion/test/conftest.py) (+16 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/reachability_task_frames_smoke.yaml](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/ros2_ws/src/crackvision_motion/test/fixtures/reachability_task_frames_smoke.yaml) (+36 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/reachability_view_smoke.yaml](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/ros2_ws/src/crackvision_motion/test/fixtures/reachability_view_smoke.yaml) (+33 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/scene_table.yaml](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/ros2_ws/src/crackvision_motion/test/fixtures/scene_table.yaml) (+21 / −0)
- [ros2_ws/src/crackvision_motion/test/test_placement.py](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/ros2_ws/src/crackvision_motion/test/test_placement.py) (+43 / −4)
- [ros2_ws/src/crackvision_motion/test/test_reachability_core.py](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/ros2_ws/src/crackvision_motion/test/test_reachability_core.py) (+92 / −4)
- [scripts/ros/test_reachability_task_frames.sh](https://github.com/Janga786/rebot-crack-vision/blob/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d/scripts/ros/test_reachability_task_frames.sh) (+57 / −0)

Commits: [`54e522e`](https://github.com/Janga786/rebot-crack-vision/commit/54e522e871b235ae0625ecaf49adc57cd292d336), [`b3bbc16`](https://github.com/Janga786/rebot-crack-vision/commit/b3bbc1648f1dac8ccc0c77aeb3b5fcd3bb7f8d5d)

## Why it was done

The sweep targets the physical task frame (tool_tip for tool clearances, camera_link for viewing distance) against self-collision incl. the camera, the workcell table and a specimen block that never overlaps the robot base; recommend_placement also emits a wrist-camera view check; the committed config uses this model.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **16/29** tasks accepted; whole project: **69/108**.
- REQ-MOT-1: 14/26 contributing tasks done
- REQ-MOT-2: 6/11 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, task-frames-smoke ✔, legacy-sweep-smoke ✔, cli-smoke ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair adds a single conftest.py to ros2_ws/src/crackvision_motion/test/ that puts the in-tree crackvision_description and crackvision_motion package directories on sys.path before pytest collection, fixing the collection failure of test_execute_trajectory.py (which transitively needs crackvision_description via execution_gate.py) when the unit check runs without sourcing this repo's install/setup.bash. I independently re-ran all five scheduler acceptance checks (build, unit, task-frames-smoke, legacy-sweep-smoke, cli-smoke) and all passed, and confirmed the diff is exa
…[96 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task MOT-04.6.R1): check unit fails after upstream change.

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

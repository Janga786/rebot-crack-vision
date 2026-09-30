# MOT-04.6 — Reachability on task frames (tool_tip / camera_link) with a workcell-faithful specimen proxy + camera view check

**Accepted:** 2026-09-30 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`7771477`](https://github.com/Janga786/rebot-crack-vision/commit/7771477296d44739140613a356d67aea89b21512)

## What was done

Implemented interactively by the technical-lead recovery session (Claude Opus 5.5, operator-authorised, 2026-09-30); commit 54e522e. Fixes MOT-04.5's frame error (standoffs meant as tool-tip clearances were applied to gripper_tcp) and the slab-under-base artefact without weakening any collision check. Coarse probe (scratch): 599/910 reachable, feasible 0.20 m placement at (0.29, 0), 200/200 half-step verification.

_This step was a physical procedure performed by the operator; the evidence files below were recorded during it and then independently audited._

Files changed:
- [config/motion/reachability.yaml](https://github.com/Janga786/rebot-crack-vision/blob/7771477296d44739140613a356d67aea89b21512/config/motion/reachability.yaml) (+27 / −17)
- [ros2_ws/src/crackvision_motion/crackvision_motion/placement.py](https://github.com/Janga786/rebot-crack-vision/blob/7771477296d44739140613a356d67aea89b21512/ros2_ws/src/crackvision_motion/crackvision_motion/placement.py) (+85 / −5)
- [ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py](https://github.com/Janga786/rebot-crack-vision/blob/7771477296d44739140613a356d67aea89b21512/ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py) (+93 / −4)
- [ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py](https://github.com/Janga786/rebot-crack-vision/blob/7771477296d44739140613a356d67aea89b21512/ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py) (+71 / −21)
- [ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py](https://github.com/Janga786/rebot-crack-vision/blob/7771477296d44739140613a356d67aea89b21512/ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py) (+14 / −1)
- [ros2_ws/src/crackvision_motion/test/fixtures/reachability_task_frames_smoke.yaml](https://github.com/Janga786/rebot-crack-vision/blob/7771477296d44739140613a356d67aea89b21512/ros2_ws/src/crackvision_motion/test/fixtures/reachability_task_frames_smoke.yaml) (+36 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/reachability_view_smoke.yaml](https://github.com/Janga786/rebot-crack-vision/blob/7771477296d44739140613a356d67aea89b21512/ros2_ws/src/crackvision_motion/test/fixtures/reachability_view_smoke.yaml) (+33 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/scene_table.yaml](https://github.com/Janga786/rebot-crack-vision/blob/7771477296d44739140613a356d67aea89b21512/ros2_ws/src/crackvision_motion/test/fixtures/scene_table.yaml) (+21 / −0)
- [ros2_ws/src/crackvision_motion/test/test_placement.py](https://github.com/Janga786/rebot-crack-vision/blob/7771477296d44739140613a356d67aea89b21512/ros2_ws/src/crackvision_motion/test/test_placement.py) (+43 / −4)
- [ros2_ws/src/crackvision_motion/test/test_reachability_core.py](https://github.com/Janga786/rebot-crack-vision/blob/7771477296d44739140613a356d67aea89b21512/ros2_ws/src/crackvision_motion/test/test_reachability_core.py) (+92 / −4)
- [scripts/ros/test_reachability_task_frames.sh](https://github.com/Janga786/rebot-crack-vision/blob/7771477296d44739140613a356d67aea89b21512/scripts/ros/test_reachability_task_frames.sh) (+57 / −0)

Commits: [`54e522e`](https://github.com/Janga786/rebot-crack-vision/commit/54e522e871b235ae0625ecaf49adc57cd292d336)

## Why it was done

The sweep targets the physical task frame (tool_tip for tool clearances, camera_link for viewing distance) against self-collision incl. the camera, the workcell table and a specimen block that never overlaps the robot base; recommend_placement also emits a wrist-camera view check; the committed config uses this model.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **1/16** tasks accepted; whole project: **25/77**.
- REQ-MOT-1: 2/13 contributing tasks done
- REQ-MOT-2: 1/10 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, task-frames-smoke ✔, legacy-sweep-smoke ✔, cli-smoke ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “MOT-04.6 (commit 54e522e, reviewed at 7771477) does what the card asks. I re-ran the unit tests (124 passed) and the live task-frames smoke test (exit 0). With the specimen block and the table present, tool_tip is reachable at both 0.01 and 0.04 m clearance at x=0.23 for all 6 near targets, the 6 far targets are unreachable, and the camera_link view pose is reachable. I also ran recommend_placement with --emit-view-config: it writes a valid view config with camera_link, a 0.25 m standoff, tilt {0, 15} deg and a proxy margin of 0.12 m (0.20/2 + 0.02). The caveats start with 
…[726 chars clipped]”

## What it unlocks next

- Closer: MOT-04.5 — Run the full reachability sweep, recommend + independently verify the specimen placement, document (still needs MOT-04.3, MOT-04.4, MOT-03)

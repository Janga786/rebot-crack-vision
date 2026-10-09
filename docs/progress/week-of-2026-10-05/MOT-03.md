# MOT-03 — Planning scene from config (nominal until measured)

**Accepted:** 2026-10-09 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`5f00b13`](https://github.com/Janga786/rebot-crack-vision/commit/5f00b1364c5d12304b3c671d3c8aba866b93c524)

## What was done

Corrected the MOT-03 collision scene per the 2026-09-30 re-spec: removed the world-fixed camera_mount object/ACM entry (the D405 and mount are eye-in-hand robot links from GEOM-10, already in every planning request), moved the table top face to exactly z=0 in base_link with a widened footprint covering the base and the reachability grid, added an explicitly-labelled placeholder specimen pose/height (specimen_placement.yaml is currently infeasible), and committed scripts/ros/test_scene.sh (now in scope) exercising the full scene-apply/assert/collision-rejection smoke test.

Files changed:
- [config/scene/scene.yaml](https://github.com/Janga786/rebot-crack-vision/blob/5f00b1364c5d12304b3c671d3c8aba866b93c524/config/scene/scene.yaml) (+107 / −40)
- [docs/motion/SCENE.md](https://github.com/Janga786/rebot-crack-vision/blob/5f00b1364c5d12304b3c671d3c8aba866b93c524/docs/motion/SCENE.md) (+266 / −59)
- [ros2_ws/src/crackvision_motion/crackvision_motion/assert_scene_objects.py](https://github.com/Janga786/rebot-crack-vision/blob/5f00b1364c5d12304b3c671d3c8aba866b93c524/ros2_ws/src/crackvision_motion/crackvision_motion/assert_scene_objects.py) (+65 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/scene_apply.py](https://github.com/Janga786/rebot-crack-vision/blob/5f00b1364c5d12304b3c671d3c8aba866b93c524/ros2_ws/src/crackvision_motion/crackvision_motion/scene_apply.py) (+178 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py](https://github.com/Janga786/rebot-crack-vision/blob/5f00b1364c5d12304b3c671d3c8aba866b93c524/ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py) (+192 / −0)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/5f00b1364c5d12304b3c671d3c8aba866b93c524/ros2_ws/src/crackvision_motion/setup.py) (+2 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/scene_collision_smoke.yaml](https://github.com/Janga786/rebot-crack-vision/blob/5f00b1364c5d12304b3c671d3c8aba866b93c524/ros2_ws/src/crackvision_motion/test/fixtures/scene_collision_smoke.yaml) (+26 / −0)
- [ros2_ws/src/crackvision_motion/test/test_scene_core.py](https://github.com/Janga786/rebot-crack-vision/blob/5f00b1364c5d12304b3c671d3c8aba866b93c524/ros2_ws/src/crackvision_motion/test/test_scene_core.py) (+244 / −1)
- [scripts/ros/test_scene.sh](https://github.com/Janga786/rebot-crack-vision/blob/5f00b1364c5d12304b3c671d3c8aba866b93c524/scripts/ros/test_scene.sh) (+50 / −0)

Commits: [`5e84657`](https://github.com/Janga786/rebot-crack-vision/commit/5e8465738c9fac05926189ae5b8cb907c20a8aad), [`5f00b13`](https://github.com/Janga786/rebot-crack-vision/commit/5f00b1364c5d12304b3c671d3c8aba866b93c524)

## Why it was done

Collision scene (table, specimen, camera mount) loaded from config; UNMEASURED values block commissioning.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene

## How it moves the project forward

- Motion planning: **7/16** tasks accepted; whole project: **54/93**.
- REQ-MOT-1: 5/13 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, scene-test ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “MOT-03 passes on every criterion and I found nothing that needs fixing. The scene is defined in config/scene/scene.yaml. It contains two objects, table and specimen, and two allowed-collision (ACM) entries, table~base_link and specimen~table. Every object and entry is tagged nominal and has a source. The table's top face is at exactly z=0 and it covers the robot base, the reachability grid and the margin. The specimen sits on the table and is clearly labelled a placeholder, because specimen_placement.yaml currently has no feasible placement (feasible: false). The old world 
…[225 chars clipped]”

## What it unlocks next

- **Ready to start:** MOT-04.5 — Run the full reachability sweep, recommend + independently verify the specimen placement, document
- **Can now be planned in detail:** MOT-10 — Measure the workcell geometry (operator)
- Closer: MOT-05 — Commissioning-gated execution interface (mock/dry/real) (still needs MOT-02)
- Closer: MOT-06 — Cartesian path planning along crack paths (still needs MOT-04, MOT-02)

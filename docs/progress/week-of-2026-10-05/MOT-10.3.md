# MOT-10.3 — survey_to_scene CLI: survey → scene.yaml (+ verify configs at the measured pose), drift check; decouple scene tests from the production scene's nominal state

**Accepted:** 2026-10-09 · **Work package:** Motion planning (L-MOTION) · **Track:** software · **Verified commit:** [`c8c7692`](https://github.com/Janga786/rebot-crack-vision/commit/c8c76925aa127a06177bc0503de1c4e633d4a720)

## What was done

Built the survey_to_scene tool that turns a physically-measured workcell survey into the robot's collision-safety scene file, replacing engineering guesses with real tape-measure/caliper numbers. It can also generate two extra check files that verify the robot can still reach and see the specimen at its actual measured position, and it can flag if a saved scene file no longer matches a re-measurement. All 7 required checks passed, including a live run that converted a sample survey into a fully 'measured' scene and confirmed it would pass the safety gate before any real robot motion.

Files changed:
- [docs/motion/SCENE.md](https://github.com/Janga786/rebot-crack-vision/blob/c8c76925aa127a06177bc0503de1c4e633d4a720/docs/motion/SCENE.md) (+68 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/survey_to_scene.py](https://github.com/Janga786/rebot-crack-vision/blob/c8c76925aa127a06177bc0503de1c4e633d4a720/ros2_ws/src/crackvision_motion/crackvision_motion/survey_to_scene.py) (+243 / −0)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/c8c76925aa127a06177bc0503de1c4e633d4a720/ros2_ws/src/crackvision_motion/setup.py) (+1 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/scene_nominal_example.yaml](https://github.com/Janga786/rebot-crack-vision/blob/c8c76925aa127a06177bc0503de1c4e633d4a720/ros2_ws/src/crackvision_motion/test/fixtures/scene_nominal_example.yaml) (+73 / −0)
- [ros2_ws/src/crackvision_motion/test/test_scene_core.py](https://github.com/Janga786/rebot-crack-vision/blob/c8c76925aa127a06177bc0503de1c4e633d4a720/ros2_ws/src/crackvision_motion/test/test_scene_core.py) (+15 / −3)

Commits: [`c8c7692`](https://github.com/Janga786/rebot-crack-vision/commit/c8c76925aa127a06177bc0503de1c4e633d4a720)

## Why it was done

`ros2 run crackvision_motion survey_to_scene` validates a survey and writes a measured crackvision.scene_config/1 file. It can also emit placement/view verification reachability configs centred on the measured specimen pose, and it can check that a committed scene.yaml still matches its survey. test_scene_core.py no longer depends on the production scene being nominal.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-OPS-2**: Reproducible install/config/tests/evaluation results, handoff docs, requirements→evidence map, honest readiness report

## How it moves the project forward

- Motion planning: **7/27** tasks accepted; whole project: **53/104**.
- REQ-MOT-1: 6/24 contributing tasks done
- REQ-OPS-2: 6/8 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, validate-fixture ✔, template-refused ✔, convert-and-check ✔, drift-detected ✔, dry-run-writes-nothing ✔, unit ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “MOT-10.3's survey_to_scene CLI was independently re-verified: all 7 scheduler acceptance checks reproduce exactly (validate/refuse/convert/check-drift/dry-run/unit tests), the emitted verify/view configs correctly reuse placement.verification_config/view_verification_config unchanged while preserving ik_link, standoffs, orientation, IK tolerance and collision settings, the scene header/byte-determinism and never-overwrite-scene.yaml-by-default behavior hold, the test fixture decoupling from the production scene is correctly implemented, and SCENE.md §4 is untouched. No defects found.”

## What it unlocks next

- Closer: MOT-10.4 — Operator: place the specimen at the recommended pose and perform the workcell survey (no motion) (still needs MOT-10.1)
- Closer: MOT-10.5 — Apply the measured survey to config/scene/scene.yaml and re-run the scene + reachability verification at the as-placed specimen (still needs MOT-10.4)

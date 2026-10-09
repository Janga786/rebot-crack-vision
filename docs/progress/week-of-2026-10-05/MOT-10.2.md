# MOT-10.2 — Workcell survey core: validate a crackvision.workcell_survey/1 record and derive measured scene objects (pure Python)

**Accepted:** 2026-10-09 · **Work package:** Motion planning (L-MOTION) · **Track:** software · **Verified commit:** [`c6a290a`](https://github.com/Janga786/rebot-crack-vision/commit/c6a290a5449658ca5a16cfba81eb6e3734f707a9)

## What was done

Added the workcell-survey validator/derivation module (survey_core.py) that turns a raw tape-measure/caliper survey file into MoveIt-ready collision objects (table, specimen, obstacles) with propagated measurement uncertainty, enforcing every refusal rule from the survey spec (non-rectangular corners, specimen not resting on the table, obstacles overlapping the robot's base, incomplete readings). Verified with 26 passing unit tests against a labelled synthetic example survey, including a hand-checked worked example accurate to better than a micrometer (1e-9 m).

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/survey_core.py](https://github.com/Janga786/rebot-crack-vision/blob/c6a290a5449658ca5a16cfba81eb6e3734f707a9/ros2_ws/src/crackvision_motion/crackvision_motion/survey_core.py) (+610 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/workcell_survey_example.yaml](https://github.com/Janga786/rebot-crack-vision/blob/c6a290a5449658ca5a16cfba81eb6e3734f707a9/ros2_ws/src/crackvision_motion/test/fixtures/workcell_survey_example.yaml) (+71 / −0)
- [ros2_ws/src/crackvision_motion/test/test_survey_core.py](https://github.com/Janga786/rebot-crack-vision/blob/c6a290a5449658ca5a16cfba81eb6e3734f707a9/ros2_ws/src/crackvision_motion/test/test_survey_core.py) (+346 / −0)

Commits: [`c6a290a`](https://github.com/Janga786/rebot-crack-vision/commit/c6a290a5449658ca5a16cfba81eb6e3734f707a9)

## Why it was done

crackvision_motion/survey_core.py loads and validates a survey record, applies §12's derivation and refusal rules, and returns crackvision.scene_config/1 objects plus ACM entries, all tagged measured with survey-sha sources. Unit tests cover it against a worked fixture.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene

## How it moves the project forward

- Motion planning: **12/27** tasks accepted; whole project: **59/104**.
- REQ-MOT-1: 10/24 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, no-ros-import ✔, scene-core-still-green ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “survey_core.py correctly implements §12's validation and derivation rules, matches the spec's formulas exactly (verified by hand-tracing rule 1/3/4 against the code), its derived output round-trips through scene_core.load_config's validation, and all 26 tests plus the two scheduler acceptance checks pass. Scope is respected (only the 3 in-scope files touched, scene_core.py untouched and still green), and no ROS imports are present.”

## What it unlocks next

- **Ready to start:** MOT-10.3 — survey_to_scene CLI: survey → scene.yaml (+ verify configs at the measured pose), drift check; decouple scene tests from the production scene's nominal state

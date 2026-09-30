# GEOM-08.2 — Pure-numpy B601-DM forward kinematics + end-effector transforms (crackvision.kinematics)

**Accepted:** 2026-09-30 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`0d7b11c`](https://github.com/Janga786/rebot-crack-vision/commit/0d7b11c381431e3ba199a2db10824e29d8f33d0e)

## What was done

Added a pure-Python (no ROS) forward-kinematics module for the B601-DM arm that computes the base-to-gripper transform straight from the committed URDF, plus a loader for the wrist camera and tool-tip offsets and their calibration uncertainties from end_effector.yaml. All 10 new tests pass, including a cross-check against the real vendor URDF in ~/rebot_ws and a match to the yaml's documented camera optical-axis direction and tool-tip distance; the full project test suite (281 tests) still passes.

After the first audit, a repair round (GEOM-08.2.R1) fixed the reported issues: Fixed a calibration bug where a measured camera/tool offset with no documented source for its uncertainty was silently accepted with a fake "measured" placeholder instead of being rejected. Added tests confirming the loader now errors out in that case for both the tool and wrist camera. Full test suite (282 tests) still passes.

Files changed:
- [src/crackvision/kinematics.py](https://github.com/Janga786/rebot-crack-vision/blob/0d7b11c381431e3ba199a2db10824e29d8f33d0e/src/crackvision/kinematics.py) (+300 / −1)
- [tests/test_kinematics.py](https://github.com/Janga786/rebot-crack-vision/blob/0d7b11c381431e3ba199a2db10824e29d8f33d0e/tests/test_kinematics.py) (+234 / −0)

Commits: [`9ef2422`](https://github.com/Janga786/rebot-crack-vision/commit/9ef24225a1a4d9329f6d8471656beeaba5a4911d), [`0d7b11c`](https://github.com/Janga786/rebot-crack-vision/commit/0d7b11c381431e3ba199a2db10824e29d8f33d0e)

## Why it was done

`crackvision.kinematics` computes T_base_link_gripper_link = FK(q) from the committed URDF copy, and loads T_gripper_link_camera_link / T_gripper_link_tool_tip, their value_status and §10 calibration sigmas, and the file sha256 from end_effector.yaml. It also exposes the nominal D405 camera_link→optical transform. No ROS import.

- Requirement **REQ-GEOM-2**: Camera-to-robot calibration with explicit transform conventions and residuals

## How it moves the project forward

- Geometry & calibration: **8/24** tasks accepted; whole project: **41/92**.
- REQ-GEOM-2: 5/18 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, no-ros-import ✔, suite ✔); independent audit accepted it (criteria: 2 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this change. `_load_block_sigma` now raises `KinematicsError` when a measured block's `uncertainty.source` is missing or empty. It no longer fills in the placeholder "measured". The new test covers both the tool and wrist_camera blocks, each with a missing and an empty source. I checked the tests with my own script. With a source present, the file loads and `sigma_source` is `{'wrist_camera': 'c', 'tool': 't'}`. Removing or emptying the source in either block raises the new error. The unit tests pass (11), and the scheduler's full suite passed (282 passed, 3 s
…[147 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task GEOM-08.2.R1): Measured uncertainty without `source` is accepted and a placeholder source is substituted.

## What it unlocks next

- **Ready to start:** GEOM-08.3 — FK parity: crackvision.kinematics vs live MoveIt /compute_fk (gripper_link, tool_tip, camera_link) on the mock stack
- **Ready to start:** GEOM-08.4 — Capture record library + assemble/validate CLI (crackvision.capture_record, INTERFACES §9)
- Closer: GEOM-08.5 — Lift ordered pixel polylines to base_link surface points with normals, gap policy and per-point covariance (crackvision.lift3d) (still needs GEOM-08.4)

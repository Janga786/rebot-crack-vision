# MOT-05.3 — Offline execution gate: commissioning/approval/execution-config loaders + full gate report (pure Python)

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`43699f1`](https://github.com/Janga786/rebot-crack-vision/commit/43699f1f651bd1c5d5769da52be0c752d828e711)

## What was done

Built the offline safety gate that must pass before the robot arm's trajectory executor is allowed to run in real mode: it checks trajectory validity, joint-limit compliance, speed caps, scene/end-effector calibration status, hardware commissioning and e-stop verification, operator approval, and 3D-path eligibility, all without needing ROS running. Verified with a synthetic "everything measured and commissioned" fixture that real mode correctly passes, and that corrupting any single input (wrong limits hash, stale approval, ineligible path, too-fast speed, test-only trajectory, or unmeasured scene) causes exactly that one check to fail — 67 tests pass, with no regressions in the two safety modules it depends on.

Files changed:
- [config/motion/execution.yaml](https://github.com/Janga786/rebot-crack-vision/blob/43699f1f651bd1c5d5769da52be0c752d828e711/config/motion/execution.yaml) (+86 / −0)
- [config/robot/commissioning.yaml](https://github.com/Janga786/rebot-crack-vision/blob/43699f1f651bd1c5d5769da52be0c752d828e711/config/robot/commissioning.yaml) (+36 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py](https://github.com/Janga786/rebot-crack-vision/blob/43699f1f651bd1c5d5769da52be0c752d828e711/ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py) (+874 / −91)
- [ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/commissioning_ready.yaml](https://github.com/Janga786/rebot-crack-vision/blob/43699f1f651bd1c5d5769da52be0c752d828e711/ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/commissioning_ready.yaml) (+16 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/end_effector_measured.yaml](https://github.com/Janga786/rebot-crack-vision/blob/43699f1f651bd1c5d5769da52be0c752d828e711/ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/end_effector_measured.yaml) (+36 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/paths3d_eligible.json](https://github.com/Janga786/rebot-crack-vision/blob/43699f1f651bd1c5d5769da52be0c752d828e711/ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/paths3d_eligible.json) (+9 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/scene_measured.yaml](https://github.com/Janga786/rebot-crack-vision/blob/43699f1f651bd1c5d5769da52be0c752d828e711/ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/scene_measured.yaml) (+36 / −0)
- [ros2_ws/src/crackvision_motion/test/test_execution_gate.py](https://github.com/Janga786/rebot-crack-vision/blob/43699f1f651bd1c5d5769da52be0c752d828e711/ros2_ws/src/crackvision_motion/test/test_execution_gate.py) (+736 / −11)

Commits: [`40e6ba8`](https://github.com/Janga786/rebot-crack-vision/commit/40e6ba8a622cf7e003993235ae9fa422e3614e69), [`e9528a1`](https://github.com/Janga786/rebot-crack-vision/commit/e9528a197c0d709054e76736ccc596b3cd1d1a99), [`4dd9f5a`](https://github.com/Janga786/rebot-crack-vision/commit/4dd9f5a3098332f119abf41e8c8d9152c180e302), [`8c96fef`](https://github.com/Janga786/rebot-crack-vision/commit/8c96fefa2d7568798dbc0ac2769facf56d9f49ed), [`7338dbc`](https://github.com/Janga786/rebot-crack-vision/commit/7338dbc4a771a49ef26011addea891d99802ae6c)

## Why it was done

crackvision_motion/execution_gate.py evaluates every §11 offline gate check for a mode and returns a structured GateReport. It calls scene_core.assert_commissioning_ready and end_effector.assert_commissioning_ready, checks the limits hash, approval binding, paths3d eligibility, speed cap and arming env, and provides the confirmation-phrase helpers. It ships config/robot/commissioning.yaml (uncommissioned) and config/motion/execution.yaml. Tests prove that the repo defaults refuse real mode with the expected reasons.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **46/57** tasks accepted; whole project: **108/144**.
- REQ-MOT-1: 35/41 contributing tasks done
- REQ-INT-2: 10/16 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, upstream-gates-regression ✔, shipped-uncommissioned ✔, pure-python ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Re-verification of MOT-05.3 after upstream MOT-07.10.2(.R1). Since this card's last repair (7338dbc), execution_gate.py has changed in only one place: commit 24ac771 adds an `except UnicodeDecodeError` branch to `_evaluate_eligible_view_move` (execution_gate.py:741-743). Before the change, that error was already caught as a ValueError and reported as 'error'. It is now reported as 'fail'. Both outcomes refuse, so nothing escapes evaluate_offline. Since 7338dbc there are no changes to docs/INTERFACES.md, the shipped configs, scene_core.py or end_effector.py. I re-ran all fou
…[574 chars clipped]”
- Quality loop: the audit found 6 issue(s) that were fixed before acceptance: evaluate_offline and the strict loaders still raise TypeError (and UnicodeDecodeError) on malformed config/input files instead of GateConfigError / an error gate; evaluate_offline raises on bad inputs and out-of-range speed_scale instead of returning failed checks; dry-mode G-SPEED warn cell (s > commissioning speed_scale_cap, or unreadable record) is reported as pass; G-ELIGIBLE lets UnicodeDecodeError escape evaluate_offline for a non-UTF-8 paths3d file; view_move G-ELIGIBLE lets UnicodeDecodeError / YAML errors escape evaluate_offline.

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

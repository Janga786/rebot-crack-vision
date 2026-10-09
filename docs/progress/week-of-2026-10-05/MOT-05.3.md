# MOT-05.3 — Offline execution gate: commissioning/approval/execution-config loaders + full gate report (pure Python)

**Accepted:** 2026-10-09 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`8c96fef`](https://github.com/Janga786/rebot-crack-vision/commit/8c96fefa2d7568798dbc0ac2769facf56d9f49ed)

## What was done

Built the offline safety gate that must pass before the robot arm's trajectory executor is allowed to run in real mode: it checks trajectory validity, joint-limit compliance, speed caps, scene/end-effector calibration status, hardware commissioning and e-stop verification, operator approval, and 3D-path eligibility, all without needing ROS running. Verified with a synthetic "everything measured and commissioned" fixture that real mode correctly passes, and that corrupting any single input (wrong limits hash, stale approval, ineligible path, too-fast speed, test-only trajectory, or unmeasured scene) causes exactly that one check to fail — 67 tests pass, with no regressions in the two safety modules it depends on.

After the first audit, a repair round (MOT-05.3.R1) fixed the reported issues: The offline execution gate now returns a structured refusal for every bad input I tried, including a bound crack-path file that is not text, instead of crashing. A scratch probe ran 132 evaluations with bad or missing files and out-of-range speed scales across mock, dry and real modes, and none raised an exception. The gate and trajectory test suites pass (99 tests), as do the upstream scene and end-effector suites (45 tests).

Files changed:
- [config/motion/execution.yaml](https://github.com/Janga786/rebot-crack-vision/blob/8c96fefa2d7568798dbc0ac2769facf56d9f49ed/config/motion/execution.yaml) (+86 / −0)
- [config/robot/commissioning.yaml](https://github.com/Janga786/rebot-crack-vision/blob/8c96fefa2d7568798dbc0ac2769facf56d9f49ed/config/robot/commissioning.yaml) (+36 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py](https://github.com/Janga786/rebot-crack-vision/blob/8c96fefa2d7568798dbc0ac2769facf56d9f49ed/ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py) (+872 / −91)
- [ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/commissioning_ready.yaml](https://github.com/Janga786/rebot-crack-vision/blob/8c96fefa2d7568798dbc0ac2769facf56d9f49ed/ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/commissioning_ready.yaml) (+16 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/end_effector_measured.yaml](https://github.com/Janga786/rebot-crack-vision/blob/8c96fefa2d7568798dbc0ac2769facf56d9f49ed/ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/end_effector_measured.yaml) (+36 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/paths3d_eligible.json](https://github.com/Janga786/rebot-crack-vision/blob/8c96fefa2d7568798dbc0ac2769facf56d9f49ed/ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/paths3d_eligible.json) (+9 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/scene_measured.yaml](https://github.com/Janga786/rebot-crack-vision/blob/8c96fefa2d7568798dbc0ac2769facf56d9f49ed/ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/scene_measured.yaml) (+36 / −0)
- [ros2_ws/src/crackvision_motion/test/test_execution_gate.py](https://github.com/Janga786/rebot-crack-vision/blob/8c96fefa2d7568798dbc0ac2769facf56d9f49ed/ros2_ws/src/crackvision_motion/test/test_execution_gate.py) (+704 / −11)

Commits: [`40e6ba8`](https://github.com/Janga786/rebot-crack-vision/commit/40e6ba8a622cf7e003993235ae9fa422e3614e69), [`e9528a1`](https://github.com/Janga786/rebot-crack-vision/commit/e9528a197c0d709054e76736ccc596b3cd1d1a99), [`4dd9f5a`](https://github.com/Janga786/rebot-crack-vision/commit/4dd9f5a3098332f119abf41e8c8d9152c180e302), [`8c96fef`](https://github.com/Janga786/rebot-crack-vision/commit/8c96fefa2d7568798dbc0ac2769facf56d9f49ed)

## Why it was done

crackvision_motion/execution_gate.py evaluates every §11 offline gate check for a mode and returns a structured GateReport. It calls scene_core.assert_commissioning_ready and end_effector.assert_commissioning_ready, checks the limits hash, approval binding, paths3d eligibility, speed cap and arming env, and provides the confirmation-phrase helpers. It ships config/robot/commissioning.yaml (uncommissioned) and config/motion/execution.yaml. Tests prove that the repo defaults refuse real mode with the expected reasons.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **11/27** tasks accepted; whole project: **58/104**.
- REQ-MOT-1: 9/24 contributing tasks done
- REQ-INT-2: 4/13 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, upstream-gates-regression ✔, shipped-uncommissioned ✔, pure-python ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “This repair attempt closes the one remaining gap from the prior review (a non-UTF-8 paths3d file bypassing the JSON-decode guard and raising UnicodeDecodeError) while leaving the earlier Finding 1/Finding 2 fixes untouched. I independently reproduced all four scheduler acceptance checks (99+45 tests passing, shipped-uncommissioned and pure-python grep checks), read through execution_gate.py end to end and cross-checked every gate's outcome logic against the §11.7 table row by row, and ran my own adversarial probe (dir-as-file, permission-denied files, empty/list-typed YAML,
…[286 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task MOT-05.3.R1): evaluate_offline raises on bad inputs and out-of-range speed_scale instead of returning failed checks; §11.7 warn/error outcomes folded into pass/fail; to_dict is not the §11.9 gate_report shape.

## What it unlocks next

- **Ready to start:** MOT-05.4 — execute_trajectory ROS node/CLI: online read-only checks, mock/dry/real execution, typed confirmation, e-stop + tracking monitor, execution record

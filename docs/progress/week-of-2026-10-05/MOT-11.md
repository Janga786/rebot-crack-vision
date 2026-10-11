# MOT-11 — Vendor-driver readiness check: verify reBotArmController's port, motor IDs, joint order, direction and gripper mapping against the 2026-10-10 bring-up evidence (read-only ~/rebot_ws)

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`c1363ae`](https://github.com/Janga786/rebot-crack-vision/commit/c1363ae1392fe53a7b58466721a9ed93baa4fab1)

## What was done

Implemented scripts/motion/check_vendor_driver.py, a pure read-only static checker verifying reBotArmController's port, motor IDs, joint order, direction, zero, gripper mapping, safe_park, SDK API, Python deps, QoS, profile, arbitration and URDF choice against the 2026-10-10 bring-up facts and this repo's limits/execution config, plus tests and an operator-facing doc.

After the first audit, a repair round (MOT-11.R1) fixed the reported issues: The vendor-driver checker's SDK-API check again names commit 8427281 as the last known SDK commit exporting RobotArm/ArmEndPos, both in its output and in the docs. All 14 unit tests pass from a copy containing only tracked files. The live run cites a file:line for every check and exits with code 3 ("not ready") because the SDK API, the safe-park check and the Python dependencies currently fail.

Files changed:
- [docs/motion/VENDOR_DRIVER.md](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/docs/motion/VENDOR_DRIVER.md) (+83 / −17)
- [scripts/motion/check_vendor_driver.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/scripts/motion/check_vendor_driver.py) (+1137 / −64)
- [tests/fixtures/vendor_driver/good/deps/motorbridge/__init__.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/deps/motorbridge/__init__.py) (+0 / −0)
- [tests/fixtures/vendor_driver/good/deps/pinocchio/__init__.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/deps/pinocchio/__init__.py) (+0 / −0)
- [tests/fixtures/vendor_driver/good/execution.yaml](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/execution.yaml) (+6 / −0)
- [tests/fixtures/vendor_driver/good/executor.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/executor.py) (+6 / −0)
- [tests/fixtures/vendor_driver/good/facts.yaml](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/facts.yaml) (+46 / −0)
- [tests/fixtures/vendor_driver/good/limits.yaml](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/limits.yaml) (+9 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarm_bringup/config/arm.yaml](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarm_bringup/config/arm.yaml) (+63 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarm_bringup/config/driver_params.yaml](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarm_bringup/config/driver_params.yaml) (+6 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarm_bringup/config/gripper.yaml](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarm_bringup/config/gripper.yaml) (+12 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarm_bringup/config/safe_park.yaml](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarm_bringup/config/safe_park.yaml) (+14 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarm_bringup/launch/bringup.launch.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarm_bringup/launch/bringup.launch.py) (+3 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py) (+28 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py) (+6 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py) (+7 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py) (+17 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/ros_services.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/ros_services.py) (+2 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/third_party/reBotArm_control_py/reBotArm_control_py/actuator/__init__.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/third_party/reBotArm_control_py/reBotArm_control_py/actuator/__init__.py) (+3 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/third_party/reBotArm_control_py/reBotArm_control_py/actuator/gripper.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/third_party/reBotArm_control_py/reBotArm_control_py/actuator/gripper.py) (+2 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/third_party/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/third_party/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py) (+3 / −0)
- [tests/fixtures/vendor_driver/good/rebot_ws/third_party/reBotArm_control_py/reBotArm_control_py/controllers/__init__.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/fixtures/vendor_driver/good/rebot_ws/third_party/reBotArm_control_py/reBotArm_control_py/controllers/__init__.py) (+3 / −0)
- [tests/test_check_vendor_driver.py](https://github.com/Janga786/rebot-crack-vision/blob/c1363ae1392fe53a7b58466721a9ed93baa4fab1/tests/test_check_vendor_driver.py) (+209 / −0)

Commits: [`d52b2c5`](https://github.com/Janga786/rebot-crack-vision/commit/d52b2c59929da58767245a121638d8ceec20c622), [`0246d3c`](https://github.com/Janga786/rebot-crack-vision/commit/0246d3cc143916dcfbded5484fad010e60a56e33), [`c1363ae`](https://github.com/Janga786/rebot-crack-vision/commit/c1363ae1392fe53a7b58466721a9ed93baa4fab1)

## Why it was done

A read-only checker, scripts/motion/check_vendor_driver.py, with tests and an operator note, docs/motion/VENDOR_DRIVER.md. Before any real run it proves that the vendor driver's port, IDs, joint order, direction and gripper mapping (open = negative) match the operator's 2026-10-10 evidence. It also flags the vendor-stack defects that must be fixed first.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **19/29** tasks accepted; whole project: **72/108**.
- REQ-MOT-1: 17/26 contributing tasks done
- REQ-INT-2: 7/15 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, live-report ✔, read-only ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Both blocking/major findings from the prior review are resolved and verified independently: a clean `git archive` checkout now carries tracked placeholder fixture deps and passes 14/14 unit tests, and the live report against the real ~/rebot_ws shows every check carrying a citation with real, content-computed line numbers (spot-checked against the actual source file and confirmed exact). The reinstated 8427281 SDK-commit note (finding edf2d08bea06ba28) is present in V-SDK-API's citation, remedy, and the VENDOR_DRIVER.md row. All three scheduler acceptance checks pass, and t
…[84 chars clipped]”
- Quality loop: the audit found 3 issue(s) that were fixed before acceptance (repair task MOT-11.R1): Most checks lack file:line citations into ~/rebot_ws; card-specified line citations missing; Unit tests depend on untracked empty fixture directories; 10/13 fail from a clean checkout; Repair dropped the required '8427281 is the last commit exporting RobotArm/ArmEndPos' note from V-SDK-API and the doc.

## What it unlocks next

- Closer: GEOM-05 — Execute hand-eye calibration on hardware (operator) (still needs GEOM-04, CAM-01, MOT-09, GEOM-04)
- Closer: GEOM-07 — Execute TCP calibration + physical boresight verification (operator) (still needs MOT-09)
- Closer: MOT-05.6 — Operator execution runbook (mock → dry → real) + ROS_WORKSPACE.md entry (still needs MOT-05.5)
- Closer: MOT-09 — Arm bring-up commissioning (operator) (still needs MOT-05)

# MOT-11 — Vendor-driver readiness check: verify reBotArmController's port, motor IDs, joint order, direction and gripper mapping against the 2026-10-10 bring-up evidence (read-only ~/rebot_ws)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-01"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "scripts/motion/check_vendor_driver.py",
      "tests/test_check_vendor_driver.py",
      "tests/fixtures/vendor_driver/**",
      "docs/motion/VENDOR_DRIVER.md"
    ]
  },
  "inputs": [
    "evidence/operator/MOT-09/bringup_2026-10-10_facts.yaml",
    "evidence/operator/MOT-09/bringup_2026-10-10_summary.md",
    "config/robot/b601_dm_limits.yaml",
    "config/motion/execution.yaml",
    "docs/adr/016-commissioning-gated-execution.md",
    "docs/INTERFACES.md#0",
    "docs/motion/ROBOT_MODEL.md",
    "scripts/motion/compare_urdf.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_check_vendor_driver.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "live-report",
        "cmd": "bash -c './env.sh python scripts/motion/check_vendor_driver.py --rebot-ws \"$HOME/rebot_ws\" --facts evidence/operator/MOT-09/bringup_2026-10-10_facts.yaml --format json > /dev/null; rc=$?; echo rc=$rc; test $rc -eq 0 -o $rc -eq 3'",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "read-only",
        "cmd": "bash -c '! grep -nE \"import (motorbridge|serial|rclpy|subprocess)|from (motorbridge|serial|rclpy|subprocess) |os\\.system|Popen|rmtree|\\.unlink\\(|os\\.remove|shutil\\.(move|copy)\" scripts/motion/check_vendor_driver.py'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "check_vendor_driver.py is a pure, read-only static checker. It reads vendor files under --rebot-ws (default ~/rebot_ws) and this repo's files, imports no motorbridge/serial/rclpy/subprocess, starts no process and opens no device. It writes only the §0.4 logs under logs/ and an explicit --json-out PATH, and never anything under --rebot-ws. A unit test runs it against a fixture vendor tree copied to a tmp dir and asserts that every file hash in that tree is unchanged afterwards. It follows INTERFACES §0: common flags, exit 0 = no FAIL, 3 = at least one FAIL (vendor stack not ready), 2 = usage error or missing input file, 1 = unexpected error.",
      "It reports each check below as PASS, FAIL, WARN or INFO with a stable id, a file:line citation into ~/rebot_ws and the compared values, as a text table, and as JSON with --format json. Every FAIL and WARN names its remedy. Checks: V-PORT (arm.yaml/gripper.yaml/driver_params channel versus facts.bus.port; any configured channel must start with /dev/tty because both SDK versions pick dm-serial only then, rebotarm.py:485-487 and arm.py@8427281:119-121; WARN if /dev/rebot_arm exists, because deploy/_env.sh:59 would prefer it), V-BAUD (the SDK's hard-coded 921600 versus facts), V-IDS (motor_id/feedback_id/model per joint and gripper versus facts), V-ORDER (arm joints are exactly joint1..joint6 in order, matching INTERFACES §11.2 and the URDF), V-DIRECTION (no sign/direction/offset/invert keys in vendor joint configs and no sign flip in the SDK position path, plus every facts sign_check verdict 'match'), V-ZERO (no zero offsets in vendor configs; facts zero consistent with the URDF home), V-GRIPPER-SIGN (vendor open positions negative and closed 0.0 in ros_services.py:16-17 and ros_publishers.py:7-8, matching facts.joints.gripper.open_sign), V-GRIPPER-RANGE (WARN: the vendor full-open command -4.9 rad and the publisher's -5.0 rad are unverified on this unit, since facts full_open_rad is null and the largest observed opening is -1.776 rad), V-GRIPPER-WIDTH (WARN: published finger width max 0.045 m from ros_publishers.py:9,15 versus URDF gripper_joint1/2 upper 0.0715 m), V-GRIPPER-UNITS (WARN: the GripperCommand action passes goal.position, metres by ROS convention, to set_gripper_target as motor radians, ros_actions.py:213 and hardware_manager.py:610-615), V-SAFE-PARK (FAIL when safe_park is enabled and any safe_park_q is outside b601_dm_limits.yaml; it must report joint2 +0.011253 and joint3 +0.135614 above upper 0.0, i.e. into the folded contact per the facts sign check, and that the driver seeds this target at connect and parks to it on shutdown, hardware_manager.py:199-216,243-245 and safe_park.yaml), V-STARTUP (WARN with citations: launching the driver switches the arm motors to POS_VEL, a CTRL_MODE register write plus POS_VEL gain registers 25-28, enables all motors including the gripper, and starts a position-hold loop, hardware_manager.py:190-197,572-600 and the SDK's ArmEndPos.start), V-GAINS (WARN: POS_VEL vlim 5.0/3.0 rad/s versus the 1.0 rad/s planning velocity limit; MIT kp/kd 120/10 versus bring-up caps kp<=10, kd<=1), V-SDK-API (FAIL when the vendored SDK does not export what hardware_manager.py:38-39,573 imports: RobotArm, ArmEndPos and actuator.gripper.load_cfg; report the SDK HEAD read from its .git files, plus the note that 8427281 is the last commit exporting them), V-PYDEPS (FAIL when motorbridge or pinocchio is not found on the driver interpreter's /usr/bin/python3, py3.10, search paths, found by a filesystem search of the standard dist/site-packages locations plus any --driver-pythonpath), V-QOS (FAIL unless execute_trajectory.py's joint_states subscriptions are BEST_EFFORT/sensor-data, matching the driver's qos_profile_sensor_data publisher, rebotarm_controller.py:9,24 and ros_publishers.py:22-27; this flips to PASS once MOT-05.7 lands), V-PROFILE (execution.yaml vendor profile action, joint_states, arm_status and expected node versus the driver's node name and namespace), V-ARBITRATION (cmd_arbitration is reject), V-URDF (WARN: bringup.launch.py publishes the gripper-less reBot-DevArm_fixend.urdf, ROBOT_MODEL.md §3).",
      "Unit tests use small synthetic fixture copies under tests/fixtures/vendor_driver/ and never read ~/rebot_ws. A known-good fixture set gives exit 0. Each single-defect variant gives exit 3 with exactly the expected FAIL id: wrong ID, swapped joint order, a direction key, positive gripper open, safe_park_q above a limit, missing SDK export, missing deps, RELIABLE executor QoS, and a channel that does not start with /dev/tty. Missing input files give exit 2.",
      "Against the real ~/rebot_ws at the time of implementation the live report (pasted into the result) shows at least: V-IDS, V-ORDER, V-DIRECTION, V-GRIPPER-SIGN, V-BAUD and V-PROFILE as PASS; V-SAFE-PARK, V-SDK-API and V-PYDEPS as FAIL; V-QOS as FAIL or PASS depending on whether MOT-05.7 has landed. Any deviation from this list is investigated and explained, not hidden.",
      "docs/motion/VENDOR_DRIVER.md is the operator-facing readiness note. It covers what the vendor driver does at launch and at shutdown (energises, writes the mode register, seeds safe_park_q), each FAIL/WARN with its citation and the vendor-side remedy for the operator to apply (this repo never edits ~/rebot_ws), and the exact command to re-run the checker before any vendor-driver session. That command is MOT-09 step D1. The note states that a PASS here does not commission anything and is not evidence for config/robot/commissioning.yaml."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "python-pure-lib",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 85,
  "id": "MOT-11",
  "title": "Vendor-driver readiness check: verify reBotArmController's port, motor IDs, joint order, direction and gripper mapping against the 2026-10-10 bring-up evidence (read-only ~/rebot_ws)",
  "parent": "L-MOTION",
  "outcome": "A read-only checker, scripts/motion/check_vendor_driver.py, with tests and an operator note, docs/motion/VENDOR_DRIVER.md. Before any real run it proves that the vendor driver's port, IDs, joint order, direction and gripper mapping (open = negative) match the operator's 2026-10-10 evidence. It also flags the vendor-stack defects that must be fixed first."
}
```

Operator-filed 2026-10-10 (overnight session, on the operator's instruction). Before any real run, the vendor
driver's joint ordering, direction mapping, gripper mapping (open = negative), port and IDs must be verified
against the 2026-10-10 bring-up evidence. MOT-09 depends on this card and re-runs the checker as a step before
the vendor-driver dry rehearsal.

## Read-only survey of the vendor stack (overnight, 2026-10-10)

The survey is of `~/rebot_ws@2ebc4b8`, with `third_party/reBotArm_control_py` at `0324b73`. It is the expected
content of the live report, and the implementer must reproduce it from source, not copy it.

- **Matches the evidence.** Channel `/dev/ttyACM0` (`src/rebotarm_bringup/config/arm.yaml:4`, `gripper.yaml:3`;
  `driver_params.yaml` channel `""`). Baud 921600, hard-coded in the SDK. IDs 0x01–0x06/0x11–0x16 with models
  4340P×3 and 4310×3 (`arm.yaml`), and the gripper at 0x07/0x17 4310 (`gripper.yaml`). Joint order
  joint1..joint6. No direction or offset fields anywhere, so motor sign = joint sign, and every sign matched the
  URDF on 2026-10-10. Gripper open is negative: `ros_services.py:16` `_GRIPPER_OPEN_POSITION = -4.9`;
  `ros_publishers.py:7-8` open -5.0, closed 0.0.
- **FAIL: `safe_park.yaml`.** It is loaded by default in both launch files (`driver.launch.py:13,32-34,47`;
  `bringup.launch.py`). It has `safe_park_enabled: true` and `safe_park_on_shutdown: true`, with
  `safe_park_q = [-0.000191, 0.011253, 0.135614, 0.036431, 0.015068, -0.008965]`. joint2 and joint3 are above
  their URDF upper limit 0.0. On this unit, positive joint2/joint3 point into the folded contact, so
  `safe_park` would press the forearm into the upper arm. The driver seeds this target at connect
  (`hardware_manager.py:243-245`) and drives to it on shutdown (`hardware_manager.py:199-216`).
- **FAIL: SDK API mismatch.** `hardware_manager.py:38-39` imports `RobotArm` and `ArmEndPos`, but the vendored
  SDK at `0324b73` exports only `RebotArm` and `RebotArmEndPose` (`actuator/__init__.py:23`,
  `controllers/__init__.py:3`). `arm.py`/`gripper.py`/`arm_endpos_controller.py` were removed in `346bf41`
  (2026-06-03); `8427281` (2026-05-21) is the last commit exporting both. The driver cannot start against the
  vendored SDK.
- **FAIL: runtime dependencies.** `motorbridge` and `pinocchio` are not importable by `/usr/bin/python3`, even
  with ROS sourced. motorbridge 0.5.6 exists only in the py3.12 conda env `rebot`.
- **FAIL until MOT-05.7: QoS.** The driver publishes joint_states with `qos_profile_sensor_data`
  (`rebotarm_controller.py:9,24`; `ros_publishers.py:22-27`), and the executor subscribes RELIABLE.
- **WARN: startup behaviour.** `HardwareManager.connect()` → `ArmEndPos.start()` (SDK@8427281
  `arm_endpos_controller.py:105-111`) calls `mode_pos_vel()` (a CTRL_MODE register write, RAM only, because
  motorbridge `ensure_control_mode` writes register 10 only when it differs and never stores), plus POS_VEL gain
  registers 25-28, then `enable_all()`, then starts the control loop. `init_gripper()` (`hardware_manager.py:572-600`)
  calls `enable_all()` again and writes the gripper's POS_VEL registers. So launching the driver energises
  every motor.
- **WARN: gains and speeds.** POS_VEL `vlim` is 5.0 rad/s for joint1-3 and 3.0 for joint4-6, against the 1.0 rad/s
  planning limit. MIT kp/kd is 120/10 for joint1-3 in the bringup `arm.yaml` and 120/8 in the SDK's
  `rebotarm_dm.yaml`.

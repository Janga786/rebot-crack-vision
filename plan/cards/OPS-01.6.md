# OPS-01.6 — capture stage: D405 + joint-state/optical-tf + capture_record assembly, with optional move-to-view-pose

```json card
{
  "kind": "integration",
  "depends_on": [
    "OPS-01.2",
    "OPS-01.5"
  ],
  "requirements": [
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "scripts/operator/capture.sh",
      "scripts/ros/test_operator_capture.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#19. Operator CLI workflow",
    "docs/INTERFACES.md#8. End-of-arm frames, task frames and the eye-in-hand capture contract (ADR-014)",
    "docs/INTERFACES.md#9. Eye-in-hand capture record (crackvision.capture_3d/1)",
    "src/crackvision/realsense_capture.py",
    "src/crackvision/capture_record.py",
    "ros2_ws/src/crackvision_camera/crackvision_camera/capture_joint_state.py",
    "ros2_ws/src/crackvision_camera/crackvision_camera/capture_optical_tf.py",
    "scripts/operator/run_trajectory.sh",
    "docs/motion/CARTESIAN_PLANNING.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "e2e",
        "cmd": "bash scripts/ros/test_operator_capture.sh",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "`scripts/operator/capture.sh --case-id CASE [--view-plan PATH --mode mock|dry|real] [--synthetic] [--root PATH]` exists; when `--view-plan` is given, it first calls `scripts/operator/run_trajectory.sh preview` then `approve-execute --view-plan PATH --mode MODE` (OPS-01.5), and only then captures; when omitted, it captures at the robot's current pose (operator has already positioned it, or `--synthetic` is used for CI/no-hardware runs).",
      "Runs `crackvision.realsense_capture` (or `--synthetic`) for the D405 frame, `ros2 run crackvision_camera capture_joint_state` and `capture_optical_tf` for the pose metadata, then `crackvision.capture_record assemble` to produce `data/captures/{case}_capture.json`, matching naming/paths exactly as those existing tools already default them -- no new file-naming scheme invented here.",
      "On success, records the `capture` stage done in crackvision.operator_state with the capture record's path+sha256; on any step's non-zero exit, records `failed` and exits non-zero without writing a partial/corrupt capture record.",
      "`scripts/ros/test_operator_capture.sh` runs this headlessly against the mock stack and a synthetic D405 frame (`--synthetic`), covering both the with-view-plan and without-view-plan paths, and asserts the resulting `data/captures/{case}_capture.json` (written to a scratch `--root`, never the repo's real `data/`) validates against `crackvision.capture_record`'s own validator."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 4,
    "task_class": "motion-integration",
    "local_ok": false
  },
  "motion": true,
  "id": "OPS-01.6",
  "title": "capture stage: D405 + joint-state/optical-tf + capture_record assembly, with optional move-to-view-pose",
  "parent": "OPS-01",
  "outcome": "A case-scoped capture command that assembles data/captures/{case}_capture.json per the §8.4/§9 eye-in-hand contract, either against the robot's current pose or after driving a supplied view_plan through OPS-01.5's run_trajectory.sh, resumable via crackvision.operator_state."
}
```

Write `scripts/operator/capture.sh` as a sequencing script over already-accepted tools (CAM-02's `crackvision.realsense_capture`, CAM-05.2's `capture_joint_state`/`capture_optical_tf`, GEOM-08.4's `crackvision.capture_record assemble`), plus OPS-01.5's `run_trajectory.sh` for the optional move. Do not write new capture or FK logic.

Sequence:
1. If `--view-plan` given: `scripts/operator/run_trajectory.sh preview --view-plan PATH` then, after the script prints the preview location, `approve-execute --view-plan PATH --mode MODE` (this is where a real typed `APPROVE <sha8>` happens for a live run; your headless test uses the same pseudo-tty technique OPS-01.5's own test uses).
2. Capture the D405 frame: `./env.sh python scripts/capture_d405.py ...` (or `--synthetic`), capturing its metadata JSON path.
3. Capture joint state + optical TF: `scripts/ros/env_ros.sh ros2 run crackvision_camera capture_joint_state ...` / `capture_optical_tf ...`.
4. Assemble: `./env.sh python -m crackvision.capture_record assemble --case CASE --d405-metadata ... --joint-state ... --optical-tf ... [--end-effector-config ...]`.
5. Record the `capture` stage via `crackvision.operator_state` (import it directly, this script's non-ROS parts run under `./env.sh`; for the parts that must run in the ROS shell, wrap the `operator_state` calls themselves in `./env.sh python -m crackvision.operator_state ...` rather than trying to import rclpy-adjacent code into the conda interpreter).

For `scripts/ros/test_operator_capture.sh`: bring up the mock stack the same way `scripts/ros/test_view_retime_execute.sh`/`test_preview_approve.sh` do, use a scratch `--root` (never the repo's `data/`), and run both the `--view-plan` and no-`--view-plan` paths, validating the resulting capture record with `crackvision.capture_record`'s existing validate command.

# MOT-05.3 — Offline execution gate: commissioning/approval/execution-config loaders + full gate report (pure Python)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-05.2"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
      "ros2_ws/src/crackvision_motion/test/test_execution_gate.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/**",
      "config/robot/commissioning.yaml",
      "config/motion/execution.yaml"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#10",
    "docs/adr/016-commissioning-gated-execution.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py",
    "ros2_ws/src/crackvision_description/crackvision_description/end_effector.py",
    "config/scene/scene.yaml",
    "config/robot/end_effector.yaml",
    "config/robot/b601_dm_limits.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_execution_gate.py ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "upstream-gates-regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_scene_core.py ros2_ws/src/crackvision_description/test/test_end_effector.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "shipped-uncommissioned",
        "cmd": "bash -c 'grep -qE \"^commissioned: *false\" config/robot/commissioning.yaml && grep -q \"crackvision.commissioning/1\" config/robot/commissioning.yaml && grep -q \"crackvision.execution_config/1\" config/motion/execution.yaml'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "pure-python",
        "cmd": "bash -c '! grep -nE \"^\\s*(import|from) +(rclpy|control_msgs|moveit_msgs|sensor_msgs)\" ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "evaluate_offline evaluates every applicable check and returns all failures. Gate ids and the mode applicability matrix match §11 exactly. In mock and dry, real-only checks are 'skip', never 'pass'.",
      "With the repo's real configs, mode=real is refused, and the refusal ids include at least G-ARM (env unset), G-SCENE, G-EE and G-COMMISSIONING/G-ESTOP. The test pins these ids.",
      "Using a measured fixture set under test/fixtures/execution_gate/ (measured scene, end_effector, commissioning with matching limits sha, verified e-stop, fresh approval bound to the trajectory sha, eligible paths3d stub) with CRACKVISION_ARM_REAL=1 injected via an env mapping argument, never os.environ mutation in production code, real mode passes offline. Flipping any single input (sha mismatch, stale approval, ineligible paths3d, scale > cap, purpose=test, nominal block) makes exactly the expected check fail.",
      "Both upstream gates are called through their public functions. Their exceptions become failed checks with the original message, and nothing propagates.",
      "The confirmation helpers are pure. confirmation_phrase(sha) = §11 format, and confirmation_ok(typed, sha) does an exact match after strip. Case variants, a missing sha suffix and a wrong sha are rejected (tested).",
      "The shipped commissioning.yaml has commissioned: false, null shas, speed_scale_cap 0.10 and an unverified e-stop, with comments saying MOT-09 fills it from operator evidence. execution.yaml holds the §11 profiles and numeric defaults with rationale comments.",
      "GateReport.to_dict() is JSON-serialisable and is the gate block of crackvision.execution_record/1."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "python-pure-lib",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-05.3",
  "parent": "MOT-05",
  "title": "Offline execution gate: commissioning/approval/execution-config loaders + full gate report (pure Python)",
  "outcome": "crackvision_motion/execution_gate.py evaluates every §11 offline gate check for a mode and returns a structured GateReport. It calls scene_core.assert_commissioning_ready and end_effector.assert_commissioning_ready, checks the limits hash, approval binding, paths3d eligibility, speed cap and arming env, and provides the confirmation-phrase helpers. It ships config/robot/commissioning.yaml (uncommissioned) and config/motion/execution.yaml. Tests prove that the repo defaults refuse real mode with the expected reasons."
}
```

Implement the §11 offline gate. If this body and §11 disagree, §11 wins. It runs under system python3 without a build. Tests add ros2_ws/src/crackvision_motion and ros2_ws/src/crackvision_description to sys.path.

Public API (normative for MOT-05.4):
- `class GateConfigError(ValueError)`
- `load_execution_config(path)`, `load_commissioning(path)`, `load_approval(path)`: strict schema validation; unknown keys are errors.
- `@dataclass GateCheck(id, status: 'pass'|'fail'|'skip', message)`. `@dataclass GateReport(mode, checks, offline_only=True)` with `.passed` (no 'fail'), `.refusals` (failed ids) and `.to_dict()`.
- `evaluate_offline(mode, trajectory_path, *, root, execution_config_path, commissioning_path, limits_path, scene_config_path, end_effector_config_path, approval_path=None, speed_scale=None, env: Mapping[str,str]) -> GateReport`. Resolve the default speed_scale per mode from execution config.
- `confirmation_phrase(trajectory_sha256) -> str`, `confirmation_ok(typed, trajectory_sha256) -> bool`.
- `G_*` id constants.

The paths3d eligibility check reads the referenced §10 file: execution_eligible must be true and its sha256 must match the trajectory's source binding. Resolve relative paths against root. Do not read logs/ to count runs: the speed cap comes only from the commissioning record. Fixture configs are small copies with value_status measured. Never edit the real config/scene/scene.yaml or config/robot/end_effector.yaml.

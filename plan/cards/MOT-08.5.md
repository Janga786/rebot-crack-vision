# MOT-08.5 — execution_gate G-APPROVAL: verify preview_artifacts and the bound trajectory_preview/1 manifest (§18 amendment)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-08.1"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
      "ros2_ws/src/crackvision_motion/test/test_execution_gate.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/**"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#18",
    "docs/INTERFACES.md#11",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
    "ros2_ws/src/crackvision_motion/test/test_execution_gate.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_execution_gate.py ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "ros-free",
        "cmd": "bash -c '! grep -nE \"^\\s*(import|from) +(rclpy|control_msgs|moveit_msgs|sensor_msgs|crackvision\\b)\" ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 1500,
        "expect_exit": 0
      },
      {
        "id": "execute-unit",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; python3 -m pytest ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py -q -p no:cacheprovider'",
        "timeout_s": 900,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The new checks run only after the existing sha-match and age checks pass, in the §18 order. Each violation gives a specific detail. In mock/dry the outcome is warn with the existing ' (warn outside real)' suffix; in real it is fail.",
      "Paths are resolved against the evaluate_offline root. Missing files, sha mismatches, invalid JSON, non-UTF-8 bytes, OSError, zero or two manifests, a manifest naming another trajectory, and all_pass false are each tested in both mock and real modes, and none of them raises.",
      "Approvals that are complete and valid pass. All existing test_execution_gate.py and test_execute_trajectory.py tests stay green, with fixtures updated only where they relied on approvals that §18 now rejects.",
      "The approval file sha still appears in the execution record's input_shas. No other gate's behaviour changes."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "safety-gate",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-08.5",
  "title": "execution_gate G-APPROVAL: verify preview_artifacts and the bound trajectory_preview/1 manifest (§18 amendment)",
  "parent": "MOT-08",
  "outcome": "G-APPROVAL additionally verifies that every preview artefact exists with a matching sha and that exactly one is a trajectory_preview/1 manifest for the current trajectory with all_pass true. Violations are warn in mock/dry and fail in real, and malformed inputs never raise."
}
```

Amend `_evaluate_approval` in ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py per INTERFACES §18. Thread `root` from `evaluate_offline` into it.

- Keep `load_approval`'s strict five-key schema unchanged.
- After the existing sha and age checks pass, iterate `preview_artifacts`:
  - Resolve each path against root and reject absolute paths or paths that escape root, if §18 says so.
  - Read the bytes, catching OSError, and compare the sha256.
  - Try `json.loads` on each artefact whose name ends in .json, catching ValueError and UnicodeDecodeError, and check `schema == "crackvision.trajectory_preview/1"`.
  - Require exactly one manifest, with `trajectory.sha256 == traj_sha` and `check_report.all_pass is True`.
- Do minimal structural validation of the manifest fields you read. Do not import crackvision (conda) code: this module stays ROS-free and conda-free.
- Put the first violation (or all of them, joined with '; ') in the detail.

Tests: add cases to test_execution_gate.py, building approval, manifest and PNG-byte fixtures in tmp_path, or under test/fixtures/execution_gate/ if static. Cover every violation listed in the acceptance criteria for mode mock (→ warn) and mode real (→ fail), plus one full pass. Then run the whole suite, update any existing approval-pass fixture to include a valid manifest artefact, and list the fixtures you changed in the PR.

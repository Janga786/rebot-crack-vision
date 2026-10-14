# MOT-08.5.R2 — Repair MOT-08.5: preview_artifacts entry with non-string or NUL-containing path makes e

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/**",
      "ros2_ws/src/crackvision_motion/test/test_execution_gate.py"
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
      "When an approval's preview_artifacts path is an int, null, a list or a NUL-containing string, or its sha256 is not a string, evaluate_offline returns G-APPROVAL warn (mock/dry) or fail (real) and raises nothing. Parametrized tests pin these cases. Repair card MOT-08.5.R1 already tracks this.",
      "All acceptance criteria of MOT-08.5 still hold at the new revision"
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
  "priority": 95,
  "repairs": "MOT-08.5",
  "findings": [
    "6a1934e18cc39f49"
  ],
  "id": "MOT-08.5.R2",
  "title": "Repair MOT-08.5: preview_artifacts entry with non-string or NUL-containing path makes e",
  "parent": "MOT-08",
  "outcome": "Resolve the review findings on MOT-08.5 while every acceptance criterion of MOT-08.5 still holds."
}
```

Repair work generated deterministically from review attempt `01217-MOT-05.3.R1-rereview` of `MOT-08.5`.
Original card: `plan/cards/MOT-08.5.md` — its acceptance criteria must still hold.

## Finding 1 [major] preview_artifacts entry with non-string or NUL-containing path makes evaluate_offline raise
- Evidence: load_approval (execution_gate.py:327-329) checks only that each entry has exactly the keys {path, sha256}; it does not check their types. _evaluate_preview_binding then calls _resolve(root, artifact["path"]) and path.read_bytes() and catches only OSError. My probe used _passing_fixture_set with the first artifact's path mutated. path=123 gave 'RAISE TypeError expected str, bytes or os.PathLike object, not int'. path=None also raised TypeError. path='a\x00b.json' gave 'RAISE ValueError embedded null byte'. Each of these happened in both mock and real mode.
- Consequence: This violates §18.3 ('any other exception while loading an artefact ... never an uncaught exception'). A malformed approval crashes the gate, so MOT-05.4 exits 1 instead of producing a structured G-APPROVAL warn/fail with exit 3.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py, ros2_ws/src/crackvision_motion/test/test_execution_gate.py
- Acceptance condition: When an approval's preview_artifacts path is an int, null, a list or a NUL-containing string, or its sha256 is not a string, evaluate_offline returns G-APPROVAL warn (mock/dry) or fail (real) and raises nothing. Parametrized tests pin these cases. Repair card MOT-08.5.R1 already tracks this.

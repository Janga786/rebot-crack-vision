# MOT-08.5.R3 — Repair MOT-08.5: G-APPROVAL preview binding lets TypeError/ValueError escape evaluate_o

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
      "Either load_approval rejects non-string or empty path/sha256 values with GateConfigError, or _evaluate_preview_binding catches every exception per artefact. In both cases, parametrised tests with path=123, a path containing a NUL byte and sha256=5 must show evaluate_offline returning G-APPROVAL 'fail' in real mode and 'warn' in mock/dry, without raising.",
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
    "e5cc7474d30db83a"
  ],
  "id": "MOT-08.5.R3",
  "title": "Repair MOT-08.5: G-APPROVAL preview binding lets TypeError/ValueError escape evaluate_o",
  "parent": "MOT-08",
  "outcome": "Resolve the review findings on MOT-08.5 while every acceptance criterion of MOT-08.5 still holds."
}
```

Repair work generated deterministically from review attempt `01225-MOT-05.3-rereview` of `MOT-08.5`.
Original card: `plan/cards/MOT-08.5.md` — its acceptance criteria must still hold.

## Finding 1 [major] G-APPROVAL preview binding lets TypeError/ValueError escape evaluate_offline for malformed preview_artifacts entries
- Evidence: load_approval (execution_gate.py:327-329) checks only `set(a) != {"path", "sha256"}` and never the value types. _evaluate_preview_binding (execution_gate.py:841-845) runs `path = _resolve(root, artifact["path"])`, then `path.read_bytes()` inside `except OSError` only. Probe through _passing_fixture_set with evaluate_offline('real', ...): path=123 raised TypeError 'expected str, bytes or os.PathLike object, not int'; path='a\x00b.json' raised ValueError 'embedded null byte'. §18.3 says: 'Unreadable files, malformed JSON, a non-UTF-8 artefact, or any other exception while loading an artefact or manifest become that check's violation outcome, never an uncaught exception'.
- Consequence: A hand-edited or corrupted approval file crashes the gate (and MOT-05.4's execute_trajectory) instead of producing a G-APPROVAL warn or fail. This breaks this card's 'nothing propagates' criterion and brings back the resolved finding class 01ac0548.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py, ros2_ws/src/crackvision_motion/test/test_execution_gate.py
- Acceptance condition: Either load_approval rejects non-string or empty path/sha256 values with GateConfigError, or _evaluate_preview_binding catches every exception per artefact. In both cases, parametrised tests with path=123, a path containing a NUL byte and sha256=5 must show evaluate_offline returning G-APPROVAL 'fail' in real mode and 'warn' in mock/dry, without raising.

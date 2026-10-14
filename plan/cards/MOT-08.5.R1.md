# MOT-08.5.R1 — Repair MOT-08.5: Non-string or NUL-containing preview_artifacts path makes evaluate_off

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
      "For an approval whose preview_artifacts entry has path set to an int, null, a list, or a string containing \\u0000, and for a sha256 that is not a string, evaluate_offline returns G-APPROVAL outcome 'warn' (mock) and 'fail' (real) with a specific detail and raises nothing. Parametrized mock/real tests cover at least the non-str path and NUL-byte cases.",
      "Each §18.3 violation detail includes its check number (e.g. '§18.3 check 3: …'), and at least one test per check asserts the number appears in the detail.",
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
    "6f6a3f33357e1085",
    "e7791f5803a69151"
  ],
  "id": "MOT-08.5.R1",
  "title": "Repair MOT-08.5: Non-string or NUL-containing preview_artifacts path makes evaluate_off",
  "parent": "MOT-08",
  "outcome": "Resolve the review findings on MOT-08.5 while every acceptance criterion of MOT-08.5 still holds."
}
```

Repair work generated deterministically from review attempt `01215-MOT-08.5-review` of `MOT-08.5`.
Original card: `plan/cards/MOT-08.5.md` — its acceptance criteria must still hold.

## Finding 1 [major] Non-string or NUL-containing preview_artifacts path makes evaluate_offline raise instead of returning a G-APPROVAL violation
- Evidence: execution_gate.py:841 `path = _resolve(root, artifact["path"])` runs outside any try block, and only `path.read_bytes()` is guarded with `except OSError`. load_approval (lines 327-329) checks only `set(a) == {"path","sha256"}` and never the value types. My probe, which takes _passing_fixture_set and sets preview_artifacts[1].path to 5 / null / ["x"] / "a\u0000b.json", printed: 'int path mock RAISED TypeError expected str, bytes or os.PathLike object, not int', 'null path real RAISED TypeError … not NoneType', 'list path mock RAISED TypeError … not list', 'nul byte real RAISED ValueError embedded null byte'. The f-string at line 849 would also take whatever non-str sha256 value is in the entry without checking it.
- Consequence: This breaks the card outcome ('malformed inputs never raise') and §18.3 ('any other exception while loading an artefact or manifest become that check's violation outcome, never an uncaught exception'). execute_trajectory calls evaluate_offline in a try block with no handler for TypeError or ValueError, so a malformed approval crashes the CLI with a traceback instead of producing a gate report and an execution record. It fails closed, so there is no motion risk.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py, ros2_ws/src/crackvision_motion/test/test_execution_gate.py
- Acceptance condition: For an approval whose preview_artifacts entry has path set to an int, null, a list, or a string containing \u0000, and for a sha256 that is not a string, evaluate_offline returns G-APPROVAL outcome 'warn' (mock) and 'fail' (real) with a specific detail and raises nothing. Parametrized mock/real tests cover at least the non-str path and NUL-byte cases.

## Finding 2 [minor] Violation details do not name the numbered §18.3 check, as the contract requires
- Evidence: docs/INTERFACES.md §18.3: 'A violation of any one of 1–5 is reported with a detail naming which numbered check failed and why'. The implementation's details are e.g. execution_gate.py:835 'approval.preview_artifacts is empty', :866 'expected exactly one … manifest among preview_artifacts, found N', :871 "preview manifest's trajectory.sha256 does not match the current trajectory". None of them says which check number (1–5) failed.
- Consequence: The detail strings do not follow the normative contract the card names as an input. Operators and downstream tooling can't tie a G-APPROVAL refusal to its §18.3 check number.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py, ros2_ws/src/crackvision_motion/test/test_execution_gate.py
- Acceptance condition: Each §18.3 violation detail includes its check number (e.g. '§18.3 check 3: …'), and at least one test per check asserts the number appears in the detail.

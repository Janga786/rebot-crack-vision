# MOT-05.3.R2 — Repair MOT-05.3: view_move G-ELIGIBLE lets UnicodeDecodeError / YAML errors escape eval

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "config/motion/execution.yaml",
      "config/robot/commissioning.yaml",
      "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/**",
      "ros2_ws/src/crackvision_motion/test/test_execution_gate.py"
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
      "evaluate_offline returns a GateReport (no exception) in which G-ELIGIBLE is 'error' or 'fail', both for a view_move trajectory whose sha-matching view_plan file is not valid UTF-8 and for one evaluated with an unparseable limits_path. Regression tests cover both cases.",
      "All acceptance criteria of MOT-05.3 still hold at the new revision"
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
  "priority": 95,
  "repairs": "MOT-05.3",
  "findings": [
    "8e8462a234497fb6"
  ],
  "id": "MOT-05.3.R2",
  "title": "Repair MOT-05.3: view_move G-ELIGIBLE lets UnicodeDecodeError / YAML errors escape eval",
  "parent": "MOT-05",
  "outcome": "Resolve the review findings on MOT-05.3 while every acceptance criterion of MOT-05.3 still holds."
}
```

Repair work generated deterministically from review attempt `01053-MOT-05.3-rereview` of `MOT-05.3`.
Original card: `plan/cards/MOT-05.3.md` — its acceptance criteria must still hold.

## Finding 1 [major] view_move G-ELIGIBLE lets UnicodeDecodeError / YAML errors escape evaluate_offline
- Evidence: execution_gate.py `_evaluate_eligible_view_move` only has `except CartesianPlanError` around `load_view_plan(vp_path, limits_path)`. cartesian_plan.py:926 `Path(path).read_text(encoding="utf-8")` only catches OSError, and :933 `load_limits(limits_path)` calls `yaml.safe_load(Path(path).read_text())` without wrapping. evaluate_offline has no outer try. Probe output: `nonutf8 ESCAPED UnicodeDecodeError 'utf-8' codec can't decode byte 0xff` and `badlimits ESCAPED ParserError while parsing a flow sequence`.
- Consequence: A corrupt view-plan or limits file crashes the gate instead of producing a GateReport with an error/fail check. This breaks the 'nothing propagates' contract MOT-05.4 relies on to write an execution record and pick the exit code. It reintroduces the defect class of resolved finding 53f25b87 on the new view_move path.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py, ros2_ws/src/crackvision_motion/test/test_execution_gate.py
- Acceptance condition: evaluate_offline returns a GateReport (no exception) in which G-ELIGIBLE is 'error' or 'fail', both for a view_move trajectory whose sha-matching view_plan file is not valid UTF-8 and for one evaluated with an unparseable limits_path. Regression tests cover both cases.

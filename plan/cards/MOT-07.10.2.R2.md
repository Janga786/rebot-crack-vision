# MOT-07.10.2.R2 — Repair MOT-07.10.2: View_move G-ELIGIBLE lets UnicodeDecodeError, OSError and YAML errors

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/view_move/**",
      "ros2_ws/src/crackvision_motion/test/test_execution_gate.py",
      "ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#17",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/adr/019-time-parameterization.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 1500,
        "expect_exit": 0
      },
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; python3 -m pytest ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py ros2_ws/src/crackvision_motion/test/test_execution_gate.py ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py -q -p no:cacheprovider'",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "gate-regression",
        "cmd": "bash scripts/ros/test_execution.sh",
        "timeout_s": 1800,
        "expect_exit": 0
      },
      {
        "id": "real-refusal",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; unset CRACKVISION_ARM_REAL; timeout 60 ros2 run crackvision_motion execute_trajectory --mode real --trajectory ros2_ws/src/crackvision_motion/test/fixtures/trajectory_smoke.json --root \"$PWD\" </dev/null; rc=$?; echo rc=$rc; test $rc -eq 3'",
        "timeout_s": 120,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "For a view_move trajectory, evaluate_offline returns a GateReport and does not raise in each of these cases: (a) the bound view-plan file is non-UTF-8 → G-ELIGIBLE fail or error; (b) limits_path is missing or (c) is malformed YAML → G-ELIGIBLE error, or the view-plan validation runs against the limits already loaded and the check is reported as not evaluable. Parametrized tests over mock, dry and real pin each case.",
      "All acceptance criteria of MOT-07.10.2 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "execution-gate",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-07.10.2",
  "findings": [
    "bb9fa426d003590e"
  ],
  "id": "MOT-07.10.2.R2",
  "title": "Repair MOT-07.10.2: View_move G-ELIGIBLE lets UnicodeDecodeError, OSError and YAML errors",
  "parent": "MOT-07.10",
  "outcome": "Resolve the review findings on MOT-07.10.2 while every acceptance criterion of MOT-07.10.2 still holds."
}
```

Repair work generated deterministically from review attempt `01042-MOT-05.3.R1-rereview` of `MOT-07.10.2`.
Original card: `plan/cards/MOT-07.10.2.md` — its acceptance criteria must still hold.

## Finding 1 [major] View_move G-ELIGIBLE lets UnicodeDecodeError, OSError and YAML errors escape evaluate_offline
- Evidence: execution_gate.py:736-740 is `try: view_plan = load_view_plan(vp_path, limits_path) except CartesianPlanError`. cartesian_plan.py:926-933 catches only OSError around read_text(encoding='utf-8') and then calls `limits = load_limits(limits_path)` with no guard. Probe in real mode: (1) the view-plan file is non-UTF-8 but its sha matches → 'RAISE UnicodeDecodeError'; (2) limits_path is missing → 'RAISE FileNotFoundError'; (3) the limits YAML is malformed → 'RAISE ParserError'. For cases 2 and 3, line 440 has already recorded G-LIMITS-HASH/G-LIMITS as error, but the eligible branch at line 639 still runs.
- Consequence: Previously resolved findings 07f73b3be28b9c43, 01ac0548ddc1f43f and 53f25b87b738ef47 have regressed for purpose view_move. MOT-05.4 would crash (exit 1) instead of producing a structured gate report and refusing with exit 3. §11.7 also requires every gate to be evaluated and reported, which a crash prevents.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py, ros2_ws/src/crackvision_motion/test/test_execution_gate.py
- Acceptance condition: For a view_move trajectory, evaluate_offline returns a GateReport and does not raise in each of these cases: (a) the bound view-plan file is non-UTF-8 → G-ELIGIBLE fail or error; (b) limits_path is missing or (c) is malformed YAML → G-ELIGIBLE error, or the view-plan validation runs against the limits already loaded and the check is reported as not evaluable. Parametrized tests over mock, dry and real pin each case.

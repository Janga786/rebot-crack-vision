# MOT-07.10.2.R1 — Repair MOT-07.10.2: G-ELIGIBLE view_move branch crashes the whole offline gate on non-Cart

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
      "Two cases must return a complete GateReport instead of raising. (a) A sha-bound view-plan file that is not UTF-8: G-ELIGIBLE is 'fail' in mock, dry and real. (b) An unreadable or missing limits file with a view_move trajectory: G-ELIGIBLE is 'error' (or 'fail') and G-LIMITS/G-LIMITS-HASH stay 'error'. Each case needs a unit test in test_execution_gate.py.",
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
    "c13b05680a0183f6"
  ],
  "id": "MOT-07.10.2.R1",
  "title": "Repair MOT-07.10.2: G-ELIGIBLE view_move branch crashes the whole offline gate on non-Cart",
  "parent": "MOT-07.10",
  "outcome": "Resolve the review findings on MOT-07.10.2 while every acceptance criterion of MOT-07.10.2 still holds."
}
```

Repair work generated deterministically from review attempt `01040-MOT-07.10.2-review` of `MOT-07.10.2`.
Original card: `plan/cards/MOT-07.10.2.md` — its acceptance criteria must still hold.

## Finding 1 [major] G-ELIGIBLE view_move branch crashes the whole offline gate on non-CartesianPlanError load failures
- Evidence: execution_gate.py _evaluate_eligible_view_move: `try: view_plan = load_view_plan(vp_path, limits_path) except CartesianPlanError as exc:`. cartesian_plan.load_view_plan (lines 923-934) catches only OSError on read and JSONDecodeError on parse. It calls load_limits(limits_path) without wrapping it, and load_limits raises raw OSError/KeyError/yaml errors. I reproduced both failures with a probe that used the tests' own helpers (_write_view_plan, _view_move_trajectory, _view_move_kwargs). (1) A view_plan.json containing b'\xff\xfe\x00garbage', with the trajectory sha bound to it: evaluate_offline raised `UnicodeDecodeError 'utf-8' codec can't decode byte 0xff`. `ros2 run crackvision_motion execute_trajectory --mode dry --trajectory <that traj>` logged 'unhandled exception: ...' and exited rc=1. (2) A valid view plan with limits_path pointing to a missing file: evaluate_offline raised `FileNotFoundError`. By contrast, the same missing limits path with trajectory_smoke.json returns a full report with G-LIMITS-HASH/G-LIMITS 'error'. The crack_task branch also handles UnicodeDecodeError explicitly as an integrity 'fail'.
- Consequence: §17.2 integrity item 3 says that if the bound file does not load via load_view_plan without raising, G-ELIGIBLE is fail/error in every mode. Instead the gate produces no report at all and the CLI exits 1 (internal error) rather than 3 (refused). The gate's per-check error reporting no longer holds for view_move inputs, unlike every other branch of evaluate_offline. No motion results, but the operator gets no gate diagnosis and the exit-code contract (§0) is not met.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py, ros2_ws/src/crackvision_motion/test/test_execution_gate.py
- Acceptance condition: Two cases must return a complete GateReport instead of raising. (a) A sha-bound view-plan file that is not UTF-8: G-ELIGIBLE is 'fail' in mock, dry and real. (b) An unreadable or missing limits file with a view_move trajectory: G-ELIGIBLE is 'error' (or 'fail') and G-LIMITS/G-LIMITS-HASH stay 'error'. Each case needs a unit test in test_execution_gate.py.

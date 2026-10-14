# MOT-07.10.2.R2.R1 — Repair MOT-07.10.2.R2: check gate-regression fails after upstream change

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
      "`bash scripts/ros/test_execution.sh` exits 0",
      "All acceptance criteria of MOT-07.10.2.R2 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "execution-gate",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 99,
  "repairs": "MOT-07.10.2.R2",
  "findings": [
    "2085370d14cf7e7d"
  ],
  "id": "MOT-07.10.2.R2.R1",
  "title": "Repair MOT-07.10.2.R2: check gate-regression fails after upstream change",
  "parent": "MOT-07.10",
  "outcome": "Resolve the review findings on MOT-07.10.2.R2 while every acceptance criterion of MOT-07.10.2.R2 still holds."
}
```

Repair work generated deterministically from review attempt `recheck-MOT-07.10.2.R2` of `MOT-07.10.2.R2`.
Original card: `plan/cards/MOT-07.10.2.R2.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] check gate-regression fails after upstream change
- Evidence: `bash scripts/ros/test_execution.sh` exit 1: interface mock (rebot_motion mock_driver) ==
_mock_stack.sh: launch pid=416 pgid=416 up
[INFO] [1791945516.616873835] [apply_scene]: applied scene from /home/boosterk1/Projects/rebot_crack_vision/config/scene/scene.yaml: objects=[table, specimen]
test_execution.sh: mock_driver pid=561 pgid=561 up
-- mock mode (vendor_mock profile): trajectory_smoke (home->P) against mock_driver --
2026-10-14 02:38:52,573 ERROR   crackvision_motion.execute_trajectory: an online read-only gate refused
[ros2run]: Process exited with failure 3
test_execution.sh: FAIL: vendor_mock smoke: expected exit 0, got 3 (see header note: execute_trajectory subscribes to joint_states with RELIABLE QoS, incompatible with mock_driver's BEST_EFFORT publisher; this is an executor bug outside this card's sc
…[720 chars clipped]
- Consequence: previously accepted behaviour regressed
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py, ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py, ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/view_move/**, ros2_ws/src/crackvision_motion/test/test_execution_gate.py, ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py
- Acceptance condition: `bash scripts/ros/test_execution.sh` exits 0

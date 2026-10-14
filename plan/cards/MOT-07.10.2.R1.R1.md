# MOT-07.10.2.R1.R1 — Repair MOT-07.10.2.R1: check gate-regression fails after upstream change

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
      "All acceptance criteria of MOT-07.10.2.R1 still hold at the new revision"
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
  "repairs": "MOT-07.10.2.R1",
  "findings": [
    "2085370d14cf7e7d"
  ],
  "id": "MOT-07.10.2.R1.R1",
  "title": "Repair MOT-07.10.2.R1: check gate-regression fails after upstream change",
  "parent": "MOT-07.10",
  "outcome": "Resolve the review findings on MOT-07.10.2.R1 while every acceptance criterion of MOT-07.10.2.R1 still holds."
}
```

Repair work generated deterministically from review attempt `recheck-MOT-07.10.2.R1` of `MOT-07.10.2.R1`.
Original card: `plan/cards/MOT-07.10.2.R1.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] check gate-regression fails after upstream change
- Evidence: `bash scripts/ros/test_execution.sh` exit 1: ted with failure 1
   OK: exit 1, stopped short of Q (max|dq to Q|=0.08258281146666668)
-- mock mode: a short trajectory from the live e-stopped state is refused by G-COLLISION once an oversized collider is applied --
[INFO] [1791940121.650497912] [apply_scene]: applied scene from /home/boosterk1/Projects/rebot_crack_vision/ros2_ws/src/crackvision_motion/test/fixtures/scene_collision_smoke.yaml: objects=[collider]
2026-10-14 01:08:43,577 ERROR   crackvision_motion.execute_trajectory: an online read-only gate refused
[ros2run]: Process exited with failure 3
   OK: exit 3, G-COLLISION fail, no motion
== tearing down the first MoveIt mock stack instance before starting mock_driver ==
== phase 3: vendor-interface mock (rebot_motion mock_driver) ==
_mock_stack.sh: launch pid
…[720 chars clipped]
- Consequence: previously accepted behaviour regressed
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py, ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py, ros2_ws/src/crackvision_motion/test/fixtures/execution_gate/view_move/**, ros2_ws/src/crackvision_motion/test/test_execution_gate.py, ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py
- Acceptance condition: `bash scripts/ros/test_execution.sh` exits 0

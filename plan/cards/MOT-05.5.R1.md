# MOT-05.5.R1 — Repair MOT-05.5: check execution-mock-e2e fails after upstream change

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/test/fixtures/execution_test_*.json",
      "ros2_ws/src/crackvision_motion/test/fixtures/execution_test_*.yaml",
      "scripts/ros/test_execution.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#11",
    "docs/adr/016-commissioning-gated-execution.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
    "ros2_ws/src/crackvision_motion/test/fixtures/trajectory_smoke.json",
    "ros2_ws/src/crackvision_motion/test/fixtures/trajectory_estop.json",
    "ros2_ws/src/crackvision_motion/test/fixtures/trajectory_overspeed.json",
    "ros2_ws/src/crackvision_motion/test/fixtures/scene_collision_smoke.yaml",
    "scripts/ros/_mock_stack.sh",
    "scripts/ros/test_scene.sh",
    "scripts/ros/env_ros.sh",
    "config/motion/execution.yaml"
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
        "id": "execution-mock-e2e",
        "cmd": "bash scripts/ros/test_execution.sh",
        "timeout_s": 1200,
        "expect_exit": 0
      },
      {
        "id": "scene-regression",
        "cmd": "bash scripts/ros/test_scene.sh",
        "timeout_s": 900,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "`bash scripts/ros/test_execution.sh` exits 0",
      "All acceptance criteria of MOT-05.5 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 4,
    "consequence": 4,
    "task_class": "ros2-integration-test",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 99,
  "repairs": "MOT-05.5",
  "findings": [
    "7ed115e0004116ca"
  ],
  "id": "MOT-05.5.R1",
  "title": "Repair MOT-05.5: check execution-mock-e2e fails after upstream change",
  "parent": "MOT-05",
  "outcome": "Resolve the review findings on MOT-05.5 while every acceptance criterion of MOT-05.5 still holds."
}
```

Repair work generated deterministically from review attempt `recheck-MOT-05.5` of `MOT-05.5`.
Original card: `plan/cards/MOT-05.5.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] check execution-mock-e2e fails after upstream change
- Evidence: `bash scripts/ros/test_execution.sh` exit 1: ted with failure 1
   OK: exit 1, stopped short of Q (max|dq to Q|=0.08258325120000001)
-- mock mode: a short trajectory from the live e-stopped state is refused by G-COLLISION once an oversized collider is applied --
[INFO] [1791940893.517771733] [apply_scene]: applied scene from /home/boosterk1/Projects/rebot_crack_vision/ros2_ws/src/crackvision_motion/test/fixtures/scene_collision_smoke.yaml: objects=[collider]
2026-10-14 01:21:36,450 ERROR   crackvision_motion.execute_trajectory: an online read-only gate refused
[ros2run]: Process exited with failure 3
   OK: exit 3, G-COLLISION fail, no motion
== tearing down the first MoveIt mock stack instance before starting mock_driver ==
== phase 3: vendor-interface mock (rebot_motion mock_driver) ==
_mock_stack.sh: launch pid
…[720 chars clipped]
- Consequence: previously accepted behaviour regressed
- Affected: scripts/ros/test_execution.sh, ros2_ws/src/crackvision_motion/test/fixtures/execution_test_*.yaml, ros2_ws/src/crackvision_motion/test/fixtures/execution_test_*.json
- Acceptance condition: `bash scripts/ros/test_execution.sh` exits 0

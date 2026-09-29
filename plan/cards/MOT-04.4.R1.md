# MOT-04.4.R1 — Repair MOT-04.4: Non-infeasibility /compute_ik error codes are silently classed as 'unr

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "docs/motion/ROS_WORKSPACE.md",
      "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/reachability_smoke.yaml",
      "scripts/ros/_mock_stack.sh",
      "scripts/ros/run_reachability.sh",
      "scripts/ros/test_reachability.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#7",
    "docs/motion/ROS_WORKSPACE.md",
    "docs/motion/ROBOT_MODEL.md#4",
    "scripts/ros/env_ros.sh",
    "scripts/ros/test_mock_plan.sh",
    "ros2_ws/src/crackvision_motion/launch/mock_planning.launch.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_map.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "config/motion/reachability.yaml",
    "config/robot/b601_dm_limits.yaml"
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
        "id": "sweep-smoke",
        "cmd": "bash scripts/ros/test_reachability.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "no-execution-paths",
        "cmd": "bash -c '! grep -nE \"ExecuteTrajectory|FollowJointTrajectory|execute_trajectory|MoveGroup\\b|plan_only *= *False|/move_action\" ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "mock-plan-regression",
        "cmd": "bash scripts/ros/test_mock_plan.sh",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Only NO_IK_SOLUTION (and TIMED_OUT, if it is treated as infeasible) count toward 'unreachable'. Any other non-SUCCESS code (e.g. INVALID_GROUP_NAME, INVALID_LINK_NAME, FRAME_TRANSFORM_FAILURE, FAILURE) gives status 'error' with the code named in 'error', and the sweep exits 1. Re-running the invalid-group repro above then exits 1 with status error targets, and test_reachability.sh still passes.",
      "All acceptance criteria of MOT-04.4 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 4,
    "task_class": "ros2-moveit-integration",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-04.4",
  "findings": [
    "0f62e3c3eb58f8d5"
  ],
  "id": "MOT-04.4.R1",
  "title": "Repair MOT-04.4: Non-infeasibility /compute_ik error codes are silently classed as 'unr",
  "parent": "MOT-04",
  "outcome": "Resolve the review findings on MOT-04.4 while every acceptance criterion of MOT-04.4 still holds."
}
```

Repair work generated deterministically from review attempt `00086-MOT-04.4-review` of `MOT-04.4`.
Original card: `plan/cards/MOT-04.4.md` — its acceptance criteria must still hold.

## Finding 1 [major] Non-infeasibility /compute_ik error codes are silently classed as 'unreachable' (exit 0)
- Evidence: reachability_sweep.py:356 `if resp.error_code.val != MoveItErrorCodes.SUCCESS: continue` treats every failure code as an infeasible IK result. Repro: sed 's/^group: arm/group: armx/' on the fixture → /tmp/badgroup.yaml; `bash scripts/ros/run_reachability.sh --reachability-config /tmp/badgroup.yaml --out /tmp/rs_bad.json` → exit=0, log 'counts={'unreachable': 6}', map summary {'counts_by_status': {'unreachable': 6}} with complete=True.
- Consequence: §7.2 says 'error' is for a failed call rather than IK cleanly reporting infeasible, and 'unreachable' is only for genuine IK failure. A wrong group or link, a frame-transform failure, or a missing or failed kinematics solver produces a complete, valid-looking all-unreachable map and exit 0. MOT-04.3 placement would then consume it as ground truth. The card explicitly requires that a failing IK path is surfaced as an error, not as 'unreachable'.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py
- Acceptance condition: Only NO_IK_SOLUTION (and TIMED_OUT, if it is treated as infeasible) count toward 'unreachable'. Any other non-SUCCESS code (e.g. INVALID_GROUP_NAME, INVALID_LINK_NAME, FRAME_TRANSFORM_FAILURE, FAILURE) gives status 'error' with the code named in 'error', and the sweep exits 1. Re-running the invalid-group repro above then exits 1 with status error targets, and test_reachability.sh still passes.

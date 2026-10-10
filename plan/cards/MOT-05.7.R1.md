# MOT-05.7.R1 — Repair MOT-05.7: No unit test drives start_state with a fake whose only message is olde

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py",
      "ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py",
      "scripts/ros/test_joint_states_qos.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#11",
    "docs/adr/016-commissioning-gated-execution.md",
    "config/motion/execution.yaml",
    "scripts/ros/env_ros.sh",
    "scripts/ros/_mock_stack.sh",
    "scripts/ros/test_reachability.sh"
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
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; python3 -m pytest ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py ros2_ws/src/crackvision_motion/test/test_execution_gate.py ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py -q -p no:cacheprovider'",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "qos-live",
        "cmd": "bash scripts/ros/test_joint_states_qos.sh",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "real-refused-offline",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; unset CRACKVISION_ARM_REAL; timeout 60 ros2 run crackvision_motion execute_trajectory --mode real --trajectory ros2_ws/src/crackvision_motion/test/fixtures/trajectory_smoke.json --root \"$PWD\" </dev/null; rc=$?; echo rc=$rc; test $rc -eq 3'",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "single-action-client",
        "cmd": "bash -c 'test \"$(grep -c \"ActionClient(\" ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py)\" -eq 1'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "no-vendor-motion-services",
        "cmd": "bash -c '! grep -nE \"safe_home|/park|\\\"park\\\"|/enable|\\\"enable\\\"|move_to_pose|gravity_compensation|set_zero|set_mode|gripper/\" ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py'",
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
      "Add a test that calls OnlineChecks.start_state with _TimedFakeNode(delay_s small, stamp_age_s > joint_state_timeout_s, e.g. 1.0 vs 0.5) and a discovery bound of 5.0, and asserts outcome == 'fail' and 'stale' in detail. With start_state's age computation mutated to 0.0, that test must fail.",
      "Add a test in which execute() receives at least one joint_states message and then stops publishing while the goal is still running (result not done). Assert outcome == 'aborted', goal_handle.cancel_calls == 1, and the trip occurs about joint_state_s after the last message. Mutating line 636 to `if False:` must make that test fail.",
      "All acceptance criteria of MOT-05.7 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "ros2-moveit-integration",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-05.7",
  "findings": [
    "69d5e8543bc06ea8",
    "ab2a70e05475c40d"
  ],
  "id": "MOT-05.7.R1",
  "title": "Repair MOT-05.7: No unit test drives start_state with a fake whose only message is olde",
  "parent": "MOT-05",
  "outcome": "Resolve the review findings on MOT-05.7 while every acceptance criterion of MOT-05.7 still holds."
}
```

Repair work generated deterministically from review attempt `00419-MOT-05.7-review` of `MOT-05.7`.
Original card: `plan/cards/MOT-05.7.md` — its acceptance criteria must still hold.

## Finding 1 [major] No unit test drives start_state with a fake whose only message is older than joint_state_s
- Evidence: The criterion requires 'a fake whose only message is older than joint_state_s (fails as stale)'. The only stale test is test_execute_trajectory.py:376 `et.check_start_state("mock", "moveit_mock", positions, 5.0, ...)`, a pure function with a precomputed age. _TimedFakeNode supports stamp_age_s, but no test sets it above joint_state_s. Mutation in a /tmp copy, execute_trajectory.py:413 `age_s = max(0.0, now_s - stamp_s)` -> `age_s = 0.0`: pytest test_execute_trajectory.py -> '55 passed'.
- Consequence: A regression in which start_state stops deriving age from the received message, for example while reworking the discovery wait, would silently turn off G-START-STATE staleness refusal with no failing test.
- Affected: ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py
- Acceptance condition: Add a test that calls OnlineChecks.start_state with _TimedFakeNode(delay_s small, stamp_age_s > joint_state_timeout_s, e.g. 1.0 vs 0.5) and a discovery bound of 5.0, and asserts outcome == 'fail' and 'stale' in detail. With start_state's age computation mutated to 0.0, that test must fail.

## Finding 2 [major] Mid-execution joint_states gap -> §11.8 trip is not pinned by any unit test
- Evidence: The criterion says 'an existing or new unit test pins that'. Mutation in a /tmp copy, execute_trajectory.py:636 `if elapsed > joint_state_timeout and is_stale(age_s, joint_state_timeout):` -> `if False:`: pytest test_execute_trajectory.py -> '55 passed'. test_execute_successful_with_stale_joint_states_is_aborted (line 929) only goes silent after the result completes, so _verify_arrival catches it, not the in-goal monitor.
- Consequence: The in-goal staleness e-stop path, a safety monitor for real-arm execution, could be removed or weakened, for example by a QoS or subscription change that stops callbacks, without any test failing.
- Affected: ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py
- Acceptance condition: Add a test in which execute() receives at least one joint_states message and then stops publishing while the goal is still running (result not done). Assert outcome == 'aborted', goal_handle.cancel_calls == 1, and the trip occurs about joint_state_s after the last message. Mutating line 636 to `if False:` must make that test fail.

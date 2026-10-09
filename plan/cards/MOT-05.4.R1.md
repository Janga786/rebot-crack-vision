# MOT-05.4.R1 — Repair MOT-05.4: Mode/profile table taken from overridable --execution-config lets mock

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
      "ros2_ws/src/crackvision_motion/package.xml",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#11",
    "docs/adr/016-commissioning-gated-execution.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/check_end_effector.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/assert_scene_objects.py",
    "config/motion/execution.yaml",
    "config/robot/commissioning.yaml"
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
      "The executor enforces a hard-coded §11.1 table: real is allowed only with profile 'vendor' and mock only with moveit_mock/vendor_mock, whatever execution.yaml says. A config whose modes, action or expected_node differ from §11.1 is rejected with exit 2. A unit test shows that run(mode='mock', profile='vendor') with such a config raises ConfigError and never calls execute().",
      "After a trip during the pending send, execute() keeps waiting (bounded) for the goal response. If the goal is accepted it cancels it and runs the settle check; if no response arrives in time it prints 'PRESS THE HARDWARE E-STOP'. A unit test covers this: trip before send_future completes, then accepted → cancel_goal_async is called exactly once.",
      "The wait for the cancel is bounded by timeouts.cancel_settle_s. An unacknowledged or rejected cancel prints 'PRESS THE HARDWARE E-STOP' and returns estopped/aborted. A unit test with a never-done cancel future proves execute() returns within cancel_settle_s plus a small margin and prints the message.",
      "The stuck-goal threshold is scaled_traj's last t_s plus timeouts.goal_s. A unit test using the shipped goal_s=5.0 and a scaled trajectory longer than 5 s that tracks perfectly returns 'completed'.",
      "After SUCCESSFUL, execute() checks fresh joint_states against the final point within a stated tolerance and returns 'aborted' on mismatch or stale data. A unit test covers both the arrived and not-arrived cases.",
      "All acceptance criteria of MOT-05.4 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 5,
    "ambiguity": 3,
    "context": 4,
    "consequence": 5,
    "task_class": "ros2-moveit-integration",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 99,
  "repairs": "MOT-05.4",
  "findings": [
    "cee9f4e2614d7fff",
    "602b730548989646",
    "40cf38d1b1438814",
    "b95c6858700dcaa2",
    "81273d3879974ef9"
  ],
  "id": "MOT-05.4.R1",
  "title": "Repair MOT-05.4: Mode/profile table taken from overridable --execution-config lets mock",
  "parent": "MOT-05",
  "outcome": "Resolve the review findings on MOT-05.4 while every acceptance criterion of MOT-05.4 still holds."
}
```

Repair work generated deterministically from review attempt `00318-MOT-05.4-review` of `MOT-05.4`.
Original card: `plan/cards/MOT-05.4.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] Mode/profile table taken from overridable --execution-config lets mock mode send goals to the real driver with every real-only gate skipped
- Evidence: execute_trajectory.py:568 `if mode not in profiles[profile]["modes"]` uses exec_cfg from args.execution_config. execution_gate.py:192-194 only checks that modes is a non-empty subset of MODES. Probe /tmp/audit_mot054/probe.py: a config copy with vendor `modes: [mock, dry, real]`, run(mode='mock', profile='vendor', env={}) → 'probe1 result {'status': 'ok'} execute called with action ['/rebotarm/follow_joint_trajectory']'. check_graph treats profile 'vendor' with its real-driver branch and passes when reBotArmController hosts the action.
- Consequence: A CLI flag (--execution-config) lets real-arm motion skip G-ARM, G-COMMISSIONING, G-ESTOP and G-CONFIRM. This breaks the criterion 'There is no CLI flag or env that skips a gate… Real mode allows only the vendor profile, and mock mode only mock profiles' and ADR-016 §2/§9. The same applies to action names and expected_node, which are also read from config.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py
- Acceptance condition: The executor enforces a hard-coded §11.1 table: real is allowed only with profile 'vendor' and mock only with moveit_mock/vendor_mock, whatever execution.yaml says. A config whose modes, action or expected_node differ from §11.1 is rejected with exit 2. A unit test shows that run(mode='mock', profile='vendor') with such a config raises ConfigError and never calls execute().

## Finding 2 [blocker] Trip while the send_goal request is pending returns without cancelling; the goal can then run unmonitored
- Evidence: execute_trajectory.py:489-495: the loop stops spinning once trip is set; `goal_handle = send_future.result() if send_future.done() else None`; `if trip["kind"] is not None: return trip["kind"], trace`. No cancel is sent and nothing waits for the pending goal response. Probe B (probe3.py B): e-stop at spin 3, goal accepted at spin 50 → 'returned estopped elapsed=0.06 cancels= 0'.
- Consequence: If an e-stop, SIGINT or SIGTERM arrives in the window between send and accept, the record says 'estopped'. The driver then accepts the goal and runs the full trajectory, with the node destroyed and no monitoring. This is the opposite of ADR-016 §7.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py
- Acceptance condition: After a trip during the pending send, execute() keeps waiting (bounded) for the goal response. If the goal is accepted it cancels it and runs the settle check; if no response arrives in time it prints 'PRESS THE HARDWARE E-STOP'. A unit test covers this: trip before send_future completes, then accepted → cancel_goal_async is called exactly once.

## Finding 3 [major] Unacknowledged cancel blocks forever and never prints PRESS THE HARDWARE E-STOP
- Evidence: execute_trajectory.py:523-525 `cancel_future = goal_handle.cancel_goal_async()` / `while not cancel_future.done(): rclpy.spin_once(...)` has no deadline, and the cancel response's return_code is never inspected. Probe A: cancel future never done → 'STILL BLOCKED after 8s … cancels= 1' and no e-stop message printed.
- Consequence: If the driver is dead or hung, which is exactly when a cancel goes unacknowledged, §11.8 step 3 requires 'PRESS THE HARDWARE E-STOP' and exit 1. Instead the executor hangs silently and writes no record.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py
- Acceptance condition: The wait for the cancel is bounded by timeouts.cancel_settle_s. An unacknowledged or rejected cancel prints 'PRESS THE HARDWARE E-STOP' and returns estopped/aborted. A unit test with a never-done cancel future proves execute() returns within cancel_settle_s plus a small margin and prints the message.

## Finding 4 [major] timeouts.goal_s used as an absolute limit instead of scaled duration + goal_s; real-speed runs always abort
- Evidence: execute_trajectory.py:519 `if elapsed > timeouts["goal_s"]:`. config/motion/execution.yaml: '# Scaled trajectory duration + this = stuck-goal cancel threshold' with goal_s: 5.0. §11.12 says 'scaled trajectory duration + 5 s'. Probe C: a 30 s scaled trajectory (the 3 s smoke fixture at real_default 0.10) tracked perfectly → 'returned aborted elapsed=5.22 cancels= 1'. The unit test sidesteps this by using goal_s=30.0.
- Consequence: Every real or mock run whose scaled duration is over 5 s is cancelled mid-motion and exits 1. A real execution at the commissioned speed can never complete.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py, ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py
- Acceptance condition: The stuck-goal threshold is scaled_traj's last t_s plus timeouts.goal_s. A unit test using the shipped goal_s=5.0 and a scaled trajectory longer than 5 s that tracks perfectly returns 'completed'.

## Finding 5 [major] No arrival verification after a successful result
- Evidence: execute_trajectory.py:533-536 returns 'completed' whenever result.error_code == SUCCESSFUL. It never compares the final joint_states with the last trajectory point. The card asks execute() to 'verif[y] arrival within tolerance at the end', and §11.10 exit 0 = 'goal completed and arrival verified'.
- Consequence: A driver that reports SUCCESSFUL without reaching the target gives exit 0 and outcome 'completed', which is false evidence in a commissioning record.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py
- Acceptance condition: After SUCCESSFUL, execute() checks fresh joint_states against the final point within a stated tolerance and returns 'aborted' on mismatch or stale data. A unit test covers both the arrived and not-arrived cases.

# MOT-05.7 — execute_trajectory: subscribe to joint_states with sensor-data (BEST_EFFORT) QoS and separate the DDS-discovery wait from the staleness window

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-05.4"
  ],
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
      "Every joint_states subscription in execute_trajectory.py uses a BEST_EFFORT profile, e.g. rclpy.qos.qos_profile_sensor_data. That covers the non-latched _wait_for_message used by G-START-STATE and js_sub in execute(). A BEST_EFFORT reader matches both the BEST_EFFORT vendor publishers (reBotArmController: rebotarm_controller.py:9,24 and ros_publishers.py:22-27; rebot_motion mock_driver.py:53-54) and the RELIABLE ros2_control joint_state_broadcaster on /joint_states. The latched /rebotarm/arm_status subscription stays RELIABLE + TRANSIENT_LOCAL, matching ros_publishers.py:37-47. The e-stop Bool subscriptions are unchanged. A unit test with a fake node asserts the reliability policy each subscription is created with.",
      "G-START-STATE separates DDS discovery from staleness. It waits for the FIRST joint_states message up to a discovery bound (--service-timeout-s, or a named constant of at least 5 s documented in the module), then applies timeouts.joint_state_s only to the age of the message it uses. A missing message after the discovery bound refuses with a message naming the bound; a too-old message refuses as stale. The latched arm_status wait uses the same discovery bound instead of min(joint_state_s, 5.0). Unit tests use a fake that delivers its first message after 1.5 s (passes with the shipped joint_state_s = 0.5) and a fake whose only message is older than joint_state_s (fails as stale).",
      "During execution (mock/real), staleness monitoring is unchanged: after the first message, a gap longer than timeouts.joint_state_s trips the §11.8 e-stop path. The QoS change does not weaken it, and an existing or new unit test pins that.",
      "scripts/ros/test_joint_states_qos.sh, modelled on test_reachability.sh (own process group, trap teardown, no strays): refuses with exit 3 before starting anything if a real driver process is running (pgrep -x reBotArmControl). It starts `ros2 run rebot_motion mock_driver` from the read-only ~/rebot_ws underlay. With an inline python3 probe it then (a) proves the executor's own joint_states subscription helper receives /rebotarm/joint_states within the discovery bound, and (b) shows that an explicitly RELIABLE subscription receives nothing in the same window, documenting the bug this card fixes. It exits 0 only if both hold. Keep ROS_LOCALHOST_ONLY=1 and the inherited ROS_DOMAIN_ID.",
      "Nothing else changes: still exactly one ActionClient( call site, no vendor service names, the §11 gate ids, modes, profiles and exit codes, and the record format. All MOT-05.4 and MOT-05.4.R1 acceptance criteria still hold.",
      "Optional, if it reproduces: MOT-05.5 attempt 00393 saw one G-COLLISION refusal of the home waypoint (index 0) on the moveit_mock stack in 7 runs (suspected race between apply_scene and /check_state_validity). If the cause is in execute_trajectory.py, fix it here with a test. Otherwise put the diagnosis in the result and change nothing for it."
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
  "priority": 90,
  "id": "MOT-05.7",
  "parent": "MOT-05",
  "title": "execute_trajectory: subscribe to joint_states with sensor-data (BEST_EFFORT) QoS and separate the DDS-discovery wait from the staleness window",
  "outcome": "execute_trajectory receives /rebotarm/joint_states from the real reBotArmController and from rebot_motion's mock_driver, both of which publish BEST_EFFORT. A fresh node no longer refuses G-START-STATE because DDS discovery took longer than joint_state_s. This unblocks MOT-05.5 and MOT-09's `--mode dry --profile vendor` rehearsal."
}
```

Operator-filed 2026-10-10 (overnight session, on the operator's instruction) to resolve MOT-05.5's `dependency` block.
Before the fix, `execute_trajectory --mode dry --profile vendor` could never pass G-START-STATE against the real
arm, which is the rehearsal MOT-09 needs.

## Evidence

- MOT-05.5 attempt 00393 (`.claude-auto/attempts/00393-MOT-05.5-impl/out/run.json`) found it. The executor
  subscribes to joint_states with `qos = 10` (RELIABLE, depth 10) in `_wait_for_message`
  (`execute_trajectory.py:327-342`) and in `execute()` (`js_sub`, `execute_trajectory.py:553`). The vendor_mock
  smoke case therefore always refused, and rclpy logged "offering incompatible QoS ... RELIABILITY". The same
  attempt saw "no joint_states received within 0.5s" in 2 of 7 runs. That flake comes from `start_state()`
  passing `joint_state_timeout_s` (0.5 s) as the whole wait for the first message.
- Verified read-only on 2026-10-10 for the real driver, which the attempt only presumed: `reBotArmController`
  sets `self.sensor_qos = qos_profile_sensor_data` (`~/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:9,24`).
  It publishes `/{ns}/joint_states` and the per-joint states with it (`ros_publishers.py:22-36`) and `arm_status`
  latched RELIABLE (`ros_publishers.py:37-47`).

## Implementation notes

- Keep the change minimal. Swap the QoS on the joint_states subscriptions. Add an explicit discovery bound to
  `_wait_for_message`'s callers, threading it from `--service-timeout-s` through
  `OnlineChecks.start_state`. Base staleness on the message age, as today, plus the receive time where the
  publisher stamp is unusable. Do not change §11 semantics or `config/motion/execution.yaml`.
- Unit tests must not need a running ROS graph; reuse the existing fakes. Only `test_joint_states_qos.sh`
  starts ROS processes.
- MOT-05.5 depends on this card, and its integration script should then pass unchanged.

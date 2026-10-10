# MOT-05.5 — Headless mock verification of the execution gate: dry/mock execution, refusals, collision, e-stop injection, vendor-interface mock

```json card
{
  "kind": "integration",
  "depends_on": [
    "MOT-05.4",
    "MOT-05.7"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "scripts/ros/test_execution.sh",
      "ros2_ws/src/crackvision_motion/test/fixtures/execution_test_*.yaml",
      "ros2_ws/src/crackvision_motion/test/fixtures/execution_test_*.json"
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
      "Before anything starts, the script refuses (exit 3) if a real vendor driver process (pgrep -x reBotArmControl) is running. Every real-mode invocation in the script runs detached from the controlling terminal (setsid -w … </dev/null) with a test execution config, so even an unexpected gate pass could not reach a real driver.",
      "Offline: real mode with repo defaults gives exit 3, and the record/§0.4 json lists the expected refusal ids. The overspeed fixture gives exit 3 in mock mode with no ROS contact.",
      "MoveIt mock stack with the production scene applied: dry mode exits 0 with joint_states unchanged (max |Δq| < 1e-4) before and after. Mock trajectory_smoke exits 0, the final state is within tolerance of the last point, and the record has status ok. Rerunning the same trajectory (start mismatch) gives exit 3 with no motion. trajectory_estop with /crackvision/estop published after ~2 s gives exit 1 and a final state strictly short of the goal, with record outcome e-stop. Last, with scene_collision_smoke.yaml applied, a trajectory starting at the current state gives exit 3 (G-COLLISION) with no motion.",
      "Vendor-interface mock: ros2 run rebot_motion mock_driver (from the read-only ~/rebot_ws/install underlay). Mock mode with profile vendor_mock reaches the goal. Real mode against it, armed and with the measured fixture set, is refused by G-GRAPH (exit 3) with no motion. If mock_driver needs /rebotarm/enable, the test script calls it on the mock; the executor never does.",
      "Each case asserts the executor's exit code and the record's gate ids/outcome, not just log text. At the end, no process from the launched process groups survives (asserted, as in test_reachability.sh)."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 4,
    "consequence": 4,
    "task_class": "ros2-integration-test",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-05.5",
  "parent": "MOT-05",
  "title": "Headless mock verification of the execution gate: dry/mock execution, refusals, collision, e-stop injection, vendor-interface mock",
  "outcome": "scripts/ros/test_execution.sh proves the executor end to end without hardware. Real mode is refused offline, and against a mock driver it is refused by the graph check. Dry mode leaves the arm state unchanged. Mock mode reaches the goal on the MoveIt mock stack and on rebot_motion's vendor-interface mock_driver. Start-state mismatch, overspeed and collision are refused with no motion, an injected e-stop stops mid-trajectory, and no stray processes survive."
}
```

Model the script on scripts/ros/test_scene.sh and test_reachability.sh: source env_ros.sh, source the overlay (exit 3 with a hint if it is not built), use _mock_stack.sh start/stop with traps, and write logs under ros2_ws/log/ and records to a temp --record-dir under logs/. Do not modify accepted scripts or the executor; if the executor has a bug, fail loudly and report it rather than working around it. Sequence the trajectories so each start state matches: home→P (smoke), P→Q (e-stop, slow), then a short collision trajectory from wherever the e-stop left the arm. Generate that last one at runtime from the live joint_states with a small inline python3 snippet that writes a temp crackvision.joint_trajectory/1 file. Read joint states for the assertions with a short rclpy helper (inline python3 heredoc), not by parsing `ros2 topic echo` text. Tear down the MoveIt mock stack before starting mock_driver, so /joint_states and /rebotarm/joint_states cannot be confused. The vendor_mock real-mode refusal still needs the online services. Either keep the mock stack up for that sub-case or assert that the refusal happens at G-GRAPH before any service wait; pick whichever matches ADR-016's check order and document the choice in the script header. Keep ROS_LOCALHOST_ONLY=1 and the inherited ROS_DOMAIN_ID.

## 2026-10-10 operator note (overnight session): dependency block resolved by MOT-05.7
Attempt 00393 blocked this card on two `execute_trajectory.py` defects outside its scope: RELIABLE joint_states
QoS versus the BEST_EFFORT vendor publishers, and a 0.5 s first-message wait that was shorter than DDS discovery.
MOT-05.7 fixes both in MOT-05.4's code, and this card now depends on it. Keep the attempt-00393 script, which
should pass unchanged once MOT-05.7 is accepted. If the moveit_mock G-COLLISION refusal at waypoint 0 recurs,
report it rather than retrying around it. The real reBotArmController also publishes joint_states BEST_EFFORT
(`rebotarm_controller.py:9,24`; `ros_publishers.py:22-27`), so the same fix is what makes MOT-09's
`--mode dry --profile vendor` rehearsal possible.

# MOT-05.4 — execute_trajectory ROS node/CLI: online read-only checks, mock/dry/real execution, typed confirmation, e-stop + tracking monitor, execution record

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-05.3"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py",
      "ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/package.xml"
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
      "Order is enforced in code: offline gate, then rclpy.init, online checks, (real) confirmation, then the single ActionClient. An offline refusal never calls rclpy.init (unit-tested by monkeypatching rclpy.init to raise).",
      "The default mode is dry, and dry mode never constructs an ActionClient (unit-tested). There is no CLI flag or env that skips a gate, the collision check or the confirmation. Real mode allows only the `vendor` profile, and mock mode only mock profiles.",
      "Graph discrimination follows §11: mock refuses if the real driver node is visible, and real refuses if a mock driver is the server or the expected real node is absent.",
      "Start state: fresh joint_states within timeout, every arm joint within start tolerance of point 0, otherwise exit 3. Collision: every densified waypoint is checked via /check_state_validity (group arm, explicit robot_state), the scene objects named by the scene config are verified present via /get_planning_scene, and any invalid waypoint gives exit 3 with its indices.",
      "Confirmation reads only from /dev/tty. If it cannot be opened, the result is exit 3. A wrong phrase gives exit 3 and no goal is sent. Both are unit-tested with an injected tty opener; no test ever reaches goal sending in real mode.",
      "Monitoring: /crackvision/estop and /rebot_motion/estop (Bool true), SIGINT/SIGTERM, joint_states staleness and tracking error over tolerance each cancel the goal and give exit 1. The post-cancel behaviour follows ADR-016.",
      "The goal holds the speed-scaled trajectory (joint1..6 only, time_from_start from the scaled t_s). The JointTrajectory conversion is unit-tested.",
      "A crackvision.execution_record/1 is written in every non --dry-run invocation, refusals included (in a finally block), alongside the §0.4 log/json via cli_common.run_cli. `--dry-run` runs the offline gate only, writes nothing and exits 0. Exit codes follow §11."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 5,
    "task_class": "ros2-moveit-integration",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-05.4",
  "parent": "MOT-05",
  "title": "execute_trajectory ROS node/CLI: online read-only checks, mock/dry/real execution, typed confirmation, e-stop + tracking monitor, execution record",
  "outcome": "`ros2 run crackvision_motion execute_trajectory` runs the MOT-05.3 offline gate first, so a refusal never initialises ROS. It then runs the online read-only checks (graph/profile discrimination, fresh start state, /check_state_validity on densified waypoints with the scene objects present). Dry mode stops there without constructing an ActionClient. Mock mode executes on a mock profile. Real mode additionally needs the /dev/tty typed confirmation and executes with e-stop/staleness/tracking monitoring. An execution record and §0.4 logs are always written."
}
```

This is the only code path in the repo that can command arm motion. It follows ADR-016 and §11 exactly, and is modelled on ~/rebot_ws/deploy/run_real_demo.sh and rebot_motion/motion_runner.py (read-only references: arming env plus typed tty confirmation, refusal without a tty, teardown on any exit, cancel on e-stop, arrival verification).

Structure (keep functions small and testable without ROS):
- `main()` → cli_common.run_cli with the §0.3 flags plus --mode {mock,dry,real} (default dry), --trajectory (required), --profile (default per §11), --execution-config, --commissioning, --approval, --speed-scale, --limits, --scene-config, --end-effector-config, --service-timeout-s, --record-dir (default logs/execution).
- `run(args, env, tty_opener=open_dev_tty)`: offline gate (execution_gate.evaluate_offline, env passed explicitly). On failure raise PreconditionError after writing the record.
- `OnlineChecks` (rclpy node): graph check via get_node_names_and_namespaces and the action-server node lookup; joint_states subscription on the profile topic; /check_state_validity and /get_planning_scene clients. The services come from the MOT-02 mock_planning stack, which per ADR-016 is also the real-mode validity oracle.
- `confirm_real(report, traj_sha, tty_opener)`: print the summary (sha, scaled duration, scale, per-joint max velocity, e-stop instructions) and require execution_gate.confirmation_ok.
- `execute(node, profile, scaled_traj)`: the ONLY place an ActionClient is constructed. It sends a FollowJointTrajectory goal, monitors e-stop topics, signals, staleness and tracking at ≥ 20 Hz, cancels on any trip and verifies arrival within tolerance at the end.
- No calls to any vendor Trigger service except what ADR-016 explicitly allows after an e-stop cancel. The grep check forbids enable/safe_home/park/move_to_pose/gravity_compensation/set_zero/set_mode/gripper.
Add the console script `execute_trajectory` to setup.py, and add exec_depends for control_msgs, trajectory_msgs, sensor_msgs, std_msgs and moveit_msgs to package.xml if missing. Unit tests must not need a running ROS graph; use fakes for node/clients where needed. The live mock behaviour is proven in MOT-05.5.

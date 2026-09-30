# GEOM-08.3 — FK parity: crackvision.kinematics vs live MoveIt /compute_fk (gripper_link, tool_tip, camera_link) on the mock stack

```json card
{
  "kind": "integration",
  "depends_on": [
    "GEOM-08.2",
    "MOT-02",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-GEOM-2"
  ],
  "scope": {
    "write": [
      "scripts/ros/fk_parity_dump.py",
      "scripts/ros/test_fk_parity.sh",
      "scripts/geometry/compare_fk_parity.py"
    ]
  },
  "inputs": [
    "scripts/ros/_mock_stack.sh",
    "scripts/ros/env_ros.sh",
    "scripts/ros/test_end_effector.sh",
    "ros2_ws/src/crackvision_motion/crackvision_motion/check_end_effector.py",
    "src/crackvision/kinematics.py",
    "config/robot/b601_dm_limits.yaml",
    "config/robot/end_effector.yaml",
    "docs/INTERFACES.md#8"
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
        "id": "fk-parity",
        "cmd": "bash scripts/ros/test_fk_parity.sh",
        "timeout_s": 900,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The dump is read-only: it calls only /compute_fk. It never calls execute or trajectory actions, and never touches controllers.",
      "The joint set is q=0 plus 20 seeded uniform samples inside b601_dm_limits.yaml, identical between the dump and the compare step (the seed is recorded in the JSON).",
      "The compare step runs under ./env.sh python and reports max position and angular error per link. It exits 1 on any excess, and prints the per-link maxima on success.",
      "The script tears the mock stack down and asserts no stray processes, following the test_end_effector.sh pattern.",
      "No files outside scope are changed. No package entry points are added."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "ros-integration-test",
    "local_ok": false
  },
  "track": "simulation",
  "id": "GEOM-08.3",
  "parent": "GEOM-08",
  "title": "FK parity: crackvision.kinematics vs live MoveIt /compute_fk (gripper_link, tool_tip, camera_link) on the mock stack",
  "outcome": "A headless script shows the pure-numpy FK chain (plus the end_effector.yaml transforms) agrees with MoveIt's own FK of the planning model on 21 joint states, to ≤ 1e-6 m and ≤ 1e-6 rad, for gripper_link, tool_tip and camera_link. This is independent evidence that 3D paths are placed in the same base_link MoveIt plans in."
}
```

1. `scripts/ros/fk_parity_dump.py` (system /usr/bin/python3, rclpy).
   - Args: `--out PATH --seed N --samples 20 --limits config/robot/b601_dm_limits.yaml`.
   - Build the joint set: all-zero plus 20 seeded uniform draws within each joint's [lower, upper] (numpy default_rng(seed)).
   - For each set, call /compute_fk with header.frame_id base_link and fk_link_names [gripper_link, tool_tip, camera_link]. Set the gripper joints to 0; they don't affect these links.
   - Write JSON {seed, joint_names, samples: [{q, links: {name: {xyz, quat_xyzw}}}]}.
   - Exit codes follow §0.2: 3 if the service is unavailable within 30 s, 1 on an FK error code.
2. `scripts/geometry/compare_fk_parity.py` (./env.sh python).
   - Load the dump. For each sample compute `crackvision.kinematics` FK(q), then ·T_gripper_link_tool_tip and ·T_gripper_link_camera_link from the repo's end_effector.yaml.
   - Compare: position error ≤ 1e-6 m; angle of R_movitᵀ·R_ours ≤ 1e-6 rad.
   - Print a per-link max table. Exit 0 if all pass, 1 otherwise, 2 on a malformed dump.
3. `scripts/ros/test_fk_parity.sh`, modelled on scripts/ros/test_end_effector.sh.
   - Source env_ros.sh (`set +u` around sourcing) and the overlay setup; exit 3 if not built.
   - start_mock_stack → run the dump to ros2_ws/log/fk_parity.json → stop_mock_stack → assert no stray PGID → run `"$REPO_ROOT/env.sh" python scripts/geometry/compare_fk_parity.py ros2_ws/log/fk_parity.json`.
   - Propagate the exit code.

Keep ROS_LOCALHOST_ONLY=1 and the existing ROS_DOMAIN_ID. MoveIt's model loads the vendor URDF from ~/rebot_ws through the overlay xacro, so this also re-confirms that the repo copy matches the vendor kinematics. If parity fails, do not tweak tolerances: report the per-link numbers and stop.

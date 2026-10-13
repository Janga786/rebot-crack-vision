# MOT-08.6 — preview_trajectory ROS node: DisplayTrajectory + tool-path/crack MarkerArray, latched, motion-free

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-08.1"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/preview_trajectory.py",
      "ros2_ws/src/crackvision_motion/test/test_preview_trajectory.py",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/package.xml"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#18",
    "docs/INTERFACES.md#11",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/tool_fk.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "ros2_ws/src/crackvision_motion/setup.py"
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
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; python3 -m pytest ros2_ws/src/crackvision_motion/test/test_preview_trajectory.py -q -p no:cacheprovider'",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "motion-free",
        "cmd": "bash -c '! grep -nE \"ActionClient|create_client|follow_joint_trajectory|/rebotarm|joint_states|send_goal\" ros2_ws/src/crackvision_motion/crackvision_motion/preview_trajectory.py'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "help",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; ros2 run crackvision_motion preview_trajectory --help'",
        "timeout_s": 120,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Pure builder functions are tested without a ROS graph. build_display_trajectory has model_id 'rebotarm', the joint_names in §11.2 order, time_from_start = t_s / speed_scale, and trajectory_start set to points[0]. build_markers produces a LINE_STRIP of the tool-tip path from crackvision_motion.tool_fk in base_link, and crack polylines when a sha-verified paths3d file is given.",
      "Publishers use TRANSIENT_LOCAL + RELIABLE QoS on exactly /display_planned_path and /crackvision/preview_markers. --once publishes, waits up to --wait-s for a subscriber, and exits 0. Without --once the node republishes until SIGINT.",
      "The trajectory is validated with joint_trajectory.load_trajectory. An invalid file exits 3 and a bad flag exits 2, per §0.2. --speed-scale defaults to execution.yaml speed_scale.real_default.",
      "setup.py gains only the console script. package.xml gains only the exec_depends this node needs (moveit_msgs is already present; visualization_msgs and geometry_msgs if used). Existing entry points are unchanged."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "ros-node",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-08.6",
  "title": "preview_trajectory ROS node: DisplayTrajectory + tool-path/crack MarkerArray, latched, motion-free",
  "parent": "MOT-08",
  "outcome": "`ros2 run crackvision_motion preview_trajectory --trajectory PATH` publishes a latched moveit_msgs/DisplayTrajectory on /display_planned_path and a MarkerArray on /crackvision/preview_markers, built by pure, unit-tested functions. It never creates an action or service client."
}
```

Add crackvision_motion/preview_trajectory.py, an rclpy node plus CLI, per INTERFACES §18.

- Split the module into pure functions and a node shell:
  - `build_display_trajectory(traj: dict, speed_scale: float) -> moveit_msgs.msg.DisplayTrajectory`, which sets `trajectory[0].joint_trajectory` and `trajectory_start.joint_state`.
  - `build_markers(traj, tool_xyz, paths3d_doc|None, frame='base_link') -> visualization_msgs.msg.MarkerArray`, with a namespace per marker type and stable ids.
  - `main()` does argparse, loading, publishing and exit codes.
- Use crackvision_motion.tool_fk for the tool-tip positions. Do not import crackvision (conda).
- QoS: `QoSProfile(depth=1, durability=TRANSIENT_LOCAL, reliability=RELIABLE)`.
- Flags: --trajectory (required), --paths3d (optional; verified against the trajectory's source.paths3d.sha256 and ignored with a warning on mismatch), --speed-scale, --execution-config, --end-effector-config, --urdf, --once, --wait-s (default 10), --root, plus whatever §0.3 flags cli_common already provides.
- This node must be motion-free by construction: no ActionClient, no service clients, no subscriptions or publications on joint_states or /rebotarm topics.

Tests (system python3, with ROS sourced) cover the pure builders only: field values, time scaling, joint order, the marker count and frame, paths3d sha-mismatch handling, and invalid-trajectory exit 3 via main().

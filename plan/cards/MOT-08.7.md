# MOT-08.7 — RViz preview launch + rviz config (no MotionPlanning display) + headless publish smoke on the mock stack

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-08.6"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/launch/preview_rviz.launch.py",
      "ros2_ws/src/crackvision_motion/rviz/preview.rviz",
      "ros2_ws/src/crackvision_motion/setup.py",
      "scripts/ros/test_preview_rviz.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#18",
    "ros2_ws/src/crackvision_motion/launch/mock_planning.launch.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/preview_trajectory.py",
    "scripts/ros/_mock_stack.sh",
    "scripts/ros/test_view_retime_execute.sh"
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
        "id": "rviz-config",
        "cmd": "bash -c 'f=ros2_ws/src/crackvision_motion/rviz/preview.rviz; python3 -c \"import yaml,sys; yaml.safe_load(open(sys.argv[1]))\" $f && grep -q \"moveit_rviz_plugin/Trajectory\" $f && grep -q \"/display_planned_path\" $f && grep -q \"/crackvision/preview_markers\" $f && ! grep -q \"MotionPlanning\" $f'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "launch-args",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; ros2 launch crackvision_motion preview_rviz.launch.py --show-args'",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "smoke",
        "cmd": "bash scripts/ros/test_preview_rviz.sh",
        "timeout_s": 900,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The launch file reuses mock_planning.launch.py through IncludeLaunchDescription rather than copying it. It starts rviz2 with the installed preview.rviz and the preview_trajectory node, has a `rviz:=false` launch argument for headless use, and declares a `trajectory` argument.",
      "preview.rviz contains only RobotModel, moveit_rviz_plugin/Trajectory (topic /display_planned_path, looped), PlanningScene and MarkerArray (/crackvision/preview_markers), with fixed frame base_link. There is no MotionPlanning display and no execute button.",
      "test_preview_rviz.sh follows test_view_retime_execute.sh: it refuses if reBotArmControl is running, never sets CRACKVISION_ARM_REAL, tears down with trap, and writes only under logs/. It runs the node with --once on a committed fixture trajectory (test/fixtures/trajectory_smoke.json). A late-joining rclpy subscriber receives a DisplayTrajectory with the same point count, and a MarkerArray. The script also asserts that /joint_states is unchanged across the run (no motion).",
      "setup.py installs launch/*.launch.py (already globbed) and rviz/*.rviz under share/crackvision_motion/rviz."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "ros-integration",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-08.7",
  "title": "RViz preview launch + rviz config (no MotionPlanning display) + headless publish smoke on the mock stack",
  "parent": "MOT-08",
  "outcome": "`ros2 launch crackvision_motion preview_rviz.launch.py trajectory:=PATH` brings up the mock planning stack, rviz2 with rviz/preview.rviz and the preview node. scripts/ros/test_preview_rviz.sh proves headlessly that /display_planned_path carries the trajectory."
}
```

Add launch/preview_rviz.launch.py, rviz/preview.rviz and the setup.py data_files entry for rviz/, plus scripts/ros/test_preview_rviz.sh.

Launch: declare the args `trajectory` (required), `paths3d` (default ''), `speed_scale` (default '' → node default) and `rviz` (default 'true'). Include mock_planning.launch.py (it needs CRACKVISION_ROOT, as today). Start `rviz2 -d <share>/rviz/preview.rviz` under `IfCondition(rviz)`. Start preview_trajectory without --once.

The host has no DISPLAY and no Xvfb, so RViz itself cannot be exercised in CI. The smoke script uses the mock stack (start_mock_stack from _mock_stack.sh) and runs the node directly with --once. It verifies with a short python3 rclpy subscriber, using TRANSIENT_LOCAL QoS so it receives the latched messages, and checks:
- DisplayTrajectory point count equals len(points) in the fixture;
- joint_names match;
- MarkerArray is non-empty;
- the /joint_states position sampled before and after differs by < 1e-6.
Also run `ros2 launch ... --show-args` in the script. In the script header, document the manual step for an operator with a display: `ros2 launch crackvision_motion preview_rviz.launch.py trajectory:=...`. Print a PASS summary.

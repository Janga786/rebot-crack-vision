# MOT-06.9 — plan_view node/CLI + headless smoke: reachable, collision-free camera view pose with the specimen in FOV

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-06.8",
    "MOT-06.4",
    "MOT-06.5",
    "MOT-06.6"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/plan_view.py",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/test/test_plan_view.py",
      "scripts/ros/test_view_plan.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#0",
    "ros2_ws/src/crackvision_motion/crackvision_motion/view_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
    "ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke/**",
    "scripts/ros/_mock_stack.sh"
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
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_plan_view.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "smoke",
        "cmd": "bash scripts/ros/test_view_plan.sh",
        "timeout_s": 1200,
        "expect_exit": 0
      },
      {
        "id": "regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The view configuration is accepted only after an independent /compute_fk(camera_link) agrees with the target within §13 tolerance, /check_state_validity passes with the scene applied, and the inclusive limits hold; the transit from home is continuous and every configuration is valid.",
      "The smoke script asserts from the file plus its own FK call: camera_link at 0.25 ± 0.002 m from the specimen top centre along the optical axis; optical-axis angle to −n ≤ 15° + 0.1°; FOV check passed for all four corners. An --center far outside reach exits 3 with candidates' reasons listed.",
      "--dry-run lists candidates and FOV results with no rclpy.init and no writes (tested); usage/config errors exit 2 (tested); no trajectory is ever sent to a controller (no execution client in the module).",
      "setup.py keeps every existing entry point and adds plan_view only (plan_crack_path from MOT-06.6 untouched)."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "ros-node",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-06.9",
  "title": "plan_view node/CLI + headless smoke: reachable, collision-free camera view pose with the specimen in FOV",
  "parent": "MOT-06",
  "outcome": "`ros2 run crackvision_motion plan_view` picks the first view candidate that passes the FOV check, IK for camera_link, independent FK/validity/limits re-check and an OMPL transit from home, writes a validated crackvision.view_plan/1, and scripts/ros/test_view_plan.sh proves it on the mock stack at the recommended placement."
}
```

Write the node per §13's view-plan section, reusing view_core, cartesian_moveit and cartesian_plan. Output goes to `--out`, defaulting to the §13 path. Register `plan_view = crackvision_motion.plan_view:main`.

The smoke uses the MOT-06.7 fixture scene (specimen at the recommended placement; reuse that fixture read-only) and the mock stack via _mock_stack.sh. It covers a positive case and an unreachable --center case.

The view plan is the pose for the §9 capture. It does not capture or move anything.

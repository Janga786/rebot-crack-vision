# MOT-02 — ROS 2 overlay workspace + headless MoveIt mock planning

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-01",
    "ARCH-01"
  ],
  "requirements": [
    "REQ-MOT-1"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/**",
      "scripts/ros/**",
      ".gitignore",
      "docs/motion/ROS_WORKSPACE.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 1500
      },
      {
        "id": "mock-plan",
        "cmd": "bash scripts/ros/test_mock_plan.sh",
        "timeout_s": 900
      }
    ],
    "criteria": [
      "scripts/ros/env_ros.sh strips conda vars, uses `set +u`, sources /opt/ros/humble then ~/rebot_ws/install (underlay, read-only), keeps ROS_LOCALHOST_ONLY=1",
      "ros2_ws/src/crackvision_motion (ament_python) builds with colcon; build/install/log ignored by git",
      "test_mock_plan.sh launches move_group with ros2_control mock components, no RViz, plans a joint goal, shuts everything down (no stray processes), exit 0",
      "~/rebot_ws is never modified"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 3
  },
  "track": "simulation",
  "timeout_min": {
    "impl": 90
  },
  "id": "MOT-02",
  "title": "ROS 2 overlay workspace + headless MoveIt mock planning",
  "parent": "L-MOTION",
  "outcome": "This repo builds a ROS 2 overlay on ~/rebot_ws and can plan with MoveIt against mock hardware headlessly."
}
```



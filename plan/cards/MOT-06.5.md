# MOT-06.5 — MoveIt client layer for Cartesian/transit/IK/FK/validity requests (cartesian_moveit)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-06.2",
    "MOT-06.1"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py",
      "ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "docs/motion/CARTESIAN_PROBE.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_apply.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py"
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
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py -q -p no:cacheprovider",
        "timeout_s": 300,
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
      "Request builders set avoid_collisions true unconditionally (there is no parameter to disable it), use group `arm`, frame base_link and the configured link_name, and set max_step and jump thresholds from the §13 config using the field names MOT-06.1 found. Tested by inspecting the built messages.",
      "The response parsers map MoveIt error codes to the §13 reason vocabulary, and timestamps from MoveIt are discarded. The last reached waypoint is determined by FK matching within §13 tolerances, never inferred from `fraction` alone. Tested with synthetic response messages.",
      "Service calls have timeouts; a missing service or timeout surfaces as a distinct error (maps to exit 3 or 1 per §13), never as 'unreachable'.",
      "Unit tests construct messages without a running ROS graph (no rclpy.init) and pass under the env_ros system python3.",
      "Shared helpers are reused from reachability_sweep/scene_apply/cli_common where suitable, not copy-pasted (or the duplication is justified in a comment)."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "ros-client",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-06.5",
  "title": "MoveIt client layer for Cartesian/transit/IK/FK/validity requests (cartesian_moveit)",
  "parent": "MOT-06",
  "outcome": "crackvision_motion.cartesian_moveit builds GetCartesianPath (link_name tool_tip, avoid_collisions true), GetMotionPlan (joint goal, explicit start_state), IK, FK and state-validity requests and parses their responses into plain Python results (configs, last reached waypoint by FK match, error codes), with message-level unit tests and no collision relaxation possible through its API."
}
```

A thin, testable layer between the pure core and the node.

Request builders:
- `build_cartesian_request(start_positions, poses, link_name, cfg)`
- `build_motion_plan_request(start_positions, goal_positions, cfg)`, with OMPL and the planner id from config
- `build_ik_request(pose, seed, link_name, cfg)`, reusing the `reachability_sweep._build_ik_request` logic where possible
- `build_fk_request(positions, links)`
- `build_validity_request(positions)`

Parsers:
- `parse_cartesian(resp)`: returns `(configs, fraction, error)`
- `parse_motion_plan(resp)`
- `parse_ik(resp)`
- `match_reached_waypoints(fk_poses, target_poses, tol)`: returns `last_index` and errors

Also write a small `MoveItClients` class: create clients on a given node, wait for services with a timeout, and `call(client, req, timeout)`.

Scene application reuses `scene_apply.apply_scene`. No CLI here.

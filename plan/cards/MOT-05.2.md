# MOT-05.2 — Joint-trajectory file library: validation, hashing, limit checks, time scaling, densification (pure Python)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-05.1"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
      "ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/trajectory_*.json"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#11",
    "docs/adr/016-commissioning-gated-execution.md",
    "config/robot/b601_dm_limits.yaml",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_map.py",
    "ros2_ws/src/crackvision_motion/test/test_scene_core.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "pure-python",
        "cmd": "bash -c '! grep -nE \"^\\s*(import|from) +(rclpy|control_msgs|moveit_msgs|sensor_msgs|trajectory_msgs)\" ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The schema follows INTERFACES §11 exactly. Each malformed case gets its own failing test with a specific TrajectoryError message: wrong or reordered joint_names, fewer than 2 points, first t_s ≠ 0, non-increasing t_s, wrong vector length, NaN/inf, bad sha hex, unknown keys, bad purpose.",
      "The limit checks report every violation as (point index, joint, kind, value, bound) and do not stop at the first. Velocity and acceleration are checked from explicit fields when present and always from finite differences of positions/time. The margin policy matches §11, including the all-zero home on joint2/joint3 upper=0.0.",
      "Time scaling is uniform: t/s, v·s, a·s². Scale must be in (0, 1], otherwise TrajectoryError. A test shows that an overspeed trajectory passes after scaling and the original does not.",
      "Densification inserts linearly interpolated configurations so that no joint step exceeds max_step_rad, and it keeps the original points.",
      "The limits loader reuses an existing crackvision_motion helper if one exists; otherwise it lives here, and the reviewer checks that nothing is duplicated.",
      "Fixtures: trajectory_smoke.json moves home→an interior pose P within limits and margin. trajectory_estop.json is slow (≥10 s) P→Q. trajectory_overspeed.json and trajectory_out_of_limits.json are also included. Fixtures use limits_file.sha256 and the config sha fields null (unbound), so they survive future config measurements."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 2,
    "consequence": 4,
    "task_class": "python-pure-lib",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-05.2",
  "parent": "MOT-05",
  "title": "Joint-trajectory file library: validation, hashing, limit checks, time scaling, densification (pure Python)",
  "outcome": "crackvision_motion/joint_trajectory.py loads and validates crackvision.joint_trajectory/1 and checks it against config/robot/b601_dm_limits.yaml (position with margin policy, explicit and finite-difference velocity/acceleration). It also implements the §11 uniform time scaling and densification for collision checks, with no ROS imports, unit tests and fixtures."
}
```

Implement exactly the §11 joint-trajectory contract. If this body and §11 disagree, §11 wins. The module runs under system python3 (ROS Humble py3.10) without a colcon build. Tests insert the package directory on sys.path, as test_scene_core.py does.

Public API, names normative for MOT-05.3/.4:
- `class TrajectoryError(ValueError)`
- `JOINT_NAMES = ('joint1',…,'joint6')`
- `load_trajectory(path) -> dict`: validated and normalised (floats, lists), with key `_path`.
- `file_sha256(path) -> str`
- `load_limits(path) -> dict[str, dict]` with lower/upper/velocity/acceleration per arm joint. Reuse an existing loader if present.
- `check_limits(traj, limits, *, position_margin_rad, start_tolerance_rad=...) -> list[Violation]` (Violation = dataclass index, joint, kind ∈ {position, position_margin, velocity, acceleration}, value, bound)
- `scale_time(traj, speed_scale) -> dict` (new dict; never mutates)
- `densify(traj, max_step_rad) -> list[list[float]]`
- `max_abs_velocity(traj) -> list[float]` (per joint, finite-difference)
No file writes, no ROS. Tests cover every rule above, plus a test that the repo's actual config/robot/b601_dm_limits.yaml loads. Pose P in the fixtures must be well inside the limits (e.g. joint2 ≈ -0.6, joint3 ≈ -0.9; others small). Choose it so that MOT-05.5 can reuse it on the mock stack, and record the choice in a fixture comment field (`notes`).

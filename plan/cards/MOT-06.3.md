# MOT-06.3 — Cartesian planning core: config loader, paths3d segment tasks, roll/direction candidates, surface-scene check (pure Python)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-06.2"
  ],
  "requirements": [
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py",
      "ros2_ws/src/crackvision_motion/test/test_cartesian_core.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/cartesian/**",
      "config/motion/cartesian_planning.yaml"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#10",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
    "config/scene/scene.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_cartesian_core.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "config-loads",
        "cmd": "bash scripts/ros/env_ros.sh python3 -c \"import sys; sys.path.insert(0,'ros2_ws/src/crackvision_motion'); from crackvision_motion.cartesian_core import load_config; load_config('config/motion/cartesian_planning.yaml')\"",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Module imports nothing from rclpy/moveit_msgs, so it runs under system python3 without a ROS graph. Fixtures are committed small files (or created by the tests in tmp_path), never untracked directories.",
      "The config loader enforces exactly the §13 schema: an unknown or missing key, or an out-of-range value, raises a typed error. Every value in the committed config carries its §13 default and a source/nominal note.",
      "paths3d loading validates the subset the planner relies on: schema id, frame base_link, segment waypoints with phases approach/trace/retract in order, unit quaternions, finite vectors. Errors are typed, not KeyError.",
      "Segment iteration order matches §13 (order_index, main then branches by branch_index, segments in order). Tested with a multi-component fixture.",
      "Roll offsets rotate each pose about its own +X: tool_tip +X (the boresight) and position are unchanged and +Z rotates by the offset (tested numerically). Reverse direction swaps approach and retract and reverses the trace. The candidate order is deterministic and matches §13.",
      "The surface/scene check flags trace tool positions inside, or less than the clearance above, any scene box (rotated boxes handled) and passes a consistent fixture; tested both ways."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "python-core",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-06.3",
  "title": "Cartesian planning core: config loader, paths3d segment tasks, roll/direction candidates, surface-scene check (pure Python)",
  "parent": "MOT-06",
  "outcome": "crackvision_motion.cartesian_core loads the §13 planning config and a §10 paths3d file into ordered segment tasks (tool_tip approach/trace/retract poses), generates the deterministic direction×roll candidate set, applies uniform roll offsets, and flags surface/scene mismatches — ROS-free and unit-tested; config/motion/cartesian_planning.yaml ships with sourced nominal values."
}
```

Implement `cartesian_core.py` exactly per INTERFACES §13. Pattern: `reachability_core.py`/`scene_core.py` (numpy allowed, no ROS imports).

Functions, with names you may refine if §13 names them:
- `load_config(path)`: returns a dict or dataclass; raises `CartesianConfigError`.
- `load_paths3d(path)`: returns `(doc, sha256)`; raises `Paths3dError`.
- `segment_tasks(doc)`: returns an ordered list of `SegmentTask(segment_ref, approach, trace[], retract, normals, surface_points, trace_length_m)`. Each pose is a (position, quat_xyzw) in base_link.
- `candidates(cfg)`: returns an ordered list of (direction, roll_offset_rad).
- `apply_candidate(task, direction, roll)`: returns the transformed task.
- `split_remainder(task, last_reached_index)`: returns the remainder sub-task, or None when fewer than 3 waypoints remain.
- `surface_scene_mismatch(task, scene_cfg, clearance_m)`: returns a list of reasons.

Ship `config/motion/cartesian_planning.yaml` with every §13 key. Values are nominal, and each carries a source (ADR-014/017, MOT-06.1 probe facts).

Tests use small paths3d fixtures under `test/fixtures/cartesian/`, either hand-built minimal §10 docs or ones written by the test in tmp_path. Cover the ordering, the roll math, reverse, split, the mismatch check and config refusals.

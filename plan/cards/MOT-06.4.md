# MOT-06.4 — Cartesian plan document: assembly, coverage metrics, strict validator and loader (pure Python)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-06.3"
  ],
  "requirements": [
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
      "ros2_ws/src/crackvision_motion/test/test_cartesian_plan.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#11",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
    "config/robot/b601_dm_limits.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_cartesian_plan.py -q -p no:cacheprovider",
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
      "The validator rejects, with typed errors: a missing or extra field; wrong joint_names; non-finite positions; a leg whose first configuration differs from the previous leg's last (beyond a stated tolerance); positions outside the inclusive limits (loaded with joint_trajectory.load_limits); and coverage numbers inconsistent with segments[].",
      "Coverage: fraction_by_length = planned trace arc length / total trace arc length over all segments, including failed and mismatched ones; `complete` is true only if every segment is `full`; upstream counts are copied verbatim from paths3d. Each is tested on a hand-built example with known numbers.",
      "Segment status derivation follows §13: full, partial (split sub-segments), failed (with reasons) and surface_scene_mismatch. No segment of the input is ever absent from segments[] (tested).",
      "sources.paths3d has the exact §11.2 shape {path, sha256, execution_eligible}, with execution_eligible copied from the paths3d file; caveats[0] is the §13 disclaimer verbatim.",
      "Writing is atomic (temp + rename) and round-trips through the loader; the module has no ROS imports."
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
  "id": "MOT-06.4",
  "title": "Cartesian plan document: assembly, coverage metrics, strict validator and loader (pure Python)",
  "parent": "MOT-06",
  "outcome": "crackvision_motion.cartesian_plan builds, validates, writes and loads crackvision.cartesian_plan/1 and crackvision.view_plan/1 documents exactly per §13, computes per-segment status and coverage (by waypoints and by arc length, plus upstream paths3d counts), and verifies leg continuity — unit-tested without ROS."
}
```

Implement `cartesian_plan.py` per §13. It is the only code that builds and reads plan and view-plan documents. MOT-07 will consume them through `load_plan`.

API (refine to §13 names):
- `new_plan(case_id, sources, parameters, start_state)`;
- `add_leg(plan, kind, segment_ref, direction, roll, positions, waypoint_index_map, recheck)`;
- `add_segment_report(...)`;
- `finalize(plan, paths3d_doc, segment_tasks)`: computes coverage and statuses, and validates;
- `write_plan(plan, path)`, `load_plan(path, limits_path)`;
- the same set for the view plan.

All checks are pure. Joint limits come from `joint_trajectory.load_limits`; do not duplicate the parsing.

Tests: build plans with synthetic legs (no MoveIt). Cover every refusal, coverage arithmetic for full, partial, failed and mismatch, the continuity check and the round-trip.

# MOT-06.8 — View-phase core: camera_link view-pose candidates and D405 FOV coverage check (pure Python)

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
      "ros2_ws/src/crackvision_motion/crackvision_motion/view_core.py",
      "ros2_ws/src/crackvision_motion/test/test_view_core.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#8",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
    "config/motion/cartesian_planning.yaml",
    "config/robot/end_effector.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_view_core.py -q -p no:cacheprovider",
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
      "View poses use the realsense-ros camera_link convention (+X optical axis): at tilt 0 the +X axis equals −n of the specimen top and the camera sits view_distance_m along +n from the centre; at 15° the angle between +X and −n is 15° within 1e-9 while the axis still passes through the centre. Tested numerically.",
      "The FOV check projects the footprint corners (from the scene specimen box, rotation included) into the camera with a pinhole using the configured HFOV/VFOV and margin, fails when any corner falls outside, and reports the per-corner angles. A 0.20 m footprint at 0.25 m passes at tilt 0, and a footprint too large fails (tested).",
      "The specimen centre and normal come from the scene `specimen` object (top-face centre, +Z of its pose) or an explicit centre; a missing specimen without --center is a typed error.",
      "Candidate order is deterministic and documented; the module has no ROS imports."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "python-core",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-06.8",
  "title": "View-phase core: camera_link view-pose candidates and D405 FOV coverage check (pure Python)",
  "parent": "MOT-06",
  "outcome": "crackvision_motion.view_core generates the §13 camera_link view-pose candidates (0.25 m standoff, tilt 0/15°, yaw set) over the specimen centre from the scene or an explicit centre, and checks that every specimen footprint corner lies inside the nominal D405 FOV with margin and within the view band — ROS-free and unit-tested."
}
```

Implement per §13's view-plan subsection:
- `specimen_frame(scene_cfg, center=None)`: returns (centre_top, n_out, footprint corners).
- `view_candidates(cfg, centre, n)`: returns an ordered list of (tilt_deg, yaw_rad, camera_link pose).
- `fov_check(pose, corners, cfg)`: returns (ok, details).

For the 15° tilt, rotate the camera position about the specimen centre so the optical axis still hits the centre (the same geometry as `recommend_placement --emit-view-config`, MOT-04.6). Reuse reachability_core rotation helpers if they exist. Distances and FOV come from config/motion/cartesian_planning.yaml (owned by MOT-06.3; read only here).

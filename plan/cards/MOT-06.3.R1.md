# MOT-06.3.R1 — Repair MOT-06.3: surface_scene_mismatch omits §13.9 surface_object_id relaxation and pe

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "config/motion/cartesian_planning.yaml",
      "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/cartesian/**",
      "ros2_ws/src/crackvision_motion/test/test_cartesian_core.py"
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
      "surface_scene_mismatch takes surface_object_id and surface_object_tolerance_m (or the loaded cfg) and uses each trace waypoint's clearance_m from paths3d. A test with the production-shaped scene (table + specimen) and a trace exactly clearance_m above the specimen top returns []. A trace more than the tolerance closer than that is flagged, and so is a non-surface object within the full clearance.",
      "load_paths3d accepts a polyline with `segments: []`, and segment_tasks yields no tasks for it. A test covers this, and a non-list `segments` is still rejected with Paths3dError.",
      "All acceptance criteria of MOT-06.3 still hold at the new revision"
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
  "priority": 95,
  "repairs": "MOT-06.3",
  "findings": [
    "4b71255349a28ca2",
    "c5959720255e017d"
  ],
  "id": "MOT-06.3.R1",
  "title": "Repair MOT-06.3: surface_scene_mismatch omits §13.9 surface_object_id relaxation and pe",
  "parent": "MOT-06",
  "outcome": "Resolve the review findings on MOT-06.3 while every acceptance criterion of MOT-06.3 still holds."
}
```

Repair work generated deterministically from review attempt `00508-MOT-06.3-review` of `MOT-06.3`.
Original card: `plan/cards/MOT-06.3.md` — its acceptance criteria must still hold.

## Finding 1 [major] surface_scene_mismatch omits §13.9 surface_object_id relaxation and per-waypoint clearance, so it flags every correct trace in the production scene
- Evidence: cartesian_core.py surface_scene_mismatch applies one scalar `clearance_m` to every object (`if distance_m < clearance_m:`), with no surface_object_id/surface_object_tolerance_m handling. SegmentTask also drops each waypoint's clearance_m. §13.9 requires `clearance_m − surface_object_tolerance_m` for the surface_object_id object and each waypoint's own clearance_m for the others. Probe with config/scene/scene.yaml and a trace 0.01 m above the specimen top flags all 4 waypoints against 'specimen'. The implementer's notes admit the relaxation was left out.
- Consequence: Called as specified, the check reports surface_scene_mismatch for every geometrically correct segment in the real scene, so no segment would be planned. A caller cannot work around this, because a single clearance cannot apply different bounds to different objects. Downstream MOT-06 planner cards would inherit a broken check or have to rewrite it outside this card's scope.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py, ros2_ws/src/crackvision_motion/test/test_cartesian_core.py
- Acceptance condition: surface_scene_mismatch takes surface_object_id and surface_object_tolerance_m (or the loaded cfg) and uses each trace waypoint's clearance_m from paths3d. A test with the production-shaped scene (table + specimen) and a trace exactly clearance_m above the specimen top returns []. A trace more than the tolerance closer than that is flagged, and so is a non-surface object within the full clearance.

## Finding 2 [major] load_paths3d rejects valid §10 files with a polyline whose segments[] is empty
- Evidence: _validate_paths3d raises `'{pctx}.segments' must be a non-empty list`. §10.3 drops segments with fewer than 3 valid points entirely, and the producer src/crackvision/path3d.py:160-178 builds segments_out only from lifted.segments, so it emits `segments: []` for such polylines. Probe: setting the fixture branch's segments to [] gives `Paths3dError 'components[1].polylines[0].segments' must be a non-empty list`.
- Consequence: Any real capture in which one branch or main polyline lost all its depth makes the whole paths3d file unloadable. One bad polyline blocks Cartesian planning for every other crack in the file.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py, ros2_ws/src/crackvision_motion/test/test_cartesian_core.py
- Acceptance condition: load_paths3d accepts a polyline with `segments: []`, and segment_tasks yields no tasks for it. A test covers this, and a non-list `segments` is still rejected with Paths3dError.

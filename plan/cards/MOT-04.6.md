# MOT-04.6 — Reachability on task frames (tool_tip / camera_link) with a workcell-faithful specimen proxy + camera view check

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-04.3",
    "MOT-04.4",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/placement.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py",
      "ros2_ws/src/crackvision_motion/test/test_reachability_core.py",
      "ros2_ws/src/crackvision_motion/test/test_placement.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/scene_table.yaml",
      "ros2_ws/src/crackvision_motion/test/fixtures/reachability_task_frames_smoke.yaml",
      "ros2_ws/src/crackvision_motion/test/fixtures/reachability_view_smoke.yaml",
      "scripts/ros/test_reachability_task_frames.sh",
      "config/motion/reachability.yaml",
      "docs/INTERFACES.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#7",
    "docs/INTERFACES.md#8",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "config/robot/end_effector.yaml"
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
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "task-frames-smoke",
        "cmd": "bash scripts/ros/test_reachability_task_frames.sh",
        "timeout_s": 1200,
        "expect_exit": 0
      },
      {
        "id": "legacy-sweep-smoke",
        "cmd": "bash scripts/ros/test_reachability.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "cli-smoke",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion recommend_placement --map ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json --reachability-config config/motion/reachability.yaml --out logs/mot046_check_placement.yaml --emit-verify-config logs/mot046_check_verify.yaml && ros2 run crackvision_motion recommend_placement --validate-placement logs/mot046_check_placement.yaml'",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "All additions are backward compatible: surface_collision.model defaults to `slab` with the MOT-04.4 geometry unchanged (tested), `environment` is optional, the map/placement schemas are unchanged (new data only under grid.environment and caveats).",
      "specimen_block: a block from floor_z_m up to 1 mm under each surface_z over grid + margin, minus base_keepout_m (base_link.STL footprint + 2 cm); ConfigError if any grid node lies in the keep-out or any surface_z is below the floor; geometry unit-tested (covers the grid, avoids the base, empty at the floor).",
      "Environment objects come from a crackvision.scene_config/1 file by id (with their ACM entries), are applied for the whole sweep and removed in a finally path, and are recorded in the map (scene path + sha256 + ids); an unknown id is a config error (exit 2).",
      "No collision check is weakened: allowed_links for the proxy is empty in the committed config; FK re-check tolerances and joint limits unchanged; the task frame is FK-verified exactly as before.",
      "Committed config/motion/reachability.yaml: ik_link tool_tip, standoffs 0.01/0.04 m documented as TOOL-TIP clearances (ADR-014 §2), every value nominal with a source; grid starts outside the keep-out.",
      "recommend_placement --emit-view-config writes a valid reachability_config/1 for camera_link at 0.25 m above the placement centre (tilt 0/15 deg), proxy margin covering the dilated footprint; caveats start with the INTERFACES §7.3 disclaimer verbatim and name the task frame and collision model.",
      "test_reachability_task_frames.sh proves, live: tool_tip reachable at BOTH 0.01 and 0.04 m clearance at x=0.23 over a raised surface with the table and specimen block present (the gripper_tcp-referenced 0.01 m was infeasible everywhere), far targets not reachable, and a camera view pose 0.25 m above a surface point reachable."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 4
  },
  "track": "simulation",
  "priority": 70,
  "id": "MOT-04.6",
  "title": "Reachability on task frames (tool_tip / camera_link) with a workcell-faithful specimen proxy + camera view check",
  "parent": "MOT-04",
  "outcome": "The sweep targets the physical task frame (tool_tip for tool clearances, camera_link for viewing distance) against self-collision incl. the camera, the workcell table and a specimen block that never overlaps the robot base; recommend_placement also emits a wrist-camera view check; the committed config uses this model."
}
```

Implemented interactively by the technical-lead recovery session (2026-09-30) and submitted for
independent review with `claude-auto hw-done`. Root cause it fixes (MOT-04.5 attempt 00094): standoffs meant as
tool-tip clearances were applied to gripper_tcp (grasp centre, 44.3 mm proximal to the tip), and the grid-wide slab
ran under the base (link2 collision at surface_z >= ~0.10) with no table in the scene.

Feasibility probe run by the session on the mock stack (coarse 0.06 m grid; scratch, not committed): 599/910
tool targets reachable (0/4320 before); feasible 0.20 m placement at (0.29, 0) on the table with >= 0.40 rad
joint margin; half-step verification 200/200; perpendicular camera views from 0.20-0.30 m reachable over it;
margin collapses toward x ~ 0.41 (consistent with TECHNICAL_APPROACH §2.5). MOT-04.5 produces the real result.

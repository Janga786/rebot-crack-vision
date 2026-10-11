# MOT-06.1 — Probe MoveIt Cartesian/transit services for task frames on the mock stack

```json card
{
  "kind": "research",
  "depends_on": [
    "MOT-02",
    "MOT-03",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "scripts/ros/probe_cartesian_path.py",
      "scripts/ros/probe_cartesian_path.sh",
      "docs/motion/CARTESIAN_PROBE.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#8",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "ros2_ws/src/crackvision_motion/launch/mock_planning.launch.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_apply.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py",
    "scripts/ros/_mock_stack.sh",
    "scripts/ros/env_ros.sh",
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
        "id": "probe",
        "cmd": "bash scripts/ros/probe_cartesian_path.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "note",
        "cmd": "test -s docs/motion/CARTESIAN_PROBE.md && grep -q 'tool_tip' docs/motion/CARTESIAN_PROBE.md && grep -qi 'fraction' docs/motion/CARTESIAN_PROBE.md",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The probe runs against the headless mock stack only (no driver, no motion path), tears the stack down in every exit path, and writes logs/cartesian_probe.json with raw observations; exit 0 iff every probe query completed and the JSON was written, whatever the facts turned out to be.",
      "CARTESIAN_PROBE.md states, each backed by a value in the JSON: the exact GetCartesianPath request fields available on this Humble install (jump_threshold and any prismatic/revolute absolute thresholds); whether link_name tool_tip and camera_link are accepted and the returned configurations FK (via /compute_fk) to the requested poses (position/angle error); the meaning of `fraction` (path length vs waypoints) and how to identify the last reached waypoint; whether the returned trajectory carries non-zero time_from_start; what happens when a waypoint lies inside the scene specimen with avoid_collisions true (stops before, fraction < 1); the effect of a roll change about tool_tip +X between waypoints; and that /plan_kinematic_path accepts a joint-space goal with an explicit start_state.",
      "No collision checking is disabled anywhere in the probe; the scene used is a probe-local table + specimen whose top face matches the probed surface height (not config/scene/scene.yaml edits).",
      "Facts that contradict an assumption in the MOT-06 card body (e.g. tool_tip not accepted as link_name) are stated plainly as blockers with the evidence, not worked around."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 3,
    "task_class": "ros-live-probe",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-06.1",
  "title": "Probe MoveIt Cartesian/transit services for task frames on the mock stack",
  "parent": "MOT-06",
  "outcome": "A committed, re-runnable live probe plus a note establishing how Humble's /compute_cartesian_path and /plan_kinematic_path behave for link_name tool_tip/camera_link on the MOT-02 mock stack (fields, fraction semantics, timing, collision stop, jump thresholds)."
}
```

Purpose: fix the facts the MOT-06 contract (MOT-06.2) needs before anyone writes the planner. No product code.

Write `scripts/ros/probe_cartesian_path.sh` (pattern: `scripts/ros/test_reachability_task_frames.sh` + `_mock_stack.sh`: `set -m`, source `env_ros.sh` and `ros2_ws/install/setup.bash`, `start_mock_stack`, run the probe, `stop_mock_stack`) and `scripts/ros/probe_cartesian_path.py` (rclpy, `/usr/bin/python3`).

The probe should:
1. Apply a probe-local collision scene through `/apply_planning_scene`, built in the script and not read from config/scene: the table top at z=0, plus a 0.20x0.20 m specimen box whose top face is at z=0.010, centred at the recommended placement (0.26, 0.0) from config/motion/specimen_placement.yaml. Keep the scene's ACM entries.
2. Call `/compute_ik` with `ik_link_name: tool_tip` for a pose 0.04 m above (0.26, 0, 0.010), with +X pointing straight down (−Z base). Seed from home.
3. Call `/compute_cartesian_path`. Use the IK result as start_state, group `arm`, link_name `tool_tip`, and two waypoint sets: (a) descend to 0.01 m clearance, then a straight 0.06 m line along +Y at 0.01 m clearance with 0.002 m spacing; (b) the same line with roll about tool_tip +X ramping 0 → 90°. Record fraction, the number of points, time_from_start of the last point, and every request field name in the message definition (introspect `moveit_msgs.srv.GetCartesianPath.Request.get_fields_and_field_types()`).
4. FK-check each returned configuration with `/compute_fk` for `tool_tip`. Report the max position/angle error against the nearest requested waypoint, and how to map the result to the last reached waypoint index.
5. Run waypoint set (a) again with one waypoint 0.005 m inside the specimen, `avoid_collisions: true`. Record the fraction and the last reached point.
6. Repeat step 3a with link_name `camera_link`, using a view pose 0.25 m above the specimen centre.
7. Call `/plan_kinematic_path` (GetMotionPlan) with an explicit start_state (home) and a joint-constraint goal equal to the step-2 IK solution. Record success, the number of points, and the planner id used.
Write all raw observations to `logs/cartesian_probe.json`. Never set `avoid_collisions: false`.

Then write `docs/motion/CARTESIAN_PROBE.md`: the date, the command, a facts table (fact | observed value | JSON key), and a 'consequences for MOT-06.2' list. Exit codes: §0 conventions (0 = probe completed, 1 = runtime failure, 3 = a required service missing).

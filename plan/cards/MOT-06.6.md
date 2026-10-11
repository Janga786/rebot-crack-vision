# MOT-06.6 — plan_crack_path node/CLI: per-segment candidates, transit + approach/trace/retract, split fallback, independent re-check, plan file

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-06.3",
    "MOT-06.4",
    "MOT-06.5"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#0",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py"
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
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py -q -p no:cacheprovider",
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
        "id": "entry-point",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion plan_crack_path --help'",
        "timeout_s": 120,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The orchestration follows §13 exactly: segment order; candidate order and selection rule; transit (IK seeded from the current config, then a joint-goal plan); Cartesian approach/trace/retract with link_name tool_tip; split re-entry bounded by max_splits_per_segment; and the end_state leg. The selection logic is a pure function unit-tested with fake planner results, covering the first-full-wins rule, the margin tie-break, the partial → split path and the all-fail path.",
      "Every output configuration passes the independent re-check (FK vs target for waypoint configs, /check_state_validity with the scene applied, inclusive limits, max joint step). A failed re-check fails the leg with a reason and the planner moves to the next candidate. Nothing is accepted on MoveIt's word alone.",
      "No code path relaxes collision checking, reduces a clearance, or omits a segment from the report. Segments flagged surface_scene_mismatch are reported and not planned.",
      "Exit codes and --dry-run per §13/§0: --dry-run validates the inputs and prints segment/candidate counts with no rclpy.init and no file writes (tested). Usage and config errors return 2 (tested). The plan's sha bindings and execution_eligible copy come from the actual files.",
      "The CLI never sends a trajectory to any controller: no FollowJointTrajectory/ExecuteTrajectory client exists in this module (reviewer greps)."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 4,
    "consequence": 4,
    "task_class": "ros-node",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-06.6",
  "title": "plan_crack_path node/CLI: per-segment candidates, transit + approach/trace/retract, split fallback, independent re-check, plan file",
  "parent": "MOT-06",
  "outcome": "`ros2 run crackvision_motion plan_crack_path --paths3d PATH` applies the scene, plans every paths3d segment per §13 (direction×roll candidates, transit via OMPL, Cartesian approach/trace/retract for tool_tip, bounded split re-entry), re-checks every configuration independently, and writes a validated crackvision.cartesian_plan/1 with coverage, following §0/§13 exit codes and a ROS-free --dry-run."
}
```

Write the node in the style of `reachability_sweep.py` (cli_common RunSummary/logging, §0 flags, offline validation before `rclpy.init()`). Register `plan_crack_path = crackvision_motion.plan_crack_path:main` in setup.py.

Sequence:
1. Load the config, paths3d, scene, end_effector sha and limits. Resolve the start state.
2. Run the pure checks (cartesian_core), then exit for --dry-run.
3. Init the node, wait for services, apply the scene (scene_apply).
4. For each SegmentTask, loop over candidates: IK(approach) → transit → Cartesian approach → Cartesian trace → re-check → Cartesian retract → re-check.
5. Select per §13; on a partial trace, keep the best prefix, add a Cartesian retract from the last reached waypoint and re-enter with the remainder.
6. Plan the end_state transit.
7. Assemble with cartesian_plan, write to `data/plans/{case}_cartesian_plan.json` or `--out`, and print a coverage summary.

Keep the candidate selection and split bookkeeping in pure, testable functions (in this module or in cartesian_core if §13 places them there; this card may only write plan_crack_path.py). Default output stays under data/ (git-ignored); never commit generated plans.

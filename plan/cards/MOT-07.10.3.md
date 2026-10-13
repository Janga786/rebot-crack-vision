# MOT-07.10.3 — retime_plan --view-plan: retime the view_plan transit leg into a view_move trajectory

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-07.10.1",
    "MOT-07.10.2"
  ],
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/retime.py",
      "ros2_ws/src/crackvision_motion/test/test_retime_view.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#17",
    "docs/INTERFACES.md#16",
    "docs/INTERFACES.md#13",
    "docs/adr/019-time-parameterization.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/retime_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py"
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
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_retime_view.py ros2_ws/src/crackvision_motion/test/test_retime_plan_cli.py ros2_ws/src/crackvision_motion/test/test_retime.py ros2_ws/src/crackvision_motion/test/test_retime_core.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "help",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; ros2 run crackvision_motion retime_plan --help | grep -q -- --view-plan'",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "ros-free",
        "cmd": "bash -c '! grep -nE \"^\\s*(from|import)\\s+(rclpy|[a-z_]+_msgs)\" ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py ros2_ws/src/crackvision_motion/crackvision_motion/retime.py'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "View retiming reuses retime_core (plan_polyline/refine/time_profile) and the existing self-check path. It treats transit_leg as a one-leg plan and does not duplicate the algorithm.",
      "The CLI rules match §17.3: --plan and --view-plan are mutually exclusive and one is required (exit 2); --purpose with --view-plan is exit 2; an invalid view plan or a null transit_leg is exit 3; limits/end_effector sha mismatches against view_plan.sources are exit 3; the default out is data/trajectories/view_trajectory.json; --dry-run writes nothing and exits 0.",
      "The written trajectory has purpose view_move, source {paths3d: null, view_plan: {path, sha256}} with a path relative to --root when inside it, and first/last points bit-exact to transit_leg's endpoints. The report has one transit leg and validates with validate_retime_report.",
      "test_retime_view.py proves the trajectory is accepted by joint_trajectory.load_trajectory and by execution_gate's offline G-TRAJ/G-LIMITS/G-ELIGIBLE in mock mode. It also covers every refusal above. The existing --plan behaviour and its tests are unchanged."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "motion-retime",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.10.3",
  "title": "retime_plan --view-plan: retime the view_plan transit leg into a view_move trajectory",
  "parent": "MOT-07.10",
  "outcome": "`retime_plan --view-plan data/plans/view_plan.json` writes a §17.3 view_move joint_trajectory plus a .retime.json sidecar from the view plan's transit_leg, through the existing §16.2 algorithm, and the result loads with joint_trajectory and passes G-LIMITS offline."
}
```

- retime.py:
  - Add `retime_view_plan_doc(view_plan, view_plan_path, root, ...)`, with the same config arguments as `retime_plan_doc`.
  - It builds a cartesian-plan-shaped shim, `{legs: [transit_leg], sources: view_plan.sources (with paths3d=None)}`, and runs the shared internals. Factor out the common body of `retime_plan_doc` into a private helper if needed; do not copy it.
  - Override `purpose`/`source` per §17.3 and set `report.input_shas.plan` to the view-plan file's sha.
  - Keep the §16.3 sha refusals, read from the view plan's sources.
- retime_plan.py:
  - Use an argparse mutually exclusive group for --plan/--view-plan (required).
  - Make --purpose default None. Resolve it to crack_task for --plan; with --view-plan it is a usage error (exit 2 via cli_common's usage path).
  - Load with cartesian_plan.load_view_plan, mapping CartesianPlanError to exit 3.
  - Default out is `data/trajectories/view_trajectory.json`.
  - Extend the dry-run leg summary to cover the single leg.
- test_retime_view.py:
  - Build a view plan in tmp_path whose transit_leg runs from home (all zeros) to a reachable configuration, with sources hashed from the real config files (or the MOT-07 retime fixtures).
  - Cover the happy path, the gate acceptance (call execution_gate's offline evaluation in mock mode with the real limits/config files), and every exit-2/exit-3 case listed in the criteria.

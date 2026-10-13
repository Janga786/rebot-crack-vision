# MOT-07.10.4 — trajectory_check: accept a view_plan and check view_move trajectories independently

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-07.10.1"
  ],
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/trajectory_check.py",
      "tests/test_trajectory_check_view.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#17",
    "docs/INTERFACES.md#16",
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#11",
    "docs/adr/019-time-parameterization.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_trajectory_check_view.py tests/test_trajectory_check.py tests/test_trajectory_check_path.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "independent",
        "cmd": "bash -c '! grep -nE \"^\\s*(from|import)\\s+(crackvision_motion|rclpy)\" src/crackvision/trajectory_check.py tests/test_trajectory_check_view.py'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Schema dispatch, purpose/plan pairing, the view_plan provenance rules, the single-leg polyline, vacuous C-TOOLSPEED and single-transit-leg C-REPORT all match §17.4.",
      "The trajectory schema validation accepts view_move only with source {paths3d: null, view_plan: {path, sha256}}, and rejects a view_plan key on crack_task/test (§17.1). This is implemented from the contract text, not by importing joint_trajectory.",
      "Tests use hand-built view plans and trajectories (crackvision.kinematics FK only). They show a good view_move trajectory exits 0, and that each of these exits 3 with the right check failing: wrong view_plan sha → C-PROVENANCE; view_move against a cartesian_plan, or crack_task against a view_plan → C-PROVENANCE; an off-path point → C-PATH; an overspeed sample → C-LIMITS; a report with a wrong point_range → C-REPORT.",
      "The existing cartesian_plan behaviour and every existing checker test are unchanged."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "trajectory-checker",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.10.4",
  "title": "trajectory_check: accept a view_plan and check view_move trajectories independently",
  "parent": "MOT-07.10",
  "outcome": "`./env.sh python -m crackvision.trajectory_check --trajectory T --plan view_plan.json` runs every §16.5 check on a view_move trajectory as amended by §17.4. It stays independent of crackvision_motion."
}
```

Extend src/crackvision/trajectory_check.py per §17.4:
- `validate_plan_schema` dispatches on `schema`. Add `validate_view_plan_schema` with the minimal §17.4 structure.
- `validate_trajectory_schema` accepts view_move with the §17.1 source shape.
- `build_plan_polyline` takes a view plan as `legs=[transit_leg]`; normalise once, so check_path/toolspeed/report work unchanged.
- `check_provenance`:
  - Enforce purpose↔plan-schema pairing.
  - For view_move, compare `source.view_plan.sha256` with `file_sha256(--plan)` (pass the plan file's sha in through `files`) and require source.paths3d to be null.
  - Read the limits/end_effector/scene sha rules from view_plan.sources.
- `check_toolspeed` with no trace legs: pass, numbers `{trace_legs: []}`.
- C-SCHEMA's detail text names whichever plan schema was parsed.

Update the `--plan` help text. Do not import crackvision_motion. Put the new tests in tests/test_trajectory_check_view.py, reusing the helpers and fixtures in the existing checker tests where possible.

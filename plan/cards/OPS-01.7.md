# OPS-01.7 — plan stage: path3d -> plan_crack_path -> retime_plan -> trajectory_check for one case

```json card
{
  "kind": "impl",
  "depends_on": [
    "OPS-01.2"
  ],
  "requirements": [
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "scripts/operator/plan.sh",
      "scripts/ros/test_operator_plan.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#19. Operator CLI workflow",
    "docs/INTERFACES.md#10. Robot-frame 3D crack paths and tool waypoints",
    "docs/INTERFACES.md#13. Cartesian crack-path plans",
    "docs/INTERFACES.md#16. Time parameterization and independent trajectory check",
    "src/crackvision/path3d.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py",
    "src/crackvision/trajectory_check.py",
    "docs/motion/CARTESIAN_PLANNING.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "e2e",
        "cmd": "bash scripts/ros/test_operator_plan.sh",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "`scripts/operator/plan.sh --case-id CASE [--root PATH] [--force]` runs, in order: `./env.sh python -m crackvision.path3d --cases CASE`, `scripts/ros/env_ros.sh ros2 run crackvision_motion plan_crack_path --paths3d data/paths3d/{case}_paths3d.json ...`, `./env.sh python -m crackvision.retime_plan --plan data/plans/{case}_cartesian_plan.json ...` (via the ROS-free console script), then `./env.sh python -m crackvision.trajectory_check ...` -- no execution, no preview, no approval step here.",
      "Each sub-step is skipped (with a log line) when `crackvision.operator_state.stage_is_fresh` says the `plan` stage is already fresh for this case, unless `--force`; `plan` depends on `capture` and `paths` being done for this case_id (per §19's dependency graph) and exits 3 if either is missing, rather than attempting a doomed run.",
      "On success, records the `plan` stage done in `crackvision.operator_state` naming the cartesian_plan, trajectory, retime-report and check-report files and their sha256s.",
      "`scripts/ros/test_operator_plan.sh` runs this headlessly against the mock stack reusing MOT-06.10's synthetic scratch fixtures (`tools/synth_scene3d.py`) for a case's capture record + paths, and asserts the final trajectory_check report has every check pass or skip."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "cli-orchestration",
    "local_ok": false
  },
  "id": "OPS-01.7",
  "title": "plan stage: path3d -> plan_crack_path -> retime_plan -> trajectory_check for one case",
  "parent": "OPS-01",
  "outcome": "A case-scoped, resumable `plan` command that lifts a case's ordered pixel paths + capture record to a checked, retimed crack-trace trajectory, ready for preview -- with no execution step."
}
```

Write `scripts/operator/plan.sh` as a sequencing wrapper only, reusing the exact invocation pattern already proven in `scripts/ros/run_cartesian_synthetic.sh` (CARTESIAN_PLANNING.md), but for one specific `--case-id` whose `data/paths3d/{case}_paths3d.json` and `data/captures/{case}_capture.json` already exist (produced by OPS-01.4/OPS-01.6), and wired into `crackvision.operator_state` for resumability instead of a fresh scratch root per run.

Do not change `path3d`/`plan_crack_path`/`retime_plan`/`trajectory_check` behaviour or flags. Resolve each stage's default output path exactly as those tools already default it (`data/paths3d/`, `data/plans/`, `data/trajectories/`, and `trajectory_check`'s default `<trajectory>.check.json` sidecar) -- do not invent new paths.

For the headless test, adapt `scripts/ros/run_cartesian_synthetic.sh`'s synthetic-fixture setup (same generator, same scratch-root config copying for `b601_dm_limits.yaml`/`end_effector.yaml`) but drive it through your new `plan.sh` instead of the four tools called by hand, and assert `crackvision.operator_state` reports `plan` as done with all four output files sha-bound afterward.

# MOT-06.10 — Synthetic end-to-end planning run (GEOM-08 synthetic case → paths3d → view + crack plan) and CARTESIAN_PLANNING.md

```json card
{
  "kind": "integration",
  "depends_on": [
    "MOT-06.7",
    "MOT-06.9",
    "GEOM-08.8"
  ],
  "requirements": [
    "REQ-MOT-2",
    "REQ-INT-1",
    "REQ-OPS-2"
  ],
  "scope": {
    "write": [
      "scripts/ros/run_cartesian_synthetic.sh",
      "docs/motion/CARTESIAN_PLANNING.md",
      "docs/motion/ROS_WORKSPACE.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#10",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "docs/geometry/PATH3D_VERIFICATION.md",
    "tools/synth_scene3d.py",
    "src/crackvision/path3d.py",
    "scripts/ros/test_cartesian_plan.sh",
    "scripts/ros/test_view_plan.sh",
    "config/motion/specimen_placement.yaml",
    "config/scene/scene.yaml"
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
        "id": "e2e",
        "cmd": "bash scripts/ros/run_cartesian_synthetic.sh",
        "timeout_s": 1800,
        "expect_exit": 0
      },
      {
        "id": "doc",
        "cmd": "test -s docs/motion/CARTESIAN_PLANNING.md && grep -q 'plan_crack_path' docs/motion/ROS_WORKSPACE.md && grep -q 'plan_view' docs/motion/ROS_WORKSPACE.md",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The script works in a scratch root under logs/ (never data/ in the repo tree, never committed), runs perception/geometry steps through ./env.sh python and ROS steps under env_ros.sh, and uses a scratch scene whose specimen top matches the synthetic surface. It asserts plan validity (load_plan), coverage.complete for the synthetic crack, and execution_eligible false carried through with the nominal reasons; exit 0 only if all hold.",
      "The doc reports the observed numbers from an actual run (candidates tried, chosen roll/direction, coverage, planning time) with the command used, and states the caveats: nominal camera/tool/scene geometry; ineligible for real execution; the production scene.yaml specimen (top z=0.01, centre 0.30) disagrees with the recommended placement (0.26, surface 0) until MOT-10 measures the cell, so production-scene planning of synthetic data reports surface_scene_mismatch by design; ordering not optimised; untimed output for MOT-07.",
      "ROS_WORKSPACE.md gains entries for plan_crack_path and plan_view (commands, inputs, outputs, exit codes) without altering existing entries.",
      "No config/ file and no earlier contract is modified."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 4,
    "consequence": 3,
    "task_class": "integration-doc",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-06.10",
  "title": "Synthetic end-to-end planning run (GEOM-08 synthetic case → paths3d → view + crack plan) and CARTESIAN_PLANNING.md",
  "parent": "MOT-06",
  "outcome": "One script runs tools/synth_scene3d.py → crackvision.paths → crackvision.path3d → plan_view/plan_crack_path on the mock stack with a scene consistent with the synthetic surface, and docs/motion/CARTESIAN_PLANNING.md documents usage, the measured coverage/fallback behaviour, caveats and the MOT-07 handoff; ROS_WORKSPACE.md lists the new CLIs."
}
```

`run_cartesian_synthetic.sh`:
1. Create a scratch root `logs/cartesian_synthetic/<stamp>/`.
2. `./env.sh python tools/synth_scene3d.py --out <root> --seed 0 --case-id synth_mot06`, using the default flat scene.
3. Run the crackvision.paths CLI and the crackvision.path3d CLI for that case against that root, the same way tests/test_path3d_synthetic.py does.
4. Write a scratch scene_config/1: table plus a specimen box whose top face is at the generator's surface height and centre, read from the generated `{case}_gt.json`.
5. Start the mock stack and run `plan_view --scene-config <scratch>`, then `plan_crack_path --paths3d <root>/data/paths3d/... --scene-config <scratch> --require-full-coverage`.
6. Validate both outputs with the loaders, assert the criteria, and stop the stack.

Then write CARTESIAN_PLANNING.md from the actual run: what it does, how to run it, results, caveats, and the MOT-07 handoff (legs → one continuous joint path → TOTG retime into §11.2). Add the ROS_WORKSPACE.md entries.

If the synthetic crack cannot be fully covered, report it honestly with the plan's reasons and stop. Do not tune the assertions to pass.

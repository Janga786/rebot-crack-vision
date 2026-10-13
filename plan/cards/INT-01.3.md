# INT-01.3 — Document the one-command synthetic-to-limits-checked-mock-execution integration test

```json card
{
  "kind": "impl",
  "depends_on": [
    "INT-01.2"
  ],
  "requirements": [
    "REQ-INT-1"
  ],
  "scope": {
    "write": [
      "docs/motion/INTEGRATION_E2E.md",
      "docs/motion/ROS_WORKSPACE.md"
    ]
  },
  "inputs": [
    "scripts/ros/run_integration_e2e.sh",
    "docs/motion/CARTESIAN_PLANNING.md",
    "docs/motion/TIME_PARAMETERIZATION.md",
    "docs/motion/ROS_WORKSPACE.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "docs",
        "cmd": "test -s docs/motion/INTEGRATION_E2E.md && grep -q 'run_integration_e2e.sh' docs/motion/ROS_WORKSPACE.md",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "INTEGRATION_E2E.md states the exact one-line command to run the whole chain (bash scripts/ros/run_integration_e2e.sh, after scripts/ros/build_ws.sh) and the scratch roots it produces.",
      "It records one real run's headline numbers: real skeleton pixel count (from INT-01.1's step), paths/path3d component and eligibility summary, plan_view/plan_crack_path candidate counts and coverage, retime duration/peak utilisation, trajectory_check outcomes, and the execute_trajectory dry/mock gate outcomes -- pulled from an actual run's output, not invented.",
      "It states the caveats honestly: nominal (not measured) camera/tool/scene geometry throughout (GEOM-05/07/MOT-10 not yet applied here), execution_eligible is false by design, mock/dry only, and this says nothing about the view-plan leg (covered separately by MOT-07.10.5) or about a tilted/partial-coverage scenario.",
      "ROS_WORKSPACE.md gains a one- or two-line pointer to run_integration_e2e.sh consistent with its existing script entries; no other content in that file is changed.",
      "No other file is modified."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 3,
    "consequence": 2,
    "task_class": "documentation",
    "local_ok": false
  },
  "track": "simulation",
  "id": "INT-01.3",
  "title": "Document the one-command synthetic-to-limits-checked-mock-execution integration test",
  "parent": "INT-01",
  "outcome": "docs/motion/INTEGRATION_E2E.md documents scripts/ros/run_integration_e2e.sh's method, the measured results of one real run, and its honest caveats, in the style of CARTESIAN_PLANNING.md/TIME_PARAMETERIZATION.md."
}
```

## Why

Mirrors the documentation pattern every sibling integration card in this project already follows (MOT-06.10 -> CARTESIAN_PLANNING.md, MOT-07.9 -> TIME_PARAMETERIZATION.md, MOT-07.10.5 -> an addendum): the script is only half the deliverable; a reviewer (and REQ-INT-1's eventual evidence map, OPS-03) needs a readable record of what it proved and, just as importantly, what it did not.

## What to build

`docs/motion/INTEGRATION_E2E.md`, structured like `docs/motion/CARTESIAN_PLANNING.md`:

1. **Header line:** date, exact command (`bash scripts/ros/run_integration_e2e.sh`, after `bash scripts/ros/build_ws.sh`), example scratch roots.
2. **"What this runs"**: the full chain diagram, `tools/synth_scene3d.py -> crackvision.visualize -> crackvision.skeleton -> crackvision.paths -> crackvision.visualize_paths -> crackvision.path3d -> plan_view -> plan_crack_path -> retime_plan -> crackvision.trajectory_check -> execute_trajectory (dry, mock)`, each arrow labelled with the card that owns it (TC-014/GEOM-08.8 generator, TC-010/PERC-02/PERC-03/PERC-04, GEOM-08, MOT-06, MOT-07).
3. **"Observed results"**: run `bash scripts/ros/build_ws.sh && bash scripts/ros/run_integration_e2e.sh` once yourself, capture its stdout, and transcribe the real headline numbers into a table (mirror `CARTESIAN_PLANNING.md` §2's table format) -- skeleton pixel count, path/branch counts, paths3d eligibility + reasons, view/crack candidate counts and coverage fraction, retime duration and peak utilisation fractions, trajectory_check per-check outcomes, dry-mode `would_refuse_in_real`, mock-mode gate outcomes and arrival delta.
4. **"Caveats"**: nominal (not measured) geometry throughout -- GEOM-05 (hand-eye), GEOM-07 (TCP/boresight) and MOT-10 (workcell survey) have not yet been applied to this chain's inputs; `execution_eligible` is `false` by design (same reasons MOT-06.10/MOT-07.9 already document); mock/dry only, no real-mode attempt; the view-plan leg's own retime/execute is out of this script's scope (see MOT-07.10.5); single scenario (default flat scene, no tilt/holes/depth-noise) -- says nothing about a tilted surface, partial coverage, or the generator's hole-interpolation behaviour (already covered directly by `docs/geometry/PATH3D_VERIFICATION.md`).
5. One or two lines appended to `docs/motion/ROS_WORKSPACE.md`'s existing script-entry list for `run_integration_e2e.sh`, in the same style as its existing entries (e.g. the `retime_plan` entry MOT-07.9 added).

## Out of scope
- Any change to `scripts/perc/run_synthetic_perception.sh` or `scripts/ros/run_integration_e2e.sh` themselves (INT-01.1/INT-01.2's files) beyond what's needed to capture the one documented run's output.
- A requirements-to-evidence map entry (OPS-03's job).

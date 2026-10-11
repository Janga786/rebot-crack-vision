# MOT-07.6 — Independent trajectory checker (conda side), part 1: schema, provenance, FD limits, §11.6 margin, rest and sampling + CLI

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-07.1"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/trajectory_check.py",
      "tests/test_trajectory_check.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#16",
    "docs/adr/019-time-parameterization.md",
    "config/robot/b601_dm_limits.yaml",
    "config/motion/execution.yaml",
    "src/crackvision/logging_setup.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_trajectory_check.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "independent",
        "cmd": "test -f src/crackvision/trajectory_check.py && ! grep -nE '^\\s*(from|import)\\s+(crackvision_motion|rclpy)' src/crackvision/trajectory_check.py",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "cli-help",
        "cmd": "./env.sh python -m crackvision.trajectory_check --help",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Implementation is written from the §11.6/§11.7/§16 text alone. It parses the limits YAML itself and does not import crackvision_motion; FD velocity uses forward differences and FD acceleration uses (v_i − v_(i−1)) / ((t_(i+1) − t_(i−1))/2), exactly as §11.7 defines them.",
      "Every check runs and is reported even after an earlier failure; a check whose precondition failed is reported as error. Report fields and outcome vocabulary are exactly §16.5, including per-joint peak FD velocity/acceleration utilisation.",
      "C-PROVENANCE: trajectory shas vs plan.sources and vs the files passed (limits, end_effector), source.paths3d equal to the plan's, and the trajectory's start/end equal to the plan's first/last configuration.",
      "Tests with hand-built trajectories show each check catching its fault: overspeed on one segment, an acceleration spike, a margin violation (the §11.6 counter-example), start/end not reachable from rest, a gap > max_period_s or a step > max_joint_step_rad, a wrong sha, an altered source.paths3d, and non-monotone t_s. A clean hand-built trajectory passes, and the §11.6 home-approach example passes.",
      "CLI follows §0 (common flags, logs/trajectory_check_*.log/.json, exit codes 0/1/2/3) and writes the report to --out (default next to the trajectory as <trajectory>.check.json)."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "verification",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.6",
  "title": "Independent trajectory checker (conda side), part 1: schema, provenance, FD limits, §11.6 margin, rest and sampling + CLI",
  "parent": "MOT-07",
  "outcome": "`./env.sh python -m crackvision.trajectory_check` independently re-implements and reports C-SCHEMA, C-PROVENANCE, C-LIMITS, C-MARGIN, C-REST and C-SAMPLING from §16.5 for a trajectory + plan, writing a crackvision.trajectory_check/1 report with exit 0 pass / 3 fail / 2 usage, sharing no code with crackvision_motion."
}
```

Create `src/crackvision/trajectory_check.py` (conda py3.11; numpy/PyYAML) as the independent verifier required by REQ-MOT-2. Independence is the point:
- do not read or copy `joint_trajectory.py`/`retime*.py`;
- implement from the contract text;
- load §11.2/§13 documents with your own minimal strict parsing.

Structure:
- one function per check returning `{check, outcome, detail, numbers}`;
- `run_checks(traj_doc, plan_doc, *, limits, margin_rad, retime_cfg, files) -> report`;
- `main(argv)`.

C-PATH, C-TOOLSPEED and C-REPORT are added by MOT-07.7. Until then, do not emit them as pass; omit them or report them as not implemented, as §16.5 allows. The CLI flags follow §16.6.

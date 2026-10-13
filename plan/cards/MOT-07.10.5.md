# MOT-07.10.5 — Headless mock run: view_plan → retime_plan --view-plan → trajectory_check → execute_trajectory dry/mock

```json card
{
  "kind": "integration",
  "depends_on": [
    "MOT-07.10.2",
    "MOT-07.10.3",
    "MOT-07.10.4"
  ],
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "scripts/ros/test_view_retime_execute.sh",
      "docs/motion/TIME_PARAMETERIZATION.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#17",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#16",
    "scripts/ros/test_retime_execute.sh",
    "scripts/ros/run_cartesian_synthetic.sh",
    "scripts/ros/_mock_stack.sh"
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
        "id": "view-mock-run",
        "cmd": "bash scripts/ros/test_view_retime_execute.sh",
        "timeout_s": 2400,
        "expect_exit": 0
      },
      {
        "id": "crosscheck-regression",
        "cmd": "bash scripts/ros/test_retime_crosscheck.sh",
        "timeout_s": 1200,
        "expect_exit": 0
      },
      {
        "id": "docs",
        "cmd": "bash -c 'grep -q -- \"--view-plan\" docs/motion/TIME_PARAMETERIZATION.md && grep -q view_move docs/motion/TIME_PARAMETERIZATION.md'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The script follows test_retime_execute.sh's pattern: it refuses if reBotArmControl is running, never sets CRACKVISION_ARM_REAL, uses only the moveit_mock profile through _mock_stack.sh with trap teardown, and writes only under logs/.",
      "Steps and assertions: run_cartesian_synthetic.sh's scratch view_plan.json → retime_plan --view-plan (exit 0) → trajectory_check with --report (exit 0, every check pass/skip as §16.5 allows) → execute_trajectory --mode dry (exit 0; G-TRAJ and G-ELIGIBLE pass in gate_report) → --mode mock (exit 0; the final joint_states are within tolerance of transit_leg's last configuration) → --mode real without CRACKVISION_ARM_REAL (exit 3).",
      "A tampered copy (view_plan sha mutated in the trajectory) is refused at G-ELIGIBLE in mock (exit 3, nothing sent).",
      "TIME_PARAMETERIZATION.md gains a 'View move' section with the exact commands, the §17 rules in prose, and the statement that real view moves remain gated by commissioning."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "ros-integration",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.10.5",
  "title": "Headless mock run: view_plan → retime_plan --view-plan → trajectory_check → execute_trajectory dry/mock",
  "parent": "MOT-07.10",
  "outcome": "scripts/ros/test_view_retime_execute.sh proves, on the mock stack only, that the synthetic view plan's transit leg can be retimed, independently checked and executed. It also shows the real-only refusals the view move still gets, and TIME_PARAMETERIZATION.md documents the view-move workflow."
}
```

Write scripts/ros/test_view_retime_execute.sh, modelled on scripts/ros/test_retime_execute.sh.

- The transit leg starts at the SRDF all-zero home, which matches the mock stack's startup state, so no pre-positioning move should be needed. If G-START-STATE still refuses, pre-position with a `purpose: "test"` mock move as test_retime_execute.sh does, and say so in the script header.
- Recover the scratch root from run_cartesian_synthetic.sh's output.
- Pass `--root` consistently, so `source.view_plan.path` resolves.
- For the tamper case, edit a copy of the trajectory with python3 -c (change source.view_plan.sha256) and expect exit 3 with G-ELIGIBLE fail in the record.
- Print a PASS summary.

Add the 'View move' section to docs/motion/TIME_PARAMETERIZATION.md. Never write under data/ or config/, and never commit logs.

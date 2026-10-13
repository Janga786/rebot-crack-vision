# MOT-07.5.R1 — Repair MOT-07.5: --dry-run per-leg summary omits configurations and refined-point count

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/test/test_retime_plan_cli.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#16",
    "docs/adr/019-time-parameterization.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/retime.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "ros2_ws/src/crackvision_motion/setup.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_retime_plan_cli.py -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "entry-point",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; ros2 run crackvision_motion retime_plan --help'",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "ros-free",
        "cmd": "test -f ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py && ! grep -nE '^\\s*(from|import)\\s+(rclpy|[a-z_]+_msgs)' ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py ros2_ws/src/crackvision_motion/crackvision_motion/retime.py ros2_ws/src/crackvision_motion/crackvision_motion/retime_core.py",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Each printed --dry-run leg line includes kind, the plan leg's configuration count, the refined/output point count (e.g. from point_range), and duration. test_dry_run_writes_nothing_and_prints_summary asserts these fields are present for every leg.",
      "A test makes the self-check fail (for example by monkeypatching retime.check_limits to return a violation, or by using a limits/margin combination that violates) and asserts main(argv) == EXIT_PRECONDITION and that no trajectory or .retime.json file exists.",
      "All acceptance criteria of MOT-07.5 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 4,
    "task_class": "python-cli",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 91,
  "repairs": "MOT-07.5",
  "findings": [
    "b64c5a564662a58f",
    "0648ed1a7296e363"
  ],
  "id": "MOT-07.5.R1",
  "title": "Repair MOT-07.5: --dry-run per-leg summary omits configurations and refined-point count",
  "parent": "MOT-07",
  "outcome": "Resolve the review findings on MOT-07.5 while every acceptance criterion of MOT-07.5 still holds."
}
```

Repair work generated deterministically from review attempt `00869-MOT-07.5-review` of `MOT-07.5`.
Original card: `plan/cards/MOT-07.5.md` — its acceptance criteria must still hold.

## Finding 1 [minor] --dry-run per-leg summary omits configurations and refined-point counts
- Evidence: retime_plan.py:118-123 `_print_leg_summary` prints only f"leg {leg_index} ({kind}): duration=..." (+ v_c for trace legs). Observed output: 'leg 0 (transit): duration=1.5627s', 'leg 2 (trace): duration=1.0327s v_c=0.03954 m/s'. The card criterion requires '(kind, configurations, refined points, provisional duration)'.
- Consequence: An operator previewing with --dry-run cannot see how many plan configurations each leg has or how densely refinement sampled it, so the explicit acceptance criterion is not met.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py, ros2_ws/src/crackvision_motion/test/test_retime_plan_cli.py
- Acceptance condition: Each printed --dry-run leg line includes kind, the plan leg's configuration count, the refined/output point count (e.g. from point_range), and duration. test_dry_run_writes_nothing_and_prints_summary asserts these fields are present for every leg.

## Finding 2 [minor] No main(argv) test for the self-check-violation refusal (exit 3)
- Evidence: The test file has exit-3 tests only for sha mismatch, invalid plan, missing plan and trace below min speed. Nothing triggers G-SELFCHECK (retime.py:409-414). The criterion says each exit code case, including 'self-check violation', is tested via main(argv).
- Consequence: If the RetimeError-to-PreconditionError mapping or the no-write guarantee regresses for self-check failures, no test catches it.
- Affected: ros2_ws/src/crackvision_motion/test/test_retime_plan_cli.py
- Acceptance condition: A test makes the self-check fail (for example by monkeypatching retime.check_limits to return a violation, or by using a limits/margin combination that violates) and asserts main(argv) == EXIT_PRECONDITION and that no trajectory or .retime.json file exists.

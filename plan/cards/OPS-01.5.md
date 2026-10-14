# OPS-01.5 — run_trajectory helper: retime -> trajectory_check -> trajectory_preview -> approve_trajectory -> execute_trajectory, one reusable sequence

```json card
{
  "kind": "integration",
  "depends_on": [
    "OPS-01.1"
  ],
  "requirements": [
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "scripts/operator/run_trajectory.sh",
      "scripts/ros/test_operator_run_trajectory.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#19. Operator CLI workflow",
    "docs/motion/EXECUTION.md",
    "docs/motion/PREVIEW_APPROVAL.md",
    "docs/INTERFACES.md#17. View-move trajectories",
    "scripts/ros/test_preview_approve.sh",
    "scripts/ros/test_view_retime_execute.sh",
    "scripts/ros/env_ros.sh",
    "ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "e2e-mock",
        "cmd": "bash scripts/ros/test_operator_run_trajectory.sh",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "`scripts/operator/run_trajectory.sh {preview|approve-execute} --plan PATH|--view-plan PATH [--mode mock|dry|real] [--speed-scale S] [--root PATH] ...` exists; `preview` runs retime_plan -> trajectory_check -> trajectory_preview and stops (writes the preview manifest + PNGs for the operator to inspect, never approves or executes); `approve-execute` runs approve_trajectory (real /dev/tty typed `APPROVE <sha8>`, never automated by this script itself) then execute_trajectory with the given --mode, refusing to run if no matching preview exists yet.",
      "Every conda-side step (retime_plan, trajectory_check, trajectory_preview, approve_trajectory) is invoked via `./env.sh python -m crackvision.<module>`; execute_trajectory is invoked via `scripts/ros/env_ros.sh` + the built overlay (`ros2 run crackvision_motion execute_trajectory`), matching the project's ROS/conda split (ADR-007).",
      "`--mode` defaults to `mock`; the script never sets `CRACKVISION_ARM_REAL` and never adds a flag that skips the preview or approval step before calling execute_trajectory, in any mode.",
      "`scripts/ros/test_operator_run_trajectory.sh` exercises this headlessly against the mock stack using the same test-only pseudo-tty APPROVE technique as `scripts/ros/test_preview_approve.sh` (clearly commented as test-only, never a bypass), on both a view_plan and a cartesian_plan input, asserting a `mock`-mode execution record with G-APPROVAL pass for each, and that `preview` alone never produces an approval or execution record.",
      "No hardware, no vendor driver, no real mode exercised by the test script; it guards against a live `reBotArmControl` process exactly as sibling `scripts/ros/test_*.sh` scripts do."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "motion-integration",
    "local_ok": false
  },
  "motion": true,
  "id": "OPS-01.5",
  "title": "run_trajectory helper: retime -> trajectory_check -> trajectory_preview -> approve_trajectory -> execute_trajectory, one reusable sequence",
  "parent": "OPS-01",
  "outcome": "One operator-invokable script that takes any plan file (a view_plan or a cartesian_plan) and drives it through the full, ungapped retime/check/preview/approve/execute sequence exactly as EXECUTION.md and PREVIEW_APPROVAL.md specify, defaulting to --mode mock, used by both the capture stage's optional view move and the top-level preview/execute stages."
}
```

Write `scripts/operator/run_trajectory.sh`, a thin sequencing script (no new planning/retiming/checking/preview/approval/execution logic -- every step is an existing accepted CLI). Model its two subcommands directly on the already-proven chains in `scripts/ros/test_preview_approve.sh` (view_plan case) and `scripts/ros/test_view_retime_execute.sh`/`run_cartesian_synthetic.sh` (cartesian_plan case):

- `preview`: `retime_plan` (`--plan` or `--view-plan`, mutually exclusive per §17.3) -> `trajectory_check` -> `trajectory_preview`. Print the resulting `data/previews/{case}_{sha8}/` directory path (per §18.1) on success.
- `approve-execute`: given the same trajectory (re-derive its preview directory the same way `trajectory_preview` names it), run `approve_trajectory` then `execute_trajectory --mode MODE --approval FILE [--profile ...] [--speed-scale S]`.

Keep all scratch output under a caller-supplied `--out-root` (default `logs/operator/<case>/`), never under `data/`/`config/` except the standard `data/trajectories/`, `data/previews/`, `data/execution_approvals/`-style locations those underlying CLIs already use by default -- do not relocate their defaults.

For `scripts/ros/test_operator_run_trajectory.sh`, copy the structure of `scripts/ros/test_preview_approve.sh` (same mock-stack bring-up/teardown via `_mock_stack.sh`, same `script -qec` pseudo-tty trick clearly marked test-only) but drive it through your new `run_trajectory.sh` wrapper instead of calling each tool by hand, and add a second pass using `run_cartesian_synthetic.sh`'s cartesian_plan output so both plan kinds are exercised.

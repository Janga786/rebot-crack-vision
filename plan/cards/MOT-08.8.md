# MOT-08.8 — Headless mock run: view plan → retime → check → preview → approve (pty) → execute_trajectory dry with G-APPROVAL pass, tamper refusals

```json card
{
  "kind": "integration",
  "depends_on": [
    "MOT-08.3",
    "MOT-08.4",
    "MOT-08.5",
    "MOT-08.7"
  ],
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "scripts/ros/test_preview_approve.sh"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#18",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#17",
    "scripts/ros/test_view_retime_execute.sh",
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
        "id": "preview-approve",
        "cmd": "bash scripts/ros/test_preview_approve.sh",
        "timeout_s": 2400,
        "expect_exit": 0
      },
      {
        "id": "view-regression",
        "cmd": "bash scripts/ros/test_view_retime_execute.sh",
        "timeout_s": 2400,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The script follows test_view_retime_execute.sh: it refuses if reBotArmControl is running, never sets CRACKVISION_ARM_REAL, uses only the moveit_mock profile with trap teardown, and writes only under logs/ (preview and approval go to --out-dir/--out under the scratch dir, never data/).",
      "Steps, each asserted: run_cartesian_synthetic.sh scratch view_plan → retime_plan --view-plan → trajectory_check (exit 0) → trajectory_preview (exit 0, valid manifest, all_pass true) → approve_trajectory. The approval is driven through a pseudo-tty with util-linux `script` and the phrase piped in; this is documented in the header as test-only. A wrong phrase gives exit 3 and no file.",
      "execute_trajectory --mode dry --approval <file> exits 0 with G-APPROVAL pass in gate_report.",
      "With a byte-modified copy of toolpath.png in a copied preview dir, the copied approval gives G-APPROVAL warn in dry. An approval for a different trajectory gives warn. --dry-run --mode real shows G-APPROVAL pass for the genuine approval. --mode real without CRACKVISION_ARM_REAL exits 3."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "ros-integration",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-08.8",
  "title": "Headless mock run: view plan → retime → check → preview → approve (pty) → execute_trajectory dry with G-APPROVAL pass, tamper refusals",
  "parent": "MOT-08",
  "outcome": "scripts/ros/test_preview_approve.sh proves on the mock stack only that a retimed view-move trajectory can be previewed and approved, and that execute_trajectory then reports G-APPROVAL pass. It also shows that tampering with the preview, approving a different trajectory, or running real without arming is refused or warned per §18."
}
```

Write scripts/ros/test_preview_approve.sh, modelled on scripts/ros/test_view_retime_execute.sh. The view-move trajectory starts at the all-zero home, which matches the mock stack's startup state, so no pre-positioning move is needed.

- Conda-side tools run via `./env.sh python -m ...`. ROS tools run via env_ros.sh plus the overlay.
- Pass `--root "$REPO_ROOT"` everywhere so repo-relative paths resolve.
- Approval via pty: `printf 'APPROVE %s\n' "$SHA8" | script -qec "./env.sh python -m crackvision.approve_trajectory --preview ... --operator ci-mock --out $WORK/approval.json" /dev/null`. Assert the exit code and that the file exists. Repeat with a wrong phrase and assert exit 3 and no file.
- Tamper cases work on copies (cp -r the preview dir and approval, rewrite the paths in the copy's preview_artifacts with python3 -c, then flip one byte of the PNG). Parse G-APPROVAL's outcome from the execution record JSON in --record-dir.
- Print a PASS summary. Never write under data/ or config/.

## Debugger note (D0013, 2026-10-14T06:28:26Z)
Blocked on MOT-08.10. approve_trajectory's `_open_tty()` opened /dev/tty with 'r+', which always raises io.UnsupportedOperation on a tty, so the typed approval always refused with exit 3. Once MOT-08.10 is accepted, resume from partial commit 04308af (scripts/ros/test_preview_approve.sh already exists), re-run all three checks and fix only what still fails. Keep the `script -qec` pty approach the card specifies.

# OPS-01.8 — operator_cli.sh dispatcher: doctor/capture/segment/paths/plan/preview/execute, resumable 'run', end-to-end headless test + OPERATOR_WORKFLOW.md

```json card
{
  "kind": "integration",
  "depends_on": [
    "OPS-01.3",
    "OPS-01.4",
    "OPS-01.5",
    "OPS-01.6",
    "OPS-01.7"
  ],
  "requirements": [
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "scripts/operator_cli.sh",
      "scripts/ros/test_operator_cli_e2e.sh",
      "docs/OPERATOR_WORKFLOW.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#19. Operator CLI workflow",
    "scripts/operator_doctor.py",
    "scripts/operator_segment_paths.py",
    "scripts/operator/run_trajectory.sh",
    "scripts/operator/capture.sh",
    "scripts/operator/plan.sh",
    "src/crackvision/operator_state.py",
    "docs/motion/EXECUTION.md",
    "docs/motion/PREVIEW_APPROVAL.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "e2e",
        "cmd": "bash scripts/ros/test_operator_cli_e2e.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "help",
        "cmd": "./env.sh bash scripts/operator_cli.sh --help",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "`scripts/operator_cli.sh {doctor,capture,segment,paths,plan,preview,execute,status,run} [--case-id CASE] [...]` dispatches to the stage scripts built in OPS-01.3/.4/.6/.7 and OPS-01.5's run_trajectory.sh (`preview` = `run_trajectory.sh preview` on the case's plan trajectory; `execute` = `run_trajectory.sh approve-execute`), with `status` printing `crackvision.operator_state`'s table.",
      "`run --case-id CASE` executes doctor once, then every case-scoped stage in order, skipping any stage `crackvision.operator_state.stage_is_fresh` already reports fresh, and stopping at the first stage that fails (non-zero exit), leaving the run-state file recording exactly how far it got.",
      "A second `run` for the same case-id (nothing changed) reports every stage as already-done/skipped and exits 0 without re-invoking any underlying tool.",
      "`execute` is reachable only after `preview` has produced a current preview for the case's trajectory (enforced via the §19 freshness/dependency check, not a separate ad hoc flag), and never bypasses `approve_trajectory`.",
      "`scripts/ros/test_operator_cli_e2e.sh` runs the whole chain headlessly against the mock stack and a synthetic case (`--synthetic` capture, MOT-06.10-style fixtures) through `run --case-id CASE --mode mock`, then re-runs `run` and asserts full-skip, then deliberately invalidates one mid-chain file (e.g. touches the paths3d file) and re-runs, asserting it resumes from exactly the invalidated stage rather than redoing earlier ones.",
      "`docs/OPERATOR_WORKFLOW.md` walks an operator through doctor -> capture -> segment -> paths -> plan -> preview -> execute end to end, states plainly that it adds nothing to and never relaxes EXECUTION.md/PREVIEW_APPROVAL.md's gates, and documents `run`'s resumability and failure-recovery behaviour."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 4,
    "consequence": 4,
    "task_class": "motion-integration",
    "local_ok": false
  },
  "motion": true,
  "id": "OPS-01.8",
  "title": "operator_cli.sh dispatcher: doctor/capture/segment/paths/plan/preview/execute, resumable 'run', end-to-end headless test + OPERATOR_WORKFLOW.md",
  "parent": "OPS-01",
  "outcome": "One operator-facing entry point (`scripts/operator_cli.sh`) exposing every stage plus a `run` subcommand that resumes a case from its first not-done stage and stops at the first failure, proven end to end headlessly on a synthetic case through mock execution, and documented for operators."
}
```

Write `scripts/operator_cli.sh` as the single top-level entry point, dispatching by subcommand to the already-built per-stage scripts -- this card adds no new stage logic, only the dispatcher, the `run` resumability loop, the end-to-end test, and operator docs.

`run --case-id CASE [--view-plan PATH] [--mode mock|dry|real] [--force] [--stage NAME]`: call `crackvision.operator_state.load`, walk the fixed stage order from §19, call `stage_is_fresh` before each, invoke the stage's script when stale, stop immediately (non-zero exit, clear message naming the failed stage) on any failure. `--stage NAME` restricts to one stage (still subject to the prerequisite check). Without `--stage`, run everything from the first stale stage onward.

For `scripts/ros/test_operator_cli_e2e.sh`: compose the mock-stack bring-up (reuse `_mock_stack.sh` as other `scripts/ros/test_*.sh` do) with a synthetic scratch case (reuse MOT-06.10's generator/fixture approach via `--root` pointed at a scratch dir, never the repo's real `data/`), then:
1. `scripts/operator_cli.sh run --case-id synth_ops --synthetic --mode mock` -> expect exit 0, every stage done.
2. Re-run the same command -> expect exit 0 and every stage logged as skipped (grep the log for a 'skip'/'fresh' marker your dispatcher prints).
3. `touch`/corrupt one mid-chain output file (invalidate its sha) and re-run -> expect it resumes starting at the stage that owns that file (assert via the printed stage sequence or the run-state file), not from `capture` again.

Write `docs/OPERATOR_WORKFLOW.md` as the operator-facing walkthrough, explicitly cross-referencing `docs/motion/EXECUTION.md` and `docs/motion/PREVIEW_APPROVAL.md` as the normative gate sources this CLI only sequences, never relaxes -- same disclaimer style those two documents already use.

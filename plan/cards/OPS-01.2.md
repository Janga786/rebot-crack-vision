# OPS-01.2 — crackvision.operator_state: per-case run-state file library + status/reset CLI

```json card
{
  "kind": "impl",
  "depends_on": [
    "OPS-01.1"
  ],
  "requirements": [
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/operator_state.py",
      "tests/test_operator_state.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#19. Operator CLI workflow",
    "src/crackvision/capture_record.py",
    "src/crackvision/logging_setup.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_operator_state.py -v",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "load/save round-trips the exact schema from §19; a missing file returns a fresh all-pending state rather than raising.",
      "mark_stage(case_id, stage, status, outputs, root) records sha256 of each declared output path and an updated_utc timestamp, and refuses an unknown stage name.",
      "stage_is_fresh(case_id, stage, root) implements the full cascade rule from §19: false if the stage is not status=='done', if any recorded output file is missing or its sha256 no longer matches, or if any declared upstream dependency stage is itself not fresh.",
      "The `status`/`reset` CLI (`./env.sh python -m crackvision.operator_state status --case CASE [--root]` and `... reset --case CASE --stage NAME [--root]`) follows INTERFACES §0 (exit codes, --root/--config/-v/-q), prints a readable per-stage table, and reset clears the named stage and every stage downstream of it to pending without touching unrelated case files.",
      "Unit tests cover: fresh-state creation, a stale upstream invalidating a downstream 'done' stage, a tampered output file being detected, and reset's downstream cascade."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 2,
    "task_class": "python-lib",
    "local_ok": true
  },
  "id": "OPS-01.2",
  "title": "crackvision.operator_state: per-case run-state file library + status/reset CLI",
  "parent": "OPS-01",
  "outcome": "A pure-Python library and CLI that creates, loads, updates and freshness-checks a case's crackvision.operator_run/1 file per INTERFACES §19, used by every later OPS-01 stage script."
}
```

Implement `src/crackvision/operator_state.py` exactly against INTERFACES §19 (written by OPS-01.1 -- read it first; if it's still missing wait/ask, do not invent your own schema). Follow this repo's existing module conventions (`crackvision.config.Config`, `crackvision.logging_setup` for `EXIT_OK`/`EXIT_USAGE`/`EXIT_PRECONDITION`, `add_common_args`/`setup_logging`, the §0.4 logging-artefact convention) -- model it on `src/crackvision/capture_record.py`'s CLI shape (argparse subcommands, `RunSummary`).

Public functions: `load(case_id, root) -> dict`, `save(state, root) -> None`, `mark_stage(case_id, stage, status, outputs, root, *, notes=None) -> dict`, `stage_is_fresh(case_id, stage, root) -> bool`, and a `STAGE_DEPENDS_ON` mapping (e.g. `{"segment": (), "paths": ("segment",), "plan": ("capture", "paths"), "preview": ("plan",), "execute": ("preview",)}` -- take the exact dependency graph from §19, this is illustrative only). `capture` has no stage dependency (it is the first case-scoped stage). `doctor` is out of scope for this module (it is global, not case-scoped; OPS-01.3 handles it separately).

CLI: `python -m crackvision.operator_state {status,reset} --case CASE_ID [--root PATH] [--stage NAME] [-v|-q]`. `status` with no `--case` lists every case under `data/operator_runs/`. Write `tests/test_operator_state.py` with fixtures under a `tmp_path` root (never touch the real `data/` tree).

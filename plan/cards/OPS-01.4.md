# OPS-01.4 — segment + paths stages: single-case wrapper over prepare_inputs/inference/skeleton/paths/visualize_paths

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
      "scripts/operator_segment_paths.py",
      "tests/test_operator_segment_paths.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#19. Operator CLI workflow",
    "run_test.sh",
    "src/crackvision/prepare_inputs.py",
    "src/crackvision/inference.py",
    "src/crackvision/skeleton.py",
    "src/crackvision/paths.py",
    "src/crackvision/visualize_paths.py",
    "src/crackvision/naming.py",
    "src/crackvision/operator_state.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_operator_segment_paths.py -v",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Supports `--case-id CASE --source IMAGE --stage {segment,paths,both} [--root] [--force]`; `segment` runs prepare_inputs then inference for just that one source (reusing their existing `main(argv)` entry points, not shelling out to run_test.sh's whole-folder scan); `paths` runs skeleton then paths then visualize_paths for that case_id.",
      "Before running a requested stage, calls crackvision.operator_state.stage_is_fresh and skips (with a clear log line) unless --force or the recorded state is stale, then calls crackvision.operator_state.mark_stage with the naming.py-derived output paths (mask_path, overlay_path, skeleton_path, etc.) on success, and records status=='failed' (not a silent pass) on any non-zero exit from a delegated stage.",
      "Running `paths` before `segment` has produced a mask for that case_id exits 3 (precondition not met), per §0.2, rather than crashing.",
      "Unit tests use small synthetic fixtures (reuse existing tests/ fixtures or the smoke_test pattern) and never require the real OpenCrack checkpoint; mock/monkeypatch the heavy inference call if needed, but segment/paths wiring logic itself must be exercised."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 3,
    "consequence": 2,
    "task_class": "cli-orchestration",
    "local_ok": true
  },
  "id": "OPS-01.4",
  "title": "segment + paths stages: single-case wrapper over prepare_inputs/inference/skeleton/paths/visualize_paths",
  "parent": "OPS-01",
  "outcome": "One command that takes one case's source image through segmentation (prepare_inputs+inference) and ordered pixel paths (skeleton+paths+visualize_paths), resumable via crackvision.operator_state, exactly reusing run_test.sh's existing per-stage logic rather than re-implementing it."
}
```

Build `scripts/operator_segment_paths.py` as a thin, case-scoped orchestrator over four already-accepted modules -- do not change their behaviour or duplicate their logic, only sequence them and wire the result into `crackvision.operator_state`. Model the import-and-call pattern (not subprocess) on how `run_test.sh` chains stages, but operate on one `--case-id`/`--source` pair instead of a whole `data/input_originals/` folder: call `crackvision.prepare_inputs.main([...])`, then `crackvision.inference.main([...])`, then `crackvision.skeleton.main([...])`, `crackvision.paths.main([...])`, `crackvision.visualize_paths.main([...])` with the right per-case flags (check each module's own `--input-dir`/`--output-dir`/case-filtering flags; fall back to running them unfiltered and checking only the one case_id's output exists, if no single-case filter flag exists -- do not add one to those modules, this card only writes the three files listed in scope).

Follow the §0.3 convention: `--skip-existing` is this script's own `--force`-inverse default (default is "regenerate", matching §0.3 for each delegated generating CLI), but the operator_state freshness check is an additional, separate skip layer on top (so a second `operator_cli.sh run` for the same case doesn't even invoke the delegated CLIs when nothing changed).

In tests, use a tiny synthetic image + a stub/fake segmentation mask (do not need the real checkpoint); assert the operator_state transitions (pending -> done, and -> failed on an injected subprocess/library failure) and the precondition-exit-3 ordering rule.

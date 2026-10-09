# LLM-04 — Project benchmark harness for local coding models

A repeatable, project-representative benchmark that drives the local-model route
(`~/claude-auto/claude_auto/localcoder.run_local`) exactly as the scheduler's `runner=local`
would, so the eventual model choice (REQ-LLM-1) is backed by this repo's own task patterns
instead of vendor leaderboard numbers.

## Layout

- `bench/llm/tasks.py` — 13 tasks across 3 classes (`script`, `test-writing`,
  `mechanical-edit`), each with a `setup` (starter files), a `reference` fixture (a known-good
  solution), an automated `checker`, and (for 11 of the 13) a `broken` fixture (a seeded wrong
  solution the checker must reject).
- `bench/llm/harness.py` — `self_test()` (no model) and `run_benchmark()` (starts/stops a
  pinned `llama-server` per candidate, routes each task through `localcoder.run_local`, records
  metrics).
- `bench/llm/run_bench.py` — CLI: `--self-test`, `--list`, or a real run (`--models`, `--out`,
  `--ctx-size`, `--ngl`, `--port`, `--max-turns`, `--task-timeout-s`).

## Task set

Drawn from this repo's own real patterns (not generic coding-exercise boilerplate), per
`docs/INTERFACES.md` §0-§1 and `~/claude-auto/claude_auto/{plan,sandbox}.py`:

| id | class | drawn from |
|---|---|---|
| `script_case_id` | script | §1.1 `derive_case_id` + its worked-example fixture table |
| `script_exit_codes` | script | §0.2/§0.3 exit-code and common-flag convention |
| `script_atomic_write` | script | `claude_auto.util.atomic_write_json` pattern |
| `script_sha256_verify` | script | `scripts/llm/verify_models.py`'s sha256-check pattern |
| `script_log_summary` | script | §0.4 machine log summary schema |
| `test_resize_guard` | test-writing | §0.5 frame invariant |
| `test_scope_path_matches` | test-writing | `claude_auto.plan.path_matches`/`glob_re` (`*` vs `**`) |
| `test_case_registry` | test-writing | §1.1 case-id collision suffixing (`__2`, `__3`, …) |
| `test_rw_scope_validation` | test-writing | `claude_auto.sandbox.validate_extra_rw` |
| `edit_offbyone_branch_count` | mechanical-edit | skeleton-stats-style off-by-one fix |
| `edit_add_quiet_flag` | mechanical-edit | §0.3 `--quiet`/`-q` convention, added to an existing CLI |
| `edit_rename_mask_path` | mechanical-edit | `naming.py::mask_path` real function name, cross-file rename |
| `edit_add_peak_vram_field` | mechanical-edit | `RunResult`-style dataclass field + consumer update |

Every `test-writing` checker runs the model-written test file with pytest against the correct
module (must pass) and then swaps in a seeded mutant module (must fail) — this rules out a
vacuous test (e.g. `assert True`) being scored as a pass. Every `mechanical-edit`/`script`
checker drives the edited file directly (import, or subprocess for CLI-exit-code/flag behaviour)
against concrete input/output pairs.

## `--self-test`

```
$ python3 bench/llm/run_bench.py --self-test
...
13 tasks across 3 classes, self-test PASSED
$ echo $?
0
```

For every task this applies the `reference` fixture and asserts the checker passes, then (where
a `broken` fixture exists) applies it and asserts the checker fails — proving the checkers
themselves work before any model is ever scored against them. No model, GPU, network or
`llama-server` process is involved; it runs in under a second.

## Real benchmark run

```
python3 bench/llm/run_bench.py --models all --out bench/llm/out
```

For each verified entry in `config/llm_models.json` (written by LLM-03): starts the pinned
`llama-server` (`config/llm_runtime.json`, from LLM-02) with `--jinja` against that GGUF on an
on-demand port, waits for `/health`, then for every task spins up a fresh scratch directory,
calls `localcoder.run_local(endpoint=..., model=..., repo=<scratch dir>, allowed=task.allowed, ...)`
exactly as `localcoder.scheduler_runner` would for a real `route=local` card, runs the task's
checker against whatever the model produced, and records:

- **pass@1** — the checker's verdict, aggregated per model.
- **tool-call validity** — fraction of tool calls that did not return an `ERROR`/`REFUSED`
  result, captured by temporarily wrapping `localcoder.Tools.call` for the duration of each task
  (not a persistent patch — restored immediately after).
- **wall time** — `RunResult.duration_s` from `run_local`.
- **tokens/s** — summed `usage.completion_tokens` from each `/v1/chat/completions` reply
  (captured the same way, by wrapping `localcoder.chat`) divided by wall time.
- **peak VRAM** — a background thread sampling `nvidia-smi --query-gpu=memory.used` every 0.5s
  for the duration of each task, maximum per task and per model-serving session.

The server is stopped (`SIGTERM`, `SIGKILL` after a 10s grace period) after each model's tasks
finish, so VRAM returns to perception/MoveIt before the next model loads — mirroring the
on-demand, never-co-resident serving rule in `config/llm_candidates.json` /
`docs/llm/RUNTIME.md` §Isolation.

### Smoke-tested end to end (2026-10-09)

One model (`Qwen3-Coder-30B-A3B-Instruct`, IQ4_XS) against one task
(`edit_add_quiet_flag`), `--ctx-size 8192`, to confirm the full
start-server → run_local → checker → stop-server path works, not just `--self-test`'s
no-model path:

```
pass=true, wall_time_s=3.4, completion_tokens=457, tokens_per_s=134.41,
tool_calls=3, tool_call_validity=1.0, peak_vram_mb=16770.0
```

`nvidia-smi` confirmed VRAM returned to the 131 MiB idle baseline and no `llama-server` process
remained after the run. The full 13-task x 3-model benchmark (needed to actually pick a model
for REQ-LLM-1) was not run in this session — each full pass loads a ~15-17 GB model and runs an
agentic tool loop per task, so it belongs in its own dedicated run/card rather than this harness
card.

## Exit codes (`docs/INTERFACES.md` §0.2)

| Code | Meaning |
|---|---|
| 0 | `--self-test` / `--list` / a real run with no per-model errors |
| 1 | a `--self-test` checker failed, or at least one model errored during a real run |
| 2 | (reserved — argparse usage errors use its own standard exit code 2) |
| 3 | precondition not met for a real run: no pinned `llama-server` binary, no verified entry in `config/llm_models.json`, or `--models` matched nothing |

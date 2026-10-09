# LLM-05 — Benchmark results, model selection, task-class qualification

Real-model benchmark run (2026-10-09) of all three LLM-01 candidates, verified/downloaded by
LLM-03, served by the pinned LLM-02 `llama-server`, against the full LLM-04 task set (13 tasks,
3 classes), via

```
./env.sh python3 bench/llm/run_bench.py --models all --out evidence/llm/results
```

Identical settings for all three: `--ctx-size 32768 --ngl 999 --port 8099 --max-turns 40`, per-task
timeout from each task's own `timeout_s` in `bench/llm/tasks.py` (300s for every task in this set).
Full per-task metrics and per-task model transcripts are committed under
`evidence/llm/results/<model-slug>/<task-id>/`; the raw aggregate is
`evidence/llm/results/report.json`.

Two of the three candidate GGUFs (`Qwen3.6-27B-Q4_K_M.gguf`, 16.82 GiB; and the bartowski
`Laguna-XS-2.1-Q3_K_M.gguf` requant, 15.56 GiB) had already been deleted from
`~/models/llm` by an earlier LLM-05 attempt that ran the benchmark but errored before its
result ever reached a structured output or this repo (no qualification evidence, no results, no
commit from that attempt survived). Both were re-downloaded directly from the exact
`repo`/`file` in `config/llm_candidates.json` and their sha256 re-verified against the hashes
already recorded in `config/llm_models.json` before this run, so `config/llm_models.json` itself
did not need to change.

## Results

### Overall pass@1

| Model | Quant | pass@1 | peak server VRAM | avg wall time/task | avg tokens/s | avg tool-call validity | `finish`-call misses (max_turns) |
|---|---|---|---|---|---|---|---|
| Qwen3.6-27B | Q4_K_M | 12/13 = 0.923 | 18638 MB | 57.9 s | 36.4 | 1.000 | 0/13 |
| **Laguna-XS-2.1** | Q3_K_M | **13/13 = 1.000** | **16966 MB** | 28.3 s | 121.6 | 0.982 | 3/13 |
| Qwen3-Coder-30B-A3B-Instruct | IQ4_XS | 11/13 = 0.846 | 19102 MB | 6.4 s | 147.7 | 1.000 | 0/13 |

### Pass rate by task class

| Model | script (5 tasks) | test-writing (4 tasks) | mechanical-edit (4 tasks) |
|---|---|---|---|
| Qwen3.6-27B | 4/5 = 0.80 | 4/4 = 1.00 | 4/4 = 1.00 |
| Laguna-XS-2.1 | 5/5 = 1.00 | 4/4 = 1.00 | 4/4 = 1.00 |
| Qwen3-Coder-30B-A3B-Instruct | 4/5 = 0.80 | 3/4 = 0.75 | 4/4 = 1.00 |

Both `Qwen3.6-27B` and `Qwen3-Coder-30B-A3B-Instruct` failed the same task, `script_case_id`
(`derive_case_id('....png') == 'png', expected 'image'`) — both misread the worked-example
mapping from `docs/INTERFACES.md` §1.1 the same way. `Qwen3-Coder` additionally failed
`test_scope_path_matches` by writing a test that asserted the wrong expected value for a
single-`*` glob matching an extra directory level (its own generated test disagreed with
`claude_auto.plan.path_matches`'s real, correct behaviour).

### Tool reliability note — Laguna-XS-2.1 `max_turns` outcomes

3 of Laguna-XS-2.1's 13 tasks (`test_resize_guard`, `test_scope_path_matches`,
`test_case_registry`, all `test-writing`) ran out the full 40-turn budget without the model ever
calling the `finish` tool, even though by the time turns ran out the task's file(s) were already
in a passing state (the harness checks final on-disk state regardless of whether `finish` was
called, so these still scored `pass=true`). This is a real latency/reliability cost (full 40 turns
burned instead of stopping early) and is the main argument against Laguna-XS-2.1, weighed below
against its otherwise-best pass@1 and lowest VRAM footprint.

## Qualification rule and why only `script` qualifies

Per this card: a task class is qualified for local routing only if it has **≥5 tasks** in the
benchmark **and** the selected model scores **≥80%** pass on them. The LLM-04 harness has 5
`script` tasks but only 4 `test-writing` and 4 `mechanical-edit` tasks — so regardless of model or
pass rate, `test-writing` and `mechanical-edit` can never satisfy the `≥5 tasks` statistical-floor
half of the rule with the current task set. This is a property of the (already-accepted) LLM-04
harness's task distribution, not something this card changes. Only `script` (5 tasks) is eligible
to qualify at all, and every candidate scored ≥80% on it (4/5, 5/5, 4/5 respectively).

## Selection: Laguna-XS-2.1 (Q3_K_M GGUF)

REQ-LLM-1's selection dimensions, evaluated on this machine:

- **Quantization / context**: Q3_K_M, 262144-token native context (run here at 32768, matching
  `config/llm_candidates.json`'s `min_context_tokens`).
- **Tool reliability**: 0.982 average tool-call validity (vs 1.000 for the other two), but the
  `max_turns` pattern above is a real caveat — it never produced a tool-call error, but it also
  never explicitly signalled done.
- **Latency**: slower than Qwen3-Coder (28.3s vs 6.4s avg) but faster than Qwen3.6-27B (57.9s);
  well within interactive range for the bounded coding tasks this route handles.
- **License**: OpenMDW-1.1 (permissive, MIT-like; see `docs/llm/SELECTION.md`).
- **VRAM coexistence**: 16966 MB peak — the lowest of the three, leaving the most headroom under
  the 24 GB budget in `config/llm_candidates.json` for safely handing the GPU back to
  nnU-Net/MoveIt between local-model sessions.

Laguna-XS-2.1 had the best pass@1 (13/13) and the best `script` pass rate (5/5, the only class
that can be routed), at the lowest VRAM cost. Correctness and VRAM headroom were weighted above
raw speed because this route exists to safely offload bounded coding tasks, not to win a latency
contest, and the two faster candidates both got the same real task wrong. It is the selection.

`Qwen3.6-27B-Q4_K_M.gguf` and `Qwen3-Coder-30B-A3B-Instruct-IQ4_XS.gguf` were deleted from
`~/models/llm` after this run (not from Hugging Face, per `config/llm_candidates.json`'s
deletion policy).

## Disk usage after selection

```
$ ls ~/models/llm/
Laguna-XS-2.1-Q3_K_M.gguf

$ du -sh ~/models/llm/*
15G     /home/boosterk1/models/llm/Laguna-XS-2.1-Q3_K_M.gguf

$ df -h ~/models/llm
Filesystem      Size  Used Avail Use% Mounted on
/dev/nvme1n1p2  468G  362G   83G  82% /home/boosterk1/models/llm
```

83 GB free after deleting the two non-selected GGUFs (32.4 GB reclaimed), comfortably above the
`min_free_disk_gb: 40` floor in `config/llm_models.json`.

## Qualification evidence

`evidence/llm/qualification.json` (checked by this card's `qual` acceptance check):

- `runner_ok: true`
- `model: "Laguna-XS-2.1-Q3_K_M.gguf"`
- `classes: ["script"]`
- `thresholds: {min_pass_rate: 0.8, min_tasks: 5}`
- `serving`: `endpoint http://127.0.0.1:8080`, `start_argv` (absolute paths: pinned
  `llama-server` binary from `config/llm_runtime.json`, `-m` the kept GGUF, `--jinja`,
  `--ctx-size 32768`, `--n-gpu-layers 999`), `ctx: 32768`, `start_timeout_s: 240`.

This is the file `scheduler.py`'s `publishes_local_qualification` copies to
`.claude-auto/local_qualification.json`, which is what `localcoder.scheduler_runner` and
`Scheduler.local_classes()` actually read at run time — only cards whose task class is `"script"`
and whose `runner` is `local` will ever be routed to this model; every other class continues to
run on the existing (non-local) runner.

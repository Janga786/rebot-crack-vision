# LLM-05 — Run benchmark, select model, qualify task classes

**Accepted:** 2026-10-09 · **Work package:** Local coding model (L-LLM) · **Track:** automation · **Verified commit:** [`9d2e3d8`](https://github.com/Janga786/rebot-crack-vision/commit/9d2e3d8e3a7fa63b9346b929de9466f47121c33d)

## What was done

Ran the LLM-04 benchmark harness against all 3 LLM-01/03 candidates with identical settings (13 tasks, 3 classes each), selected Laguna-XS-2.1 (Q3_K_M GGUF, pass@1=13/13, lowest VRAM at 16966 MB) for local routing, published evidence/llm/qualification.json (runner_ok=true, classes=["script"]) and docs/llm/BENCHMARK_RESULTS.md with the results tables, and deleted the two non-selected GGUFs (32.4 GB reclaimed, 83 GB free remaining).

Files changed:
- [docs/llm/BENCHMARK_RESULTS.md](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/docs/llm/BENCHMARK_RESULTS.md) (+129 / −0)
- [evidence/llm/qualification.json](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/qualification.json) (+66 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/edit_add_peak_vram_field/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/edit_add_peak_vram_field/local_transcript.jsonl) (+5 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/edit_add_quiet_flag/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/edit_add_quiet_flag/local_transcript.jsonl) (+8 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/edit_offbyone_branch_count/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/edit_offbyone_branch_count/local_transcript.jsonl) (+4 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/edit_rename_mask_path/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/edit_rename_mask_path/local_transcript.jsonl) (+17 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/llama_server.log](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/llama_server.log) (+1710 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/script_atomic_write/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/script_atomic_write/local_transcript.jsonl) (+10 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/script_case_id/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/script_case_id/local_transcript.jsonl) (+11 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/script_exit_codes/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/script_exit_codes/local_transcript.jsonl) (+15 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/script_log_summary/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/script_log_summary/local_transcript.jsonl) (+9 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/script_sha256_verify/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/script_sha256_verify/local_transcript.jsonl) (+14 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/test_case_registry/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/test_case_registry/local_transcript.jsonl) (+40 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/test_resize_guard/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/test_resize_guard/local_transcript.jsonl) (+40 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/test_rw_scope_validation/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/test_rw_scope_validation/local_transcript.jsonl) (+19 / −0)
- [evidence/llm/results/Laguna-XS-2.1-Q3_K_M/test_scope_path_matches/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Laguna-XS-2.1-Q3_K_M/test_scope_path_matches/local_transcript.jsonl) (+40 / −0)
- [evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/edit_add_peak_vram_field/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/edit_add_peak_vram_field/local_transcript.jsonl) (+3 / −0)
- [evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/edit_add_quiet_flag/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/edit_add_quiet_flag/local_transcript.jsonl) (+4 / −0)
- [evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/edit_offbyone_branch_count/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/edit_offbyone_branch_count/local_transcript.jsonl) (+3 / −0)
- [evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/edit_rename_mask_path/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/edit_rename_mask_path/local_transcript.jsonl) (+10 / −0)
- [evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/llama_server.log](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/llama_server.log) (+613 / −0)
- [evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/script_atomic_write/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/script_atomic_write/local_transcript.jsonl) (+5 / −0)
- [evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/script_case_id/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/script_case_id/local_transcript.jsonl) (+8 / −0)
- [evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/script_exit_codes/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/script_exit_codes/local_transcript.jsonl) (+8 / −0)
- [evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/script_log_summary/local_transcript.jsonl](https://github.com/Janga786/rebot-crack-vision/blob/9d2e3d8e3a7fa63b9346b929de9466f47121c33d/evidence/llm/results/Qwen3-Coder-30B-A3B-Instruct-IQ4_XS/script_log_summary/local_transcript.jsonl) (+3 / −0)
- … and 22 more

Commits: [`9d2e3d8`](https://github.com/Janga786/rebot-crack-vision/commit/9d2e3d8e3a7fa63b9346b929de9466f47121c33d)

## Why it was done

Evidence-based model choice; only task classes with ≥80% pass are qualified for local routing; losers deleted.

- Requirement **REQ-LLM-1**: Local coding model chosen from current primary sources + a project benchmark (quantization, context, tool reliability, latency, license, VRAM coexistence) with bounded downloads
- Requirement **REQ-LLM-2**: Local model served by a pinned runtime; only benchmark-qualified task classes are routed to it

## How it moves the project forward

- Local coding model: **5/6** tasks accepted; whole project: **65/104**.
- REQ-LLM-1: 4/4 contributing tasks done — **requirement satisfied**
- REQ-LLM-2: 3/4 contributing tasks done
- Verification: automated checks run by the pipeline itself (qual ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “LLM-05's benchmark run, model selection, and task-class qualification all check out against independently re-verified evidence: report.json's per-model pass@1/VRAM numbers match qualification.json and BENCHMARK_RESULTS.md exactly, the two documented failures and the 3 max_turns-ceiling Laguna transcripts were confirmed directly in the raw data, non-selected GGUFs are actually deleted from disk (only Laguna-XS-2.1 remains, disk usage matches claims), the scheduler's qual acceptance check passes, and the change stays within its declared scope with a clean working tree.”

## What it unlocks next

- **Can now be planned in detail:** LLM-06 — Local-route canary on real low-risk cards
- Closer: OPS-03 — Handoff package + final readiness report (still needs OPS-02, INT-02, OPS-02)

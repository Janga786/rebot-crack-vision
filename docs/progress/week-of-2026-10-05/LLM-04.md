# LLM-04 — Project benchmark harness for local coding models

**Accepted:** 2026-10-09 · **Work package:** Local coding model (L-LLM) · **Track:** automation · **Verified commit:** [`7422797`](https://github.com/Janga786/rebot-crack-vision/commit/742279774263c0178b6492f3ba3fad705f45bb8d)

## What was done

Built a benchmark harness with 13 realistic coding tasks (writing scripts, writing tests, fixing/editing existing code) drawn from this project's own conventions, each with an automated pass/fail check. Running it in self-test mode (no AI model needed) confirms all 13 checks correctly accept right answers and reject wrong ones, exiting cleanly with status 0. I also did one real end-to-end trial run with an actual local AI model doing one of the tasks, to confirm the harness can start the model server, hand it a task, time it, measure its GPU memory use and token speed, grade its answer, and shut the server back down -- it worked, completing the task correctly in 3.4 seconds.

Files changed:
- [bench/llm/__init__.py](https://github.com/Janga786/rebot-crack-vision/blob/742279774263c0178b6492f3ba3fad705f45bb8d/bench/llm/__init__.py) (+0 / −0)
- [bench/llm/harness.py](https://github.com/Janga786/rebot-crack-vision/blob/742279774263c0178b6492f3ba3fad705f45bb8d/bench/llm/harness.py) (+255 / −0)
- [bench/llm/run_bench.py](https://github.com/Janga786/rebot-crack-vision/blob/742279774263c0178b6492f3ba3fad705f45bb8d/bench/llm/run_bench.py) (+129 / −0)
- [bench/llm/tasks.py](https://github.com/Janga786/rebot-crack-vision/blob/742279774263c0178b6492f3ba3fad705f45bb8d/bench/llm/tasks.py) (+1065 / −0)
- [docs/llm/BENCHMARK.md](https://github.com/Janga786/rebot-crack-vision/blob/742279774263c0178b6492f3ba3fad705f45bb8d/docs/llm/BENCHMARK.md) (+115 / −0)

Commits: [`7422797`](https://github.com/Janga786/rebot-crack-vision/commit/742279774263c0178b6492f3ba3fad705f45bb8d)

## Why it was done

A repeatable benchmark of project-representative tasks drives the local route exactly as the scheduler would.

- Requirement **REQ-LLM-1**: Local coding model chosen from current primary sources + a project benchmark (quantization, context, tool reliability, latency, license, VRAM coexistence) with bounded downloads
- Requirement **REQ-LLM-2**: Local model served by a pinned runtime; only benchmark-qualified task classes are routed to it

## How it moves the project forward

- Local coding model: **4/6** tasks accepted; whole project: **64/104**.
- REQ-LLM-1: 3/4 contributing tasks done
- REQ-LLM-2: 2/4 contributing tasks done
- Verification: automated checks run by the pipeline itself (selftest ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 1 not verifiable without hardware).
- Audit summary: “LLM-04 delivers a benchmark harness strictly within bench/llm/** and docs/llm/BENCHMARK.md. Self-test independently reproduced (13 tasks, 3 classes: 5 script/4 test-writing/4 mechanical-edit, all pass, exit 0). Harness's run_benchmark/run_local/Tools/RunResult usage matches the real ~/claude-auto/claude_auto/localcoder.py signatures and config/llm_runtime.json + config/llm_models.json schemas verified to exist with the expected keys. No scope violations.”

## What it unlocks next

- **Ready to start:** LLM-05 — Run benchmark, select model, qualify task classes

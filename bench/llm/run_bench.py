#!/usr/bin/env python3
"""CLI entry point for the local-coding-model benchmark harness (LLM-04).

    python3 bench/llm/run_bench.py --self-test           # validate checkers, no model, no GPU
    python3 bench/llm/run_bench.py --list                 # list the task set
    python3 bench/llm/run_bench.py --models all           # run the full benchmark on every
                                                            # candidate in config/llm_models.json
    python3 bench/llm/run_bench.py --models Qwen3-Coder    # run on candidates whose name/file
                                                            # contains this substring

Exit codes follow docs/INTERFACES.md §0.2: 0 success, 1 runtime failure, 2 usage/config error,
3 precondition not met (e.g. no verified models in config/llm_models.json for a real run).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from bench.llm.harness import run_benchmark, self_test  # noqa: E402
from bench.llm.tasks import TASKS  # noqa: E402


def _read_json(path: Path):
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None


def cmd_self_test() -> int:
    ok, rows = self_test(TASKS)
    by_task: dict[str, list[dict]] = {}
    for row in rows:
        by_task.setdefault(row["task"], []).append(row)
    for task_id, task_rows in by_task.items():
        cls = task_rows[0]["cls"]
        status = "PASS" if all(r["passed"] for r in task_rows) else "FAIL"
        print(f"[{status}] {task_id} ({cls})")
        for r in task_rows:
            mark = "ok" if r["passed"] else "FAIL"
            print(f"    {r['phase']:>9}: {mark} - {r['detail']}")
    n_tasks = len(by_task)
    n_classes = len({r["cls"] for r in rows})
    print(f"\n{n_tasks} tasks across {n_classes} classes, self-test {'PASSED' if ok else 'FAILED'}")
    return 0 if ok else 1


def cmd_list() -> int:
    for task in TASKS:
        print(f"{task.id}\t{task.cls}")
    return 0


def _select_models(models_filter: str) -> tuple[list[dict], str | None]:
    runtime = _read_json(REPO_ROOT / "config" / "llm_runtime.json")
    if not runtime or not runtime.get("binary"):
        return [], "config/llm_runtime.json is missing or has no pinned llama-server binary (run LLM-02 first)"
    binary = runtime["binary"]
    if not Path(binary).exists():
        return [], f"pinned llama-server binary not found at {binary} (run LLM-02 first)"

    manifest = _read_json(REPO_ROOT / "config" / "llm_models.json")
    candidates = (manifest or {}).get("models") or []
    if not candidates:
        return [], "config/llm_models.json has no verified models (run LLM-03 first)"

    if models_filter != "all":
        wanted = [m.strip() for m in models_filter.split(",") if m.strip()]
        candidates = [c for c in candidates
                     if any(w.lower() in c["name"].lower() or w.lower() in c["file"].lower() for w in wanted)]
        if not candidates:
            return [], f"no verified model in config/llm_models.json matches --models {models_filter!r}"

    models = []
    for c in candidates:
        if not Path(c["path"]).exists():
            return [], f"model file listed in config/llm_models.json is missing on disk: {c['path']}"
        models.append({"name": c["name"], "path": c["path"], "binary": binary,
                       "slug": Path(c["file"]).stem, "served_name": c["file"]})
    return models, None


def cmd_run(args: argparse.Namespace) -> int:
    models, err = _select_models(args.models)
    if err:
        print(f"precondition not met: {err}", file=sys.stderr)
        return 3
    print(f"running {len(TASKS)} tasks on {len(models)} model(s): {', '.join(m['name'] for m in models)}")
    report = run_benchmark(models=models, tasks=TASKS, out_dir=Path(args.out), port=args.port,
                           ctx_size=args.ctx_size, ngl=args.ngl, max_turns=args.max_turns,
                           task_timeout_s=args.task_timeout_s)
    any_error = False
    for m in report["models"]:
        if m.get("error"):
            any_error = True
            print(f"[{m['name']}] ERROR: {m['error']}")
            continue
        print(f"[{m['name']}] pass@1={m['pass_at_1']:.2f} peak_vram_mb={m['server_peak_vram_mb']:.0f}")
    print(f"full report written to {Path(args.out) / 'report.json'}")
    return 1 if any_error else 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--self-test", action="store_true", help="validate task checkers, no model/GPU/network")
    p.add_argument("--list", action="store_true", help="list task ids and classes, then exit")
    p.add_argument("--models", default="all", help="'all', or a comma-separated list of name/file substrings")
    p.add_argument("--out", default=str(REPO_ROOT / "bench" / "llm" / "out"), help="output dir for reports")
    p.add_argument("--port", type=int, default=8099)
    p.add_argument("--ctx-size", type=int, default=32768)
    p.add_argument("--ngl", type=int, default=999, help="--n-gpu-layers passed to llama-server")
    p.add_argument("--max-turns", type=int, default=40)
    p.add_argument("--task-timeout-s", type=int, default=900)
    args = p.parse_args(argv)

    if args.self_test:
        return cmd_self_test()
    if args.list:
        return cmd_list()
    return cmd_run(args)


if __name__ == "__main__":
    sys.exit(main())

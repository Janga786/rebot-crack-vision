"""Benchmark harness core (LLM-04): a self-test mode that validates the task checkers without any
model, and a real-model mode that starts/stops a pinned llama-server per candidate GGUF and routes
each task through `~/claude-auto/claude_auto/localcoder.run_local` exactly as the scheduler's
route=local would, recording pass@1, tool-call validity, wall time, tokens/s and peak VRAM.
"""
from __future__ import annotations

import contextlib
import json
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
from pathlib import Path

from .tasks import TASKS, CheckResult, Task


def _safe_check(task: Task, workdir: Path) -> CheckResult:
    """Run a task's checker, converting any exception raised by the checked-in code (a broken
    fixture, or a real model's output) into a normal failing result instead of crashing the run."""
    try:
        return task.checker(workdir)
    except Exception as e:  # noqa: BLE001 — the checked code is untrusted, any exception is a fail
        return False, f"checker raised {type(e).__name__}: {e}"

CLAUDE_AUTO_HOME = Path.home() / "claude-auto"
REPO_ROOT = Path(__file__).resolve().parent.parent.parent
SYSTEM_PROMPT = (
    "You are a careful coding assistant working inside a sandboxed repository checkout. Use the "
    "provided tools (read_file, list_dir, write_file, replace_in_file, run) to complete the task "
    "exactly as instructed, then call finish with status and a short summary. Do not ask "
    "clarifying questions; make a reasonable choice and proceed."
)


def _import_localcoder():
    if str(CLAUDE_AUTO_HOME) not in sys.path:
        sys.path.insert(0, str(CLAUDE_AUTO_HOME))
    from claude_auto import localcoder  # noqa: PLC0415
    return localcoder


# -------------------------------------------------------------------------------------------
# self-test: no model, no network, no GPU. Validates each task's checker against its own
# reference (must pass) and seeded-broken (must fail) fixtures.
# -------------------------------------------------------------------------------------------

def self_test(tasks: list[Task] = TASKS) -> tuple[bool, list[dict]]:
    ok = True
    rows: list[dict] = []
    for task in tasks:
        with tempfile.TemporaryDirectory(prefix=f"bench-{task.id}-ref-") as d:
            workdir = Path(d)
            task.setup(workdir)
            task.reference(workdir)
            passed, detail = _safe_check(task, workdir)
        rows.append({"task": task.id, "cls": task.cls, "phase": "reference", "passed": passed,
                     "detail": detail})
        if not passed:
            ok = False
        if task.broken is not None:
            with tempfile.TemporaryDirectory(prefix=f"bench-{task.id}-broken-") as d:
                workdir = Path(d)
                task.setup(workdir)
                task.broken(workdir)
                broken_passed, broken_detail = _safe_check(task, workdir)
            rows.append({"task": task.id, "cls": task.cls, "phase": "broken", "passed": not broken_passed,
                         "detail": ("checker correctly rejected the broken fixture" if not broken_passed
                                   else "checker incorrectly accepted the broken fixture: " + broken_detail)})
            if broken_passed:
                ok = False
    return ok, rows


# -------------------------------------------------------------------------------------------
# real-model benchmark
# -------------------------------------------------------------------------------------------

class VramSampler:
    """Polls `nvidia-smi` for GPU 0 memory.used and tracks the peak seen while active."""

    def __init__(self, interval_s: float = 0.5):
        self.interval_s = interval_s
        self.peak_mb = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def _poll_once(self) -> float | None:
        try:
            out = subprocess.run(
                ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits", "-i", "0"],
                capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            return None
        if out.returncode != 0:
            return None
        try:
            return float(out.stdout.strip().splitlines()[0])
        except (ValueError, IndexError):
            return None

    def _run(self) -> None:
        while not self._stop.is_set():
            v = self._poll_once()
            if v is not None:
                self.peak_mb = max(self.peak_mb, v)
            self._stop.wait(self.interval_s)

    def __enter__(self) -> "VramSampler":
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)


@contextlib.contextmanager
def _capture_chat_metrics(localcoder_mod):
    """Monkeypatch `localcoder.chat`/`Tools.call` for the duration of one run_local call to
    recover per-call token usage and tool-call validity without touching claude-auto's own code."""
    sink = {"usage": [], "tool_calls": 0, "tool_errors": 0}
    orig_chat = localcoder_mod.chat
    orig_call = localcoder_mod.Tools.call

    def chat_wrapper(endpoint, model, messages, timeout=600, extra=None):
        reply = orig_chat(endpoint, model, messages, timeout=timeout, extra=extra)
        if isinstance(reply, dict) and isinstance(reply.get("usage"), dict):
            sink["usage"].append(reply["usage"])
        return reply

    def call_wrapper(self, name, a):
        out = orig_call(self, name, a)
        sink["tool_calls"] += 1
        if isinstance(out, str) and (out.startswith("ERROR") or out.startswith("REFUSED")):
            sink["tool_errors"] += 1
        return out

    localcoder_mod.chat = chat_wrapper
    localcoder_mod.Tools.call = call_wrapper
    try:
        yield sink
    finally:
        localcoder_mod.chat = orig_chat
        localcoder_mod.Tools.call = orig_call


def start_llama_server(*, binary: str, model_path: str, port: int, ctx_size: int, ngl: int,
                       extra_args: list[str] | None = None, log_path: Path) -> subprocess.Popen:
    argv = [binary, "-m", model_path, "--jinja", "--host", "127.0.0.1", "--port", str(port),
            "--ctx-size", str(ctx_size), "--n-gpu-layers", str(ngl)]
    argv += extra_args or []
    log_path.parent.mkdir(parents=True, exist_ok=True)
    logf = open(log_path, "w")
    return subprocess.Popen(argv, stdout=logf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            start_new_session=True)


def wait_healthy(localcoder_mod, endpoint: str, proc: subprocess.Popen, timeout_s: int = 240) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if localcoder_mod.healthy(endpoint):
            return True
        if proc.poll() is not None:
            return False
        time.sleep(1)
    return False


def stop_process(proc: subprocess.Popen | None, grace_s: float = 10.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    import os
    import signal
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        proc.terminate()
    try:
        proc.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            proc.kill()
        proc.wait(timeout=grace_s)


def run_benchmark(*, models: list[dict], tasks: list[Task], out_dir: Path, host: str = "127.0.0.1",
                  port: int = 8099, ctx_size: int = 32768, ngl: int = 999, max_turns: int = 40,
                  task_timeout_s: int = 900, server_start_timeout_s: int = 240,
                  sandbox_enabled: bool = True, extra_server_args: list[str] | None = None) -> dict:
    localcoder = _import_localcoder()
    endpoint = f"http://{host}:{port}"
    out_dir.mkdir(parents=True, exist_ok=True)
    report = {"endpoint": endpoint, "ctx_size": ctx_size, "models": []}

    for model_cfg in models:
        slug = model_cfg.get("slug") or model_cfg["name"].split()[0]
        model_out = out_dir / slug
        model_report = {"name": model_cfg["name"], "path": model_cfg["path"], "tasks": [],
                        "server_peak_vram_mb": 0.0}
        proc = start_llama_server(binary=model_cfg["binary"], model_path=model_cfg["path"], port=port,
                                  ctx_size=ctx_size, ngl=ngl, extra_args=extra_server_args,
                                  log_path=model_out / "llama_server.log")
        try:
            if not wait_healthy(localcoder, endpoint, proc, timeout_s=server_start_timeout_s):
                model_report["error"] = f"llama-server did not become healthy within {server_start_timeout_s}s"
                report["models"].append(model_report)
                continue
            for task in tasks:
                task_out = model_out / task.id
                with tempfile.TemporaryDirectory(prefix=f"bench-{slug}-{task.id}-") as d:
                    workdir = Path(d)
                    task.setup(workdir)
                    with VramSampler() as sampler, _capture_chat_metrics(localcoder) as metrics:
                        try:
                            res = localcoder.run_local(
                                endpoint=endpoint, model=model_cfg.get("served_name", "local"),
                                system=SYSTEM_PROMPT, prompt=task.prompt, repo=workdir,
                                allowed=task.allowed, out_dir=task_out,
                                sandbox_kw={"ro_in_repo": [], "runtime_dir": ".claude-auto"},
                                sandbox_enabled=sandbox_enabled, max_turns=max_turns,
                                timeout_s=min(task.timeout_s, task_timeout_s))
                        except (urllib.error.URLError, OSError, TimeoutError) as e:
                            class _Fallback:
                                outcome, detail, duration_s = "transient", str(e), 0.0
                            res = _Fallback()
                    passed, detail = _safe_check(task, workdir)
                completion_tokens = sum(u.get("completion_tokens", 0) for u in metrics["usage"])
                duration_s = getattr(res, "duration_s", 0.0) or 0.0
                tok_per_s = (completion_tokens / duration_s) if duration_s > 0 else 0.0
                tool_calls = metrics["tool_calls"]
                tool_validity = (1.0 - metrics["tool_errors"] / tool_calls) if tool_calls else None
                model_report["server_peak_vram_mb"] = max(model_report["server_peak_vram_mb"], sampler.peak_mb)
                model_report["tasks"].append({
                    "task": task.id, "cls": task.cls, "outcome": getattr(res, "outcome", "error"),
                    "pass": passed, "detail": detail, "wall_time_s": duration_s,
                    "completion_tokens": completion_tokens, "tokens_per_s": round(tok_per_s, 2),
                    "tool_calls": tool_calls, "tool_call_validity": tool_validity,
                    "peak_vram_mb": sampler.peak_mb,
                })
        finally:
            stop_process(proc)
        n = len(model_report["tasks"])
        model_report["pass_at_1"] = (sum(1 for t in model_report["tasks"] if t["pass"]) / n) if n else 0.0
        report["models"].append(model_report)

    (out_dir / "report.json").write_text(json.dumps(report, indent=2))
    return report

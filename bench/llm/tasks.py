"""Project-representative benchmark tasks for the local-coding-model harness (LLM-04).

Each `Task` is a self-contained unit: `setup` populates a scratch workdir with the starter
file(s) a model would see, `prompt` is the instruction given to the model, and `checker` runs an
automated pass/fail check against whatever files end up in that workdir afterwards. `reference`
and `broken` exist only for `--self-test`: they let the harness simulate a correct and an
incorrect solution respectively, without calling any model, so the checkers themselves can be
validated. A task with no `broken` fixture is still checked for the positive (reference) case.

Task content is drawn from this repo's own real patterns (docs/INTERFACES.md §0-§1, the sandbox's
`path_matches`/`validate_extra_rw`, `localcoder.RunResult`-style dataclasses) rather than generic
coding-exercise boilerplate, per the card's acceptance criteria.
"""
from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

CheckResult = tuple[bool, str]


@dataclass
class Task:
    id: str
    cls: str  # "script" | "test-writing" | "mechanical-edit"
    prompt: str
    setup: Callable[[Path], None]
    reference: Callable[[Path], None]
    checker: Callable[[Path], CheckResult]
    broken: Callable[[Path], None] | None = None
    allowed: list[str] = field(default_factory=lambda: ["**"])
    timeout_s: int = 300


def _write(workdir: Path, rel: str, content: str) -> None:
    p = workdir / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(content)


def _load_module(path: Path, attr_name: str | None = None):
    """Import a fresh module from a scratch-dir file. Each task workdir is a unique tempdir and
    gets a unique module name, so repeated self-test runs never hit a stale sys.modules entry."""
    mod_name = f"bench_llm_{uuid.uuid4().hex}"
    spec = importlib.util.spec_from_file_location(mod_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod
    try:
        spec.loader.exec_module(mod)
    finally:
        del sys.modules[mod_name]
    if attr_name is not None and not hasattr(mod, attr_name):
        raise AttributeError(f"{path} does not define {attr_name!r}")
    return mod


def _run_py(workdir: Path, rel: str, args: list[str], timeout: int = 20) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "-I", str(workdir / rel), *args], cwd=workdir,
                          capture_output=True, text=True, timeout=timeout)


def _run_pytest(workdir: Path, rel: str, timeout: int = 60) -> subprocess.CompletedProcess:
    import os
    # Disable third-party plugin autoload: an unrelated globally-installed plugin (anyio's) can be
    # version-mismatched against the system pytest and crash collection before any test runs.
    env = {**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"}
    return subprocess.run([sys.executable, "-m", "pytest", "-q", rel], cwd=workdir,
                          capture_output=True, text=True, timeout=timeout, env=env)


# --------------------------------------------------------------------------------------------
# class: script — write a new file from a written spec, no starter code to edit
# --------------------------------------------------------------------------------------------

def _case_id_setup(workdir: Path) -> None:
    _write(workdir, "README.md", "Implement case_id.py as instructed in the task prompt.\n")


_CASE_ID_IMPL = '''
import re


def derive_case_id(source: str) -> str:
    stem = source.rsplit("/", 1)[-1]
    if "." in stem:
        stem = stem.rsplit(".", 1)[0]
    s = re.sub(r"[^A-Za-z0-9-]", "_", stem)
    s = re.sub(r"_+", "_", s).strip("_")
    if s.endswith("_0000"):
        s = s[:-5].rstrip("_")
    if not s:
        s = "image"
    return s
'''

_CASE_ID_CASES = [
    ("Deck Crack 01.JPG", "Deck_Crack_01"),
    ("IMG_2043.png", "IMG_2043"),
    ("bridge-pier.7.jpeg", "bridge-pier_7"),
    ("crack (copy).png", "crack_copy"),
    ("  spaced  .bmp", "spaced"),
    ("sample_0000.png", "sample"),
    ("2026-09-16_run1.tif", "2026-09-16_run1"),
    ("....png", "image"),
]


def _case_id_reference(workdir: Path) -> None:
    _write(workdir, "case_id.py", _CASE_ID_IMPL)


def _case_id_broken(workdir: Path) -> None:
    # Off-by-spec: never strips the trailing "_0000" marker.
    _write(workdir, "case_id.py", _CASE_ID_IMPL.replace(
        '    if s.endswith("_0000"):\n        s = s[:-5].rstrip("_")\n', ""))


def _case_id_checker(workdir: Path) -> CheckResult:
    path = workdir / "case_id.py"
    if not path.exists():
        return False, "case_id.py was not created"
    mod = _load_module(path, "derive_case_id")
    for source, expected in _CASE_ID_CASES:
        got = mod.derive_case_id(source)
        if got != expected:
            return False, f"derive_case_id({source!r}) == {got!r}, expected {expected!r}"
    return True, "all case_id worked examples matched"


def _exit_codes_setup(workdir: Path) -> None:
    _write(workdir, "README.md", "Implement cli_tool.py as instructed in the task prompt.\n")


_EXIT_CODES_IMPL = '''
import argparse
import sys


def main(argv=None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--device", default="cpu")
    p.add_argument("--root")
    p.add_argument("--config")
    p.add_argument("--verbose", "-v", action="store_true")
    p.add_argument("--quiet", "-q", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--skip-existing", action="store_true")
    args = p.parse_args(argv)
    if args.device not in ("cpu", "cuda"):
        print(f"invalid --device {args.device!r}", file=sys.stderr)
        return 2
    print("ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
'''


def _exit_codes_reference(workdir: Path) -> None:
    _write(workdir, "cli_tool.py", _EXIT_CODES_IMPL)


def _exit_codes_broken(workdir: Path) -> None:
    # Accepts any device string, so the usage-error path never fires.
    _write(workdir, "cli_tool.py", _EXIT_CODES_IMPL.replace(
        '    if args.device not in ("cpu", "cuda"):\n        print(f"invalid --device {args.device!r}", file=sys.stderr)\n        return 2\n',
        ""))


def _exit_codes_checker(workdir: Path) -> CheckResult:
    if not (workdir / "cli_tool.py").exists():
        return False, "cli_tool.py was not created"
    bad = _run_py(workdir, "cli_tool.py", ["--device", "banana"])
    if bad.returncode != 2:
        return False, f"--device banana exited {bad.returncode}, expected 2"
    ok = _run_py(workdir, "cli_tool.py", ["--device", "cuda"])
    if ok.returncode != 0 or "ok" not in ok.stdout:
        return False, f"--device cuda exited {ok.returncode} stdout={ok.stdout!r}, expected 0 / 'ok'"
    default = _run_py(workdir, "cli_tool.py", [])
    if default.returncode != 0:
        return False, f"no args exited {default.returncode}, expected 0 (default device cpu)"
    return True, "exit codes 0/2 matched the usage-error convention"


def _atomic_write_setup(workdir: Path) -> None:
    _write(workdir, "README.md", "Implement atomic_io.py as instructed in the task prompt.\n")


_ATOMIC_WRITE_IMPL = '''
import json
import os
import uuid
from pathlib import Path


def atomic_write_json(path, obj) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.name}.{uuid.uuid4().hex}.tmp")
    tmp.write_text(json.dumps(obj))
    os.replace(tmp, path)
'''


def _atomic_write_reference(workdir: Path) -> None:
    _write(workdir, "atomic_io.py", _ATOMIC_WRITE_IMPL)


def _atomic_write_broken(workdir: Path) -> None:
    # Writes straight to the destination path: not atomic, and happens to leave a ".tmp" sibling.
    _write(workdir, "atomic_io.py", '''
import json
from pathlib import Path


def atomic_write_json(path, obj) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(obj))
''')


def _atomic_write_checker(workdir: Path) -> CheckResult:
    path = workdir / "atomic_io.py"
    if not path.exists():
        return False, "atomic_io.py was not created"
    mod = _load_module(path, "atomic_write_json")
    target = workdir / "out" / "result.json"
    mod.atomic_write_json(target, {"a": 1, "b": [1, 2, 3]})
    if not target.exists():
        return False, "target file was not written"
    if json.loads(target.read_text()) != {"a": 1, "b": [1, 2, 3]}:
        return False, "target file content does not round-trip"
    leftovers = [p for p in target.parent.iterdir() if p != target]
    if leftovers:
        return False, f"leftover temp file(s) after write: {[p.name for p in leftovers]}"
    return True, "atomic_write_json round-tripped with no leftover temp file"


def _sha256_setup(workdir: Path) -> None:
    _write(workdir, "README.md", "Implement verify.py as instructed in the task prompt.\n")
    _write(workdir, "data.bin", "hello world\n")


_SHA256_IMPL = '''
import hashlib
from pathlib import Path


def verify_file(path, expected_sha256: str) -> bool:
    path = Path(path)
    if not path.exists():
        return False
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest() == expected_sha256
'''


def _sha256_reference(workdir: Path) -> None:
    _write(workdir, "verify.py", _SHA256_IMPL)


def _sha256_broken(workdir: Path) -> None:
    # Raises instead of returning False when the file is missing -- violates the stated contract.
    _write(workdir, "verify.py", '''
import hashlib
from pathlib import Path


def verify_file(path, expected_sha256: str) -> bool:
    path = Path(path)
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest() == expected_sha256
''')


def _sha256_checker(workdir: Path) -> CheckResult:
    path = workdir / "verify.py"
    if not path.exists():
        return False, "verify.py was not created"
    mod = _load_module(path, "verify_file")
    import hashlib
    real = hashlib.sha256((workdir / "data.bin").read_bytes()).hexdigest()
    if mod.verify_file(workdir / "data.bin", real) is not True:
        return False, "correct hash did not verify as True"
    if mod.verify_file(workdir / "data.bin", "0" * 64) is not False:
        return False, "wrong hash did not verify as False"
    if mod.verify_file(workdir / "missing.bin", real) is not False:
        return False, "missing file must return False, not raise"
    return True, "verify_file handled match/mismatch/missing-file cases"


def _log_summary_setup(workdir: Path) -> None:
    _write(workdir, "README.md", "Implement logsummary.py as instructed in the task prompt.\n")


_LOG_SUMMARY_IMPL = '''
from datetime import datetime, timezone

_EXIT_CODE = {"ok": 0, "partial": 1, "failed": 1, "precondition": 3}


def _parse(ts: str) -> datetime:
    return datetime.fromisoformat(ts.replace("Z", "+00:00"))


def build_summary(tool: str, started_utc: str, finished_utc: str, status: str, counts: dict,
                  errors: list) -> dict:
    duration_s = (_parse(finished_utc) - _parse(started_utc)).total_seconds()
    return {
        "tool": tool,
        "schema_version": 1,
        "started_utc": started_utc,
        "finished_utc": finished_utc,
        "duration_s": round(duration_s, 2),
        "status": status,
        "exit_code": _EXIT_CODE[status],
        "counts": counts,
        "errors": errors,
    }
'''


def _log_summary_reference(workdir: Path) -> None:
    _write(workdir, "logsummary.py", _LOG_SUMMARY_IMPL)


def _log_summary_broken(workdir: Path) -> None:
    # "partial" incorrectly maps to exit_code 0 (same as "ok"), losing the ok/partial distinction.
    _write(workdir, "logsummary.py", _LOG_SUMMARY_IMPL.replace(
        '{"ok": 0, "partial": 1, "failed": 1, "precondition": 3}',
        '{"ok": 0, "partial": 0, "failed": 1, "precondition": 3}'))


def _log_summary_checker(workdir: Path) -> CheckResult:
    path = workdir / "logsummary.py"
    if not path.exists():
        return False, "logsummary.py was not created"
    mod = _load_module(path, "build_summary")
    got = mod.build_summary("prepare_inputs", "2026-09-16T18:04:11Z", "2026-09-16T18:04:13Z", "ok",
                            {"found": 12, "converted": 12}, [])
    expected = {"tool": "prepare_inputs", "schema_version": 1, "started_utc": "2026-09-16T18:04:11Z",
                "finished_utc": "2026-09-16T18:04:13Z", "duration_s": 2.0, "status": "ok",
                "exit_code": 0, "counts": {"found": 12, "converted": 12}, "errors": []}
    if got != expected:
        return False, f"build_summary(...) == {got!r}, expected {expected!r}"
    partial = mod.build_summary("x", "2026-01-01T00:00:00Z", "2026-01-01T00:00:01Z", "partial", {}, ["e"])
    if partial["exit_code"] != 1:
        return False, f"status 'partial' must map to exit_code 1, got {partial['exit_code']!r}"
    precond = mod.build_summary("x", "2026-01-01T00:00:00Z", "2026-01-01T00:00:01Z", "precondition", {}, [])
    if precond["exit_code"] != 3:
        return False, f"status 'precondition' must map to exit_code 3, got {precond['exit_code']!r}"
    return True, "build_summary matched the §0.4 schema and status->exit_code mapping"


# --------------------------------------------------------------------------------------------
# class: test-writing — a correct reference module is provided; write tests for it. The checker
# runs the written test file against the reference (must pass) and against a seeded mutant (must
# fail), so a vacuous test (e.g. `assert True`) is caught.
# --------------------------------------------------------------------------------------------

_RESIZE_GUARD_CORRECT = '''
def check_same_shape(original_shape, output_shape) -> None:
    """Frame invariant (INTERFACES.md §0.5): every generated raster must share (height, width)
    with the original input. Raise ValueError if it does not."""
    if tuple(original_shape[:2]) != tuple(output_shape[:2]):
        raise ValueError(f"shape {output_shape} does not match original {original_shape}")
'''

_RESIZE_GUARD_MUTANT = '''
def check_same_shape(original_shape, output_shape) -> None:
    if original_shape[0] != output_shape[0]:
        raise ValueError(f"shape {output_shape} does not match original {original_shape}")
'''


def _resize_guard_setup(workdir: Path) -> None:
    _write(workdir, "resize_guard.py", _RESIZE_GUARD_CORRECT)
    _write(workdir, "README.md", "Write test_resize_guard.py as instructed in the task prompt.\n")


def _resize_guard_reference(workdir: Path) -> None:
    _write(workdir, "test_resize_guard.py", '''
import pytest
from resize_guard import check_same_shape


def test_same_shape_passes():
    check_same_shape((480, 640), (480, 640))


def test_same_shape_passes_with_extra_channel_dim():
    check_same_shape((480, 640, 3), (480, 640, 1))


def test_different_height_raises():
    with pytest.raises(ValueError):
        check_same_shape((480, 640), (479, 640))


def test_different_width_raises():
    with pytest.raises(ValueError):
        check_same_shape((480, 640), (480, 639))
''')


def _resize_guard_broken(workdir: Path) -> None:
    # A vacuous test suite: it never actually exercises the failure path, so it passes against
    # both the correct implementation and the width-blind mutant alike -- exactly what the
    # self-test's negative check must catch.
    _write(workdir, "test_resize_guard.py", '''
from resize_guard import check_same_shape


def test_same_shape_passes():
    check_same_shape((480, 640), (480, 640))
''')


def _resize_guard_checker(workdir: Path) -> CheckResult:
    if not (workdir / "test_resize_guard.py").exists():
        return False, "test_resize_guard.py was not created"
    ref = _run_pytest(workdir, "test_resize_guard.py")
    if ref.returncode != 0:
        return False, f"tests failed against the correct resize_guard.py:\\n{ref.stdout[-2000:]}"
    (workdir / "resize_guard.py").write_text(_RESIZE_GUARD_MUTANT)
    mut = _run_pytest(workdir, "test_resize_guard.py")
    if mut.returncode == 0:
        return False, "tests still pass against a width-blind mutant resize_guard.py (too weak)"
    return True, "tests pass on the correct module and fail on a seeded mutant"


_SCOPE_CORRECT = '''
import re


def _glob_re(pattern: str):
    i, out = 0, ""
    while i < len(pattern):
        if pattern.startswith("**/", i):
            out += "(?:.*/)?"
            i += 3
            continue
        if pattern.startswith("**", i):
            out += ".*"
            i += 2
            continue
        c = pattern[i]
        out += {"*": "[^/]*", "?": "[^/]"}.get(c, re.escape(c))
        i += 1
    return re.compile("^" + out + "$")


def path_matches(rel_path: str, globs) -> bool:
    return any(_glob_re(g).match(rel_path) for g in (globs or []))
'''

_SCOPE_MUTANT = '''
import fnmatch


def path_matches(rel_path: str, globs) -> bool:
    # Treats a single "*" the same as "**" (fnmatch lets "*" cross "/"), so a one-level glob
    # wrongly matches arbitrarily deep paths.
    return any(fnmatch.fnmatch(rel_path, g) for g in (globs or []))
'''


def _scope_setup(workdir: Path) -> None:
    _write(workdir, "scope.py", _SCOPE_CORRECT)
    _write(workdir, "README.md", "Write test_scope.py as instructed in the task prompt.\n")


def _scope_reference(workdir: Path) -> None:
    _write(workdir, "test_scope.py", '''
from scope import path_matches


def test_exact_match():
    assert path_matches("bench/llm/run_bench.py", ["bench/llm/run_bench.py"])


def test_double_star_matches_nested_path():
    assert path_matches("bench/llm/tasks/a/b.py", ["bench/llm/**"])


def test_single_star_does_not_cross_directories():
    assert not path_matches("bench/llm/sub/run_bench.py", ["bench/*/run_bench.py"])


def test_no_match_outside_globs():
    assert not path_matches("docs/other.md", ["bench/llm/**"])


def test_empty_globs_is_false():
    assert not path_matches("bench/llm/run_bench.py", [])
''')


def _scope_broken(workdir: Path) -> None:
    _write(workdir, "test_scope.py", '''
from scope import path_matches


def test_something():
    assert path_matches("bench/llm/run_bench.py", ["bench/llm/run_bench.py"])
''')


def _scope_checker(workdir: Path) -> CheckResult:
    if not (workdir / "test_scope.py").exists():
        return False, "test_scope.py was not created"
    ref = _run_pytest(workdir, "test_scope.py")
    if ref.returncode != 0:
        return False, f"tests failed against the correct scope.py:\\n{ref.stdout[-2000:]}"
    (workdir / "scope.py").write_text(_SCOPE_MUTANT)
    mut = _run_pytest(workdir, "test_scope.py")
    if mut.returncode == 0:
        return False, "tests still pass against a mutant path_matches lacking real ** support (too weak)"
    return True, "tests pass on the correct module and fail on a seeded mutant"


_REGISTRY_CORRECT = '''
class CaseRegistry:
    def __init__(self):
        self._counts = {}

    def add(self, stem: str) -> str:
        n = self._counts.get(stem, 0) + 1
        self._counts[stem] = n
        return stem if n == 1 else f"{stem}__{n}"
'''

_REGISTRY_MUTANT = '''
class CaseRegistry:
    def __init__(self):
        self._seen = set()

    def add(self, stem: str) -> str:
        if stem in self._seen:
            return f"{stem}__2"
        self._seen.add(stem)
        return stem
'''


def _registry_setup(workdir: Path) -> None:
    _write(workdir, "case_registry.py", _REGISTRY_CORRECT)
    _write(workdir, "README.md", "Write test_case_registry.py as instructed in the task prompt.\n")


def _registry_reference(workdir: Path) -> None:
    _write(workdir, "test_case_registry.py", '''
from case_registry import CaseRegistry


def test_no_collision_for_distinct_stems():
    r = CaseRegistry()
    assert r.add("A") == "A"
    assert r.add("a") == "a"


def test_first_collision_gets_suffix_2():
    r = CaseRegistry()
    assert r.add("a_b") == "a_b"
    assert r.add("a_b") == "a_b__2"


def test_third_collision_gets_suffix_3():
    r = CaseRegistry()
    r.add("x")
    r.add("x")
    assert r.add("x") == "x__3"
''')


def _registry_broken(workdir: Path) -> None:
    _write(workdir, "test_case_registry.py", '''
from case_registry import CaseRegistry


def test_first_collision_gets_suffix_2():
    r = CaseRegistry()
    assert r.add("a_b") == "a_b"
    assert r.add("a_b") == "a_b__2"
''')


def _registry_checker(workdir: Path) -> CheckResult:
    if not (workdir / "test_case_registry.py").exists():
        return False, "test_case_registry.py was not created"
    ref = _run_pytest(workdir, "test_case_registry.py")
    if ref.returncode != 0:
        return False, f"tests failed against the correct case_registry.py:\\n{ref.stdout[-2000:]}"
    (workdir / "case_registry.py").write_text(_REGISTRY_MUTANT)
    mut = _run_pytest(workdir, "test_case_registry.py")
    if mut.returncode == 0:
        return False, "tests still pass against a mutant that never reaches suffix __3 (too weak)"
    return True, "tests pass on the correct module and fail on a seeded mutant"


# --------------------------------------------------------------------------------------------
# class: mechanical-edit — an existing file has a bug or is missing one small piece; the model
# must edit it in place (not rewrite it from scratch) so the provided checks pass.
# --------------------------------------------------------------------------------------------

def _offbyone_setup(workdir: Path) -> None:
    # Bug: uses "<=" so a skeleton with endpoints == branch junctions is miscounted by one.
    _write(workdir, "skeleton_stats.py", '''
def branch_count(skeleton_px: int, endpoints: int) -> int:
    """Number of branches in a thinned skeleton: each endpoint terminates exactly one branch,
    and the skeleton has at least one branch whenever it has any pixels at all."""
    if skeleton_px <= 0:
        return 0
    return endpoints if endpoints <= 1 else endpoints - 1
''')


def _offbyone_reference(workdir: Path) -> None:
    _write(workdir, "skeleton_stats.py", '''
def branch_count(skeleton_px: int, endpoints: int) -> int:
    """Number of branches in a thinned skeleton: each endpoint terminates exactly one branch,
    and the skeleton has at least one branch whenever it has any pixels at all."""
    if skeleton_px <= 0:
        return 0
    return max(endpoints, 1)
''')


def _offbyone_checker(workdir: Path) -> CheckResult:
    path = workdir / "skeleton_stats.py"
    if not path.exists():
        return False, "skeleton_stats.py is missing"
    mod = _load_module(path, "branch_count")
    cases = [(0, 0, 0), (10, 0, 1), (10, 1, 1), (10, 2, 2), (10, 5, 5)]
    for skeleton_px, endpoints, expected in cases:
        got = mod.branch_count(skeleton_px, endpoints)
        if got != expected:
            return False, f"branch_count({skeleton_px}, {endpoints}) == {got}, expected {expected}"
    return True, "branch_count matched the spec for all cases including the boundary case"


def _quiet_flag_setup(workdir: Path) -> None:
    _write(workdir, "report_tool.py", '''
import argparse
import sys


def main(argv=None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--root")
    args = p.parse_args(argv)
    print("starting report_tool")
    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
''')


def _quiet_flag_reference(workdir: Path) -> None:
    _write(workdir, "report_tool.py", '''
import argparse
import sys


def main(argv=None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--root")
    p.add_argument("--quiet", "-q", action="store_true")
    args = p.parse_args(argv)
    if not args.quiet:
        print("starting report_tool")
    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
''')


def _quiet_flag_checker(workdir: Path) -> CheckResult:
    path = workdir / "report_tool.py"
    if not path.exists():
        return False, "report_tool.py is missing"
    loud = _run_py(workdir, "report_tool.py", [])
    if loud.returncode != 0 or "starting report_tool" not in loud.stdout or "done" not in loud.stdout:
        return False, f"without --quiet, expected both lines and exit 0, got {loud.returncode}/{loud.stdout!r}"
    quiet = _run_py(workdir, "report_tool.py", ["--quiet"])
    if quiet.returncode != 0:
        return False, f"--quiet exited {quiet.returncode}, expected 0"
    if "starting report_tool" in quiet.stdout:
        return False, "--quiet did not suppress the info line"
    if "done" not in quiet.stdout:
        return False, "--quiet suppressed output that should remain (e.g. final 'done')"
    short = _run_py(workdir, "report_tool.py", ["-q"])
    if "starting report_tool" in short.stdout:
        return False, "-q (short flag) did not suppress the info line"
    return True, "--quiet/-q suppresses the info line but keeps the final output"


def _rename_setup(workdir: Path) -> None:
    _write(workdir, "core.py", '''
def compute_mask_path(case: str) -> str:
    return f"data/overlays/{case}_mask.png"
''')
    _write(workdir, "pipeline.py", '''
from core import compute_mask_path


def mask_for_case(case: str) -> str:
    return compute_mask_path(case)
''')


def _rename_reference(workdir: Path) -> None:
    _write(workdir, "core.py", '''
def mask_path(case: str) -> str:
    return f"data/overlays/{case}_mask.png"
''')
    _write(workdir, "pipeline.py", '''
from core import mask_path


def mask_for_case(case: str) -> str:
    return mask_path(case)
''')


def _rename_broken(workdir: Path) -> None:
    # Half-finished rename: adds the new name as an alias instead of actually renaming, so the
    # old symbol is still importable -- exactly what the task says not to do.
    _write(workdir, "core.py", '''
def compute_mask_path(case: str) -> str:
    return f"data/overlays/{case}_mask.png"


mask_path = compute_mask_path
''')
    _write(workdir, "pipeline.py", '''
from core import mask_path


def mask_for_case(case: str) -> str:
    return mask_path(case)
''')


def _rename_checker(workdir: Path) -> CheckResult:
    # core.py and pipeline.py import from each other by module name, so (unlike the other
    # checkers) this needs real sys.path-based imports rather than a standalone file load.
    if not (workdir / "core.py").exists() or not (workdir / "pipeline.py").exists():
        return False, "core.py / pipeline.py missing"
    sys.path.insert(0, str(workdir))
    for name in ("core", "pipeline"):
        sys.modules.pop(name, None)
    try:
        core_mod = importlib.import_module("core")
        if hasattr(core_mod, "compute_mask_path"):
            return False, "old symbol compute_mask_path still exists in core.py after the rename"
        if not hasattr(core_mod, "mask_path"):
            return False, "renamed symbol mask_path is missing from core.py"
        pipeline_mod = importlib.import_module("pipeline")
        if pipeline_mod.mask_for_case("Deck_Crack_01") != "data/overlays/Deck_Crack_01_mask.png":
            return False, "pipeline.mask_for_case no longer produces the correct path after the rename"
        return True, "compute_mask_path renamed to mask_path consistently, with no leftover alias"
    finally:
        sys.path.remove(str(workdir))
        for name in ("core", "pipeline"):
            sys.modules.pop(name, None)


def _dataclass_field_setup(workdir: Path) -> None:
    _write(workdir, "result.py", '''
from dataclasses import dataclass


@dataclass
class RunResult:
    outcome: str = "error"
    duration_s: float = 0.0


def to_row(r: RunResult) -> dict:
    return {"outcome": r.outcome, "duration_s": r.duration_s}
''')


def _dataclass_field_reference(workdir: Path) -> None:
    _write(workdir, "result.py", '''
from dataclasses import dataclass


@dataclass
class RunResult:
    outcome: str = "error"
    duration_s: float = 0.0
    peak_vram_mb: float | None = None


def to_row(r: RunResult) -> dict:
    return {"outcome": r.outcome, "duration_s": r.duration_s, "peak_vram_mb": r.peak_vram_mb}
''')


def _dataclass_field_broken(workdir: Path) -> None:
    # Adds the field to the dataclass but forgets to thread it through to_row -- still fails.
    _write(workdir, "result.py", '''
from dataclasses import dataclass


@dataclass
class RunResult:
    outcome: str = "error"
    duration_s: float = 0.0
    peak_vram_mb: float | None = None


def to_row(r: RunResult) -> dict:
    return {"outcome": r.outcome, "duration_s": r.duration_s}
''')


_RW_VALIDATE_CORRECT = '''
import os


def validate_extra_rw(entry: str, home: str, protected) -> bool:
    home_r = os.path.realpath(home)
    p = os.path.realpath(entry)
    if p == home_r or not p.startswith(home_r + os.sep):
        return False
    for q in protected:
        q_r = os.path.realpath(q)
        if p == q_r or p.startswith(q_r + os.sep) or q_r.startswith(p + os.sep):
            return False
    return True
'''

_RW_VALIDATE_MUTANT = '''
import os


def validate_extra_rw(entry: str, home: str, protected) -> bool:
    home_r = os.path.realpath(home)
    p = os.path.realpath(entry)
    return p.startswith(home_r)
'''


def _rw_validate_setup(workdir: Path) -> None:
    _write(workdir, "rw_validate.py", _RW_VALIDATE_CORRECT)
    _write(workdir, "README.md", "Write test_rw_validate.py as instructed in the task prompt.\n")


def _rw_validate_reference(workdir: Path) -> None:
    _write(workdir, "test_rw_validate.py", '''
from rw_validate import validate_extra_rw

HOME = "/home/user"
PROTECTED = ["/home/user/.ssh", "/home/user/.config/claude-auto"]


def test_nested_path_under_home_is_allowed():
    assert validate_extra_rw("/home/user/models/llm", HOME, PROTECTED) is True


def test_home_itself_is_rejected():
    assert validate_extra_rw(HOME, HOME, PROTECTED) is False


def test_path_outside_home_is_rejected():
    assert validate_extra_rw("/etc/passwd", HOME, PROTECTED) is False


def test_path_inside_protected_is_rejected():
    assert validate_extra_rw("/home/user/.ssh/id_rsa", HOME, PROTECTED) is False
''')


def _rw_validate_broken(workdir: Path) -> None:
    _write(workdir, "test_rw_validate.py", '''
from rw_validate import validate_extra_rw

HOME = "/home/user"
PROTECTED = ["/home/user/.ssh", "/home/user/.config/claude-auto"]


def test_nested_path_under_home_is_allowed():
    assert validate_extra_rw("/home/user/models/llm", HOME, PROTECTED) is True
''')


def _rw_validate_checker(workdir: Path) -> CheckResult:
    if not (workdir / "test_rw_validate.py").exists():
        return False, "test_rw_validate.py was not created"
    ref = _run_pytest(workdir, "test_rw_validate.py")
    if ref.returncode != 0:
        return False, f"tests failed against the correct rw_validate.py:\n{ref.stdout[-2000:]}"
    (workdir / "rw_validate.py").write_text(_RW_VALIDATE_MUTANT)
    mut = _run_pytest(workdir, "test_rw_validate.py")
    if mut.returncode == 0:
        return False, "tests still pass against a mutant that allows entry == home and ignores protected (too weak)"
    return True, "tests pass on the correct module and fail on a seeded mutant"


def _dataclass_field_checker(workdir: Path) -> CheckResult:
    path = workdir / "result.py"
    if not path.exists():
        return False, "result.py is missing"
    mod = _load_module(path)
    r = mod.RunResult()
    row = mod.to_row(r)
    if "peak_vram_mb" not in row:
        return False, "to_row() output is missing the new peak_vram_mb field"
    if row["peak_vram_mb"] is not None:
        return False, f"peak_vram_mb default should be None, got {row['peak_vram_mb']!r}"
    r2 = mod.RunResult(outcome="ok", duration_s=1.5, peak_vram_mb=18234.0)
    if mod.to_row(r2)["peak_vram_mb"] != 18234.0:
        return False, "to_row() does not pass an explicit peak_vram_mb value through"
    return True, "RunResult.peak_vram_mb added and threaded through to_row()"


TASKS: list[Task] = [
    Task("script_case_id", "script",
         prompt=("Create a file `case_id.py` defining `derive_case_id(source: str) -> str`.\n"
                 "Given a source filename (possibly with a path prefix), derive a filesystem- and "
                 "URL-safe case id:\n"
                 "1. Take the filename (drop any directory prefix) and drop its extension.\n"
                 "2. Replace every character that is not `[A-Za-z0-9-]` with `_`.\n"
                 "3. Collapse runs of `_` into a single `_` and strip leading/trailing `_`.\n"
                 "4. If the result ends with `_0000`, drop that suffix (and any trailing `_` left behind).\n"
                 "5. If the result is now empty, use `image`.\n"
                 "Example: 'Deck Crack 01.JPG' -> 'Deck_Crack_01'; 'sample_0000.png' -> 'sample'; "
                 "'....png' -> 'image'."),
         setup=_case_id_setup, reference=_case_id_reference, checker=_case_id_checker,
         broken=_case_id_broken),
    Task("script_exit_codes", "script",
         prompt=("Create a file `cli_tool.py`, runnable as `python3 cli_tool.py [args]`, that uses "
                 "argparse to accept `--device` (default `cpu`) plus these always-accepted flags: "
                 "`--root`, `--config`, `--verbose`/`-v`, `--quiet`/`-q`, `--dry-run`, "
                 "`--skip-existing` (all optional, no effect needed beyond being accepted).\n"
                 "If `--device` is not `cpu` or `cuda`, print an error to stderr and exit with code 2 "
                 "(usage/config error). Otherwise print `ok` to stdout and exit with code 0."),
         setup=_exit_codes_setup, reference=_exit_codes_reference, checker=_exit_codes_checker,
         broken=_exit_codes_broken),
    Task("script_atomic_write", "script",
         prompt=("Create a file `atomic_io.py` defining `atomic_write_json(path, obj) -> None` that "
                 "writes `obj` as JSON to `path` atomically: write to a temp file in the same "
                 "directory, then rename/replace it onto the final path (`os.replace`). The final "
                 "path must never be observed in a half-written state, and no temp file must be left "
                 "behind once the function returns. Create the parent directory if needed."),
         setup=_atomic_write_setup, reference=_atomic_write_reference, checker=_atomic_write_checker,
         broken=_atomic_write_broken),
    Task("script_sha256_verify", "script",
         prompt=("Create a file `verify.py` defining `verify_file(path, expected_sha256: str) -> bool`. "
                 "It must stream-hash the file at `path` with SHA-256 and return whether the hex "
                 "digest equals `expected_sha256`. If `path` does not exist, return `False` -- do "
                 "not raise."),
         setup=_sha256_setup, reference=_sha256_reference, checker=_sha256_checker,
         broken=_sha256_broken),
    Task("script_log_summary", "script",
         prompt=("Create a file `logsummary.py` defining `build_summary(tool, started_utc, "
                 "finished_utc, status, counts, errors) -> dict` returning a dict with exactly "
                 "these keys: `tool`, `schema_version` (always `1`), `started_utc`, `finished_utc`, "
                 "`duration_s` (finished minus started, in seconds, from the two ISO-8601 'Z' "
                 "timestamps, rounded to 2 decimals), `status`, `exit_code`, `counts`, `errors`. "
                 "`exit_code` is derived from `status`: `ok` -> 0, `partial` -> 1, `failed` -> 1, "
                 "`precondition` -> 3."),
         setup=_log_summary_setup, reference=_log_summary_reference, checker=_log_summary_checker,
         broken=_log_summary_broken),
    Task("test_resize_guard", "test-writing",
         prompt=("`resize_guard.py` (already present, do not modify it) defines "
                 "`check_same_shape(original_shape, output_shape)`, which must raise `ValueError` "
                 "whenever the (height, width) of `output_shape` differs from `original_shape` "
                 "(extra trailing dims, e.g. channels, are ignored). Write `test_resize_guard.py` "
                 "with pytest tests that call `check_same_shape` and verify: (a) equal shapes do not "
                 "raise, (b) a differing height raises `ValueError`, (c) a differing width raises "
                 "`ValueError`."),
         setup=_resize_guard_setup, reference=_resize_guard_reference, checker=_resize_guard_checker,
         broken=_resize_guard_broken),
    Task("test_scope_path_matches", "test-writing",
         prompt=("`scope.py` (already present, do not modify it) defines `path_matches(rel_path, "
                 "globs) -> bool`, true if `rel_path` matches any glob in `globs`. A single `*` "
                 "matches within one path segment only (never crosses a `/`); `**` matches across "
                 "any number of segments. Write `test_scope.py` with pytest tests that verify: "
                 "(a) an exact-path glob matches, (b) a `**` glob matches a deeply nested path "
                 "under it, (c) a single-`*` glob does NOT match a path with an extra directory "
                 "level in the starred segment, (d) a path outside every glob does not match, "
                 "(e) an empty glob list never matches."),
         setup=_scope_setup, reference=_scope_reference, checker=_scope_checker, broken=_scope_broken),
    Task("test_case_registry", "test-writing",
         prompt=("`case_registry.py` (already present, do not modify it) defines class "
                 "`CaseRegistry` with method `add(stem: str) -> str`: the first time a stem is "
                 "added it is returned unchanged; the 2nd time the same stem is added it returns "
                 "`stem + '__2'`; the 3rd time, `stem + '__3'`; and so on. Write "
                 "`test_case_registry.py` with pytest tests that verify: (a) two different stems "
                 "never collide, (b) the first repeat of a stem gets the `__2` suffix, (c) the "
                 "second repeat of a stem gets the `__3` suffix (not `__2` again)."),
         setup=_registry_setup, reference=_registry_reference, checker=_registry_checker,
         broken=_registry_broken),
    Task("test_rw_scope_validation", "test-writing",
         prompt=("`rw_validate.py` (already present, do not modify it) defines `validate_extra_rw"
                 "(entry: str, home: str, protected: list[str]) -> bool`, which returns `True` only "
                 "if `entry` resolves to a path strictly under `home` and is not equal to, inside, "
                 "or an ancestor of any path in `protected`. Write `test_rw_validate.py` with pytest "
                 "tests that verify: (a) a plain nested path under home and outside `protected` "
                 "returns `True`, (b) `entry == home` returns `False`, (c) a path outside `home` "
                 "entirely returns `False`, (d) a path inside one of the `protected` entries returns "
                 "`False`."),
         setup=_rw_validate_setup, reference=_rw_validate_reference, checker=_rw_validate_checker,
         broken=_rw_validate_broken),
    Task("edit_offbyone_branch_count", "mechanical-edit",
         prompt=("`skeleton_stats.py` has a bug in `branch_count(skeleton_px, endpoints)`: per its "
                 "own docstring, a non-empty skeleton always has at least 1 branch, and branch count "
                 "equals the endpoint count whenever there is at least 1 endpoint. Edit the existing "
                 "file in place (do not rewrite it from scratch, keep the docstring) to fix the bug "
                 "so `branch_count` matches the docstring exactly, including when `endpoints == 1`."),
         setup=_offbyone_setup, reference=_offbyone_reference, checker=_offbyone_checker),
    Task("edit_add_quiet_flag", "mechanical-edit",
         prompt=("`report_tool.py` is missing the `--quiet`/`-q` flag required by this project's CLI "
                 "convention. Edit the file in place to add `--quiet`/`-q` (argparse "
                 "`action='store_true'`) and suppress the `\"starting report_tool\"` info line when "
                 "it is set, while still printing `\"done\"` either way."),
         setup=_quiet_flag_setup, reference=_quiet_flag_reference, checker=_quiet_flag_checker),
    Task("edit_rename_mask_path", "mechanical-edit",
         prompt=("`core.py` defines `compute_mask_path(case)`, used by `pipeline.py`. Rename it to "
                 "`mask_path` (the project's actual naming convention) everywhere it is defined or "
                 "used across both files. Do not leave `compute_mask_path` as a leftover alias -- "
                 "the old name must no longer exist anywhere, and `pipeline.mask_for_case` must still "
                 "return the same result as before."),
         setup=_rename_setup, reference=_rename_reference, checker=_rename_checker,
         broken=_rename_broken),
    Task("edit_add_peak_vram_field", "mechanical-edit",
         prompt=("`result.py` defines a `RunResult` dataclass and a `to_row(r)` helper. Edit the file "
                 "in place to add a new field `peak_vram_mb: float | None = None` to `RunResult`, "
                 "and update `to_row` so its returned dict also includes a `\"peak_vram_mb\"` key "
                 "with that field's value."),
         setup=_dataclass_field_setup, reference=_dataclass_field_reference,
         checker=_dataclass_field_checker, broken=_dataclass_field_broken),
]


assert len({t.id for t in TASKS}) == len(TASKS), "duplicate task ids"
TASK_CLASSES = sorted({t.cls for t in TASKS})

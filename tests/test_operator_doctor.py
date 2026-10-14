"""Unit tests for scripts/operator_doctor.py — stub the three check subprocesses (OPS-01.3)."""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "scripts" / "operator_doctor.py"

_spec = importlib.util.spec_from_file_location("operator_doctor", MODULE_PATH)
operator_doctor = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
sys.modules["operator_doctor"] = operator_doctor
_spec.loader.exec_module(operator_doctor)


def _completed(stdout: str, returncode: int = 0) -> subprocess.CompletedProcess:
    return subprocess.CompletedProcess(args=[], returncode=returncode, stdout=stdout, stderr="")


ENV_PASS = json.dumps({"checks": [{"name": "python_version", "status": "PASS", "value": "3.11.0"}]})
HOST_PASS = json.dumps({"status": "ok", "diffs": [], "isolation": []})
REALSENSE_PASS = json.dumps({"checks": [{"name": "pyrealsense2_import", "status": "PASS", "value": "ok"}]})

ENV_WARN = json.dumps(
    {"checks": [{"name": "ld_library_path_clean", "status": "WARN", "value": "/opt/ros", "hint": "env.sh should filter"}]}
)

HOST_FAIL = json.dumps(
    {
        "status": "drift",
        "diffs": [{"field": "kernel", "expected": "a", "actual": "b", "matches": False, "warn_only": False}],
        "isolation": [],
    }
)

REALSENSE_FAIL = json.dumps(
    {"checks": [{"name": "pyrealsense2_import", "status": "FAIL", "value": "ImportError", "hint": "install it"}]}
)


def _patch_run(monkeypatch, responses: dict[str, subprocess.CompletedProcess]):
    def fake_run(cmd, **kwargs):
        script = Path(cmd[1]).name
        for key, response in responses.items():
            if key in script:
                return response
        raise AssertionError(f"unexpected command: {cmd}")

    monkeypatch.setattr(operator_doctor.subprocess, "run", fake_run)


def test_all_pass(monkeypatch, tmp_path):
    _patch_run(
        monkeypatch,
        {
            "check_env.py": _completed(ENV_PASS),
            "check_host.py": _completed(HOST_PASS),
            "check_realsense.py": _completed(REALSENSE_PASS),
        },
    )
    root = tmp_path  # ros2_ws/install/setup.bash absent -> WARN, not FAIL
    rows = operator_doctor.collect_rows(root, sys.executable)
    statuses = {row.status for row in rows}
    assert "FAIL" not in statuses
    assert any(row.name == "ros2_ws.overlay" and row.status == "WARN" for row in rows)

    exit_code = operator_doctor.EXIT_FAIL if any(r.status == "FAIL" for r in rows) else operator_doctor.EXIT_OK
    assert exit_code == 0


def test_warn_only(monkeypatch, tmp_path):
    _patch_run(
        monkeypatch,
        {
            "check_env.py": _completed(ENV_WARN),
            "check_host.py": _completed(HOST_PASS),
            "check_realsense.py": _completed(REALSENSE_PASS),
        },
    )
    rows = operator_doctor.collect_rows(tmp_path, sys.executable)
    assert not any(row.status == "FAIL" for row in rows)
    assert any(row.status == "WARN" for row in rows)
    exit_code = operator_doctor.EXIT_FAIL if any(r.status == "FAIL" for r in rows) else operator_doctor.EXIT_OK
    assert exit_code == 0


def test_fail_case(monkeypatch, tmp_path):
    _patch_run(
        monkeypatch,
        {
            "check_env.py": _completed(ENV_PASS),
            "check_host.py": _completed(HOST_FAIL),
            "check_realsense.py": _completed(REALSENSE_FAIL),
        },
    )
    rows = operator_doctor.collect_rows(tmp_path, sys.executable)
    assert any(row.status == "FAIL" for row in rows)
    exit_code = operator_doctor.EXIT_FAIL if any(r.status == "FAIL" for r in rows) else operator_doctor.EXIT_OK
    assert exit_code == 3


def test_non_json_output_is_warn_not_crash(monkeypatch, tmp_path):
    _patch_run(
        monkeypatch,
        {
            "check_env.py": _completed("not json at all", returncode=1),
            "check_host.py": _completed(HOST_PASS),
            "check_realsense.py": _completed(REALSENSE_PASS),
        },
    )
    rows = operator_doctor.collect_rows(tmp_path, sys.executable)
    env_rows = [r for r in rows if r.name == "env"]
    assert len(env_rows) == 1
    assert env_rows[0].status == "WARN"


def test_ros_overlay_pass_when_present(tmp_path):
    (tmp_path / "ros2_ws" / "install").mkdir(parents=True)
    (tmp_path / "ros2_ws" / "install" / "setup.bash").write_text("#!/bin/bash\n")
    row = operator_doctor.check_ros_overlay(tmp_path)
    assert row.status == "PASS"


def test_main_exit_codes(monkeypatch, tmp_path):
    _patch_run(
        monkeypatch,
        {
            "check_env.py": _completed(ENV_PASS),
            "check_host.py": _completed(HOST_FAIL),
            "check_realsense.py": _completed(REALSENSE_PASS),
        },
    )
    code = operator_doctor.main(["--json", "--dry-run", "--root", str(tmp_path)])
    assert code == 3

    _patch_run(
        monkeypatch,
        {
            "check_env.py": _completed(ENV_PASS),
            "check_host.py": _completed(HOST_PASS),
            "check_realsense.py": _completed(REALSENSE_PASS),
        },
    )
    code = operator_doctor.main(["--json", "--dry-run", "--root", str(tmp_path)])
    assert code == 0

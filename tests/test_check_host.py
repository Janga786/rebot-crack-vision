"""tests/test_check_host.py — comparison logic for HOST-01, fake inventories only.

`compare_manifest` is pure (no I/O), so it is exercised directly with hand-built manifest/live
dicts. `main()` is exercised too, but with `collect_live_inventory` and `run_isolation_checks`
monkeypatched — this suite never shells out to dpkg, nvidia-smi, ./env.sh or ROS.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

_MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "check_host.py"
_spec = importlib.util.spec_from_file_location("check_host", _MODULE_PATH)
check_host = importlib.util.module_from_spec(_spec)
sys.modules["check_host"] = check_host
_spec.loader.exec_module(check_host)


def _sample_versions() -> dict:
    return {
        "os": {"id": "ubuntu", "version_id": "22.04", "pretty_name": "Ubuntu 22.04.5 LTS"},
        "kernel": "6.8.0-124-generic",
        "nvidia_driver": "580.173.02",
        "cuda_toolkits": ["11.8", "12.6"],
        "ros": {"distro": "humble", "packages": {"ros-humble-desktop": "0.10.0-1jammy"}},
        "gz_sim": "Gazebo Sim, version 8.12.0",
        "crackvision": {"python": "3.11.16", "torch": "2.14.0+cu126", "nnunetv2": "2.8.1"},
        "claude_cli": "2.1.283 (Claude Code)",
        "bwrap": "bubblewrap 0.6.1",
    }


def _manifest(versions: dict) -> dict:
    return {"schema_version": 1, "recorded_utc": "2026-09-28T00:00:00Z", "versions": versions}


# --- compare_manifest (pure) -----------------------------------------------------------------


def test_compare_manifest_identical_is_all_matches():
    versions = _sample_versions()
    diffs = check_host.compare_manifest(_manifest(versions), versions)
    assert diffs
    assert all(d.matches for d in diffs)


def test_compare_manifest_detects_hard_drift():
    recorded = _sample_versions()
    live = _sample_versions()
    live["crackvision"]["torch"] = "2.15.0+cu126"

    diffs = check_host.compare_manifest(_manifest(recorded), live)
    mismatches = {d.field: d for d in diffs if not d.matches}

    assert "crackvision.torch" in mismatches
    assert mismatches["crackvision.torch"].warn_only is False
    assert mismatches["crackvision.torch"].recorded == "2.14.0+cu126"
    assert mismatches["crackvision.torch"].live == "2.15.0+cu126"


def test_compare_manifest_kernel_and_claude_cli_are_warn_only():
    recorded = _sample_versions()
    live = _sample_versions()
    live["kernel"] = "6.9.0-1-generic"
    live["claude_cli"] = "2.2.0 (Claude Code)"

    diffs = check_host.compare_manifest(_manifest(recorded), live)
    mismatches = {d.field: d for d in diffs if not d.matches}

    assert mismatches["kernel"].warn_only is True
    assert mismatches["claude_cli"].warn_only is True


def test_compare_manifest_nested_dict_field_drift():
    recorded = _sample_versions()
    live = _sample_versions()
    live["ros"]["packages"]["ros-humble-desktop"] = "0.11.0-1jammy"

    diffs = check_host.compare_manifest(_manifest(recorded), live)
    mismatches = {d.field: d for d in diffs if not d.matches}

    assert "ros.packages.ros-humble-desktop" in mismatches
    assert mismatches["ros.packages.ros-humble-desktop"].warn_only is False


def test_compare_manifest_missing_field_on_either_side_is_drift():
    recorded = _sample_versions()
    live = _sample_versions()
    del live["gz_sim"]

    diffs = check_host.compare_manifest(_manifest(recorded), live)
    mismatch = next(d for d in diffs if d.field == "gz_sim")

    assert mismatch.matches is False
    assert mismatch.live is None


def test_flatten_leaves_lists_intact():
    flat = check_host._flatten("", {"cuda_toolkits": ["11.8", "12.6"], "kernel": "6.8.0"})
    assert flat["cuda_toolkits"] == ["11.8", "12.6"]
    assert flat["kernel"] == "6.8.0"


# --- main() with collectors monkeypatched (no real system calls) -----------------------------


@pytest.fixture(autouse=True)
def _no_real_isolation_checks(monkeypatch):
    monkeypatch.setattr(
        check_host,
        "run_isolation_checks",
        lambda root: [
            check_host.IsolationResult("crackvision_no_ros_on_syspath", True, "False"),
            check_host.IsolationResult("ros_python_imports_rclpy_moveit", True, "ok"),
            check_host.IsolationResult("no_shared_site_packages", True, "(disjoint)"),
        ],
    )


def test_main_exits_ok_on_match(tmp_path, monkeypatch):
    versions = _sample_versions()
    manifest_path = tmp_path / "host_versions.json"
    manifest_path.write_text(json.dumps(_manifest(versions)))
    monkeypatch.setattr(check_host, "collect_live_inventory", lambda root: versions)

    exit_code = check_host.main(["--manifest", str(manifest_path)])

    assert exit_code == check_host.EXIT_OK


def test_main_exits_runtime_failure_on_drift(tmp_path, monkeypatch):
    recorded = _sample_versions()
    live = _sample_versions()
    live["crackvision"]["torch"] = "2.15.0+cu126"
    manifest_path = tmp_path / "host_versions.json"
    manifest_path.write_text(json.dumps(_manifest(recorded)))
    monkeypatch.setattr(check_host, "collect_live_inventory", lambda root: live)

    exit_code = check_host.main(["--manifest", str(manifest_path)])

    assert exit_code == check_host.EXIT_RUNTIME


def test_main_warn_only_drift_still_exits_ok(tmp_path, monkeypatch):
    recorded = _sample_versions()
    live = _sample_versions()
    live["kernel"] = "6.9.0-1-generic"
    manifest_path = tmp_path / "host_versions.json"
    manifest_path.write_text(json.dumps(_manifest(recorded)))
    monkeypatch.setattr(check_host, "collect_live_inventory", lambda root: live)

    exit_code = check_host.main(["--manifest", str(manifest_path)])

    assert exit_code == check_host.EXIT_OK


def test_main_exits_runtime_failure_on_isolation_failure(tmp_path, monkeypatch):
    versions = _sample_versions()
    manifest_path = tmp_path / "host_versions.json"
    manifest_path.write_text(json.dumps(_manifest(versions)))
    monkeypatch.setattr(check_host, "collect_live_inventory", lambda root: versions)
    monkeypatch.setattr(
        check_host,
        "run_isolation_checks",
        lambda root: [check_host.IsolationResult("no_shared_site_packages", False, "shared: {...}")],
    )

    exit_code = check_host.main(["--manifest", str(manifest_path)])

    assert exit_code == check_host.EXIT_RUNTIME


def test_main_missing_manifest_is_precondition_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(check_host, "collect_live_inventory", lambda root: _sample_versions())

    exit_code = check_host.main(["--manifest", str(tmp_path / "does_not_exist.json")])

    assert exit_code == check_host.EXIT_PRECONDITION


def test_main_malformed_manifest_is_usage_error(tmp_path, monkeypatch):
    manifest_path = tmp_path / "host_versions.json"
    manifest_path.write_text("{not valid json")
    monkeypatch.setattr(check_host, "collect_live_inventory", lambda root: _sample_versions())

    exit_code = check_host.main(["--manifest", str(manifest_path)])

    assert exit_code == check_host.EXIT_USAGE


def test_main_record_writes_manifest_from_live_inventory(tmp_path, monkeypatch):
    versions = _sample_versions()
    manifest_path = tmp_path / "host_versions.json"
    monkeypatch.setattr(check_host, "collect_live_inventory", lambda root: versions)

    exit_code = check_host.main(["--manifest", str(manifest_path), "--record"])

    assert exit_code == check_host.EXIT_OK
    written = json.loads(manifest_path.read_text())
    assert written["versions"] == versions
    assert written["schema_version"] == check_host.SCHEMA_VERSION
    assert "recorded_utc" in written


def test_main_json_flag_emits_valid_json(tmp_path, monkeypatch, capsys):
    versions = _sample_versions()
    manifest_path = tmp_path / "host_versions.json"
    manifest_path.write_text(json.dumps(_manifest(versions)))
    monkeypatch.setattr(check_host, "collect_live_inventory", lambda root: versions)

    exit_code = check_host.main(["--manifest", str(manifest_path), "--json"])
    payload = json.loads(capsys.readouterr().out)

    assert exit_code == check_host.EXIT_OK
    assert payload["status"] == "ok"
    assert isinstance(payload["diffs"], list)
    assert isinstance(payload["isolation"], list)

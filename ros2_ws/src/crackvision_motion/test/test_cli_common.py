"""Unit tests for cli_common (MOT-04.2). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_cli_common.py -q -p no:cacheprovider
"""

import argparse
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from crackvision_motion.cli_common import (  # noqa: E402
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    PreconditionError,
    add_common_args,
    find_root,
    run_cli,
)
from crackvision_motion.reachability_core import ConfigError  # noqa: E402
from crackvision_motion.reachability_map import MapError, PlacementError  # noqa: E402


# --------------------------------------------------------------------------------------
# add_common_args
# --------------------------------------------------------------------------------------

def test_add_common_args_has_exactly_the_spec_0_3_flags():
    parser = argparse.ArgumentParser()
    add_common_args(parser)
    args = parser.parse_args([])
    assert args.root is None
    assert args.config is None
    assert args.verbose is False
    assert args.quiet is False
    assert args.dry_run is False
    assert args.skip_existing is False


def test_add_common_args_short_flags_and_dry_run_skip_existing():
    parser = argparse.ArgumentParser()
    add_common_args(parser)
    args = parser.parse_args(["-v", "-q", "--dry-run", "--skip-existing", "--root", "/tmp/x", "--config", "/tmp/c.yaml"])
    assert args.verbose is True
    assert args.quiet is True
    assert args.dry_run is True
    assert args.skip_existing is True
    assert str(args.root) == "/tmp/x"
    assert str(args.config) == "/tmp/c.yaml"


# --------------------------------------------------------------------------------------
# find_root
# --------------------------------------------------------------------------------------

def _make_root(tmp_path):
    (tmp_path / "pyproject.toml").write_text("[project]\n")
    docs = tmp_path / "docs"
    docs.mkdir()
    (docs / "INTERFACES.md").write_text("# interfaces\n")
    nested = tmp_path / "a" / "b"
    nested.mkdir(parents=True)
    return nested


def test_find_root_walks_up_to_pyproject_and_interfaces(tmp_path, monkeypatch):
    monkeypatch.delenv("CRACKVISION_ROOT", raising=False)
    nested = _make_root(tmp_path)
    found = find_root(start=nested)
    assert found == tmp_path.resolve()


def test_find_root_env_var_override(tmp_path, monkeypatch):
    _make_root(tmp_path)
    monkeypatch.setenv("CRACKVISION_ROOT", str(tmp_path))
    found = find_root()
    assert found == tmp_path.resolve()


def test_find_root_env_var_bad_raises(tmp_path, monkeypatch):
    empty = tmp_path / "empty"
    empty.mkdir()
    monkeypatch.setenv("CRACKVISION_ROOT", str(empty))
    with pytest.raises(PreconditionError):
        find_root()


def test_find_root_no_ancestor_raises(tmp_path, monkeypatch):
    monkeypatch.delenv("CRACKVISION_ROOT", raising=False)
    lonely = tmp_path / "lonely"
    lonely.mkdir()
    with pytest.raises(PreconditionError):
        find_root(start=lonely)


# --------------------------------------------------------------------------------------
# run_cli exit-code mapping
# --------------------------------------------------------------------------------------

def _read_latest_summary(tmp_path, tool):
    return json.loads((tmp_path / "logs" / f"{tool}_latest.json").read_text(encoding="utf-8"))


def test_run_cli_ok(tmp_path):
    def main_fn(args, log, summary):
        summary.increment("did_a_thing")
        return None

    code = run_cli("t_ok", main_fn, ["--root", str(tmp_path)])
    assert code == EXIT_OK
    summary = _read_latest_summary(tmp_path, "t_ok")
    assert summary["status"] == "ok"
    assert summary["exit_code"] == EXIT_OK
    assert summary["counts"] == {"did_a_thing": 1}


def test_run_cli_returns_partial_status(tmp_path):
    def main_fn(args, log, summary):
        return {"status": "partial"}

    code = run_cli("t_partial", main_fn, ["--root", str(tmp_path)])
    assert code == EXIT_RUNTIME
    assert _read_latest_summary(tmp_path, "t_partial")["status"] == "partial"


def test_run_cli_config_error_maps_to_usage_exit(tmp_path):
    def main_fn(args, log, summary):
        raise ConfigError("bad config")

    code = run_cli("t_cfgerr", main_fn, ["--root", str(tmp_path)])
    assert code == EXIT_USAGE
    summary = _read_latest_summary(tmp_path, "t_cfgerr")
    assert summary["status"] == "failed"
    assert summary["errors"] == ["bad config"]


def test_run_cli_map_error_maps_to_usage_exit(tmp_path):
    def main_fn(args, log, summary):
        raise MapError("bad map")

    code = run_cli("t_maperr", main_fn, ["--root", str(tmp_path)])
    assert code == EXIT_USAGE


def test_run_cli_placement_error_maps_to_usage_exit(tmp_path):
    def main_fn(args, log, summary):
        raise PlacementError("bad placement")

    code = run_cli("t_placerr", main_fn, ["--root", str(tmp_path)])
    assert code == EXIT_USAGE


def test_run_cli_precondition_error_maps_to_precondition_exit(tmp_path):
    def main_fn(args, log, summary):
        raise PreconditionError("services not up")

    code = run_cli("t_precond", main_fn, ["--root", str(tmp_path)])
    assert code == EXIT_PRECONDITION
    assert _read_latest_summary(tmp_path, "t_precond")["status"] == "precondition"


def test_run_cli_unhandled_exception_maps_to_runtime_exit(tmp_path):
    def main_fn(args, log, summary):
        raise RuntimeError("boom")

    code = run_cli("t_boom", main_fn, ["--root", str(tmp_path)])
    assert code == EXIT_RUNTIME
    summary = _read_latest_summary(tmp_path, "t_boom")
    assert summary["status"] == "failed"
    assert summary["errors"] == ["boom"]


def test_run_cli_extra_args_hook(tmp_path):
    seen = {}

    def main_fn(args, log, summary):
        seen["value"] = args.widget

    def extra_args(parser):
        parser.add_argument("--widget", default=None)

    code = run_cli("t_extra", main_fn, ["--root", str(tmp_path), "--widget", "gizmo"], extra_args=extra_args)
    assert code == EXIT_OK
    assert seen["value"] == "gizmo"


# --------------------------------------------------------------------------------------
# --dry-run behaviour
# --------------------------------------------------------------------------------------

def test_run_cli_dry_run_never_calls_main_fn(tmp_path):
    called = {"yes": False}

    def main_fn(args, log, summary):
        called["yes"] = True
        (tmp_path / "should_not_exist.txt").write_text("oops")

    code = run_cli("t_dry", main_fn, ["--root", str(tmp_path), "--dry-run"])
    assert code == EXIT_OK
    assert called["yes"] is False
    assert not (tmp_path / "should_not_exist.txt").exists()


def test_run_cli_dry_run_still_writes_logs(tmp_path):
    def main_fn(args, log, summary):
        raise AssertionError("must not be called in dry-run")

    run_cli("t_dry_logs", main_fn, ["--root", str(tmp_path), "--dry-run"])
    log_dir = tmp_path / "logs"
    assert any(p.suffix == ".log" for p in log_dir.iterdir())
    summary = _read_latest_summary(tmp_path, "t_dry_logs")
    assert summary["status"] == "ok"
    assert summary["exit_code"] == EXIT_OK


def test_run_cli_dry_run_only_produces_log_files(tmp_path):
    def main_fn(args, log, summary):
        raise AssertionError("must not be called in dry-run")

    run_cli("t_dry_only", main_fn, ["--root", str(tmp_path), "--dry-run"])
    written = {p for p in tmp_path.rglob("*") if p.is_file()}
    log_dir = (tmp_path / "logs").resolve()
    assert all(p.resolve().parent == log_dir for p in written)

"""tests/test_approve_trajectory.py — MOT-08.4 crackvision.approve_trajectory CLI
(docs/INTERFACES.md §18.2).

Builds a synthetic `crackvision.trajectory_preview/1` manifest plus fake PNG artefacts directly
(via `preview_core`'s own schema shape, not the renderer) and drives `approve_trajectory.main([...])`
in-process, injecting the `/dev/tty` confirmation through a monkeypatched `make_tty_reader`/
`_open_tty` rather than through stdin.
"""

from __future__ import annotations

import importlib.util
import json
import os
import pty
import sys
import types
from pathlib import Path

import pytest

from crackvision import approve_trajectory as mod
from crackvision.preview_core import file_sha256, validate_manifest

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]


def _per_joint():
    return [
        {
            "joint": name, "min_q": -0.1, "max_q": 0.1, "lower": -10.0, "upper": 10.0,
            "min_headroom_rad": 9.9, "max_abs_v": 0.1, "v_limit": 100.0, "v_ratio": 0.001,
            "max_abs_a": 0.1, "a_limit": 100.0, "a_ratio": 0.001,
        }
        for name in JOINT_NAMES
    ]


def _summary():
    return {
        "n_points": 5,
        "duration_s": 4.0,
        "duration_scaled_s": 40.0,
        "per_joint": _per_joint(),
        "tool": {
            "path_length_m": 0.1,
            "max_speed_m_s": 0.02,
            "max_speed_scaled_m_s": 0.002,
            "min_z_m": 0.0,
            "legs": [
                {"leg_index": 0, "kind": "transit", "point_range": [0, 1], "duration_s": 1.0, "length_m": 0.05, "mean_speed_m_s": 0.05},
                {"leg_index": 1, "kind": "trace", "point_range": [1, 4], "duration_s": 3.0, "length_m": 0.05, "mean_speed_m_s": 0.0166},
            ],
        },
    }


@pytest.fixture
def case(tmp_path):
    root = tmp_path

    limits_path = root / "config" / "robot" / "b601_dm_limits.yaml"
    limits_path.parent.mkdir(parents=True)
    limits_path.write_text("limits-v1\n", encoding="utf-8")

    scene_path = root / "config" / "scene" / "scene.yaml"
    scene_path.parent.mkdir(parents=True)
    scene_path.write_text("scene-v1\n", encoding="utf-8")

    ee_path = root / "config" / "robot" / "end_effector.yaml"
    ee_path.write_text("end-effector-v1\n", encoding="utf-8")

    data_dir = root / "data" / "trajectories"
    data_dir.mkdir(parents=True)
    traj_path = data_dir / "case1_trajectory.json"
    traj_path.write_text(json.dumps({"schema": "crackvision.joint_trajectory/1", "purpose": "crack_task"}), encoding="utf-8")
    traj_sha = file_sha256(traj_path)
    sha8 = traj_sha[:8]

    preview_dir = root / "data" / "previews" / f"case1_{sha8}"
    preview_dir.mkdir(parents=True)

    joints_path = preview_dir / "joints.png"
    toolpath_path = preview_dir / "toolpath.png"
    toolspeed_path = preview_dir / "toolspeed.png"
    joints_path.write_bytes(b"PNG-joints-v1")
    toolpath_path.write_bytes(b"PNG-toolpath-v1")
    toolspeed_path.write_bytes(b"PNG-toolspeed-v1")

    check_path = data_dir / "case1_trajectory.json.check.json"
    check_doc = {
        "schema": "crackvision.trajectory_check/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "trajectory_sha256": traj_sha,
        "checks": [{"check": "C-SCHEMA", "outcome": "pass", "detail": "ok", "numbers": {}}],
    }
    check_path.write_text(json.dumps(check_doc), encoding="utf-8")

    manifest = {
        "schema": "crackvision.trajectory_preview/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "trajectory_preview",
        "case_id": "case1",
        "trajectory": {"path": "data/trajectories/case1_trajectory.json", "sha256": traj_sha, "purpose": "crack_task"},
        "plan": {"path": "data/plans/case1_plan.json", "sha256": "a" * 64, "schema": "crackvision.cartesian_plan/1"},
        "retime_report": {"path": "data/trajectories/case1_trajectory.json.retime.json", "sha256": "b" * 64},
        "check_report": {
            "path": "data/trajectories/case1_trajectory.json.check.json",
            "sha256": file_sha256(check_path),
            "all_pass": True,
            "failed_checks": [],
        },
        "bindings": {
            "limits_file_sha256": file_sha256(limits_path),
            "end_effector_config_sha256": file_sha256(ee_path),
            "scene_config_sha256": file_sha256(scene_path),
            "paths3d_sha256": None,
        },
        "preview_speed_scale": 0.1,
        "summary": _summary(),
        "artifacts": [
            {"kind": "joints", "path": f"data/previews/case1_{sha8}/joints.png", "sha256": file_sha256(joints_path)},
            {"kind": "toolpath", "path": f"data/previews/case1_{sha8}/toolpath.png", "sha256": file_sha256(toolpath_path)},
            {"kind": "toolspeed", "path": f"data/previews/case1_{sha8}/toolspeed.png", "sha256": file_sha256(toolspeed_path)},
        ],
        "warnings": [],
    }
    assert validate_manifest(manifest) == []

    manifest_path = preview_dir / "preview.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    return {
        "root": root,
        "manifest": manifest_path,
        "traj": traj_path,
        "check": check_path,
        "limits": limits_path,
        "scene": scene_path,
        "end_effector": ee_path,
        "joints": joints_path,
        "toolpath": toolpath_path,
        "toolspeed": toolspeed_path,
        "sha8": sha8,
        "traj_sha": traj_sha,
    }


def _argv(case, *, preview=None, operator="op1", out=None, dry_run=False):
    argv = [
        "--root", str(case["root"]),
        "--preview", str(preview or case["manifest"]),
        "--operator", operator,
        "--limits", str(case["limits"]),
        "--scene-config", str(case["scene"]),
        "--end-effector-config", str(case["end_effector"]),
    ]
    if out is not None:
        argv += ["--out", str(out)]
    if dry_run:
        argv.append("--dry-run")
    return argv


def _default_out_path(case):
    return case["root"] / "data" / "approvals" / f"case1_{case['sha8']}_approval.json"


def _inject_approving_reader(monkeypatch, case):
    sha8 = case["sha8"]
    monkeypatch.setattr(mod, "make_tty_reader", lambda **kwargs: (lambda: f"APPROVE {sha8}\n"))


def test_happy_path_writes_approval(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)

    rc = mod.main(_argv(case))
    assert rc == 0

    out_path = _default_out_path(case)
    assert out_path.is_file()

    approval = json.loads(out_path.read_text(encoding="utf-8"))
    assert set(approval) == {"schema", "trajectory_sha256", "approved_by", "approved_utc", "preview_artifacts"}
    assert approval["schema"] == "crackvision.execution_approval/1"
    assert approval["trajectory_sha256"] == case["traj_sha"]
    assert approval["approved_by"] == "op1"
    assert isinstance(approval["approved_utc"], str) and approval["approved_utc"]

    artifacts = approval["preview_artifacts"]
    assert len(artifacts) == 4
    assert artifacts[0] == {
        "path": str(case["manifest"].relative_to(case["root"]).as_posix()),
        "sha256": file_sha256(case["manifest"]),
    }
    manifest = json.loads(case["manifest"].read_text(encoding="utf-8"))
    assert artifacts[1:] == [{"path": a["path"], "sha256": a["sha256"]} for a in manifest["artifacts"]]
    for art in artifacts:
        assert set(art) == {"path", "sha256"}


def test_tampered_trajectory_refuses(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)
    case["traj"].write_text(json.dumps({"schema": "crackvision.joint_trajectory/1", "purpose": "crack_task", "x": 1}), encoding="utf-8")

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()


def test_tampered_png_refuses(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)
    case["joints"].write_bytes(b"TAMPERED")

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()


def test_tampered_check_report_sha_refuses(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)
    case["check"].write_text(
        json.dumps(
            {
                "schema": "crackvision.trajectory_check/1",
                "created_utc": "2026-01-01T00:00:00Z",
                "trajectory_sha256": case["traj_sha"],
                "checks": [{"check": "C-SCHEMA", "outcome": "pass", "detail": "tampered", "numbers": {}}],
            }
        ),
        encoding="utf-8",
    )

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()


def test_failing_check_report_refuses(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)
    check_doc = {
        "schema": "crackvision.trajectory_check/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "trajectory_sha256": case["traj_sha"],
        "checks": [{"check": "C-LIMITS", "outcome": "fail", "detail": "exceeded", "numbers": {}}],
    }
    case["check"].write_text(json.dumps(check_doc), encoding="utf-8")
    manifest = json.loads(case["manifest"].read_text(encoding="utf-8"))
    manifest["check_report"]["sha256"] = file_sha256(case["check"])
    manifest["check_report"]["all_pass"] = False
    manifest["check_report"]["failed_checks"] = ["C-LIMITS"]
    case["manifest"].write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()


def _resync_check_report_sha(case):
    manifest = json.loads(case["manifest"].read_text(encoding="utf-8"))
    manifest["check_report"]["sha256"] = file_sha256(case["check"])
    case["manifest"].write_text(json.dumps(manifest, indent=2), encoding="utf-8")


def _latest_run_errors(case):
    log_path = case["root"] / "logs" / "approve_trajectory_latest.json"
    summary = json.loads(log_path.read_text(encoding="utf-8"))
    return summary["errors"]


def test_check_report_json_array_refuses(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)
    case["check"].write_text(json.dumps([{"check": "C-SCHEMA", "outcome": "pass"}]), encoding="utf-8")
    _resync_check_report_sha(case)

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()
    assert any("JSON object" in e for e in _latest_run_errors(case))


def test_check_report_non_utf8_refuses(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)
    case["check"].write_bytes(b"\xff\xfe\x00\x01")
    _resync_check_report_sha(case)

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()
    assert any("not valid UTF-8" in e for e in _latest_run_errors(case))


def test_check_report_failing_entry_without_check_key_refuses(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)
    check_doc = {
        "schema": "crackvision.trajectory_check/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "trajectory_sha256": case["traj_sha"],
        "checks": [{"outcome": "fail", "detail": "exceeded", "numbers": {}}],
    }
    case["check"].write_text(json.dumps(check_doc), encoding="utf-8")
    manifest = json.loads(case["manifest"].read_text(encoding="utf-8"))
    manifest["check_report"]["sha256"] = file_sha256(case["check"])
    manifest["check_report"]["all_pass"] = False
    case["manifest"].write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()


def test_check_report_null_refuses(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)
    case["check"].write_bytes(b"null")
    _resync_check_report_sha(case)

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()
    assert any("must be a JSON object" in e for e in _latest_run_errors(case))


def test_check_report_checks_not_a_list_refuses(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)
    check_doc = {
        "schema": "crackvision.trajectory_check/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "trajectory_sha256": case["traj_sha"],
        "checks": 5,
    }
    case["check"].write_text(json.dumps(check_doc), encoding="utf-8")
    _resync_check_report_sha(case)

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()


@pytest.mark.parametrize("bad_name", [None, 5])
def test_check_report_failing_entry_non_string_check_refuses(case, monkeypatch, bad_name):
    _inject_approving_reader(monkeypatch, case)
    check_doc = {
        "schema": "crackvision.trajectory_check/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "trajectory_sha256": case["traj_sha"],
        "checks": [{"check": bad_name, "outcome": "fail", "detail": "exceeded", "numbers": {}}],
    }
    case["check"].write_text(json.dumps(check_doc), encoding="utf-8")
    _resync_check_report_sha(case)

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()


def test_purpose_test_refuses(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)
    manifest = json.loads(case["manifest"].read_text(encoding="utf-8"))
    manifest["trajectory"]["purpose"] = "test"
    case["manifest"].write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()


@pytest.mark.parametrize("config_key", ["limits", "scene", "end_effector"])
def test_stale_binding_refuses(case, monkeypatch, config_key):
    _inject_approving_reader(monkeypatch, case)
    case[config_key].write_text("changed-content\n", encoding="utf-8")

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()


def test_null_binding_is_allowed_with_warning(case, monkeypatch, caplog):
    _inject_approving_reader(monkeypatch, case)
    manifest = json.loads(case["manifest"].read_text(encoding="utf-8"))
    manifest["bindings"]["limits_file_sha256"] = None
    case["manifest"].write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    rc = mod.main(_argv(case))
    assert rc == 0
    assert _default_out_path(case).is_file()


def test_wrong_phrase_refuses(case, monkeypatch):
    monkeypatch.setattr(mod, "make_tty_reader", lambda **kwargs: (lambda: "nope\n"))

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()


def test_no_tty_refuses(case, monkeypatch):
    def _raise():
        raise OSError("no controlling terminal")

    monkeypatch.setattr(mod, "_open_tty", _raise)

    rc = mod.main(_argv(case))
    assert rc == 3
    assert not _default_out_path(case).is_file()


def test_existing_output_refuses(case, monkeypatch):
    _inject_approving_reader(monkeypatch, case)
    out_path = _default_out_path(case)
    out_path.parent.mkdir(parents=True)
    out_path.write_text("already-here", encoding="utf-8")

    rc = mod.main(_argv(case))
    assert rc == 3
    assert out_path.read_text(encoding="utf-8") == "already-here"


def test_dry_run_happy_path_writes_nothing_and_exits_0(case, monkeypatch):
    def _should_not_be_called(**kwargs):
        raise AssertionError("make_tty_reader must not be called in --dry-run")

    monkeypatch.setattr(mod, "make_tty_reader", _should_not_be_called)

    rc = mod.main(_argv(case, dry_run=True))
    assert rc == 0
    assert not _default_out_path(case).is_file()


def test_dry_run_refusal_still_exits_0(case, monkeypatch):
    def _should_not_be_called(**kwargs):
        raise AssertionError("make_tty_reader must not be called in --dry-run")

    monkeypatch.setattr(mod, "make_tty_reader", _should_not_be_called)
    manifest = json.loads(case["manifest"].read_text(encoding="utf-8"))
    manifest["trajectory"]["purpose"] = "test"
    case["manifest"].write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    rc = mod.main(_argv(case, dry_run=True))
    assert rc == 0
    assert not _default_out_path(case).is_file()


def test_missing_preview_refuses(case, monkeypatch):
    monkeypatch.setattr(mod, "make_tty_reader", lambda **kwargs: (lambda: "irrelevant\n"))

    rc = mod.main(_argv(case, preview=case["root"] / "does_not_exist.json"))
    assert rc == 3


def test_empty_operator_is_usage_error(case):
    argv = _argv(case, operator="   ")
    rc = mod.main(argv)
    assert rc == 2


def test_confirm_matches_exact_phrase():
    assert mod.confirm("deadbeef", lambda: "APPROVE deadbeef\n") is True
    assert mod.confirm("deadbeef", lambda: "approve deadbeef\n") is False
    assert mod.confirm("deadbeef", lambda: "APPROVE deadbeef") is True


def test_confirm_no_tty_returns_false():
    def _raise():
        raise OSError("no tty")

    assert mod.confirm("deadbeef", _raise) is False


# ---------------------------------------------------------------------------
# load_approval compatibility (ros2_ws execution_gate.py, loaded without colcon or rclpy)
# ---------------------------------------------------------------------------


def _load_module_from_path(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _load_execution_gate(repo_root: Path):
    """Loads `crackvision_motion/execution_gate.py` (and its own ROS-free dependencies:
    `joint_trajectory.py`, `cartesian_plan.py`, `scene_core.py`, `crackvision_description/
    end_effector.py`) directly from their source paths, under fake `crackvision_motion`/
    `crackvision_description` packages, without a colcon build and without importing the
    package normally (`crackvision_motion` would otherwise shadow nothing here, but this keeps
    the conda interpreter from ever needing the ROS overlay on `sys.path`)."""
    motion_dir = repo_root / "ros2_ws" / "src" / "crackvision_motion" / "crackvision_motion"
    desc_dir = repo_root / "ros2_ws" / "src" / "crackvision_description" / "crackvision_description"

    pkg_motion = types.ModuleType("crackvision_motion")
    pkg_motion.__path__ = []
    sys.modules["crackvision_motion"] = pkg_motion
    pkg_desc = types.ModuleType("crackvision_description")
    pkg_desc.__path__ = []
    sys.modules["crackvision_description"] = pkg_desc

    _load_module_from_path("crackvision_motion.joint_trajectory", motion_dir / "joint_trajectory.py")
    _load_module_from_path("crackvision_motion.scene_core", motion_dir / "scene_core.py")
    _load_module_from_path("crackvision_motion.cartesian_plan", motion_dir / "cartesian_plan.py")
    _load_module_from_path("crackvision_description.end_effector", desc_dir / "end_effector.py")
    return _load_module_from_path("crackvision_motion.execution_gate", motion_dir / "execution_gate.py")


# ---------------------------------------------------------------------------
# real-pty regression test for make_tty_reader / confirm (bug: text-mode 'r+' on /dev/tty
# raised io.UnsupportedOperation because tty/pty devices are not seekable)
# ---------------------------------------------------------------------------


def _run_confirm_in_pty(sha8: str, typed_line: str) -> int:
    """Forks a child whose controlling terminal is a real pty slave, runs
    make_tty_reader/confirm there with no monkeypatching, and returns the child's exit status
    (0 = confirm() was True, 1 = confirm() was False)."""
    module_path = Path(__file__).resolve().parents[1] / "src" / "crackvision" / "approve_trajectory.py"

    pid, master_fd = pty.fork()
    if pid == 0:
        try:
            child_mod = _load_module_from_path("crackvision_approve_trajectory_ptychild", module_path)
            reader = child_mod.make_tty_reader(
                trajectory_path="data/trajectories/case1_trajectory.json",
                purpose="crack_task",
                duration_scaled_s=40.0,
                manifest_path="data/previews/case1_deadbeef/preview.json",
            )
            ok = child_mod.confirm(sha8, reader)
        except Exception:
            os._exit(2)
        os._exit(0 if ok else 1)

    try:
        prompt = b""
        deadline = 5.0
        import select
        import time

        start = time.monotonic()
        while b"confirmation phrase" not in prompt and time.monotonic() - start < deadline:
            ready, _, _ = select.select([master_fd], [], [], deadline - (time.monotonic() - start))
            if not ready:
                break
            try:
                prompt += os.read(master_fd, 4096)
            except OSError:
                break

        os.write(master_fd, (typed_line + "\n").encode("utf-8"))

        _, status = os.waitpid(pid, 0)
        return os.waitstatus_to_exitcode(status)
    finally:
        os.close(master_fd)


def test_make_tty_reader_confirm_on_real_pty_approves():
    rc = _run_confirm_in_pty("deadbeef", "APPROVE deadbeef")
    assert rc == 0


def test_make_tty_reader_confirm_on_real_pty_wrong_phrase_refuses():
    rc = _run_confirm_in_pty("deadbeef", "nope")
    assert rc == 1


def test_approval_file_loads_via_execution_gate(case, monkeypatch, tmp_path):
    _inject_approving_reader(monkeypatch, case)
    assert mod.main(_argv(case)) == 0

    repo_root = Path(__file__).resolve().parents[1]
    gate_path = repo_root / "ros2_ws" / "src" / "crackvision_motion" / "crackvision_motion" / "execution_gate.py"
    if not gate_path.is_file():
        pytest.skip("execution_gate.py not present in this checkout")

    execution_gate = _load_execution_gate(repo_root)
    try:
        loaded = execution_gate.load_approval(_default_out_path(case))
    finally:
        for name in (
            "crackvision_motion", "crackvision_motion.joint_trajectory", "crackvision_motion.scene_core",
            "crackvision_motion.cartesian_plan", "crackvision_motion.execution_gate",
            "crackvision_description", "crackvision_description.end_effector",
        ):
            sys.modules.pop(name, None)

    assert loaded["trajectory_sha256"] == case["traj_sha"]
    assert loaded["approved_by"] == "op1"

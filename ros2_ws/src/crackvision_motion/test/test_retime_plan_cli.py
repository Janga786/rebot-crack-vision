"""Unit tests for retime_plan (MOT-07.5): the §16.6 ROS-free console-script CLI wrapping
`retime.retime_plan_doc`/`write_outputs`.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_retime_plan_cli.py -q -p no:cacheprovider

Every test calls `retime_plan.main(argv)` directly (never a subprocess) and passes `--root
tmp_path` plus explicit `--limits`/`--end-effector-config`/`--urdf`/`--retime-config`/
`--execution-config` flags pointing at tmp copies of the real production files (so `--root` never
needs to look like a real project root) -- mirroring `test_retime.py`'s own synthetic-plan
construction via `cartesian_plan`'s incremental builders.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any, Dict, List

import pytest
import yaml

from crackvision_motion.cartesian_plan import add_leg, add_segment_report, build_caveats, new_plan, validate_plan
from crackvision_motion.cli_common import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE
from crackvision_motion.joint_trajectory import JOINT_NAMES, file_sha256, load_limits, load_trajectory
from crackvision_motion.retime import validate_retime_report
from crackvision_motion.retime_plan import main

REPO_ROOT = Path(__file__).resolve().parents[4]
LIMITS_PATH = REPO_ROOT / "config" / "robot" / "b601_dm_limits.yaml"
END_EFFECTOR_PATH = REPO_ROOT / "config" / "robot" / "end_effector.yaml"
URDF_PATH = REPO_ROOT / "presentation" / "sim" / "reBot_B601_DM_with_gripper.urdf"

LIMITS = load_limits(LIMITS_PATH)
SEG_REF = {
    "component_id": "c0", "polyline_kind": "main", "branch_index": None, "segment_index": 0, "sub_segment": None,
}

Q_HOME = [0.0] * 6
Q_APPROACH_START = [0.0, -0.9, -0.6, 0.3, 0.3, 0.0]
Q_TRACE_FIRST = [0.0, -1.0, -0.7, 0.35, 0.3, 0.0]
Q_END_CURVED = [0.03, -1.0, -0.65, 0.4, 0.3, 0.0]
Q_RETRACT_END = Q_APPROACH_START
TRACE = [Q_TRACE_FIRST, Q_END_CURVED]


def _recheck(positions: List[List[float]]) -> Dict[str, Any]:
    arr = [positions[i + 1][j] - positions[i][j] for i in range(len(positions) - 1) for j in range(6)]
    step = max(abs(x) for x in arr) if arr else 0.0
    return {"max_pos_err_m": 0.0, "max_ang_err_rad": 0.0, "all_valid": True, "within_limits": True, "max_joint_step_rad": step}


def _copy_in(tmp_path: Path, src: Path, name: str) -> Path:
    dst = tmp_path / name
    shutil.copyfile(src, dst)
    return dst


def _build_plan(tmp_path: Path, limits_path: Path, end_effector_path: Path, *, label: str = "case") -> Path:
    paths3d = tmp_path / "paths3d.json"
    paths3d.write_text(json.dumps({"dummy": "paths3d"}))
    scene = tmp_path / "scene.yaml"
    scene.write_text("dummy: scene\n")
    config = tmp_path / "config.yaml"
    config.write_text("dummy: config\n")
    sources = {
        "paths3d": {"path": str(paths3d), "sha256": file_sha256(paths3d), "execution_eligible": True},
        "scene": {"path": str(scene), "sha256": file_sha256(scene)},
        "end_effector": {"path": str(end_effector_path), "sha256": file_sha256(end_effector_path)},
        "limits": {"path": str(limits_path), "sha256": file_sha256(limits_path)},
        "config": {"path": str(config), "sha256": file_sha256(config)},
    }
    plan = new_plan(
        label, sources, {"note": "synthetic MOT-07.5 fixture"},
        {"joint_names": list(JOINT_NAMES), "positions_rad": Q_HOME, "source": "start_joints_override"},
        {"mode": "none", "positions_rad": None},
    )
    add_leg(plan, "transit", SEG_REF, "forward", 0.0, [Q_HOME, Q_APPROACH_START], [None, None], _recheck([Q_HOME, Q_APPROACH_START]))
    add_leg(plan, "approach", SEG_REF, "forward", 0.0, [Q_APPROACH_START, Q_TRACE_FIRST], [None, 0], _recheck([Q_APPROACH_START, Q_TRACE_FIRST]))
    add_leg(plan, "trace", SEG_REF, "forward", 0.0, TRACE, [0, 1], _recheck(TRACE))
    add_leg(plan, "retract", SEG_REF, "forward", 0.0, [Q_END_CURVED, Q_RETRACT_END], [0, None], _recheck([Q_END_CURVED, Q_RETRACT_END]))
    add_segment_report(plan, SEG_REF, [], 1, {"direction": "forward", "roll_offset_rad": 0.0}, 2, 2, 0.03, 0.03)
    plan["coverage"] = {
        "trace_waypoints_total": 2, "trace_waypoints_planned": 2,
        "length_m_total": 0.03, "length_m_planned": 0.03,
        "fraction_by_length": 1.0, "complete": True,
        "upstream": {"points": 2, "valid": 2, "invalid_by_reason": {}},
    }
    plan["caveats"] = build_caveats("tool_tip; synthetic test fixture (MOT-07.5)", execution_eligible=True)
    plan = validate_plan(plan, load_limits(limits_path))

    plan_path = tmp_path / f"plan_{label}.json"
    plan_path.write_text(json.dumps(plan))
    return plan_path


def _write_retime_config(tmp_path: Path, *, name: str = "retime.yaml", extra_key: bool = False) -> Path:
    raw = {
        "schema": "crackvision.retime_config/1",
        "limit_fraction": {"velocity": 0.95, "acceleration": 0.95},
        "sampling": {"max_joint_step_rad": 0.05, "max_period_s": 1.0},
        "trace": {"tool_speed_m_s": 0.05, "min_tool_speed_m_s": 0.005, "speed_tolerance": 0.15, "max_ramp_m": 0.01},
        "value_status": "nominal",
        "sources": {"limit_fraction": "x", "sampling": "x", "trace": "x"},
    }
    if extra_key:
        raw["bogus_key"] = 123
    path = tmp_path / name
    path.write_text(yaml.safe_dump(raw))
    return path


def _write_execution_config(tmp_path: Path, *, margin: float = 0.03, name: str = "execution.yaml") -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump({"schema": "crackvision.execution_config/1", "position_margin_rad": margin}))
    return path


def _common_argv(tmp_path: Path, plan_path: Path, out_path: Path, limits: Path, ee: Path, urdf: Path, retime_cfg: Path, exec_cfg: Path, *, extra=()) -> List[str]:
    return [
        "--root", str(tmp_path),
        "--plan", str(plan_path),
        "--out", str(out_path),
        "--limits", str(limits),
        "--end-effector-config", str(ee),
        "--urdf", str(urdf),
        "--retime-config", str(retime_cfg),
        "--execution-config", str(exec_cfg),
        *extra,
    ]


@pytest.fixture()
def fixtures(tmp_path):
    limits = _copy_in(tmp_path, LIMITS_PATH, "b601_dm_limits.yaml")
    ee = _copy_in(tmp_path, END_EFFECTOR_PATH, "end_effector.yaml")
    urdf = _copy_in(tmp_path, URDF_PATH, "robot.urdf")
    plan_path = _build_plan(tmp_path, limits, ee)
    retime_cfg = _write_retime_config(tmp_path)
    exec_cfg = _write_execution_config(tmp_path)
    out_path = tmp_path / "out" / "case_trajectory.json"
    return {
        "tmp_path": tmp_path, "limits": limits, "ee": ee, "urdf": urdf, "plan_path": plan_path,
        "retime_cfg": retime_cfg, "exec_cfg": exec_cfg, "out_path": out_path,
    }


# --------------------------------------------------------------------------------------
# exit 0: success
# --------------------------------------------------------------------------------------

def test_main_writes_trajectory_and_report_on_success(fixtures):
    argv = _common_argv(
        fixtures["tmp_path"], fixtures["plan_path"], fixtures["out_path"], fixtures["limits"],
        fixtures["ee"], fixtures["urdf"], fixtures["retime_cfg"], fixtures["exec_cfg"],
    )
    rc = main(argv)
    assert rc == EXIT_OK

    out_path = fixtures["out_path"]
    assert out_path.is_file()
    report_path = Path(str(out_path) + ".retime.json")
    assert report_path.is_file()

    traj = load_trajectory(out_path)
    assert traj["producer"] == "retime_plan"
    assert traj["purpose"] == "crack_task"

    report = validate_retime_report(json.loads(report_path.read_text()))
    assert report["self_check"] == {"violations": 0}

    # §0.4 logging artefacts.
    log_dir = fixtures["tmp_path"] / "logs"
    assert list(log_dir.glob("retime_plan_*.log"))
    assert (log_dir / "retime_plan_latest.json").is_file()


def test_main_respects_purpose_test(fixtures):
    argv = _common_argv(
        fixtures["tmp_path"], fixtures["plan_path"], fixtures["out_path"], fixtures["limits"],
        fixtures["ee"], fixtures["urdf"], fixtures["retime_cfg"], fixtures["exec_cfg"],
        extra=["--purpose", "test"],
    )
    assert main(argv) == EXIT_OK
    traj = load_trajectory(fixtures["out_path"])
    assert traj["purpose"] == "test"


def test_main_skip_existing(fixtures):
    argv = _common_argv(
        fixtures["tmp_path"], fixtures["plan_path"], fixtures["out_path"], fixtures["limits"],
        fixtures["ee"], fixtures["urdf"], fixtures["retime_cfg"], fixtures["exec_cfg"],
    )
    assert main(argv) == EXIT_OK
    mtime_before = fixtures["out_path"].stat().st_mtime_ns
    assert main([*argv, "--skip-existing"]) == EXIT_OK
    assert fixtures["out_path"].stat().st_mtime_ns == mtime_before


# --------------------------------------------------------------------------------------
# --dry-run
# --------------------------------------------------------------------------------------

def test_dry_run_writes_nothing_and_prints_summary(fixtures, capsys):
    argv = _common_argv(
        fixtures["tmp_path"], fixtures["plan_path"], fixtures["out_path"], fixtures["limits"],
        fixtures["ee"], fixtures["urdf"], fixtures["retime_cfg"], fixtures["exec_cfg"],
        extra=["--dry-run"],
    )
    rc = main(argv)
    assert rc == EXIT_OK
    assert not fixtures["out_path"].exists()
    assert not Path(str(fixtures["out_path"]) + ".retime.json").exists()
    assert not (fixtures["tmp_path"] / "logs").exists()

    captured = capsys.readouterr()
    lines = [line for line in captured.out.splitlines() if line.startswith("leg ")]
    assert len(lines) == 4
    for line in lines:
        assert "configurations=" in line
        assert "refined_points=" in line
        assert "duration=" in line
    assert "trace" in captured.out
    assert "v_c=" in captured.out


def test_dry_run_on_refusal_still_exits_0_and_writes_nothing(fixtures, capsys):
    bad_limits = _copy_in(fixtures["tmp_path"], LIMITS_PATH, "other_limits.yaml")
    argv = _common_argv(
        fixtures["tmp_path"], fixtures["plan_path"], fixtures["out_path"], bad_limits,
        fixtures["ee"], fixtures["urdf"], fixtures["retime_cfg"], fixtures["exec_cfg"],
        extra=["--dry-run"],
    )
    # mutate the copy so its sha256 differs from what the plan recorded.
    bad_limits.write_text(bad_limits.read_text() + "\n# tweak\n")
    rc = main(argv)
    assert rc == EXIT_OK
    assert not fixtures["out_path"].exists()
    assert "refused" in capsys.readouterr().err


# --------------------------------------------------------------------------------------
# exit 3: refusal
# --------------------------------------------------------------------------------------

def test_limits_sha_mismatch_refuses_and_writes_nothing(fixtures):
    bad_limits = fixtures["tmp_path"] / "other_limits.yaml"
    bad_limits.write_text(LIMITS_PATH.read_text() + "\n# tweak\n")
    argv = _common_argv(
        fixtures["tmp_path"], fixtures["plan_path"], fixtures["out_path"], bad_limits,
        fixtures["ee"], fixtures["urdf"], fixtures["retime_cfg"], fixtures["exec_cfg"],
    )
    rc = main(argv)
    assert rc == EXIT_PRECONDITION
    assert not fixtures["out_path"].exists()


def test_invalid_plan_refuses(fixtures):
    bad_plan = fixtures["tmp_path"] / "bad_plan.json"
    bad_plan.write_text(json.dumps({"schema": "crackvision.cartesian_plan/1"}))
    argv = _common_argv(
        fixtures["tmp_path"], bad_plan, fixtures["out_path"], fixtures["limits"],
        fixtures["ee"], fixtures["urdf"], fixtures["retime_cfg"], fixtures["exec_cfg"],
    )
    rc = main(argv)
    assert rc == EXIT_PRECONDITION
    assert not fixtures["out_path"].exists()


def test_missing_plan_file_refuses(fixtures):
    argv = _common_argv(
        fixtures["tmp_path"], fixtures["tmp_path"] / "missing_plan.json", fixtures["out_path"],
        fixtures["limits"], fixtures["ee"], fixtures["urdf"], fixtures["retime_cfg"], fixtures["exec_cfg"],
    )
    rc = main(argv)
    assert rc == EXIT_PRECONDITION
    assert not fixtures["out_path"].exists()


def test_self_check_violation_refuses(fixtures, monkeypatch):
    import crackvision_motion.retime as retime_mod
    from crackvision_motion.joint_trajectory import Violation

    def _fake_check_limits(*args, **kwargs):
        return [Violation(index=0, joint=JOINT_NAMES[0], kind="position", value=99.0, bound=1.0)]

    monkeypatch.setattr(retime_mod, "check_limits", _fake_check_limits)
    argv = _common_argv(
        fixtures["tmp_path"], fixtures["plan_path"], fixtures["out_path"], fixtures["limits"],
        fixtures["ee"], fixtures["urdf"], fixtures["retime_cfg"], fixtures["exec_cfg"],
    )
    rc = main(argv)
    assert rc == EXIT_PRECONDITION
    assert not fixtures["out_path"].exists()
    assert not Path(str(fixtures["out_path"]) + ".retime.json").exists()


def test_trace_below_min_speed_refuses(fixtures):
    retime_cfg = _write_retime_config(fixtures["tmp_path"], name="retime_strict.yaml")
    raw = yaml.safe_load(retime_cfg.read_text())
    raw["trace"]["min_tool_speed_m_s"] = 10.0
    raw["trace"]["tool_speed_m_s"] = 20.0
    retime_cfg.write_text(yaml.safe_dump(raw))
    argv = _common_argv(
        fixtures["tmp_path"], fixtures["plan_path"], fixtures["out_path"], fixtures["limits"],
        fixtures["ee"], fixtures["urdf"], retime_cfg, fixtures["exec_cfg"],
    )
    rc = main(argv)
    assert rc == EXIT_PRECONDITION
    assert not fixtures["out_path"].exists()


# --------------------------------------------------------------------------------------
# exit 2: usage/config error
# --------------------------------------------------------------------------------------

def test_unknown_retime_config_key_is_usage_error(fixtures):
    retime_cfg = _write_retime_config(fixtures["tmp_path"], name="retime_bad.yaml", extra_key=True)
    argv = _common_argv(
        fixtures["tmp_path"], fixtures["plan_path"], fixtures["out_path"], fixtures["limits"],
        fixtures["ee"], fixtures["urdf"], retime_cfg, fixtures["exec_cfg"],
    )
    rc = main(argv)
    assert rc == EXIT_USAGE
    assert not fixtures["out_path"].exists()


def test_bad_flag_is_usage_error(fixtures):
    with pytest.raises(SystemExit) as exc_info:
        main(["--plan", str(fixtures["plan_path"]), "--bogus-flag", "x"])
    assert exc_info.value.code == EXIT_USAGE


def test_missing_required_plan_flag_is_usage_error():
    with pytest.raises(SystemExit) as exc_info:
        main([])
    assert exc_info.value.code == EXIT_USAGE


# --------------------------------------------------------------------------------------
# exit 1: unexpected runtime error
# --------------------------------------------------------------------------------------

def test_unexpected_exception_is_runtime_error(fixtures, monkeypatch):
    import crackvision_motion.retime_plan as retime_plan_mod

    def _boom(*args, **kwargs):
        raise RuntimeError("boom")

    monkeypatch.setattr(retime_plan_mod, "write_outputs", _boom)
    argv = _common_argv(
        fixtures["tmp_path"], fixtures["plan_path"], fixtures["out_path"], fixtures["limits"],
        fixtures["ee"], fixtures["urdf"], fixtures["retime_cfg"], fixtures["exec_cfg"],
    )
    rc = main(argv)
    assert rc == EXIT_RUNTIME
    assert not fixtures["out_path"].exists()

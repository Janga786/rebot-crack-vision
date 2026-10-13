"""Unit tests for retime_plan's `--view-plan` leg (MOT-07.10.3, §17.3): retiming a
`crackvision.view_plan/1`'s `transit_leg` into a `purpose: "view_move"` joint_trajectory, through
the shared §16.2 algorithm (`retime.retime_view_plan_doc`).

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_retime_view.py -q -p no:cacheprovider

The view plan is built with `cartesian_plan`'s own incremental builders (`new_view_plan`/
`add_view_candidate`/`build_leg`/`finalize_view`), never a bare hand-rolled dict, with `sources`
hashed from the real production config files -- mirroring `test_execution_gate.py`'s own
`purpose: "view_move"` fixtures (MOT-07.10.2) and `test_retime.py`'s synthetic-plan construction.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

import pytest

from crackvision_motion.cartesian_plan import (
    add_view_candidate,
    build_leg,
    finalize_view,
    new_view_plan,
    write_view_plan,
)
from crackvision_motion.cli_common import EXIT_OK, EXIT_PRECONDITION, EXIT_USAGE
from crackvision_motion.execution_gate import G_ELIGIBLE, G_LIMITS, G_TRAJ, evaluate_offline
from crackvision_motion.joint_trajectory import file_sha256, load_limits, load_trajectory
from crackvision_motion.retime import validate_retime_report
from crackvision_motion.retime_plan import main

REPO_ROOT = Path(__file__).resolve().parents[4]
LIMITS_PATH = REPO_ROOT / "config" / "robot" / "b601_dm_limits.yaml"
END_EFFECTOR_PATH = REPO_ROOT / "config" / "robot" / "end_effector.yaml"
URDF_PATH = REPO_ROOT / "presentation" / "sim" / "reBot_B601_DM_with_gripper.urdf"
RETIME_CONFIG_PATH = REPO_ROOT / "config" / "motion" / "retime.yaml"
EXECUTION_CONFIG_PATH = REPO_ROOT / "config" / "motion" / "execution.yaml"
COMMISSIONING_PATH = REPO_ROOT / "config" / "robot" / "commissioning.yaml"
SCENE_PATH = REPO_ROOT / "config" / "scene" / "scene.yaml"

LIMITS = load_limits(LIMITS_PATH)

Q_HOME = [0.0] * 6
Q_VIEW = [0.0, -0.9, -0.6, 0.3, 0.3, 0.0]


def _view_recheck() -> Dict[str, Any]:
    arr = [Q_VIEW[j] - Q_HOME[j] for j in range(6)]
    return {
        "max_pos_err_m": 0.0, "max_ang_err_rad": 0.0, "all_valid": True, "within_limits": True,
        "max_joint_step_rad": max(abs(x) for x in arr),
    }


def _view_pose() -> Dict[str, Any]:
    return {"position_m": [0.0, 0.0, 0.5], "quat_xyzw": [0.0, 0.0, 0.0, 1.0]}


def _view_fov_check() -> Dict[str, Any]:
    return {"passed": True, "horizontal_margins_deg": [1.0] * 4, "vertical_margins_deg": [1.0] * 4}


def _view_plan_sources(config_sha: str = "c" * 64) -> Dict[str, Any]:
    return {
        "scene": {"path": "config/scene/scene.yaml", "sha256": file_sha256(SCENE_PATH)},
        "end_effector": {"path": "config/robot/end_effector.yaml", "sha256": file_sha256(END_EFFECTOR_PATH)},
        "limits": {"path": "config/robot/b601_dm_limits.yaml", "sha256": file_sha256(LIMITS_PATH)},
        "config": {"path": "config/motion/cartesian_planning.yaml", "sha256": config_sha},
    }


def _write_view_plan(
    tmp_path: Path, name: str = "view_plan.json", *, chosen: bool = True,
    positions: List[List[float]] = None, sources: Dict[str, Any] = None,
) -> Path:
    plan = new_view_plan(
        sources if sources is not None else _view_plan_sources(), {"view_distance_m": 0.25},
        "--center", [0.1, 0.2], 0.0, [0.2, 0.2],
    )
    if chosen:
        add_view_candidate(plan, 0.0, 0.0, _view_pose(), _view_fov_check(), True, True)
        pts = positions if positions is not None else [Q_HOME, Q_VIEW]
        transit_leg = build_leg("transit", None, None, None, pts, [None] * len(pts), _view_recheck())
        finalize_view(plan, {"tilt_deg": 0.0, "yaw_deg": 0.0}, transit_leg, LIMITS)
    else:
        finalize_view(plan, None, None, LIMITS)
    path = tmp_path / name
    write_view_plan(plan, path)
    return path


def _argv(tmp_path: Path, view_plan_path: Path, out_path: Path, *, extra=()) -> List[str]:
    return [
        "--root", str(tmp_path),
        "--view-plan", str(view_plan_path),
        "--out", str(out_path),
        "--limits", str(LIMITS_PATH),
        "--end-effector-config", str(END_EFFECTOR_PATH),
        "--urdf", str(URDF_PATH),
        "--retime-config", str(RETIME_CONFIG_PATH),
        "--execution-config", str(EXECUTION_CONFIG_PATH),
        *extra,
    ]


# --------------------------------------------------------------------------------------
# happy path
# --------------------------------------------------------------------------------------

def test_view_plan_retimes_to_view_move_trajectory(tmp_path):
    vp_path = _write_view_plan(tmp_path)
    out_path = tmp_path / "out" / "view_trajectory.json"
    rc = main(_argv(tmp_path, vp_path, out_path))
    assert rc == EXIT_OK

    assert out_path.is_file()
    report_path = Path(str(out_path) + ".retime.json")
    assert report_path.is_file()

    traj = load_trajectory(out_path)
    assert traj["purpose"] == "view_move"
    assert traj["source"]["paths3d"] is None
    assert traj["source"]["view_plan"]["sha256"] == file_sha256(vp_path)
    assert traj["source"]["view_plan"]["path"] == str(vp_path.resolve().relative_to(tmp_path.resolve()))

    assert traj["points"][0]["positions"] == Q_HOME
    assert traj["points"][-1]["positions"] == Q_VIEW

    report = json.loads(report_path.read_text())
    report = validate_retime_report(report)
    assert len(report["legs"]) == 1
    assert report["legs"][0]["kind"] == "transit"
    assert "tool_speed" not in report["legs"][0]


def test_dry_run_writes_nothing(tmp_path):
    vp_path = _write_view_plan(tmp_path)
    out_path = tmp_path / "out" / "view_trajectory.json"
    rc = main(_argv(tmp_path, vp_path, out_path, extra=["--dry-run"]))
    assert rc == EXIT_OK
    assert not out_path.exists()


def test_default_out_is_view_trajectory_json(tmp_path):
    vp_path = _write_view_plan(tmp_path)
    argv = [
        "--root", str(tmp_path),
        "--view-plan", str(vp_path),
        "--limits", str(LIMITS_PATH),
        "--end-effector-config", str(END_EFFECTOR_PATH),
        "--urdf", str(URDF_PATH),
        "--retime-config", str(RETIME_CONFIG_PATH),
        "--execution-config", str(EXECUTION_CONFIG_PATH),
    ]
    rc = main(argv)
    assert rc == EXIT_OK
    default_out = tmp_path / "data" / "trajectories" / "view_trajectory.json"
    assert default_out.is_file()


# --------------------------------------------------------------------------------------
# usage errors (exit 2)
# --------------------------------------------------------------------------------------

def test_both_plan_and_view_plan_is_exit2(tmp_path):
    vp_path = _write_view_plan(tmp_path)
    argv = _argv(tmp_path, vp_path, tmp_path / "out.json")
    argv += ["--plan", str(vp_path)]
    with pytest.raises(SystemExit) as exc_info:
        main(argv)
    assert exc_info.value.code == EXIT_USAGE


def test_neither_plan_nor_view_plan_is_exit2(tmp_path):
    argv = [
        "--root", str(tmp_path),
        "--limits", str(LIMITS_PATH),
        "--end-effector-config", str(END_EFFECTOR_PATH),
        "--urdf", str(URDF_PATH),
        "--retime-config", str(RETIME_CONFIG_PATH),
        "--execution-config", str(EXECUTION_CONFIG_PATH),
    ]
    with pytest.raises(SystemExit) as exc_info:
        main(argv)
    assert exc_info.value.code == EXIT_USAGE


def test_purpose_with_view_plan_is_exit2(tmp_path):
    vp_path = _write_view_plan(tmp_path)
    out_path = tmp_path / "out.json"
    argv = _argv(tmp_path, vp_path, out_path, extra=["--purpose", "test"])
    rc = main(argv)
    assert rc == EXIT_USAGE
    assert not out_path.exists()


# --------------------------------------------------------------------------------------
# refusals (exit 3)
# --------------------------------------------------------------------------------------

def test_invalid_view_plan_is_exit3(tmp_path):
    vp_path = tmp_path / "bad_view_plan.json"
    vp_path.write_text(json.dumps({"schema": "crackvision.view_plan/1"}))
    out_path = tmp_path / "out.json"
    rc = main(_argv(tmp_path, vp_path, out_path))
    assert rc == EXIT_PRECONDITION
    assert not out_path.exists()


def test_null_transit_leg_is_exit3(tmp_path):
    vp_path = _write_view_plan(tmp_path, chosen=False)
    out_path = tmp_path / "out.json"
    rc = main(_argv(tmp_path, vp_path, out_path))
    assert rc == EXIT_PRECONDITION
    assert not out_path.exists()


def test_limits_sha_mismatch_is_exit3(tmp_path):
    # `validate_view_plan`/`load_view_plan` never cross-check `sources.limits.sha256` against the
    # limits file's actual content (only its shape, §13.14.1) -- the §16.3 sha-agreement rule
    # against the limits file `retime_plan` itself reads is `retime_view_plan_doc`'s own check.
    sources = _view_plan_sources()
    sources["limits"]["sha256"] = "a" * 64
    vp_path = _write_view_plan(tmp_path, sources=sources)
    out_path = tmp_path / "out.json"
    rc = main(_argv(tmp_path, vp_path, out_path))
    assert rc == EXIT_PRECONDITION
    assert not out_path.exists()


def test_end_effector_sha_mismatch_is_exit3(tmp_path):
    sources = _view_plan_sources()
    sources["end_effector"]["sha256"] = "b" * 64
    vp_path = _write_view_plan(tmp_path, sources=sources)
    out_path = tmp_path / "out.json"
    rc = main(_argv(tmp_path, vp_path, out_path))
    assert rc == EXIT_PRECONDITION
    assert not out_path.exists()


# --------------------------------------------------------------------------------------
# joint_trajectory + execution_gate acceptance (§17.1/§17.2)
# --------------------------------------------------------------------------------------

def test_trajectory_accepted_by_execution_gate_mock(tmp_path):
    vp_path = _write_view_plan(tmp_path)
    out_path = tmp_path / "out" / "view_trajectory.json"
    rc = main(_argv(tmp_path, vp_path, out_path))
    assert rc == EXIT_OK

    report = evaluate_offline(
        "mock", out_path,
        root=tmp_path,
        execution_config_path=EXECUTION_CONFIG_PATH,
        commissioning_path=COMMISSIONING_PATH,
        limits_path=LIMITS_PATH,
        scene_config_path=SCENE_PATH,
        end_effector_config_path=END_EFFECTOR_PATH,
        approval_path=None,
        speed_scale=None,
        env={},
    )
    by_id = {c.id: c for c in report.checks}
    assert by_id[G_TRAJ].outcome == "pass"
    assert by_id[G_LIMITS].outcome == "pass"
    assert by_id[G_ELIGIBLE].outcome == "pass"

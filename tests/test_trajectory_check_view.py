"""tests/test_trajectory_check_view.py — MOT-07.10.4 `view_move` extension to
`crackvision.trajectory_check` (docs/INTERFACES.md §17.4).

Exercises schema dispatch between `crackvision.cartesian_plan/1` and `crackvision.view_plan/1`,
purpose<->plan-schema pairing, the `view_plan` provenance rules, the single-leg polyline built from
`transit_leg`, the vacuous C-TOOLSPEED pass and single-transit-leg C-REPORT -- all against hand-built
view plans and trajectories (`crackvision.kinematics` FK only, never `crackvision_motion`).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import yaml

import crackvision.kinematics as kin
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION
from crackvision.trajectory_check import (
    JOINT_NAMES,
    check_limits,
    check_path,
    check_provenance,
    check_report,
    file_sha256,
    main,
    run_checks,
    validate_plan_schema,
    validate_trajectory_schema,
    validate_view_plan_schema,
)

CHAIN = kin.load_chain()
END_EFFECTOR = kin.load_end_effector()

LIMITS_SHA = "l" * 64
EE_SHA = "e" * 64
SCENE_SHA = "s" * 64
CONFIG_SHA = "c" * 64
VIEW_PLAN_SHA = "ab" * 32

LIMITS = {
    "joints": {
        "joint1": {"lower": -2.8, "upper": 2.8, "velocity": 1.0, "acceleration": 1.0},
        "joint2": {"lower": -3.14, "upper": 0.0, "velocity": 1.0, "acceleration": 1.0},
        "joint3": {"lower": -3.14, "upper": 0.0, "velocity": 1.0, "acceleration": 1.0},
        "joint4": {"lower": -1.87, "upper": 1.57, "velocity": 1.0, "acceleration": 1.0},
        "joint5": {"lower": -1.57, "upper": 1.57, "velocity": 1.0, "acceleration": 1.0},
        "joint6": {"lower": -3.14, "upper": 3.14, "velocity": 1.0, "acceleration": 1.0},
    }
}
MARGIN_RAD = 0.03
RETIME_CFG = {
    "limit_fraction": {"velocity": 0.95, "acceleration": 0.95},
    "sampling": {"max_joint_step_rad": 1.0, "max_period_s": 1.0},
}

HOME = [0.0] * 6
CHOSEN = [0.3, -0.5, -0.4, 0.2, 0.1, 0.0]


def _ease_points(n: int = 20, dt: float = 0.5) -> list[dict]:
    """Rest-to-rest joint-space ease (cosine profile) HOME -> CHOSEN, slow enough to pass
    C-LIMITS/C-MARGIN/C-REST/C-SAMPLING against `LIMITS`/`RETIME_CFG` by a wide margin; endpoints
    are forced bit-exact to HOME/CHOSEN (G-PATH requires this, §16.2 Step 1)."""
    points = []
    for i in range(n + 1):
        s = (1.0 - math.cos(math.pi * i / n)) / 2.0
        pos = [h + (c - h) * s for h, c in zip(HOME, CHOSEN)]
        points.append({"t_s": i * dt, "positions": pos})
    points[0]["positions"] = list(HOME)
    points[-1]["positions"] = list(CHOSEN)
    return points


def _view_plan(
    limits_sha=LIMITS_SHA, ee_sha=EE_SHA, scene_sha=SCENE_SHA, config_sha=CONFIG_SHA,
    transit_positions=None, transit_kind="transit", leg_index=0,
):
    if transit_positions is None:
        transit_positions = [list(HOME), list(CHOSEN)]
    return {
        "schema": "crackvision.view_plan/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "plan_view",
        "planning_frame": "base_link",
        "sources": {
            "scene": {"path": "config/scene/scene.yaml", "sha256": scene_sha},
            "end_effector": {"path": "config/robot/end_effector.yaml", "sha256": ee_sha},
            "limits": {"path": "config/robot/b601_dm_limits.yaml", "sha256": limits_sha},
            "config": {"path": "config/motion/cartesian_planning.yaml", "sha256": config_sha},
        },
        "parameters": {},
        "center_source": "scene.specimen",
        "center_xy_m": [0.0, 0.0],
        "surface_z_m": 0.0,
        "footprint_m": [0.1, 0.1],
        "candidates": [],
        "chosen": {"tilt_deg": 0, "yaw_deg": 0},
        "transit_leg": {
            "leg_index": leg_index,
            "kind": transit_kind,
            "segment_ref": None,
            "direction": None,
            "roll_offset_rad": None,
            "positions": [list(p) for p in transit_positions],
            "waypoint_index_map": [None] * len(transit_positions),
            "recheck": {
                "max_pos_err_m": 0.0,
                "max_ang_err_rad": 0.0,
                "all_valid": True,
                "within_limits": True,
                "max_joint_step_rad": 0.0,
            },
        },
        "caveats": [],
    }


def _view_traj(
    points, purpose="view_move", view_plan_sha=VIEW_PLAN_SHA, view_plan_path="data/plans/view_plan.json",
    limits_sha=LIMITS_SHA, ee_sha=EE_SHA, scene_sha=SCENE_SHA, source=None,
):
    if source is None:
        source = {"paths3d": None, "view_plan": {"path": view_plan_path, "sha256": view_plan_sha}}
    return {
        "schema": "crackvision.joint_trajectory/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "test",
        "purpose": purpose,
        "planning_frame": "base_link",
        "joint_names": JOINT_NAMES,
        "points": points,
        "limits_file": {"path": "config/robot/b601_dm_limits.yaml", "sha256": limits_sha},
        "end_effector_config_sha256": ee_sha,
        "scene_config_sha256": scene_sha,
        "source": source,
    }


def _crack_task_traj(points, **kwargs):
    paths3d = {"path": "data/paths/case1_paths3d.json", "sha256": "z" * 64, "execution_eligible": True}
    kwargs.setdefault("purpose", "crack_task")
    return _view_traj(points, source={"paths3d": paths3d}, **kwargs)


def _cartesian_plan():
    return {
        "schema": "crackvision.cartesian_plan/1",
        "case_id": "case1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "plan_crack_path",
        "planning_frame": "base_link",
        "joint_names": JOINT_NAMES,
        "sources": {
            "paths3d": {"path": "data/paths/case1_paths3d.json", "sha256": "z" * 64, "execution_eligible": True},
            "scene": {"path": "config/scene/scene.yaml", "sha256": SCENE_SHA},
            "end_effector": {"path": "config/robot/end_effector.yaml", "sha256": EE_SHA},
            "limits": {"path": "config/robot/b601_dm_limits.yaml", "sha256": LIMITS_SHA},
            "config": {"path": "config/motion/cartesian_planning.yaml", "sha256": CONFIG_SHA},
        },
        "parameters": {},
        "start_state": {"joint_names": JOINT_NAMES, "positions_rad": HOME, "source": "start_joints_override"},
        "end_state": {"mode": "none", "positions_rad": None},
        "legs": [
            {
                "leg_index": 0,
                "kind": "transit",
                "segment_ref": None,
                "direction": None,
                "roll_offset_rad": None,
                "positions": [list(HOME), list(CHOSEN)],
                "waypoint_index_map": [None, None],
                "recheck": {
                    "max_pos_err_m": 0.0, "max_ang_err_rad": 0.0, "all_valid": True,
                    "within_limits": True, "max_joint_step_rad": 0.0,
                },
            }
        ],
        "segments": [],
        "coverage": {},
        "caveats": [],
    }


def _files(plan_sha=VIEW_PLAN_SHA):
    return {
        "trajectory_sha256": "t" * 64,
        "limits": {"sha256": LIMITS_SHA},
        "end_effector": {"sha256": EE_SHA},
        "plan_sha256": plan_sha,
    }


# ---------------------------------------------------------------------------
# C-SCHEMA: dispatch and the view_plan/view_move shapes
# ---------------------------------------------------------------------------


def test_validate_plan_schema_dispatches_to_view_plan():
    assert validate_plan_schema(_view_plan()) == []
    assert validate_view_plan_schema(_view_plan()) == []


def test_validate_plan_schema_still_validates_cartesian_plan():
    assert validate_plan_schema(_cartesian_plan()) == []


def test_validate_view_plan_schema_requires_transit_kind():
    bad = _view_plan(transit_kind="trace")
    errors = validate_view_plan_schema(bad)
    assert any("transit_leg.kind" in e for e in errors)


def test_validate_view_plan_schema_requires_chosen():
    bad = _view_plan()
    bad["chosen"] = None
    errors = validate_view_plan_schema(bad)
    assert any("chosen" in e for e in errors)


def test_validate_view_plan_schema_requires_sources_blocks():
    bad = _view_plan()
    del bad["sources"]["scene"]
    errors = validate_view_plan_schema(bad)
    assert any("sources.scene" in e for e in errors)


def test_validate_trajectory_schema_view_move_valid_passes():
    assert validate_trajectory_schema(_view_traj(_ease_points())) == []


def test_validate_trajectory_schema_view_move_requires_null_paths3d():
    traj = _view_traj(_ease_points())
    traj["source"]["paths3d"] = {"path": "x", "sha256": "y" * 64, "execution_eligible": True}
    errors = validate_trajectory_schema(traj)
    assert any("source.paths3d must be null" in e for e in errors)


def test_validate_trajectory_schema_view_move_requires_view_plan_key():
    traj = _view_traj(_ease_points())
    del traj["source"]["view_plan"]
    errors = validate_trajectory_schema(traj)
    assert any("view_plan" in e for e in errors)


def test_validate_trajectory_schema_view_move_rejects_bad_sha():
    traj = _view_traj(_ease_points())
    traj["source"]["view_plan"]["sha256"] = "not-hex"
    errors = validate_trajectory_schema(traj)
    assert any("view_plan.sha256" in e for e in errors)


def test_validate_trajectory_schema_crack_task_rejects_view_plan_key_even_when_null():
    traj = _crack_task_traj(_ease_points())
    traj["source"]["view_plan"] = None
    errors = validate_trajectory_schema(traj)
    assert any("view_plan" in e and "must not be present" in e for e in errors)


def test_validate_trajectory_schema_purpose_accepts_view_move():
    traj = _view_traj(_ease_points())
    assert "purpose" not in "".join(validate_trajectory_schema(traj))


# ---------------------------------------------------------------------------
# C-PROVENANCE: purpose<->plan-schema pairing and view_plan sha rules
# ---------------------------------------------------------------------------


def test_provenance_view_move_passes_when_everything_matches():
    result = check_provenance(_view_traj(_ease_points()), _view_plan(), _files())
    assert result["outcome"] == "pass", result["detail"]


def test_provenance_wrong_view_plan_sha_fails():
    traj = _view_traj(_ease_points(), view_plan_sha="0" * 64)
    result = check_provenance(traj, _view_plan(), _files())
    assert result["outcome"] == "fail"
    assert "view_plan.sha256" in result["detail"]


def test_provenance_view_move_against_cartesian_plan_fails():
    traj = _view_traj(_ease_points())
    result = check_provenance(traj, _cartesian_plan(), _files())
    assert result["outcome"] == "fail"
    assert "requires --plan schema" in result["detail"]


def test_provenance_crack_task_against_view_plan_fails():
    traj = _crack_task_traj(_ease_points())
    result = check_provenance(traj, _view_plan(), _files())
    assert result["outcome"] == "fail"
    assert "requires --plan schema" in result["detail"]


def test_provenance_view_move_limits_sha_mismatch_fails():
    traj = _view_traj(_ease_points(), limits_sha="9" * 64)
    result = check_provenance(traj, _view_plan(), _files())
    assert result["outcome"] == "fail"
    assert "limits_file.sha256" in result["detail"]


# ---------------------------------------------------------------------------
# C-PATH: single-leg polyline (transit_leg only, no concatenation)
# ---------------------------------------------------------------------------


def test_path_clean_view_move_trajectory_passes():
    result = check_path(_view_traj(_ease_points()), _view_plan())
    assert result["outcome"] == "pass", result["detail"]
    assert result["numbers"]["n_plan_vertices"] == 2


def test_path_off_plan_point_fails():
    points = _ease_points()
    points[5]["positions"][2] += 0.05  # well over the 1e-6 rad C-PATH tolerance
    result = check_path(_view_traj(points), _view_plan())
    assert result["outcome"] == "fail"
    assert "tolerance" in result["detail"]


# ---------------------------------------------------------------------------
# C-LIMITS: unaffected by purpose, still runs against the view_move trajectory
# ---------------------------------------------------------------------------


def test_limits_overspeed_view_move_sample_fails():
    points = [
        {"t_s": 0.0, "positions": [0, 0, 0, 0, 0, 0]},
        {"t_s": 0.5, "positions": [0.05, 0, 0, 0, 0, 0]},
        {"t_s": 1.0, "positions": [2.05, 0, 0, 0, 0, 0]},  # v=4.0 rad/s, exceeds joint1 limit 1.0
        {"t_s": 1.5, "positions": [2.1, 0, 0, 0, 0, 0]},
    ]
    result = check_limits(_view_traj(points), LIMITS)
    assert result["outcome"] == "fail"
    assert "exceeds limit" in result["detail"]


# ---------------------------------------------------------------------------
# C-TOOLSPEED: vacuous pass (no trace legs on a view-move polyline)
# ---------------------------------------------------------------------------


def test_toolspeed_vacuous_pass_via_run_checks():
    points = _ease_points()
    traj = _view_traj(points)
    plan = _view_plan()
    t = [p["t_s"] for p in points]
    report = {
        "schema": "crackvision.retime_report/1",
        "legs": [{"leg_index": 0, "kind": "transit", "point_range": [0, len(points) - 1], "duration_s": t[-1] - t[0]}],
    }
    result = run_checks(
        traj, plan, limits=LIMITS, margin_rad=MARGIN_RAD, retime_cfg=RETIME_CFG, files=_files(),
        extended=True, chain=CHAIN, end_effector=END_EFFECTOR, report_doc=report,
    )
    outcomes = {c["check"]: c["outcome"] for c in result["checks"]}
    assert outcomes == {
        "C-SCHEMA": "pass", "C-PROVENANCE": "pass", "C-LIMITS": "pass", "C-MARGIN": "pass",
        "C-REST": "pass", "C-SAMPLING": "pass", "C-PATH": "pass", "C-TOOLSPEED": "pass",
        "C-REPORT": "pass",
    }
    toolspeed = next(c for c in result["checks"] if c["check"] == "C-TOOLSPEED")
    assert toolspeed["numbers"] == {"trace_legs": []}
    schema_check = next(c for c in result["checks"] if c["check"] == "C-SCHEMA")
    assert "crackvision.view_plan/1" in schema_check["detail"]


# ---------------------------------------------------------------------------
# C-REPORT: single transit-leg point_range/duration_s, no tool_speed block
# ---------------------------------------------------------------------------


def test_report_consistent_view_move_passes():
    points = _ease_points()
    traj = _view_traj(points)
    plan = _view_plan()
    t = [p["t_s"] for p in points]
    report = {
        "schema": "crackvision.retime_report/1",
        "legs": [{"leg_index": 0, "kind": "transit", "point_range": [0, len(points) - 1], "duration_s": t[-1] - t[0]}],
    }
    result = check_report(traj, plan, RETIME_CFG, None, None, report)
    assert result["outcome"] == "pass", result["detail"]


def test_report_wrong_point_range_fails():
    points = _ease_points()
    traj = _view_traj(points)
    plan = _view_plan()
    t = [p["t_s"] for p in points]
    report = {
        "schema": "crackvision.retime_report/1",
        "legs": [{"leg_index": 0, "kind": "transit", "point_range": [0, len(points) - 2], "duration_s": t[-1] - t[0]}],
    }
    result = check_report(traj, plan, RETIME_CFG, None, None, report)
    assert result["outcome"] == "fail"
    assert "point_range" in result["detail"]


# ---------------------------------------------------------------------------
# CLI: full wiring end-to-end
# ---------------------------------------------------------------------------


def _project_root(tmp_path: Path) -> Path:
    root = tmp_path / "proj"
    (root / "config" / "robot").mkdir(parents=True)
    (root / "config" / "motion").mkdir(parents=True)
    (root / "config" / "robot" / "end_effector.yaml").write_text("tool: {}\n", encoding="utf-8")
    (root / "config" / "robot" / "b601_dm_limits.yaml").write_text(yaml.safe_dump(LIMITS), encoding="utf-8")
    (root / "config" / "motion" / "execution.yaml").write_text(
        yaml.safe_dump({"position_margin_rad": MARGIN_RAD}), encoding="utf-8"
    )
    (root / "config" / "motion" / "retime.yaml").write_text(yaml.safe_dump(RETIME_CFG), encoding="utf-8")
    (root / "config" / "project.yaml").write_text("schema_version: 1\n", encoding="utf-8")
    return root


def _write(path: Path, doc) -> None:
    path.write_text(json.dumps(doc), encoding="utf-8")


def _cli_view_files(tmp_path: Path, root: Path, points):
    limits_sha = file_sha256(root / "config" / "robot" / "b601_dm_limits.yaml")
    ee_sha = file_sha256(root / "config" / "robot" / "end_effector.yaml")
    plan_path = tmp_path / "view_plan.json"
    _write(plan_path, _view_plan(limits_sha=limits_sha, ee_sha=ee_sha))
    plan_sha = file_sha256(plan_path)
    traj_path = tmp_path / "traj.json"
    _write(traj_path, _view_traj(points, view_plan_sha=plan_sha, limits_sha=limits_sha, ee_sha=ee_sha))
    return traj_path, plan_path


def test_cli_good_view_move_trajectory_exits_ok(tmp_path):
    root = _project_root(tmp_path)
    traj_path, plan_path = _cli_view_files(tmp_path, root, _ease_points())
    out_path = tmp_path / "out.check.json"

    code = main(
        ["--root", str(root), "--trajectory", str(traj_path), "--plan", str(plan_path), "--out", str(out_path)]
    )
    assert code == EXIT_OK
    report = json.loads(out_path.read_text(encoding="utf-8"))
    outcomes = {c["check"]: c["outcome"] for c in report["checks"]}
    assert outcomes["C-SCHEMA"] == "pass"
    assert outcomes["C-PROVENANCE"] == "pass"
    assert outcomes["C-PATH"] == "pass"
    assert outcomes["C-TOOLSPEED"] == "pass"


def test_cli_wrong_view_plan_sha_exits_precondition(tmp_path):
    root = _project_root(tmp_path)
    limits_sha = file_sha256(root / "config" / "robot" / "b601_dm_limits.yaml")
    ee_sha = file_sha256(root / "config" / "robot" / "end_effector.yaml")
    plan_path = tmp_path / "view_plan.json"
    _write(plan_path, _view_plan(limits_sha=limits_sha, ee_sha=ee_sha))
    traj_path = tmp_path / "traj.json"
    _write(
        traj_path,
        _view_traj(_ease_points(), view_plan_sha="0" * 64, limits_sha=limits_sha, ee_sha=ee_sha),
    )
    out_path = tmp_path / "out.check.json"

    code = main(
        ["--root", str(root), "--trajectory", str(traj_path), "--plan", str(plan_path), "--out", str(out_path)]
    )
    assert code == EXIT_PRECONDITION
    report = json.loads(out_path.read_text(encoding="utf-8"))
    by_check = {c["check"]: c["outcome"] for c in report["checks"]}
    assert by_check["C-PROVENANCE"] == "fail"

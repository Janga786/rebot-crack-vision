"""tests/test_preview_core.py — MOT-08.2 crackvision.preview_core (docs/INTERFACES.md §18).

Exercises `bind_inputs`'s §18.1 refusal rules, `compute_summary`'s hand-checked FD/tool metrics,
`tool_tip_path`'s FK parity with `crackvision.kinematics` directly, and the
`build_manifest`/`validate_manifest`/`load_manifest` round trip.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

import crackvision.kinematics as kin
from crackvision.preview_core import (
    MANIFEST_SCHEMA,
    PreviewRefusal,
    PreviewUsageError,
    bind_inputs,
    build_manifest,
    compute_summary,
    file_sha256,
    load_manifest,
    tool_tip_path,
    validate_manifest,
)
from crackvision.trajectory_check import JOINT_NAMES

LIMITS_SHA = "1" * 64
EE_SHA = "2" * 64
SCENE_SHA = "3" * 64
PLAN_SHA_PLACEHOLDER = "4" * 64

LIMITS = {
    "joints": {
        name: {"lower": -1.0, "upper": 1.0, "velocity": 1.0, "acceleration": 1.0} for name in JOINT_NAMES
    }
}


def _traj(points, purpose="test", limits_sha=LIMITS_SHA, ee_sha=EE_SHA, scene_sha=SCENE_SHA, paths3d=None):
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
        "source": {"paths3d": paths3d},
    }


def _view_traj(points, view_plan_sha, limits_sha=LIMITS_SHA, ee_sha=EE_SHA, scene_sha=SCENE_SHA):
    return {
        "schema": "crackvision.joint_trajectory/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "test",
        "purpose": "view_move",
        "planning_frame": "base_link",
        "joint_names": JOINT_NAMES,
        "points": points,
        "limits_file": {"path": "config/robot/b601_dm_limits.yaml", "sha256": limits_sha},
        "end_effector_config_sha256": ee_sha,
        "scene_config_sha256": scene_sha,
        "source": {
            "paths3d": None,
            "view_plan": {"path": "data/plans/view_plan.json", "sha256": view_plan_sha},
        },
    }


def _cartesian_plan(legs, limits_sha=LIMITS_SHA, ee_sha=EE_SHA, scene_sha=SCENE_SHA):
    return {
        "schema": "crackvision.cartesian_plan/1",
        "case_id": "case1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "plan_crack_path",
        "planning_frame": "base_link",
        "joint_names": JOINT_NAMES,
        "sources": {
            "paths3d": {"path": "data/paths/case1_paths3d.json", "sha256": PLAN_SHA_PLACEHOLDER},
            "scene": {"path": "config/scene/scene.yaml", "sha256": scene_sha},
            "end_effector": {"path": "config/robot/end_effector.yaml", "sha256": ee_sha},
            "limits": {"path": "config/robot/b601_dm_limits.yaml", "sha256": limits_sha},
            "config": {"path": "config/motion/cartesian_planning.yaml", "sha256": "c" * 64},
        },
        "parameters": {},
        "start_state": {"joint_names": JOINT_NAMES, "positions_rad": [0.0] * 6, "source": "start_joints_override"},
        "end_state": {"mode": "none", "positions_rad": None},
        "legs": legs,
        "segments": [],
        "coverage": {},
        "caveats": [],
    }


def _view_plan(limits_sha=LIMITS_SHA, ee_sha=EE_SHA, scene_sha=SCENE_SHA):
    return {
        "schema": "crackvision.view_plan/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "plan_view",
        "sources": {
            "scene": {"path": "config/scene/scene.yaml", "sha256": scene_sha},
            "end_effector": {"path": "config/robot/end_effector.yaml", "sha256": ee_sha},
            "limits": {"path": "config/robot/b601_dm_limits.yaml", "sha256": limits_sha},
            "config": {"path": "config/motion/view_plan.yaml", "sha256": "c" * 64},
        },
        "chosen": {"pose_index": 0},
        "transit_leg": {
            "leg_index": 0,
            "kind": "transit",
            "positions": [[0.0] * 6, [0.1] * 6],
        },
    }


def _retime_report(trajectory_sha256, legs):
    return {
        "schema": "crackvision.retime_report/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "trajectory_sha256": trajectory_sha256,
        "input_shas": {},
        "parameters": {},
        "legs": legs,
        "totals": {"n_points": 0, "duration_s": 0.0},
        "self_check": {"violations": 0},
    }


def _check_report(trajectory_sha256, checks):
    return {
        "schema": "crackvision.trajectory_check/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "trajectory_sha256": trajectory_sha256,
        "checks": checks,
    }


def _write(path, doc):
    path.write_text(json.dumps(doc, indent=2), encoding="utf-8")
    return path


def _write_bundle(tmp_path, *, traj, plan, retime_legs=None, checks=None):
    traj_path = _write(tmp_path / "traj.json", traj)
    plan_path = _write(tmp_path / "plan.json", plan)
    traj_sha = file_sha256(traj_path)
    if retime_legs is None:
        retime_legs = [{"leg_index": 0, "kind": "trace", "point_range": [0, 1], "duration_s": 1.0}]
    if checks is None:
        checks = [{"check": "C-SCHEMA", "outcome": "pass", "detail": "ok", "numbers": {}}]
    retime_path = _write(tmp_path / "traj.json.retime.json", _retime_report(traj_sha, retime_legs))
    check_path = _write(tmp_path / "traj.json.check.json", _check_report(traj_sha, checks))
    return traj_path, plan_path, retime_path, check_path


# ---------------------------------------------------------------------------
# bind_inputs refusals
# ---------------------------------------------------------------------------


def test_bind_inputs_happy_path_cartesian_plan(tmp_path):
    points = [
        {"t_s": 0.0, "positions": [0.0] * 6},
        {"t_s": 1.0, "positions": [0.1] * 6},
    ]
    traj = _traj(points)
    plan = _cartesian_plan([{"leg_index": 0, "kind": "trace", "positions": [[0.0] * 6, [0.1] * 6]}])
    traj_path, plan_path, retime_path, check_path = _write_bundle(tmp_path, traj=traj, plan=plan)

    bound = bind_inputs(traj_path, plan_path, retime_path, check_path, root=tmp_path)

    assert bound["trajectory"]["path"] == "traj.json"
    assert bound["plan"]["schema"] == "crackvision.cartesian_plan/1"
    assert bound["check_report"]["all_pass"] is True
    assert bound["check_report"]["failed_checks"] == []


def test_bind_inputs_accepts_view_plan(tmp_path):
    view_plan = _view_plan()
    view_plan_path = _write(tmp_path / "view_plan.json", view_plan)
    view_plan_sha = file_sha256(view_plan_path)
    points = [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 1.0, "positions": [0.1] * 6}]
    traj = _view_traj(points, view_plan_sha)
    traj_path, plan_path, retime_path, check_path = _write_bundle(
        tmp_path, traj=traj, plan=view_plan,
        retime_legs=[{"leg_index": 0, "kind": "transit", "point_range": [0, 1], "duration_s": 1.0}],
    )

    bound = bind_inputs(traj_path, plan_path, retime_path, check_path, root=tmp_path)

    assert bound["plan"]["schema"] == "crackvision.view_plan/1"
    assert bound["check_report"]["all_pass"] is True


def test_bind_inputs_refuses_invalid_trajectory(tmp_path):
    points = [{"t_s": 0.0, "positions": [0.0] * 6}]
    traj = _traj(points)
    del traj["purpose"]
    plan = _cartesian_plan([{"leg_index": 0, "kind": "trace", "positions": [[0.0] * 6]}])
    traj_path, plan_path, retime_path, check_path = _write_bundle(tmp_path, traj=traj, plan=plan)

    with pytest.raises(PreviewRefusal, match="joint_trajectory"):
        bind_inputs(traj_path, plan_path, retime_path, check_path, root=tmp_path)


def test_bind_inputs_refuses_wrong_plan_schema(tmp_path):
    points = [{"t_s": 0.0, "positions": [0.0] * 6}]
    traj = _traj(points)
    plan = _cartesian_plan([{"leg_index": 0, "kind": "trace", "positions": [[0.0] * 6]}])
    plan["schema"] = "bogus/1"
    traj_path, plan_path, retime_path, check_path = _write_bundle(tmp_path, traj=traj, plan=plan)

    with pytest.raises(PreviewRefusal, match="plan does not parse"):
        bind_inputs(traj_path, plan_path, retime_path, check_path, root=tmp_path)


def test_bind_inputs_refuses_retime_report_sha_mismatch(tmp_path):
    points = [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 1.0, "positions": [0.1] * 6}]
    traj = _traj(points)
    plan = _cartesian_plan([{"leg_index": 0, "kind": "trace", "positions": [[0.0] * 6, [0.1] * 6]}])
    traj_path = _write(tmp_path / "traj.json", traj)
    plan_path = _write(tmp_path / "plan.json", plan)
    retime_path = _write(tmp_path / "traj.json.retime.json", _retime_report("0" * 64, []))
    check_path = _write(
        tmp_path / "traj.json.check.json",
        _check_report(file_sha256(traj_path), [{"check": "C-SCHEMA", "outcome": "pass"}]),
    )

    with pytest.raises(PreviewRefusal, match="retime report"):
        bind_inputs(traj_path, plan_path, retime_path, check_path, root=tmp_path)


def test_bind_inputs_refuses_check_report_sha_mismatch(tmp_path):
    points = [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 1.0, "positions": [0.1] * 6}]
    traj = _traj(points)
    plan = _cartesian_plan([{"leg_index": 0, "kind": "trace", "positions": [[0.0] * 6, [0.1] * 6]}])
    traj_path = _write(tmp_path / "traj.json", traj)
    plan_path = _write(tmp_path / "plan.json", plan)
    traj_sha = file_sha256(traj_path)
    retime_path = _write(
        tmp_path / "traj.json.retime.json",
        _retime_report(traj_sha, [{"leg_index": 0, "kind": "trace", "point_range": [0, 1], "duration_s": 1.0}]),
    )
    check_path = _write(tmp_path / "traj.json.check.json", _check_report("0" * 64, []))

    with pytest.raises(PreviewRefusal, match="check report"):
        bind_inputs(traj_path, plan_path, retime_path, check_path, root=tmp_path)


def test_bind_inputs_reports_failing_check_without_raising(tmp_path):
    points = [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 1.0, "positions": [0.1] * 6}]
    traj = _traj(points)
    plan = _cartesian_plan([{"leg_index": 0, "kind": "trace", "positions": [[0.0] * 6, [0.1] * 6]}])
    checks = [
        {"check": "C-SCHEMA", "outcome": "pass", "detail": "ok", "numbers": {}},
        {"check": "C-LIMITS", "outcome": "fail", "detail": "exceeded", "numbers": {}},
    ]
    traj_path, plan_path, retime_path, check_path = _write_bundle(tmp_path, traj=traj, plan=plan, checks=checks)

    bound = bind_inputs(traj_path, plan_path, retime_path, check_path, root=tmp_path)

    assert bound["check_report"]["all_pass"] is False
    assert bound["check_report"]["failed_checks"] == ["C-LIMITS"]


def test_bind_inputs_usage_error_on_missing_file(tmp_path):
    with pytest.raises(PreviewUsageError):
        bind_inputs(
            tmp_path / "missing.json", tmp_path / "missing2.json",
            tmp_path / "missing3.json", tmp_path / "missing4.json", root=tmp_path,
        )


def test_bind_inputs_usage_error_on_path_outside_root(tmp_path):
    points = [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 1.0, "positions": [0.1] * 6}]
    traj = _traj(points)
    plan = _cartesian_plan([{"leg_index": 0, "kind": "trace", "positions": [[0.0] * 6, [0.1] * 6]}])
    traj_path, plan_path, retime_path, check_path = _write_bundle(tmp_path, traj=traj, plan=plan)

    with pytest.raises(PreviewUsageError):
        bind_inputs(traj_path, plan_path, retime_path, check_path, root=tmp_path / "other_root")


# ---------------------------------------------------------------------------
# tool_tip_path FK sanity (real URDF/end_effector.yaml)
# ---------------------------------------------------------------------------


def test_tool_tip_path_matches_kinematics_at_home():
    chain = kin.load_chain()
    end_effector = kin.load_end_effector()
    traj = {"points": [{"positions": [0.0] * 6, "t_s": 0.0}]}

    result = tool_tip_path(traj, chain, end_effector)

    expected = (chain.fk([0.0] * 6) @ end_effector.T_gripper_link_tool_tip)[:3, 3]
    assert result.shape == (1, 3)
    np.testing.assert_allclose(result[0], expected)


# ---------------------------------------------------------------------------
# compute_summary — hand-checked
# ---------------------------------------------------------------------------


def test_compute_summary_hand_checked():
    points = [
        {"t_s": 0.0, "positions": [0.0] * 6},
        {"t_s": 1.0, "positions": [0.2] + [0.0] * 5},
        {"t_s": 3.0, "positions": [0.5] + [0.0] * 5},
    ]
    traj = {"joint_names": JOINT_NAMES, "points": points}
    tool_xyz = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 3.0]])
    retime_report = {
        "legs": [{"leg_index": 0, "kind": "trace", "point_range": [0, 2], "duration_s": 3.0}],
    }

    summary = compute_summary(traj, LIMITS, tool_xyz, retime_report, preview_speed_scale=0.1)

    assert summary["n_points"] == 3
    assert summary["duration_s"] == pytest.approx(3.0)
    assert summary["duration_scaled_s"] == pytest.approx(30.0)

    j1 = summary["per_joint"][0]
    assert j1["max_abs_v"] == pytest.approx(0.2)
    assert j1["max_abs_a"] == pytest.approx(0.2 / 6.0)
    assert j1["min_headroom_rad"] == pytest.approx(0.5)
    assert j1["v_ratio"] == pytest.approx(0.2)
    assert j1["a_ratio"] == pytest.approx(0.2 / 6.0)

    j2 = summary["per_joint"][1]
    assert j2["max_abs_v"] == pytest.approx(0.0)
    assert j2["max_abs_a"] == pytest.approx(0.0)
    assert j2["min_headroom_rad"] == pytest.approx(1.0)

    tool = summary["tool"]
    assert tool["path_length_m"] == pytest.approx(3.0)
    assert tool["max_speed_m_s"] == pytest.approx(1.0)
    assert tool["max_speed_scaled_m_s"] == pytest.approx(0.1)
    assert tool["min_z_m"] == pytest.approx(0.0)
    assert tool["legs"] == [
        {
            "leg_index": 0,
            "kind": "trace",
            "point_range": [0, 2],
            "duration_s": pytest.approx(3.0),
            "length_m": pytest.approx(3.0),
            "mean_speed_m_s": pytest.approx(1.0),
        }
    ]
    assert summary["preview_speed_scale"] == pytest.approx(0.1)


def test_compute_summary_shared_boundary_leg_ranges():
    points = [
        {"t_s": 0.0, "positions": [0.0] * 6},
        {"t_s": 1.0, "positions": [0.1] * 6},
        {"t_s": 2.0, "positions": [0.2] * 6},
    ]
    traj = {"joint_names": JOINT_NAMES, "points": points}
    tool_xyz = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]])
    retime_report = {
        "legs": [
            {"leg_index": 0, "kind": "transit", "point_range": [0, 1], "duration_s": 1.0},
            {"leg_index": 1, "kind": "trace", "point_range": [1, 2], "duration_s": 1.0},
        ],
    }

    summary = compute_summary(traj, LIMITS, tool_xyz, retime_report, preview_speed_scale=1.0)

    legs = {leg["leg_index"]: leg for leg in summary["tool"]["legs"]}
    assert legs[0]["point_range"] == [0, 1]
    assert legs[0]["length_m"] == pytest.approx(1.0)
    assert legs[1]["point_range"] == [1, 2]
    assert legs[1]["length_m"] == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# build_manifest / validate_manifest / load_manifest
# ---------------------------------------------------------------------------


def _bound_fixture(tmp_path):
    points = [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 1.0, "positions": [0.1] * 6}]
    traj = _traj(points)
    plan = _cartesian_plan([{"leg_index": 0, "kind": "trace", "positions": [[0.0] * 6, [0.1] * 6]}])
    traj_path, plan_path, retime_path, check_path = _write_bundle(tmp_path, traj=traj, plan=plan)
    return bind_inputs(traj_path, plan_path, retime_path, check_path, root=tmp_path)


def _summary_fixture():
    points = [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 1.0, "positions": [0.1] * 6}]
    traj = {"joint_names": JOINT_NAMES, "points": points}
    tool_xyz = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    retime_report = {"legs": [{"leg_index": 0, "kind": "trace", "point_range": [0, 1], "duration_s": 1.0}]}
    return compute_summary(traj, LIMITS, tool_xyz, retime_report, preview_speed_scale=0.1)


def _artifacts():
    return [
        {"kind": "joints", "path": "data/previews/x/joints.png", "sha256": "a" * 64},
        {"kind": "toolpath", "path": "data/previews/x/toolpath.png", "sha256": "b" * 64},
        {"kind": "toolspeed", "path": "data/previews/x/toolspeed.png", "sha256": "c" * 64},
    ]


def test_build_manifest_validate_round_trip(tmp_path):
    bound = _bound_fixture(tmp_path)
    summary = _summary_fixture()

    manifest = build_manifest(
        bound, summary, _artifacts(), created_utc="2026-01-01T00:00:00Z", producer="trajectory_preview",
    )

    assert manifest["schema"] == MANIFEST_SCHEMA
    assert manifest["case_id"] == "case1"
    assert manifest["preview_speed_scale"] == pytest.approx(0.1)
    assert "preview_speed_scale" not in manifest["summary"]
    assert validate_manifest(manifest) == []

    manifest_path = tmp_path / "preview.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    loaded = load_manifest(manifest_path)
    assert loaded == manifest


def test_build_manifest_warns_on_failing_check_and_null_bindings(tmp_path):
    points = [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 1.0, "positions": [0.1] * 6}]
    traj = _traj(points, limits_sha=None, ee_sha=None, scene_sha=None)
    plan = _cartesian_plan([{"leg_index": 0, "kind": "trace", "positions": [[0.0] * 6, [0.1] * 6]}])
    checks = [
        {"check": "C-SCHEMA", "outcome": "pass", "detail": "ok", "numbers": {}},
        {"check": "C-LIMITS", "outcome": "fail", "detail": "exceeded", "numbers": {}},
    ]
    traj_path, plan_path, retime_path, check_path = _write_bundle(tmp_path, traj=traj, plan=plan, checks=checks)
    bound = bind_inputs(traj_path, plan_path, retime_path, check_path, root=tmp_path)
    summary = _summary_fixture()

    manifest = build_manifest(
        bound, summary, _artifacts(), created_utc="2026-01-01T00:00:00Z", producer="trajectory_preview",
    )

    assert manifest["bindings"] == {
        "limits_file_sha256": None,
        "end_effector_config_sha256": None,
        "scene_config_sha256": None,
        "paths3d_sha256": None,
    }
    assert any("failing check" in w for w in manifest["warnings"])
    assert sum("bindings." in w for w in manifest["warnings"]) == 4
    assert validate_manifest(manifest) == []


def test_validate_manifest_rejects_unknown_key(tmp_path):
    bound = _bound_fixture(tmp_path)
    summary = _summary_fixture()
    manifest = build_manifest(
        bound, summary, _artifacts(), created_utc="2026-01-01T00:00:00Z", producer="trajectory_preview",
    )
    manifest["unexpected"] = True

    errors = validate_manifest(manifest)
    assert any("unexpected key" in e for e in errors)


def test_validate_manifest_rejects_missing_key(tmp_path):
    bound = _bound_fixture(tmp_path)
    summary = _summary_fixture()
    manifest = build_manifest(
        bound, summary, _artifacts(), created_utc="2026-01-01T00:00:00Z", producer="trajectory_preview",
    )
    del manifest["warnings"]

    errors = validate_manifest(manifest)
    assert any("missing key" in e for e in errors)


def test_validate_manifest_rejects_bad_sha(tmp_path):
    bound = _bound_fixture(tmp_path)
    summary = _summary_fixture()
    manifest = build_manifest(
        bound, summary, _artifacts(), created_utc="2026-01-01T00:00:00Z", producer="trajectory_preview",
    )
    manifest["trajectory"]["sha256"] = "not-a-sha"

    errors = validate_manifest(manifest)
    assert any("sha256" in e for e in errors)


def test_load_manifest_raises_on_invalid(tmp_path):
    manifest_path = tmp_path / "bad.json"
    manifest_path.write_text(json.dumps({"schema": MANIFEST_SCHEMA}), encoding="utf-8")

    with pytest.raises(PreviewRefusal):
        load_manifest(manifest_path)


def test_load_manifest_raises_usage_error_on_missing_file(tmp_path):
    with pytest.raises(PreviewUsageError):
        load_manifest(tmp_path / "missing.json")

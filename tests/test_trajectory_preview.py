"""tests/test_trajectory_preview.py — MOT-08.3 crackvision.trajectory_preview CLI
(docs/INTERFACES.md §18).

Builds a small synthetic preview case (real URDF/end_effector, synthetic wide-open limits, scene
and execution configs) in `tmp_path` and drives `main([...])` in-process.
"""

from __future__ import annotations

import json

import pytest
import yaml

from crackvision.kinematics import DEFAULT_END_EFFECTOR, DEFAULT_URDF
from crackvision.preview_core import file_sha256, load_manifest, validate_manifest
from crackvision.trajectory_check import JOINT_NAMES
from crackvision.trajectory_preview import main

LIMITS_DOC = {
    "joints": {
        name: {"lower": -10.0, "upper": 10.0, "velocity": 100.0, "acceleration": 100.0}
        for name in JOINT_NAMES
    }
}

EXECUTION_DOC = {"speed_scale": {"real_default": 0.1}}

SCENE_DOC = {
    "schema": "crackvision.scene_config/1",
    "frame": "base_link",
    "objects": [
        {
            "id": "table",
            "shape": "box",
            "dimensions_m": [1.0, 1.0, 0.05],
            "pose": {"frame": "base_link", "position_m": [0.3, 0.0, -0.025], "rpy_rad": [0.0, 0.0, 0.0]},
        },
        {
            "id": "widget",
            "shape": "cylinder",
            "pose": {"frame": "base_link", "position_m": [0.1, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
        },
    ],
    "allowed_collisions": [],
}


def _write(path, doc, *, as_yaml=False):
    if as_yaml:
        path.write_text(yaml.safe_dump(doc), encoding="utf-8")
    else:
        path.write_text(json.dumps(doc, indent=2), encoding="utf-8")
    return path


def _positions(i: int) -> list[float]:
    return [0.02 * i, -0.5 + 0.01 * i, -0.5, 0.0, 0.0, 0.0]


def _traj_points(n=5):
    return [{"t_s": float(i), "positions": _positions(i)} for i in range(n)]


def _paths3d_doc():
    return {
        "schema": "crackvision.paths3d/1",
        "case_id": "case1",
        "components": [
            {
                "polylines": [
                    {
                        "points": [
                            {"row": 0, "col": 0, "u": 0, "v": 0, "valid": True, "p_base_m": [0.2, 0.0, 0.01]},
                            {"row": 0, "col": 1, "u": 1, "v": 0, "valid": True, "p_base_m": [0.21, 0.0, 0.01]},
                            {"row": 0, "col": 2, "u": 2, "v": 0, "valid": False, "p_base_m": None},
                        ]
                    }
                ]
            }
        ],
    }


def _cartesian_plan(limits_sha, ee_sha, scene_sha, paths3d_sha):
    positions = [_positions(i) for i in range(5)]
    return {
        "schema": "crackvision.cartesian_plan/1",
        "case_id": "case1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "plan_crack_path",
        "planning_frame": "base_link",
        "joint_names": JOINT_NAMES,
        "sources": {
            "paths3d": {"path": "paths3d.json", "sha256": paths3d_sha},
            "scene": {"path": "scene.yaml", "sha256": scene_sha},
            "end_effector": {"path": "end_effector.yaml", "sha256": ee_sha},
            "limits": {"path": "limits.yaml", "sha256": limits_sha},
            "config": {"path": "config/motion/cartesian_planning.yaml", "sha256": "c" * 64},
        },
        "parameters": {},
        "start_state": {"joint_names": JOINT_NAMES, "positions_rad": positions[0], "source": "start_joints_override"},
        "end_state": {"mode": "none", "positions_rad": None},
        "legs": [
            {"leg_index": 0, "kind": "transit", "positions": positions[:2]},
            {"leg_index": 1, "kind": "trace", "positions": positions[1:]},
        ],
        "segments": [],
        "coverage": {},
        "caveats": [],
    }


def _traj(limits_sha, ee_sha, scene_sha, paths3d_sha):
    return {
        "schema": "crackvision.joint_trajectory/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "retime_plan",
        "purpose": "crack_task",
        "planning_frame": "base_link",
        "joint_names": JOINT_NAMES,
        "points": _traj_points(),
        "limits_file": {"path": "limits.yaml", "sha256": limits_sha},
        "end_effector_config_sha256": ee_sha,
        "scene_config_sha256": scene_sha,
        "source": {"paths3d": {"path": "paths3d.json", "sha256": paths3d_sha}},
    }


def _retime_report(traj_sha, tol=0.05, v_c=0.01):
    return {
        "schema": "crackvision.retime_report/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "trajectory_sha256": traj_sha,
        "input_shas": {},
        "parameters": {"trace": {"speed_tolerance": tol}},
        "legs": [
            {"leg_index": 0, "kind": "transit", "point_range": [0, 1], "duration_s": 1.0},
            {
                "leg_index": 1,
                "kind": "trace",
                "point_range": [1, 4],
                "duration_s": 3.0,
                "tool_speed": {"target": 0.01, "constant": v_c, "ramp_in_m": 0.0, "ramp_out_m": 0.0, "min": 0.0, "max": 0.0},
            },
        ],
        "totals": {"n_points": 5, "duration_s": 4.0},
        "self_check": {"violations": 0},
    }


def _check_report(traj_sha, all_pass=True):
    checks = [{"check": "C-SCHEMA", "outcome": "pass", "detail": "ok", "numbers": {}}]
    if not all_pass:
        checks.append({"check": "C-LIMITS", "outcome": "fail", "detail": "exceeded", "numbers": {}})
    return {
        "schema": "crackvision.trajectory_check/1",
        "created_utc": "2026-01-01T00:00:00Z",
        "trajectory_sha256": traj_sha,
        "checks": checks,
    }


@pytest.fixture
def case(tmp_path):
    limits_path = _write(tmp_path / "limits.yaml", LIMITS_DOC, as_yaml=True)
    scene_path = _write(tmp_path / "scene.yaml", SCENE_DOC, as_yaml=True)
    execution_path = _write(tmp_path / "execution.yaml", EXECUTION_DOC, as_yaml=True)
    paths3d_path = _write(tmp_path / "paths3d.json", _paths3d_doc())

    limits_sha = file_sha256(limits_path)
    ee_sha = file_sha256(DEFAULT_END_EFFECTOR)
    scene_sha = file_sha256(scene_path)
    paths3d_sha = file_sha256(paths3d_path)

    traj = _traj(limits_sha, ee_sha, scene_sha, paths3d_sha)
    traj_path = _write(tmp_path / "traj.json", traj)
    traj_sha = file_sha256(traj_path)

    plan_path = _write(tmp_path / "plan.json", _cartesian_plan(limits_sha, ee_sha, scene_sha, paths3d_sha))
    retime_path = _write(tmp_path / "traj.json.retime.json", _retime_report(traj_sha))
    check_path = _write(tmp_path / "traj.json.check.json", _check_report(traj_sha))

    return {
        "root": tmp_path,
        "traj": traj_path,
        "plan": plan_path,
        "retime": retime_path,
        "check": check_path,
        "limits": limits_path,
        "scene": scene_path,
        "execution": execution_path,
        "out_dir": tmp_path / "previews" / f"case1_{traj_sha[:8]}",
    }


def _argv(case, out_dir=None, check_path=None, dry_run=False):
    argv = [
        "--root", str(case["root"]),
        "--trajectory", str(case["traj"]),
        "--plan", str(case["plan"]),
        "--retime-report", str(case["retime"]),
        "--check-report", str(check_path or case["check"]),
        "--limits", str(case["limits"]),
        "--end-effector-config", str(DEFAULT_END_EFFECTOR),
        "--scene-config", str(case["scene"]),
        "--execution-config", str(case["execution"]),
        "--urdf", str(DEFAULT_URDF),
        "--out-dir", str(out_dir or case["out_dir"]),
    ]
    if dry_run:
        argv.append("--dry-run")
    return argv


def test_happy_path_writes_valid_manifest_and_pngs(case):
    rc = main(_argv(case))
    assert rc == 0

    out_dir = case["out_dir"]
    for name in ("joints.png", "toolpath.png", "toolspeed.png"):
        p = out_dir / name
        assert p.is_file()
        assert p.stat().st_size > 0

    manifest = load_manifest(out_dir / "preview.json")
    assert validate_manifest(manifest) == []
    for artifact in manifest["artifacts"]:
        actual_path = case["root"] / artifact["path"]
        assert file_sha256(actual_path) == artifact["sha256"]
    assert manifest["check_report"]["all_pass"] is True


def test_rerender_is_byte_identical(case):
    out_dir = case["out_dir"]

    assert main(_argv(case)) == 0
    shas_first = {
        name: file_sha256(out_dir / name) for name in ("joints.png", "toolpath.png", "toolspeed.png")
    }

    assert main(_argv(case)) == 0
    shas_second = {
        name: file_sha256(out_dir / name) for name in ("joints.png", "toolpath.png", "toolspeed.png")
    }

    assert shas_first == shas_second


def test_dry_run_writes_nothing(case):
    out_dir = case["out_dir"]
    rc = main(_argv(case, dry_run=True))
    assert rc == 0
    assert not out_dir.exists()


def test_check_report_sha_mismatch_refuses(case, tmp_path):
    bad_check = _write(tmp_path / "bad_check.json", _check_report("0" * 64))
    out_dir = case["out_dir"]

    rc = main(_argv(case, check_path=bad_check))

    assert rc == 3
    assert not out_dir.exists()


def test_failing_check_report_renders_with_all_pass_false(case, tmp_path):
    traj_sha = file_sha256(case["traj"])
    failing_check = _write(tmp_path / "failing_check.json", _check_report(traj_sha, all_pass=False))
    out_dir = case["root"] / "previews_failing"

    rc = main(_argv(case, out_dir=out_dir, check_path=failing_check))

    assert rc == 0
    manifest = load_manifest(out_dir / "preview.json")
    assert manifest["check_report"]["all_pass"] is False
    assert manifest["check_report"]["failed_checks"] == ["C-LIMITS"]
    for name in ("joints.png", "toolpath.png", "toolspeed.png"):
        assert (out_dir / name).stat().st_size > 0

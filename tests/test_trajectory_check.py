"""tests/test_trajectory_check.py — MOT-07.6 crackvision.trajectory_check independent checker.

Exercises C-SCHEMA, C-PROVENANCE, C-LIMITS, C-MARGIN, C-REST, C-SAMPLING against hand-built
trajectories/plans, each isolating one §16.5 fault, plus the CLI's exit codes and §0 flags.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_USAGE
from crackvision.trajectory_check import (
    JOINT_NAMES,
    build_parser,
    check_limits,
    check_margin,
    check_provenance,
    check_rest,
    check_sampling,
    file_sha256,
    main,
    run_checks,
    validate_plan_schema,
    validate_trajectory_schema,
)

LIMITS_SHA = "l" * 64
EE_SHA = "e" * 64
SCENE_SHA = "s" * 64
PATHS3D = {"path": "data/paths/case1_paths3d.json", "sha256": "p" * 64, "execution_eligible": True}

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

RETIME_CFG = {"limit_fraction": {"velocity": 0.95, "acceleration": 0.95}, "sampling": {"max_joint_step_rad": 1.0, "max_period_s": 1.0}}
MARGIN_RAD = 0.03


def _traj(points, purpose="test", paths3d=PATHS3D, limits_sha=LIMITS_SHA, ee_sha=EE_SHA, scene_sha=SCENE_SHA):
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


def _plan(paths3d=None, limits_sha=LIMITS_SHA, ee_sha=EE_SHA, scene_sha=SCENE_SHA):
    return {
        "schema": "crackvision.cartesian_plan/1",
        "case_id": "case1",
        "created_utc": "2026-01-01T00:00:00Z",
        "producer": "plan_crack_path",
        "planning_frame": "base_link",
        "joint_names": JOINT_NAMES,
        "sources": {
            "paths3d": paths3d if paths3d is not None else PATHS3D,
            "scene": {"path": "config/scene/scene.yaml", "sha256": scene_sha},
            "end_effector": {"path": "config/robot/end_effector.yaml", "sha256": ee_sha},
            "limits": {"path": "config/robot/b601_dm_limits.yaml", "sha256": limits_sha},
            "config": {"path": "config/motion/cartesian_planning.yaml", "sha256": "c" * 64},
        },
        "parameters": {},
        "start_state": {"joint_names": JOINT_NAMES, "positions_rad": [0] * 6, "source": "start_joints_override"},
        "end_state": {"mode": "none", "positions_rad": None},
        "legs": [
            {
                "leg_index": 0,
                "kind": "transit",
                "segment_ref": None,
                "direction": None,
                "roll_offset_rad": None,
                "positions": [[0.0] * 6, [0.0] * 6],
                "waypoint_index_map": [None, None],
                "recheck": {
                    "max_pos_err_m": 0.0,
                    "max_ang_err_rad": 0.0,
                    "all_valid": True,
                    "within_limits": True,
                    "max_joint_step_rad": 0.0,
                },
            }
        ],
        "segments": [],
        "coverage": {},
        "caveats": [],
    }


def _point(t_s, j2=0.0):
    pos = [0.0] * 6
    pos[1] = j2
    return {"t_s": t_s, "positions": pos}


def _clean_points():
    return [
        {"t_s": 0.0, "positions": [0, 0, 0, 0, 0, 0]},
        {"t_s": 0.5, "positions": [0, -0.05, 0, 0, 0, 0]},
        {"t_s": 1.0, "positions": [0, -0.1, 0, 0, 0, 0]},
        {"t_s": 1.5, "positions": [0, -0.05, 0, 0, 0, 0]},
        {"t_s": 2.0, "positions": [0, 0.0, 0, 0, 0, 0]},
    ]


# ---------------------------------------------------------------------------
# C-SCHEMA
# ---------------------------------------------------------------------------


def test_schema_valid_trajectory_and_plan_pass():
    assert validate_trajectory_schema(_traj(_clean_points())) == []
    assert validate_plan_schema(_plan()) == []


def test_schema_non_monotone_t_s_fails():
    points = [
        {"t_s": 0.0, "positions": [0] * 6},
        {"t_s": 0.5, "positions": [0.1] * 6},
        {"t_s": 0.3, "positions": [0.2] * 6},
    ]
    errors = validate_trajectory_schema(_traj(points))
    assert any("t_s" in e for e in errors)


def test_run_checks_schema_fail_skips_downstream_checks():
    points = [
        {"t_s": 0.0, "positions": [0] * 6},
        {"t_s": 0.5, "positions": [0.1] * 6},
        {"t_s": 0.3, "positions": [0.2] * 6},
    ]
    files = {"trajectory_sha256": "x" * 64, "limits": {"sha256": LIMITS_SHA}, "end_effector": {"sha256": EE_SHA}}
    report = run_checks(
        _traj(points), _plan(), limits=LIMITS, margin_rad=MARGIN_RAD, retime_cfg=RETIME_CFG, files=files
    )
    by_check = {c["check"]: c["outcome"] for c in report["checks"]}
    assert by_check["C-SCHEMA"] == "fail"
    for name in ("C-PROVENANCE", "C-LIMITS", "C-MARGIN", "C-REST", "C-SAMPLING"):
        assert by_check[name] == "skip"


# ---------------------------------------------------------------------------
# C-PROVENANCE
# ---------------------------------------------------------------------------


def test_provenance_passes_when_everything_matches():
    files = {"trajectory_sha256": "x" * 64, "limits": {"sha256": LIMITS_SHA}, "end_effector": {"sha256": EE_SHA}}
    result = check_provenance(_traj(_clean_points()), _plan(), files)
    assert result["outcome"] == "pass"


def test_provenance_wrong_sha_fails():
    files = {"trajectory_sha256": "x" * 64, "limits": {"sha256": "wrong" * 12 + "wrong"[:4]}, "end_effector": {"sha256": EE_SHA}}
    result = check_provenance(_traj(_clean_points()), _plan(), files)
    assert result["outcome"] == "fail"
    assert "limits_file.sha256" in result["detail"]


def test_provenance_altered_paths3d_fails():
    altered = dict(PATHS3D)
    altered["sha256"] = "z" * 64
    files = {"trajectory_sha256": "x" * 64, "limits": {"sha256": LIMITS_SHA}, "end_effector": {"sha256": EE_SHA}}
    traj = _traj(_clean_points(), paths3d=PATHS3D)
    plan = _plan(paths3d=altered)
    result = check_provenance(traj, plan, files)
    assert result["outcome"] == "fail"
    assert "paths3d" in result["detail"]


# ---------------------------------------------------------------------------
# C-LIMITS
# ---------------------------------------------------------------------------


def test_limits_overspeed_segment_fails():
    points = [
        {"t_s": 0.0, "positions": [0, 0, 0, 0, 0, 0]},
        {"t_s": 0.5, "positions": [0.05, 0, 0, 0, 0, 0]},
        {"t_s": 1.0, "positions": [2.05, 0, 0, 0, 0, 0]},  # v=4.0 rad/s, exceeds joint1 limit 1.0
        {"t_s": 1.5, "positions": [2.1, 0, 0, 0, 0, 0]},
    ]
    result = check_limits(_traj(points), LIMITS)
    assert result["outcome"] == "fail"
    assert "exceeds limit" in result["detail"]


def test_limits_acceleration_spike_fails_within_velocity_bound():
    points = [
        {"t_s": 0.0, "positions": [0, 0, 0, 0, 0, 0]},
        {"t_s": 0.1, "positions": [0.09, 0, 0, 0, 0, 0]},  # v=0.9, within limit
        {"t_s": 0.2, "positions": [0.0, 0, 0, 0, 0, 0]},  # v=-0.9, within limit; accel spikes
        {"t_s": 0.3, "positions": [0.0, 0, 0, 0, 0, 0]},
    ]
    result = check_limits(_traj(points), LIMITS)
    assert result["outcome"] == "fail"
    assert "acceleration" in result["detail"]
    assert result["numbers"]["peak_velocity_fraction"]["joint1"] <= 1.0 + 1e-9


def test_limits_clean_trajectory_passes():
    result = check_limits(_traj(_clean_points()), LIMITS)
    assert result["outcome"] == "pass"
    assert set(result["numbers"]["peak_velocity_fraction"]) == set(JOINT_NAMES)
    assert set(result["numbers"]["peak_acceleration_fraction"]) == set(JOINT_NAMES)


# ---------------------------------------------------------------------------
# C-MARGIN (§11.6)
# ---------------------------------------------------------------------------


def test_margin_home_approach_example_passes():
    # §11.6: joint2 ..., -0.04, -0.02, -0.01, 0.00 at 0.10 rad/s on every segment.
    points = [
        _point(0.0, -0.04),
        _point(0.2, -0.02),
        _point(0.3, -0.01),
        _point(0.4, 0.00),
    ]
    result = check_margin(_traj(points), LIMITS, MARGIN_RAD)
    assert result["outcome"] == "pass"


def test_margin_counter_example_fails():
    # §11.6: -0.03 -> -0.02 at 0.10 rad/s, then -0.02 -> 0.00 at 0.20 rad/s: a violation,
    # though every position is within bounds.
    points = [
        _point(0.0, -0.03),
        _point(0.1, -0.02),
        _point(0.2, 0.00),
    ]
    result = check_margin(_traj(points), LIMITS, MARGIN_RAD)
    assert result["outcome"] == "fail"


def test_margin_pass_through_bound_passes():
    points = [
        _point(0.0, -0.20),
        _point(0.1, -0.01),
        _point(0.2, 0.00),
        _point(0.3, -0.01),
        _point(0.4, -0.20),
    ]
    result = check_margin(_traj(points), LIMITS, MARGIN_RAD)
    assert result["outcome"] == "pass"


def test_margin_clean_trajectory_passes():
    result = check_margin(_traj(_clean_points()), LIMITS, MARGIN_RAD)
    assert result["outcome"] == "pass"


# ---------------------------------------------------------------------------
# C-REST (§16.2 G-REST)
# ---------------------------------------------------------------------------


def test_rest_start_not_reachable_from_rest_fails():
    points = [
        {"t_s": 0.0, "positions": [0, 0, 0.0, 0, 0, 0]},
        {"t_s": 0.1, "positions": [0, 0, 1.0, 0, 0, 0]},  # v0 = 10 rad/s from rest, far too fast
        {"t_s": 0.6, "positions": [0, 0, 1.05, 0, 0, 0]},
    ]
    result = check_rest(_traj(points), LIMITS, RETIME_CFG)
    assert result["outcome"] == "fail"
    assert "rest-start" in result["detail"]


def test_rest_end_not_reachable_from_rest_fails():
    points = [
        {"t_s": 0.0, "positions": [0, 0, 0.0, 0, 0, 0]},
        {"t_s": 0.5, "positions": [0, 0, 0.05, 0, 0, 0]},
        {"t_s": 0.6, "positions": [0, 0, 1.05, 0, 0, 0]},  # v_last = 10 rad/s into the final point
    ]
    result = check_rest(_traj(points), LIMITS, RETIME_CFG)
    assert result["outcome"] == "fail"
    assert "rest-end" in result["detail"]


def test_rest_clean_trajectory_passes():
    result = check_rest(_traj(_clean_points()), LIMITS, RETIME_CFG)
    assert result["outcome"] == "pass"


def test_rest_skips_when_fewer_than_two_points():
    result = check_rest(_traj([{"t_s": 0.0, "positions": [0] * 6}]), LIMITS, RETIME_CFG)
    assert result["outcome"] == "skip"


# ---------------------------------------------------------------------------
# C-SAMPLING (§16.2 G-STEP/G-PERIOD)
# ---------------------------------------------------------------------------


def test_sampling_period_gap_fails():
    tight_cfg = {"limit_fraction": {"velocity": 0.95, "acceleration": 0.95}, "sampling": {"max_joint_step_rad": 1.0, "max_period_s": 0.02}}
    points = [
        {"t_s": 0.0, "positions": [0] * 6},
        {"t_s": 1.0, "positions": [0.001] * 6},  # 1 s gap, far over max_period_s
    ]
    result = check_sampling(_traj(points), tight_cfg)
    assert result["outcome"] == "fail"
    assert "max_period_s" in result["detail"]


def test_sampling_step_fails():
    tight_cfg = {"limit_fraction": {"velocity": 0.95, "acceleration": 0.95}, "sampling": {"max_joint_step_rad": 0.01, "max_period_s": 1.0}}
    points = [
        {"t_s": 0.0, "positions": [0] * 6},
        {"t_s": 0.1, "positions": [0.5] * 6},  # 0.5 rad step, far over max_joint_step_rad
    ]
    result = check_sampling(_traj(points), tight_cfg)
    assert result["outcome"] == "fail"
    assert "max_joint_step_rad" in result["detail"]


def test_sampling_clean_trajectory_passes():
    result = check_sampling(_traj(_clean_points()), RETIME_CFG)
    assert result["outcome"] == "pass"


def test_sampling_skips_when_fewer_than_two_points():
    result = check_sampling(_traj([{"t_s": 0.0, "positions": [0] * 6}]), RETIME_CFG)
    assert result["outcome"] == "skip"


# ---------------------------------------------------------------------------
# run_checks: fully clean trajectory passes every implemented check
# ---------------------------------------------------------------------------


def test_run_checks_clean_trajectory_passes_every_check():
    files = {"trajectory_sha256": "x" * 64, "limits": {"sha256": LIMITS_SHA}, "end_effector": {"sha256": EE_SHA}}
    report = run_checks(
        _traj(_clean_points()), _plan(), limits=LIMITS, margin_rad=MARGIN_RAD, retime_cfg=RETIME_CFG, files=files
    )
    assert report["schema"] == "crackvision.trajectory_check/1"
    assert report["trajectory_sha256"] == "x" * 64
    outcomes = {c["check"]: c["outcome"] for c in report["checks"]}
    assert outcomes == {
        "C-SCHEMA": "pass",
        "C-PROVENANCE": "pass",
        "C-LIMITS": "pass",
        "C-MARGIN": "pass",
        "C-REST": "pass",
        "C-SAMPLING": "pass",
    }
    # C-PATH/C-TOOLSPEED/C-REPORT are added by MOT-07.7; omitted here per §16.5.
    assert "C-PATH" not in outcomes
    assert "C-TOOLSPEED" not in outcomes
    assert "C-REPORT" not in outcomes


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _write(path: Path, doc) -> None:
    path.write_text(json.dumps(doc), encoding="utf-8")


def _project_root(tmp_path: Path) -> Path:
    root = tmp_path / "proj"
    (root / "config" / "robot").mkdir(parents=True)
    (root / "config" / "motion").mkdir(parents=True)
    (root / "config" / "robot" / "end_effector.yaml").write_text("tool: {}\n", encoding="utf-8")
    import yaml

    (root / "config" / "robot" / "b601_dm_limits.yaml").write_text(yaml.safe_dump(LIMITS), encoding="utf-8")
    (root / "config" / "motion" / "execution.yaml").write_text(
        yaml.safe_dump({"position_margin_rad": MARGIN_RAD}), encoding="utf-8"
    )
    (root / "config" / "motion" / "retime.yaml").write_text(yaml.safe_dump(RETIME_CFG), encoding="utf-8")
    (root / "config" / "project.yaml").write_text("schema_version: 1\n", encoding="utf-8")
    return root


def _cli_files(tmp_path: Path, root: Path, points, paths3d=PATHS3D):
    limits_sha = file_sha256(root / "config" / "robot" / "b601_dm_limits.yaml")
    ee_sha = file_sha256(root / "config" / "robot" / "end_effector.yaml")
    traj_path = tmp_path / "traj.json"
    plan_path = tmp_path / "plan.json"
    _write(traj_path, _traj(points, paths3d=paths3d, limits_sha=limits_sha, ee_sha=ee_sha))
    _write(plan_path, _plan(paths3d=paths3d, limits_sha=limits_sha, ee_sha=ee_sha))
    return traj_path, plan_path


def test_cli_help_exits_zero():
    with pytest.raises(SystemExit) as exc_info:
        build_parser().parse_args(["--help"])
    assert exc_info.value.code == 0


def test_cli_passing_trajectory_exits_ok(tmp_path):
    root = _project_root(tmp_path)
    traj_path, plan_path = _cli_files(tmp_path, root, _clean_points())
    out_path = tmp_path / "out.check.json"

    code = main(
        [
            "--root", str(root),
            "--trajectory", str(traj_path),
            "--plan", str(plan_path),
            "--out", str(out_path),
        ]
    )
    assert code == EXIT_OK
    report = json.loads(out_path.read_text(encoding="utf-8"))
    assert all(c["outcome"] in ("pass", "skip") for c in report["checks"])


def test_cli_failing_trajectory_exits_precondition(tmp_path):
    root = _project_root(tmp_path)
    points = [
        {"t_s": 0.0, "positions": [0, 0, 0, 0, 0, 0]},
        {"t_s": 0.5, "positions": [0.05, 0, 0, 0, 0, 0]},
        {"t_s": 1.0, "positions": [2.05, 0, 0, 0, 0, 0]},  # overspeed
        {"t_s": 1.5, "positions": [2.1, 0, 0, 0, 0, 0]},
    ]
    traj_path, plan_path = _cli_files(tmp_path, root, points)
    out_path = tmp_path / "out.check.json"

    code = main(
        [
            "--root", str(root),
            "--trajectory", str(traj_path),
            "--plan", str(plan_path),
            "--out", str(out_path),
        ]
    )
    assert code == EXIT_PRECONDITION
    report = json.loads(out_path.read_text(encoding="utf-8"))
    assert any(c["outcome"] == "fail" for c in report["checks"])


def test_cli_missing_trajectory_file_exits_usage(tmp_path):
    root = _project_root(tmp_path)
    _, plan_path = _cli_files(tmp_path, root, _clean_points())

    code = main(
        [
            "--root", str(root),
            "--trajectory", str(tmp_path / "does_not_exist.json"),
            "--plan", str(plan_path),
        ]
    )
    assert code == EXIT_USAGE


def test_cli_dry_run_writes_nothing_and_exits_ok(tmp_path):
    root = _project_root(tmp_path)
    traj_path, plan_path = _cli_files(tmp_path, root, _clean_points())
    out_path = tmp_path / "out.check.json"

    code = main(
        [
            "--root", str(root),
            "--trajectory", str(traj_path),
            "--plan", str(plan_path),
            "--out", str(out_path),
            "--dry-run",
        ]
    )
    assert code == EXIT_OK
    assert not out_path.exists()


def test_cli_skip_existing_leaves_existing_report_untouched(tmp_path):
    root = _project_root(tmp_path)
    traj_path, plan_path = _cli_files(tmp_path, root, _clean_points())
    out_path = tmp_path / "out.check.json"
    out_path.write_text("sentinel", encoding="utf-8")

    code = main(
        [
            "--root", str(root),
            "--trajectory", str(traj_path),
            "--plan", str(plan_path),
            "--out", str(out_path),
            "--skip-existing",
        ]
    )
    assert code == EXIT_OK
    assert out_path.read_text(encoding="utf-8") == "sentinel"

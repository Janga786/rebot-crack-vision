"""Unit tests for placement.py and recommend_placement.py (MOT-04.3). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_placement.py -q -p no:cacheprovider
"""

import math
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from crackvision_motion import placement  # noqa: E402
from crackvision_motion.cli_common import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE  # noqa: E402
from crackvision_motion.reachability_core import enumerate_targets, load_config  # noqa: E402
from crackvision_motion.reachability_map import (  # noqa: E402
    add_target,
    finalize,
    load_placement,
    new_map,
    validate_placement,
    write_map,
)
from crackvision_motion.recommend_placement import main as recommend_placement_main  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[4]
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
LIMITS_PATH = REPO_ROOT / "config" / "robot" / "b601_dm_limits.yaml"
REAL_REACHABILITY_CONFIG = REPO_ROOT / "config" / "motion" / "reachability.yaml"

_JOINTS = {"joint1": 0.1, "joint2": -0.2, "joint3": -0.3, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0}

_ROBOT_INFO = {
    "group": "arm",
    "ik_link": "gripper_tcp",
    "ik_solver": "trac_ik",
    "reach_bound_m": 0.9,
    "limits_file": "config/robot/b601_dm_limits.yaml",
    "limits_sha256": "0" * 64,
}

# --------------------------------------------------------------------------------------
# the committed synthetic fixture: an annulus (with a hole at its own centre) of reachable
# nodes near the annulus centre, margin = 1 - distance-to-centre (hand-computable), PLUS a
# second always-reachable patch offset far enough from the hole to fit the *production*
# reachability.yaml's placement.footprint_m (0.20 m, dilated by tolerance_m=0.02 -> a 0.24 m
# half-extent window) without touching the hole. The patch exists purely so the CLI-smoke
# acceptance check (recommend_placement run against the real config/motion/reachability.yaml)
# finds a feasible placement; its own margin value is not asserted anywhere.
#
# All lengths below are the original annulus/patch geometry (grid step 0.025) uniformly scaled
# by 1.2 = 0.03 / 0.025, to track the real config/motion/reachability.yaml's grid step (MOT-04.5
# runtime change 0.025 -> 0.03). Uniform scaling preserves every containment/distance relationship
# the hand-computed tests below rely on (grid-step-relative coverage, tie-breaks, the hole), so
# the same relative structure holds at the new step.
# --------------------------------------------------------------------------------------

_SCALE = 1.2  # 0.03 / 0.025 -- see comment above


def _s(v: float) -> float:
    return round(v * _SCALE, 9)


_ANNULUS_GRID_CFG = {
    "frame": "base_link",
    "boresight": {"axis_local": (1.0, 0.0, 0.0), "provenance": "prior_evidence"},
    "grid": {
        "x_m": {"min": _s(0.05), "max": _s(0.40), "step": _s(0.025)},
        "y_m": {"min": _s(-0.125), "max": _s(0.125), "step": _s(0.025)},
        "surface_z_m": (0.0,),
        "standoffs_m": (0.01,),
    },
    "orientation": {"roll_samples": 1, "tilt_deg": (0.0,), "tilt_azimuth_samples": 1},
    "ik": {"timeout_s": 0.02, "avoid_collisions": True, "seed": "neighbour", "roll_search": "best"},
    "surface_collision": {"enabled": True, "thickness_m": 0.02, "margin_m": 0.05, "allowed_links": ("base_link",)},
}

_ANNULUS_CENTER = (_s(0.15), 0.0)
_ANNULUS_R_IN = _s(0.02)
_ANNULUS_R_OUT = _s(0.08)
_ANNULUS_X = (_s(0.075), _s(0.225))
_ANNULUS_Y = (_s(-0.075), _s(0.075))

# Sized for the real config/motion/reachability.yaml: footprint_m [0.20, 0.20] dilated by
# tolerance_m 0.02 -> half-extent 0.12 (unscaled -- footprint/tolerance come from the production
# config, not the map); offset > 0.12 from _ANNULUS_CENTER so the dilated footprint never touches
# the hole, at the map's grid step (now 0.03).
_PATCH_X = (_s(0.175), _s(0.375))
_PATCH_Y = (_s(-0.10), _s(0.10))
_PATCH_MARGIN = 0.5
_BOX_EPS = 1e-9


def _in_box(x: float, y: float, x_range, y_range) -> bool:
    return x_range[0] - _BOX_EPS <= x <= x_range[1] + _BOX_EPS and y_range[0] - _BOX_EPS <= y <= y_range[1] + _BOX_EPS


def _annulus_status_and_margin(x: float, y: float):
    dist = math.hypot(x - _ANNULUS_CENTER[0], y - _ANNULUS_CENTER[1])
    if _in_box(x, y, _ANNULUS_X, _ANNULUS_Y) and _ANNULUS_R_IN <= dist <= _ANNULUS_R_OUT:
        return "reachable", round(1.0 - dist, 6)
    if _in_box(x, y, _PATCH_X, _PATCH_Y):
        return "reachable", _PATCH_MARGIN
    return "unreachable", None


def build_synthetic_map() -> dict:
    """Deterministic annulus-with-a-hole map (§7.2). Reachable iff r_in <= dist(node, centre) <=
    r_out, margin = 1 - dist. The centre node is the 'hole' (dist 0 < r_in), so the true optimum
    for a small footprint sits one ring out -- hand-computable from the distance formula. A
    disjoint always-reachable patch (see module docstring above) supports the production-config
    CLI-smoke check."""
    m = new_map(_ANNULUS_GRID_CFG, "config/motion/reachability.yaml", "a" * 64, _ROBOT_INFO, "0" * 40)
    m["created_utc"] = "2026-01-01T00:00:00Z"

    for target in enumerate_targets(_ANNULUS_GRID_CFG):
        status, margin = _annulus_status_and_margin(target.x, target.y)
        position = [target.x, target.y, target.surface_z]
        if status == "reachable":
            add_target(
                m, target, "reachable", position,
                ik_calls=1, ik_time_s=0.001,
                tilt_deg=0.0, azimuth_rad=0.0, roll_rad=0.0,
                quat_xyzw=[0.0, 0.0, 0.0, 1.0], joints=_JOINTS,
                min_joint_limit_margin_rad=margin,
                fk_position_error_m=0.0003, fk_axis_error_deg=0.1,
            )
        else:
            add_target(m, target, "unreachable", position, ik_calls=1, ik_time_s=0.001)

    finalize(m, complete=True, duration_s=12.5)
    return m


def _small_placement_cfg(footprint_m=(_s(0.04), _s(0.04)), tolerance_m=_s(0.005), yaw_candidates_rad=(0.0, math.pi / 2), top_k=5):
    return {
        "frame": "base_link",
        "placement": {
            "footprint_m": list(footprint_m),
            "tolerance_m": tolerance_m,
            "yaw_candidates_rad": list(yaw_candidates_rad),
            "top_k": top_k,
        },
    }


# --------------------------------------------------------------------------------------
# fixture regeneration (determinism)
# --------------------------------------------------------------------------------------

def test_synthetic_fixture_is_small_and_deterministic(tmp_path):
    m = build_synthetic_map()
    out = tmp_path / "synthetic_map.json"
    write_map(m, out)
    fixture = FIXTURES_DIR / "synthetic_map.json"
    assert out.read_bytes() == fixture.read_bytes()
    # The fixture also has to carry a reachable window sized for the real production
    # placement.footprint_m (see the patch region above), which alone costs ~35 KB at the
    # reachability_map/1 schema's per-target payload size -- too big to fit under 50 KB
    # together with the hand-computable annulus region, so the budget here is looser than a
    # pure hand-computed map would need.
    assert fixture.stat().st_size < 65_000


# --------------------------------------------------------------------------------------
# score_candidates / recommend on the annulus fixture: known unique optimum, tie-break, hole
# --------------------------------------------------------------------------------------

def test_recommend_known_unique_optimum():
    m = build_synthetic_map()
    result = placement.recommend(m, _small_placement_cfg(), "a" * 64, "b" * 64)

    assert result["feasible"] is True
    winner = result["placement"]
    assert winner["center_xy_m"] == [_s(0.10), 0.0]
    assert winner["yaw_rad"] == 0.0
    expected_score = round(1.0 - math.hypot(_s(0.075), _s(0.025)), 6)
    assert winner["score_min_joint_margin_rad"] == pytest.approx(expected_score)
    validate_placement(result)


def test_tie_break_prefers_smaller_abs_y_then_smaller_x():
    m = build_synthetic_map()
    cfg = _small_placement_cfg(yaw_candidates_rad=(0.0,))
    candidates = placement.score_candidates(m, cfg)

    top_score = candidates[0]["score_min_joint_margin_rad"]
    tied = [c for c in candidates if c["score_min_joint_margin_rad"] == pytest.approx(top_score)]
    assert [c["center_xy_m"] for c in tied] == [
        [_s(0.10), 0.0], [_s(0.20), 0.0], [_s(0.15), _s(-0.05)], [_s(0.15), _s(0.05)],
    ]


def test_square_footprint_ties_across_yaw_and_smaller_yaw_wins():
    m = build_synthetic_map()
    cfg = _small_placement_cfg(yaw_candidates_rad=(math.pi / 2, 0.0))
    candidates = placement.score_candidates(m, cfg)
    assert candidates[0]["center_xy_m"] == [_s(0.10), 0.0]
    assert candidates[0]["yaw_rad"] == 0.0


def test_hole_at_annulus_centre_makes_that_candidate_infeasible():
    m = build_synthetic_map()
    cfg = _small_placement_cfg(yaw_candidates_rad=(0.0,))
    candidates = placement.score_candidates(m, cfg)
    centers = [c["center_xy_m"] for c in candidates]
    assert [_s(0.15), 0.0] not in centers  # the hole itself
    assert [_s(0.125), 0.0] not in centers  # neighbour: footprint still covers the hole
    assert [_s(0.10), 0.0] in centers  # 2 steps out: footprint clears the hole


def test_alternatives_deduplicated_by_grid_step():
    m = build_synthetic_map()
    cfg = _small_placement_cfg(yaw_candidates_rad=(0.0,), top_k=3)
    result = placement.recommend(m, cfg, "a" * 64, "b" * 64)
    centers = [result["placement"]["center_xy_m"]] + [a["center_xy_m"] for a in result["alternatives"]]
    assert centers[0] == [_s(0.10), 0.0]
    # (0.20, 0.0) ties on score but is not within one grid step of the winner -> kept as an alt.
    assert [_s(0.20), 0.0] in centers
    assert len(result["alternatives"]) <= cfg["placement"]["top_k"] - 1


# --------------------------------------------------------------------------------------
# global infeasibility + max_feasible_square_m
# --------------------------------------------------------------------------------------

def _build_uniform_map(x_axis, y_axis, margin=0.5):
    grid_cfg = {
        "frame": "base_link",
        "boresight": {"axis_local": (1.0, 0.0, 0.0), "provenance": "prior_evidence"},
        "grid": {"x_m": x_axis, "y_m": y_axis, "surface_z_m": (0.0,), "standoffs_m": (0.0,)},
        "orientation": {"roll_samples": 1, "tilt_deg": (0.0,), "tilt_azimuth_samples": 1},
        "ik": {"timeout_s": 0.02, "avoid_collisions": True, "seed": "neighbour", "roll_search": "best"},
        "surface_collision": {"enabled": True, "thickness_m": 0.02, "margin_m": 0.05, "allowed_links": ("base_link",)},
    }
    m = new_map(grid_cfg, "config/motion/reachability.yaml", "c" * 64, _ROBOT_INFO, "0" * 40)
    for target in enumerate_targets(grid_cfg):
        add_target(
            m, target, "reachable", [target.x, target.y, target.surface_z],
            ik_calls=1, ik_time_s=0.001, tilt_deg=0.0, azimuth_rad=0.0, roll_rad=0.0,
            quat_xyzw=[0.0, 0.0, 0.0, 1.0], joints=_JOINTS,
            min_joint_limit_margin_rad=margin, fk_position_error_m=0.0001, fk_axis_error_deg=0.1,
        )
    finalize(m, complete=True, duration_s=1.0)
    return m


def test_recommend_infeasible_reports_max_feasible_square():
    m = _build_uniform_map({"min": 0.0, "max": 0.2, "step": 0.05}, {"min": 0.0, "max": 0.2, "step": 0.05})
    cfg = {
        "frame": "base_link",
        "placement": {"footprint_m": [0.30, 0.30], "tolerance_m": 0.0, "yaw_candidates_rad": [0.0], "top_k": 3},
    }
    result = placement.recommend(m, cfg, "a" * 64, "b" * 64)

    assert result["feasible"] is False
    assert result["placement"] is None
    assert result["alternatives"] == []
    assert result["max_feasible_square_m"] == pytest.approx(0.20)
    validate_placement(result)


def test_max_feasible_square_is_zero_when_nothing_fits_even_smallest():
    m = _build_uniform_map({"min": 0.0, "max": 0.03, "step": 0.03}, {"min": 0.0, "max": 0.03, "step": 0.03})
    cfg = {
        "frame": "base_link",
        "placement": {"footprint_m": [0.5, 0.5], "tolerance_m": 0.0, "yaw_candidates_rad": [0.0], "top_k": 3},
    }
    result = placement.recommend(m, cfg, "a" * 64, "b" * 64)
    assert result["feasible"] is False
    # The grid has only 2 samples per axis (both grid corners): any nonzero square centred on a
    # corner necessarily extends past the grid bound on the outward side, so nothing ever fits.
    assert result["max_feasible_square_m"] == pytest.approx(0.0)


# --------------------------------------------------------------------------------------
# verification_config
# --------------------------------------------------------------------------------------

def test_verification_config_is_valid_and_covers_dilated_footprint(tmp_path):
    cfg = load_config(REAL_REACHABILITY_CONFIG)
    # Centre outside the committed config's base keep-out (ADR-014): a specimen cannot sit on the robot base.
    candidate = {
        "center_xy_m": [0.30, 0.0], "surface_z_m": 0.0, "yaw_rad": 0.0,
        "footprint_m": [0.04, 0.04], "tolerance_m": 0.0125, "standoffs_m": [0.01, 0.04],
        "score_min_joint_margin_rad": 0.42,
    }
    verify_raw = placement.verification_config(candidate, cfg)

    verify_path = tmp_path / "verify.yaml"
    verify_path.write_text(yaml.safe_dump(verify_raw, sort_keys=False), encoding="utf-8")
    verify_cfg = load_config(verify_path)  # must itself be a valid reachability_config/1

    assert verify_cfg["frame"] == cfg["frame"]
    assert verify_cfg["group"] == cfg["group"]
    assert verify_cfg["ik_link"] == cfg["ik_link"]
    assert verify_cfg["grid"]["surface_z_m"] == (0.0,)
    assert verify_cfg["grid"]["standoffs_m"] == (0.01, 0.04)
    assert verify_cfg["prefilter"]["enabled"] is True

    step = cfg["grid"]["x_m"]["step"] / 2.0
    assert verify_cfg["grid"]["x_m"]["step"] == pytest.approx(step)
    assert verify_cfg["grid"]["y_m"]["step"] == pytest.approx(cfg["grid"]["y_m"]["step"] / 2.0)

    hw = hd = 0.04 / 2.0 + 0.0125
    assert verify_cfg["grid"]["x_m"]["min"] <= 0.30 - hw + 1e-9
    assert verify_cfg["grid"]["x_m"]["max"] >= 0.30 + hw - 1e-9
    assert verify_cfg["grid"]["y_m"]["min"] <= 0.0 - hd + 1e-9
    assert verify_cfg["grid"]["y_m"]["max"] >= 0.0 + hd - 1e-9


def test_verification_config_carries_the_collision_model_and_environment(tmp_path):
    cfg = load_config(REAL_REACHABILITY_CONFIG)
    candidate = {
        "center_xy_m": [0.30, 0.0], "surface_z_m": 0.04, "yaw_rad": 0.0,
        "footprint_m": [0.20, 0.20], "tolerance_m": 0.02, "standoffs_m": [0.01, 0.04],
        "score_min_joint_margin_rad": 0.3,
    }
    raw = placement.verification_config(candidate, cfg)
    path = tmp_path / "verify.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    verify_cfg = load_config(path)
    assert verify_cfg["surface_collision"] == cfg["surface_collision"]
    assert verify_cfg.get("environment") == cfg.get("environment")


def test_view_verification_config_targets_the_camera_above_the_centre(tmp_path):
    cfg = load_config(REAL_REACHABILITY_CONFIG)
    candidate = {
        "center_xy_m": [0.29, 0.03], "surface_z_m": 0.04, "yaw_rad": 0.0,
        "footprint_m": [0.20, 0.20], "tolerance_m": 0.02, "standoffs_m": [0.01, 0.04],
        "score_min_joint_margin_rad": 0.3,
    }
    raw = placement.view_verification_config(candidate, cfg)
    path = tmp_path / "view.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    view = load_config(path)
    assert view["ik_link"] == "camera_link"
    assert view["boresight"]["axis_local"] == (1.0, 0.0, 0.0)
    assert view["grid"]["standoffs_m"] == (placement.VIEW_DISTANCE_M,)
    assert view["grid"]["x_m"]["min"] == view["grid"]["x_m"]["max"] == pytest.approx(0.29)
    assert view["grid"]["y_m"]["min"] == view["grid"]["y_m"]["max"] == pytest.approx(0.03)
    assert view["orientation"]["tilt_deg"] == (0.0, 15.0)
    assert view["ik"]["roll_search"] == "first"
    assert view["surface_collision"]["margin_m"] == pytest.approx(0.12)  # whole dilated footprint


# --------------------------------------------------------------------------------------
# recommend_placement CLI (exit codes, files written, --validate-placement, --skip-existing)
# --------------------------------------------------------------------------------------

@pytest.fixture()
def fake_root(tmp_path):
    limits_dst = tmp_path / "config" / "robot" / "b601_dm_limits.yaml"
    limits_dst.parent.mkdir(parents=True)
    limits_dst.write_text(LIMITS_PATH.read_text(encoding="utf-8"), encoding="utf-8")
    return tmp_path


def _full_config_dict(x_step=0.03, y_step=0.03, footprint_m=(0.04, 0.04), tolerance_m=0.015,
                       yaw_candidates_rad=(0.0, math.pi / 2), top_k=5):
    return {
        "frame": "base_link",
        "group": "arm",
        "ik_link": "gripper_tcp",
        "boresight": {
            "axis_local": [1.0, 0.0, 0.0],
            "down_world": [0.0, 0.0, -1.0],
            "provenance": "prior_evidence",
            "source": "test fixture",
        },
        "grid": {
            "x_m": {"min": 0.0, "max": 0.30, "step": x_step},
            "y_m": {"min": -0.15, "max": 0.15, "step": y_step},
            "surface_z_m": [0.0],
            "standoffs_m": [0.01],
        },
        "orientation": {"roll_samples": 1, "tilt_deg": [0.0], "tilt_azimuth_samples": 1},
        "ik": {"timeout_s": 0.02, "avoid_collisions": True, "seed": "neighbour", "roll_search": "best"},
        "surface_collision": {"enabled": True, "thickness_m": 0.02, "margin_m": 0.05, "allowed_links": ["base_link"]},
        "prefilter": {"enabled": True},
        "placement": {
            "footprint_m": list(footprint_m),
            "yaw_candidates_rad": list(yaw_candidates_rad),
            "tolerance_m": tolerance_m,
            "top_k": top_k,
        },
        "value_status": "nominal",
        "sources": {"all": "test fixture, not measured"},
    }


def _write_yaml(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _write_synthetic_map(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    write_map(build_synthetic_map(), path)


def test_cli_missing_map_is_precondition(fake_root, tmp_path):
    config_path = tmp_path / "reachability.yaml"
    _write_yaml(config_path, _full_config_dict())
    code = recommend_placement_main([
        "--root", str(fake_root),
        "--map", str(tmp_path / "does_not_exist.json"),
        "--reachability-config", str(config_path),
        "--out", str(tmp_path / "out.yaml"),
    ])
    assert code == EXIT_PRECONDITION


def test_cli_incomplete_map_is_precondition(fake_root, tmp_path):
    m = build_synthetic_map()
    m["complete"] = False
    map_path = tmp_path / "map.json"
    write_map(m, map_path)
    config_path = tmp_path / "reachability.yaml"
    _write_yaml(config_path, _full_config_dict())

    code = recommend_placement_main([
        "--root", str(fake_root),
        "--map", str(map_path),
        "--reachability-config", str(config_path),
        "--out", str(tmp_path / "out.yaml"),
    ])
    assert code == EXIT_PRECONDITION


def test_cli_grid_step_mismatch_is_config_error(fake_root, tmp_path):
    map_path = tmp_path / "map.json"
    _write_synthetic_map(map_path)
    config_path = tmp_path / "reachability.yaml"
    _write_yaml(config_path, _full_config_dict(x_step=0.025, y_step=0.025, tolerance_m=0.03))

    code = recommend_placement_main([
        "--root", str(fake_root),
        "--map", str(map_path),
        "--reachability-config", str(config_path),
        "--out", str(tmp_path / "out.yaml"),
    ])
    assert code == EXIT_USAGE


def test_cli_feasible_run_writes_valid_placement(fake_root, tmp_path):
    map_path = tmp_path / "map.json"
    _write_synthetic_map(map_path)
    config_path = tmp_path / "reachability.yaml"
    _write_yaml(config_path, _full_config_dict())
    out_path = tmp_path / "out.yaml"

    code = recommend_placement_main([
        "--root", str(fake_root),
        "--map", str(map_path),
        "--reachability-config", str(config_path),
        "--out", str(out_path),
    ])
    assert code == EXIT_OK
    written = load_placement(out_path)
    assert written["feasible"] is True
    assert written["value_status"] == "nominal"
    assert written["placement"]["center_xy_m"] == [_s(0.10), 0.0]
    assert len(written["map_sha256"]) == 64
    assert len(written["config_sha256"]) == 64
    assert written["boresight_provenance"] == "prior_evidence"
    assert written["caveats"][: len(placement.CAVEATS)] == placement.CAVEATS
    assert written["caveats"][0] == placement.NOMINAL_DISCLAIMER  # INTERFACES §7.3 verbatim disclaimer
    assert any("Targets are poses of" in c for c in written["caveats"])
    assert out_path.read_text(encoding="utf-8").startswith("# Generated by recommend_placement")


def test_cli_infeasible_run_exits_1_but_still_writes_file(fake_root, tmp_path):
    map_path = tmp_path / "map.json"
    _write_synthetic_map(map_path)
    config_path = tmp_path / "reachability.yaml"
    _write_yaml(config_path, _full_config_dict(footprint_m=(5.0, 5.0), tolerance_m=0.015))
    out_path = tmp_path / "out.yaml"

    code = recommend_placement_main([
        "--root", str(fake_root),
        "--map", str(map_path),
        "--reachability-config", str(config_path),
        "--out", str(out_path),
    ])
    assert code == EXIT_RUNTIME
    written = load_placement(out_path)
    assert written["feasible"] is False
    assert written["placement"] is None
    assert written["max_feasible_square_m"] is not None


def test_cli_validate_placement_valid_and_invalid(fake_root, tmp_path):
    map_path = tmp_path / "map.json"
    _write_synthetic_map(map_path)
    config_path = tmp_path / "reachability.yaml"
    _write_yaml(config_path, _full_config_dict())
    out_path = tmp_path / "out.yaml"
    recommend_placement_main([
        "--root", str(fake_root), "--map", str(map_path),
        "--reachability-config", str(config_path), "--out", str(out_path),
    ])

    valid_code = recommend_placement_main(["--root", str(fake_root), "--validate-placement", str(out_path)])
    assert valid_code == EXIT_OK

    bogus_path = tmp_path / "bogus.yaml"
    bogus_path.write_text("schema: not.the.right.schema\n", encoding="utf-8")
    invalid_code = recommend_placement_main(["--root", str(fake_root), "--validate-placement", str(bogus_path)])
    assert invalid_code == EXIT_USAGE

    missing_code = recommend_placement_main(
        ["--root", str(fake_root), "--validate-placement", str(tmp_path / "nope.yaml")]
    )
    assert missing_code == EXIT_USAGE


def test_cli_skip_existing_leaves_file_untouched(fake_root, tmp_path):
    map_path = tmp_path / "map.json"
    _write_synthetic_map(map_path)
    config_path = tmp_path / "reachability.yaml"
    _write_yaml(config_path, _full_config_dict())
    out_path = tmp_path / "out.yaml"
    out_path.write_text("SENTINEL\n", encoding="utf-8")

    code = recommend_placement_main([
        "--root", str(fake_root), "--map", str(map_path), "--reachability-config", str(config_path),
        "--out", str(out_path), "--skip-existing",
    ])
    assert code == EXIT_OK
    assert out_path.read_text(encoding="utf-8") == "SENTINEL\n"


def test_cli_emit_verify_config_is_valid(fake_root, tmp_path):
    map_path = tmp_path / "map.json"
    _write_synthetic_map(map_path)
    config_path = tmp_path / "reachability.yaml"
    _write_yaml(config_path, _full_config_dict())
    out_path = tmp_path / "out.yaml"
    verify_path = tmp_path / "verify.yaml"

    code = recommend_placement_main([
        "--root", str(fake_root), "--map", str(map_path), "--reachability-config", str(config_path),
        "--out", str(out_path), "--emit-verify-config", str(verify_path),
    ])
    assert code == EXIT_OK
    verify_cfg = load_config(verify_path)
    assert verify_cfg["grid"]["x_m"]["step"] == pytest.approx(0.015)
    assert verify_cfg["grid"]["surface_z_m"] == (0.0,)

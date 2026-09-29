"""Unit tests for reachability_map (MOT-04.2). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_reachability_map.py -q -p no:cacheprovider
"""

import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from crackvision_motion.reachability_core import Target  # noqa: E402
from crackvision_motion.reachability_map import (  # noqa: E402
    MapError,
    PlacementError,
    SCHEMA,
    STATUSES,
    add_target,
    finalize,
    joint_limit_margin,
    limits_from_yaml,
    load_map,
    load_placement,
    new_map,
    validate_map,
    validate_placement,
    write_map,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
LIMITS_PATH = REPO_ROOT / "config" / "robot" / "b601_dm_limits.yaml"

_CFG = {
    "frame": "base_link",
    "boresight": {"axis_local": (1.0, 0.0, 0.0), "provenance": "prior_evidence"},
    "grid": {
        "x_m": {"min": 0.05, "max": 0.10, "step": 0.05},
        "y_m": {"min": 0.0, "max": 0.0, "step": 0.05},
        "surface_z_m": (0.0, 0.04),
        "standoffs_m": (0.01,),
    },
    "orientation": {"roll_samples": 1, "tilt_deg": (0.0,), "tilt_azimuth_samples": 1},
    "ik": {"timeout_s": 0.02, "avoid_collisions": True, "seed": "neighbour", "roll_search": "best"},
    "surface_collision": {"enabled": True, "thickness_m": 0.02, "margin_m": 0.05, "allowed_links": ("base_link",)},
}

_ROBOT_INFO = {
    "group": "arm",
    "ik_link": "gripper_tcp",
    "ik_solver": "trac_ik",
    "reach_bound_m": 0.9,
    "limits_file": "config/robot/b601_dm_limits.yaml",
    "limits_sha256": "0" * 64,
}


def _target(target_id="t0", x=0.05, y=0.0, surface_z=0.0, standoff_m=0.01):
    return Target(target_id, 0, 0, 0, x, y, surface_z, standoff_m)


def _reachable_joints(margin_ok=True):
    if margin_ok:
        return {"joint1": 0.1, "joint2": -0.2, "joint3": -0.3, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0}
    # joint1 upper limit is 2.8 (see config/robot/b601_dm_limits.yaml) -> out of range
    return {"joint1": 999.0, "joint2": -0.2, "joint3": -0.3, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0}


def _new_map():
    return new_map(_CFG, "config/motion/reachability.yaml", "a" * 64, _ROBOT_INFO, "deadbeef")


@pytest.fixture(scope="module")
def limits():
    return limits_from_yaml(LIMITS_PATH)


# --------------------------------------------------------------------------------------
# new_map / add_target / finalize
# --------------------------------------------------------------------------------------

def test_statuses_are_exactly_the_five_defined_states():
    assert STATUSES == {"reachable", "unreachable", "prefiltered", "fk_mismatch", "error"}


def test_new_map_shape():
    m = _new_map()
    assert m["schema"] == SCHEMA
    assert m["frame"] == "base_link"
    assert m["quaternion_order"] == "xyzw"
    assert m["complete"] is False
    assert m["targets"] == []
    assert m["boresight"]["axis_local"] == [1.0, 0.0, 0.0]  # tuple -> list
    assert isinstance(m["grid"]["grid"]["surface_z_m"], list)  # tuple -> list


def test_add_target_reachable_requires_quat_and_joints():
    m = _new_map()
    with pytest.raises(MapError):
        add_target(m, _target(), "reachable", [0.05, 0.0, 0.01])


def test_add_target_unknown_status_rejected():
    m = _new_map()
    with pytest.raises(MapError):
        add_target(m, _target(), "bogus_status", [0.05, 0.0, 0.01])


def test_add_target_non_reachable_has_empty_error_default():
    m = _new_map()
    add_target(m, _target(), "unreachable", [0.05, 0.0, 0.01])
    assert m["targets"][0]["error"] == ""
    assert "quat_xyzw" not in m["targets"][0]


def test_finalize_summary_counts_and_fractions():
    m = _new_map()
    add_target(m, _target("t0", surface_z=0.0), "reachable", [0.05, 0.0, 0.01],
               quat_xyzw=[0, 0, 0, 1], joints=_reachable_joints(),
               min_joint_limit_margin_rad=0.4, fk_position_error_m=0.0001, fk_axis_error_deg=0.1,
               tilt_deg=0.0, azimuth_rad=0.0, roll_rad=0.0)
    add_target(m, _target("t1", surface_z=0.0), "unreachable", [0.10, 0.0, 0.01])
    add_target(m, _target("t2", surface_z=0.04), "prefiltered", [0.05, 0.0, 0.05])
    finalize(m, complete=True, duration_s=1.5)

    assert m["complete"] is True
    assert m["duration_s"] == 1.5
    assert m["summary"]["counts_by_status"] == {"reachable": 1, "unreachable": 1, "prefiltered": 1}
    assert m["summary"]["reachable_fraction_by_surface_z"]["0"] == pytest.approx(0.5)
    assert m["summary"]["reachable_fraction_by_surface_z"]["0.04"] == pytest.approx(0.0)


# --------------------------------------------------------------------------------------
# write_map / load_map round-trip
# --------------------------------------------------------------------------------------

def test_write_load_round_trip_is_lossless(tmp_path):
    m = _new_map()
    add_target(m, _target(), "reachable", [0.05, 0.0, 0.01],
               quat_xyzw=[0, 0, 0, 1], joints=_reachable_joints(),
               min_joint_limit_margin_rad=0.4, fk_position_error_m=0.0001, fk_axis_error_deg=0.1,
               tilt_deg=0.0, azimuth_rad=0.0, roll_rad=0.0)
    finalize(m, complete=True, duration_s=0.5)

    path = tmp_path / "reachability_map.json"
    write_map(m, path)
    loaded = load_map(path, limits_path=LIMITS_PATH)
    assert loaded == m


def test_write_map_is_atomic_no_tmp_files_left(tmp_path):
    m = _new_map()
    finalize(m, complete=True, duration_s=0.1)
    path = tmp_path / "sub" / "reachability_map.json"
    write_map(m, path)
    assert path.is_file()
    leftovers = [p for p in path.parent.iterdir() if p.name != path.name]
    assert leftovers == []


def test_write_map_compact_no_indent(tmp_path):
    m = _new_map()
    finalize(m, complete=True, duration_s=0.1)
    path = tmp_path / "reachability_map.json"
    write_map(m, path)
    text = path.read_text(encoding="utf-8")
    assert "\n" not in text
    assert ", " not in text and ": " not in text


# --------------------------------------------------------------------------------------
# validate_map rejections
# --------------------------------------------------------------------------------------

def test_validate_map_rejects_wrong_schema(limits):
    m = _new_map()
    m["schema"] = "crackvision.reachability_map/999"
    with pytest.raises(MapError):
        validate_map(m, limits)


def test_validate_map_rejects_unknown_status(limits):
    m = _new_map()
    m["targets"].append({"target_id": "t0", "status": "not_a_real_status"})
    with pytest.raises(MapError):
        validate_map(m, limits)


def test_validate_map_rejects_reachable_without_quat(limits):
    m = _new_map()
    m["targets"].append({
        "target_id": "t0", "status": "reachable",
        "joints": _reachable_joints(),
    })
    with pytest.raises(MapError):
        validate_map(m, limits)


def test_validate_map_rejects_reachable_without_all_six_joints(limits):
    m = _new_map()
    incomplete_joints = _reachable_joints()
    del incomplete_joints["joint6"]
    m["targets"].append({
        "target_id": "t0", "status": "reachable",
        "quat_xyzw": [0, 0, 0, 1], "joints": incomplete_joints,
    })
    with pytest.raises(MapError):
        validate_map(m, limits)


def test_validate_map_rejects_joint_outside_limits(limits):
    m = _new_map()
    m["targets"].append({
        "target_id": "t0", "status": "reachable",
        "quat_xyzw": [0, 0, 0, 1], "joints": _reachable_joints(margin_ok=False),
    })
    with pytest.raises(MapError):
        validate_map(m, limits)


def test_validate_map_rejects_duplicate_target_ids(limits):
    m = _new_map()
    m["targets"].append({"target_id": "dup", "status": "unreachable"})
    m["targets"].append({"target_id": "dup", "status": "unreachable"})
    with pytest.raises(MapError):
        validate_map(m, limits)


def test_validate_map_accepts_valid_map(limits):
    m = _new_map()
    add_target(m, _target(), "reachable", [0.05, 0.0, 0.01],
               quat_xyzw=[0, 0, 0, 1], joints=_reachable_joints(),
               min_joint_limit_margin_rad=0.4, fk_position_error_m=0.0001, fk_axis_error_deg=0.1,
               tilt_deg=0.0, azimuth_rad=0.0, roll_rad=0.0)
    finalize(m, complete=True, duration_s=0.1)
    validate_map(m, limits)  # must not raise


# --------------------------------------------------------------------------------------
# limits_from_yaml / joint_limit_margin
# --------------------------------------------------------------------------------------

def test_limits_from_yaml_has_all_arm_joints(limits):
    for i in range(1, 7):
        assert f"joint{i}" in limits
        lower, upper = limits[f"joint{i}"]
        assert lower < upper


def test_joint_limit_margin_known_answer(limits):
    joints = {"joint1": 0.0, "joint2": -1.57, "joint3": -1.57, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0}
    margin = joint_limit_margin(joints, limits)
    # joint1 in [-2.8, 2.8] at 0.0 -> margin 2.8; joint4 in [-1.87, 1.57] at 0.0 -> margin 1.57
    # joint5 in [-1.57, 1.57] at 0.0 -> margin 1.57 (tightest so far); check the true min directly
    expected = min(
        min(q - limits[name][0], limits[name][1] - q)
        for name, q in joints.items()
    )
    assert margin == pytest.approx(expected)


def test_joint_limit_margin_negative_when_out_of_range(limits):
    joints = _reachable_joints(margin_ok=False)
    assert joint_limit_margin(joints, limits) < 0


# --------------------------------------------------------------------------------------
# specimen placement
# --------------------------------------------------------------------------------------

def _valid_placement_candidate():
    return {
        "center_xy_m": [0.10, 0.00],
        "surface_z_m": 0.04,
        "yaw_rad": 0.0,
        "footprint_m": [0.20, 0.20],
        "tolerance_m": 0.02,
        "standoffs_m": [0.01, 0.04],
        "score_min_joint_margin_rad": 0.42,
    }


def _valid_placement(feasible=True):
    d = {
        "schema": "crackvision.specimen_placement/1",
        "value_status": "nominal",
        "feasible": feasible,
        "frame": "base_link",
        "placement": _valid_placement_candidate() if feasible else None,
        "alternatives": [],
        "map_sha256": "a" * 64,
        "config_sha256": "b" * 64,
        "boresight_provenance": "prior_evidence",
        "caveats": ["Nominal recommendation only; MOT-10 measures and supersedes this file."],
        "source": "recommend_placement (MOT-04.3)",
    }
    if not feasible:
        d["max_feasible_square_m"] = 0.15
    return d


def test_validate_placement_accepts_feasible():
    validate_placement(_valid_placement(feasible=True))  # must not raise


def test_validate_placement_accepts_infeasible():
    validate_placement(_valid_placement(feasible=False))  # must not raise


def test_validate_placement_accepts_with_alternatives():
    d = _valid_placement(feasible=True)
    d["alternatives"] = [_valid_placement_candidate()]
    validate_placement(d)  # must not raise


def test_validate_placement_rejects_wrong_schema():
    d = _valid_placement()
    d["schema"] = "crackvision.specimen_placement/999"
    with pytest.raises(PlacementError):
        validate_placement(d)


def test_validate_placement_rejects_non_nominal_value_status():
    d = _valid_placement()
    d["value_status"] = "measured"
    with pytest.raises(PlacementError):
        validate_placement(d)


def test_validate_placement_rejects_feasible_with_null_placement():
    d = _valid_placement(feasible=True)
    d["placement"] = None
    with pytest.raises(PlacementError):
        validate_placement(d)


def test_validate_placement_rejects_infeasible_with_placement():
    d = _valid_placement(feasible=False)
    d["placement"] = _valid_placement_candidate()
    with pytest.raises(PlacementError):
        validate_placement(d)


def test_validate_placement_rejects_infeasible_missing_max_square():
    d = _valid_placement(feasible=False)
    del d["max_feasible_square_m"]
    with pytest.raises(PlacementError):
        validate_placement(d)


def test_validate_placement_rejects_bad_sha():
    d = _valid_placement()
    d["map_sha256"] = "not-hex"
    with pytest.raises(PlacementError):
        validate_placement(d)


def test_validate_placement_rejects_missing_caveats():
    d = _valid_placement()
    del d["caveats"]
    with pytest.raises(PlacementError):
        validate_placement(d)


def test_validate_placement_rejects_bad_candidate_shape():
    d = _valid_placement()
    del d["placement"]["footprint_m"]
    with pytest.raises(PlacementError):
        validate_placement(d)


def test_load_placement_round_trip(tmp_path):
    d = _valid_placement()
    path = tmp_path / "specimen_placement.yaml"
    path.write_text(yaml.safe_dump(d), encoding="utf-8")
    loaded = load_placement(path)
    assert loaded["feasible"] is True
    assert loaded["placement"]["center_xy_m"] == [0.10, 0.00]


def test_load_placement_rejects_invalid(tmp_path):
    d = _valid_placement()
    d["schema"] = "wrong"
    path = tmp_path / "specimen_placement.yaml"
    path.write_text(yaml.safe_dump(d), encoding="utf-8")
    with pytest.raises(PlacementError):
        load_placement(path)

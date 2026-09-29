"""Unit tests for reachability_core (MOT-04.1). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_reachability_core.py -q -p no:cacheprovider
"""

import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from crackvision_motion.reachability_core import (  # noqa: E402
    ConfigError,
    axis_values,
    config_sha256,
    enumerate_targets,
    load_config,
    orientations,
    prefiltered,
    reach_bound_from_urdf,
    target_position,
)

REPO_ROOT = Path(__file__).resolve().parents[4]


# --------------------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------------------

def _base_config() -> dict:
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
            "x_m": {"min": 0.05, "max": 0.50, "step": 0.025},
            "y_m": {"min": -0.40, "max": 0.40, "step": 0.025},
            "surface_z_m": [0.0, 0.04],
            "standoffs_m": [0.01, 0.04],
        },
        "orientation": {"roll_samples": 8, "tilt_deg": [0.0], "tilt_azimuth_samples": 4},
        "ik": {"timeout_s": 0.02, "avoid_collisions": True, "seed": "neighbour", "roll_search": "best"},
        "surface_collision": {
            "enabled": True,
            "thickness_m": 0.02,
            "margin_m": 0.05,
            "allowed_links": ["base_link", "link1"],
        },
        "prefilter": {"enabled": True},
        "placement": {
            "footprint_m": [0.20, 0.20],
            "yaw_candidates_rad": [0.0, 1.5707963267948966],
            "tolerance_m": 0.02,
            "top_k": 5,
        },
        "value_status": "nominal",
        "sources": {"grid": "test rationale"},
    }


def _write_yaml(tmp_path: Path, cfg_dict: dict) -> Path:
    path = tmp_path / "cfg.yaml"
    path.write_text(yaml.safe_dump(cfg_dict))
    return path


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# --------------------------------------------------------------------------------------
# orientations() geometry
# --------------------------------------------------------------------------------------

_SKEW_AXIS = tuple((np.array([1.0, 2.0, -1.0]) / np.linalg.norm([1.0, 2.0, -1.0])).tolist())


@pytest.mark.parametrize("axis_local", [(1.0, 0.0, 0.0), _SKEW_AXIS])
def test_orientations_axis_maps_to_d_world_and_is_a_valid_rotation(axis_local):
    cfg = {
        "boresight": {"axis_local": list(axis_local), "down_world": [0.0, 0.0, -1.0]},
        "orientation": {"roll_samples": 6, "tilt_deg": [0.0, 30.0], "tilt_azimuth_samples": 4},
    }
    orients = orientations(cfg)
    assert len(orients) == 6 * (1 + 4)  # tilt 0 -> 1 azimuth, tilt 30 -> 4 azimuths

    for o in orients:
        r = _quat_to_matrix(o.quat_xyzw)
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)
        assert abs(np.linalg.det(r) - 1.0) < 1e-9
        mapped = r @ np.array(axis_local)
        assert np.allclose(mapped, o.d_world, atol=1e-9)
        assert np.allclose(_quat_to_matrix(o.quat_xyzw), r)  # quat<->matrix round trip
        assert o.quat_xyzw[3] >= 0.0
        assert abs(np.linalg.norm(o.quat_xyzw) - 1.0) < 1e-9
        assert abs(np.linalg.norm(o.d_world) - 1.0) < 1e-9


def test_orientations_distinct_rolls_and_order():
    cfg = {
        "boresight": {"axis_local": [0.0, 0.0, 1.0], "down_world": [0.0, 0.0, -1.0]},
        "orientation": {"roll_samples": 5, "tilt_deg": [0.0, 20.0], "tilt_azimuth_samples": 3},
    }
    orients = orientations(cfg)

    # order: tilt asc, azimuth asc (within tilt), roll asc (within tilt+azimuth)
    keys = [(o.tilt_deg, o.azimuth_rad, o.roll_rad) for o in orients]
    assert keys == sorted(keys)

    by_group = {}
    for o in orients:
        by_group.setdefault((o.tilt_deg, o.azimuth_rad), []).append(tuple(np.round(o.quat_xyzw, 9)))
    for group_quats in by_group.values():
        assert len(set(group_quats)) == 5  # 5 distinct rolls -> 5 distinct quaternions


def test_orientations_antiparallel_axis_handled():
    cfg = {
        "boresight": {"axis_local": [0.0, 0.0, 1.0], "down_world": [0.0, 0.0, -1.0]},
        "orientation": {"roll_samples": 4, "tilt_deg": [0.0], "tilt_azimuth_samples": 1},
    }
    orients = orientations(cfg)
    for o in orients:
        r = _quat_to_matrix(o.quat_xyzw)
        assert np.allclose(r @ np.array([0.0, 0.0, 1.0]), o.d_world, atol=1e-9)
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)


# --------------------------------------------------------------------------------------
# target_position()
# --------------------------------------------------------------------------------------

def test_target_position_tilt_zero():
    d_world = np.array([0.0, 0.0, -1.0])
    surface = np.array([0.1, 0.2, 0.05])
    standoff = 0.04
    pos = target_position(surface, standoff, d_world)
    assert np.allclose(pos, [0.1, 0.2, 0.09])
    assert np.linalg.norm(pos - surface) == pytest.approx(standoff, abs=1e-12)


def test_target_position_tilt_30deg():
    cfg = {
        "boresight": {"axis_local": [1.0, 0.0, 0.0], "down_world": [0.0, 0.0, -1.0]},
        "orientation": {"roll_samples": 1, "tilt_deg": [30.0], "tilt_azimuth_samples": 4},
    }
    o = orientations(cfg)[0]
    surface = np.array([0.2, -0.1, 0.08])
    standoff = 0.01
    pos = target_position(surface, standoff, o.d_world)
    assert np.allclose(pos, surface - standoff * o.d_world)
    assert np.linalg.norm(pos - surface) == pytest.approx(standoff, abs=1e-9)
    assert not np.allclose(o.d_world, [0.0, 0.0, -1.0])  # actually tilted


# --------------------------------------------------------------------------------------
# axis_values() / enumerate_targets()
# --------------------------------------------------------------------------------------

def test_axis_values_counts_and_endpoints():
    vals = axis_values({"min": 0.05, "max": 0.50, "step": 0.025})
    assert len(vals) == 19
    assert vals[0] == pytest.approx(0.05)
    assert vals[-1] == pytest.approx(0.50)

    vals_y = axis_values({"min": -0.40, "max": 0.40, "step": 0.025})
    assert len(vals_y) == 33
    assert vals_y[0] == pytest.approx(-0.40)
    assert vals_y[-1] == pytest.approx(0.40)


def test_enumerate_targets_serpentine_and_deterministic_ids(tmp_path):
    cfg_dict = _base_config()
    cfg_dict["grid"]["x_m"] = {"min": 0.0, "max": 0.05, "step": 0.025}  # 3 x values
    cfg_dict["grid"]["y_m"] = {"min": 0.0, "max": 0.05, "step": 0.025}  # 3 y values
    cfg_dict["grid"]["surface_z_m"] = [0.0]
    cfg_dict["grid"]["standoffs_m"] = [0.04]
    cfg = load_config(_write_yaml(tmp_path, cfg_dict))

    targets = enumerate_targets(cfg)
    assert len(targets) == 3 * 3 * 1 * 1

    ids = [t.target_id for t in targets]
    assert len(set(ids)) == len(ids)

    for a, b in zip(targets, targets[1:]):
        dx = round(abs(a.x - b.x), 9)
        dy = round(abs(a.y - b.y), 9)
        assert (dx, dy) in {(0.0, 0.025), (0.025, 0.0)}

    targets_again = enumerate_targets(cfg)
    assert [t.target_id for t in targets_again] == ids


# --------------------------------------------------------------------------------------
# reach_bound_from_urdf()
# --------------------------------------------------------------------------------------

_SYNTHETIC_URDF = """<?xml version="1.0"?>
<robot name="synthetic">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tip_link"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.20 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j3" type="fixed">
    <parent link="link2"/>
    <child link="tip_link"/>
    <origin xyz="0 0.15 0" rpy="0 0 0"/>
  </joint>
</robot>
"""

_SYNTHETIC_URDF_PRISMATIC = """<?xml version="1.0"?>
<robot name="synthetic">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="tip_link"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="prismatic">
    <parent link="link1"/>
    <child link="tip_link"/>
    <origin xyz="0.20 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.1" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def test_reach_bound_known_answer():
    bound = reach_bound_from_urdf(_SYNTHETIC_URDF, "base_link", "tip_link")
    assert bound == pytest.approx(0.10 + 0.20 + 0.15, abs=1e-9)


def test_reach_bound_prismatic_joint_raises():
    with pytest.raises(ValueError):
        reach_bound_from_urdf(_SYNTHETIC_URDF_PRISMATIC, "base_link", "tip_link")


def test_reach_bound_broken_chain_raises():
    with pytest.raises(ValueError):
        reach_bound_from_urdf(_SYNTHETIC_URDF, "base_link", "no_such_link")


def test_prefiltered():
    assert prefiltered(np.array([1.0, 0.0, 0.0]), 0.5) is True
    assert prefiltered(np.array([0.3, 0.0, 0.0]), 0.5) is False
    assert prefiltered(np.array([0.5, 0.0, 0.0]), 0.5) is False


# --------------------------------------------------------------------------------------
# load_config() ConfigError paths
# --------------------------------------------------------------------------------------

def test_bad_step_raises(tmp_path):
    cfg = _base_config()
    cfg["grid"]["x_m"]["step"] = 0.04  # (0.50-0.05)/0.04 == 11.25, not integral
    with pytest.raises(ConfigError):
        load_config(_write_yaml(tmp_path, cfg))


def test_negative_standoff_raises(tmp_path):
    cfg = _base_config()
    cfg["grid"]["standoffs_m"] = [-0.01, 0.04]
    with pytest.raises(ConfigError):
        load_config(_write_yaml(tmp_path, cfg))


def test_roll_samples_lt_1_raises(tmp_path):
    cfg = _base_config()
    cfg["orientation"]["roll_samples"] = 0
    with pytest.raises(ConfigError):
        load_config(_write_yaml(tmp_path, cfg))


def test_unknown_top_level_key_raises(tmp_path):
    cfg = _base_config()
    cfg["bogus_extra_key"] = 1
    with pytest.raises(ConfigError):
        load_config(_write_yaml(tmp_path, cfg))


def test_footprint_nonpositive_raises(tmp_path):
    cfg = _base_config()
    cfg["placement"]["footprint_m"] = [0.0, 0.20]
    with pytest.raises(ConfigError):
        load_config(_write_yaml(tmp_path, cfg))


def test_tolerance_below_half_step_raises(tmp_path):
    cfg = _base_config()
    cfg["placement"]["tolerance_m"] = 0.001  # < max(0.025, 0.025)/2 == 0.0125
    with pytest.raises(ConfigError):
        load_config(_write_yaml(tmp_path, cfg))


def test_zero_axis_local_raises(tmp_path):
    cfg = _base_config()
    cfg["boresight"]["axis_local"] = [0.0, 0.0, 0.0]
    with pytest.raises(ConfigError):
        load_config(_write_yaml(tmp_path, cfg))


def test_zero_down_world_raises(tmp_path):
    cfg = _base_config()
    cfg["boresight"]["down_world"] = [0.0, 0.0, 0.0]
    with pytest.raises(ConfigError):
        load_config(_write_yaml(tmp_path, cfg))


def test_tilt_deg_out_of_range_raises(tmp_path):
    cfg = _base_config()
    cfg["orientation"]["tilt_deg"] = [61.0]
    with pytest.raises(ConfigError):
        load_config(_write_yaml(tmp_path, cfg))


def test_top_k_lt_1_raises(tmp_path):
    cfg = _base_config()
    cfg["placement"]["top_k"] = 0
    with pytest.raises(ConfigError):
        load_config(_write_yaml(tmp_path, cfg))


def test_unknown_nested_key_raises(tmp_path):
    cfg = _base_config()
    cfg["grid"]["x_m"]["bogus"] = 1.0
    with pytest.raises(ConfigError):
        load_config(_write_yaml(tmp_path, cfg))


def test_base_config_fixture_is_actually_valid(tmp_path):
    """Guards the negative tests above against a broken fixture giving false positives."""
    cfg = load_config(_write_yaml(tmp_path, _base_config()))
    assert cfg["value_status"] == "nominal"


# --------------------------------------------------------------------------------------
# config_sha256() and the committed config
# --------------------------------------------------------------------------------------

def test_config_sha256_is_stable_and_hex(tmp_path):
    p = tmp_path / "cfg.yaml"
    p.write_text("a: 1\n")
    h1 = config_sha256(p)
    h2 = config_sha256(p)
    assert h1 == h2
    assert len(h1) == 64
    int(h1, 16)  # valid hex


def test_load_committed_config():
    cfg = load_config(REPO_ROOT / "config" / "motion" / "reachability.yaml")
    assert cfg["frame"] == "base_link"
    assert cfg["group"] == "arm"
    assert cfg["ik_link"] == "gripper_tcp"
    assert cfg["value_status"] == "nominal"
    assert cfg["boresight"]["provenance"] == "prior_evidence"
    # grid step 0.03 (MOT-04.5 runtime change, see config/motion/reachability.yaml "Grid change"):
    # x in [0.05, 0.50] -> 16 samples, y in [-0.39, 0.39] -> 27 samples.
    assert len(axis_values(cfg["grid"]["x_m"])) == 16
    assert len(axis_values(cfg["grid"]["y_m"])) == 27

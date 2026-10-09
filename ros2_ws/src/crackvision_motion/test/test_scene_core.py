"""Unit tests for scene_core (MOT-03). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_scene_core.py -q -p no:cacheprovider
"""

import copy
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from crackvision_motion.scene_core import (  # noqa: E402
    SceneError,
    SceneNotCommissionedError,
    all_measured,
    assert_commissioning_ready,
    config_sha256,
    load_config,
    nominal_items,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
PRODUCTION_CONFIG = REPO_ROOT / "config" / "scene" / "scene.yaml"
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
COLLIDER_FIXTURE = FIXTURES_DIR / "scene_collision_smoke.yaml"
NOMINAL_FIXTURE = FIXTURES_DIR / "scene_nominal_example.yaml"

# §12.6's rectangularity_tolerance_m default (0.003 m) is the only generic tolerance constant §12
# defines; reused below as a loose stand-in so the production-table z check survives MOT-10.5
# flipping the table to `measured` (top face at z = -(adapter plate thickness), not necessarily
# exactly 0) without needing to be rewritten.
_SURVEY_TOLERANCE_M = 0.003

_VALID_RAW = {
    "schema": "crackvision.scene_config/1",
    "frame": "base_link",
    "objects": [
        {
            "id": "table",
            "shape": "box",
            "dimensions_m": [1.0, 0.8, 0.05],
            "pose": {"frame": "base_link", "position_m": [0.4, 0.0, -0.045], "rpy_rad": [0.0, 0.0, 0.0]},
            "value_status": "nominal",
            "source": "test fixture",
        },
        {
            "id": "specimen",
            "shape": "box",
            "dimensions_m": [0.2, 0.2, 0.01],
            "pose": {"frame": "base_link", "position_m": [0.3, 0.0, -0.015], "rpy_rad": [0.0, 0.0, 0.0]},
            "value_status": "nominal",
            "source": "test fixture",
        },
    ],
    "allowed_collisions": [
        {
            "link_a": "specimen",
            "link_b": "table",
            "reason": "rests on the table",
            "value_status": "nominal",
            "source": "test fixture",
        },
    ],
}


def _write(tmp_path: Path, raw: dict, name: str = "scene.yaml") -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return path


# --------------------------------------------------------------------------------------
# load_config: happy path
# --------------------------------------------------------------------------------------

def test_load_config_valid(tmp_path):
    path = _write(tmp_path, _VALID_RAW)
    config = load_config(path)
    assert config["schema"] == "crackvision.scene_config/1"
    assert config["frame"] == "base_link"
    assert [o["id"] for o in config["objects"]] == ["table", "specimen"]
    assert config["allowed_collisions"][0]["link_a"] == "specimen"


def test_load_config_production_scene_is_valid():
    config = load_config(PRODUCTION_CONFIG)
    assert {o["id"] for o in config["objects"]} == {"table", "specimen"}
    assert len(config["allowed_collisions"]) >= 1


def test_load_config_production_scene_has_no_world_camera_object():
    # ADR-014: the D405 and its mount are eye-in-hand robot geometry (config/robot/end_effector.yaml,
    # GEOM-10), attached to gripper_link, not a static world object -- a world-fixed camera/mount
    # CollisionObject here would be a second, wrong model of the same hardware.
    config = load_config(PRODUCTION_CONFIG)
    ids = {o["id"] for o in config["objects"]}
    assert "camera_mount" not in ids
    assert "camera" not in ids


def test_load_config_production_table_top_face_at_base_link_origin():
    # The robot base stands on the table: the top face sits at z = -(adapter plate thickness),
    # which is exactly 0 today (the base is bolted directly to the table, no adapter plate). A
    # loose, named tolerance is used instead of an exact-zero check so this test keeps passing
    # once MOT-10.5 flips the production table to a `measured` value from a real survey.
    config = load_config(PRODUCTION_CONFIG)
    table = next(o for o in config["objects"] if o["id"] == "table")
    top_z = table["pose"]["position_m"][2] + table["dimensions_m"][2] / 2.0
    assert top_z == pytest.approx(0.0, abs=_SURVEY_TOLERANCE_M)


def test_load_config_collider_fixture_is_valid():
    config = load_config(COLLIDER_FIXTURE)
    assert [o["id"] for o in config["objects"]] == ["collider"]


# --------------------------------------------------------------------------------------
# load_config: rejects malformed input
# --------------------------------------------------------------------------------------

def test_rejects_wrong_schema(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["schema"] = "crackvision.scene_config/2"
    with pytest.raises(SceneError, match="schema"):
        load_config(_write(tmp_path, raw))


def test_rejects_unknown_top_level_key(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["extra"] = 1
    with pytest.raises(SceneError, match="unknown key"):
        load_config(_write(tmp_path, raw))


def test_rejects_empty_objects(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["objects"] = []
    with pytest.raises(SceneError, match="non-empty list"):
        load_config(_write(tmp_path, raw))


def test_rejects_duplicate_object_id(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["objects"].append(copy.deepcopy(raw["objects"][0]))
    with pytest.raises(SceneError, match="duplicate object id"):
        load_config(_write(tmp_path, raw))


def test_rejects_unknown_shape(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["objects"][0]["shape"] = "sphere"
    with pytest.raises(SceneError, match="shape"):
        load_config(_write(tmp_path, raw))


def test_rejects_non_positive_dimensions(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["objects"][0]["dimensions_m"] = [1.0, 0.0, 0.05]
    with pytest.raises(SceneError, match="dimensions_m"):
        load_config(_write(tmp_path, raw))


def test_rejects_bad_value_status(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["objects"][0]["value_status"] = "guessed"
    with pytest.raises(SceneError, match="value_status"):
        load_config(_write(tmp_path, raw))


def test_rejects_missing_source(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    del raw["objects"][0]["source"]
    with pytest.raises(SceneError, match="source"):
        load_config(_write(tmp_path, raw))


def test_rejects_acm_self_pair(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["allowed_collisions"][0]["link_b"] = raw["allowed_collisions"][0]["link_a"]
    with pytest.raises(SceneError, match="must differ"):
        load_config(_write(tmp_path, raw))


def test_rejects_missing_acm_value_status(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    del raw["allowed_collisions"][0]["value_status"]
    with pytest.raises(SceneError, match="value_status"):
        load_config(_write(tmp_path, raw))


# --------------------------------------------------------------------------------------
# config_sha256
# --------------------------------------------------------------------------------------

def test_config_sha256_deterministic(tmp_path):
    path = _write(tmp_path, _VALID_RAW)
    assert config_sha256(path) == config_sha256(path)
    other = _write(tmp_path, {**copy.deepcopy(_VALID_RAW), "frame": "world"}, name="other.yaml")
    assert config_sha256(path) != config_sha256(other)


# --------------------------------------------------------------------------------------
# the commissioning gate — MOT-05 will import and call assert_commissioning_ready directly
# --------------------------------------------------------------------------------------

def test_all_measured_false_when_any_nominal(tmp_path):
    config = load_config(_write(tmp_path, _VALID_RAW))
    assert all_measured(config) is False
    assert nominal_items(config) == ["allowed_collision:specimen~table", "object:specimen", "object:table"]


def test_all_measured_true_when_all_measured(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    for obj in raw["objects"]:
        obj["value_status"] = "measured"
    for entry in raw["allowed_collisions"]:
        entry["value_status"] = "measured"
    config = load_config(_write(tmp_path, raw))
    assert all_measured(config) is True
    assert nominal_items(config) == []


def test_assert_commissioning_ready_raises_on_nominal_config():
    # Uses a committed nominal fixture (not the production scene) so this test is decoupled from
    # MOT-10.5 eventually flipping config/scene/scene.yaml to measured.
    config = load_config(NOMINAL_FIXTURE)
    with pytest.raises(SceneNotCommissionedError, match="nominal"):
        assert_commissioning_ready(config)


def test_assert_commissioning_ready_passes_when_all_measured(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    for obj in raw["objects"]:
        obj["value_status"] = "measured"
    for entry in raw["allowed_collisions"]:
        entry["value_status"] = "measured"
    config = load_config(_write(tmp_path, raw))
    assert_commissioning_ready(config)  # must not raise


def test_assert_commissioning_ready_raises_if_any_single_value_is_nominal(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    for obj in raw["objects"]:
        obj["value_status"] = "measured"
    for entry in raw["allowed_collisions"]:
        entry["value_status"] = "measured"
    raw["objects"][1]["value_status"] = "nominal"  # exactly one straggler
    config = load_config(_write(tmp_path, raw))
    with pytest.raises(SceneNotCommissionedError, match="object:specimen"):
        assert_commissioning_ready(config)

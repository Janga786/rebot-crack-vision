"""Unit tests for tool_fk (MOT-07.2). Runs without a colcon build, no conda env.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_tool_fk.py -q -p no:cacheprovider
"""

from __future__ import annotations

import json
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from crackvision_motion.tool_fk import JOINT_NAMES, ToolFKError, load_tool_chain  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[4]
URDF_PATH = REPO_ROOT / "presentation" / "sim" / "reBot_B601_DM_with_gripper.urdf"
END_EFFECTOR_PATH = REPO_ROOT / "config" / "robot" / "end_effector.yaml"
FIXTURE_PATH = Path(__file__).resolve().parent / "fixtures" / "retime" / "fk_reference.json"


def _quat_xyzw_to_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


@pytest.fixture(scope="module")
def fixture():
    return json.loads(FIXTURE_PATH.read_text())


@pytest.fixture(scope="module")
def chain():
    return load_tool_chain(URDF_PATH, END_EFFECTOR_PATH)


def test_shas_match_committed_files(chain, fixture):
    assert chain.urdf_sha256 == fixture["urdf"]["sha256"]
    assert chain.end_effector_sha256 == fixture["end_effector"]["sha256"]


def test_joint_names(chain):
    assert chain.joint_names == list(JOINT_NAMES)


def test_parity_against_kinematics_fixture(chain, fixture):
    for q, pose in zip(fixture["configs"], fixture["poses"]):
        T = chain.tool_pose(q)
        expected_pos = np.array(pose["position_m"])
        expected_rot = _quat_xyzw_to_matrix(pose["quat_xyzw"])
        assert np.max(np.abs(T[:3, 3] - expected_pos)) <= 1e-9
        assert np.max(np.abs(T[:3, :3] - expected_rot)) <= 1e-9


def test_tool_positions_matches_tool_pose(chain, fixture):
    Q = np.array(fixture["configs"])
    positions = chain.tool_positions(Q)
    expected = np.array([chain.tool_pose(q)[:3, 3] for q in fixture["configs"]])
    assert np.max(np.abs(positions - expected)) <= 1e-12


def test_tool_positions_is_fast_for_thousands_of_configs(chain):
    rng = np.random.default_rng(0)
    Q = rng.uniform(-1.0, 1.0, size=(4000, 6))
    start = time.monotonic()
    positions = chain.tool_positions(Q)
    elapsed = time.monotonic() - start
    assert positions.shape == (4000, 3)
    assert elapsed < 5.0, f"tool_positions took {elapsed:.2f}s for 4000 configs"


def test_missing_urdf_raises_typed_error(tmp_path):
    with pytest.raises(ToolFKError):
        load_tool_chain(tmp_path / "missing.urdf", END_EFFECTOR_PATH)


def test_missing_end_effector_config_raises_typed_error(tmp_path):
    with pytest.raises(ToolFKError):
        load_tool_chain(URDF_PATH, tmp_path / "missing.yaml")


def test_urdf_without_base_to_gripper_chain_raises_typed_error(tmp_path):
    urdf = tmp_path / "broken.urdf"
    urdf.write_text(
        '<?xml version="1.0"?>\n<robot name="broken">\n'
        '  <link name="base_link"/>\n  <link name="other_link"/>\n'
        '  <joint name="joint1" type="revolute">\n'
        '    <parent link="base_link"/><child link="other_link"/>\n'
        '    <axis xyz="0 0 1"/>\n    <limit lower="-1" upper="1" velocity="1" effort="1"/>\n'
        '  </joint>\n</robot>\n'
    )
    with pytest.raises(ToolFKError):
        load_tool_chain(urdf, END_EFFECTOR_PATH)


def test_wrong_joint_count_raises_typed_error(chain):
    with pytest.raises(ToolFKError):
        chain.tool_pose([0.0, 0.0, 0.0])
    with pytest.raises(ToolFKError):
        chain.tool_positions(np.zeros((3, 5)))


def test_end_effector_parent_link_mismatch_raises_typed_error(tmp_path):
    bad = tmp_path / "end_effector.yaml"
    text = END_EFFECTOR_PATH.read_text().replace("parent_link: gripper_link", "parent_link: wrong_link")
    bad.write_text(text)
    with pytest.raises(ToolFKError):
        load_tool_chain(URDF_PATH, bad)


def test_no_ros_or_crackvision_imports():
    src = (Path(__file__).resolve().parents[1] / "crackvision_motion" / "tool_fk.py").read_text()
    for line in src.splitlines():
        stripped = line.strip()
        if stripped.startswith("import rclpy") or stripped.startswith("from rclpy"):
            pytest.fail("tool_fk.py must not import rclpy")
        if stripped.startswith("import crackvision") and not stripped.startswith("import crackvision_"):
            pytest.fail("tool_fk.py must not import the conda-side crackvision package")
        if stripped.startswith("from crackvision ") or stripped.startswith("from crackvision."):
            pytest.fail("tool_fk.py must not import the conda-side crackvision package")

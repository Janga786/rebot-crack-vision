"""Tests for crackvision.kinematics (GEOM-08.2)."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from crackvision import kinematics as kin

VENDOR_URDF = Path.home() / "rebot_ws" / "src" / "rebotarm_bringup" / "description" / "urdf" / "reBot_B601_DM_with_gripper.urdf"


def test_rpy_convention_matches_scipy_extrinsic_xyz():
    rpy = (0.3, -0.6, 1.1)
    T = kin.transform_from_xyz_rpy((0.0, 0.0, 0.0), rpy)
    expected = Rotation.from_euler("xyz", list(rpy), degrees=False).as_matrix()
    assert np.allclose(T[:3, :3], expected, atol=1e-12)


def _parse_urdf_joint_origins(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    by_child = {}
    for j in root.findall("joint"):
        by_child[j.find("child").get("link")] = j
    origins = {}
    link = "gripper_link"
    while link != "base_link":
        j = by_child[link]
        origin_el = j.find("origin")
        xyz = tuple(float(x) for x in (origin_el.get("xyz") if origin_el is not None else "0 0 0").split())
        rpy = tuple(float(x) for x in (origin_el.get("rpy") if origin_el is not None else "0 0 0").split())
        origins[j.get("name")] = (xyz, rpy)
        link = j.find("parent").get("link")
    return origins


def test_fk_at_zero_matches_independently_parsed_origins():
    chain = kin.load_chain()
    origins = _parse_urdf_joint_origins(kin.DEFAULT_URDF)

    T_expected = np.eye(4)
    # Walk base->tip in the same order the chain does.
    ordered_names = [j.name for j in chain.joints]
    for name in ordered_names:
        xyz, rpy = origins[name]
        T_expected = T_expected @ kin.transform_from_xyz_rpy(xyz, rpy)

    T_fk = chain.fk([0.0] * 6)
    assert np.allclose(T_fk, T_expected, atol=1e-10)


def test_rotating_joint1_rotates_gripper_about_joint1_axis():
    chain = kin.load_chain()
    T0 = chain.fk([0.0] * 6)
    theta = 0.4
    T1 = chain.fk([theta, 0.0, 0.0, 0.0, 0.0, 0.0])

    j1 = chain.joints[0]
    assert j1.name == "joint1"
    origin1 = kin.transform_from_xyz_rpy(j1.xyz, j1.rpy)
    # origin1 is T_base_link1 for joint1: joint1's rotation is applied about its own axis
    # at its own origin, so in base_link the rotation axis is origin1's rotation applied to axis.
    axis_in_base = origin1[:3, :3] @ np.asarray(j1.axis, dtype=float)
    axis_in_base = axis_in_base / np.linalg.norm(axis_in_base)
    origin_pos = origin1[:3, 3]

    p0 = T0[:3, 3]
    p1 = T1[:3, 3]
    R = Rotation.from_rotvec(axis_in_base * theta).as_matrix()
    p1_expected = origin_pos + R @ (p0 - origin_pos)
    assert np.allclose(p1, p1_expected, atol=1e-8)


def test_fk_rejects_wrong_names_and_counts():
    chain = kin.load_chain()
    with pytest.raises(kin.KinematicsError):
        chain.fk([0.0] * 5)
    with pytest.raises(kin.KinematicsError):
        chain.fk([0.0] * 7)
    with pytest.raises(kin.KinematicsError):
        chain.fk({"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0, "joint5": 0.0, "wrong": 0.0})


def test_fk_rejects_limit_violation():
    chain = kin.load_chain()
    j1 = next(j for j in chain.joints if j.name == "joint1")
    with pytest.raises(kin.KinematicsError):
        chain.fk([j1.upper + 0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    # within tolerance should not raise
    chain.fk([j1.upper + 1e-4, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_fk_rejects_non_finite():
    chain = kin.load_chain()
    with pytest.raises(kin.KinematicsError):
        chain.fk([float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0])


@pytest.mark.skipif(not VENDOR_URDF.is_file(), reason="~/rebot_ws vendor URDF not present")
def test_fk_matches_vendor_urdf_for_seeded_q():
    chain_copy = kin.load_chain()
    chain_vendor = kin.load_chain(urdf_path=VENDOR_URDF)
    rng = np.random.default_rng(42)
    limits = [(j.lower, j.upper) for j in chain_copy.joints if j.type != "fixed"]
    for _ in range(10):
        q = [rng.uniform(lo, hi) for lo, hi in limits]
        T_copy = chain_copy.fk(q)
        T_vendor = chain_vendor.fk(q)
        assert np.allclose(T_copy, T_vendor, atol=1e-9)


def test_load_end_effector_real_yaml():
    ee = kin.load_end_effector()
    with open(kin.DEFAULT_END_EFFECTOR) as f:
        raw = yaml.safe_load(f)

    expected_axis = np.array(raw["wrist_camera"]["derived"]["optical_axis_in_parent"])
    actual_axis = ee.T_gripper_link_camera_link[:3, :3] @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(actual_axis, expected_axis, atol=1e-4)

    expected_dist = raw["wrist_camera"]["derived"]["optical_centre_to_tool_tip_m"]
    actual_dist = np.linalg.norm(
        ee.T_gripper_link_camera_link[:3, 3] - ee.T_gripper_link_tool_tip[:3, 3]
    )
    assert abs(actual_dist - expected_dist) < 1e-3

    assert ee.wrist_camera_value_status == "nominal"
    assert ee.tool_value_status == "nominal"
    assert ee.camera_sigma_pos_m == pytest.approx(0.010)
    assert ee.camera_sigma_rot_rad == pytest.approx(math.radians(5.0))
    assert ee.tool_sigma_pos_m == pytest.approx(0.005)


def test_load_end_effector_measured_requires_uncertainty(tmp_path):
    base = {
        "schema": "crackvision.end_effector/1",
        "parent_link": "gripper_link",
        "tool": {
            "frame": "tool_tip",
            "xyz_m": [0.0, 0.0, 0.0],
            "rpy_rad": [0.0, 0.0, 0.0],
            "value_status": "measured",
        },
        "wrist_camera": {
            "frame": "camera_link",
            "xyz_m": [0.0, 0.0, 0.0],
            "rpy_rad": [0.0, 0.0, 0.0],
            "value_status": "nominal",
        },
    }
    p = tmp_path / "ee.yaml"
    p.write_text(yaml.safe_dump(base))
    with pytest.raises(kin.KinematicsError):
        kin.load_end_effector(p)

    base["tool"]["uncertainty"] = {"position_sigma_m": 0.001, "source": "test"}
    p.write_text(yaml.safe_dump(base))
    ee = kin.load_end_effector(p)
    assert ee.tool_sigma_pos_m == pytest.approx(0.001)
    assert ee.tool_value_status == "measured"

    base["wrist_camera"]["value_status"] = "measured"
    p.write_text(yaml.safe_dump(base))
    with pytest.raises(kin.KinematicsError):
        kin.load_end_effector(p)

    base["wrist_camera"]["uncertainty"] = {
        "position_sigma_m": 0.002,
        "rotation_sigma_rad": 0.01,
        "source": "test",
    }
    p.write_text(yaml.safe_dump(base))
    ee = kin.load_end_effector(p)
    assert ee.camera_sigma_pos_m == pytest.approx(0.002)
    assert ee.camera_sigma_rot_rad == pytest.approx(0.01)


def test_nominal_optical_axis_mapping():
    T = kin.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME
    R = T[:3, :3]
    assert np.allclose(R @ np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]), atol=1e-9)
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), np.array([0.0, -1.0, 0.0]), atol=1e-9)
    assert np.allclose(R @ np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, -1.0]), atol=1e-9)

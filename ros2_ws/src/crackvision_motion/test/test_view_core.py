"""Unit tests for view_core (MOT-06.8). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_view_core.py -q -p no:cacheprovider
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from crackvision_motion.cartesian_core import load_config  # noqa: E402
from crackvision_motion.view_core import (  # noqa: E402
    Pose,
    ViewCoreError,
    fov_check,
    specimen_frame,
    view_candidates,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
PRODUCTION_CONFIG = REPO_ROOT / "config" / "motion" / "cartesian_planning.yaml"
PRODUCTION_SCENE = REPO_ROOT / "config" / "scene" / "scene.yaml"


def _no_ros_imports():
    import ast

    import crackvision_motion.view_core as mod

    tree = ast.parse(Path(mod.__file__).read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names = [n.name for n in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or ""]
        else:
            continue
        for name in names:
            if name.startswith("rclpy") or "_msgs" in name:
                raise AssertionError(f"ROS import found in view_core.py: {name!r}")


def test_module_has_no_ros_imports():
    _no_ros_imports()


# --------------------------------------------------------------------------------------
# specimen_frame
# --------------------------------------------------------------------------------------

def test_specimen_frame_explicit_center_axis_aligned():
    frame = specimen_frame(center=(0.3, 0.1), footprint_m=(0.2, 0.2), surface_z_m=0.05)
    assert frame.centre_top_m == (0.3, 0.1, 0.05)
    assert frame.normal_out == (0.0, 0.0, 1.0)
    assert len(frame.corners_m) == 4
    xs = sorted({round(c[0], 9) for c in frame.corners_m})
    ys = sorted({round(c[1], 9) for c in frame.corners_m})
    assert xs == [0.2, 0.4]
    assert ys == [0.0, 0.2]
    for c in frame.corners_m:
        assert c[2] == pytest.approx(0.05)


def test_specimen_frame_center_requires_footprint_and_surface_z():
    with pytest.raises(ViewCoreError):
        specimen_frame(center=(0.3, 0.1))


def test_specimen_frame_missing_specimen_without_center_raises():
    with pytest.raises(ViewCoreError):
        specimen_frame(scene_cfg={"objects": []})


def test_specimen_frame_from_scene_axis_aligned():
    scene_cfg = yaml.safe_load(PRODUCTION_SCENE.read_text())
    specimen = next(o for o in scene_cfg["objects"] if o["id"] == "specimen")
    frame = specimen_frame(scene_cfg=scene_cfg)
    expected_top_z = specimen["pose"]["position_m"][2] + specimen["dimensions_m"][2] / 2.0
    assert frame.centre_top_m == pytest.approx(
        (specimen["pose"]["position_m"][0], specimen["pose"]["position_m"][1], expected_top_z)
    )
    assert frame.normal_out == pytest.approx((0.0, 0.0, 1.0))
    assert len(frame.corners_m) == 4
    half_x = specimen["dimensions_m"][0] / 2.0
    half_y = specimen["dimensions_m"][1] / 2.0
    cx, cy = specimen["pose"]["position_m"][0], specimen["pose"]["position_m"][1]
    expected_xs = sorted({round(cx - half_x, 9), round(cx + half_x, 9)})
    expected_ys = sorted({round(cy - half_y, 9), round(cy + half_y, 9)})
    assert sorted({round(c[0], 9) for c in frame.corners_m}) == expected_xs
    assert sorted({round(c[1], 9) for c in frame.corners_m}) == expected_ys


def test_specimen_frame_center_takes_precedence_over_scene():
    scene_cfg = yaml.safe_load(PRODUCTION_SCENE.read_text())
    frame = specimen_frame(
        scene_cfg=scene_cfg, center=(1.0, 2.0), footprint_m=(0.4, 0.4), surface_z_m=0.5,
    )
    assert frame.centre_top_m == (1.0, 2.0, 0.5)


def test_specimen_frame_rotated_specimen_rotates_footprint_and_normal():
    yaw = math.radians(45.0)
    scene_cfg = {
        "objects": [
            {
                "id": "specimen",
                "dimensions_m": [0.2, 0.1, 0.02],
                "pose": {"position_m": [0.3, 0.0, 0.01], "rpy_rad": [0.0, 0.0, yaw]},
            }
        ]
    }
    frame = specimen_frame(scene_cfg=scene_cfg)
    assert frame.centre_top_m == pytest.approx((0.3, 0.0, 0.02))
    assert frame.normal_out == pytest.approx((0.0, 0.0, 1.0))
    # A corner that would be axis-aligned at (0.3 + 0.1, 0.0 + 0.05, 0.02) is rotated 45 deg about z.
    half = np.array([0.1, 0.05])
    c = math.cos(yaw)
    s = math.sin(yaw)
    expected = (0.3 + c * half[0] - s * half[1], 0.0 + s * half[0] + c * half[1], 0.02)
    assert any(
        all(abs(a - b) < 1e-9 for a, b in zip(corner, expected)) for corner in frame.corners_m
    )


# --------------------------------------------------------------------------------------
# view_candidates
# --------------------------------------------------------------------------------------

def _cfg():
    return load_config(PRODUCTION_CONFIG)


def test_view_candidates_count_and_order():
    cfg = _cfg()
    cands = view_candidates(cfg, (0.3, 0.0, 0.01), (0.0, 0.0, 1.0))
    n_tilt = len(cfg["view"]["tilt_candidates_deg"])
    n_yaw = len(cfg["view"]["yaw_candidates_deg"])
    assert len(cands) == n_tilt * n_yaw

    tilts = [c.tilt_deg for c in cands]
    assert tilts == sorted(tilts)
    # within each tilt block, yaw is ascending.
    for i in range(0, len(cands), n_yaw):
        block_yaws = [c.yaw_deg for c in cands[i:i + n_yaw]]
        assert block_yaws == sorted(block_yaws)


def test_view_candidate_tilt_zero_optical_axis_is_minus_normal_and_standoff():
    cfg = _cfg()
    centre = (0.3, 0.0, 0.01)
    normal_out = (0.0, 0.0, 1.0)
    cands = view_candidates(cfg, centre, normal_out)
    zero_tilt = [c for c in cands if c.tilt_deg == 0.0]
    assert zero_tilt

    for cand in zero_tilt:
        x_axis, _, _ = _axes_from_quat(cand.pose.quat_xyzw)
        assert x_axis == pytest.approx((-normal_out[0], -normal_out[1], -normal_out[2]), abs=1e-9)
        # Camera sits view_distance_m along +n from the centre.
        expected_position = np.asarray(centre) + cfg["view"]["view_distance_m"] * np.asarray(normal_out)
        assert np.asarray(cand.pose.position_m) == pytest.approx(expected_position, abs=1e-9)


def test_view_candidate_tilt_15_angle_and_passes_through_centre():
    cfg = _cfg()
    centre = np.array([0.3, 0.0, 0.01])
    normal_out = np.array([0.0, 0.0, 1.0])
    cands = view_candidates(cfg, centre, normal_out)
    tilt15 = [c for c in cands if c.tilt_deg == 15.0]
    assert tilt15

    for cand in tilt15:
        x_axis, _, _ = _axes_from_quat(cand.pose.quat_xyzw)
        minus_n = -normal_out
        cos_angle = float(np.dot(x_axis, minus_n))
        angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))
        assert angle_deg == pytest.approx(15.0, abs=1e-9)

        # The optical axis ray from the camera position, travelled view_distance_m along +X,
        # reaches the centre exactly.
        position = np.asarray(cand.pose.position_m)
        reached = position + cfg["view"]["view_distance_m"] * np.asarray(x_axis)
        assert reached == pytest.approx(centre, abs=1e-9)


def _axes_from_quat(q):
    from crackvision_motion.view_core import _axes_from_quat as _impl

    return _impl(q)


# --------------------------------------------------------------------------------------
# fov_check
# --------------------------------------------------------------------------------------

def _straight_down_pose(cfg, centre):
    cands = view_candidates(cfg, centre, (0.0, 0.0, 1.0))
    return next(c.pose for c in cands if c.tilt_deg == 0.0)


def test_fov_check_passes_for_nominal_footprint_at_tilt_zero():
    cfg = _cfg()
    centre = (0.3, 0.0, 0.01)
    pose = _straight_down_pose(cfg, centre)
    footprint = cfg["view"]["footprint_m"]
    z_top = centre[2]
    half_x, half_y = footprint[0] / 2.0, footprint[1] / 2.0
    corners = [
        (centre[0] + sx * half_x, centre[1] + sy * half_y, z_top)
        for sx, sy in ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0))
    ]
    ok, details = fov_check(pose, corners, cfg)
    assert ok is True
    assert len(details["horizontal_angles_deg"]) == 4
    assert len(details["vertical_angles_deg"]) == 4
    assert all(m >= 0.0 for m in details["horizontal_margins_deg"])
    assert all(m >= 0.0 for m in details["vertical_margins_deg"])


def test_fov_check_fails_for_oversized_footprint_at_tilt_zero():
    cfg = _cfg()
    centre = (0.3, 0.0, 0.01)
    pose = _straight_down_pose(cfg, centre)
    z_top = centre[2]
    half = 1.0  # far larger than the 0.25 m standoff can cover
    corners = [
        (centre[0] + sx * half, centre[1] + sy * half, z_top)
        for sx, sy in ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0))
    ]
    ok, details = fov_check(pose, corners, cfg)
    assert ok is False
    assert any(m < 0.0 for m in details["horizontal_margins_deg"] + details["vertical_margins_deg"])


def test_fov_check_reports_per_corner_angles_matching_manual_projection():
    cfg = _cfg()
    centre = (0.3, 0.0, 0.01)
    pose = _straight_down_pose(cfg, centre)
    corner = (centre[0] + 0.05, centre[1], centre[2])
    ok, details = fov_check(pose, [corner], cfg)
    assert ok is True
    v = np.asarray(corner) - np.asarray(pose.position_m)
    x_axis, y_axis, z_axis = _axes_from_quat(pose.quat_xyzw)
    expected_h = math.degrees(math.atan2(float(np.dot(v, y_axis)), float(np.dot(v, x_axis))))
    expected_v = math.degrees(math.atan2(float(np.dot(v, z_axis)), float(np.dot(v, x_axis))))
    assert details["horizontal_angles_deg"][0] == pytest.approx(expected_h)
    assert details["vertical_angles_deg"][0] == pytest.approx(expected_v)

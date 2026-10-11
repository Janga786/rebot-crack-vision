"""tests/test_uncertainty.py — GEOM-09.2.1 analytic pixel/depth/handeye/tcp/fk decomposition."""

from __future__ import annotations

import numpy as np
import pytest

from crackvision import kinematics
from crackvision.geometry import Intrinsics, depth_uncertainty_m
from crackvision.lift3d import CalibSigmas, LiftParams, lift_polyline
from crackvision.tool_waypoints import WaypointParams, segment_waypoints
from crackvision.uncertainty import (
    decompose_point,
    decompose_waypoint,
    fk_covariance,
    measured_source_index,
    waypoint_source_indices,
)

SCALE = 1e-4


def make_intrinsics(**overrides) -> Intrinsics:
    intr = Intrinsics(
        width=640, height=480, fx=500.0, fy=500.0, ppx=320.0, ppy=240.0,
        model="none", coeffs=(0.0, 0.0, 0.0, 0.0, 0.0),
    )
    if overrides:
        from dataclasses import replace

        intr = replace(intr, **overrides)
    return intr


def _flat_depth_image(shape, depth_m, scale=SCALE):
    counts = int(round(depth_m / scale))
    return np.full(shape, counts, dtype=np.uint16)


# --- 1. decompose_point reproduces cov_base and isolates each term ---------------------


def test_decompose_point_reproduces_cov_base_from_real_lift3d():
    intr = make_intrinsics()
    depth_u16 = _flat_depth_image((100, 100), 0.25)
    mask = np.zeros((100, 100), dtype=bool)
    points_rc = [(50, c) for c in range(40, 50)]
    calib = CalibSigmas(cam_pos_m=0.01, cam_rot_rad=np.radians(5.0))
    params = LiftParams()
    lifted = lift_polyline(points_rc, depth_u16, mask, intr, SCALE, np.eye(4), calib, params)
    assert lifted.valid.all()
    assert not lifted.interpolated.any()

    i = 3
    breakdown = decompose_point(
        lifted.p_optical[i], intr.fx, intr.fy, np.eye(4), params.pixel_sigma_px, calib,
        valid=bool(lifted.valid[i]), interpolated=bool(lifted.interpolated[i]),
        cov_base=lifted.cov_base[i],
    )
    total = breakdown.pixel + breakdown.depth + breakdown.handeye
    assert np.allclose(total, lifted.cov_base[i], atol=1e-14)
    assert np.allclose(breakdown.total, lifted.cov_base[i], atol=1e-14)
    assert np.allclose(breakdown.fk, np.zeros((3, 3)))


def test_zero_sigma_px_removes_only_pixel_term():
    intr = make_intrinsics()
    depth_u16 = _flat_depth_image((100, 100), 0.25)
    mask = np.zeros((100, 100), dtype=bool)
    points_rc = [(50, 45)]
    calib = CalibSigmas(cam_pos_m=0.01, cam_rot_rad=np.radians(5.0))

    bd_px = decompose_point(
        [0.01, 0.02, 0.25], intr.fx, intr.fy, np.eye(4), 1.0, calib,
        valid=True, interpolated=False, cov_base=np.eye(3),
    )
    bd_nopx = decompose_point(
        [0.01, 0.02, 0.25], intr.fx, intr.fy, np.eye(4), 0.0, calib,
        valid=True, interpolated=False, cov_base=np.eye(3),
    )
    assert np.allclose(bd_nopx.pixel, np.zeros((3, 3)))
    assert not np.allclose(bd_px.pixel, np.zeros((3, 3)))
    assert np.allclose(bd_px.depth, bd_nopx.depth)
    assert np.allclose(bd_px.handeye, bd_nopx.handeye)


def test_pure_depth_term_matches_closed_form():
    intr = make_intrinsics()
    p_opt = np.array([0.01, 0.02, 0.25])
    x, y, z = p_opt
    calib = CalibSigmas(cam_pos_m=0.0, cam_rot_rad=0.0)
    R = np.eye(3)

    bd = decompose_point(
        p_opt, intr.fx, intr.fy, np.eye(4), 0.0, calib,
        valid=True, interpolated=False, cov_base=np.eye(3),
    )
    sigma_z = float(depth_uncertainty_m(z, intr.fx))
    J = np.array([[z / intr.fx, 0.0, x / z], [0.0, z / intr.fy, y / z], [0.0, 0.0, 1.0]])
    expected = R @ (J @ np.diag([0.0, 0.0, sigma_z**2]) @ J.T) @ R.T
    assert np.allclose(bd.depth, expected, atol=1e-18)
    assert np.allclose(bd.pixel, np.zeros((3, 3)))
    assert np.allclose(bd.handeye, np.zeros((3, 3)))


def test_handeye_closed_form_and_zero_when_calib_zero():
    intr = make_intrinsics()
    p_opt = np.array([0.01, 0.02, 0.25])
    calib_zero = CalibSigmas(cam_pos_m=0.0, cam_rot_rad=0.0)
    cam_pos, cam_rot = 0.01, np.radians(5.0)
    calib = CalibSigmas(cam_pos_m=cam_pos, cam_rot_rad=cam_rot)

    bd_zero = decompose_point(
        p_opt, intr.fx, intr.fy, np.eye(4), 1.0, calib_zero,
        valid=True, interpolated=False, cov_base=np.eye(3),
    )
    assert np.allclose(bd_zero.handeye, np.zeros((3, 3)))

    bd = decompose_point(
        p_opt, intr.fx, intr.fy, np.eye(4), 1.0, calib,
        valid=True, interpolated=False, cov_base=np.eye(3),
    )
    expected = (cam_pos**2 + (float(np.linalg.norm(p_opt)) * cam_rot) ** 2) * np.eye(3)
    assert np.allclose(bd.handeye, expected, atol=1e-18)


def test_decompose_point_refuses_invalid_and_interpolated():
    intr = make_intrinsics()
    calib = CalibSigmas(cam_pos_m=0.0, cam_rot_rad=0.0)
    with pytest.raises(ValueError):
        decompose_point(
            [0.0, 0.0, 0.25], intr.fx, intr.fy, np.eye(4), 1.0, calib,
            valid=False, interpolated=False, cov_base=np.eye(3),
        )
    with pytest.raises(ValueError):
        decompose_point(
            [0.0, 0.0, 0.25], intr.fx, intr.fy, np.eye(4), 1.0, calib,
            valid=True, interpolated=False, cov_base=np.full((3, 3), np.nan),
        )
    with pytest.raises(ValueError):
        decompose_point(
            [0.0, 0.0, 0.25], intr.fx, intr.fy, np.eye(4), 1.0, calib,
            valid=True, interpolated=True, cov_base=np.eye(3),
        )


# --- 2. fk_covariance: zero at sigma=0, sigma^2 scaling, PSD -----------------------------


def _real_T_base_optical_of_q():
    chain = kinematics.load_chain()
    end_effector = kinematics.load_end_effector()
    T_camera_nominal = kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME

    def T_base_optical_of_q(q):
        return chain.fk(q) @ end_effector.T_gripper_link_camera_link @ T_camera_nominal

    return T_base_optical_of_q


def test_fk_covariance_zero_scaling_and_psd():
    T_base_optical_of_q = _real_T_base_optical_of_q()
    q = np.array([-0.3, -1.0, -1.0, 0.0, 0.0, 0.0])
    p_optical = np.array([0.01, -0.02, 0.2])

    zero = fk_covariance(T_base_optical_of_q, q, p_optical, 0.0)
    assert np.array_equal(zero, np.zeros((3, 3)))

    sigma_1 = fk_covariance(T_base_optical_of_q, q, p_optical, 0.001)
    sigma_2 = fk_covariance(T_base_optical_of_q, q, p_optical, 0.002)
    assert np.allclose(sigma_2, 4.0 * sigma_1, rtol=1e-6)

    eigvals = np.linalg.eigvalsh(sigma_1)
    assert np.all(eigvals >= -1e-12)
    assert np.any(eigvals > 0)


# --- 3. decompose_waypoint: term isolation + totals --------------------------------------


def test_decompose_waypoint_sums_and_tcp_isolated():
    from crackvision.uncertainty import PointBreakdown

    pixel = np.diag([1e-6, 0.0, 0.0])
    depth = np.diag([0.0, 2e-6, 0.0])
    handeye = np.diag([0.0, 0.0, 3e-6])
    fk = np.diag([4e-6, 0.0, 0.0])
    bd = PointBreakdown(pixel=pixel, depth=depth, handeye=handeye, fk=fk)

    n = np.array([1.0, 0.0, 0.0])
    wp = decompose_waypoint(bd, n, tool_sigma_pos_m=0.005)
    assert wp.pixel == pytest.approx(1e-6)
    assert wp.depth == pytest.approx(0.0)
    assert wp.handeye == pytest.approx(0.0)
    assert wp.fk == pytest.approx(4e-6)
    assert wp.tcp == pytest.approx(0.005**2)
    assert wp.total == pytest.approx(wp.pixel + wp.depth + wp.handeye + wp.fk + wp.tcp)


# --- 4. gap-fill scenario: real lift3d + tool_waypoints, without-fk sum matches ----------


def _island_depth_image(n_points, spacing=30, patch=12, invalid_indices=(), depths=None):
    height = 100
    width = spacing * (n_points + 2)
    row = height // 2
    cols = [spacing * (i + 1) for i in range(n_points)]
    boundaries = [0] + [(cols[i] + cols[i + 1]) // 2 for i in range(n_points - 1)] + [width]
    depth_u16 = np.zeros((height, width), dtype=np.uint16)
    for i in range(n_points):
        c0, c1 = boundaries[i], boundaries[i + 1]
        depth_u16[:, c0:c1] = int(round(depths[i] / SCALE))
    for idx, col in enumerate(cols):
        if idx in invalid_indices:
            r0, r1 = max(0, row - patch), min(height, row + patch + 1)
            c0, c1 = max(0, col - patch), min(width, col + patch + 1)
            depth_u16[r0:r1, c0:c1] = 0
    mask = np.zeros((height, width), dtype=bool)
    points_rc = [(row, c) for c in cols]
    return depth_u16, mask, points_rc


def test_gap_fill_waypoint_sum_matches_tool_waypoints_without_fk():
    n_points = 12
    invalid = set(range(5, 8))  # 3px interior gap, bridged (max_gap_px default 5)
    depths = [0.2] * n_points
    depths[4] = 0.10  # left bound of the gap
    depths[8] = 0.45  # right bound of the gap: different trace than the left bound
    depth_u16, mask, points_rc = _island_depth_image(n_points, invalid_indices=invalid, depths=depths)

    intr = make_intrinsics()
    calib = CalibSigmas(cam_pos_m=0.01, cam_rot_rad=np.radians(5.0))
    params = LiftParams()
    T = np.eye(4)
    lifted = lift_polyline(points_rc, depth_u16, mask, intr, SCALE, T, calib, params)

    assert lifted.interpolated.any()
    assert lifted.segments == [(0, n_points - 1)]
    left, right = 4, 8
    assert not np.isclose(np.trace(lifted.cov_base[left]), np.trace(lifted.cov_base[right]))

    seg = lifted.segments[0]
    wp_params = WaypointParams(waypoint_spacing_m=0.002)
    waypoints = segment_waypoints(lifted, seg, np.array([1.0, 0.0, 0.0]), tool_sigma_pos_m=0.005, params=wp_params)

    source_indices = waypoint_source_indices(lifted.p_base, seg, wp_params.waypoint_spacing_m)
    assert len(source_indices) == len(waypoints)
    assert any(lifted.interpolated[idx] for idx in source_indices)

    for wp, dense_idx in zip(waypoints, source_indices):
        measured_idx = measured_source_index(lifted.valid, lifted.interpolated, lifted.cov_base, dense_idx)
        assert not lifted.interpolated[measured_idx]

        point_bd = decompose_point(
            lifted.p_optical[measured_idx], intr.fx, intr.fy, T, params.pixel_sigma_px, calib,
            valid=bool(lifted.valid[measured_idx]), interpolated=bool(lifted.interpolated[measured_idx]),
            cov_base=lifted.cov_base[measured_idx],
        )
        wp_bd = decompose_waypoint(point_bd, wp.normal, tool_sigma_pos_m=0.005)
        assert wp_bd.total == pytest.approx(wp.sigma_along_normal_m**2, rel=1e-9)

        # with an fk term added, the new total is >= the base value.
        point_bd_fk = decompose_point(
            lifted.p_optical[measured_idx], intr.fx, intr.fy, T, params.pixel_sigma_px, calib,
            valid=bool(lifted.valid[measured_idx]), interpolated=bool(lifted.interpolated[measured_idx]),
            cov_base=lifted.cov_base[measured_idx], fk_cov=np.diag([1e-8, 1e-8, 1e-8]),
        )
        wp_bd_fk = decompose_waypoint(point_bd_fk, wp.normal, tool_sigma_pos_m=0.005)
        assert wp_bd_fk.total >= wp_bd.total


# --- 5. measured_source_index / waypoint_source_indices error handling ------------------


def test_measured_source_index_errors():
    valid = np.array([True, False, True])
    interpolated = np.array([False, False, False])
    cov_base = np.tile(np.eye(3), (3, 1, 1))
    with pytest.raises(ValueError):
        measured_source_index(valid, interpolated, cov_base, 1)

    valid2 = np.array([False, True])
    interpolated2 = np.array([False, True])
    with pytest.raises(ValueError):
        measured_source_index(valid2, interpolated2, cov_base[:2], 1)

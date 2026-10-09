"""tests/test_lift3d.py — GEOM-08.5 lift ordered pixel polylines to base_link surface points."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from crackvision.geometry import (
    Intrinsics,
    REASON_NO_ANNULUS_CANDIDATES,
    REASON_NO_VALID_DEPTH_IN_ANNULUS,
    depth_uncertainty_m,
    deproject_pixels,
    fit_plane,
)
from crackvision.lift3d import (
    SEGMENT_TOO_SHORT,
    CalibSigmas,
    LiftParams,
    lift_polyline,
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


def _tilted_plane_depth_image(intr: Intrinsics, a: float, b: float, z0: float, scale=SCALE):
    rows = np.arange(intr.height)[:, None].astype(np.float64)
    cols = np.arange(intr.width)[None, :].astype(np.float64)
    x = (cols - intr.ppx) / intr.fx
    y = (rows - intr.ppy) / intr.fy
    depth_m = z0 / (1.0 - a * x - b * y)
    depth_u16 = np.round(depth_m / scale).astype(np.uint16)
    return depth_u16


# --- 1. Fronto-parallel plane, crack cavity under the mask -----------------------------


def test_fronto_parallel_surface_depth_excludes_crack_cavity():
    intr = make_intrinsics()
    depth_u16 = _flat_depth_image((100, 100), 0.25)
    mask = np.zeros((100, 100), dtype=bool)
    mask[45:48, 30:70] = True
    depth_u16[mask] = int(round(0.255 / SCALE))  # crack cavity reads 5mm deeper

    points_rc = [(46, c) for c in range(30, 70)]
    calib = CalibSigmas(cam_pos_m=0.0, cam_rot_rad=0.0)
    lifted = lift_polyline(points_rc, depth_u16, mask, intr, SCALE, np.eye(4), calib, LiftParams())

    assert lifted.valid.all()
    assert np.allclose(lifted.depth_m, 0.25, atol=1e-6)
    assert not np.any(np.isclose(lifted.depth_m, 0.255, atol=1e-4))
    assert np.allclose(lifted.n_base, np.array([0.0, 0.0, -1.0]), atol=1e-6)
    assert np.allclose(lifted.p_optical[:, 2], 0.25, atol=1e-6)


# --- 2. Tilted plane: outward orientation vs. fit_plane's raw sign -----------------------


def test_tilted_plane_normal_outward_and_within_half_degree():
    intr = make_intrinsics()
    a, b, z0 = 0.5774, 0.0, 0.25  # tan(30deg) tilt about one axis
    depth_u16 = _tilted_plane_depth_image(intr, a, b, z0)
    mask = np.zeros((intr.height, intr.width), dtype=bool)

    row, col = intr.height // 2, intr.width // 2
    points_rc = [(row, col + i) for i in range(-2, 3)]
    calib = CalibSigmas(cam_pos_m=0.0, cam_rot_rad=0.0)
    lifted = lift_polyline(points_rc, depth_u16, mask, intr, SCALE, np.eye(4), calib, LiftParams())

    idx = 2
    assert lifted.valid[idx]

    true_outward = np.array([a, -b, -1.0])
    true_outward /= np.linalg.norm(true_outward)

    cos_angle = float(np.clip(np.dot(lifted.n_base[idx], true_outward), -1.0, 1.0))
    angle_deg = np.degrees(np.arccos(cos_angle))
    assert angle_deg < 0.5

    # Demonstrate the documented pitfall: fit_plane's own raw (unflipped) normal, on this same
    # surface, points away from the camera -- callers must not rely on its sign directly.
    rng = np.random.default_rng(0)
    rr = rng.uniform(row - 50, row + 50, size=200)
    cc = rng.uniform(col - 50, col + 50, size=200)
    x = (cc - intr.ppx) / intr.fx
    y = (rr - intr.ppy) / intr.fy
    depth = z0 / (1.0 - a * x - b * y)
    raw_points = deproject_pixels(intr, rr, cc, depth)
    raw_fit = fit_plane(raw_points)
    p_opt = lifted.p_optical[idx]
    assert float(np.dot(raw_fit.normal, -p_opt)) < 0.0


# --- 3. T_base_link_optical chain --------------------------------------------------------


def test_p_base_matches_known_transform():
    intr = make_intrinsics()
    depth_u16 = _flat_depth_image((100, 100), 0.3)
    mask = np.zeros((100, 100), dtype=bool)
    points_rc = [(50, c) for c in range(40, 50)]
    calib = CalibSigmas(cam_pos_m=0.0, cam_rot_rad=0.0)

    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", [10, 20, 30], degrees=True).as_matrix()
    T[:3, 3] = [0.1, -0.2, 0.5]

    lifted = lift_polyline(points_rc, depth_u16, mask, intr, SCALE, T, calib, LiftParams())
    assert lifted.valid.all()

    for i in range(len(points_rc)):
        expected = (T @ np.array([*lifted.p_optical[i], 1.0]))[:3]
        assert np.allclose(lifted.p_base[i], expected, atol=1e-9)


# --- 4. Gap interpolation / splitting / trimming / drop ---------------------------------


def _island_depth_image(n_points, spacing=30, patch=12, valid_depth=0.2, invalid_indices=(), depths=None):
    """`depths`, if given, is a per-point depth (m), letting neighbouring islands differ so a
    gap's bounding neighbours have genuinely different covariance traces. Column boundaries
    between islands sit at the midpoints, far outside any point's annulus (outer_px=8 << half
    the 30px spacing), so each point's own annulus stays within its own uniform depth."""
    height = 100
    width = spacing * (n_points + 2)
    if depths is None:
        depths = [valid_depth] * n_points
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


def test_gap_policy_interpolate_split_trim_and_drop():
    # 0,1: leading trim; 2-6: valid; 7-9: 3px gap (interpolated); 10-14: valid;
    # 15-22: 8px gap (splits); 23,24: valid but too short (dropped); 25,26: trailing trim.
    invalid = set(range(0, 2)) | set(range(7, 10)) | set(range(15, 23)) | set(range(25, 27))
    n_points = 27
    # point 6 and point 10 bound the 3px gap (7-9); give them different depths so their
    # covariance traces differ and the "conservative (larger-trace) neighbour" choice is
    # actually distinguishable (sigma_z is quadratic in depth).
    depths = [0.2] * n_points
    depths[6] = 0.12
    depths[10] = 0.45
    depth_u16, mask, points_rc = _island_depth_image(n_points, invalid_indices=invalid, depths=depths)

    calib = CalibSigmas(cam_pos_m=0.0, cam_rot_rad=0.0)
    T = np.eye(4)
    lifted = lift_polyline(points_rc, depth_u16, mask, make_intrinsics(), SCALE, T, calib, LiftParams())

    # leading/trailing trims stay invalid, never interpolated/extrapolated
    for i in (0, 1, 25, 26):
        assert not lifted.valid[i]
        assert not lifted.interpolated[i]

    # the 3px interior gap is bridged
    left, right = 6, 10
    assert not np.isclose(np.trace(lifted.cov_base[left]), np.trace(lifted.cov_base[right]))
    conservative = left if np.trace(lifted.cov_base[left]) >= np.trace(lifted.cov_base[right]) else right
    span = right - left
    T_optical_base = np.linalg.inv(T)
    for i in (7, 8, 9):
        assert lifted.valid[i]
        assert lifted.interpolated[i]
        assert lifted.reason[i].startswith("interpolated:")

        t = (i - left) / span
        expected_p_base = (1 - t) * lifted.p_base[left] + t * lifted.p_base[right]
        assert np.allclose(lifted.p_base[i], expected_p_base, atol=1e-9)

        nlerp = (1 - t) * lifted.n_base[left] + t * lifted.n_base[right]
        expected_n_base = nlerp / np.linalg.norm(nlerp)
        assert np.allclose(lifted.n_base[i], expected_n_base, atol=1e-9)
        assert np.isclose(np.linalg.norm(lifted.n_base[i]), 1.0, atol=1e-9)

        assert np.allclose(lifted.cov_base[i], lifted.cov_base[conservative], atol=1e-12)
        assert np.isclose(lifted.normal_rms_m[i], lifted.normal_rms_m[conservative], atol=1e-12)
        assert np.isclose(lifted.sigma_normal_rad[i], lifted.sigma_normal_rad[conservative], atol=1e-12)

        expected_p_opt = (T_optical_base @ np.array([*lifted.p_base[i], 1.0]))[:3]
        assert np.allclose(lifted.p_optical[i], expected_p_opt, atol=1e-9)
        assert np.isclose(lifted.depth_m[i], lifted.p_optical[i][2], atol=1e-9)

    # every valid point, including interpolated ones, carries fully finite geometry (no NaN leak)
    for i in range(n_points):
        if not lifted.valid[i]:
            continue
        assert np.isfinite(lifted.depth_m[i])
        assert np.all(np.isfinite(lifted.p_optical[i]))
        assert np.all(np.isfinite(lifted.p_base[i]))
        assert np.all(np.isfinite(lifted.n_base[i]))
        assert np.isfinite(lifted.normal_rms_m[i])
        assert np.isfinite(lifted.sigma_normal_rad[i])
        assert np.all(np.isfinite(lifted.cov_base[i]))

    # the 8px interior gap is not bridged and splits the polyline
    for i in range(15, 23):
        assert not lifted.valid[i]
        assert not lifted.interpolated[i]

    assert lifted.segments == [(2, 14)]
    assert lifted.dropped_segments == 1

    # the 2-point tail segment (23, 24) is dropped for being below min_segment_points
    for i in (23, 24):
        assert not lifted.valid[i]
        assert lifted.reason[i] == SEGMENT_TOO_SHORT
        assert np.all(np.isnan(lifted.p_base[i]))

    # the kept segment's genuinely-measured points stay valid with real geometry
    for i in (2, 3, 4, 5, 6, 10, 11, 12, 13, 14):
        assert lifted.valid[i]
        assert not lifted.interpolated[i]
        assert not np.any(np.isnan(lifted.p_base[i]))


# --- 5. Covariance: finite-difference Jacobian + calibration term growth ----------------


def _finite_diff_sigma_opt(intr: Intrinsics, row, col, z, sigma_px, sigma_z, h=1e-4, hz=1e-7):
    def f(r, c, zz):
        return deproject_pixels(intr, r, c, zz)

    J = np.zeros((3, 3))
    J[:, 0] = (f(row, col + h, z) - f(row, col - h, z)) / (2 * h)  # d/d col
    J[:, 1] = (f(row + h, col, z) - f(row - h, col, z)) / (2 * h)  # d/d row
    J[:, 2] = (f(row, col, z + hz) - f(row, col, z - hz)) / (2 * hz)  # d/d z

    cov_pix = np.diag([sigma_px**2, sigma_px**2, sigma_z**2])
    return J @ cov_pix @ J.T


def test_covariance_matches_finite_difference_jacobian():
    # ppx/ppy placed at the exact centre of the 100x100 image, so (row=ppy, col=ppx)=(50,50) is
    # the true principal-point pixel (x=y=0, where J's off-diagonal z-terms vanish) -- not just
    # an arbitrarily-named "centre" pixel far off-axis.
    intr = make_intrinsics(ppx=50.0, ppy=50.0)
    depth_u16 = _flat_depth_image((100, 100), 0.3)
    mask = np.zeros((100, 100), dtype=bool)
    # principal-point pixel + a genuinely off-axis pixel, far enough apart to avoid annulus
    # overlap at this size.
    points_rc = [(50, 50), (50, 50), (50, 50), (80, 20), (80, 20), (80, 20)]
    calib = CalibSigmas(cam_pos_m=0.0, cam_rot_rad=0.0)
    params = LiftParams()
    lifted = lift_polyline(points_rc, depth_u16, mask, intr, SCALE, np.eye(4), calib, params)
    assert lifted.valid.all()

    for idx, (row, col) in [(1, (50, 50)), (4, (80, 20))]:
        z = float(lifted.p_optical[idx][2])
        sigma_z = float(depth_uncertainty_m(z, intr.fx))
        sigma_opt_fd = _finite_diff_sigma_opt(intr, row, col, z, params.pixel_sigma_px, sigma_z)
        assert np.allclose(lifted.cov_base[idx], sigma_opt_fd, rtol=1e-6, atol=1e-12)


def test_calibration_term_grows_with_range():
    intr = make_intrinsics()
    mask = np.zeros((100, 100), dtype=bool)
    points_rc = [(50, 50)] * 3
    params = LiftParams()

    cam_pos_m, cam_rot_rad = 0.01, np.radians(5.0)
    calib_zero = CalibSigmas(cam_pos_m=0.0, cam_rot_rad=0.0)
    calib_real = CalibSigmas(cam_pos_m=cam_pos_m, cam_rot_rad=cam_rot_rad)

    extras = {}
    for z in (0.10, 0.40):
        depth_u16 = _flat_depth_image((100, 100), z)
        lifted_zero = lift_polyline(points_rc, depth_u16, mask, intr, SCALE, np.eye(4), calib_zero, params)
        lifted_real = lift_polyline(points_rc, depth_u16, mask, intr, SCALE, np.eye(4), calib_real, params)
        extra = lifted_real.cov_base[0] - lifted_zero.cov_base[0]
        p_opt = lifted_zero.p_optical[0]
        expected = cam_pos_m**2 + (np.linalg.norm(p_opt) * cam_rot_rad) ** 2
        assert np.allclose(extra, expected * np.eye(3), atol=1e-12)
        extras[z] = expected

    assert extras[0.40] > extras[0.10]


# --- 6. Invalid points never carry NaN-leaking or origin-sentinel geometry --------------


def test_invalid_points_have_null_geometry_and_reason():
    intr = make_intrinsics()
    depth_u16 = np.zeros((60, 60), dtype=np.uint16)  # entirely invalid depth
    mask = np.zeros((60, 60), dtype=bool)
    points_rc = [(30, 30), (30, 31), (30, 32)]
    calib = CalibSigmas(cam_pos_m=0.0, cam_rot_rad=0.0)
    lifted = lift_polyline(points_rc, depth_u16, mask, intr, SCALE, np.eye(4), calib, LiftParams())

    assert not lifted.valid.any()
    for i in range(3):
        assert lifted.reason[i] != ""
        assert np.all(np.isnan(lifted.p_base[i]))
        assert np.all(np.isnan(lifted.p_optical[i]))
        assert np.all(np.isnan(lifted.cov_base[i]))

    # a point with no annulus candidates at all (fully masked) also gets a non-empty reason
    full_mask = np.ones((60, 60), dtype=bool)
    lifted2 = lift_polyline([(30, 30)], depth_u16, full_mask, intr, SCALE, np.eye(4), calib, LiftParams())
    assert lifted2.reason[0] in (REASON_NO_ANNULUS_CANDIDATES, REASON_NO_VALID_DEPTH_IN_ANNULUS)

    # no valid point is ever within 1cm of the camera origin
    depth_u16_flat = _flat_depth_image((100, 100), 0.25)
    mask_flat = np.zeros((100, 100), dtype=bool)
    points_valid = [(50, c) for c in range(40, 50)]
    lifted3 = lift_polyline(points_valid, depth_u16_flat, mask_flat, intr, SCALE, np.eye(4), calib, LiftParams())
    assert lifted3.valid.all()
    assert np.all(np.linalg.norm(lifted3.p_optical, axis=1) > 0.01)
    assert np.all(np.linalg.norm(lifted3.p_base, axis=1) > 0.01)

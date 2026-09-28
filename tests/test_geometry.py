"""tests/test_geometry.py — GEOM-02 depth projection library."""

from __future__ import annotations

import numpy as np
import pytest

from crackvision.geometry import (
    DEFAULT_VALID_DEPTH_RANGE_M,
    REASON_INSUFFICIENT_VALID_FRACTION,
    REASON_NAN,
    REASON_NEGATIVE,
    REASON_NO_ANNULUS_CANDIDATES,
    REASON_NO_VALID_DEPTH_IN_ANNULUS,
    REASON_OUT_OF_BAND,
    REASON_ZERO,
    Intrinsics,
    classify_depth,
    depth_uncertainty_m,
    deproject_pixels,
    deproject_pixels_from_image,
    fit_plane,
    sample_surface_depth_annulus,
)

D405_COLOR_STREAM = {
    "width": 848,
    "height": 480,
    "intrinsics": {
        "fx": 430.2,
        "fy": 430.5,
        "ppx": 424.7,
        "ppy": 239.8,
        "model": "distortion.inverse_brown_conrady",
        # Typical Brown-Conrady calibration magnitude (>=1e-2 for k1/k2/k3), large enough that
        # getting the tangential-term scaling wrong (GEOM-02.R1) shows up as tens-to-hundreds of
        # micrometres of parity error against pyrealsense2, not lost in float32 noise.
        "coeffs": [-0.0547, 0.0576, 0.00036, 0.00066, -0.0187],
    },
}

# A second coefficient set with |p1|, |p2| (tangential terms, coeffs[2]/coeffs[3]) >= 0.004,
# to exercise the tangential-term scaling specifically (GEOM-02.R1).
D405_COLOR_STREAM_STRONG_TANGENTIAL = {
    **D405_COLOR_STREAM,
    "intrinsics": {
        **D405_COLOR_STREAM["intrinsics"],
        "coeffs": [0.05, 0.03, 0.005, -0.004, 0.01],
    },
}


def make_intrinsics(**overrides) -> Intrinsics:
    intr = Intrinsics.from_stream_dict(D405_COLOR_STREAM)
    if overrides:
        from dataclasses import replace

        intr = replace(intr, **overrides)
    return intr


# --- Intrinsics -------------------------------------------------------------------------


def test_intrinsics_from_stream_dict():
    intr = Intrinsics.from_stream_dict(D405_COLOR_STREAM)
    assert intr.width == 848
    assert intr.height == 480
    assert intr.fx == pytest.approx(430.2)
    assert intr.model == "distortion.inverse_brown_conrady"
    assert intr.coeffs == (-0.0547, 0.0576, 0.00036, 0.00066, -0.0187)


def test_intrinsics_rejects_wrong_coeff_count():
    bad = {**D405_COLOR_STREAM, "intrinsics": {**D405_COLOR_STREAM["intrinsics"], "coeffs": [0.0, 0.0]}}
    with pytest.raises(ValueError):
        Intrinsics.from_stream_dict(bad)


# --- Deprojection: pixel-centre convention and pyrealsense2 parity -----------------------


def test_deproject_pixel_centre_no_half_pixel_offset():
    """ADR-012: an integer pixel addresses its own centre, no +0.5 offset (rsutil.h formula)."""
    intr = make_intrinsics(model="none", coeffs=(0.0, 0.0, 0.0, 0.0, 0.0))
    point = deproject_pixels(intr, np.array([intr.ppy]), np.array([intr.ppx]), np.array([1.0]))
    assert point[0, 0] == pytest.approx(0.0, abs=1e-12)
    assert point[0, 1] == pytest.approx(0.0, abs=1e-12)
    assert point[0, 2] == pytest.approx(1.0)


def test_deproject_matches_pyrealsense2_none_model():
    rs = pytest.importorskip("pyrealsense2")
    intr = make_intrinsics(model="none", coeffs=(0.0, 0.0, 0.0, 0.0, 0.0))
    rs_intr = _to_rs_intrinsics(rs, intr)

    rng = np.random.default_rng(0)
    rows = rng.uniform(0, intr.height - 1, size=50)
    cols = rng.uniform(0, intr.width - 1, size=50)
    depths = rng.uniform(0.07, 0.5, size=50)

    mine = deproject_pixels(intr, rows, cols, depths)
    for row, col, depth, expected in zip(rows, cols, depths, mine):
        actual = rs.rs2_deproject_pixel_to_point(rs_intr, [float(col), float(row)], float(depth))
        assert np.max(np.abs(np.array(actual) - expected)) < 1e-6


@pytest.mark.parametrize(
    "stream", [D405_COLOR_STREAM, D405_COLOR_STREAM_STRONG_TANGENTIAL], ids=["k-heavy", "p-heavy"]
)
def test_deproject_matches_pyrealsense2_inverse_brown_conrady(stream):
    """GEOM-02.R1: xq=x/icdist, yq=y/icdist tangential scaling, matched at realistic (>=1e-2)
    coefficient magnitudes -- not just the small coefficients that hide the missing scaling."""
    rs = pytest.importorskip("pyrealsense2")
    intr = Intrinsics.from_stream_dict(stream)
    rs_intr = _to_rs_intrinsics(rs, intr)

    rng = np.random.default_rng(1)
    rows = rng.uniform(0, intr.height - 1, size=200)
    cols = rng.uniform(0, intr.width - 1, size=200)
    depths = rng.uniform(0.07, 0.5, size=200)

    mine = deproject_pixels(intr, rows, cols, depths)
    max_err = 0.0
    for row, col, depth, expected in zip(rows, cols, depths, mine):
        actual = np.array(
            rs.rs2_deproject_pixel_to_point(rs_intr, [float(col), float(row)], float(depth))
        )
        max_err = max(max_err, float(np.max(np.abs(actual - expected))))
    assert max_err < 1e-6


@pytest.mark.parametrize(
    "stream", [D405_COLOR_STREAM, D405_COLOR_STREAM_STRONG_TANGENTIAL], ids=["k-heavy", "p-heavy"]
)
def test_deproject_matches_pyrealsense2_brown_conrady(stream):
    rs = pytest.importorskip("pyrealsense2")
    base = Intrinsics.from_stream_dict(stream)
    from dataclasses import replace

    intr = replace(base, model="distortion.brown_conrady")
    rs_intr = _to_rs_intrinsics(rs, intr)

    rng = np.random.default_rng(2)
    rows = rng.uniform(0, intr.height - 1, size=200)
    cols = rng.uniform(0, intr.width - 1, size=200)
    depths = rng.uniform(0.07, 0.5, size=200)

    mine = deproject_pixels(intr, rows, cols, depths)
    max_err = 0.0
    for row, col, depth, expected in zip(rows, cols, depths, mine):
        actual = rs.rs2_deproject_pixel_to_point(rs_intr, [float(col), float(row)], float(depth))
        max_err = max(max_err, float(np.max(np.abs(np.array(actual) - expected))))
    assert max_err < 1e-6


def _to_rs_intrinsics(rs, intr: Intrinsics):
    rs_intr = rs.intrinsics()
    rs_intr.width, rs_intr.height = intr.width, intr.height
    rs_intr.fx, rs_intr.fy = intr.fx, intr.fy
    rs_intr.ppx, rs_intr.ppy = intr.ppx, intr.ppy
    model_token = intr.model.rsplit(".", 1)[-1]
    rs_intr.model = getattr(rs.distortion, model_token)
    rs_intr.coeffs = list(intr.coeffs)
    return rs_intr


def test_deproject_unsupported_model_raises():
    intr = make_intrinsics(model="distortion.ftheta")
    with pytest.raises(NotImplementedError):
        deproject_pixels(intr, np.array([0]), np.array([0]), np.array([0.2]))


def test_deproject_vectorised_shape():
    intr = make_intrinsics()
    rows = np.arange(5)
    cols = np.arange(5)
    depths = np.full(5, 0.2)
    points = deproject_pixels(intr, rows, cols, depths)
    assert points.shape == (5, 3)


# --- Invalid-depth policy -----------------------------------------------------------------


def test_classify_depth_zero_is_invalid_with_reason():
    result = classify_depth(np.array([0], dtype=np.uint16), depth_scale_m_per_unit=1e-4)
    assert not result.valid[0]
    assert result.reason[0] == REASON_ZERO
    assert np.isnan(result.depth_m[0])


def test_classify_depth_out_of_band_is_invalid_with_reason():
    # 1.0 m raw depth at 1e-4 scale = way beyond the 0.07-0.50 m band.
    result = classify_depth(np.array([10000], dtype=np.uint16), depth_scale_m_per_unit=1e-4)
    assert not result.valid[0]
    assert result.reason[0] == REASON_OUT_OF_BAND
    assert np.isnan(result.depth_m[0])


def test_classify_depth_nan_is_invalid_with_reason():
    result = classify_depth(np.array([np.nan]), depth_scale_m_per_unit=1e-4)
    assert not result.valid[0]
    assert result.reason[0] == REASON_NAN


def test_classify_depth_negative_is_invalid_with_reason():
    result = classify_depth(np.array([-5.0]), depth_scale_m_per_unit=1e-4)
    assert not result.valid[0]
    assert result.reason[0] == REASON_NEGATIVE


def test_classify_depth_valid_in_band():
    # 2000 raw * 1e-4 = 0.2 m, within (0.07, 0.50).
    result = classify_depth(np.array([2000], dtype=np.uint16), depth_scale_m_per_unit=1e-4)
    assert result.valid[0]
    assert result.reason[0] == ""
    assert result.depth_m[0] == pytest.approx(0.2)


def test_classify_depth_respects_custom_band():
    result = classify_depth(
        np.array([2000], dtype=np.uint16), depth_scale_m_per_unit=1e-4, valid_range_m=(0.5, 1.0)
    )
    assert not result.valid[0]
    assert result.reason[0] == REASON_OUT_OF_BAND


def test_deproject_pixels_from_image_never_silently_produces_point_at_zero():
    intr = make_intrinsics()
    depth_image = np.zeros((10, 10), dtype=np.uint16)
    depth_image[3, 4] = 2000  # 0.2 m, valid
    rows = np.array([3, 3])
    cols = np.array([4, 5])  # second pixel has raw depth 0 -> invalid

    points, valid, reason = deproject_pixels_from_image(intr, rows, cols, depth_image, 1e-4)

    assert valid[0]
    assert not np.isnan(points[0]).any()

    assert not valid[1]
    assert reason[1] == REASON_ZERO
    assert np.isnan(points[1]).all()


def test_depth_uncertainty_grows_with_depth():
    sigma_near = depth_uncertainty_m(0.1, fx=430.0)
    sigma_far = depth_uncertainty_m(0.4, fx=430.0)
    assert sigma_far > sigma_near > 0


# --- Crack-cavity annulus policy (RISKS.md R-08) -------------------------------------------


def _flat_depth_image(shape, depth_m, depth_scale_m_per_unit):
    raw = np.round(depth_m / depth_scale_m_per_unit).astype(np.uint16)
    return np.full(shape, raw, dtype=np.uint16)


def test_annulus_sample_recovers_surface_depth_excluding_crack():
    scale = 1e-4
    depth_image = _flat_depth_image((50, 50), 0.20, scale)
    mask = np.zeros((50, 50), dtype=bool)
    mask[24:27, 20:30] = True  # a horizontal "crack" through the sample point
    depth_image[mask] = 0  # crack pixels read back as depth holes

    result = sample_surface_depth_annulus(
        depth_image, mask, row=25, col=25,
        depth_scale_m_per_unit=scale, inner_radius_px=4, outer_radius_px=10,
    )

    assert result.valid
    assert result.reason == ""
    assert result.depth_m == pytest.approx(0.20, abs=1e-6)
    assert result.fraction_valid == pytest.approx(1.0)
    assert result.n_candidates > 0


def test_annulus_sample_reports_fraction_valid_with_partial_holes():
    scale = 1e-4
    depth_image = _flat_depth_image((50, 50), 0.20, scale)
    mask = np.zeros((50, 50), dtype=bool)
    # Punch extra depth holes (not masked, just sensor dropouts) in half the annulus ring.
    depth_image[25, 15:25] = 0

    result = sample_surface_depth_annulus(
        depth_image, mask, row=25, col=25,
        depth_scale_m_per_unit=scale, inner_radius_px=4, outer_radius_px=10,
        min_fraction_valid=0.0,
    )

    assert result.valid
    assert 0.0 < result.fraction_valid < 1.0
    assert result.n_valid < result.n_candidates


def test_annulus_sample_invalid_when_all_holes():
    scale = 1e-4
    depth_image = np.zeros((50, 50), dtype=np.uint16)
    mask = np.zeros((50, 50), dtype=bool)

    result = sample_surface_depth_annulus(
        depth_image, mask, row=25, col=25,
        depth_scale_m_per_unit=scale, inner_radius_px=4, outer_radius_px=10,
    )

    assert not result.valid
    assert result.reason == REASON_NO_VALID_DEPTH_IN_ANNULUS
    assert result.fraction_valid == pytest.approx(0.0)
    assert np.isnan(result.depth_m)


def test_annulus_sample_insufficient_fraction_flagged_invalid():
    scale = 1e-4
    depth_image = _flat_depth_image((50, 50), 0.20, scale)
    mask = np.zeros((50, 50), dtype=bool)
    depth_image[10:40, 10:40] = 0  # blow out almost the whole annulus neighbourhood
    depth_image[25, 34] = int(round(0.20 / scale))  # leave exactly one valid sample

    result = sample_surface_depth_annulus(
        depth_image, mask, row=25, col=25,
        depth_scale_m_per_unit=scale, inner_radius_px=4, outer_radius_px=10,
        min_fraction_valid=0.5,
    )

    assert not result.valid
    assert result.reason == REASON_INSUFFICIENT_VALID_FRACTION


def test_annulus_sample_no_candidates_when_fully_masked():
    scale = 1e-4
    depth_image = _flat_depth_image((50, 50), 0.20, scale)
    mask = np.ones((50, 50), dtype=bool)

    result = sample_surface_depth_annulus(
        depth_image, mask, row=25, col=25,
        depth_scale_m_per_unit=scale, inner_radius_px=4, outer_radius_px=10,
    )

    assert not result.valid
    assert result.reason == REASON_NO_ANNULUS_CANDIDATES


def test_annulus_sample_rejects_bad_radii():
    depth_image = np.zeros((20, 20), dtype=np.uint16)
    mask = np.zeros((20, 20), dtype=bool)
    with pytest.raises(ValueError):
        sample_surface_depth_annulus(depth_image, mask, 10, 10, 1e-4, inner_radius_px=5, outer_radius_px=5)


# --- Synthetic plane recovery ---------------------------------------------------------------


def _synthetic_plane_points(intr: Intrinsics, a: float, b: float, z0: float, noise_std_m: float, seed: int):
    rng = np.random.default_rng(seed)
    rows = rng.uniform(50, intr.height - 50, size=400)
    cols = rng.uniform(50, intr.width - 50, size=400)
    x = (cols - intr.ppx) / intr.fx
    y = (rows - intr.ppy) / intr.fy
    depth = z0 / (1 - a * x - b * y)
    depth_noisy = depth + rng.normal(0.0, noise_std_m, size=depth.shape)
    points = deproject_pixels(intr, rows, cols, depth_noisy)
    true_normal = np.array([-a, -b, 1.0])
    true_normal /= np.linalg.norm(true_normal)
    return points, true_normal, z0


def test_fit_plane_recovers_tilted_noisy_plane():
    intr = make_intrinsics(model="none", coeffs=(0.0, 0.0, 0.0, 0.0, 0.0))
    points, true_normal, z0 = _synthetic_plane_points(intr, a=0.15, b=-0.10, z0=0.25, noise_std_m=0.0008, seed=3)

    fit = fit_plane(points)

    cos_angle = float(np.clip(np.dot(fit.normal, true_normal), -1.0, 1.0))
    angle_deg = np.degrees(np.arccos(abs(cos_angle)))
    assert angle_deg < 2.0
    assert fit.rms_residual_m < 0.003

    # GEOM-02.R1: the fitted plane's offset along its own normal, normal . centroid, must equal
    # z0 * true_normal[2] (the true plane's offset along that same normal) to within 1 mm -- this
    # is what catches a depth-scale/distance bug (RISKS.md R-09) that a normal-angle-only check
    # would miss (e.g. depths scaled x2 barely moves the angle but moves the offset by ~0.25 m).
    offset_err_m = abs(float(np.dot(fit.normal, fit.centroid)) - z0 * true_normal[2])
    assert offset_err_m < 1e-3


def test_fit_plane_flat_surface_low_residual():
    intr = make_intrinsics(model="none", coeffs=(0.0, 0.0, 0.0, 0.0, 0.0))
    points, true_normal, z0 = _synthetic_plane_points(intr, a=0.0, b=0.0, z0=0.3, noise_std_m=0.0, seed=4)

    fit = fit_plane(points)

    assert fit.rms_residual_m < 1e-9
    assert np.allclose(np.abs(fit.normal), np.abs(true_normal), atol=1e-6)

    # GEOM-02.R1: also check the recovered offset, not just the normal direction/residual.
    offset_err_m = abs(float(np.dot(fit.normal, fit.centroid)) - z0 * true_normal[2])
    assert offset_err_m < 1e-3


def test_fit_plane_requires_at_least_three_points():
    with pytest.raises(ValueError):
        fit_plane(np.array([[0.0, 0.0, 1.0], [0.1, 0.0, 1.0]]))

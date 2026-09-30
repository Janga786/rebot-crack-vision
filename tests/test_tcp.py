"""tests/test_tcp.py — GEOM-06 TCP pivot calibration solver."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from crackvision.calibration.tcp import (
    GRASP_CENTRE_OFFSET_M,
    MIN_POSES,
    PoseSample,
    check_boresight,
    load_tool_tip_prior,
    pivot_calibrate,
)

TRUE_TCP_OFFSET_M = np.array([-0.0443, 0.0012, -0.0005])
TRUE_PIVOT_POINT_M = np.array([0.32, -0.05, 0.18])


def _synthesize_poses(
    num_poses: int,
    tcp_offset_m: np.ndarray,
    pivot_point_m: np.ndarray,
    translation_noise_sigma_m: float = 0.0,
    rotation_noise_sigma_deg: float = 0.0,
    seed: int = 0,
) -> list[PoseSample]:
    """Build `T_base_link_tool0` poses consistent with pivoting about `pivot_point_m`.

    `t_i = pivot_point_m - R_i @ tcp_offset_m` exactly, then optional Gaussian noise is added to
    both the FK translation (simulating encoder/FK noise) and the FK rotation (simulating small
    orientation error), independently per pose.
    """
    rng = np.random.default_rng(seed)
    rotations = Rotation.random(num_poses, random_state=rng)
    poses = []
    for rotation in rotations:
        if rotation_noise_sigma_deg > 0.0:
            noise_axis = rng.normal(size=3)
            noise_axis /= np.linalg.norm(noise_axis)
            noise_angle_rad = np.radians(rng.normal(scale=rotation_noise_sigma_deg))
            noise_rotation = Rotation.from_rotvec(noise_axis * noise_angle_rad)
            rotation = noise_rotation * rotation

        exact_translation = pivot_point_m - rotation.as_matrix() @ tcp_offset_m
        noisy_translation = exact_translation + rng.normal(
            scale=translation_noise_sigma_m, size=3
        )
        poses.append(
            PoseSample(
                quat_xyzw=tuple(rotation.as_quat()),
                translation_m=tuple(noisy_translation),
            )
        )
    return poses


def test_pivot_calibrate_recovers_exact_tcp_noise_free():
    poses = _synthesize_poses(12, TRUE_TCP_OFFSET_M, TRUE_PIVOT_POINT_M)

    result = pivot_calibrate(poses)

    assert np.allclose(result.tcp_offset_m, TRUE_TCP_OFFSET_M, atol=1e-9)
    assert np.allclose(result.pivot_point_m, TRUE_PIVOT_POINT_M, atol=1e-9)
    assert result.rms_residual_m < 1e-9
    assert result.max_residual_m < 1e-9
    assert result.num_poses == 12


def test_pivot_calibrate_recovers_tcp_within_0_1mm_under_noise():
    poses = _synthesize_poses(
        40,
        TRUE_TCP_OFFSET_M,
        TRUE_PIVOT_POINT_M,
        translation_noise_sigma_m=5e-5,  # 0.05 mm, plausible FK translation noise
        rotation_noise_sigma_deg=0.02,
        seed=42,
    )

    result = pivot_calibrate(poses)

    tcp_error_m = np.linalg.norm(result.tcp_offset_m - TRUE_TCP_OFFSET_M)
    assert tcp_error_m < 1e-4, f"TCP recovery error {tcp_error_m * 1000:.4f} mm exceeds 0.1 mm"
    pivot_error_m = np.linalg.norm(result.pivot_point_m - TRUE_PIVOT_POINT_M)
    assert pivot_error_m < 1e-4
    # Residuals should track the injected noise level, not blow up.
    assert result.rms_residual_m < 5e-4
    assert result.residual_norms_m.shape == (40,)
    assert result.residuals_m.shape == (40, 3)


def test_pivot_calibrate_requires_minimum_poses():
    poses = _synthesize_poses(MIN_POSES - 1, TRUE_TCP_OFFSET_M, TRUE_PIVOT_POINT_M)

    with pytest.raises(ValueError, match="at least"):
        pivot_calibrate(poses)


def test_pivot_calibrate_accepts_exactly_minimum_poses():
    poses = _synthesize_poses(MIN_POSES, TRUE_TCP_OFFSET_M, TRUE_PIVOT_POINT_M)

    result = pivot_calibrate(poses)

    assert np.allclose(result.tcp_offset_m, TRUE_TCP_OFFSET_M, atol=1e-9)


def test_pivot_calibrate_degenerate_single_orientation_does_not_recover_tcp():
    """All poses sharing one orientation leaves the 6-unknown system rank-deficient.

    lstsq still returns *a* minimum-norm solution with ~zero residual, but it does not equal the
    true TCP offset — this pins down why the procedure requires orientation diversity, not just a
    pose count.
    """
    rotation = Rotation.from_euler("xyz", [10, 20, 30], degrees=True)
    poses = []
    for _ in range(6):
        exact_translation = TRUE_PIVOT_POINT_M - rotation.as_matrix() @ TRUE_TCP_OFFSET_M
        poses.append(
            PoseSample(quat_xyzw=tuple(rotation.as_quat()), translation_m=tuple(exact_translation))
        )

    result = pivot_calibrate(poses)

    assert result.rms_residual_m < 1e-9  # numerically "solved" ...
    assert not np.allclose(result.tcp_offset_m, TRUE_TCP_OFFSET_M, atol=1e-4)  # ... but wrong


def test_load_tool_tip_prior_reads_nominal_zero_from_config():
    prior = load_tool_tip_prior()

    assert np.allclose(prior.xyz_m, (0.0, 0.0, 0.0))
    assert prior.value_status == "nominal"
    assert prior.provenance == "urdf_mesh_prior"


def test_check_boresight_defaults_to_config_tool_tip_prior():
    check = check_boresight(np.zeros(3))

    assert np.allclose(check.prior_point_m, (0.0, 0.0, 0.0))


def test_check_boresight_closed_gripper_calibration_near_zero_is_consistent():
    """A correct closed-gripper pivot calibration lands near the nominal (0, 0, 0) tool-tip prior."""
    measured_tip_m = np.array([0.0006, -0.0002, 0.0004])

    check = check_boresight(measured_tip_m)

    assert check.position_delta_norm_m < 0.001  # sub-millimetre delta from the prior
    # Both vectors are near the origin: direction is undefined, not reported.
    assert check.angle_from_prior_deg is None


def test_check_boresight_held_probe_reports_positive_delta_not_opposite_direction():
    """A probe held +40 mm along +X is a positive delta from the tool-tip prior, not an error."""
    measured_tip_m = np.array([0.04, 0.0, 0.0])

    check = check_boresight(measured_tip_m)

    assert check.position_delta_norm_m == pytest.approx(0.04, abs=1e-9)
    assert check.position_delta_m[0] == pytest.approx(0.04, abs=1e-9)
    # Prior (0,0,0) is below the minimum length for a direction: no spurious 180 deg flag.
    assert check.angle_from_prior_deg is None


def test_check_boresight_old_grasp_centre_prior_would_have_flagged_44mm():
    """Regression: GEOM-06's old default prior (gripper_tcp, -44.3 mm) is NOT today's default, and
    using it explicitly against a correct closed-gripper calibration shows why — it disagrees by
    ~44 mm, which would have masked a real 44 mm tool-tip defect as "matches the prior"."""
    measured_tip_m = np.array([0.0, 0.0, 0.0])  # a correct closed-gripper calibration

    check_against_grasp_centre = check_boresight(measured_tip_m, prior_offset_m=GRASP_CENTRE_OFFSET_M)
    check_against_tool_tip = check_boresight(measured_tip_m)

    assert check_against_grasp_centre.position_delta_norm_m == pytest.approx(0.0443, abs=1e-9)
    assert check_against_tool_tip.position_delta_norm_m == pytest.approx(0.0, abs=1e-9)


def test_check_boresight_reports_angle_when_both_vectors_long_enough():
    prior_m = np.array([0.04, 0.0, 0.0])
    measured_tip_m = np.array([0.0, 0.04, 0.0])  # same length, rotated 90 deg onto +Y

    check = check_boresight(measured_tip_m, prior_offset_m=tuple(prior_m))

    assert check.angle_from_prior_deg == pytest.approx(90.0, abs=1e-6)

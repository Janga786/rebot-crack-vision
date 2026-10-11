"""crackvision.uncertainty_mc — §14/ADR-018 §4 Monte-Carlo cross-check (GEOM-09.3.1).

Companion to `crackvision.uncertainty` (GEOM-09.2): instead of linearizing pixel/depth/
hand-eye/FK noise through a closed-form Jacobian, this module draws seeded samples of those
four terms and reruns the *real* code paths — `geometry.deproject_pixels` for pixel+depth,
`kinematics.KinematicChain.fk` for FK, and a real right-perturbation of the hand-eye extrinsic
composed through the same `chain.fk(q) @ T_gripper_link_camera_link @ T_camera_optical` chain
`uncertainty.py`'s callers build — then compares the resulting empirical covariance/sigma
against a caller-supplied analytic breakdown (e.g. `uncertainty.PointBreakdown.total` or
`uncertainty.WaypointBreakdown.total`).

The `tcp` term is the one exception: §14.2 documents it has no separable point-level recompute
— it is the robot's physical tool-offset residual, not a function of pixel/depth/extrinsic/joint
inputs — so it cannot be "rerun" the way the other four can. Instead it is modelled as a
directly-added isotropic `N(0, sigma_tool_pos_m^2 I)` 3D perturbation applied to the final
tool_tip position *after* the real waypoint geometry (surface point + clearance along normal)
has been computed from a perturbed sample. `mc_point_covariance` therefore has no `tcp`
parameter at all — a bare lifted point has no tool offset to perturb.

Pure library — no CLI, no file I/O, no argparse. Only imports from
`crackvision.{geometry,kinematics}` plus numpy/scipy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from crackvision import geometry, kinematics


@dataclass
class PointMCResult:
    """§14.4 Monte-Carlo cross-check result for one dense point."""

    cov_base_mc: np.ndarray  # 3x3 empirical covariance of the sampled p_base, m^2
    ratio_trace: float  # trace(Sigma_mc) / trace(Sigma_analytic)
    ratio_eig_max: float  # lambda_max(Sigma_mc) / lambda_max(Sigma_analytic)
    flagged: bool  # True iff either ratio falls outside the agreement band


@dataclass
class WaypointMCResult:
    """§14.5 Monte-Carlo cross-check result for one waypoint."""

    sigma_along_normal_mc_m: float  # sample stddev of the signed along-normal displacement
    ratio_along_normal: float  # sigma_mc / analytic_sigma_along_normal_m
    flagged: bool  # True iff the ratio falls outside the agreement band


def _draw_perturbed_p_base(
    *,
    row: float,
    col: float,
    depth_m: float,
    intrinsics: geometry.Intrinsics,
    sigma_px: float,
    chain: "kinematics.KinematicChain",
    q: Sequence[float],
    T_gripper_camera: np.ndarray,
    T_camera_optical: np.ndarray,
    sigma_joint_rad: float,
    cam_pos_m: float,
    cam_rot_rad: float,
    n_samples: int,
    rng: np.random.Generator,
) -> np.ndarray:
    """Draw `n_samples` perturbed `p_base` realizations.

    Each sample perturbs pixel row/col, depth, the hand-eye extrinsic (rotation+translation)
    and the FK joint angles all at once, then reruns the real deprojection/FK chain on that one
    perturbed sample — never perturbs/recomputes a term in isolation and sums variances (that
    would just reintroduce the analytic linearization this module exists to check against).
    """
    q_nominal = np.asarray(q, dtype=np.float64)
    n_joints = q_nominal.shape[0]
    T_gripper_camera = np.asarray(T_gripper_camera, dtype=np.float64)
    T_camera_optical = np.asarray(T_camera_optical, dtype=np.float64)

    d_rowcol = rng.normal(0.0, sigma_px, size=(n_samples, 2))
    sigma_z = float(geometry.depth_uncertainty_m(depth_m, intrinsics.fx))
    d_z = rng.normal(0.0, sigma_z, size=n_samples)
    d_theta = rng.normal(0.0, cam_rot_rad, size=(n_samples, 3))
    d_t = rng.normal(0.0, cam_pos_m, size=(n_samples, 3))
    d_q = rng.normal(0.0, sigma_joint_rad, size=(n_samples, n_joints))

    rows_s = row + d_rowcol[:, 0]
    cols_s = col + d_rowcol[:, 1]
    depth_s = depth_m + d_z
    p_opt_s = geometry.deproject_pixels(intrinsics, rows_s, cols_s, depth_s)

    p_base = np.empty((n_samples, 3), dtype=np.float64)
    for i in range(n_samples):
        delta_T = np.eye(4)
        delta_T[:3, :3] = Rotation.from_rotvec(d_theta[i]).as_matrix()
        delta_T[:3, 3] = d_t[i]
        T_cam_sample = T_gripper_camera @ delta_T
        T_base_optical_sample = chain.fk(q_nominal + d_q[i]) @ T_cam_sample @ T_camera_optical
        p_opt_h = np.array([p_opt_s[i, 0], p_opt_s[i, 1], p_opt_s[i, 2], 1.0])
        p_base[i] = (T_base_optical_sample @ p_opt_h)[:3]

    return p_base


def mc_point_covariance(
    *,
    row: float,
    col: float,
    depth_m: float,
    intrinsics: geometry.Intrinsics,
    sigma_px: float,
    chain: "kinematics.KinematicChain",
    q: Sequence[float],
    T_gripper_camera: np.ndarray,
    T_camera_optical: np.ndarray,
    sigma_joint_rad: float,
    cam_pos_m: float,
    cam_rot_rad: float,
    n_samples: int,
    seed: int,
    analytic_cov_base: np.ndarray,
    agreement_band: tuple[float, float] = (0.5, 2.0),
) -> PointMCResult:
    """§14.4 MC cross-check for one dense lifted point.

    Draws `n_samples` perturbed realizations of pixel/depth/hand-eye/FK, recomputes `p_base`
    via the real `geometry.deproject_pixels`/`kinematics.KinematicChain.fk` chain for each, and
    compares the resulting empirical covariance against `analytic_cov_base` (e.g.
    `uncertainty.PointBreakdown.total`). `seed` fixes a `Generator(PCG64(seed))` so the result
    is bit-for-bit reproducible.
    """
    rng = np.random.Generator(np.random.PCG64(seed))
    p_base = _draw_perturbed_p_base(
        row=row, col=col, depth_m=depth_m, intrinsics=intrinsics, sigma_px=sigma_px,
        chain=chain, q=q, T_gripper_camera=T_gripper_camera, T_camera_optical=T_camera_optical,
        sigma_joint_rad=sigma_joint_rad, cam_pos_m=cam_pos_m, cam_rot_rad=cam_rot_rad,
        n_samples=n_samples, rng=rng,
    )

    cov_mc = np.cov(p_base, rowvar=False, ddof=1)
    analytic = np.asarray(analytic_cov_base, dtype=np.float64)

    ratio_trace = float(np.trace(cov_mc) / np.trace(analytic))
    ratio_eig_max = float(
        np.linalg.eigvalsh(cov_mc).max() / np.linalg.eigvalsh(analytic).max()
    )
    lo, hi = agreement_band
    flagged = not (lo <= ratio_trace <= hi and lo <= ratio_eig_max <= hi)

    return PointMCResult(
        cov_base_mc=cov_mc, ratio_trace=ratio_trace, ratio_eig_max=ratio_eig_max, flagged=flagged,
    )


def mc_waypoint_sigma(
    *,
    row: float,
    col: float,
    depth_m: float,
    intrinsics: geometry.Intrinsics,
    sigma_px: float,
    chain: "kinematics.KinematicChain",
    q: Sequence[float],
    T_gripper_camera: np.ndarray,
    T_camera_optical: np.ndarray,
    sigma_joint_rad: float,
    cam_pos_m: float,
    cam_rot_rad: float,
    normal: Sequence[float],
    clearance_m: float,
    tool_sigma_pos_m: float,
    n_samples: int,
    seed: int,
    analytic_sigma_along_normal_m: float,
    agreement_band: tuple[float, float] = (0.5, 2.0),
) -> WaypointMCResult:
    """§14.5 MC cross-check for one `tool_waypoints.Waypoint`.

    Perturbs the same four terms as `mc_point_covariance` for the waypoint's source point/
    normal, then — since `tcp` has no recompute (module docstring) — adds an isotropic
    `N(0, tool_sigma_pos_m^2 I)` perturbation directly to the resulting tool_tip position
    (`surface_point + clearance_m * normal`). Returns the sample stddev of the signed
    along-normal displacement relative to the noiseless nominal tool_tip, and its ratio against
    `analytic_sigma_along_normal_m` (e.g. `uncertainty.WaypointBreakdown.total ** 0.5`, or the
    accepted `sigma_along_normal_m` it is compared against in §14.5).
    """
    rng = np.random.Generator(np.random.PCG64(seed))
    p_base = _draw_perturbed_p_base(
        row=row, col=col, depth_m=depth_m, intrinsics=intrinsics, sigma_px=sigma_px,
        chain=chain, q=q, T_gripper_camera=T_gripper_camera, T_camera_optical=T_camera_optical,
        sigma_joint_rad=sigma_joint_rad, cam_pos_m=cam_pos_m, cam_rot_rad=cam_rot_rad,
        n_samples=n_samples, rng=rng,
    )
    d_tool = rng.normal(0.0, tool_sigma_pos_m, size=(n_samples, 3))

    n = np.asarray(normal, dtype=np.float64)
    q_nominal = np.asarray(q, dtype=np.float64)
    p_opt_nominal = geometry.deproject_pixels(intrinsics, row, col, depth_m)
    T_base_optical_nominal = (
        chain.fk(q_nominal) @ np.asarray(T_gripper_camera) @ np.asarray(T_camera_optical)
    )
    p_opt_nominal_h = np.array([p_opt_nominal[0], p_opt_nominal[1], p_opt_nominal[2], 1.0])
    p_base_nominal = (T_base_optical_nominal @ p_opt_nominal_h)[:3]

    tool_tip_sample = p_base + clearance_m * n + d_tool
    tool_tip_nominal = p_base_nominal + clearance_m * n

    displacement = (tool_tip_sample - tool_tip_nominal) @ n
    sigma_mc = float(np.std(displacement, ddof=1))

    ratio = sigma_mc / float(analytic_sigma_along_normal_m)
    lo, hi = agreement_band
    flagged = not (lo <= ratio <= hi)

    return WaypointMCResult(
        sigma_along_normal_mc_m=sigma_mc, ratio_along_normal=ratio, flagged=flagged,
    )

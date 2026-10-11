"""crackvision.uncertainty_mc — §14/ADR-018 §4 Monte-Carlo cross-check (GEOM-09.3.1).

Companion to `crackvision.uncertainty` (GEOM-09.2): instead of linearizing pixel/depth/
hand-eye/FK noise through a closed-form Jacobian, this module draws seeded samples of those
four terms and reruns the *real* geometry pipeline on each one — `crackvision.lift3d.lift_polyline`
(annulus surface-depth sampling, outward-oriented plane-fit normal, §10.3/§10.4 covariance) for
the point case, and `crackvision.tool_waypoints.segment_waypoints` (real `+X = -n_out` roll-axis
frame, clearance offset, `sigma_along_normal`/`within_budget`) for the waypoint case — then
compares the resulting empirical covariance/sigma against a caller-supplied analytic breakdown
(e.g. `uncertainty.PointBreakdown.total` or `uncertainty.WaypointBreakdown.total`).

Each sample perturbs pixel row/col, depth, the hand-eye extrinsic (rotation+translation) and the
FK joint angles all at once, then reruns `lift3d.lift_polyline` on a synthetic one-pixel
`paths.json`-style polyline: a small depth patch encoding a fixed real-world surface (flat for
`mc_point_covariance`, tilted to the caller's nominal normal for `mc_waypoint_sigma`), queried at
the perturbed pixel with the per-sample depth-sensor noise added uniformly across the patch (so
the real annulus median/plane fit sees exactly the modelled per-sample pixel/depth error — never
perturbs/recomputes a term in isolation and sums variances, which would just reintroduce the
analytic linearization this module exists to check against), and transformed through the
per-sample perturbed hand-eye/FK extrinsic. `lift_polyline`'s own `calib` hand-eye term is passed
as zero because the hand-eye perturbation is already applied to the extrinsic itself.

The `tcp` term is the one exception: §14.2 documents it has no separable point-level recompute
— it is the robot's physical tool-offset residual, not a function of pixel/depth/extrinsic/joint
inputs — so it cannot be "rerun" the way the other four can. Instead it is modelled as a
directly-added isotropic `N(0, sigma_tool_pos_m^2 I)` 3D perturbation applied to the real
`tool_waypoints.segment_waypoints` tool_tip position *after* the real waypoint geometry (surface
point + clearance along the plane-fit normal) has been computed from a perturbed sample.
`mc_point_covariance` therefore has no `tcp` parameter at all — a bare lifted point has no tool
offset to perturb.

Pure library — no CLI, no file I/O, no argparse. Only imports from
`crackvision.{geometry,kinematics,lift3d,tool_waypoints}` plus numpy/scipy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from crackvision import geometry, kinematics, lift3d, tool_waypoints

# Fine enough (10 micron steps) that depth quantization is negligible next to the modelled
# depth-sensor sigma_z at every distance these tests exercise, while keeping `depth_m /
# _DEPTH_SCALE_M` comfortably inside uint16 range for the whole valid depth band.
_DEPTH_SCALE_M = 1.0e-5

# A single-point "polyline": `lift3d._apply_gap_policy`'s default `min_segment_points=3` would
# drop a lone measured point as too-short, so it is lowered to 1 here. Every other `LiftParams`
# field is left at its real default.
_LIFT_PARAMS = lift3d.LiftParams(min_segment_points=1)

# Hand-eye noise is injected straight into `T_base_optical_sample` (see `_perturbed_T_base_optical`);
# zero here so `lift_polyline`'s own `calib` term doesn't double-count it in the `cov_base` this
# module never reads anyway (the MC empirical covariance is computed from `p_base` across samples).
_ZERO_CALIB = lift3d.CalibSigmas(cam_pos_m=0.0, cam_rot_rad=0.0)

# A flat patch facing the camera (`lift3d`'s own camera-facing convention: `normal . -p_opt > 0`
# for a point roughly along +Z) — used by `mc_point_covariance`, which never reads the fitted
# normal, so its orientation is otherwise arbitrary.
_FLAT_NORMAL_OPTICAL = np.array([0.0, 0.0, -1.0])

# Seed for the roll-axis propagation in `tool_waypoints.segment_waypoints`; the resulting
# quaternion is unused by this module (only `Waypoint.position` is read), so its value is
# otherwise arbitrary.
_CAPTURE_TOOL_Z = np.array([0.0, 0.0, 1.0])


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


def _perturbed_T_base_optical(
    chain: "kinematics.KinematicChain",
    q_nominal: np.ndarray,
    T_gripper_camera: np.ndarray,
    T_camera_optical: np.ndarray,
    d_theta: np.ndarray,
    d_t: np.ndarray,
    d_q: np.ndarray,
) -> np.ndarray:
    """One perturbed `T_base_optical`: a right-perturbation of the hand-eye extrinsic composed
    through the same `chain.fk(q) @ T_gripper_link_camera_link @ T_camera_optical` chain
    `uncertainty.py`'s callers build, with `chain.fk` rerun at the perturbed joint angles."""
    delta_T = np.eye(4)
    delta_T[:3, :3] = Rotation.from_rotvec(d_theta).as_matrix()
    delta_T[:3, 3] = d_t
    T_cam_sample = T_gripper_camera @ delta_T
    return chain.fk(q_nominal + d_q) @ T_cam_sample @ T_camera_optical


def _lift_one_sample(
    *,
    row: int,
    col: int,
    depth_m: float,
    intrinsics: geometry.Intrinsics,
    normal_optical_nominal: np.ndarray,
    d_row: float,
    d_col: float,
    d_z: float,
    T_base_optical_sample: np.ndarray,
    depth_u16_buf: np.ndarray,
    mask_bool_buf: np.ndarray,
) -> lift3d.LiftedPolyline:
    """Build a synthetic one-pixel `paths.json`-style polyline for one perturbed sample and lift
    it through the real `lift3d.lift_polyline` annulus/plane-fit/covariance pipeline.

    The real-world surface is a plane with (camera-optical-frame) normal
    `normal_optical_nominal`, anchored at the noiseless `(row, col, depth_m)` — fixed across
    every sample, since hand-eye/FK noise is an error in where the camera *thinks* it is, not in
    where the real surface is. `(d_row, d_col)` perturb which pixel is queried (rounded to int,
    as real dense-path pixels are); `d_z` is depth-sensor noise added uniformly across the patch
    so the real annulus median sees exactly the modelled per-sample depth error. Writes into the
    caller-owned `depth_u16_buf`/`mask_bool_buf` (reused across samples — each sample fully
    overwrites its own annulus window before reading it back, so no cross-sample leakage).
    """
    row_s = int(round(row + d_row))
    col_s = int(round(col + d_col))
    margin = _LIFT_PARAMS.annulus_outer_px + 1
    height, width = depth_u16_buf.shape
    if not (margin <= row_s < height - margin and margin <= col_s < width - margin):
        raise ValueError("perturbed pixel too close to the synthetic patch buffer's edge")

    local = np.arange(-margin, margin + 1)
    rr = row_s + local[:, None]
    cc = col_s + local[None, :]

    p0 = geometry.deproject_pixels(intrinsics, row, col, depth_m)
    rhs = float(np.dot(normal_optical_nominal, p0))
    denom = (
        normal_optical_nominal[0] * (cc - intrinsics.ppx) / intrinsics.fx
        + normal_optical_nominal[1] * (rr - intrinsics.ppy) / intrinsics.fy
        + normal_optical_nominal[2]
    )
    z_ideal = rhs / denom
    raw = np.clip(np.round((z_ideal + d_z) / _DEPTH_SCALE_M), 1, 65535).astype(np.uint16)
    depth_u16_buf[row_s - margin : row_s + margin + 1, col_s - margin : col_s + margin + 1] = raw

    lifted = lift3d.lift_polyline(
        points_rc=[(row_s, col_s)],
        depth_u16=depth_u16_buf,
        mask_bool=mask_bool_buf,
        intrinsics=intrinsics,
        depth_scale=_DEPTH_SCALE_M,
        T_base_link_optical=T_base_optical_sample,
        calib=_ZERO_CALIB,
        params=_LIFT_PARAMS,
    )
    if not bool(lifted.valid[0]):
        raise RuntimeError(f"synthetic MC sample failed to lift (reason={lifted.reason[0]!r})")
    return lifted


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

    Draws `n_samples` perturbed realizations of pixel/depth/hand-eye/FK, relifts a synthetic
    flat patch through the real `lift3d.lift_polyline` for each, and compares the resulting
    empirical covariance of `p_base` against `analytic_cov_base` (e.g.
    `uncertainty.PointBreakdown.total`). `seed` fixes a `Generator(PCG64(seed))` so the result
    is bit-for-bit reproducible.
    """
    rng = np.random.Generator(np.random.PCG64(seed))
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

    depth_u16_buf = np.zeros((intrinsics.height, intrinsics.width), dtype=np.uint16)
    mask_bool_buf = np.zeros_like(depth_u16_buf, dtype=bool)

    p_base = np.empty((n_samples, 3), dtype=np.float64)
    for i in range(n_samples):
        T_sample = _perturbed_T_base_optical(
            chain, q_nominal, T_gripper_camera, T_camera_optical, d_theta[i], d_t[i], d_q[i],
        )
        lifted = _lift_one_sample(
            row=row, col=col, depth_m=depth_m, intrinsics=intrinsics,
            normal_optical_nominal=_FLAT_NORMAL_OPTICAL,
            d_row=float(d_rowcol[i, 0]), d_col=float(d_rowcol[i, 1]), d_z=float(d_z[i]),
            T_base_optical_sample=T_sample,
            depth_u16_buf=depth_u16_buf, mask_bool_buf=mask_bool_buf,
        )
        p_base[i] = lifted.p_base[0]

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

    Perturbs the same four terms as `mc_point_covariance` for the waypoint's source point, relifts
    a synthetic patch tilted to `normal` through `lift3d.lift_polyline`, and builds the tool_tip
    position via the real `tool_waypoints.segment_waypoints` (its single "trace" waypoint) for
    each sample. Since `tcp` has no recompute (module docstring), an isotropic
    `N(0, tool_sigma_pos_m^2 I)` perturbation is then added directly to that real tool_tip
    position. Returns the sample stddev of the signed along-normal displacement relative to the
    noiseless nominal tool_tip, and its ratio against `analytic_sigma_along_normal_m` (e.g.
    `uncertainty.WaypointBreakdown.total ** 0.5`, or the accepted `sigma_along_normal_m` it is
    compared against in §14.5).
    """
    rng = np.random.Generator(np.random.PCG64(seed))
    q_nominal = np.asarray(q, dtype=np.float64)
    n_joints = q_nominal.shape[0]
    T_gripper_camera = np.asarray(T_gripper_camera, dtype=np.float64)
    T_camera_optical = np.asarray(T_camera_optical, dtype=np.float64)
    n_base_fixed = np.asarray(normal, dtype=np.float64)

    T_base_optical_nominal = chain.fk(q_nominal) @ T_gripper_camera @ T_camera_optical
    normal_optical_nominal = T_base_optical_nominal[:3, :3].T @ n_base_fixed
    normal_optical_nominal = normal_optical_nominal / np.linalg.norm(normal_optical_nominal)

    d_rowcol = rng.normal(0.0, sigma_px, size=(n_samples, 2))
    sigma_z = float(geometry.depth_uncertainty_m(depth_m, intrinsics.fx))
    d_z = rng.normal(0.0, sigma_z, size=n_samples)
    d_theta = rng.normal(0.0, cam_rot_rad, size=(n_samples, 3))
    d_t = rng.normal(0.0, cam_pos_m, size=(n_samples, 3))
    d_q = rng.normal(0.0, sigma_joint_rad, size=(n_samples, n_joints))
    d_tool = rng.normal(0.0, tool_sigma_pos_m, size=(n_samples, 3))

    depth_u16_buf = np.zeros((intrinsics.height, intrinsics.width), dtype=np.uint16)
    mask_bool_buf = np.zeros_like(depth_u16_buf, dtype=bool)
    wp_params = tool_waypoints.WaypointParams(
        trace_clearance_m=clearance_m, approach_clearance_m=clearance_m, waypoint_spacing_m=1.0,
    )

    nominal_lifted = _lift_one_sample(
        row=row, col=col, depth_m=depth_m, intrinsics=intrinsics,
        normal_optical_nominal=normal_optical_nominal,
        d_row=0.0, d_col=0.0, d_z=0.0, T_base_optical_sample=T_base_optical_nominal,
        depth_u16_buf=depth_u16_buf, mask_bool_buf=mask_bool_buf,
    )
    tool_tip_nominal = tool_waypoints.segment_waypoints(
        nominal_lifted, (0, 0), _CAPTURE_TOOL_Z, 0.0, wp_params,
    )[1].position

    displacement = np.empty(n_samples, dtype=np.float64)
    for i in range(n_samples):
        T_sample = _perturbed_T_base_optical(
            chain, q_nominal, T_gripper_camera, T_camera_optical, d_theta[i], d_t[i], d_q[i],
        )
        lifted = _lift_one_sample(
            row=row, col=col, depth_m=depth_m, intrinsics=intrinsics,
            normal_optical_nominal=normal_optical_nominal,
            d_row=float(d_rowcol[i, 0]), d_col=float(d_rowcol[i, 1]), d_z=float(d_z[i]),
            T_base_optical_sample=T_sample,
            depth_u16_buf=depth_u16_buf, mask_bool_buf=mask_bool_buf,
        )
        trace_waypoint = tool_waypoints.segment_waypoints(
            lifted, (0, 0), _CAPTURE_TOOL_Z, 0.0, wp_params,
        )[1]
        tool_tip_sample = trace_waypoint.position + d_tool[i]
        displacement[i] = float(np.dot(tool_tip_sample - tool_tip_nominal, n_base_fixed))

    sigma_mc = float(np.std(displacement, ddof=1))
    ratio = sigma_mc / float(analytic_sigma_along_normal_m)
    lo, hi = agreement_band
    flagged = not (lo <= ratio <= hi)

    return WaypointMCResult(
        sigma_along_normal_mc_m=sigma_mc, ratio_along_normal=ratio, flagged=flagged,
    )

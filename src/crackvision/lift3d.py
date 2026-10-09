"""crackvision.lift3d — lift ordered pixel polylines to base_link surface points (GEOM-08.5).

Implements `docs/INTERFACES.md` §10.3/§10.4's per-point lifting policy: surface depth from the
annulus sampler (never the crack-cavity depth under the mask), an outward-oriented plane-fit
normal, a first-order covariance, and the §10.3 gap interpolation/segmentation/trimming/drop
rules. Pure library — no CLI, no file I/O. Waypoint resampling (§10.5) is out of scope here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from crackvision import geometry

NORMAL_FIT_FAILED = "normal_fit_failed"
SEGMENT_TOO_SHORT = "segment_too_short"


@dataclass
class LiftParams:
    annulus_inner_px: int = 3
    annulus_outer_px: int = 8
    min_fraction_valid: float = 0.3
    valid_range_m: tuple[float, float] = geometry.DEFAULT_VALID_DEPTH_RANGE_M
    pixel_sigma_px: float = 1.0
    max_gap_px: int = 5
    min_segment_points: int = 3
    min_normal_points: int = 6


@dataclass
class CalibSigmas:
    """§10.4 calibration sigmas for the wrist camera, typically `kinematics.EndEffector`'s
    `camera_sigma_pos_m`/`camera_sigma_rot_rad`."""

    cam_pos_m: float
    cam_rot_rad: float


@dataclass
class LiftedPolyline:
    rows: np.ndarray
    cols: np.ndarray
    valid: np.ndarray
    reason: np.ndarray
    interpolated: np.ndarray
    depth_m: np.ndarray
    fraction_valid: np.ndarray
    p_optical: np.ndarray
    p_base: np.ndarray
    n_base: np.ndarray
    normal_rms_m: np.ndarray
    sigma_normal_rad: np.ndarray
    cov_base: np.ndarray
    segments: list[tuple[int, int]] = field(default_factory=list)
    dropped_segments: int = 0


def _annulus_pixels(
    row: int, col: int, height: int, width: int, inner_px: int, outer_px: int, mask: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Same ring/in-bounds/non-mask geometry as `geometry.sample_surface_depth_annulus`.

    Reimplemented locally (not imported) because the sampler only returns the aggregated
    depth, not the candidate pixel coordinates the normal fit needs.
    """
    r0 = max(0, row - outer_px)
    r1 = min(height, row + outer_px + 1)
    c0 = max(0, col - outer_px)
    c1 = min(width, col + outer_px + 1)

    rr, cc = np.meshgrid(np.arange(r0, r1), np.arange(c0, c1), indexing="ij")
    dist2 = (rr - row) ** 2 + (cc - col) ** 2
    ring = (dist2 >= inner_px**2) & (dist2 <= outer_px**2)
    ring &= ~mask[r0:r1, c0:c1]
    return rr[ring], cc[ring]


def _apply_gap_policy(
    valid: np.ndarray,
    reason: np.ndarray,
    interpolated: np.ndarray,
    depth_m: np.ndarray,
    p_optical: np.ndarray,
    p_base: np.ndarray,
    n_base: np.ndarray,
    normal_rms_m: np.ndarray,
    sigma_normal_rad: np.ndarray,
    cov_base: np.ndarray,
    params: LiftParams,
    T_optical_base: np.ndarray,
) -> tuple[list[tuple[int, int]], int]:
    """§10.3 gap interpolation, splitting, trimming and minimum-segment-length policy.

    Mutates every array in place to its final state and returns `(segments, dropped_segments)`.
    """
    n = valid.shape[0]
    original_valid = valid.copy()
    pre_interp_reason = reason.copy()
    covered = valid.copy()
    interp_mask = np.zeros(n, dtype=bool)

    i = 0
    while i < n:
        if original_valid[i]:
            i += 1
            continue
        j = i
        while j < n and not original_valid[j]:
            j += 1
        run_start, run_end = i, j - 1
        is_leading = run_start == 0
        is_trailing = run_end == n - 1
        run_len = run_end - run_start + 1
        if not is_leading and not is_trailing and run_len <= params.max_gap_px:
            left, right = run_start - 1, run_end + 1
            conservative = left if np.trace(cov_base[left]) >= np.trace(cov_base[right]) else right
            src_cov = cov_base[conservative]
            src_rms = normal_rms_m[conservative]
            src_sigma_normal = sigma_normal_rad[conservative]
            span = right - left
            for k in range(run_start, run_end + 1):
                t = (k - left) / span
                p_base[k] = (1 - t) * p_base[left] + t * p_base[right]
                nlerp = (1 - t) * n_base[left] + t * n_base[right]
                norm = np.linalg.norm(nlerp)
                n_base[k] = nlerp / norm if norm > 1e-12 else nlerp
                cov_base[k] = src_cov
                normal_rms_m[k] = src_rms
                sigma_normal_rad[k] = src_sigma_normal
                p_opt_h = T_optical_base @ np.array(
                    [p_base[k][0], p_base[k][1], p_base[k][2], 1.0]
                )
                p_optical[k] = p_opt_h[:3]
                depth_m[k] = p_optical[k][2]
                reason[k] = f"interpolated:{pre_interp_reason[k]}"
                interpolated[k] = True
                valid[k] = True
                covered[k] = True
                interp_mask[k] = True
        # leading/trailing runs are trimmed (left uncovered, never bridged/extrapolated);
        # interior runs longer than max_gap_px are left uncovered too (they split the polyline).
        i = j

    segments: list[tuple[int, int]] = []
    dropped_segments = 0
    i = 0
    while i < n:
        if not covered[i]:
            i += 1
            continue
        j = i
        while j < n and covered[j]:
            j += 1
        start, end = i, j - 1
        measured = int(np.sum(original_valid[start : end + 1] & ~interp_mask[start : end + 1]))
        if measured < params.min_segment_points:
            dropped_segments += 1
            for k in range(start, end + 1):
                if interp_mask[k]:
                    reason[k] = pre_interp_reason[k]
                    interpolated[k] = False
                else:
                    reason[k] = SEGMENT_TOO_SHORT
                valid[k] = False
                depth_m[k] = np.nan
                p_optical[k] = np.nan
                p_base[k] = np.nan
                n_base[k] = np.nan
                normal_rms_m[k] = np.nan
                sigma_normal_rad[k] = np.nan
                cov_base[k] = np.nan
        else:
            segments.append((start, end))
        i = j

    return segments, dropped_segments


def lift_polyline(
    points_rc: Sequence[tuple[int, int]],
    depth_u16: np.ndarray,
    mask_bool: np.ndarray,
    intrinsics: geometry.Intrinsics,
    depth_scale: float,
    T_base_link_optical: np.ndarray,
    calib: CalibSigmas,
    params: LiftParams | None = None,
) -> LiftedPolyline:
    """Lift one dense `paths.json` polyline to `base_link` surface points with normals.

    `points_rc` is a sequence of `(row, col)` array-convention pixel coordinates, dense order
    (§3.13). Per point: §10.3 surface depth + normal, §10.4 covariance, then the §10.3 gap
    policy over the whole polyline. Never produces a point at the camera origin — invalid
    points carry NaN geometry and a non-empty `reason`.
    """
    if params is None:
        params = LiftParams()

    pts = np.asarray(list(points_rc), dtype=np.int64).reshape(-1, 2)
    n = pts.shape[0]
    rows = pts[:, 0].copy()
    cols = pts[:, 1].copy()

    depth_u16 = np.asarray(depth_u16)
    mask_bool = np.asarray(mask_bool, dtype=bool)
    height, width = depth_u16.shape

    valid = np.zeros(n, dtype=bool)
    reason = np.full(n, "", dtype=object)
    interpolated = np.zeros(n, dtype=bool)
    depth_m = np.full(n, np.nan, dtype=np.float64)
    fraction_valid = np.full(n, np.nan, dtype=np.float64)
    p_optical = np.full((n, 3), np.nan, dtype=np.float64)
    p_base = np.full((n, 3), np.nan, dtype=np.float64)
    n_base = np.full((n, 3), np.nan, dtype=np.float64)
    normal_rms_m = np.full(n, np.nan, dtype=np.float64)
    sigma_normal_rad = np.full(n, np.nan, dtype=np.float64)
    cov_base = np.full((n, 3, 3), np.nan, dtype=np.float64)

    R = np.asarray(T_base_link_optical)[:3, :3]

    for i in range(n):
        row = int(rows[i])
        col = int(cols[i])

        ann = geometry.sample_surface_depth_annulus(
            depth_u16,
            mask_bool,
            row,
            col,
            depth_scale,
            params.annulus_inner_px,
            params.annulus_outer_px,
            valid_range_m=params.valid_range_m,
            min_fraction_valid=params.min_fraction_valid,
        )
        fraction_valid[i] = ann.fraction_valid
        if not ann.valid:
            reason[i] = ann.reason
            continue

        p_opt = geometry.deproject_pixels(intrinsics, row, col, ann.depth_m)
        x, y, z = float(p_opt[0]), float(p_opt[1]), float(p_opt[2])

        ring_rows, ring_cols = _annulus_pixels(
            row, col, height, width, params.annulus_inner_px, params.annulus_outer_px, mask_bool
        )
        raw_ring_depth = depth_u16[ring_rows, ring_cols]
        ring_classification = geometry.classify_depth(raw_ring_depth, depth_scale, params.valid_range_m)
        if int(ring_classification.valid.sum()) < params.min_normal_points:
            reason[i] = NORMAL_FIT_FAILED
            continue

        vr = ring_rows[ring_classification.valid]
        vc = ring_cols[ring_classification.valid]
        vz = ring_classification.depth_m[ring_classification.valid]
        annulus_points = geometry.deproject_pixels(intrinsics, vr, vc, vz)
        fit = geometry.fit_plane(annulus_points)

        normal = fit.normal
        # geometry.fit_plane canonicalises n_z >= 0, which points away from the camera -- never
        # rely on that sign, always re-orient explicitly toward the camera (docs §10.3 pitfall).
        if float(np.dot(normal, -p_opt)) <= 0.0:
            normal = -normal

        sigma_normal = fit.rms_residual_m / (params.annulus_outer_px * z / intrinsics.fx)

        sigma_z = float(geometry.depth_uncertainty_m(z, intrinsics.fx))
        J = np.array(
            [
                [z / intrinsics.fx, 0.0, x / z],
                [0.0, z / intrinsics.fy, y / z],
                [0.0, 0.0, 1.0],
            ]
        )
        cov_pix = np.diag([params.pixel_sigma_px**2, params.pixel_sigma_px**2, sigma_z**2])
        sigma_opt = J @ cov_pix @ J.T

        p_base_i = (np.asarray(T_base_link_optical) @ np.array([x, y, z, 1.0]))[:3]
        n_base_raw = R @ normal
        n_base_i = n_base_raw / np.linalg.norm(n_base_raw)

        calib_term = calib.cam_pos_m**2 + (float(np.linalg.norm(p_opt)) * calib.cam_rot_rad) ** 2
        sigma_base = R @ sigma_opt @ R.T + calib_term * np.eye(3)

        valid[i] = True
        reason[i] = ""
        depth_m[i] = ann.depth_m
        p_optical[i] = p_opt
        p_base[i] = p_base_i
        n_base[i] = n_base_i
        normal_rms_m[i] = fit.rms_residual_m
        sigma_normal_rad[i] = sigma_normal
        cov_base[i] = sigma_base

    T_optical_base = np.linalg.inv(np.asarray(T_base_link_optical))
    segments, dropped_segments = _apply_gap_policy(
        valid, reason, interpolated, depth_m, p_optical, p_base, n_base,
        normal_rms_m, sigma_normal_rad, cov_base, params, T_optical_base,
    )

    return LiftedPolyline(
        rows=rows,
        cols=cols,
        valid=valid,
        reason=reason,
        interpolated=interpolated,
        depth_m=depth_m,
        fraction_valid=fraction_valid,
        p_optical=p_optical,
        p_base=p_base,
        n_base=n_base,
        normal_rms_m=normal_rms_m,
        sigma_normal_rad=sigma_normal_rad,
        cov_base=cov_base,
        segments=segments,
        dropped_segments=dropped_segments,
    )

"""crackvision.uncertainty — §14.2/§14.4/§14.5 analytic per-term uncertainty decomposition
(GEOM-09.2).

Implements ADR-018 §2-§3 and `docs/INTERFACES.md` §14 on top of an already-lifted
`lift3d.LiftedPolyline`/`tool_waypoints.Waypoint`: splits a measured point's §10.4 `cov_base`
into its `pixel`/`depth`/`handeye` sources (replicating `lift3d.lift_polyline`'s own formula,
never recomputing a different one), adds the new `fk` term via a central finite-difference FK
Jacobian, and maps every waypoint/interpolated point to its §14.5/§10.3 measured source point.
Pure library — no CLI, no file I/O, no mutation of `lift3d`/`tool_waypoints`/`kinematics`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np

from crackvision import geometry, lift3d


@dataclass
class PointBreakdown:
    """§14.2/§14.4 per-term 3x3 `Σ_base^(term)` contributions, m²."""

    pixel: np.ndarray
    depth: np.ndarray
    handeye: np.ndarray
    fk: np.ndarray

    @property
    def total(self) -> np.ndarray:
        return self.pixel + self.depth + self.handeye + self.fk


@dataclass
class WaypointBreakdown:
    """§14.5 per-term scalar (`nᵀ·Σ_base^(term)·n`, plus `tcp = σ_tool_pos²`) contributions, m²."""

    pixel: float
    depth: float
    handeye: float
    fk: float
    tcp: float

    @property
    def total(self) -> float:
        return self.pixel + self.depth + self.handeye + self.fk + self.tcp


def decompose_point(
    p_optical: Sequence[float],
    fx: float,
    fy: float,
    R_base_optical: np.ndarray,
    sigma_px: float,
    calib: "lift3d.CalibSigmas",
    *,
    valid: bool,
    interpolated: bool,
    cov_base: np.ndarray,
    fk_cov: np.ndarray | None = None,
) -> PointBreakdown:
    """Decompose one measured point's §10.4 `cov_base` into pixel/depth/handeye (+ fk).

    Mirrors `lift3d.lift_polyline` (lift3d.py ~257-280) exactly: `cov_pix` is split into its
    pixel (`diag(σ_px², σ_px², 0)`) and depth (`diag(0, 0, σ_z²)`) halves *before* the shared
    `J`/`R` sandwich, so `pixel + depth + handeye == cov_base` for this same point.
    """
    if not valid or np.any(np.isnan(cov_base)):
        raise ValueError("decompose_point requires a valid point with finite cov_base")
    if interpolated:
        raise ValueError(
            "decompose_point refuses interpolated points: decompose their §10.3 measured "
            "source point instead (see measured_source_index)"
        )

    p_opt = np.asarray(p_optical, dtype=np.float64)
    x, y, z = float(p_opt[0]), float(p_opt[1]), float(p_opt[2])
    R = np.asarray(R_base_optical, dtype=np.float64)[:3, :3]

    sigma_z = float(geometry.depth_uncertainty_m(z, fx))
    J = np.array(
        [
            [z / fx, 0.0, x / z],
            [0.0, z / fy, y / z],
            [0.0, 0.0, 1.0],
        ]
    )
    cov_pix_pixel = np.diag([sigma_px**2, sigma_px**2, 0.0])
    cov_pix_depth = np.diag([0.0, 0.0, sigma_z**2])

    pixel = R @ (J @ cov_pix_pixel @ J.T) @ R.T
    depth = R @ (J @ cov_pix_depth @ J.T) @ R.T

    calib_term = calib.cam_pos_m**2 + (float(np.linalg.norm(p_opt)) * calib.cam_rot_rad) ** 2
    handeye = calib_term * np.eye(3)

    fk = np.zeros((3, 3)) if fk_cov is None else np.asarray(fk_cov, dtype=np.float64)

    return PointBreakdown(pixel=pixel, depth=depth, handeye=handeye, fk=fk)


def fk_covariance(
    T_base_optical_of_q: Callable[[np.ndarray], np.ndarray],
    q: Sequence[float],
    p_optical: Sequence[float],
    sigma_joint_rad: float,
    step: float = 1e-6,
) -> np.ndarray:
    """ADR-018 §3 `Σ_base^(fk) = J_fk · J_fk^T · σ_joint_rad²`.

    `J_fk` is a central finite-difference Jacobian (3xN, `step` rad per joint) of
    `p_base(q) = T_base_optical(q) · p_optical` through `T_base_optical_of_q` — built by the
    caller from `kinematics.KinematicChain.fk` composed the same way
    `capture_record.T_base_link_optical` does. Exactly `np.zeros((3,3))` when
    `sigma_joint_rad == 0` (the `σ²` factor zeroes any finite Jacobian), scales with `σ²`, and
    is PSD by construction (`J_fk · J_fk^T`).
    """
    q = np.asarray(q, dtype=np.float64)
    n = q.shape[0]
    p_opt_h = np.array([*p_optical, 1.0], dtype=np.float64)

    J_fk = np.zeros((3, n))
    for i in range(n):
        dq = np.zeros(n)
        dq[i] = step
        p_plus = (T_base_optical_of_q(q + dq) @ p_opt_h)[:3]
        p_minus = (T_base_optical_of_q(q - dq) @ p_opt_h)[:3]
        J_fk[:, i] = (p_plus - p_minus) / (2.0 * step)

    return (J_fk @ J_fk.T) * (sigma_joint_rad**2)


def decompose_waypoint(
    point_breakdown: PointBreakdown, normal: Sequence[float], tool_sigma_pos_m: float
) -> WaypointBreakdown:
    """§14.5 scalar waypoint terms: `nᵀ·Σ_base^(term)·n` for pixel/depth/handeye/fk, plus
    `tcp = σ_tool_pos²`. `.total` equals the new `sigma_along_normal_m²`."""
    n = np.asarray(normal, dtype=np.float64)
    return WaypointBreakdown(
        pixel=float(n @ point_breakdown.pixel @ n),
        depth=float(n @ point_breakdown.depth @ n),
        handeye=float(n @ point_breakdown.handeye @ n),
        fk=float(n @ point_breakdown.fk @ n),
        tcp=float(tool_sigma_pos_m**2),
    )


def waypoint_source_indices(
    p_base: np.ndarray, seg: tuple[int, int], spacing_m: float
) -> list[int]:
    """§14.5 dense-point index whose terms each waypoint (approach, trace..., retract)
    inherits, in `tool_waypoints.segment_waypoints` order.

    Reproduces `tool_waypoints._resample_segment`'s arc-length resampling and its
    `nearer = j-1 if t <= 0.5 else j` rule (tool_waypoints.py:58-97) on plain `p_base` dense
    points, and `segment_waypoints`'s approach/retract bracket (first/last trace sample,
    tool_waypoints.py:144-183) — operating only on arrays so GEOM-09.4 can call it on
    `paths3d` JSON data directly.
    """
    start, end = seg
    idx = list(range(start, end + 1))
    p = np.asarray(p_base, dtype=np.float64)[idx]
    k = len(idx)

    seg_lens = np.zeros(k)
    if k > 1:
        seg_lens[1:] = np.linalg.norm(np.diff(p, axis=0), axis=1)
    cum = np.cumsum(seg_lens)
    total = float(cum[-1]) if k > 0 else 0.0

    if total < 1e-12:
        samples = [0.0]
    else:
        samples = list(np.arange(0.0, total, spacing_m))
        if samples[-1] < total - 1e-9:
            samples.append(total)
        else:
            samples[-1] = total

    trace_indices: list[int] = []
    for s in samples:
        j = int(np.searchsorted(cum, s, side="right"))
        j = min(max(j, 1), k - 1) if k > 1 else 0
        if k == 1:
            trace_indices.append(idx[0])
            continue
        s0, s1 = cum[j - 1], cum[j]
        span = s1 - s0
        t = 0.0 if span < 1e-12 else (s - s0) / span
        t = min(max(t, 0.0), 1.0)
        nearer = j - 1 if t <= 0.5 else j
        trace_indices.append(idx[nearer])

    return [trace_indices[0], *trace_indices, trace_indices[-1]]


def measured_source_index(
    valid: np.ndarray, interpolated: np.ndarray, cov_base: np.ndarray, k: int
) -> int:
    """§10.3/§14.5 measured source index for dense point `k`.

    If `k` is itself measured, returns `k`. If `k` is interpolated, walks to the nearest
    measured neighbour on each side (the gap-fill's `left`/`right`, lift3d.py:118-124) and
    returns `left if trace(cov_base[left]) >= trace(cov_base[right]) else right` (ties ->
    left), exactly as `lift3d._apply_gap_policy` (lift3d.py:120).
    """
    n = len(valid)
    if k < 0 or k >= n or not valid[k]:
        raise ValueError(f"point {k} is invalid; it has no measured source point")
    if not interpolated[k]:
        return k

    left = k - 1
    while left >= 0 and not (valid[left] and not interpolated[left]):
        left -= 1
    right = k + 1
    while right < n and not (valid[right] and not interpolated[right]):
        right += 1
    if left < 0 or right >= n:
        raise ValueError(f"interpolated point {k} is missing a measured bounding neighbour")

    return left if np.trace(cov_base[left]) >= np.trace(cov_base[right]) else right

"""crackvision.tool_waypoints — segment resampling + §10.5 tool_tip waypoints (GEOM-08.6).

Implements `docs/INTERFACES.md` §10.5's waypoint policy on top of a GEOM-08.5 `LiftedPolyline`
segment: 3D-arc-length resampling, the `+X = -n_out` / parallel-transported `+Z` roll rule, the
approach/retract bracket at `approach_retract_offset_m`, and the §10.4 `sigma_along_normal`/
`within_budget` per-waypoint uncertainty. Pure library — no CLI, no file I/O.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from crackvision import kinematics, lift3d

_DEGENERATE_NORM = 1e-6

# §10.5 fallback chain for a degenerate reference-Z projection.
_FALLBACK_AXES = (np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]))


@dataclass
class WaypointParams:
    trace_clearance_m: float = 0.01
    approach_clearance_m: float = 0.04
    waypoint_spacing_m: float = 0.002


@dataclass
class Waypoint:
    phase: str
    position: np.ndarray
    quat_xyzw: np.ndarray
    surface_point: np.ndarray
    normal: np.ndarray
    clearance_m: float
    sigma_along_normal_m: float
    sigma_max_m: float
    interpolated: bool
    within_budget: bool


def capture_tool_z(chain: "kinematics.KinematicChain", end_effector: "kinematics.EndEffector", q) -> np.ndarray:
    """The Z column of `FK(q) @ T_gripper_link_tool_tip`, in `base_link`."""
    T_base_gripper = chain.fk(q)
    T_base_tool_tip = T_base_gripper @ end_effector.T_gripper_link_tool_tip
    return T_base_tool_tip[:3, 2].copy()


def _nlerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    v = (1.0 - t) * a + t * b
    norm = np.linalg.norm(v)
    return v / norm if norm > 1e-12 else v


def _resample_segment(lifted: "lift3d.LiftedPolyline", seg: tuple[int, int], spacing_m: float):
    start, end = seg
    idx = list(range(start, end + 1))
    p = lifted.p_base[idx]
    n = lifted.n_base[idx]
    cov = lifted.cov_base[idx]
    interp = lifted.interpolated[idx]

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

    results = []
    for s in samples:
        j = int(np.searchsorted(cum, s, side="right"))
        j = min(max(j, 1), k - 1) if k > 1 else 0
        if k == 1:
            results.append((p[0], n[0], cov[0], bool(interp[0])))
            continue
        s0, s1 = cum[j - 1], cum[j]
        span = s1 - s0
        t = 0.0 if span < 1e-12 else (s - s0) / span
        t = min(max(t, 0.0), 1.0)
        pos = (1.0 - t) * p[j - 1] + t * p[j]
        nrm = _nlerp(n[j - 1], n[j], t)
        nearer = j - 1 if t <= 0.5 else j
        results.append((pos, nrm, cov[nearer], bool(interp[j - 1]) or bool(interp[j])))
    return results


def _propagate_z(prev_z: np.ndarray, x_axis: np.ndarray) -> np.ndarray:
    z0 = prev_z - np.dot(prev_z, x_axis) * x_axis
    norm = np.linalg.norm(z0)
    if norm >= _DEGENERATE_NORM:
        return z0 / norm
    for fallback in _FALLBACK_AXES:
        z0 = fallback - np.dot(fallback, x_axis) * x_axis
        norm = np.linalg.norm(z0)
        if norm >= _DEGENERATE_NORM:
            return z0 / norm
    raise ValueError("all §10.5 roll fallbacks degenerate against this waypoint's +X")


def _waypoint(
    phase: str,
    surface_point: np.ndarray,
    normal: np.ndarray,
    quat_xyzw: np.ndarray,
    cov: np.ndarray,
    interpolated: bool,
    clearance_m: float,
    tool_sigma_pos_m: float,
) -> Waypoint:
    position = surface_point + clearance_m * normal

    sigma_along_normal = float(np.sqrt(normal @ cov @ normal + tool_sigma_pos_m**2))
    eigvals = np.linalg.eigvalsh(cov)
    sigma_max = float(np.sqrt(max(eigvals.max(), 0.0) + tool_sigma_pos_m**2))
    within_budget = bool(3.0 * sigma_along_normal <= clearance_m)

    return Waypoint(
        phase=phase,
        position=position,
        quat_xyzw=quat_xyzw,
        surface_point=surface_point,
        normal=normal,
        clearance_m=clearance_m,
        sigma_along_normal_m=sigma_along_normal,
        sigma_max_m=sigma_max,
        interpolated=interpolated,
        within_budget=within_budget,
    )


def segment_waypoints(
    lifted: "lift3d.LiftedPolyline",
    seg: tuple[int, int],
    capture_tool_z_base: np.ndarray,
    tool_sigma_pos_m: float,
    params: WaypointParams | None = None,
) -> list[Waypoint]:
    """§10.5 tool_tip waypoints (approach, trace..., retract) for one lifted segment."""
    if params is None:
        params = WaypointParams()

    samples = _resample_segment(lifted, seg, params.waypoint_spacing_m)

    trace_waypoints: list[Waypoint] = []
    prev_z = np.asarray(capture_tool_z_base, dtype=np.float64)
    for pos, nrm, cov, interp in samples:
        x_axis = -nrm
        z_axis = _propagate_z(prev_z, x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        R = np.column_stack([x_axis, y_axis, z_axis])
        quat = Rotation.from_matrix(R).as_quat()
        wp = _waypoint(
            "trace", pos, nrm, quat, cov, interp, params.trace_clearance_m, tool_sigma_pos_m,
        )
        trace_waypoints.append(wp)
        prev_z = z_axis

    first, last = trace_waypoints[0], trace_waypoints[-1]
    first_sample, last_sample = samples[0], samples[-1]
    approach = _waypoint(
        "approach", first.surface_point, first.normal, first.quat_xyzw,
        first_sample[2], first.interpolated, params.approach_clearance_m, tool_sigma_pos_m,
    )
    retract = _waypoint(
        "retract", last.surface_point, last.normal, last.quat_xyzw,
        last_sample[2], last.interpolated, params.approach_clearance_m, tool_sigma_pos_m,
    )

    return [approach, *trace_waypoints, retract]

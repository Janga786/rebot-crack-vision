"""crackvision.arm_ik — tool-tip inverse kinematics for the B601-DM and Cartesian path -> joint trajectory.

Turns a tool-tip path in `base_link` (positions, optionally a pointing direction per point) into a
`crackvision.joint_trajectory/1` document, the format `~/rebot_setup/demo_move.py --trajectory` (and its wrapper
`run-trajectory.sh`) plays on the arm behind arm_guard. Used by `tools/make_fancy_trajectory.py` (show shapes) and
`crackvision.crack_trace` (camera crack -> trace).

- IK: damped least squares on the repo URDF (`crackvision.kinematics`, the same file arm_guard checks against),
  position plus an optional pointing constraint on the tool's +X axis (roll about it is left free), a null-space
  pull toward a rest pose so the redundant wrist does not wander, joints kept `limit_margin_rad` inside the URDF.
- Paths are followed with a warm start from the previous point, so the solution never jumps branch; a jump larger
  than `max_joint_step_rad` between neighbouring samples is refused, not smoothed over.
- Timing: the tool moves along the path's arc length with a cosine-ramped trapezoid (smooth start and stop, a
  constant cruise speed); `Piece(stop=True)` boundaries are full stops (a star's corners). The finished trajectory
  is slowed uniformly until every joint stays under `speed_fraction` of the runner's planned-speed caps.

Nothing here talks to hardware.
"""

from __future__ import annotations

import datetime as dt
import math
from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from crackvision import kinematics

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
TRAJECTORY_SCHEMA = "crackvision.joint_trajectory/1"
# ~/rebot_setup/demo_move.py PLAN_VEL_MAX: planned peak speed per joint it refuses above (J1-J3 DM4340P, J4-J6 DM4310)
RUNNER_SPEED_CAP_RAD_S = np.array([0.60, 0.60, 0.60, 1.20, 1.20, 1.20])
# arm_guard.toml [[keepout.floor]] gripper_link z_min; the runner's TABLE_Z_MIN is the same number
GRIPPER_FLOOR_Z_M = 0.12
DT_S = 0.05
REST_Q = np.array([0.0, -0.8, -1.0, 0.0, 0.0, 0.0])


class IKError(Exception):
    """A target (or a path sample) the arm cannot reach within tolerance, or a branch jump along a path."""


@dataclass
class IKResult:
    q: np.ndarray
    pos_err_m: float
    point_err_rad: float
    ok: bool


@dataclass
class Piece:
    """One stretch of a tool path. `points` (N, 3) in base_link; `pointing` (N, 3) unit vectors or None (free).
    `speed_m_s` is the cruise speed along it; `stop` = come to rest at its end (else blend into the next piece)."""

    points: np.ndarray
    pointing: np.ndarray | None = None
    speed_m_s: float = 0.04
    stop: bool = True
    label: str = ""


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


class ArmIK:
    def __init__(self, chain: kinematics.KinematicChain | None = None,
                 end_effector: kinematics.EndEffector | None = None, *, tool: str = "tool_tip",
                 limit_margin_rad: float = 0.05, rest_q: Sequence[float] = REST_Q) -> None:
        self.chain = chain or kinematics.load_chain()
        if tool == "tool_tip":
            ee = end_effector or kinematics.load_end_effector()
            self.T_tool = np.asarray(ee.T_gripper_link_tool_tip, dtype=float)
            self.tool_value_status = ee.tool_value_status
        elif tool == "gripper_link":
            self.T_tool = np.eye(4)
            self.tool_value_status = "urdf"
        else:
            raise ValueError(f"tool must be tool_tip or gripper_link, not {tool!r}")
        movable = [j for j in self.chain.joints if j.type != "fixed"]
        self.lo = np.array([j.lower for j in movable]) + limit_margin_rad
        self.hi = np.array([j.upper for j in movable]) - limit_margin_rad
        self.rest_q = np.clip(np.asarray(rest_q, dtype=float), self.lo, self.hi)

    # -- kinematics ----------------------------------------------------------------------------------------------
    def _walk(self, q: np.ndarray, tip: np.ndarray | None = None):
        """(T_base_tip, joint origins (6,3), joint axes in base (6,3)); tip = a transform after gripper_link."""
        T = np.eye(4)
        origins, axes, k = [], [], 0
        for j in self.chain.joints:
            T = T @ kinematics.transform_from_xyz_rpy(j.xyz, j.rpy)
            if j.type != "fixed":
                a = np.asarray(j.axis, dtype=float)
                a = a / np.linalg.norm(a)
                origins.append(T[:3, 3].copy())
                axes.append(T[:3, :3] @ a)
                R = np.eye(4)
                R[:3, :3] = Rotation.from_rotvec(a * q[k]).as_matrix()
                T = T @ R
                k += 1
        return T @ (self.T_tool if tip is None else tip), np.array(origins), np.array(axes)

    def fk(self, q: Sequence[float]) -> np.ndarray:
        """T_base_tool (4x4)."""
        return self._walk(np.asarray(q, dtype=float))[0]

    def fk_frame(self, q: Sequence[float], T_gripper_link_frame: np.ndarray) -> np.ndarray:
        """T_base_frame for any frame rigidly attached to gripper_link (the camera, for example)."""
        return self._walk(np.asarray(q, dtype=float), tip=T_gripper_link_frame)[0]

    # -- IK ------------------------------------------------------------------------------------------------------
    def solve(self, target: Sequence[float], q0: Sequence[float], pointing: Sequence[float] | None = None, *,
              T_tip: np.ndarray | None = None, point_weight_m: float = 0.15, null_gain: float = 0.05,
              iters: int = 300, tol_pos_m: float = 1e-4, tol_point_rad: float = 2e-3) -> IKResult:
        """Move the tool (or `T_tip`, a frame on gripper_link) to `target`; with `pointing`, its +X axis along it.

        point_weight_m converts a pointing error in rad into an equivalent distance for the least squares."""
        target = np.asarray(target, dtype=float)
        want = None if pointing is None else np.asarray(pointing, dtype=float) / np.linalg.norm(pointing)
        q = np.clip(np.asarray(q0, dtype=float), self.lo, self.hi)
        lam2 = 1e-4
        for _ in range(iters):
            T, o, a = self._walk(q, tip=T_tip)
            p = T[:3, 3]
            Jv = np.cross(a, p - o).T                       # (3, 6)
            e = [target - p]
            J = [Jv]
            if want is not None:
                x = T[:3, 0]
                e.append(point_weight_m * (want - x))
                J.append(point_weight_m * (-_skew(x) @ a.T))
            e = np.concatenate(e)
            J = np.vstack(J)
            pos_err = float(np.linalg.norm(target - p))
            ang_err = 0.0 if want is None else float(math.acos(max(-1.0, min(1.0, float(T[:3, 0] @ want)))))
            if pos_err < tol_pos_m and ang_err < tol_point_rad:
                return IKResult(q, pos_err, ang_err, True)
            JJt = J @ J.T
            dq = J.T @ np.linalg.solve(JJt + lam2 * np.eye(len(e)), e)
            Jpinv = J.T @ np.linalg.pinv(JJt)
            dq += (np.eye(6) - Jpinv @ J) @ (null_gain * (self.rest_q - q))
            m = np.abs(dq).max()
            if m > 0.2:
                dq *= 0.2 / m
            q = np.clip(q + dq, self.lo, self.hi)
        T = self._walk(q, tip=T_tip)[0]
        pos_err = float(np.linalg.norm(target - T[:3, 3]))
        ang_err = 0.0 if want is None else float(math.acos(max(-1.0, min(1.0, float(T[:3, 0] @ want)))))
        return IKResult(q, pos_err, ang_err, pos_err < tol_pos_m and ang_err < tol_point_rad)

    def solve_global(self, target, pointing=None, seeds: Iterable[Sequence[float]] | None = None, **kw) -> IKResult:
        """`solve` from several seeds; the reached solution closest to the rest pose wins."""
        seeds = list(seeds) if seeds is not None else [
            self.rest_q, [0.0, -0.5, -1.0, 0.0, 0.5, 0.0], [0.0, -1.0, -0.5, 0.0, 0.8, 0.0],
            [0.0, -1.5, -1.5, 0.0, 0.0, 0.0], [0.0, -0.3, -1.5, 0.5, 0.3, 0.0], [0.0, -1.2, -1.8, 1.0, 0.0, 0.0]]
        y = float(np.asarray(target)[1])
        x = float(np.asarray(target)[0])
        j1 = math.atan2(y, x) if abs(x) + abs(y) > 1e-6 else 0.0
        best = None
        for s in seeds:
            s = np.array(s, dtype=float)
            s[0] = j1
            r = self.solve(target, s, pointing, **kw)
            if r.ok and (best is None or np.linalg.norm(r.q - self.rest_q) < np.linalg.norm(best.q - self.rest_q)):
                best = r
        if best is None:
            raise IKError(f"no IK solution for target {np.round(np.asarray(target), 4).tolist()}"
                          + ("" if pointing is None else f" pointing {np.round(np.asarray(pointing), 3).tolist()}"))
        return best

    def follow(self, points: np.ndarray, pointing: np.ndarray | None, q_start: Sequence[float], *,
               max_joint_step_rad: float = 0.08, **kw) -> np.ndarray:
        """Joint solutions along densely sampled path points, warm-started from the previous one."""
        q = np.asarray(q_start, dtype=float)
        out = np.zeros((len(points), 6))
        for i, p in enumerate(points):
            r = self.solve(p, q, None if pointing is None else pointing[i], **kw)
            if not r.ok:
                raise IKError(f"path point {i} {np.round(p, 4).tolist()} unreachable "
                              f"(residual {r.pos_err_m * 1000:.1f} mm, {math.degrees(r.point_err_rad):.1f} deg)")
            step = float(np.abs(r.q - q).max())
            if i > 0 and step > max_joint_step_rad:
                raise IKError(f"path point {i} {np.round(p, 4).tolist()}: the joint solution jumps {step:.3f} rad "
                              "(a branch change or a singularity): resample the path or move it")
            out[i] = q = r.q
        return out


# -- timing ------------------------------------------------------------------------------------------------------

def ramp_profile(length_m: float, speed_m_s: float, ramp_s: float) -> tuple[float, callable]:
    """Cosine-ramped trapezoid along a path of `length_m`: (duration, s(t)). Velocity and acceleration are
    continuous; peak speed `speed_m_s` (lowered for a path too short to reach it)."""
    if length_m <= 0:
        return 0.0, (lambda t: 0.0)
    V = min(speed_m_s, length_m / ramp_s)          # ramps cover V*ramp_s in total; the cruise needs >= 0
    T = length_m / V + ramp_s

    def s(t: float) -> float:
        t = min(max(t, 0.0), T)
        if t < ramp_s:
            return V / 2 * (t - ramp_s / math.pi * math.sin(math.pi * t / ramp_s))
        if t > T - ramp_s:
            return length_m - s(T - t)
        return V * ramp_s / 2 + V * (t - ramp_s)
    return T, s


def _arc(points: np.ndarray) -> np.ndarray:
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])


def _interp(points: np.ndarray, cum: np.ndarray, s: float) -> tuple[np.ndarray, int, float]:
    i = int(np.clip(np.searchsorted(cum, s, side="right") - 1, 0, len(points) - 2))
    span = cum[i + 1] - cum[i]
    u = 0.0 if span < 1e-12 else (s - cum[i]) / span
    return points[i] + u * (points[i + 1] - points[i]), i, u


def time_pieces(pieces: Sequence[Piece], ramp_s: float = 0.8, dt: float = DT_S
                ) -> tuple[np.ndarray, np.ndarray, np.ndarray | None, list[tuple[float, float, str]]]:
    """Sample a sequence of pieces in time: (t (N,), positions (N,3), pointing (N,3) or None, spans).
    Consecutive non-stop pieces are merged into one profile, so the tool only stops where a piece says so."""
    groups: list[list[Piece]] = [[]]
    for pc in pieces:
        groups[-1].append(pc)
        if pc.stop:
            groups.append([])
    groups = [g for g in groups if g]
    has_point = any(pc.pointing is not None for pc in pieces)
    if has_point and any(pc.pointing is None for pc in pieces):
        raise ValueError("either every piece has a pointing direction or none has")
    ts, ps, ns, spans = [], [], [], []
    t0 = 0.0
    for g in groups:
        pts = np.vstack([g[0].points] + [pc.points[1:] for pc in g[1:]])
        if has_point:
            nrm = np.vstack([pc.pointing[(0 if k == 0 else 1):] for k, pc in enumerate(g)])
        cum = _arc(pts)
        T, s_of = ramp_profile(float(cum[-1]), min(pc.speed_m_s for pc in g), ramp_s)
        n = max(1, int(math.ceil(T / dt)))
        for k in range(0 if not ts else 1, n + 1):
            t = T * k / n
            p, i, u = _interp(pts, cum, s_of(t))
            ts.append(t0 + t)
            ps.append(p)
            if has_point:
                v = (1 - u) * nrm[i] + u * nrm[i + 1]
                ns.append(v / np.linalg.norm(v))
        spans.append((t0, t0 + T, "+".join(pc.label for pc in g if pc.label)))
        t0 += T
    return np.array(ts), np.array(ps), (np.array(ns) if has_point else None), spans


def peak_speed_fraction(t: np.ndarray, q: np.ndarray) -> np.ndarray:
    v = np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]
    return v.max(axis=0) / RUNNER_SPEED_CAP_RAD_S


def slow_to_fit(t: np.ndarray, q: np.ndarray, speed_fraction: float) -> tuple[np.ndarray, float]:
    """Stretch time uniformly so no joint's peak exceeds speed_fraction of its runner cap. Returns (t, factor)."""
    f = float(peak_speed_fraction(t, q).max() / speed_fraction)
    return (t * f, f) if f > 1.0 else (t, 1.0)


# -- output ------------------------------------------------------------------------------------------------------

@dataclass
class TrajectoryReport:
    n_points: int
    duration_s: float
    peak_speed_rad_s: list[float]
    peak_speed_fraction_of_cap: list[float]
    slowed_by: float
    min_gripper_z_m: float
    max_tip_error_mm: float
    joint_ranges: list[list[float]]
    notes: list[str] = field(default_factory=list)


def build_trajectory(ik: ArmIK, pieces: Sequence[Piece], q_start: Sequence[float], *, ramp_s: float = 0.8,
                     speed_fraction: float = 0.7, dt: float = DT_S, max_joint_step_rad: float = 0.08,
                     floor_z_m: float = GRIPPER_FLOOR_Z_M + 0.01) -> tuple[np.ndarray, np.ndarray, TrajectoryReport]:
    """Pieces -> (t, q) at dt spacing (slowed to fit the caps), plus a report. Refuses (IKError) a sample the
    arm cannot reach, a branch jump, or a gripper_link below floor_z_m."""
    t, pts, nrm, _spans = time_pieces(pieces, ramp_s=ramp_s, dt=dt)
    q = ik.follow(pts, nrm, q_start, max_joint_step_rad=max_joint_step_rad)
    t, factor = slow_to_fit(t, q, speed_fraction)
    tips = np.array([ik.fk(qq)[:3, 3] for qq in q])
    zg = np.array([ik.fk_frame(qq, np.eye(4))[2, 3] for qq in q])
    if zg.min() < floor_z_m:
        k = int(zg.argmin())
        raise IKError(f"gripper_link would go to z {zg.min():.3f} m at t={t[k]:.2f} s, below {floor_z_m:.3f} m "
                      "(arm_guard's floor is 0.12 m): raise the path")
    v = np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]
    rep = TrajectoryReport(
        n_points=len(t), duration_s=round(float(t[-1]), 3),
        peak_speed_rad_s=[round(float(x), 3) for x in v.max(axis=0)],
        peak_speed_fraction_of_cap=[round(float(x), 3) for x in (v.max(axis=0) / RUNNER_SPEED_CAP_RAD_S)],
        slowed_by=round(factor, 3), min_gripper_z_m=round(float(zg.min()), 4),
        max_tip_error_mm=round(float(np.linalg.norm(tips - pts, axis=1).max() * 1000), 3),
        joint_ranges=[[round(float(q[:, i].min()), 3), round(float(q[:, i].max()), 3)] for i in range(6)])
    return t, q, rep


def trajectory_doc(t: np.ndarray, q: np.ndarray, *, producer: str, purpose: str, notes: str,
                   source: dict | None = None, extra: dict | None = None) -> dict:
    """A crackvision.joint_trajectory/1 document (the fields make_box_trajectory.py writes)."""
    if abs(float(t[0])) > 1e-9:
        raise ValueError("the first point must be at t=0")
    doc = {
        "schema": TRAJECTORY_SCHEMA,
        "created_utc": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "producer": producer,
        "purpose": purpose,
        "planning_frame": "base_link",
        "joint_names": JOINT_NAMES,
        "notes": notes,
        "points": [{"t_s": round(float(tt), 6), "positions": [round(float(v), 6) for v in qq]} for tt, qq in zip(t, q)],
        "limits_file": {"path": "config/robot/b601_dm_limits.yaml", "sha256": None},
        "end_effector_config_sha256": None,
        "scene_config_sha256": None,
        "source": source or {"paths3d": None},
    }
    if extra:
        doc.update(extra)
    return doc

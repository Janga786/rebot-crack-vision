#!/usr/bin/env python3
"""demo_move.py: a multi-waypoint, all-joint "show" move with the 2026-10-17 tuned gains (operator workspace only).

Every motor is enabled and follows a joint-space min-jerk path through the waypoints of a choreography, then
returns to the measured start (the folded rest) and everything is disabled. Feed-forward = URDF gravity
(gravity.py) x TUNED scale + Coulomb friction along the commanded velocity, as in first_move_j1.py.

Before anything is enabled the whole path is checked: URDF limits, planned peak speed per joint, and table
clearance (gripper_link z, forward kinematics every 0.05 s).

Abort handling (the arm may be raised, so it must not simply drop):
  - controllable faults (tracking, torque, velocity, a held position slipping, Ctrl-C): PARK = a slow min-jerk
    return from the measured pose to the start pose with the same gains, then disable all. A second Ctrl-C, a bus
    error or a lost motor during the park disables immediately.
  - uncontrollable faults (bus error, stale feedback, a motor fault status): hold 0.5 s if possible, disable all.
The 24 V switch is still the e-stop: cutting it while the arm is raised DROPS the arm.

arm_guard (2026-10-21; ~/Projects/rebot_crack_vision/packages/arm_guard, docs/motion/ARM_GUARD.md):
  - a real run refuses to start unless the guard is active on motorbridge with an approved config;
  - before anything is enabled, every command of the run is checked offline against the guard's rules (its
    startup check, joint windows, speed and stopping distance, gains, keep-out): a run the guard would stop is
    refused up front instead of stopping half-way;
  - if the guard faults anyway, it has already sent a hold and latched. This script then never parks past it or
    disables on its own (the arm may be raised): it keeps the arm holding and waits for the operator. Ctrl-C (or
    "stop" in run-trajectory.sh / basic-demo.sh) = reset the guard (logged) and PARK slowly to the fold; Ctrl-C
    again during the park = switch off now.
  - with --wait-go FILE, a file DEMO_OFF next to it ("off" in run-trajectory.sh) = switch everything off NOW, at any
    point after enabling (also from that hold, e.g. when every park attempt faults again). The arm drops if raised:
    support it first. A stale DEMO_OFF is removed before anything is enabled.

--trajectory FILE runs a crackvision.joint_trajectory/1 file (the repo's planner output, joint1..joint6) instead of
a choreography: after the zeroing and the second go, a min-jerk approach from the fold to its first point, the
trajectory itself (a monotone cubic through its points: no overshoot between points, at rest at both ends), a
min-jerk return to the fold, then everything is disabled. The gripper holds at 0. Needs --wait-go.

Usage (operator's go given in chat):
  env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python demo_move.py --operator-go "go" [--choreo wave]
  ... --trajectory FILE --wait-go runs/DEMO_GO [--slow 2]      (run-trajectory.sh wraps this)
  ... --simulate [--guard-sim]   (fakebus, simulated time; --guard-sim puts the real guard config in front of it)
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any, Optional

import first_move_j1 as fm
import gravity

SCRIPT_VERSION = "2026-10-23.1"   # + --state-file (arm pose for the camera); 2026-10-22.1: --gains (per-run override); 2026-10-21.1: arm_guard pre-flight / fault handling, --trajectory
NAMES = [n for n, *_ in fm.MOTORS]
GAINS = {n: fm.TUNED[n][:2] for n in fm.TUNED} | {"j7": (40.0, 0.5)}
GRAV_SCALE = {n: fm.TUNED[n][2] for n in fm.TUNED} | {"j7": 0.0}
FRICTION = {n: fm.TUNED[n][3] for n in fm.TUNED} | {"j7": 0.0}
PLAN_VEL_MAX = {"4340P": 0.60, "4310": 1.20}      # planned peak speed per joint, rad/s (refused above); was 0.4/0.8
VEL_ABORT = {"4340P": 1.00, "4310": 1.60}         # measured |vel| abort (DM4310 readings spike single samples)
# 2026-10-17 operator: J1 "facing the audience" = -3.765 rad as read after the operator turned it by hand
# (beyond the URDF's +-2.8). The demo may use J1 down to -3.9; everything else keeps the URDF limits.
AUDIENCE_J1 = -3.76
LIMITS = dict(fm.URDF_LIMITS) | {"j1": (-3.9, 2.8)}
SPEED = 1.0   # --speed: every choreography duration / SPEED; planned-speed caps and velocity aborts x SPEED
SPEED_MAX = 2.0
EXIT_J1_UNKNOWN = 4   # --j1-ref-file: J1 matches neither forward nor audience -> re-reference by hand


def plan_cap(model: str) -> float:
    return PLAN_VEL_MAX[model] * SPEED


def vel_abort(model: str) -> float:
    return VEL_ABORT[model] * SPEED


# --j1-here-is-zero (2026-10-18): after a power cycle J1 read +3.03 instead of -3.77, so J1's multi-turn position is
# not trusted across power cycles. The operator turns J1 by hand to the original forward direction; its reading
# there becomes J1's zero for this run (software offset; the motor's stored zero is never touched).
J1_OFFSET = 0.0
J2_DROOP_MAX = 0.45   # with the flag: J2 may start drooped (+) below the fold by this much; the zeroing lifts it


J2_ALLOW_DROOP = False


def limits(n: str) -> tuple[float, float]:
    lo, hi = LIMITS[n]
    if n == "j2" and J2_ALLOW_DROOP:
        hi = J2_DROOP_MAX     # J2 may sit drooped below the fold at the start; the zeroing lifts it to 0
    return (lo + J1_OFFSET, hi + J1_OFFSET) if n == "j1" else (lo, hi)


AUTO = None   # segment duration computed from the speed caps (90 %)
TRACK_ABORT_RAD = 0.10
PARK_TRACK_ABORT_RAD = 0.25
PARK_SPEED = 0.25                                 # rad/s planned peak while parking
TABLE_Z_MIN = 0.12                                # gripper_link origin above the base plane, m
START_TOL_RAD = {"j7": 0.40}                      # gripper may start part-open (it rests near -0.2)
START_TOL_DEFAULT = 0.10                          # every other joint must start within this of 0 (the fold)
SETTLE_S = 1.0                                    # hold at start while the extra gravity fades in
FINAL_HOLD_S = 1.0
LOOP_HZ = 200.0
STALE_S = 0.20
STALL_S = 0.05
MAX_WAIT_S = 300.0     # --wait-go: no go within this -> disable all (the arm is at zero = its folded rest)

S = "start"   # a waypoint value meaning "the measured start position of that joint"
# (duration_s, {joint: absolute target rad or S}); joints not named keep their previous target.
Z = {n: 0.0 for n in ("j2", "j3", "j4", "j5", "j6", "j7")}
CHOREOS = {
    "show": [   # 2026-10-17: turn to the audience, then a faster, longer routine; ends folded, facing them
        (AUTO, {"j1": AUDIENCE_J1}),                                  # turn to face the audience (folded)
        (4.0, {"j2": -0.6, "j3": -1.2}),                              # rise
        (0.5, {}),
        (2.5, {"j1": AUDIENCE_J1 + 0.4, "j5": 0.5}),                  # look left
        (3.5, {"j1": AUDIENCE_J1 - 0.1, "j5": -0.6}),                 # look right (wrist; the base stays >= -3.9)
        (2.0, {"j1": AUDIENCE_J1, "j5": 0.0}),                        # face them
        (2.0, {"j4": 0.6}),                                           # nod
        (2.5, {"j6": 1.5, "j7": -1.0}),                               # roll + open
        (5.0, {"j6": -1.5, "j7": 0.0}),                               # roll the other way + close
        (2.5, {"j6": 0.0, "j4": 0.0}),
        (1.0, {"j5": 0.5}), (1.6, {"j5": -0.5}), (1.6, {"j5": 0.5}), (1.6, {"j5": -0.5}), (1.0, {"j5": 0.0}),  # wave
        (1.6, {"j7": -1.0}), (1.6, {"j7": 0.0}), (1.6, {"j7": -1.0}), (1.6, {"j7": 0.0}),                    # snap x2
        (3.0, {"j2": -0.8, "j3": -1.6, "j4": 0.5}),                   # reach up
        (0.8, {}),
        (3.0, {"j2": -0.6, "j3": -1.2, "j4": 0.0}),
        (5.0, dict(Z)),                                               # fold down, still facing the audience
    ],
    "wave": [
        (6.0, {"j2": -0.6, "j3": -1.2}),                      # rise out of the fold
        (1.0, {}),
        (4.0, {"j1": 0.5, "j5": 0.5}),                        # look left
        (1.0, {}),
        (6.0, {"j1": -0.5, "j5": -0.5}),                      # sweep right
        (1.0, {}),
        (4.0, {"j1": 0.0, "j5": 0.0, "j4": 0.4}),             # centre, nod the wrist
        (3.0, {"j6": 1.0, "j7": -1.0}),                       # roll the hand, open the gripper
        (4.0, {"j6": 0.0, "j7": S}),                          # roll back, close
        (2.0, {"j4": 0.0}),
        (6.0, {n: S for n in NAMES}),                         # back to the folded rest
    ],
}


def min_jerk(q0: float, q1: float, T: float, t: float) -> tuple[float, float]:
    s = max(0.0, min(1.0, t / T))
    return q0 + (q1 - q0) * (10 * s**3 - 15 * s**4 + 6 * s**5), (q1 - q0) * (30 * s**2 - 60 * s**3 + 30 * s**4) / T


class Path_:
    """Piecewise min-jerk joint path through absolute waypoints."""

    def __init__(self, start: dict[str, float], choreo: list, j1_offset: bool = True) -> None:
        """Choreography values for J1 are relative to J1_OFFSET; pass j1_offset=False for absolute targets."""
        self.segs: list[tuple[float, float, dict, dict]] = []   # (t0, T, q_from, q_to)
        q, t = dict(start), 0.0
        for T, wp in choreo:
            q_to = dict(q)
            for n, v in wp.items():
                q_to[n] = start[n] if v == S else float(v) + (J1_OFFSET if n == "j1" and j1_offset else 0.0)
            if T is AUTO:
                T = max(2.0, max(1.875 * abs(q_to[n] - q[n]) / (0.9 * plan_cap(fm.MODEL_OF[n])) for n in q))
            self.segs.append((t, T, dict(q), q_to))
            q, t = q_to, t + T
        self.duration = t
        self.end = q

    def at(self, t: float) -> tuple[int, dict[str, float], dict[str, float]]:
        for k, (t0, T, qa, qb) in enumerate(self.segs):
            if t < t0 + T:
                qv = {n: min_jerk(qa[n], qb[n], T, t - t0) for n in qa}
                return k, {n: v[0] for n, v in qv.items()}, {n: v[1] for n, v in qv.items()}
        return len(self.segs), dict(self.end), {n: 0.0 for n in self.end}


TRAJ_SCHEMA = "crackvision.joint_trajectory/1"
STATE_PERIOD_S = 0.05                             # --state-file publishing period
TRAJ_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
CHECK_DT = 0.005                                  # sampling step of the trajectory path check


def load_trajectory(path: str) -> tuple[dict, list[float], list[dict[str, float]]]:
    """Read and validate a crackvision.joint_trajectory/1 file. Returns (header, times, [{j1..j6: rad}])."""
    raw = Path(path).expanduser().read_bytes()
    d = json.loads(raw)
    if d.get("schema") != TRAJ_SCHEMA:
        raise ValueError(f"schema {d.get('schema')!r} != {TRAJ_SCHEMA!r}")
    if d.get("joint_names") != TRAJ_JOINTS:
        raise ValueError(f"joint_names {d.get('joint_names')} != {TRAJ_JOINTS}")
    pts = d.get("points") or []
    if len(pts) < 2:
        raise ValueError("needs at least 2 points")
    times, qs = [], []
    for k, pt in enumerate(pts):
        t, pos = pt.get("t_s"), pt.get("positions")
        if not (isinstance(t, (int, float)) and math.isfinite(t)) or not (isinstance(pos, list) and len(pos) == 6
                                                                           and all(isinstance(x, (int, float)) and
                                                                                   math.isfinite(x) for x in pos)):
            raise ValueError(f"point {k}: needs finite t_s and 6 finite positions")
        if (k == 0 and t != 0) or (k > 0 and t <= times[-1]):
            raise ValueError(f"point {k}: t_s must start at 0 and strictly increase")
        times.append(float(t))
        qs.append({f"j{i + 1}": float(x) for i, x in enumerate(pos)})
    header = {k: v for k, v in d.items() if k != "points"} | {"path": str(Path(path).expanduser().resolve()),
                                                               "sha256": hashlib.sha256(raw).hexdigest(),
                                                               "n_points": len(pts)}
    return header, times, qs


def pchip_slopes(t: list[float], y: list[float]) -> list[float]:
    """Monotone cubic (Fritsch-Carlson / PCHIP) slopes, zero at both ends: the curve never overshoots between two
    points, so a trajectory whose points are inside the limits stays inside them."""
    n = len(t)
    h = [t[i + 1] - t[i] for i in range(n - 1)]
    d = [(y[i + 1] - y[i]) / h[i] for i in range(n - 1)]
    m = [0.0] * n
    for i in range(1, n - 1):
        if d[i - 1] * d[i] > 0:
            w1, w2 = 2 * h[i] + h[i - 1], h[i] + 2 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])
    return m


def hermite(y0: float, y1: float, m0: float, m1: float, h: float, s: float) -> tuple[float, float]:
    s2, s3 = s * s, s * s * s
    q = (2 * s3 - 3 * s2 + 1) * y0 + (s3 - 2 * s2 + s) * h * m0 + (-2 * s3 + 3 * s2) * y1 + (s3 - s2) * h * m1
    v = ((6 * s2 - 6 * s) * y0 + (3 * s2 - 4 * s + 1) * h * m0 + (-6 * s2 + 6 * s) * y1 + (3 * s2 - 2 * s) * h * m1) / h
    return q, v


class TrajPath:
    """approach (min-jerk, start -> first point) + the trajectory (monotone cubic) + return (min-jerk, last point ->
    start). Same interface as Path_: segs [(t0, T, q_from, q_to)] (one per piece, for logging), duration, end, at()."""

    def __init__(self, start: dict[str, float], times: list[float], qs: list[dict[str, float]], slow: float) -> None:
        def auto(qa: dict, qb: dict) -> float:
            moves = [1.875 * abs(qb[n] - qa[n]) / (0.9 * plan_cap(fm.MODEL_OF[n])) for n in qa]
            return 0.0 if max(abs(qb[n] - qa[n]) for n in qa) < 1e-4 else max(2.0, max(moves)) * slow

        full = [dict(start) | q for q in qs]          # the gripper (and anything not in the file) keeps its start
        for q in full:
            q["j1"] += J1_OFFSET                       # file angles are physical; J1 runs relative to its reference
        self.times = [t * slow for t in times]
        self.points = full
        self.slopes = {n: pchip_slopes(self.times, [q[n] for q in full]) for n in start}
        pieces = [("approach", auto(start, full[0]), start, full[0]),
                  ("trajectory", self.times[-1], full[0], full[-1]),
                  ("return", auto(full[-1], start), full[-1], dict(start))]
        self.segs, self.kinds, t = [], [], 0.0
        for kind, T, qa, qb in pieces:
            if T > 0:
                self.segs.append((t, T, dict(qa), dict(qb)))
                self.kinds.append(kind)
                t += T
        self.duration = t
        self.end = dict(start)

    def at(self, t: float) -> tuple[int, dict[str, float], dict[str, float]]:
        for k, (t0, T, qa, qb) in enumerate(self.segs):
            if t < t0 + T:
                if self.kinds[k] != "trajectory":
                    qv = {n: min_jerk(qa[n], qb[n], T, t - t0) for n in qa}
                    return k, {n: v[0] for n, v in qv.items()}, {n: v[1] for n, v in qv.items()}
                tau = t - t0
                i = max(0, min(len(self.times) - 2, next(i for i in range(len(self.times) - 1)
                                                          if tau < self.times[i + 1])))
                h = self.times[i + 1] - self.times[i]
                s = (tau - self.times[i]) / h
                q, v = {}, {}
                for n in qa:
                    q[n], v[n] = hermite(self.points[i][n], self.points[i + 1][n], self.slopes[n][i],
                                         self.slopes[n][i + 1], h, s)
                return k, q, v
        return len(self.segs), dict(self.end), {n: 0.0 for n in self.end}


def check_path_sampled(path, gm: gravity.GravityModel, start: dict[str, float]) -> list[str]:
    """check_path for any path: limits and planned speed sampled every CHECK_DT, table clearance every 0.05 s."""
    problems, seen = [], set()
    t, next_z = 0.0, 0.0
    while t <= path.duration + 1e-9:
        _, q, v = path.at(t)
        for n in NAMES:
            lo, hi = limits(n)
            at_rest = abs(q[n] - start[n]) < 1e-9 and lo - START_LIMIT_MARGIN <= q[n] <= hi + START_LIMIT_MARGIN
            if not (lo <= q[n] <= hi or at_rest) and (n, "lim") not in seen:
                seen.add((n, "lim"))
                problems.append(f"t={t:.2f} s: {n} {q[n]:+.3f} outside URDF [{lo}, {hi}]")
            vmax = plan_cap(fm.MODEL_OF[n])
            if abs(v[n]) > vmax + 1e-9 and (n, "vel") not in seen:
                seen.add((n, "vel"))
                problems.append(f"t={t:.2f} s: {n} peak {abs(v[n]):.2f} rad/s > {vmax} (slow it down: --slow)")
        if t >= next_z:
            z = gripper_z(gm, q)
            if z < TABLE_Z_MIN and "z" not in seen:
                seen.add("z")
                problems.append(f"t={t:.2f} s: gripper_link z {z:.3f} m < {TABLE_Z_MIN} (table)")
            next_z += 0.05
        t += CHECK_DT
    return problems


def gripper_z(gm: gravity.GravityModel, q: dict[str, float]) -> float:
    urdf_q = {gravity.ARM_JOINTS[k]: v for k, v in q.items() if k in gravity.ARM_JOINTS}
    T = {gm.base: [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]}
    for name, typ, p, c, org, ax in gm.order:
        Tj = gravity._mm(T[p], org)
        if typ in ("revolute", "continuous"):
            Tj = gravity._mm(Tj, gravity._axis_rot(ax, urdf_q.get(name, 0.0)))
        T[c] = Tj
    return T["gripper_link"][2][3]


START_LIMIT_MARGIN = 0.03   # a return to the measured rest may sit this far past a URDF limit (j2 rests at +0.002)


def check_path(path: Path_, gm: gravity.GravityModel, start: dict[str, float]) -> list[str]:
    problems = []
    for k, (t0, T, qa, qb) in enumerate(path.segs):
        for n in NAMES:
            lo, hi = limits(n)
            at_rest = qb[n] == start[n] and lo - START_LIMIT_MARGIN <= qb[n] <= hi + START_LIMIT_MARGIN
            if not (lo <= qb[n] <= hi or at_rest):
                problems.append(f"segment {k}: {n} target {qb[n]:+.3f} outside URDF [{lo}, {hi}]")
            peak = 1.875 * abs(qb[n] - qa[n]) / T
            vmax = plan_cap(fm.MODEL_OF[n])
            if peak > vmax:
                problems.append(f"segment {k}: {n} peak {peak:.2f} rad/s > {vmax}")
    t = 0.0
    zmin_seen = math.inf
    while t <= path.duration:
        z = gripper_z(gm, path.at(t)[1])
        zmin_seen = min(zmin_seen, z)
        if z < TABLE_Z_MIN:
            problems.append(f"t={t:.2f} s: gripper_link z {z:.3f} m < {TABLE_Z_MIN} (table)")
            break
        t += 0.05
    return problems


def parse_gains(spec: str) -> dict[str, tuple[float, float]]:
    """'j2=100/4,j3=80/4' -> {'j2': (100.0, 4.0), 'j3': (80.0, 4.0)}, within this script's caps."""
    out = {}
    for item in spec.split(","):
        try:
            name, val = item.strip().split("=")
            kp_s, kd_s = val.split("/")
            kp, kd = float(kp_s), float(kd_s)
        except ValueError:
            raise ValueError(f"{item!r}: expected joint=kp/kd, e.g. j2=100/4") from None
        if name not in GAINS:
            raise ValueError(f"{name!r} is not one of {sorted(GAINS)}")
        if not (0.0 < kp <= fm.KP_MAX and 0.0 <= kd <= fm.KD_MAX):
            raise ValueError(f"{name}: kp {kp:g} / kd {kd:g} outside kp (0, {fm.KP_MAX:g}], kd [0, {fm.KD_MAX:g}]")
        out[name] = (kp, kd)
    return out


class DemoMotor:
    """enable/disable/reads and send_mit within this script's gain and feed-forward caps. No register writes."""
    _ALLOWED = ("request_feedback", "get_state", "get_register_u32", "get_register_f32", "enable", "disable")

    def __init__(self, name: str, motor: Any) -> None:
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "_m", motor)

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        if (kp, kd) != GAINS[self.name]:
            raise PermissionError(f"{self.name}: gains {kp}/{kd} != {GAINS[self.name]}")
        if not abs(tau) <= fm.FF_CAP_BY_MODEL[fm.MODEL_OF[self.name]]:
            raise PermissionError(f"{self.name}: |tau_ff| {tau:.2f} over cap")
        lo, hi = limits(self.name)
        if not (math.isfinite(pos) and math.isfinite(vel) and lo - START_LIMIT_MARGIN <= pos <= hi + START_LIMIT_MARGIN):
            raise PermissionError(f"{self.name}: target {pos} outside URDF range")
        self._m.send_mit(float(pos), float(vel), float(kp), float(kd), float(tau))

    def __getattr__(self, attr: str) -> Any:
        if attr in DemoMotor._ALLOWED:
            return getattr(self._m, attr)
        raise PermissionError(f"{self.name}: '{attr}' is not allowed")

    def __setattr__(self, attr: str, value: Any) -> None:
        raise PermissionError("DemoMotor is immutable")


class Abort(Exception):
    def __init__(self, trigger: str, detail: str, controllable: bool) -> None:
        super().__init__(f"{trigger}: {detail}")
        self.trigger, self.detail, self.controllable = trigger, detail, controllable


# ---- arm_guard --------------------------------------------------------------------------------------------------
GUARD = None                  # the arm_guard Guard in front of the motors for this run (real, or --guard-sim)
GUARD_ERRORS: tuple = ()      # (arm_guard.GuardError,) once the guard is in place


def guard_missing() -> str:
    """'' if motorbridge in this process is patched by arm_guard with an approved config, else why not."""
    try:
        import arm_guard
    except Exception as e:  # noqa: BLE001
        return f"arm_guard is not importable ({e!r}); install its hook: python -m arm_guard install-hook"
    core = sys.modules.get("motorbridge.core")
    if core is None or not (getattr(core.Motor, "_arm_guard_patched", False)
                            and getattr(core.Controller, "_arm_guard_patched", False)):
        return "motorbridge is not patched by arm_guard in this interpreter (python -m arm_guard hook-status)"
    g = arm_guard.guard()
    if g.cfg is None:
        return f"arm_guard has no approved config, it would refuse all motion: {g.config_error}"
    return ""


def guarded_fakebus(bus, out_dir: Path):
    """--guard-sim: the real guard (repo config, approval checked) patched onto fakebus classes, on simulated time.
    Returns (guard, controller factory). Locks and events go to out_dir, never to the real ones."""
    from arm_guard import config as agc, runtime as agr
    from arm_guard.patch import patch
    cfg = agc.load()
    agr.LOCK_DIR = Path(out_dir) / "guard_locks_SIM"
    agr.EVENT_LOG = Path(out_dir) / "guard_events_SIM.jsonl"
    g = agr.Guard(cfg, clock=bus.clock, sleep=bus.clock.sleep)

    class SimMotor(fakebus_mod().FakeMotor):
        pass

    class SimController(fakebus_mod().FakeController):
        def __init__(self, channel: str, fbus) -> None:
            super().__init__(fbus)

        def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str):
            base = fakebus_mod().FakeController.add_damiao_motor(self, motor_id, feedback_id, model)
            m = SimMotor(self, base._sim)
            self.motors[base._sim.name] = m
            return m

    patch(SimController, SimMotor, g)

    def factory(port: str, baud: int):
        bus.calls.append(("ctrl", "from_dm_serial", (port, baud)))
        bus.controller = SimController(port, bus)
        return bus.controller
    return g, factory


def fakebus_mod():
    import fakebus
    return fakebus


def feed_forward(gm: gravity.GravityModel, q: dict, v_ref: dict, fade: float) -> dict:
    """URDF gravity x tuned scale (the part above 1 fading in) + Coulomb friction along the commanded velocity."""
    g = gm.hold_torques(q)
    out = {}
    for n in NAMES:
        sc = GRAV_SCALE[n]
        out[n] = (min(sc, 1.0) + fade * max(sc - 1.0, 0.0)) * g.get(n, 0.0) \
            + FRICTION[n] * math.tanh(v_ref.get(n, 0.0) / fm.FRICTION_VEL_EPS)
    return out


def sampled_peaks(path, dt: float = CHECK_DT) -> dict[str, float]:
    peaks, t = {n: 0.0 for n in NAMES}, 0.0
    while t <= path.duration:
        for n, v in path.at(t)[2].items():
            peaks[n] = max(peaks[n], abs(v))
        t += dt
    return peaks


def guard_preflight(path, zeroing, measured: dict, start: dict, gm: gravity.GravityModel) -> dict:
    """Every MIT command this run will stream (settle, zeroing, the wait at zero, the path, the final hold), sampled
    at the loop rate exactly as control() builds them, through the guard's rules (arm_guard.preflight)."""
    from arm_guard import preflight
    period = 1.0 / LOOP_HZ
    zero = {n: 0.0 for n in NAMES}

    def ticks():
        t, k = 0.0, 0
        t_zero_end = SETTLE_S + (zeroing.duration if zeroing is not None else 0.0)
        while t < t_zero_end + (period if zeroing is not None else 0.0):
            if t < SETTLE_S:
                q, v = measured, zero
            elif t < t_zero_end:
                _, q, v = zeroing.at(t - SETTLE_S)
            else:
                q, v = start, zero                         # holding at zero, waiting for the second go
            ff = feed_forward(gm, q, v, min(1.0, t / SETTLE_S))
            yield t, {n: (q[n], v[n], *GAINS[n], ff[n]) for n in NAMES}
            k += 1
            t = k * period
        t0 = t
        while t - t0 < path.duration + FINAL_HOLD_S:
            _, q, v = path.at(t - t0)
            ff = feed_forward(gm, q, v, 1.0)
            yield t, {n: (q[n], v[n], *GAINS[n], ff[n]) for n in NAMES}
            k += 1
            t = k * period
    return preflight.check_plan(GUARD.cfg, measured, ticks())


def run(argv: list[str], rt: Optional[fm.Runtime] = None, bus_factory=None, sim_setup=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--choreo", default="wave", choices=sorted(CHOREOS))
    ap.add_argument("--slow", type=float, default=1.0, help="multiply every duration (>= 1)")
    ap.add_argument("--j1-here-is-zero", action="store_true",
                    help="J1's current reading is its zero for this run (operator turned it to the original forward "
                         "direction by hand); J2 may start drooped up to +0.45 rad and is lifted while zeroing")
    ap.add_argument("--j1-ref-file", default=None, metavar="FILE",
                    help="saved J1 reference: reused if J1 reads forward/audience relative to it, else exit 4; "
                         "with --j1-here-is-zero the new reference is saved there")
    ap.add_argument("--j1-offset", type=float, default=None, metavar="RAD",
                    help="reuse an earlier --j1-here-is-zero reading as J1 zero (valid only until the next power cycle)")
    ap.add_argument("--speed", type=float, default=1.0,
                    help=f"1..{SPEED_MAX}: durations / SPEED, planned-speed caps and velocity aborts x SPEED "
                         "(tracking abort and the park speed unchanged)")
    ap.add_argument("--operator-go", default=None, metavar="TEXT")
    ap.add_argument("--check-only", action="store_true", help="read-only pre-checks and path check; enable nothing")
    ap.add_argument("--port", default=fm.PORT_DEFAULT)
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--simulate", action="store_true")
    ap.add_argument("--wait-go", default=None, metavar="FILE",
                    help="enable, go to zero and hold; start the demo when FILE appears (disable after 5 min)")
    ap.add_argument("--trajectory", default=None, metavar="FILE",
                    help="run a crackvision.joint_trajectory/1 file instead of a choreography (needs --wait-go)")
    ap.add_argument("--gains", default=None, metavar="SPEC",
                    help="this run only: override MIT gains, e.g. j2=100/4,j3=80/4 (kp <= 100, kd <= 5)")
    ap.add_argument("--state-file", default=None, metavar="FILE",
                    help="publish the measured arm pose here ~20 times a second (JSON, atomic replace; physical "
                         "joint angles, J1 reference removed) so a camera capture can be tied to it; output only")
    ap.add_argument("--guard-sim", action="store_true",
                    help="with --simulate: put arm_guard (repo config, approval checked) in front of the fake bus")
    ap.add_argument("--sim-auto-go", type=float, default=None, metavar="S",
                    help="with --simulate --wait-go: give the second go S seconds after reaching zero")
    args = ap.parse_args(argv)
    if args.slow < 1.0:
        print("usage error: --slow must be >= 1", file=sys.stderr)
        return fm.EXIT_USAGE
    if args.j1_offset is not None and args.j1_here_is_zero:
        print("usage error: --j1-offset and --j1-here-is-zero are exclusive", file=sys.stderr)
        return fm.EXIT_USAGE
    if not 1.0 <= args.speed <= SPEED_MAX:
        print(f"usage error: --speed must be in [1, {SPEED_MAX}]", file=sys.stderr)
        return fm.EXIT_USAGE
    traj = None
    if args.trajectory:
        if not args.wait_go:
            print("usage error: --trajectory needs --wait-go FILE (zero, hold, then the operator's second go)",
                  file=sys.stderr)
            return fm.EXIT_USAGE
        if args.speed != 1.0:
            print("usage error: --trajectory runs at the file's own timing; slow it down with --slow, never --speed",
                  file=sys.stderr)
            return fm.EXIT_USAGE
        try:
            traj = load_trajectory(args.trajectory)
        except (OSError, ValueError) as e:
            print(f"usage error: trajectory {args.trajectory}: {e}", file=sys.stderr)
            return fm.EXIT_USAGE
    if args.sim_auto_go is not None and not (args.simulate and args.wait_go):
        print("usage error: --sim-auto-go needs --simulate and --wait-go", file=sys.stderr)
        return fm.EXIT_USAGE
    global SIM_AUTO_STOP_S
    SIM_AUTO_STOP_S = 3.0 if args.sim_auto_go is not None else None
    if args.guard_sim and not args.simulate:
        print("usage error: --guard-sim only applies to --simulate (a real run always needs the guard)",
              file=sys.stderr)
        return fm.EXIT_USAGE
    try:
        gains_override = parse_gains(args.gains) if args.gains else {}
    except ValueError as e:
        print(f"usage error: --gains: {e}", file=sys.stderr)
        return fm.EXIT_USAGE
    global SPEED
    SPEED = args.speed
    if not args.simulate and not args.check_only and not (args.operator_go and args.operator_go.strip()):
        print("usage error: --operator-go must quote the operator's go", file=sys.stderr)
        return fm.EXIT_USAGE
    if rt is None:
        fm_args = argparse.Namespace(out_dir=args.out_dir, simulate=args.simulate, simulate_fault="none", gravity=False)
        rt = fm.make_runtime(fm_args)
        if isinstance(rt, int):
            return rt
        if args.simulate:
            bus_factory = None
    choreo = [(T if T is AUTO else T * args.slow / args.speed, wp) for T, wp in CHOREOS[args.choreo]]
    log = rt.log
    gm = gravity.GravityModel()
    stem = f"traj_{Path(args.trajectory).stem}" if traj else f"demo_{args.choreo}"
    base = Path(rt.out_dir) / f"{stem}_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}" \
                              f"{'_SIM' if rt.simulated else ''}"
    Path(rt.out_dir).mkdir(parents=True, exist_ok=True)
    summary: dict = {"script": "demo_move.py", "version": SCRIPT_VERSION,
                     "choreo": "trajectory" if traj else args.choreo, "slow": args.slow, "speed": args.speed,
                     "trajectory": traj[0] if traj else None,
                     "plan_cap": {m: PLAN_VEL_MAX[m] * args.speed for m in PLAN_VEL_MAX},
                     "vel_abort": {m: VEL_ABORT[m] * args.speed for m in VEL_ABORT},
                     "operator_go": args.operator_go, "gains": dict(GAINS) | gains_override,
                     "gains_override": gains_override, "gravity_scale": GRAV_SCALE,
                     "friction": FRICTION, "simulated": rt.simulated}
    t_wall0 = time.time()
    saved_gains = dict(GAINS)
    GAINS.update(gains_override)
    if gains_override:
        log("gains override for this run: " + ", ".join(f"{n} kp {kp:g} kd {kd:g}" for n, (kp, kd) in gains_override.items()))
    try:
        return _run(args, rt, bus_factory, choreo, gm, base, summary, sim_setup, traj)
    finally:
        GAINS.clear()
        GAINS.update(saved_gains)
        if GUARD is not None:
            summary["guard"] = {"simulated": bool(rt.simulated), "config_sha256": GUARD.cfg.sha256,
                                "latched_at_end": GUARD.latched,
                                "events": [e for e in GUARD.events if e.get("t_wall", 0) >= t_wall0]}
        Path(f"{base}.json").write_text(json.dumps(summary, indent=2, default=str))
        log(f"summary: {base}.json")


def _run(args, rt, bus_factory, choreo, gm, base, summary, sim_setup, traj=None) -> int:
    global GUARD, GUARD_ERRORS, OFF_FILE
    OFF_FILE = None
    log, clock, sleep = rt.log, rt.clock, rt.sleep
    CallError = rt.mb.CallError
    if getattr(rt.mb, "__version__", "?") != fm.MOTORBRIDGE_VERSION:
        summary["outcome"] = "refused"
        log(f"REFUSED: motorbridge {getattr(rt.mb, '__version__', '?')} != {fm.MOTORBRIDGE_VERSION}")
        return fm.EXIT_REFUSED
    GUARD, GUARD_ERRORS = None, ()
    if not rt.simulated:
        why = guard_missing()
        if why:
            summary["outcome"] = "refused"
            log(f"REFUSED (nothing was enabled): {why}")
            return fm.EXIT_REFUSED
        import arm_guard
        GUARD, GUARD_ERRORS = arm_guard.guard(), (arm_guard.GuardError,)
    if rt.check_host:
        try:
            fm.host_prechecks(args.port)
        except fm.Refused as e:
            summary["outcome"] = "refused"
            log(f"REFUSED: {e}")
            return fm.EXIT_REFUSED
    if bus_factory is None and rt.simulated:
        import fakebus
        bus = fakebus.FakeBus()
        rt.mb = bus.mod
        rt.clock, rt.sleep = bus.clock, bus.clock.sleep
        clock, sleep = rt.clock, rt.sleep
        bus_factory = bus.factory
        for n, tq in gm.hold_torques(dict(bus.rest)).items():
            bus.sims[n].load = -tq * GRAV_SCALE[n]
        for n, (inertia, coulomb) in {"j1": (0.3, 1.0), "j2": (0.5, 1.0), "j3": (0.3, 2.3)}.items():
            bus.sims[n].inertia, bus.sims[n].coulomb = inertia, coulomb    # measured 2026-10-17 (friction)
        if sim_setup:
            sim_setup(bus)
        if args.guard_sim:
            import arm_guard
            try:
                GUARD, bus_factory = guarded_fakebus(bus, Path(rt.out_dir))
            except arm_guard.GuardConfigError as e:
                summary["outcome"] = "refused"
                log(f"REFUSED: arm_guard config: {e}")
                return fm.EXIT_REFUSED
            GUARD_ERRORS = (arm_guard.GuardError,)
    factory = bus_factory or rt.mb.Controller.from_dm_serial
    raw_ctrl = factory(args.port, fm.BAUD)
    raw: dict[str, Any] = {}
    fm.ACTIVE, fm.HOLD_OTHERS = "j1", True          # precheck: every motor must be in MIT mode
    try:
        for name, mid, fid, model in fm.MOTORS:
            raw[name] = raw_ctrl.add_damiao_motor(mid, fid, model)
        ro = {n: fm.ReadOnlyMotor(n, m) for n, m in raw.items()}
        try:
            readings = fm.precheck_bus(ro, rt)
        except fm.Refused as e:
            # one retry: a single missed register read has refused several runs on 2026-10-17
            log(f"pre-check failed once ({e}); retrying")
            try:
                readings = fm.precheck_bus(ro, rt)
            except fm.Refused as e2:
                summary["outcome"] = "refused"
                log(f"REFUSED (nothing was enabled): {e2}")
                return fm.EXIT_REFUSED
        start = {n: readings[n]["pos"] for n in NAMES}
        summary["start"] = start
        # The waypoints are absolute angles: they assume encoder zero = the folded pose (set by the vendor,
        # never changed). Starting in the fold proves it before a big move (a re-zeroed motor would read far off).
        global J1_OFFSET, J2_ALLOW_DROOP
        ref_file = Path(args.j1_ref_file).expanduser() if args.j1_ref_file else None
        if ref_file is not None and not args.j1_here_is_zero:
            # reuse the saved reference only if J1 reads forward or audience relative to it; anything else means
            # a power cycle (J1 loses its turn count) or a hand move -> the operator must re-reference
            try:
                saved = json.loads(ref_file.read_text())["offset"]
            except (OSError, ValueError, KeyError):
                saved = None
            q1 = start["j1"]
            if saved is None or min(abs(q1 - saved), abs(q1 - saved - AUDIENCE_J1)) > 0.15:
                summary["outcome"] = "refused_j1_reference_unknown"
                log(f"REFUSED (nothing was enabled): J1 reads {q1:+.4f}, which is neither forward nor the audience "
                    f"relative to the saved reference {saved}. Turn the base to FORWARD by hand and re-reference.")
                return EXIT_J1_UNKNOWN
            args.j1_offset = saved
        if args.j1_offset is not None:
            J1_OFFSET = args.j1_offset      # today's reference, reused while the power stays on
        else:
            J1_OFFSET = start["j1"] if args.j1_here_is_zero else 0.0
        J2_ALLOW_DROOP = bool(args.j1_here_is_zero or args.j1_offset is not None)
        summary["j1_offset"] = J1_OFFSET
        if args.j1_here_is_zero and ref_file is not None:
            ref_file.write_text(json.dumps({"offset": J1_OFFSET, "set_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                                                                        time.gmtime())}))
            log(f"J1 reference saved to {ref_file}")
        if args.j1_here_is_zero:
            log(f"J1 reference: the operator turned J1 to the original forward direction; its reading "
                f"{J1_OFFSET:+.4f} rad is J1 zero for this run (software offset)")

        def _off(n: str, q: float) -> bool:
            if args.j1_offset is not None and n == "j1":   # forward or facing the audience, relative to the offset
                return min(abs(q - J1_OFFSET), abs(q - J1_OFFSET - AUDIENCE_J1)) > 0.15
            if args.j1_here_is_zero and n == "j1":
                return False            # J1 is the reference by definition
            if J2_ALLOW_DROOP and n == "j2":
                return not -START_TOL_DEFAULT <= q <= J2_DROOP_MAX
            if n == "j1" and abs(q - AUDIENCE_J1) <= 0.15:
                return False            # folded but already facing the audience
            return abs(q) > START_TOL_RAD.get(n, START_TOL_DEFAULT)
        off = [f"{n} {q:+.3f}" for n, q in start.items() if _off(n, q)]
        if off:
            summary["outcome"], summary["refusal"] = "refused", f"not in the folded rest: {off}"
            log(f"REFUSED (nothing was enabled): not starting from the folded rest (|q| > "
                f"{START_TOL_DEFAULT} rad): {', '.join(off)}. Put the arm in its fold; if it IS folded, a motor "
                "zero may have changed -> stop and check.")
            return fm.EXIT_REFUSED
        measured = dict(start)
        zeroing = None
        go_file = Path(args.wait_go).expanduser() if args.wait_go else None
        if go_file is not None:
            # 2026-10-17 operator: go to zero and hold first; the demo starts on the second go (a file the Claude
            # session creates with the operator's words). The demo then starts and ends at zero.
            start = {n: 0.0 for n in NAMES} | {"j1": measured["j1"]}   # J1 stays; the choreography turns it
            # (with --j1-here-is-zero, measured j1 IS zero: no base rotation while zeroing)
            T_z = max(3.0, max(1.875 * abs(measured[n] - start[n]) / PARK_SPEED for n in NAMES))
            zeroing = Path_(measured, [(T_z, dict(start))], j1_offset=False)   # absolute targets (no double offset)
            OFF_FILE = go_file.parent / "DEMO_OFF"
            try:
                go_file.unlink(missing_ok=True)
                OFF_FILE.unlink(missing_ok=True)
            except OSError as e:
                summary["outcome"] = "refused"
                log(f"REFUSED: cannot clear a stale go/off file next to {go_file}: {e}")
                return fm.EXIT_REFUSED
        if traj:
            path = TrajPath(start, traj[1], traj[2], args.slow)
            problems = check_path_sampled(path, gm, start)
        else:
            path = Path_(start, choreo)
            problems = check_path(path, gm, start)
        if problems:
            summary["outcome"], summary["refusal"] = "refused", problems
            log("REFUSED (path check; nothing was enabled):\n  " + "\n  ".join(problems))
            return fm.EXIT_REFUSED
        if traj:
            pieces = ", ".join(f"{k} {seg[1]:.1f} s" for k, seg in zip(path.kinds, path.segs))
            log(f"pre-checks OK; path OK: trajectory {Path(args.trajectory).name} ({traj[0]['n_points']} points, "
                f"purpose {traj[0].get('purpose')!r}, x{args.slow:g} slower): {pieces}; gripper stays >= "
                f"{TABLE_Z_MIN} m above the base plane")
            peaks = sampled_peaks(path)
        else:
            log(f"pre-checks OK; path OK: {len(choreo)} segments, {path.duration:.1f} s + settle/hold, gripper stays "
                f">= {TABLE_Z_MIN} m above the base plane")
            peaks = {n: max(1.875 * abs(qb[n] - qa[n]) / T for _, T, qa, qb in path.segs) for n in NAMES}
        if GUARD is not None:
            pf = guard_preflight(path, zeroing, measured, start, gm)
            summary["guard_preflight"] = {k: pf[k] for k in ("problems", "warnings", "offsets", "n_ticks")} \
                | {"min_keepout_m": pf["min_keepout_m"]}
            if pf["problems"]:
                summary["outcome"], summary["refusal"] = "refused", ["arm_guard: " + x for x in pf["problems"]]
                log("REFUSED (arm_guard pre-flight: the guard would stop this run; nothing was enabled):\n  "
                    + "\n  ".join(pf["problems"]))
                return fm.EXIT_REFUSED
            for wmsg in pf["warnings"]:
                log(f"WARNING (arm_guard pre-flight): {wmsg}")
            turns = {n: round(o / (2 * math.pi)) for n, o in pf["offsets"].items() if o}
            log(f"arm_guard pre-flight OK: {pf['n_ticks']} command cycles x {len(NAMES)} motors accepted; keep-out "
                f"margin >= {pf['min_keepout_m'] * 1000:.0f} mm" + (f"; whole-turn offsets {turns}" if turns else ""))
        summary["planned_peak_rad_s"] = peaks
        log(f"speed x{SPEED:g}: planned peaks " + ", ".join(f"{n} {v:.2f}" for n, v in peaks.items())
            + f" rad/s; velocity aborts {vel_abort('4340P'):.2f} (J1-J3) / {vel_abort('4310'):.2f} (J4-J7) rad/s; "
              f"tracking abort {TRACK_ABORT_RAD} rad")
        log("start: " + ", ".join(f"{n} {q:+.3f}" for n, q in start.items()))
        log("ABORT -> controlled PARK back to this start pose, then all motors off. 24 V switch = e-stop: "
            "cutting it while raised DROPS the arm.")
        if args.check_only:
            summary["outcome"] = "check_ok"
            return fm.EXIT_OK
        log(f"confirmation: the operator's go, given in chat: {args.operator_go!r}")
        motors = {n: DemoMotor(n, raw[n]) for n in NAMES}
        if go_file is not None:
            log(f"mode: enable, move to zero over {zeroing.duration:.1f} s, HOLD; the demo starts when {go_file} "
                f"appears (the operator's second go), or everything is disabled after {MAX_WAIT_S:.0f} s")
        return control(args, rt, raw_ctrl, motors, path, start, gm, base, summary, measured, zeroing, go_file)
    finally:
        for m in raw.values():
            try:
                m.close()
            except Exception:  # noqa: BLE001
                pass
        for f in ("close_bus", "close"):
            try:
                getattr(raw_ctrl, f)()
            except Exception:  # noqa: BLE001
                pass


def control(args, rt, raw_ctrl, motors, path: Path_, start, gm, base, summary, measured=None, zeroing=None,
            go_file=None) -> int:
    log, clock, sleep = rt.log, rt.clock, rt.sleep
    CallError = rt.mb.CallError
    fresh = {n: fm.Freshness(clock, CallError) for n in NAMES}
    q_meas = dict(measured if measured is not None else start)
    enabled: list[str] = []
    stats = {"n_cmd": 0, "max_err": {n: 0.0 for n in NAMES}, "max_gap_s": 0.0, "parked": False}
    trigger: Optional[Abort] = None
    stop = {"n": 0}
    old = {}

    def on_signal(signum, _frame):  # noqa: ANN001
        stop["n"] += 1

    if rt.install_signals:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old[sig] = signal.signal(sig, on_signal)
    fh = open(f"{base}.csv", "w", newline="")
    w = csv.writer(fh)
    w.writerow(["t_s", "mode", "seg"] + [f"{n}_{k}" for n in NAMES for k in ("ref", "pos", "vel", "torq", "ff")])
    state_file = Path(args.state_file).expanduser() if getattr(args, "state_file", None) else None
    state_next = [-math.inf]
    q_vel = {n: 0.0 for n in NAMES}
    kinds = getattr(path, "kinds", None)
    if state_file is not None:
        try:
            state_file.unlink(missing_ok=True)   # never let a stale pose from an earlier run be read
        except OSError as e:
            log(f"WARNING: --state-file {state_file}: {e}; not publishing the arm state")
            state_file = None

    def publish_state(t: float, mode: str, seg: int, q_ref: dict) -> None:
        """--state-file: the measured pose for crackvision.crack_trace (a camera capture at the scan pose). Never
        raises into the control loop: a write error only stops the publishing."""
        nonlocal state_file
        now = clock()
        if state_file is None or now < state_next[0]:
            return
        state_next[0] = now + STATE_PERIOD_S
        phys = {n: q_meas[n] - (J1_OFFSET if n == "j1" else 0.0) for n in NAMES}
        doc = {"schema": "rebot.arm_state/1", "script_version": SCRIPT_VERSION, "wall_time_ns": time.time_ns(),
               "simulated": bool(args.simulate), "t_s": round(t, 4), "mode": mode, "seg": seg,
               "segment_kind": (kinds[seg] if kinds and 0 <= seg < len(kinds) else None),
               "joint_names": TRAJ_JOINTS, "positions_rad": [round(phys[f"j{i}"], 6) for i in range(1, 7)],
               "ref_rad": [round(q_ref[f"j{i}"] - (J1_OFFSET if i == 1 else 0.0), 6) for i in range(1, 7)],
               "velocity_rad_s": [round(q_vel[f"j{i}"], 5) for i in range(1, 7)], "gripper_rad": round(q_meas["j7"], 5)}
        try:
            tmp = state_file.with_name(state_file.name + ".tmp")
            tmp.write_text(json.dumps(doc))
            os.replace(tmp, state_file)
        except OSError as e:
            log(f"WARNING: --state-file {state_file}: {e}; no longer publishing the arm state")
            state_file = None

    def ff_for(q: dict, v_ref: dict, fade: float) -> dict:
        out = feed_forward(gm, q, v_ref, fade)
        for n, ff in out.items():
            cap = fm.FF_CAP_BY_MODEL[fm.MODEL_OF[n]]
            if abs(ff) > cap:
                raise Abort("gravity", f"{n} feed-forward {ff:+.2f} Nm > {cap}", True)
        return out

    def cycle(q_ref: dict, v_ref: dict, ff: dict, mode: str, seg: int, t: float, track_lim: float,
              t_prev: list) -> None:
        now = clock()
        gap = now - t_prev[0]
        stats["max_gap_s"] = max(stats["max_gap_s"], gap)
        if gap > STALL_S:
            raise Abort("stream", f"{gap * 1000:.0f} ms between command cycles", True)
        t_prev[0] = now
        for n in NAMES:
            try:
                motors[n].send_mit(q_ref[n], v_ref[n], *GAINS[n], ff[n])
            except CallError as e:
                raise Abort("bus", f"{n} send_mit: {e}", False) from e
            except GUARD_ERRORS as e:   # the guard already sent a hold and latched: never park/disable past it
                raise Abort("guard", f"{n}: {e}", True) from e
        if GUARD is not None and GUARD.latched and mode != "guard_hold":
            raise Abort("guard", f"latched: {GUARD.latched}", True)   # latched without a fault reaching us
        stats["n_cmd"] += 1
        row = [f"{t:.4f}", mode, seg]
        first: Optional[Abort] = None
        for n in NAMES:
            st = motors[n].get_state()
            fresh[n].observe(motors[n], st)
            if fresh[n].age() > STALE_S:
                first = first or Abort("stale", f"{n}: no fresh feedback for {fresh[n].age():.2f} s", False)
            if st is None:
                row += [f"{q_ref[n]:+.4f}", "", "", "", f"{ff[n]:+.3f}"]
                continue
            q_meas[n] = st.pos
            q_vel[n] = st.vel
            row += [f"{q_ref[n]:+.4f}", f"{st.pos:+.5f}", f"{st.vel:+.4f}", f"{st.torq:+.3f}", f"{ff[n]:+.3f}"]
            if st.status_code != fm.ST_ENABLED:
                first = first or Abort("status", f"{n} reports {fm.STATUS_NAMES.get(st.status_code)}", False)
            err = abs(q_ref[n] - st.pos)
            stats["max_err"][n] = max(stats["max_err"][n], err)
            kp, kd = GAINS[n]
            if err > track_lim:
                first = first or Abort("tracking", f"{n} |ref-pos| {err:.3f} > {track_lim}", True)
            tl = kp * track_lim + kd * vel_abort(fm.MODEL_OF[n]) + fm.TORQUE_MARGIN_NM + abs(ff[n])
            if abs(st.torq) > tl:
                first = first or Abort("torque", f"{n} |torque| {abs(st.torq):.2f} > {tl:.2f}", True)
            if abs(st.vel) > vel_abort(fm.MODEL_OF[n]):
                first = first or Abort("velocity", f"{n} |vel| {abs(st.vel):.2f} > {vel_abort(fm.MODEL_OF[n]):.2f}",
                                       True)
        w.writerow(row)
        publish_state(t, mode, seg, q_ref)
        if first is not None:
            raise first

    period = 1.0 / LOOP_HZ
    outcome = "aborted"
    try:
        try:
            # ---- enable one by one, each immediately holding its measured position ------------------------------
            ff0 = ff_for(q_meas, {}, 0.0)
            for n in NAMES:
                try:
                    motors[n].enable()
                except GUARD_ERRORS as e:
                    raise Abort("guard_enable", f"{n} enable refused by arm_guard: {e}", False) from e
                enabled.append(n)
                st = fm.wait_state(motors[n], rt, fm.ENABLE_CONFIRM_S,
                                   lambda s: s is not None and s.status_code == fm.ST_ENABLED)
                if st is None or st.status_code != fm.ST_ENABLED:
                    raise Abort("enable", f"{n} did not report ENABLED", False)
                q_meas[n] = st.pos
                try:
                    motors[n].send_mit(st.pos, 0.0, *GAINS[n], ff0[n])
                except GUARD_ERRORS as e:
                    raise Abort("guard", f"{n}: {e}", True) from e
                fresh[n].observe(motors[n], st)
            hold = dict(q_meas)
            log("all 7 motors ENABLED, holding the start pose; settling " + f"{SETTLE_S} s")
            t0 = clock()
            t_prev = [t0]
            next_t = t0 + period
            zero = {n: 0.0 for n in NAMES}
            t_path = None if zeroing is not None else SETTLE_S     # when the choreography starts
            t_wait = SETTLE_S + zeroing.duration if zeroing is not None else None
            last_seg = -1
            while True:
                poll_stop(rt, stop)
                if off_requested():
                    raise Abort("operator_off", f"switch off now: {off_requested()!r}", False)
                if stop["n"]:
                    raise Abort("signal", "Ctrl-C/SIGTERM", True)
                t = clock() - t0
                if t_path is not None and t >= t_path + path.duration + FINAL_HOLD_S:
                    outcome = "completed"
                    break
                if t < SETTLE_S:
                    seg, q_ref, v_ref, mode = -1, hold, zero, "settle"
                elif t_path is None and t < t_wait:
                    _, q_ref, v_ref = zeroing.at(t - SETTLE_S)
                    seg, mode = -3, "to_zero"
                elif t_path is None:
                    seg, q_ref, v_ref, mode = -4, start, zero, "wait_go"
                    if t - t_wait > MAX_WAIT_S:
                        log(f"no go within {MAX_WAIT_S:.0f} s: disabling (the arm is at zero, its folded rest)")
                        outcome = "no_go_timeout"
                        break
                    if rt.simulated and args.sim_auto_go is not None and t - t_wait >= args.sim_auto_go \
                            and not go_file.exists():
                        go_file.write_text("auto go (simulation)")
                    if go_file.exists():
                        try:
                            words = go_file.read_text().strip()
                        except OSError:
                            words = "?"
                        summary["second_go"] = words
                        log(f"second go received: {words!r}; starting the demo")
                        t_path = t
                    elif not stats.get("wait_logged"):
                        stats["wait_logged"] = True
                        log("at zero, HOLDING; waiting for the operator's go")
                else:
                    seg, q_ref, v_ref = path.at(t - t_path)
                    mode = "path" if seg < len(path.segs) else "final_hold"
                    if seg != last_seg and seg < len(path.segs):
                        log(f"  segment {seg}: {path.segs[seg][1]:.1f} s")
                        last_seg = seg
                ff = ff_for(q_meas, v_ref, min(1.0, t / SETTLE_S))
                cycle(q_ref, v_ref, ff, mode, seg, t, TRACK_ABORT_RAD, t_prev)
                slack = next_t - clock()
                if slack > 0:
                    sleep(slack)
                    next_t += period
                else:
                    next_t = clock() + period
        except Abort as a:
            trigger = a
        except CallError as e:
            trigger = Abort("bus", str(e), False)
        except Exception as e:  # noqa: BLE001
            trigger = Abort("internal", f"{type(e).__name__}: {e}", True)

        if trigger is not None:
            log(f"ABORT: [{trigger.trigger}] {trigger.detail}")
            if trigger.controllable and len(enabled) == len(NAMES):
                if trigger.trigger == "guard":
                    outcome = "park_guard_fault"
                else:
                    outcome = park(rt, motors, q_meas, start, ff_for, cycle, stats, stop, log)
                while outcome == "park_guard_fault":       # hold until the operator says park (reset + park)
                    outcome = guard_recover(rt, motors, q_meas, start, ff_for, cycle, stats, stop, log)
            elif enabled:
                log("the operator switches off: holding 0.5 s, then disabling all" if trigger.trigger == "operator_off"
                    else "uncontrollable fault: holding 0.5 s where possible, then disabling all")
                t_end = clock() + fm.ABORT_HOLD_S
                q_h = dict(q_meas)
                while clock() < t_end:
                    try:
                        for n in enabled:
                            motors[n].send_mit(q_h[n], 0.0, *GAINS[n], 0.0)
                    except Exception:  # noqa: BLE001
                        break
                    sleep(period)
    finally:
        not_off = []
        for n in reversed(enabled):
            try:
                motors[n].disable()
            except Exception as e:  # noqa: BLE001
                log(f"{n} disable failed: {e}; trying disable_all")
                try:
                    raw_ctrl.disable_all()
                except Exception:  # noqa: BLE001
                    pass
        for n in enabled:
            try:
                st = fm.wait_state(motors[n], rt, fm.DISABLE_CONFIRM_S,
                                   lambda s: s is not None and s.status_code == fm.ST_DISABLED)
                if st is None or st.status_code != fm.ST_DISABLED:
                    not_off.append(n)
            except Exception:  # noqa: BLE001
                not_off.append(n)
        if enabled and not not_off:
            log(f"all motors DISABLED (confirmed): {', '.join(enabled)}")
        elif not_off:
            log(f"WARNING: not confirmed DISABLED: {not_off} -> CUT 24 V NOW")
        for sig, h in old.items():
            signal.signal(sig, h)
        fh.close()
        summary.update(outcome=outcome, trigger=None if trigger is None else
                       {"id": trigger.trigger, "detail": trigger.detail}, stats=stats, csv=f"{base}.csv",
                       all_disabled_confirmed=not not_off)
    log(f"outcome: {outcome}" + ("" if trigger is None else f" (trigger {trigger.trigger}: {trigger.detail})"))
    log("max |ref-pos| per joint: " + ", ".join(f"{n} {e:.3f}" for n, e in stats["max_err"].items())
        + f"; max gap {stats['max_gap_s'] * 1000:.1f} ms; csv {base}.csv")
    return fm.EXIT_OK if outcome == "completed" else fm.EXIT_ABORTED


def park(rt, motors, q_meas, start, ff_for, cycle, stats, stop, log) -> str:
    """Slow min-jerk return from the measured pose to the start pose, then the caller disables all."""
    clock, sleep = rt.clock, rt.sleep
    q0 = {n: min(max(q, limits(n)[0]), limits(n)[1]) for n, q in q_meas.items()}   # e.g. J3 sagged past its stop
    start = dict(start) | {"j1": q0["j1"]}   # fold the arm; the base stays where it is (it carries no gravity)
    T = max(3.0, max(1.875 * abs(start[n] - q0[n]) / PARK_SPEED for n in NAMES))
    log(f"PARK: returning to the start pose over {T:.1f} s (Ctrl-C again = disable now)")
    stats["parked"] = True
    sigs_at_start = stop["n"]
    period = 1.0 / LOOP_HZ
    t0 = clock()
    t_prev = [t0]
    next_t = t0 + period
    try:
        while True:
            poll_stop(rt, stop)
            if off_requested():
                log(f"PARK interrupted: the operator switches off now ({off_requested()!r})")
                return "aborted_operator_off"
            if stop["n"] > sigs_at_start:
                log("PARK interrupted by a second Ctrl-C: disabling now")
                return "aborted_park_interrupted"
            t = clock() - t0
            if t >= T + 0.5:
                log("PARK complete")
                return "aborted_parked"
            qv = {n: min_jerk(q0[n], start[n], T, t) for n in q0}
            q_ref = {n: v[0] for n, v in qv.items()}
            v_ref = {n: v[1] for n, v in qv.items()}
            try:
                ff = ff_for(q_meas, v_ref, 1.0)
            except Abort:
                ff = {n: 0.0 for n in q0}
            try:
                cycle(q_ref, v_ref, ff, "park", -2, t, PARK_TRACK_ABORT_RAD, t_prev)
            except Abort as a:
                # keep parking through velocity/torque readings (sensor spikes); tracking/bus/stale stop the park
                if a.trigger == "guard":
                    log(f"PARK stopped by arm_guard: {a.detail}")
                    return "park_guard_fault"
                if a.trigger not in ("velocity", "torque"):
                    log(f"PARK stopped: [{a.trigger}] {a.detail}; disabling")
                    return "aborted_park_failed"
            slack = next_t - clock()
            if slack > 0:
                sleep(slack)
                next_t += period
            else:
                next_t = clock() + period
    except Exception as e:  # noqa: BLE001
        log(f"PARK error {type(e).__name__}: {e}; disabling")
        return "aborted_park_failed"


GUARD_NOTE_S = 30.0
SIM_AUTO_STOP_S = None   # --sim-auto-go: a simulated operator also says stop this long after a guard fault
SIM_MAX_PARKS = 2        # ... and after this many guard resets it says off instead
OFF_FILE: Optional[Path] = None   # --wait-go: <dir of the go file>/DEMO_OFF = switch off now


def off_requested() -> str:
    """The operator's words if DEMO_OFF exists, else ''."""
    if OFF_FILE is None or not OFF_FILE.exists():
        return ""
    try:
        return OFF_FILE.read_text().strip() or "off"
    except OSError:
        return "off"


def poll_stop(rt, stop: dict) -> None:
    """rt.stop_flag (set by tests and simulated operators) counts like one Ctrl-C."""
    if rt.stop_flag.get("why"):
        rt.stop_flag["why"] = None
        stop["n"] += 1


def guard_recover(rt, motors, q_meas, start, ff_for, cycle, stats, stop, log) -> str:
    """The guard faulted: it sent a hold and latched, and now turns every command into a hold at the latched pose.
    Keep streaming (the arm keeps holding, feedback stays fresh) and wait for the operator; never disable on our
    own (a raised arm would drop). The next Ctrl-C/'stop' resets the guard with a logged reason and parks slowly to
    the fold; a Ctrl-C during that park switches off (park()'s own rule)."""
    clock, sleep = rt.clock, rt.sleep
    stats["guard_faults"] = stats.get("guard_faults", 0) + 1
    detail = (GUARD.latched if GUARD is not None else "") or "?"
    log(f"GUARD FAULT, HOLDING: {detail}")
    log("  The arm holds where the fault latched; nothing is switched off automatically (a raised arm would drop).")
    log("  Ctrl-C / 'stop' = reset the guard and PARK slowly to the fold. 'off' (DEMO_OFF) = switch off now, also "
        "if every park faults again: support the arm first, it DROPS. 24 V switch = e-stop (the arm DROPS).")
    sigs = stop["n"]
    period = 1.0 / LOOP_HZ
    zero = {n: 0.0 for n in NAMES}
    q_hold = dict(q_meas)
    t0 = clock()
    t_prev = [t0]
    next_t, next_note = t0 + period, t0 + GUARD_NOTE_S
    while True:
        poll_stop(rt, stop)
        if off_requested():
            log(f"the operator switches off now ({off_requested()!r})")
            return "aborted_operator_off"
        if stop["n"] != sigs:
            break
        try:
            ff = ff_for(q_meas, {}, 1.0)
        except Abort:
            ff = dict(zero)
        try:   # the guard replaces the target with its latched hold; tracking/torque bounds do not apply here
            cycle(q_hold, zero, ff, "guard_hold", -5, clock() - t0, math.inf, t_prev)
        except Abort as a:
            if a.trigger in ("stale", "status", "bus", "stream", "guard"):
                log(f"while holding after the guard fault: [{a.trigger}] {a.detail}; cannot hold, disabling")
                return "aborted_guard_hold_failed"
        if clock() >= next_note:
            log(f"  still HOLDING after the guard fault ({clock() - t0:.0f} s): Ctrl-C / 'stop' = reset and park")
            next_note += GUARD_NOTE_S
        if rt.simulated and SIM_AUTO_STOP_S is not None and clock() - t0 >= SIM_AUTO_STOP_S:
            if stats.get("guard_resets", 0) >= SIM_MAX_PARKS and OFF_FILE is not None:
                log("  (simulation: the simulated operator supports the arm and says off)")
                OFF_FILE.write_text("off (simulated operator)")
            else:
                log("  (simulation: the simulated operator says stop)")
                rt.stop_flag["why"] = "simulated operator"
        slack = next_t - clock()
        if slack > 0:
            sleep(slack)
            next_t += period
        else:
            next_t = clock() + period
    reason = f"operator stop after guard fault: {detail}"
    if GUARD is not None:
        GUARD.reset(reason)
    stats["guard_resets"] = stats.get("guard_resets", 0) + 1
    log(f"arm_guard reset by the operator ({reason}); parking")
    return park(rt, motors, q_meas, start, ff_for, cycle, stats, stop, log)


if __name__ == "__main__":
    sys.exit(run(sys.argv[1:]))

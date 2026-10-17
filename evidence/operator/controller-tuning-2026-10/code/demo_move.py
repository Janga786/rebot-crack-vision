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

Usage (operator's go given in chat):
  env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python demo_move.py --operator-go "go" [--choreo wave]
  ... --simulate   (fakebus, simulated time)
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import Any, Optional

import first_move_j1 as fm
import gravity

SCRIPT_VERSION = "2026-10-17.1"
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

    def __init__(self, start: dict[str, float], choreo: list) -> None:
        self.segs: list[tuple[float, float, dict, dict]] = []   # (t0, T, q_from, q_to)
        q, t = dict(start), 0.0
        for T, wp in choreo:
            q_to = dict(q)
            for n, v in wp.items():
                q_to[n] = start[n] if v == S else float(v)
            if T is AUTO:
                T = max(2.0, max(1.875 * abs(q_to[n] - q[n]) / (0.9 * PLAN_VEL_MAX[fm.MODEL_OF[n]]) for n in q))
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
            lo, hi = LIMITS[n]
            at_rest = qb[n] == start[n] and lo - START_LIMIT_MARGIN <= qb[n] <= hi + START_LIMIT_MARGIN
            if not (lo <= qb[n] <= hi or at_rest):
                problems.append(f"segment {k}: {n} target {qb[n]:+.3f} outside URDF [{lo}, {hi}]")
            peak = 1.875 * abs(qb[n] - qa[n]) / T
            vmax = PLAN_VEL_MAX[fm.MODEL_OF[n]]
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
        lo, hi = LIMITS[self.name]
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


def run(argv: list[str], rt: Optional[fm.Runtime] = None, bus_factory=None, sim_setup=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--choreo", default="wave", choices=sorted(CHOREOS))
    ap.add_argument("--slow", type=float, default=1.0, help="multiply every duration (>= 1)")
    ap.add_argument("--operator-go", default=None, metavar="TEXT")
    ap.add_argument("--check-only", action="store_true", help="read-only pre-checks and path check; enable nothing")
    ap.add_argument("--port", default=fm.PORT_DEFAULT)
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--simulate", action="store_true")
    ap.add_argument("--wait-go", default=None, metavar="FILE",
                    help="enable, go to zero and hold; start the demo when FILE appears (disable after 5 min)")
    args = ap.parse_args(argv)
    if args.slow < 1.0:
        print("usage error: --slow must be >= 1", file=sys.stderr)
        return fm.EXIT_USAGE
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
    choreo = [(T if T is AUTO else T * args.slow, wp) for T, wp in CHOREOS[args.choreo]]
    log = rt.log
    gm = gravity.GravityModel()
    base = Path(rt.out_dir) / f"demo_{args.choreo}_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}" \
                              f"{'_SIM' if rt.simulated else ''}"
    Path(rt.out_dir).mkdir(parents=True, exist_ok=True)
    summary: dict = {"script": "demo_move.py", "version": SCRIPT_VERSION, "choreo": args.choreo, "slow": args.slow,
                     "operator_go": args.operator_go, "gains": GAINS, "gravity_scale": GRAV_SCALE,
                     "friction": FRICTION, "simulated": rt.simulated}
    try:
        return _run(args, rt, bus_factory, choreo, gm, base, summary, sim_setup)
    finally:
        Path(f"{base}.json").write_text(json.dumps(summary, indent=2, default=str))
        log(f"summary: {base}.json")


def _run(args, rt, bus_factory, choreo, gm, base, summary, sim_setup) -> int:
    log, clock, sleep = rt.log, rt.clock, rt.sleep
    CallError = rt.mb.CallError
    if getattr(rt.mb, "__version__", "?") != fm.MOTORBRIDGE_VERSION:
        summary["outcome"] = "refused"
        log(f"REFUSED: motorbridge {getattr(rt.mb, '__version__', '?')} != {fm.MOTORBRIDGE_VERSION}")
        return fm.EXIT_REFUSED
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
        def _off(n: str, q: float) -> bool:
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
            T_z = max(3.0, max(1.875 * abs(measured[n] - start[n]) / PARK_SPEED for n in NAMES))
            zeroing = Path_(measured, [(T_z, dict(start))])
            try:
                go_file.unlink(missing_ok=True)
            except OSError as e:
                summary["outcome"] = "refused"
                log(f"REFUSED: cannot clear a stale go file {go_file}: {e}")
                return fm.EXIT_REFUSED
        path = Path_(start, choreo)
        problems = check_path(path, gm, start)
        if problems:
            summary["outcome"], summary["refusal"] = "refused", problems
            log("REFUSED (path check; nothing was enabled):\n  " + "\n  ".join(problems))
            return fm.EXIT_REFUSED
        log(f"pre-checks OK; path OK: {len(choreo)} segments, {path.duration:.1f} s + settle/hold, gripper stays "
            f">= {TABLE_Z_MIN} m above the base plane")
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

    def ff_for(q: dict, v_ref: dict, fade: float) -> dict:
        g = gm.hold_torques(q)
        out = {}
        for n in NAMES:
            sc = GRAV_SCALE[n]
            ff = (min(sc, 1.0) + fade * max(sc - 1.0, 0.0)) * g.get(n, 0.0) \
                + FRICTION[n] * math.tanh(v_ref.get(n, 0.0) / fm.FRICTION_VEL_EPS)
            cap = fm.FF_CAP_BY_MODEL[fm.MODEL_OF[n]]
            if abs(ff) > cap:
                raise Abort("gravity", f"{n} feed-forward {ff:+.2f} Nm > {cap}", True)
            out[n] = ff
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
            row += [f"{q_ref[n]:+.4f}", f"{st.pos:+.5f}", f"{st.vel:+.4f}", f"{st.torq:+.3f}", f"{ff[n]:+.3f}"]
            if st.status_code != fm.ST_ENABLED:
                first = first or Abort("status", f"{n} reports {fm.STATUS_NAMES.get(st.status_code)}", False)
            err = abs(q_ref[n] - st.pos)
            stats["max_err"][n] = max(stats["max_err"][n], err)
            kp, kd = GAINS[n]
            if err > track_lim:
                first = first or Abort("tracking", f"{n} |ref-pos| {err:.3f} > {track_lim}", True)
            tl = kp * track_lim + kd * VEL_ABORT[fm.MODEL_OF[n]] + fm.TORQUE_MARGIN_NM + abs(ff[n])
            if abs(st.torq) > tl:
                first = first or Abort("torque", f"{n} |torque| {abs(st.torq):.2f} > {tl:.2f}", True)
            if abs(st.vel) > VEL_ABORT[fm.MODEL_OF[n]]:
                first = first or Abort("velocity", f"{n} |vel| {abs(st.vel):.2f} > {VEL_ABORT[fm.MODEL_OF[n]]}",
                                       True)
        w.writerow(row)
        if first is not None:
            raise first

    period = 1.0 / LOOP_HZ
    outcome = "aborted"
    try:
        try:
            # ---- enable one by one, each immediately holding its measured position ------------------------------
            ff0 = ff_for(q_meas, {}, 0.0)
            for n in NAMES:
                motors[n].enable()
                enabled.append(n)
                st = fm.wait_state(motors[n], rt, fm.ENABLE_CONFIRM_S,
                                   lambda s: s is not None and s.status_code == fm.ST_ENABLED)
                if st is None or st.status_code != fm.ST_ENABLED:
                    raise Abort("enable", f"{n} did not report ENABLED", False)
                q_meas[n] = st.pos
                motors[n].send_mit(st.pos, 0.0, *GAINS[n], ff0[n])
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
                outcome = park(rt, motors, q_meas, start, ff_for, cycle, stats, stop, log)
            elif enabled:
                log("uncontrollable fault: holding 0.5 s where possible, then disabling all")
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
    q0 = dict(q_meas)
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


if __name__ == "__main__":
    sys.exit(run(sys.argv[1:]))

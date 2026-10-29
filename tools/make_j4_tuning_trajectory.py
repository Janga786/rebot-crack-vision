"""tools/make_j4_tuning_trajectory.py — the J4 (wrist pitch) characterisation trajectory for the operator A/B runs.

docs/motion/J4_TUNING_PLAN.md §4.3 / docs/motion/J4_TUNING.md: wrist-only moves (J1, J5, J6 at 0, J2/J3 held) at two
poses, so J4's friction, damping and stiffness can be measured on their own:

  1. the heart's mid pose (J2/J3 = the median of ~/rebot_setup/trajectories/fancy/heart.json, the logged workspace;
     J4 centred on 0.2);
  2. a crack-workspace pose with J4 centred on 1.2, taken from the gravity survey's accepted poses
     (tools/make_gravity_survey.py: same joint windows, keep-outs, table clearance, predicted-sag and cap filters),
     choosing the one whose J4 gravity load is closest to the crack trajectories' (data/crack_trace/*.json), then the
     smallest predicted J2/J3 sag. Since J2, J3 and J4 are parallel axes, J4's load depends on the wrist's absolute
     pitch, so this reproduces the crack work's J4 load without going to its J2/J3 extremes.

At each pose: a 2 s settle, then triangle sweeps of +-0.15 rad at 0.05, 0.1 and 0.2 rad/s (two cycles each, both
directions, the corners blended by min-jerk velocity ramps: friction against speed), then 0.05 rad min-jerk steps of
0.5 s each followed by a 2 s hold, three up, six down, three up (static friction band, settling, overshoot, ripple).
Transits between the fold and the poses are min-jerk at <= 60 % of the runner's caps (routed through the heart's start
pose if a direct move is not clear), the wrist moves stay far below that (J4 <= 0.2 rad/s of 1.2). Every sample is
checked like the survey: the arm_guard joint windows (0.05 rad margin at the poses, 0 on the transits, which leave the
fold at the window edge), gripper_link >= 13.5 cm, camera/mount >= 3 cm, the guard's keep-outs + 1 cm; both poses also
pass the survey's predicted-sag / model-cap filters at the sweep extremes. Starts and ends at the fold, <= 4 min at x1.

The file's j4_tune block lists every pose and section with its window in file seconds, for
tools/analyze_run_gains.py --friction-fit --seg1-window T0:T1.

  ./env.sh python tools/make_j4_tuning_trajectory.py                 # -> ~/rebot_setup/trajectories/j4_tune.json
  then (simulation): ~/rebot_setup/run-trajectory.sh --rehearse ~/rebot_setup/trajectories/j4_tune.json 2
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

from crackvision import gravity_model as gmod
from crackvision.arm_ik import IKError, RUNNER_SPEED_CAP_RAD_S, check_motion_clearance, peak_speed_fraction, \
    trajectory_doc

_spec = importlib.util.spec_from_file_location("make_gravity_survey", Path(__file__).with_name("make_gravity_survey.py"))
ms = importlib.util.module_from_spec(_spec)
sys.modules.setdefault("make_gravity_survey", ms)
_spec.loader.exec_module(ms)

OUT = Path.home() / "rebot_setup" / "trajectories" / "j4_tune.json"
HEART = Path.home() / "rebot_setup" / "trajectories" / "fancy" / "heart.json"
DT = ms.DT                          # 0.05 s between samples (the runner interpolates a monotone cubic)
AMP = 0.15                          # rad: sweep amplitude about the pose's J4
SPEEDS = (0.05, 0.10, 0.20)         # rad/s: triangle-sweep speeds
CYCLES = 2                          # sweep cycles per speed (centre -> +A -> -A -> ... -> centre)
RAMP_S = 0.5                        # s: min-jerk velocity ramp at every corner (peak accel 1.875 v / RAMP_S)
STEP = 0.05                         # rad
STEP_S = 0.5                        # s: min-jerk step duration
STEP_HOLD_S = 2.0                   # s: hold after every step
STEP_PATTERN = (+1,) * 3 + (-1,) * 6 + (+1,) * 3
SETTLE_S = 2.0                      # s: hold on arriving at a pose
GAP_S = 1.0                         # s: hold between the sweep speeds
HEART_J4, CRACK_J4 = 0.2, 1.2
MAX_MINUTES = 4.0
J4_SPEED_FRACTION = 0.6             # of the runner's 1.2 rad/s J4 cap


def heart_pose(path: Path = HEART) -> np.ndarray:
    """J2/J3 = the heart's median (rounded to 1 mrad), J4 = HEART_J4, J1/J5/J6 = 0."""
    d = json.loads(path.read_text())
    q = np.median(np.array([p["positions"] for p in d["points"]]), axis=0)
    return np.array([0.0, round(float(q[1]), 3), round(float(q[2]), 3), HEART_J4, 0.0, 0.0])


def j4_gravity(ug: gmod.UrdfGravity, q: np.ndarray, extra) -> tuple[float, float]:
    """(today's J4 gravity feed-forward = URDF x 2.5, the fitted model's) at q, Nm."""
    g = ug.hold_torques(np.atleast_2d(q))[0, 3] * gmod.RUNNER_SCALE["j4"]
    return float(g), float(ug.hold_torques(np.atleast_2d(q), extra)[0, 3]) if extra is not None else float("nan")


def pose_problems(chk: ms.Checker, risk: ms.Risk, centre: np.ndarray) -> list[str]:
    """The survey's checks at the pose and at both sweep extremes (the steps stay inside them)."""
    out = []
    for dj in (-AMP, 0.0, AMP):
        q = centre + np.array([0, 0, 0, dj, 0, 0])
        out += [f"J4 {q[3]:+.3f}: {p}" for p in chk.problems(q) + risk.problems(chk, q)]
    return out


def crack_pose(chk: ms.Checker, risk: ms.Risk) -> tuple[np.ndarray, dict]:
    """The survey's accepted poses (make_gravity_survey.select_poses with the crack anchors and the model), J4 set to
    CRACK_J4, the clear ones ranked by |J4 gravity - the crack trajectories' median| then by the largest predicted J2/J3
    sag. Returns (pose, report). Raises IKError if none is clear."""
    anchors = ms.crack_anchors()
    poses, _ = ms.select_poses(chk, 20, anchors, risk)
    ug = gmod.UrdfGravity()
    crack_g = [j4_gravity(ug, a, risk.extra) for a in anchors]
    target = float(np.median([g[1] if risk.extra is not None else g[0] for g in crack_g])) if crack_g else float("nan")
    cands, rejected, seen = [], [], set()
    for p in poses:
        c = p.copy()
        c[0], c[3], c[4], c[5] = 0.0, CRACK_J4, 0.0, 0.0
        if (round(c[1], 3), round(c[2], 3)) in seen:          # survey poses that differ only in J4
            continue
        seen.add((round(c[1], 3), round(c[2], 3)))
        why = pose_problems(chk, risk, c)
        if why:
            rejected.append({"survey_pose": np.round(p, 3).tolist(), "why": why[0]})
            continue
        g_today, g_model = j4_gravity(ug, c, risk.extra)
        g = g_model if risk.extra is not None else g_today
        sag = float(np.abs(risk.sag(c)[1:3]).max() * 1000)
        cands.append((round(abs(g - target), 2) if math.isfinite(target) else 0.0, round(sag, 1), c, g_today, g_model, p))
    if not cands:
        raise IKError(f"no survey pose is clear with J4 at {CRACK_J4} +- {AMP}: {rejected[:3]}")
    cands.sort(key=lambda x: (x[0], x[1]))
    best = cands[0]
    rep = {"survey_pose": np.round(best[5], 3).tolist(), "crack_j4_gravity_median_nm": round(target, 3),
           "crack_j4_gravity_today_median_nm": round(float(np.median([g[0] for g in crack_g])), 3) if crack_g else None,
           "j4_gravity_today_nm": round(best[3], 3), "j4_gravity_model_nm": round(best[4], 3),
           "predicted_sag_today_mrad": np.round(risk.sag(best[2])[1:4] * 1000, 1).tolist(),
           "candidates": [{"q": np.round(c[2], 3).tolist(), "j4_gravity_gap_nm": c[0], "max_sag_j2j3_mrad": c[1]}
                          for c in cands],
           "rejected": rejected}
    return best[2], rep


# ---- the pieces ----------------------------------------------------------------------------------------------------

def _ramp_pos(s: np.ndarray) -> np.ndarray:
    """Integral of the min-jerk velocity shape 10s^3 - 15s^4 + 6s^5 (0 -> 0.5 over s = 0..1)."""
    return 2.5 * s**4 - 3 * s**5 + s**6


def leg(x0: float, x1: float, v: float) -> tuple[np.ndarray, float]:
    """J4 samples (DT apart, both ends) of one sweep leg x0 -> x1 at cruise speed ~v, starting and ending at rest with
    min-jerk velocity ramps of RAMP_S (so consecutive legs meet at a corner with a smooth velocity reversal).
    Returns (samples, the cruise speed used: the duration is rounded up to whole DT)."""
    D = abs(x1 - x0)
    T = math.ceil(round((D / v + RAMP_S) / DT, 6)) * DT
    V = D / (T - RAMP_S)
    t = np.linspace(0.0, T, int(round(T / DT)) + 1)
    x = np.where(t < RAMP_S, V * RAMP_S * _ramp_pos(np.clip(t / RAMP_S, 0, 1)),
                 np.where(t > T - RAMP_S, D - V * RAMP_S * _ramp_pos(np.clip((T - t) / RAMP_S, 0, 1)),
                          V * RAMP_S / 2 + V * (t - RAMP_S)))
    return x0 + math.copysign(1.0, x1 - x0) * x, V


def sweep(c: float, v: float) -> tuple[np.ndarray, float]:
    """centre -> +AMP -> -AMP -> ... (CYCLES) -> centre: J4 samples and the cruise speed of the long legs."""
    pts = [c + AMP] + [c - AMP, c + AMP] * (CYCLES - 1) + [c - AMP, c]
    out, cur, vs = [np.array([c])], c, []
    for p in pts:
        x, V = leg(cur, p, v)
        out.append(x[1:])
        vs.append(V)
        cur = p
    return np.concatenate(out), min(vs)


def min_jerk_1d(x0: float, x1: float, T: float) -> np.ndarray:
    s = np.linspace(0.0, 1.0, int(round(T / DT)) + 1)
    return x0 + (x1 - x0) * (10 * s**3 - 15 * s**4 + 6 * s**5)


class Builder:
    """Accumulates (t, q) samples; a hold is one interval of equal points (the runner's monotone cubic stays flat)."""

    def __init__(self) -> None:
        self.t, self.q = [0.0], [np.zeros(6)]
        self.sections: list[dict] = []

    @property
    def now(self) -> float:
        return round(self.t[-1], 6)

    def samples(self, qs: np.ndarray) -> None:
        for qq in qs[1:]:
            self.t.append(round(self.t[-1] + DT, 6))
            self.q.append(np.array(qq, dtype=float))

    def hold(self, T: float) -> None:
        self.t.append(round(self.t[-1] + T, 6))
        self.q.append(self.q[-1].copy())

    def wrist(self, j4: np.ndarray) -> None:
        base = self.q[-1]
        qs = np.repeat(base[None], len(j4), axis=0)
        qs[:, 3] = j4
        self.samples(qs)


def pose_block(b: Builder, name: str, centre: np.ndarray) -> dict:
    t_start = b.now
    secs = []
    b.hold(SETTLE_S)
    secs.append({"kind": "settle", "t0_s": t_start, "t1_s": b.now})
    c = float(centre[3])
    for k, v in enumerate(SPEEDS):
        t0 = b.now
        j4, V = sweep(c, v)
        b.wrist(j4)
        secs.append({"kind": "sweep", "speed_rad_s": round(V, 4), "t0_s": t0, "t1_s": b.now})
        b.hold(GAP_S if k < len(SPEEDS) - 1 else STEP_HOLD_S)
    t0 = b.now
    cur = c
    for d in STEP_PATTERN:
        b.wrist(min_jerk_1d(cur, cur + d * STEP, STEP_S))
        cur = round(cur + d * STEP, 6)
        b.hold(STEP_HOLD_S)
    secs.append({"kind": "steps", "step_rad": STEP, "step_s": STEP_S, "hold_s": STEP_HOLD_S,
                 "pattern": list(STEP_PATTERN), "t0_s": t0, "t1_s": b.now})
    assert abs(cur - c) < 1e-9
    return {"name": name, "q": np.round(centre, 4).tolist(), "j4_range": [round(c - AMP, 4), round(c + AMP, 4)],
            "window_s": [t_start, b.now], "sections": secs}


def transit(b: Builder, chk: ms.Checker, qb: np.ndarray) -> None:
    legs = ms.route(chk, b.q[-1], qb, ms.TRANSIT_MIN_S)
    if not legs:
        raise IKError(f"no clear transit {np.round(b.q[-1], 3).tolist()} -> {np.round(qb, 3).tolist()}")
    for lg in legs:
        b.samples(lg)


def make(model_path: Path | None = gmod.DEFAULT_MODEL, heart: Path = HEART) -> dict:
    """Returns {"doc": the trajectory document, "report": {...}}. Raises IKError when a check fails."""
    chk = ms.Checker()
    risk = ms.Risk(model_path)
    hp = heart_pose(heart)
    why = pose_problems(chk, risk, hp)
    if why:
        raise IKError(f"heart pose {hp.tolist()}: {why[0]}")
    cp, crack_rep = crack_pose(chk, risk)
    b = Builder()
    blocks = []
    for name, centre in (("heart", hp), ("crack", cp)):
        transit(b, chk, centre)
        blocks.append(pose_block(b, name, centre))
    transit(b, chk, np.zeros(6))
    t, q = np.array(b.t), np.array(b.q)
    # every sample: the pose blocks with the 0.05 rad window margin, the transits without (they leave the fold)
    in_block = np.zeros(len(t), dtype=bool)
    for blk in blocks:
        in_block |= (t >= blk["window_s"][0] - 1e-9) & (t <= blk["window_s"][1] + 1e-9)
    for i, qq in enumerate(q):
        p = chk.problems(qq, margin=ms.LIMIT_MARGIN if in_block[i] else 0.0)
        if p:
            raise IKError(f"t={t[i]:.2f} s q={np.round(qq, 3).tolist()}: {p[0]}")
    if np.abs(q[0]).max() or np.abs(q[-1]).max():
        raise IKError("must start and end at the fold")
    clear = check_motion_clearance(chk.ik, q, floor_z_m=ms.BODY_Z_MIN)
    frac = peak_speed_fraction(t, q)
    if frac.max() > ms.SPEED_FRACTION + 1e-6:
        raise IKError(f"peak speed {frac.max():.3f} of the cap > {ms.SPEED_FRACTION}")
    if frac[3] > J4_SPEED_FRACTION + 1e-6:
        raise IKError(f"J4 peak {frac[3]:.3f} of its cap > {J4_SPEED_FRACTION}")
    if t[-1] > MAX_MINUTES * 60:
        raise IKError(f"{t[-1]:.1f} s > {MAX_MINUTES} min at x1")
    gz = min(float(chk.ug.fk(q[i:i + 1]).t["gripper_link"][0, 2]) for i in range(len(q)) if np.abs(q[i]).max() > 1e-9)
    ug = gmod.UrdfGravity()
    wrist = np.zeros(len(t), dtype=bool)
    for blk in blocks:
        wrist |= (t > blk["window_s"][0] + SETTLE_S - 1e-9) & (t <= blk["window_s"][1] + 1e-9)
    v = np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]
    for blk, centre in zip(blocks, (hp, cp)):
        g_today, g_model = j4_gravity(ug, centre, risk.extra)
        blk |= {"j4_gravity_today_nm": round(g_today, 3), "j4_gravity_model_nm": round(g_model, 3),
                "predicted_sag_today_mrad": np.round(risk.sag(centre)[1:4] * 1000, 1).tolist(),
                "min_gripper_z_m": round(min(float(chk.ug.fk((centre + np.array([0, 0, 0, d, 0, 0]))[None]).t["gripper_link"][0, 2])
                                             for d in (-AMP, 0.0, AMP)), 4)}
    report = {"duration_s": round(float(t[-1]), 2), "n_points": int(len(t)),
              "peak_speed_rad_s": np.round(v.max(axis=0), 3).tolist(),
              "peak_speed_fraction_of_cap": np.round(frac, 3).tolist(),
              "peak_j4_speed_in_the_wrist_moves_rad_s": round(float(v[wrist[1:], 3].max()), 3),
              "min_body_z_m": clear["min_body_z_m"], "min_gripper_z_m": round(gz, 4),
              "joint_ranges": [[round(float(q[:, i].min()), 3), round(float(q[:, i].max()), 3)] for i in range(6)],
              "poses": blocks, "crack_pose_choice": crack_rep}
    notes = (f"J4 tuning ({len(blocks)} poses: heart mid pose J2/J3 {hp[1]:+.3f}/{hp[2]:+.3f} J4 {hp[3]:.2f}; crack-workspace "
             f"pose J2/J3 {cp[1]:+.3f}/{cp[2]:+.3f} J4 {cp[3]:.2f}). Wrist only (J1/J5/J6 0, J2/J3 held): at each pose "
             f"triangle sweeps +-{AMP} rad at {', '.join(f'{s:g}' for s in SPEEDS)} rad/s ({CYCLES} cycles, min-jerk "
             f"corners), then {STEP} rad min-jerk steps ({STEP_S} s) with {STEP_HOLD_S:g} s holds, both directions. "
             f"Starts and ends at the fold; transits <= {ms.SPEED_FRACTION:.0%} of the runner caps; arm_guard windows "
             f"(0.05 rad margin at the poses), gripper_link >= {ms.GRIPPER_Z_MIN} m, camera/mount >= {ms.BODY_Z_MIN} m, "
             "guard keep-outs + 1 cm; survey sag/cap filters at both poses. Analyse with tools/analyze_run_gains.py "
             "--friction-fit [--seg1-window T0:T1] (docs/motion/J4_TUNING.md)")
    doc = trajectory_doc(t, q, producer="tools/make_j4_tuning_trajectory.py", purpose="test", notes=notes,
                         extra={"j4_tune": {"amplitude_rad": AMP, "speeds_rad_s": list(SPEEDS), "cycles": CYCLES,
                                            "ramp_s": RAMP_S, "step_rad": STEP, "step_s": STEP_S,
                                            "step_hold_s": STEP_HOLD_S, "poses": blocks,
                                            "crack_pose_choice": {k: v for k, v in crack_rep.items() if k != "rejected"}}})
    return {"doc": doc, "report": report}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--model", type=Path, default=gmod.DEFAULT_MODEL, help="gravity model for the survey's sag/cap filters")
    ap.add_argument("--report", type=Path, help="write the JSON report here")
    args = ap.parse_args(argv)
    try:
        res = make(args.model)
    except IKError as e:
        print(f"REFUSED: {e}", file=sys.stderr)
        return 1
    rep = res["report"]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(res["doc"], indent=1) + "\n")
    for blk in rep["poses"]:
        print(f"pose {blk['name']}: q {blk['q']}, J4 {blk['j4_range'][0]:+.3f}..{blk['j4_range'][1]:+.3f}, window "
              f"{blk['window_s'][0]:.2f}..{blk['window_s'][1]:.2f} s; J4 gravity today {blk['j4_gravity_today_nm']:+.2f} / model "
              f"{blk['j4_gravity_model_nm']:+.2f} Nm; predicted sag today J2/J3/J4 {blk['predicted_sag_today_mrad']} mrad; "
              f"min gripper z {blk['min_gripper_z_m']} m")
        for s in blk["sections"]:
            print(f"    {s['kind']:6s} {s.get('speed_rad_s', '')!s:7s} {s['t0_s']:7.2f}..{s['t1_s']:7.2f} s")
    print(f"crack pose from survey pose {rep['crack_pose_choice']['survey_pose']} (crack trajectories' median J4 gravity "
          f"{rep['crack_pose_choice']['crack_j4_gravity_median_nm']:+.2f} Nm with the model, "
          f"{rep['crack_pose_choice']['crack_j4_gravity_today_median_nm']:+.2f} today; this pose "
          f"{rep['crack_pose_choice']['j4_gravity_model_nm']:+.2f} / {rep['crack_pose_choice']['j4_gravity_today_nm']:+.2f})")
    print(f"{rep['duration_s']} s ({rep['duration_s'] / 60:.2f} min) at x1, {rep['n_points']} points, peak speeds "
          f"{rep['peak_speed_rad_s']} rad/s (fraction of cap {rep['peak_speed_fraction_of_cap']}), J4 in the wrist moves "
          f"<= {rep['peak_j4_speed_in_the_wrist_moves_rad_s']} rad/s, min gripper z {rep['min_gripper_z_m']} m, min "
          f"camera/mount z {rep['min_body_z_m']} m -> {args.out}")
    if args.report:
        args.report.write_text(json.dumps(rep, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

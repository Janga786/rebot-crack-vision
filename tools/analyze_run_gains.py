"""tools/analyze_run_gains.py — what the gains and the feed-forward did in a demo_move.py run (offline, from its CSV).

For each arm joint, over the moving path ('path' rows of the run CSV): the mean and rms tracking error (ref - pos, so
+ = the arm lags behind the command), its peak, the 2.5-40 Hz ripple, how much more torque than the feed-forward the
P term had to supply (mean torque - ff), and the same split by segment (approach / trajectory / return). With
--directions it also splits the near-stationary samples by the direction of the preceding move, which separates a
gravity-model bias (the same in both directions) from friction (opposite signs). A steady error with a matching
torque offset is a feed-forward problem, not a gain problem; ripple is the oscillation the encoders can see.

  ./env.sh python tools/analyze_run_gains.py RUN.csv [RUN2.csv ...] [--directions]    (also reads .csv.gz)
  ./env.sh python tools/analyze_run_gains.py RUN.csv --gravity-model config/robot/gravity_model.yaml
        re-score a past run against a fitted gravity model (crackvision.gravity_model/1): what the P and D terms would
        have had to supply had demo_move.py run with --gravity-model (same Coulomb term, the same trim at the caps),
        next to what they did supply, in Nm and in mrad at the tuned kp (the sag)
  ./env.sh python tools/analyze_run_gains.py --compare A.csv B.csv
        an A/B of two runs (e.g. the heart without and with --gravity-model): every statistic side by side, B - A
  ./env.sh python tools/analyze_run_gains.py RUN.csv [...] --friction-fit [--what-if-friction j4=0.18] [--what-if-kp j4=50]
        per joint (J1-J5) and segment (0 approach, 1 path, 2 return): least squares of the tracking error e = ref - pos
        and of the torque the P/D terms supplied (torq - ff) on [1, tanh(v_ref/0.02), v_ref, a_ref] (v_ref, a_ref by
        finite differences of the logged ref): constant, sign-of-velocity (Coulomb) term, viscous and inertia
        coefficients, R2; the implied real Coulomb friction = the runner's Coulomb feed-forward + the sign coefficient
        of the torque fit (an over-compensating feed-forward gives a negative coefficient); the coherence of each
        joint's error with J3's at the 2.5-3.5 Hz ripple line (Welch, Hann, 5.1 s segments) and how much of the 2.5-40
        Hz band that shared part carries; and a what-if: the error had the run used another Coulomb value (the sign term
        moved by (F_run - X) / kp) and/or another kp (the whole error x kp_run / kp, quasi-static). Several runs: also
        the across-run mean / sd of the implied Coulomb (docs/motion/J4_TUNING.md)
  --seg1-window T0:T1 (with any of the above): only that part of the TRAJECTORY segment (seconds of trajectory-file
        time from the start of that segment, x the run's slow), e.g. one pose of ~/rebot_setup/trajectories/j4_tune.json

The trajectory segment is found by its kind, not its number: demo_move.py (2026-10-29.3+) writes "path_kinds" (the
(kind, duration) of every segment) into the run summary RUN.json. A zero-length approach or return is dropped (e.g.
j4_tune.json / gravity_survey.json, which start and end at the fold, run with --j1-here-is-zero), so the trajectory can
be segment 0. Older summaries: inferred from the trajectory file's duration. No summary: segment 1 is ASSUMED, with a
WARNING (as are slow 1 and the tuned kp / Coulomb values). Keep every RUN.json next to its RUN.csv; do not rename them.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

JOINTS = ["j1", "j2", "j3", "j4", "j5"]


USED_COLS = ["t_s", "seg"] + [f"j{i}_{c}" for i in range(1, 7) for c in ("ref", "pos", "vel", "torq", "ff")]


def load(path: Path) -> list[dict]:
    """The 'path' rows of a run CSV that have every used column (a row with an empty cell, e.g. a motor that gave no
    feedback that cycle, is dropped instead of crashing the float conversions)."""
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt", newline="") as fh:
        return [x for x in csv.DictReader(fh) if x.get("mode") == "path"
                and all((x.get(c) or "").strip() != "" for c in USED_COLS)]


def band_rms(x: np.ndarray, dt: float, lo: float = 2.5, hi: float = 40.0) -> tuple[float, float]:
    """(rms of x in the lo..hi Hz band, the frequency of the strongest line in it). A Hann-windowed FFT with Parseval's
    window correction, so a sinusoid of amplitude A reads A/sqrt(2) wherever it sits in the band (a moving-average
    high-pass would attenuate the lowest frequencies, a 3 Hz line by about 6x)."""
    x = np.asarray(x, dtype=float) - np.mean(x)
    w = np.hanning(len(x))
    X = np.fft.rfft(x * w)
    f = np.fft.rfftfreq(len(x), dt)
    P = np.abs(X) ** 2
    band = (f >= lo) & (f <= hi)
    rms = float(np.sqrt(2.0 * P[band].sum() / (len(x) * np.sum(w**2))))
    return rms, float(f[band][P[band].argmax()])


HOLD_VREF = 0.005      # rad/s: |v_ref| below this = holding (for the oscillation-flip count)
FLIP_VEL = 0.05        # rad/s: a flip = the measured velocity changes sign with |vel| above this (CONTROLLER_TUNING §8)


def hold_flips(v_ref: np.ndarray, vel: np.ndarray) -> int:
    """Oscillation flips while holding: sign changes of the measured velocity, counting only readings with
    |vel| > FLIP_VEL, inside stretches where |v_ref| < HOLD_VREF (CONTROLLER_TUNING §8, N_k)."""
    n, last = 0, 0.0
    for vr, v in zip(v_ref, vel):
        if abs(vr) >= HOLD_VREF:
            last = 0.0
            continue
        if abs(v) > FLIP_VEL:
            if last and np.sign(v) != np.sign(last):
                n += 1
            last = v
    return n


def analyse(rows: list[dict]) -> dict:
    t = np.array([float(x["t_s"]) for x in rows])
    dt = float(np.median(np.diff(t)))
    seg = np.array([int(x["seg"]) for x in rows])
    out = {}
    for j in JOINTS:
        g = lambda c: np.array([float(x[f"{j}_{c}"]) for x in rows])
        e, tq, ff = g("ref") - g("pos"), g("torq"), g("ff")
        ripple_mrad, ripple_hz = band_rms(e, dt)
        flips = hold_flips(np.gradient(g("ref"), dt), g("vel"))
        out[j] = {"mean_err_mrad": round(float(e.mean() * 1000), 1), "rms_err_mrad": round(float(e.std() * 1000), 1),
                  "max_err_mrad": round(float(abs(e).max() * 1000), 1),
                  "ripple_mrad": round(ripple_mrad * 1000, 2), "ripple_hz": round(ripple_hz, 1),
                  "torque_minus_ff_mean_nm": round(float((tq - ff).mean()), 2),
                  "peak_torque_nm": round(float(abs(tq).max()), 1), "peak_ff_nm": round(float(abs(ff).max()), 1),
                  "hold_flips": flips,
                  "by_segment_rms_mrad": {int(s): round(float(e[seg == s].std() * 1000), 1) for s in np.unique(seg)}}
    return out


def directions(rows: list[dict]) -> dict:
    t = np.array([float(x["t_s"]) for x in rows])
    dt = float(np.median(np.diff(t)))
    k, K = max(2, int(0.2 / dt)), int(1.0 / dt)
    out = {}
    for j in JOINTS[:4]:
        g = lambda c: np.array([float(x[f"{j}_{c}"]) for x in rows])
        ref, pos, tq, ff = g("ref"), g("pos"), g("torq"), g("ff")
        vref = np.gradient(np.convolve(ref, np.ones(k) / k, mode="same"), dt)
        acc = {1: [], -1: []}
        for i in range(K, len(rows) - k):
            if abs(vref[i]) < 0.01 and abs(ref[i] - ref[i - K]) > 0.01:
                acc[1 if ref[i] > ref[i - K] else -1].append((tq[i] - ff[i], ref[i] - pos[i]))
        out[j] = {("up" if d > 0 else "down"): {"n": len(a), "torque_minus_ff_nm": round(float(np.mean([v[0] for v in a])), 2),
                                                 "static_err_mrad": round(float(np.mean([v[1] for v in a]) * 1000), 1)}
                  for d, a in acc.items() if a}
    return out


def rescore(rows: list[dict], model: dict) -> dict:
    """Per joint (J1-J5): mean / rms of torque - feed-forward as logged, and as it would have been with the gravity
    model (demo_move.py --gravity-model: s x model on the covered joints, URDF x the tuned scale elsewhere, + the
    Coulomb term on the commanded velocity, trimmed to stay inside the caps), in Nm and in mrad at the tuned kp."""
    from crackvision import gravity_model as gmod
    ug = gmod.UrdfGravity()
    t = np.array([float(x["t_s"]) for x in rows])
    dt = float(np.median(np.diff(t)))
    col = lambda c: np.array([[float(x[f"j{i}_{c}"]) for i in range(1, 7)] for x in rows])
    q, ref, tq, ff_log = col("pos"), col("ref"), col("torq"), col("ff")
    scale = model.get("scale", {})
    g_m = ug.hold_torques(q, gmod.extra_bodies(model))
    g_u = ug.hold_torques(q)
    grav = np.column_stack([g_m[:, i] * scale[k] if k in scale else g_u[:, i] * gmod.RUNNER_SCALE[k]
                            for i, k in enumerate(gmod.ARM)])
    ff, limited, refused = gmod.model_feed_forward(grav, gmod.runner_friction(np.gradient(ref, dt, axis=0)))
    seg = np.array([int(x["seg"]) for x in rows])
    out = {}
    for i, k in enumerate(gmod.ARM[:5]):
        kp = gmod.KP[k]
        a, b = tq[:, i] - ff_log[:, i], tq[:, i] - ff[:, i]
        out[k] = {"logged_mean_nm": round(float(a.mean()), 2), "logged_rms_nm": round(float(np.sqrt((a**2).mean())), 2),
                  "model_mean_nm": round(float(b.mean()), 2), "model_rms_nm": round(float(np.sqrt((b**2).mean())), 2),
                  "logged_mean_mrad": round(float(a.mean() / kp * 1000), 1), "model_mean_mrad": round(float(b.mean() / kp * 1000), 1),
                  "logged_rms_mrad": round(float(np.sqrt((a**2).mean()) / kp * 1000), 1),
                  "model_rms_mrad": round(float(np.sqrt((b**2).mean()) / kp * 1000), 1),
                  "model_by_segment_mean_mrad": {int(s_): round(float(b[seg == s_].mean() / kp * 1000), 1) for s_ in np.unique(seg)},
                  "peak_model_ff_nm": round(float(np.abs(ff[:, i]).max()), 2),
                  "trimmed_s": round(float(limited[:, i].sum() * dt), 2), "refused_s": round(float(refused[:, i].sum() * dt), 2)}
    return out


FRICTION_VEL_EPS = 0.02          # rad/s: the runner's tanh(v_ref / eps) smooth sign (first_move_j1.FRICTION_VEL_EPS)
RIPPLE_LINE_HZ = (2.5, 3.5)      # the ~3 Hz J2/J3 mode (powered show runs, analysis.txt)
BAND_HZ = (2.5, 40.0)            # the ripple band of band_rms()
WELCH_NPERSEG = 1024             # 5.1 s at 200 Hz: 0.2 Hz resolution; a 10-20 s path gives 3-7 averaged segments
VIF_FLAG = 15.0                  # variance inflation of the sign column above which it is confounded with the
                                 # constant / viscous columns (a segment that moves in one direction only)


def run_params(csv_path: Path) -> tuple[dict, dict, str]:
    """(kp per joint, Coulomb feed-forward per joint, where they came from) for a run: its demo_move.py summary (gains
    and friction as used, including per-run overrides), else the tuned defaults (crackvision.gravity_model's copy)."""
    from crackvision import gravity_model as gmod
    s = run_summary(csv_path)
    kp = {j: float(gmod.KP[j]) for j in JOINTS}
    fric = {j: float(gmod.RUNNER_FRICTION[j]) for j in JOINTS}
    src = "tuned defaults" + ("" if s else " (NO RUN SUMMARY: ASSUMED)")
    if isinstance(s.get("gains"), dict) and isinstance(s.get("friction"), dict):
        kp |= {j: float(s["gains"][j][0]) for j in JOINTS if j in s["gains"]}
        fric |= {j: float(s["friction"][j]) for j in JOINTS if j in s["friction"]}
        src = "run summary"
    return kp, fric, src


def parse_joint_values(spec: str, lo: float, hi: float, what: str) -> dict[str, float]:
    """'j4=0.18,j3=1.5' -> {'j4': 0.18, 'j3': 1.5}, each in [lo, hi], joints j1..j5."""
    out = {}
    for item in spec.split(","):
        try:
            name, val = item.strip().split("=")
            x = float(val)
        except ValueError:
            raise ValueError(f"{item!r}: expected joint=value, e.g. j4=0.18") from None
        if name not in JOINTS:
            raise ValueError(f"{name!r} is not one of {JOINTS}")
        if not (np.isfinite(x) and lo <= x <= hi):
            raise ValueError(f"{name}: {what} {x:g} outside [{lo:g}, {hi:g}]")
        out[name] = x
    return out


def lstsq(X: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float]:
    """(coefficients, R2) of y ~ X."""
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    var = float(np.var(y))
    return c, (1.0 - float(np.var(y - X @ c)) / var) if var > 0 else 0.0


def vif(X: np.ndarray, k: int) -> float:
    """Variance inflation factor of column k of X (how well the other columns predict it; 1 = independent)."""
    o = [i for i in range(X.shape[1]) if i != k]
    c, *_ = np.linalg.lstsq(X[:, o], X[:, k], rcond=None)
    r = float(np.var(X[:, k] - X[:, o] @ c))
    return float(np.var(X[:, k])) / r if r > 1e-30 else float("inf")


def coherence_with(x: np.ndarray, y: np.ndarray, dt: float) -> dict | None:
    """Coherence of x with y (Welch, Hann, WELCH_NPERSEG, 50 % overlap): its largest value in RIPPLE_LINE_HZ and where,
    the level a zero true coherence stays under 95 % of the time with this many segments, and x's 2.5-40 Hz band rms
    split into the part linearly shared with y (sum of coherence x PSD) and the rest (what x's own damping can act on)."""
    from scipy.signal import coherence, welch
    if np.std(x) < 1e-7 or np.std(y) < 1e-7:          # below the encoder's resolution: nothing to correlate
        return None
    n = len(x)
    nper = min(WELCH_NPERSEG, n)
    f, C = coherence(x, y, fs=1.0 / dt, window="hann", nperseg=nper)
    _, P = welch(x, fs=1.0 / dt, window="hann", nperseg=nper)
    line = (f >= RIPPLE_LINE_HZ[0]) & (f <= RIPPLE_LINE_HZ[1])
    band = (f >= BAND_HZ[0]) & (f <= BAND_HZ[1])
    df = float(f[1] - f[0])
    n_seg = max(1, (n - nper) // (nper // 2) + 1)
    k = int(np.argmax(np.where(line, C, -1.0)))
    return {"coherence": round(float(C[k]), 2), "at_hz": round(float(f[k]), 2), "n_segments": n_seg,
            "zero_coherence_95": round(float(1.0 - 0.05 ** (1.0 / (n_seg - 1))) if n_seg > 1 else 1.0, 2),
            "band_rms_mrad": round(float(np.sqrt(P[band].sum() * df)) * 1000, 2),
            "shared_band_rms_mrad": round(float(np.sqrt((C[band] * P[band]).sum() * df)) * 1000, 2),
            "own_band_rms_mrad": round(float(np.sqrt((np.clip(1.0 - C[band], 0, 1) * P[band]).sum() * df)) * 1000, 2)}


def friction_fit(rows: list[dict], kp: dict, fric: dict, what_if_friction: dict | None = None,
                 what_if_kp: dict | None = None) -> dict:
    """Per joint and segment, the regression of e = ref - pos [rad] and of torq - ff [Nm] on
    [1, tanh(v_ref / FRICTION_VEL_EPS), v_ref, a_ref]; see the module docstring. Units: e coefficients in mrad (per
    rad/s, per rad/s^2), torque coefficients in Nm (Nm.s/rad, Nm.s^2/rad)."""
    what_if_friction, what_if_kp = what_if_friction or {}, what_if_kp or {}
    t = np.array([float(x["t_s"]) for x in rows])
    dt = float(np.median(np.diff(t)))
    seg = np.array([int(x["seg"]) for x in rows])
    col = lambda j, c: np.array([float(x[f"{j}_{c}"]) for x in rows])
    e3 = col("j3", "ref") - col("j3", "pos")
    out = {}
    for j in JOINTS:
        ref = col(j, "ref")
        e, dtq = ref - col(j, "pos"), col(j, "torq") - col(j, "ff")
        v = np.gradient(ref, dt)
        a = np.gradient(v, dt)
        sgn = np.tanh(v / FRICTION_VEL_EPS)
        F, K = fric[j], kp[j]
        Xf, Kf = what_if_friction.get(j, F), what_if_kp.get(j, K)
        e_wi = (e + (F - Xf) / K * sgn) * K / Kf          # quasi-static: a torque deficit d gives e = d / kp
        jo = {"kp": K, "runner_friction_nm": F, "segments": {}}
        for s_ in np.unique(seg):
            m = seg == s_
            if m.sum() < 50:
                continue
            d = {"n": int(m.sum()), "v_ref_max": round(float(np.abs(v[m]).max()), 3),
                 "a_ref_max": round(float(np.abs(a[m]).max()), 2),
                 "mean_err_mrad": round(float(e[m].mean() * 1000), 2), "rms_err_mrad": round(float(e[m].std() * 1000), 2)}
            if np.abs(ref[m] - ref[m][0]).max() < 0.01 or (np.abs(sgn[m]) > 0.5).sum() < 20:
                jo["segments"][int(s_)] = d | {"static": True}     # the joint (nearly) does not move here: no fit
                continue
            X = np.column_stack([np.ones(m.sum()), sgn[m], v[m], a[m]])
            ce, r2e = lstsq(X, e[m])
            ct, r2t = lstsq(X, dtq[m])
            Xa = np.column_stack([np.ones(m.sum()), np.maximum(sgn[m], 0.0), np.minimum(sgn[m], 0.0), v[m], a[m]])
            ca, _ = lstsq(Xa, dtq[m]) if (sgn[m] > 0.5).sum() >= 20 and (sgn[m] < -0.5).sum() >= 20 else (None, 0.0)
            vf = vif(X, 1)
            d |= {"error": {"const_mrad": round(ce[0] * 1000, 2), "sgn_mrad": round(ce[1] * 1000, 2),
                           "sgn_nm_at_kp": round(ce[1] * K, 3), "visc_mrad_per_rad_s": round(ce[2] * 1000, 2),
                           "inertia_mrad_per_rad_s2": round(ce[3] * 1000, 3), "r2": round(r2e, 3)},
                 "torque": {"const_nm": round(ct[0], 3), "sgn_nm": round(ct[1], 3), "visc_nm_s_per_rad": round(ct[2], 3),
                            "inertia_nm_s2_per_rad": round(ct[3], 4), "r2": round(r2t, 3)},
                 "vif_sgn": round(vf, 1), "confounded": bool(vf > VIF_FLAG),
                 "implied_coulomb_nm": round(F + ct[1], 3)}
            if ca is not None:
                d["implied_coulomb_up_down_nm"] = [round(F + ca[1], 3), round(F + ca[2], 3)]
            if j != "j3" and np.std(e3[m]) > 0:
                d["corr_with_j3"] = round(float(np.corrcoef(e[m], e3[m])[0, 1]), 2)
                d["coherence_with_j3"] = coherence_with(e[m], e3[m], dt)
            if j in what_if_friction or j in what_if_kp:
                d["what_if"] = {"friction_nm": Xf, "kp": Kf,
                                "sgn_mrad": round((ce[1] + (F - Xf) / K) * K / Kf * 1000, 2),
                                "mean_err_mrad": round(float(e_wi[m].mean() * 1000), 2),
                                "rms_err_mrad": round(float(e_wi[m].std() * 1000), 2),
                                "rms_about_zero_mrad": round(float(np.sqrt((e_wi[m] ** 2).mean()) * 1000), 2)}
            jo["segments"][int(s_)] = d
        out[j] = jo
    return out


def friction_summary(results: dict, seg: int | None = None) -> dict:
    """Across runs, per joint: mean / sd / min / max of the implied Coulomb (and of the sign term) on one segment: each
    run's trajectory segment (r["trajectory_seg"], else 1) unless seg is given."""
    out = {}
    for j in JOINTS:
        ds = [r["friction_fit"][j]["segments"].get(seg if seg is not None else r.get("trajectory_seg", 1))
              for r in results.values() if "friction_fit" in r]
        ds = [d for d in ds if d is not None and not d.get("static")]
        c = [d["implied_coulomb_nm"] for d in ds]
        s = [d["error"]["sgn_mrad"] for d in ds]
        if c:
            out[j] = {"n_runs": len(c), "implied_coulomb_mean_nm": round(float(np.mean(c)), 3),
                      "implied_coulomb_sd_nm": round(float(np.std(c, ddof=1)) if len(c) > 1 else 0.0, 3),
                      "implied_coulomb_min_nm": round(float(np.min(c)), 3), "implied_coulomb_max_nm": round(float(np.max(c)), 3),
                      "sgn_mean_mrad": round(float(np.mean(s)), 2)}
    return out


def print_friction_fit(name: str, src: str, ff: dict, seg: int = 1, labels: str = "0 approach, 1 path, 2 return") -> None:
    print(f"  friction fit (kp / Coulomb feed-forward F_ff from the {src}), segment {seg} (the trajectory). e in mrad "
          "(visc per rad/s, inert per rad/s2), tq-ff in Nm; * = sign column confounded (VIF > 15)")
    print("  joint  kp  F_ff |  e: const   sgn  [Nm@kp]   visc  inert   R2 | tq-ff: const   sgn    visc   inert    R2 | "
          "implied F (up/down) | corr e3  coh e3 @Hz (n seg, 95% zero)  2.5-40 Hz rms: shared/own | what-if: F  kp  sgn  mean  rms")
    for j, jo in ff.items():
        d = jo["segments"].get(seg)
        if d is None or d.get("static"):
            print(f"  {j}  {jo['kp']:4.0f} {jo['runner_friction_nm']:4.2f} | (does not move in segment {seg})")
            continue
        E, T = d["error"], d["torque"]
        c = d.get("coherence_with_j3")
        coh = (f"{d['corr_with_j3']:+5.2f}  {c['coherence']:4.2f}@{c['at_hz']:4.2f} ({c['n_segments']}, {c['zero_coherence_95']:.2f})"
               f"           {c['band_rms_mrad']:4.2f}: {c['shared_band_rms_mrad']:4.2f}/{c['own_band_rms_mrad']:4.2f}") if c else \
            f"{'(reference joint)' if j == 'j3' else '(J3 error constant: no coherence)':54s}"
        ud = d.get("implied_coulomb_up_down_nm")
        uds = f"({ud[0]:4.2f}/{ud[1]:4.2f})" if ud else "(one direction)"
        wi = d.get("what_if")
        wis = (f" | {wi['friction_nm']:4.2f} {wi['kp']:3.0f} {wi['sgn_mrad']:+5.1f} {wi['mean_err_mrad']:+5.1f} {wi['rms_err_mrad']:4.1f}"
               if wi else "")
        print(f"  {j}  {jo['kp']:4.0f} {jo['runner_friction_nm']:4.2f} | {E['const_mrad']:+6.1f} {E['sgn_mrad']:+6.1f} "
              f"[{E['sgn_nm_at_kp']:+5.2f}] {E['visc_mrad_per_rad_s']:+6.1f} {E['inertia_mrad_per_rad_s2']:+6.2f} {E['r2']:4.2f} | "
              f"{T['const_nm']:+6.2f} {T['sgn_nm']:+6.3f} {T['visc_nm_s_per_rad']:+6.2f} {T['inertia_nm_s2_per_rad']:+7.4f} {T['r2']:4.2f}"
              f"{'*' if d['confounded'] else ' '}| {d['implied_coulomb_nm']:5.2f} {uds:13s} | {coh}{wis}")
    has_wi = any("what_if" in d for jo in ff.values() for d in jo["segments"].values())
    print(f"  by segment ({labels}): e sgn mrad / tq-ff sgn Nm / implied F / R2(e) / rms mrad"
          + (" -> what-if rms" if has_wi else ""))
    for j, jo in ff.items():
        parts = []
        for s_, d in jo["segments"].items():
            w = d.get("what_if")
            if d.get("static"):
                parts.append(f"{s_}: static, rms {d['rms_err_mrad']:4.1f}")
                continue
            parts.append(f"{s_}: {d['error']['sgn_mrad']:+5.1f} / {d['torque']['sgn_nm']:+5.2f} / {d['implied_coulomb_nm']:5.2f} / "
                         f"{d['error']['r2']:4.2f} / {d['rms_err_mrad']:4.1f}" + (f" -> {w['rms_err_mrad']:4.1f}" if w else "")
                         + ("*" if d["confounded"] else ""))
        print(f"  {j}  " + "   ".join(parts))


def compare(a: dict, b: dict) -> dict:
    """B - A for every per-joint number of analyse()."""
    out = {}
    for j in a:
        out[j] = {}
        for k, va in a[j].items():
            vb = b[j][k]
            if isinstance(va, (int, float)):
                out[j][k] = {"A": va, "B": vb, "B-A": round(vb - va, 2)}
            elif isinstance(va, dict):
                out[j][k] = {s: {"A": va.get(s), "B": vb.get(s)} for s in sorted(set(va) | set(vb))}
    return out


def seg1_window(rows: list[dict], window: tuple[float, float], csv_path: Path) -> list[dict]:
    """The rows of the trajectory segment (trajectory_segment: by kind, not number) between window[0] and window[1]
    seconds of trajectory-file time from that segment's start (x the run's slow)."""
    slow = float(run_summary(csv_path).get("slow") or 1.0)
    k = str(trajectory_segment(csv_path, rows)[0])
    s1 = [x for x in rows if x["seg"] == k]
    if not s1:
        return []
    t0 = float(s1[0]["t_s"])
    return [x for x in s1 if window[0] * slow <= float(x["t_s"]) - t0 <= window[1] * slow]


_WARNED: set[str] = set()


def summary_path(csv_path: Path) -> Path:
    return Path(str(csv_path).removesuffix(".gz").removesuffix(".csv") + ".json")


def run_summary(csv_path: Path) -> dict:
    """The demo_move.py summary next to a run log (RUN.json), if there is one. Missing: a loud WARNING (once per run)
    naming what is then assumed."""
    p = summary_path(csv_path)
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError):
        if str(p) not in _WARNED:
            _WARNED.add(str(p))
            from crackvision import gravity_model as gmod
            print(f"WARNING: no run summary {p.name} next to {Path(csv_path).name}: ASSUMING slow 1, kp "
                  + ", ".join(f"{j} {gmod.KP[j]:g}" for j in JOINTS) + ", Coulomb feed-forward "
                  + ", ".join(f"{j} {gmod.RUNNER_FRICTION[j]:g}" for j in JOINTS)
                  + " Nm (the tuned defaults; per-run --gains/--friction overrides are NOT known) and segment 1 = the "
                  "trajectory. Numbers that depend on them may be WRONG. Keep every .json next to its .csv; do not "
                  "rename either.", file=sys.stderr, flush=True)
        return {}


def _traj_file_duration(s: dict) -> float | None:
    """An older summary (no path_kinds): the trajectory file's duration x slow, if the file is still there unchanged."""
    h = s.get("trajectory") or {}
    try:
        raw = Path(h["path"]).read_bytes()
        if hashlib.sha256(raw).hexdigest() != h.get("sha256"):
            return None
        return float(json.loads(raw)["points"][-1]["t_s"]) * float(s.get("slow") or 1.0)
    except (OSError, KeyError, ValueError, TypeError, IndexError):
        return None


def trajectory_segment(csv_path: Path, rows: list[dict]) -> tuple[int, str, list[str] | None]:
    """(the segment number of the trajectory, how it was found, the kind of every segment or None). rows = the run's
    path rows (before any window)."""
    s = run_summary(csv_path)
    segs = sorted({int(x["seg"]) for x in rows})
    kinds = s.get("path_kinds")
    if isinstance(kinds, list) and kinds:
        names = [str(k[0]) if isinstance(k, (list, tuple)) else str(k) for k in kinds]
        if "trajectory" in names:
            return names.index("trajectory"), "run summary path_kinds", names
    if s.get("choreo") == "trajectory":
        dur = _traj_file_duration(s)
        if dur:
            span = {k: float(rr[-1]["t_s"]) - float(rr[0]["t_s"]) for k in segs
                    for rr in [[x for x in rows if int(x["seg"]) == k]]}
            k = min(span, key=lambda k: abs(span[k] - dur))
            if abs(span[k] - dur) <= 0.05 + 0.02 * dur:
                return k, "inferred: the segment as long as the trajectory file", None
        if segs == [0, 1, 2]:
            return 1, "inferred: 3 segments", ["approach", "trajectory", "return"]
        if len(segs) == 1:
            return segs[0], "inferred: 1 segment", ["trajectory"]
        print(f"WARNING: {Path(csv_path).name}: cannot tell which segment is the trajectory (segments {segs}, the "
              "summary has no path_kinds); ASSUMING segment 1", file=sys.stderr, flush=True)
        return 1, "ASSUMED segment 1 (ambiguous)", None
    if not s:
        if len(segs) == 1:
            return segs[0], "ASSUMED (no run summary; the only segment)", None
        print(f"WARNING: {Path(csv_path).name}: no run summary: ASSUMING segment 1 is the trajectory (segments "
              f"{segs})", file=sys.stderr, flush=True)
        return 1, "ASSUMED segment 1 (no run summary)", None
    return 1, "choreography run: segment 1", None


def seg_labels(kinds: list[str] | None, segs) -> str:
    if kinds:
        return ", ".join(f"{k} {n}" for k, n in enumerate(kinds))
    return ", ".join(str(k) for k in segs) + " (kinds unknown)"


def outcome_text(csv_path: Path) -> str:
    s = run_summary(csv_path)
    if not s:
        return "outcome unknown (no run summary)"
    trig = s.get("trigger") or {}
    return f"outcome {s.get('outcome')}" + (f" (trigger {trig.get('id')}: {trig.get('detail')})" if trig else "")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("csv", nargs="*", type=Path)
    ap.add_argument("--directions", action="store_true", help="split near-stationary samples by approach direction")
    ap.add_argument("--gravity-model", type=Path, metavar="FILE",
                    help="re-score each run against this crackvision.gravity_model/1 file")
    ap.add_argument("--compare", nargs=2, type=Path, metavar=("A.csv", "B.csv"), help="A/B of two runs")
    ap.add_argument("--friction-fit", action="store_true",
                    help="regress error and torq - ff on [1, sgn(v_ref), v_ref, a_ref] per joint and segment")
    ap.add_argument("--what-if-friction", metavar="SPEC", default=None,
                    help="with --friction-fit: the error had the run used this Coulomb feed-forward, e.g. j4=0.18 (Nm)")
    ap.add_argument("--what-if-kp", metavar="SPEC", default=None,
                    help="with --friction-fit: ... and this kp, e.g. j4=50 (quasi-static: error x kp_run / kp)")
    ap.add_argument("--seg1-window", metavar="T0:T1", default=None,
                    help="keep only this part of segment 1 (the trajectory): seconds of FILE time from its start (x the "
                         "run's --slow from its summary), e.g. one pose of j4_tune.json (its j4_tune.poses[].window_s)")
    ap.add_argument("--json", action="store_true", help="print JSON instead of a table")
    args = ap.parse_args(argv)
    window = None
    if args.seg1_window:
        try:
            window = tuple(float(x) for x in args.seg1_window.split(":"))
            assert len(window) == 2 and 0 <= window[0] < window[1]
        except (ValueError, AssertionError):
            ap.error(f"--seg1-window {args.seg1_window!r}: expected T0:T1 with 0 <= T0 < T1")
    try:
        wif = parse_joint_values(args.what_if_friction, 0.0, 3.0, "Coulomb Nm") if args.what_if_friction else {}
        wik = parse_joint_values(args.what_if_kp, 1.0, 100.0, "kp") if args.what_if_kp else {}
    except ValueError as e:
        ap.error(str(e))
    if (wif or wik) and not args.friction_fit:
        ap.error("--what-if-friction / --what-if-kp need --friction-fit")
    if args.compare:
        ra, rb = (seg1_window(load(p), window, p) if window else load(p) for p in args.compare)
        for tag, p in zip("AB", args.compare):
            print(f"{tag}: {outcome_text(p)}" + (f"; window {window[0]:g}..{window[1]:g} s of segment "
                  f"{trajectory_segment(p, load(p))[0]} (the trajectory)" if window else ""),
                  file=sys.stderr if args.json else sys.stdout)
        if min(len(ra), len(rb)) < 500:
            print("--compare: a run with fewer than 500 path rows (aborted or refused?)")
            return 1
        cmp_ = compare(analyse(ra), analyse(rb))
        ga, gb = (run_summary(p).get("gravity_model") for p in args.compare)
        if args.json:
            print(json.dumps({"A": str(args.compare[0]), "B": str(args.compare[1]), "gravity_model": [ga, gb],
                              "joints": cmp_}, indent=1))
            return 0
        print(f"A = {args.compare[0].name}" + (f"  (gravity model {Path(ga['file']).name})" if ga else ""))
        print(f"B = {args.compare[1].name}" + (f"  (gravity model {Path(gb['file']).name})" if gb else ""))
        print("  joint   mean err (mrad) A -> B     rms err (mrad) A -> B     max err (mrad) A -> B    tq-ff (Nm) A -> B"
              "    ripple (mrad) A -> B     hold flips A -> B")
        for j, d in cmp_.items():
            f = lambda k: f"{d[k]['A']:+6.1f} -> {d[k]['B']:+6.1f} ({d[k]['B-A']:+5.1f})"
            f2 = lambda k: f"{d[k]['A']:5.2f} -> {d[k]['B']:5.2f} ({d[k]['B-A']:+5.2f})"   # ripple: to 0.01 mrad
            print(f"  {j}  {f('mean_err_mrad')}  {f('rms_err_mrad')}  {f('max_err_mrad')}  {f('torque_minus_ff_mean_nm')}"
                  f"  {f2('ripple_mrad')}  {d['hold_flips']['A']:4d} -> {d['hold_flips']['B']:4d}")
        return 0
    if not args.csv:
        ap.error("give run CSVs, or --compare A.csv B.csv")
    model = None
    if args.gravity_model:
        from crackvision import gravity_model as gmod
        model = gmod.load_model(args.gravity_model, urdf=gmod.DEFAULT_URDF)
    result = {}
    for p in args.csv:
        rows = load(p)
        tseg, tsrc, kinds = trajectory_segment(p, rows) if rows else (1, "no rows", None)
        segs = sorted({int(x["seg"]) for x in rows})
        if window:
            rows = seg1_window(rows, window, p)
        if len(rows) < 500:
            print(f"{p}: only {len(rows)} path rows (an aborted or refused run?): skipped; {outcome_text(p)}")
            continue
        result[p.name] = {"joints": analyse(rows), "outcome": outcome_text(p), "summary_missing": not run_summary(p),
                          "trajectory_seg": tseg, "trajectory_seg_from": tsrc, "segment_kinds": kinds,
                          "segment_labels": seg_labels(kinds, segs)} \
            | ({"directions": directions(rows)} if args.directions else {}) \
            | ({"gravity_model": rescore(rows, model)} if model is not None else {})
        if args.friction_fit:
            kp, fric, src = run_params(p)
            result[p.name] |= {"params_from": src, "friction_fit": friction_fit(rows, kp, fric, wif, wik)}
    summary = friction_summary(result) if args.friction_fit and len(result) > 1 else None
    if args.json:
        print(json.dumps(result | ({"across_runs_trajectory_segment": summary} if summary else {}), indent=1))
        return 0
    for name, r in result.items():
        print(f"== {name}" + ("   [NO RUN SUMMARY: slow / kp / Coulomb / trajectory segment ASSUMED]"
                               if r["summary_missing"] else ""))
        print(f"  {r['outcome']}; trajectory = segment {r['trajectory_seg']} ({r['trajectory_seg_from']})"
              + (f"; window {window[0]:g}..{window[1]:g} s of it" if window else ""))
        print(f"  joint  mean   rms   max  ripple@Hz     tq-ff   peak tq  peak ff   rms by segment ({r['segment_labels']})")
        for j, v in r["joints"].items():
            print(f"  {j}  {v['mean_err_mrad']:+6.1f} {v['rms_err_mrad']:5.1f} {v['max_err_mrad']:5.1f}  "
                  f"{v['ripple_mrad']:4.2f}@{v['ripple_hz']:4.1f}  {v['torque_minus_ff_mean_nm']:+6.2f} Nm  {v['peak_torque_nm']:5.1f}  "
                  f"{v['peak_ff_nm']:5.1f}   {v['by_segment_rms_mrad']}")
        if "gravity_model" in r:
            print(f"  re-scored against {args.gravity_model}: torque - ff (what the P/D terms supply), as logged -> "
                  "with the model; mean / rms, Nm [mrad]")
            for j, v in r["gravity_model"].items():
                print(f"  {j}  {v['logged_mean_nm']:+5.2f} / {v['logged_rms_nm']:4.2f} [{v['logged_mean_mrad']:+5.1f} / "
                      f"{v['logged_rms_mrad']:4.1f}]  ->  {v['model_mean_nm']:+5.2f} / {v['model_rms_nm']:4.2f} "
                      f"[{v['model_mean_mrad']:+5.1f} / {v['model_rms_mrad']:4.1f}]   by segment {v['model_by_segment_mean_mrad']}"
                      + (f"  (trimmed at the cap {v['trimmed_s']} s)" if v["trimmed_s"] else "")
                      + (f"  REFUSED {v['refused_s']} s: gravity over the cap" if v["refused_s"] else ""))
        for j, d in r.get("directions", {}).items():
            print(f"  {j} stationary after moving " + ", ".join(
                f"{k}: tq-ff {v['torque_minus_ff_nm']:+.2f} Nm, err {v['static_err_mrad']:+.1f} mrad (n={v['n']})" for k, v in d.items()))
        if "friction_fit" in r:
            print_friction_fit(name, r["params_from"], r["friction_fit"], r["trajectory_seg"], r["segment_labels"])
    if summary:
        print(f"== across {len(result)} runs, the trajectory segment of each: implied Coulomb friction (runner feed-forward "
              "+ sign coefficient of the torq - ff fit), Nm")
        for j, v in summary.items():
            print(f"  {j}  mean {v['implied_coulomb_mean_nm']:5.3f}  sd {v['implied_coulomb_sd_nm']:5.3f}  range "
                  f"{v['implied_coulomb_min_nm']:5.3f}..{v['implied_coulomb_max_nm']:5.3f}  (n={v['n_runs']}; mean sign "
                  f"term of the error {v['sgn_mean_mrad']:+.1f} mrad)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""tools/fit_gravity_model.py — fit a physical, pose-dependent gravity model of the B601-DM from logged runs (offline).

The runner's feed-forward is URDF gravity x a per-joint scale (J2 0.6, J3 2.5, J4 2.5, identified near the fold).
This tool replaces the scales with what they stand in for: extra point masses on link3, link4, link5, link6 and the
gripper, plus the D405 + mount at its CAD pose (end_effector.yaml), fitted by regularised linear least squares on the
motor torques of past runs (docs/motion/GRAVITY_MODEL_PLAN.md, docs/motion/GRAVITY_MODEL.md). Gravity is linear in
each body's mass m and first moment m*r through the URDF kinematics (crackvision.gravity_model), so the model
extrapolates by geometry rather than by curve shape.

Data (demo_move.py run logs, ~/rebot_setup/runs/*.csv[.gz]):
  holds   stationary windows: |v| < 0.01 rad/s (measured, 50 ms average) and |v_ref| < 0.005 on J2-J4 for >= 0.2 s,
          > 1 s after enable (the fade-in), not in a guard hold, not near the fold (the forearm rests on the J2 motor
          there; J3 > -0.25 rad is excluded). Each window = one observation per joint (median torque). At standstill
          the torque is G +- static friction, so each window is tagged with the direction of the joint's last move;
          windows at the same pose reached from opposite directions are paired (mean = gravity, half difference =
          friction); the rest are moved back by the band the pairs measured (by their approach direction) and enter
          through a Huber loss (1.5 noise levels; per-joint noise J2/J3 0.4, J4 0.15, J5/J6 0.1 Nm).
  moving  slow path samples (|a_ref| < 0.5 rad/s^2, |v_ref| > 0.03 rad/s on that joint, 0.3 s averages,
          20 Hz) with a Coulomb + viscous friction term per joint fitted alongside.
Unknowns: per extra body m and m*r (4), the camera's mass (1); ridge toward zero extra mass (camera toward its 0.1 kg
estimate). The design is rank-deficient (only combinations of the bodies are seen); the rank, the condition number of
the identifiable part and each parameter's posterior/prior std are reported; parameters the data says nothing about
(posterior std >= 0.95 prior std) are fixed to the prior, the rest are "identified individually" (< 0.5) or "only
in combination". Slow-moving samples (--moving, default auto = whichever leave-one-run-out prefers) add a Coulomb +
viscous term per joint.
Validation: leave-one-run-out over the show runs, per joint residual mean/rms/max in Nm and in mrad at the tuned kp,
against URDF x1, the runner's scales, and the scales + the 2026-10-25 --ff-bias. Then the predicted feed-forward
along the crack trajectories against the caps.

  ./env.sh python tools/fit_gravity_model.py                       # fit on the default runs, print the report
  ./env.sh python tools/fit_gravity_model.py --write config/robot/gravity_model.yaml --plots docs/motion/figures
  ./env.sh python tools/fit_gravity_model.py --runs A.csv B.csv ... --show-runs A.csv ...   (explicit run sets)
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from crackvision import gravity_model as gmod
from crackvision.gravity_model import ARM, KP, FF_CAP, UrdfGravity

RUNS_DIR = Path.home() / "rebot_setup" / "runs"
SHOW_GLOBS = ["traj_figure8_202610*.csv*", "traj_heart_202610*.csv*", "traj_showreel_202610*.csv*", "traj_box_202610*.csv*"]
EXTRA_GLOBS = ["demo_show_*.csv*", "demo_wave_*.csv*", "traj_smoke_raise_*.csv*"]
CRACK_TRAJ = ["data/crack_trace/search.json", "data/crack_trace/scan.json"]   # + the newest demo_*_trace.json
FIT_JOINTS = (1, 2, 3, 4, 5)          # j2..j6 (index into ARM); J1's axis is vertical (no gravity torque)
REPORT_JOINTS = (1, 2, 3)             # j2, j3, j4: the joints the tracking analysis is about
FOLD_J3 = -0.05                       # fit: J3 above this = the forearm is on (or within 0.03 rad of) its rest
REST_J3 = -0.03                       # evaluation: J3 above this = resting on the J2 motor (it sits at -0.02 there,
                                      # pushed into the rest by the zero command): contact, not gravity
STAT_V, STAT_VREF, STAT_MIN_S = 0.01, 0.005, 0.2
ENABLE_SKIP_S = 1.0
PAIR_TOL = 0.03                       # rad: two holds are the same pose if every arm joint agrees within this
CAMERA_KG_PRIOR, CAMERA_KG_SIGMA = 0.10, 0.05
MASS_SIGMA, MOMENT_SIGMA = 0.5, 0.05  # prior std of an extra body's mass (kg) and first moment (kg m), ridge toward 0
NOISE_NM = {"j2": 0.4, "j3": 0.4, "j4": 0.15, "j5": 0.1, "j6": 0.1}   # per-joint residual level (friction, ripple)
HUBER_SD = 1.5                        # Huber threshold, in noise levels
MOVING_WEIGHT = 3.0                   # the moving rows together weigh like this many times the hold rows
UNIDENT_RATIO = 0.95                  # posterior std / prior std at or above this = the data says nothing: fixed
INDIV_RATIO = 0.5                     # below this a parameter is identified on its own; between: only in combination


@dataclass
class Body:
    name: str
    link: str
    kind: str                         # "free": m and m*r unknown; "mass": m unknown at the fixed xyz
    xyz: tuple = (0.0, 0.0, 0.0)      # "mass" bodies: CAD position in the link frame
    kg_prior: float = 0.0
    kg_sigma: float = MASS_SIGMA

    def params(self) -> list[str]:
        return [f"{self.name}.m"] if self.kind == "mass" else [f"{self.name}.{c}" for c in ("m", "mx", "my", "mz")]


def camera_xyz() -> tuple[float, float, float]:
    """Centre of the D405 housing (0.06 kg) and its stock mount (0.04 kg) in gripper_link, from end_effector.yaml."""
    from crackvision import kinematics
    import yaml
    ee = kinematics.load_end_effector()
    cfg = yaml.safe_load(kinematics.DEFAULT_END_EFFECTOR.read_text())
    boxes = {c["id"]: c for c in cfg["collision"]}
    T = np.asarray(ee.T_gripper_link_camera_link, dtype=float)
    housing = (T @ np.r_[boxes["camera_housing_link"]["xyz_m"], 1.0])[:3]
    mount = np.asarray(boxes["camera_mount_link"]["xyz_m"], dtype=float)
    c = (0.06 * housing + 0.04 * mount) / 0.10
    return tuple(round(float(v), 4) for v in c)


def default_bodies() -> list[Body]:
    return [Body("link3_extra", "link3", "free"), Body("link4_extra", "link4", "free"),
            Body("link5_extra", "link5", "free"), Body("link6_extra", "link6", "free"),
            Body("gripper_extra", "gripper_link", "free"),
            Body("camera_mount", "gripper_link", "mass", camera_xyz(), CAMERA_KG_PRIOR, CAMERA_KG_SIGMA)]


# ---- samples ------------------------------------------------------------------------------------------------------

@dataclass
class Holds:
    q: np.ndarray                     # (n, 6) median pose of each stationary window
    tau: np.ndarray                   # (n, 6) median measured torque
    direction: np.ndarray             # (n, 6) -1/+1 = the joint's last move went down/up in angle, 0 = unknown
    run: np.ndarray                   # (n,) run name
    t: np.ndarray                     # (n,) window start, s
    dur: np.ndarray                   # (n,) s
    mode: np.ndarray


@dataclass
class Moving:
    q: np.ndarray
    tau: np.ndarray
    v: np.ndarray                     # (n, 6) commanded velocity (direction of friction)
    use: np.ndarray                   # (n, 6) bool: row usable for that joint (moving clearly in one direction)
    run: np.ndarray


def _smooth(x: np.ndarray, k: int) -> np.ndarray:
    if k <= 1:
        return x
    ker = np.ones(k) / k
    return np.apply_along_axis(lambda c: np.convolve(c, ker, mode="same"), 0, x)


def stationary_windows(run: dict, joints=(1, 2, 3), v_thr: float = STAT_V, vref_thr: float = STAT_VREF,
                       min_s: float = STAT_MIN_S) -> list[tuple[int, int]]:
    t = run["t"]
    dtm = float(np.median(np.diff(t)))
    vref = np.gradient(run["ref"][:, :6], dtm, axis=0)
    vm = _smooth(run["vel"][:, :6], max(1, int(0.05 / dtm)))
    ok = (t > ENABLE_SKIP_S) & ~np.isin(run["mode"], ["guard_hold", "park"])
    for j in joints:
        ok &= (np.abs(vref[:, j]) < vref_thr) & (np.abs(vm[:, j]) < v_thr)
    out, i, n = [], 0, int(round(min_s / dtm))
    while i < len(ok):
        if ok[i]:
            j = i
            while j < len(ok) and ok[j]:
                j += 1
            if j - i >= n:
                out.append((i, j))
            i = j
        else:
            i += 1
    return out


def last_direction(ref: np.ndarray, i: int, dtm: float, thr: float = 0.02) -> np.ndarray:
    """(6,) sign of each joint's commanded velocity the last time it exceeded thr before row i (0 = never moved)."""
    v = np.gradient(ref[: i + 1, :6], dtm, axis=0) if i > 1 else np.zeros((1, 6))
    out = np.zeros(6)
    for j in range(6):
        idx = np.nonzero(np.abs(v[:, j]) > thr)[0]
        if len(idx):
            out[j] = np.sign(v[idx[-1], j])
    return out


def extract_holds(run: dict, fold_j3: float = FOLD_J3) -> Holds:
    dtm = float(np.median(np.diff(run["t"])))
    rows = []
    for i, j in stationary_windows(run):
        q = np.median(run["pos"][i:j, :6], axis=0)
        if q[2] > fold_j3:
            continue
        rows.append((q, np.median(run["torq"][i:j, :6], axis=0), last_direction(run["ref"], i, dtm),
                     run["name"], run["t"][i], (j - i) * dtm, run["mode"][i]))
    if not rows:
        z = np.zeros((0, 6))
        return Holds(z, z, z, np.array([], str), np.zeros(0), np.zeros(0), np.array([], str))
    q, tau, d, name, t, dur, mode = zip(*rows)
    return Holds(np.array(q), np.array(tau), np.array(d), np.array(name), np.array(t), np.array(dur), np.array(mode))


def extract_moving(run: dict, fold_j3: float = FOLD_J3, every_s: float = 0.05, avg_s: float = 0.3,
                   v_min: float = 0.03, a_max: float = 0.5) -> Moving:
    t = run["t"]
    dtm = float(np.median(np.diff(t)))
    k = max(1, int(round(avg_s / dtm)))
    q, tau = _smooth(run["pos"][:, :6], k), _smooth(run["torq"][:, :6], k)
    vref = np.gradient(run["ref"][:, :6], dtm, axis=0)
    aref = np.gradient(_smooth(vref, max(1, int(0.05 / dtm))), dtm, axis=0)
    vm = _smooth(run["vel"][:, :6], k)
    base = (run["mode"] == "path") & (t > ENABLE_SKIP_S + avg_s) & (q[:, 2] < fold_j3) \
        & (np.abs(aref[:, 1:4]).max(axis=1) < a_max)
    # the average must not straddle a mode change or a direction reversal: require the window to be clean
    use = base[:, None] & (np.abs(vref) > v_min) & (np.sign(vm) == np.sign(vref)) \
        & (_smooth(np.sign(vref), k) * np.sign(vref) > 0.99)
    keep = np.arange(0, len(t), max(1, int(round(every_s / dtm))))
    keep = keep[base[keep] & use[keep][:, 1:].any(axis=1)]
    return Moving(q[keep], tau[keep], vref[keep], use[keep], np.full(len(keep), run["name"]))


def pair_holds(h: Holds, tol: float = PAIR_TOL, wrist_tol: float = 0.10) -> tuple[list, list]:
    """Per joint: [(i, k)] holds at the same pose (J2-J4 within tol, J5/J6 within wrist_tol) reached from opposite
    directions (each hold used once), and the unpaired rest. Returns (pairs[j], singles[j]) for j in 0..5."""
    pairs, singles = [], []
    tols = np.array([np.inf, tol, tol, tol, wrist_tol, wrist_tol])
    for j in range(6):
        used, pj = set(), []
        for a in range(len(h.q)):
            if a in used or h.direction[a, j] == 0:
                continue
            best, bd = None, np.inf
            for b in range(len(h.q)):
                if b == a or b in used or h.direction[b, j] != -h.direction[a, j]:
                    continue
                d = np.abs(h.q[a] - h.q[b])
                if (d <= tols).all() and d[1:4].max() < bd:
                    best, bd = b, d[1:4].max()
            if best is not None:
                pj.append((a, best))
                used |= {a, best}
        pairs.append(pj)
        singles.append([a for a in range(len(h.q)) if a not in used])
    return pairs, singles


# ---- design matrix and fit ----------------------------------------------------------------------------------------

def design(ug: UrdfGravity, q: np.ndarray, bodies: list[Body]) -> np.ndarray:
    """(n, 6, P): d G / d params for the bodies (columns in Body.params order)."""
    fr = ug.fk(q)
    cols = []
    for b in bodies:
        c = ug.point_mass_columns(fr, b.link)
        cols.append(c if b.kind == "free" else (c[:, :, 0] + c[:, :, 1:] @ np.asarray(b.xyz))[:, :, None])
    return np.concatenate(cols, axis=2) if cols else np.zeros((len(q), 6, 0))


def priors(bodies: list[Body]) -> tuple[np.ndarray, np.ndarray]:
    mean, sd = [], []
    for b in bodies:
        if b.kind == "mass":
            mean.append(b.kg_prior)
            sd.append(b.kg_sigma)
        else:
            mean += [b.kg_prior, 0.0, 0.0, 0.0]
            sd += [b.kg_sigma, MOMENT_SIGMA, MOMENT_SIGMA, MOMENT_SIGMA]
    return np.array(mean), np.array(sd)


SPLIT_MIN_KG = 0.05      # bodies_to_extra: smallest mass of a split pair's moment-carrying point
SPLIT_MAX_XYZ_M = 0.5    # ... and the farthest a point is put from its link origin (the runner refuses > 1 m)


def bodies_to_extra(bodies: list[Body], theta: np.ndarray, names: list | None = None) -> list[tuple[str, tuple, float]]:
    """Fitted parameters -> point masses (link, xyz, kg) for GravityModel(extra=...). A free body with first moment
    mr and mass m is the point mass m at mr/m when that lies within SPLIT_MAX_XYZ_M of the link origin; otherwise
    (a near-massless body, |m| < 0.02 kg, or a small mass with a large moment) it is split into a pair carrying exactly
    that mass and moment: mp = max(0.05, |mr| / SPLIT_MAX_XYZ_M) kg at mr/mp (so |xyz| <= 0.5 m, inside the runner's
    1 m limit) and m - mp at the origin. (Before 2026-10-29 the pair was always 0.05 kg, which put the point up to
    |mr|/0.05 m away, past the runner's 1 m check, and carried mr x (1 + m/0.05) instead of mr.)"""
    out, k = [], 0
    for b in bodies:
        n0 = len(out)
        if b.kind == "mass":
            out.append((b.link, tuple(b.xyz), float(theta[k])))
            k += 1
            if names is not None:
                names.append(b.name)
            continue
        m, mr = float(theta[k]), np.asarray(theta[k + 1:k + 4], dtype=float)
        k += 4
        if abs(m) >= 0.02 and np.abs(mr / m).max() <= SPLIT_MAX_XYZ_M:
            out.append((b.link, tuple(float(v) for v in mr / m), m))
        elif np.linalg.norm(mr) > 1e-6:
            mp = max(SPLIT_MIN_KG, float(np.linalg.norm(mr)) / SPLIT_MAX_XYZ_M)
            out.append((b.link, tuple(float(v) for v in mr / mp), mp))
            out.append((b.link, (0.0, 0.0, 0.0), m - mp))
        elif abs(m) > 1e-6:
            out.append((b.link, (0.0, 0.0, 0.0), m))
        if names is not None:
            names += [b.name] * (len(out) - n0)
    return out


@dataclass
class Fit:
    bodies: list
    theta: np.ndarray
    names: list
    prior_mean: np.ndarray
    prior_sd: np.ndarray
    post_sd: np.ndarray
    fixed: list
    friction: dict                    # joint -> {"coulomb": Nm, "viscous": Nm s/rad} (moving fit) / band (holds)
    cond: float
    cond_ridge: float
    singular: list
    n_obs: dict
    use_moving: bool
    extra: list = field(default_factory=list)
    rank: int = 0
    cond_identifiable: float = math.inf
    ratio: dict = field(default_factory=dict)
    extra_names: list = field(default_factory=list)

    def predict(self, ug: UrdfGravity, q: np.ndarray) -> np.ndarray:
        return ug.hold_torques(q, self.extra)


def build_rows(ug: UrdfGravity, bodies: list[Body], holds: Holds, moving: Moving | None, joints=FIT_JOINTS):
    """Stacked linear system, every row divided by its joint's noise level NOISE_NM (so a J4 Nm counts more than a
    J3 Nm, as it does in mrad): (A (r, P+F), y (r,), w (r,), info). y = tau - G_urdf. F = 2 friction columns
    (Coulomb, viscous) per fitted joint, used only by the moving rows."""
    P = sum(len(b.params()) for b in bodies)
    F = 2 * len(joints)
    A, y, w = [], [], []
    n_obs = {"holds": len(holds.q), "pairs": 0, "singles": 0, "moving": 0}
    n_obs["single_correction_nm"] = {}
    if len(holds.q):
        Ah = design(ug, holds.q, bodies)
        g0 = ug.hold_torques(holds.q)
        pairs, singles = pair_holds(holds)
        for jj, j in enumerate(joints):
            sj = NOISE_NM[ARM[j]]
            # an unpaired hold sits at the edge of the static-friction band its approach direction selects: move it
            # back by the band the pairs measured (>= 3 pairs and a positive band, else left to the Huber loss)
            half = [0.5 * (holds.tau[a, j] - holds.tau[b, j]) * holds.direction[a, j] for a, b in pairs[j]]
            band = max(0.0, float(np.median(half))) if len(half) >= 3 else 0.0
            n_obs["single_correction_nm"][ARM[j]] = round(band, 3)
            for a, b in pairs[j]:
                row = np.zeros(P + F)
                row[:P] = 0.5 * (Ah[a, j] + Ah[b, j]) / sj
                A.append(row)
                y.append(0.5 * ((holds.tau[a, j] - g0[a, j]) + (holds.tau[b, j] - g0[b, j])) / sj)
                w.append(2.0)                     # a pair cancels the friction band: worth two holds
                n_obs["pairs"] += 1
            for a in singles[j]:
                row = np.zeros(P + F)
                row[:P] = Ah[a, j] / sj
                A.append(row)
                y.append((holds.tau[a, j] - holds.direction[a, j] * band - g0[a, j]) / sj)
                w.append(1.0)
                n_obs["singles"] += 1
    n_hold_rows = len(y)
    if moving is not None and len(moving.q):
        Am = design(ug, moving.q, bodies)
        g0 = ug.hold_torques(moving.q)
        n_mov = int(sum(moving.use[:, j].sum() for j in joints))
        wm = min(1.0, MOVING_WEIGHT * max(1, n_hold_rows) / max(1, n_mov))
        for jj, j in enumerate(joints):
            idx = np.nonzero(moving.use[:, j])[0]
            if not len(idx):
                continue
            sj = NOISE_NM[ARM[j]]
            rows = np.zeros((len(idx), P + F))
            rows[:, :P] = Am[idx, j]
            rows[:, P + 2 * jj] = np.sign(moving.v[idx, j])
            rows[:, P + 2 * jj + 1] = moving.v[idx, j]
            A += list(rows / sj)
            y += list((moving.tau[idx, j] - g0[idx, j]) / sj)
            w += [wm] * len(idx)
            n_obs["moving"] += len(idx)
    return np.array(A).reshape(-1, P + F), np.array(y), np.array(w), n_obs


def solve(ug: UrdfGravity, bodies: list[Body], holds: Holds, moving: Moving | None = None, *,
          joints=FIT_JOINTS, fix: list[str] | None = None, auto_fix: bool = True, iters: int = 50) -> Fit:
    """Ridge toward the prior (zero extra mass; the camera at its 0.1 kg estimate) + Huber loss (IRLS, delta =
    HUBER_SD noise levels: the unpaired holds sit anywhere in the static-friction band). With auto_fix, parameters the
    data says nothing about (posterior std >= UNIDENT_RATIO x prior std) are fixed exactly at the prior."""
    names = [p for b in bodies for p in b.params()]
    P = len(names)
    m0, sd0 = priors(bodies)
    A, y, w, n_obs = build_rows(ug, bodies, holds, moving, joints)
    F = A.shape[1] - P
    use_moving = moving is not None and n_obs["moving"] > 0
    fixed = list(fix or [])

    def run_once(fixed_names: list[str]):
        free = np.array([n not in fixed_names for n in names] + [use_moving] * F)
        theta = np.r_[m0, np.zeros(F)]
        prec = np.r_[1.0 / sd0**2, np.full(F, 1e-8)]
        yy = y - A[:, :P] @ m0                                               # relative to the prior
        hub = np.ones(len(y))
        Af = A[:, free]
        for _ in range(iters):
            ww = w * hub
            H = Af.T @ (Af * ww[:, None]) + np.diag(prec[free])
            dx = np.linalg.solve(H, Af.T @ (ww * yy))
            r = np.abs(yy - Af @ dx)
            new = np.where(r <= HUBER_SD, 1.0, HUBER_SD / np.maximum(r, 1e-12))
            done = np.allclose(new, hub, atol=1e-4)
            hub = new
            if done:
                break
        theta[free] += dx
        sd = np.full(P + F, np.nan)
        sd[free] = np.sqrt(np.diag(np.linalg.inv(H)))
        return theta, sd, free, hub

    theta, post, free, hub = run_once(fixed)
    ratio = post[:P] / sd0
    if auto_fix:
        newly = [n for n, r in zip(names, ratio) if np.isfinite(r) and r >= UNIDENT_RATIO and n not in fixed]
        if newly:
            fixed += newly
            theta, post, free, hub = run_once(fixed)
    # conditioning: singular values of the weighted design with the prior std as each column's unit
    Aw = A[:, :P] * np.sqrt(w)[:, None] * sd0
    s_full = np.linalg.svd(Aw, compute_uv=False)
    tol = s_full[0] * 1e-9
    rank = int((s_full > tol).sum())
    cond = float(s_full[0] / s_full[-1]) if s_full[-1] > tol else math.inf
    cond_id = float(s_full[0] / s_full[rank - 1]) if rank else math.inf
    cond_r = float(np.sqrt((s_full[0] ** 2 + 1.0) / (s_full[-1] ** 2 + 1.0)))
    friction = {}
    if use_moving:
        for jj, j in enumerate(joints):
            friction[ARM[j]] = {"coulomb_nm": round(float(theta[P + 2 * jj]), 3),
                                "viscous_nm_s_rad": round(float(theta[P + 2 * jj + 1]), 3)}
    fit = Fit(bodies, theta[:P], names, m0, sd0, post[:P], [n for n in names if n in fixed], friction, cond, cond_r,
              [round(float(v), 4) for v in s_full], n_obs, use_moving)
    fit.rank, fit.cond_identifiable = rank, cond_id
    fit.ratio = {n: round(float(r), 3) for n, r in zip(names, ratio)}
    fit.extra_names = []
    fit.extra = bodies_to_extra(bodies, fit.theta, fit.extra_names)
    return fit


def friction_band(ug: UrdfGravity, holds: Holds, extra) -> dict:
    """Per joint: static-friction half band from the paired holds (half the up/down difference), and the median
    |residual| of the unpaired holds about the model."""
    pairs, singles = pair_holds(holds)
    g = ug.hold_torques(holds.q, extra) if len(holds.q) else np.zeros((0, 6))
    out = {}
    for j in FIT_JOINTS:
        half = [0.5 * (holds.tau[a, j] - holds.tau[b, j]) * holds.direction[a, j] for a, b in pairs[j]]
        res = [holds.tau[a, j] - g[a, j] for a in singles[j]]
        out[ARM[j]] = {"n_pairs": len(half), "half_band_nm": round(float(np.median(half)), 3) if half else None,
                       "half_band_max_nm": round(float(np.max(half)), 3) if half else None,
                       "single_median_abs_resid_nm": round(float(np.median(np.abs(res))), 3) if res else None}
    return out


# ---- models to compare --------------------------------------------------------------------------------------------

def model_torques(kind: str, ug: UrdfGravity, q: np.ndarray, fit: Fit | None = None, bias_profile=None) -> np.ndarray:
    """(n, 6) feed-forward gravity torque of a model: 'urdf' (x1), 'scales' (the runner today), 'scales+bias' (+ the
    2026-10-25 --ff-bias times bias_profile), 'fit'."""
    if kind == "fit":
        return fit.predict(ug, q)
    g = ug.hold_torques(q)
    if kind == "urdf":
        return g
    g = g * np.array([gmod.RUNNER_SCALE[k] for k in ARM])
    if kind == "scales+bias":
        b = np.array([gmod.BIAS_2026_10_25.get(k, 0.0) for k in ARM])
        prof = np.ones(len(q)) if bias_profile is None else bias_profile
        g = g + prof[:, None] * b
    return g


def stats(res: np.ndarray, joints=REPORT_JOINTS) -> dict:
    out = {}
    for j in joints:
        r = res[:, j]
        if not len(r):
            out[ARM[j]] = None
            continue
        kp = KP[ARM[j]]
        out[ARM[j]] = {"n": int(len(r)), "mean_nm": round(float(r.mean()), 3), "rms_nm": round(float(np.sqrt((r**2).mean())), 3),
                       "max_nm": round(float(np.abs(r).max()), 3), "mean_mrad": round(float(r.mean() / kp * 1000), 1),
                       "rms_mrad": round(float(np.sqrt((r**2).mean()) / kp * 1000), 1),
                       "max_mrad": round(float(np.abs(r).max() / kp * 1000), 1)}
    return out


def evaluate(kind: str, ug: UrdfGravity, run: dict, holds: Holds, fit: Fit | None = None) -> dict:
    """Residuals of a model on one run: 'holds' = tau - G at its stationary windows (includes +- static friction),
    'path' = tau - (G + the runner's Coulomb ff on v_ref) over every path row = what the P and D terms had to supply,
    i.e. the sag (mrad at the tuned kp)."""
    out = {}
    if len(holds.q):
        prof = np.array([1.0 if m == "path" else 0.0 for m in holds.mode])
        out["holds"] = stats(holds.tau - model_torques(kind, ug, holds.q, fit, prof))
    p = run["mode"] == "path"
    p &= run["t"] > ENABLE_SKIP_S
    if p.any():
        q = run["pos"][p, :6]
        prof = gmod.path_bias_profile(run)[p]
        ff = model_torques(kind, ug, q, fit, prof) + gmod.runner_friction(np.gradient(run["ref"][:, :6], float(
            np.median(np.diff(run["t"]))), axis=0)[p])
        res = run["torq"][p, :6] - ff
        out["path"] = stats(res)
        off = q[:, 2] < REST_J3
        out["path_off_rest"] = stats(res[off])
        # the approach/return from the fold vs the shape itself (seg 0 / 2 vs 1)
        segs = run["seg"][p]
        out["path_by_seg"] = {int(s): stats((run["torq"][p, :6] - ff)[segs == s]) for s in np.unique(segs)}
    return out


# ---- crack trajectories -------------------------------------------------------------------------------------------

def runner_samples(doc: dict, slow: float = 1.0, dt: float = 0.02) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The run as demo_move.py --trajectory plays it: min-jerk from the fold to the first point (auto-timed at 90 %
    of the caps, >= 2 s), the points through a monotone cubic, min-jerk back to the fold. Returns (t, q, v_ref)."""
    caps = np.array([0.6, 0.6, 0.6, 1.2, 1.2, 1.2])
    ts = np.array([p["t_s"] for p in doc["points"]]) * slow
    qs = np.array([p["positions"] for p in doc["points"]])
    fold = np.zeros(6)

    def mj(qa, qb):
        d = np.abs(qb - qa)
        T = 0.0 if d.max() < 1e-4 else max(2.0, float((1.875 * d / (0.9 * caps)).max())) * slow
        if T == 0:
            return np.zeros((0, 6)), np.zeros((0, 6)), 0.0
        s = np.arange(0, T, dt) / T
        pos = qa + (qb - qa) * (10 * s**3 - 15 * s**4 + 6 * s**5)[:, None]
        vel = (qb - qa) * ((30 * s**2 - 60 * s**3 + 30 * s**4) / T)[:, None]
        return pos, vel, T

    a_q, a_v, _ = mj(fold, qs[0])
    if len(ts) > 1:
        from scipy.interpolate import PchipInterpolator
        tt = np.arange(0, ts[-1], dt)
        f = PchipInterpolator(ts, qs, axis=0)
        m_q, m_v = f(tt), f.derivative()(tt)
    else:
        m_q, m_v = qs, np.zeros_like(qs)
    r_q, r_v, _ = mj(qs[-1], fold)
    q = np.vstack([a_q, m_q, r_q])
    v = np.vstack([a_v, m_v, r_v])
    return np.arange(len(q)) * dt, q, v


def crack_ff_report(ug: UrdfGravity, fit: Fit, paths: list[Path]) -> dict:
    """Feed-forward per joint along each crack trajectory as the runner would play it (fold -> approach -> points ->
    return), against the caps (10 Nm J1-J3, 2.5 Nm J4-J6 = the runner's and arm_guard's tau_ff_max):
      fit     the fitted model with demo_move.py --gravity-model's rule (Coulomb term reduced to stay inside the cap;
              gravity alone over the cap = refused)
      scales  the runner today (URDF x 0.6/2.5/2.5 + Coulomb; over the cap = abort)
    plus the sag today's feed-forward would leave if the fitted model is right ((G_fit - G_scales) / kp)."""
    out = {}
    kp = np.array([KP[k] for k in ARM])
    for p in paths:
        doc = json.loads(Path(p).read_text())
        t, q, v = runner_samples(doc)
        fr = gmod.runner_friction(v)
        rep = {"duration_s": round(float(t[-1]), 1),
               "joint_range": {ARM[j]: [round(float(q[:, j].min()), 3), round(float(q[:, j].max()), 3)] for j in range(6)}}
        g_fit = model_torques("fit", ug, q, fit)
        ff, limited, refused = gmod.model_feed_forward(g_fit, fr)
        g_sc = model_torques("scales", ug, q, fit)
        raw_sc = g_sc + fr
        rep["fit"] = {ARM[j]: {"peak_gravity_nm": round(float(np.abs(g_fit[:, j]).max()), 2),
                               "peak_gravity_plus_coulomb_nm": round(float(np.abs(g_fit[:, j] + fr[:, j]).max()), 2),
                               "peak_ff_nm": round(float(np.abs(ff[:, j]).max()), 2), "cap_nm": FF_CAP[ARM[j]],
                               "margin_nm": round(float(FF_CAP[ARM[j]] - np.abs(ff[:, j]).max()), 2),
                               "friction_limited_s": round(float(limited[:, j].sum() * (t[1] - t[0])), 2),
                               "where_peak_gravity": {ARM[i]: round(float(q[int(np.abs(g_fit[:, j]).argmax()), i]), 3)
                                                      for i in (1, 2, 3)}}
                      for j in range(6)}
        rep["fit"]["refused"] = [ARM[j] for j in range(6) if refused[:, j].any()]
        rep["fit"]["friction_limited"] = [ARM[j] for j in range(6) if limited[:, j].any()]
        rep["scales"] = {ARM[j]: {"peak_gravity_nm": round(float(np.abs(g_sc[:, j]).max()), 2),
                                  "peak_ff_nm": round(float(np.abs(raw_sc[:, j]).max()), 2), "cap_nm": FF_CAP[ARM[j]],
                                  "margin_nm": round(float(FF_CAP[ARM[j]] - np.abs(raw_sc[:, j]).max()), 2)}
                         for j in range(6)}
        rep["scales"]["refused"] = [ARM[j] for j in range(6) if np.abs(raw_sc[:, j]).max() > FF_CAP[ARM[j]]]
        sag = (g_fit - g_sc) / kp * 1000
        rep["sag_with_scales_if_fit_is_right_mrad"] = {ARM[j]: {"max_abs": round(float(np.abs(sag[:, j]).max()), 1),
                                                               "mean": round(float(sag[:, j].mean()), 1)}
                                                      for j in (1, 2, 3)}
        out[str(p)] = rep
    return out


# ---- the whole pipeline -------------------------------------------------------------------------------------------

def default_runs() -> tuple[list[Path], list[Path]]:
    show = sorted({Path(p) for g in SHOW_GLOBS for p in glob.glob(str(RUNS_DIR / g))})
    extra = sorted({Path(p) for g in EXTRA_GLOBS for p in glob.glob(str(RUNS_DIR / g))})
    # one copy per run (a .csv and its .csv.gz)
    def dedup(ps):
        seen, out = set(), []
        for p in ps:
            k = p.name.replace(".gz", "")
            if k not in seen:
                seen.add(k)
                out.append(p)
        return out
    return dedup(show), dedup(extra)


def newest_demo_trace(root: Path) -> Path | None:
    c = sorted((root / "data" / "crack_trace").glob("demo_*_trace.json"))
    return c[-1] if c else None


def load_all(paths: list[Path], log=print) -> dict:
    out = {}
    for p in paths:
        try:
            run = gmod.load_run(p)
        except (ValueError, OSError, KeyError) as e:
            log(f"  skipped {p.name}: {e}")
            continue
        if len(run["t"]) < 500:
            log(f"  skipped {p.name}: only {len(run['t'])} rows")
            continue
        out[run["name"]] = {"run": run, "holds": extract_holds(run), "moving": extract_moving(run)}
    return out


def concat_holds(hs: list[Holds]) -> Holds:
    hs = [h for h in hs if len(h.q)]
    if not hs:
        z = np.zeros((0, 6))
        return Holds(z, z, z, np.array([], str), np.zeros(0), np.zeros(0), np.array([], str))
    return Holds(*[np.concatenate([getattr(h, f) for h in hs]) for f in ("q", "tau", "direction", "run", "t", "dur", "mode")])


def concat_moving(ms: list[Moving]) -> Moving | None:
    ms = [m for m in ms if len(m.q)]
    if not ms:
        return None
    return Moving(*[np.concatenate([getattr(m, f) for m in ms]) for f in ("q", "tau", "v", "use", "run")])


def fit_on(data: dict, names: list[str], ug: UrdfGravity, bodies: list[Body], use_moving: bool) -> Fit:
    holds = concat_holds([data[n]["holds"] for n in names])
    moving = concat_moving([data[n]["moving"] for n in names]) if use_moving else None
    return solve(ug, bodies, holds, moving)


def loro(data: dict, show: list[str], ug: UrdfGravity, bodies: list[Body], use_moving: bool) -> dict:
    out = {}
    for held in show:
        fit = fit_on(data, [n for n in data if n != held], ug, bodies, use_moving)
        d = data[held]
        out[held] = {k: evaluate(k, ug, d["run"], d["holds"], fit) for k in ("urdf", "scales", "scales+bias", "fit")}
    return out


def valid_range(data: dict, names: list[str]) -> dict:
    hs = concat_holds([data[n]["holds"] for n in names])
    ms = concat_moving([data[n]["moving"] for n in names])
    q = np.vstack([hs.q] + ([ms.q] if ms is not None else []))
    return {ARM[j]: [round(float(q[:, j].min()), 3), round(float(q[:, j].max()), 3)] for j in range(1, 6)}


def plot_residuals(ug: UrdfGravity, data: dict, names: list[str], fit: Fit, out: Path) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    hs = concat_holds([data[n]["holds"] for n in names])
    ms = concat_moving([data[n]["moving"] for n in names])
    fig, axes = plt.subplots(3, 3, figsize=(13, 9), sharey="row")
    for r, j in enumerate(REPORT_JOINTS):
        for c, (jx, lab) in enumerate(((1, "q2"), (2, "q3"), (3, "q4"))):
            ax = axes[r, c]
            for kind, col in (("scales", "#d95f02"), ("fit", "#1b9e77")):
                if ms is not None:
                    u = ms.use[:, j]
                    fr = np.array([fit.friction.get(ARM[j], {}).get("coulomb_nm", 0.0)]) * np.sign(ms.v[u, j]) \
                        if fit.friction else 0.0
                    res = ms.tau[u, j] - model_torques(kind, ug, ms.q[u], fit)[:, j] - fr
                    ax.scatter(ms.q[u, jx], res, s=2, alpha=0.15, color=col)
                res = hs.tau[:, j] - model_torques(kind, ug, hs.q, fit)[:, j]
                ax.scatter(hs.q[:, jx], res, s=18, color=col, edgecolor="k", linewidth=0.3,
                           label=f"{kind} (holds)" if r == 0 and c == 0 else None)
            ax.axhline(0, color="k", lw=0.5)
            ax.set_xlabel(f"{lab} (rad)")
            if c == 0:
                ax.set_ylabel(f"{ARM[j]} torque - model (Nm)")
    axes[0, 0].legend(loc="best", fontsize=8)
    fig.suptitle("Gravity residual vs pose: runner scales (orange) vs fitted model (green); dots = moving samples "
                 "(friction removed), circles = stationary holds")
    fig.tight_layout()
    out.mkdir(parents=True, exist_ok=True)
    path = out / "gravity_model_residuals.png"
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def model_doc(fit: Fit, ug: UrdfGravity, data: dict, train: list[str], show: list[str], residuals: dict,
              crack: dict, vrange: dict, band: dict, fold_j3: float) -> dict:
    """The crackvision.gravity_model/1 document."""
    covered = ["j2", "j3", "j4", "j5", "j6"]
    crack_ranges = {}
    for rep in crack.values():
        for k, (lo, hi) in rep["joint_range"].items():
            a, b = crack_ranges.get(k, (math.inf, -math.inf))
            crack_ranges[k] = (min(a, lo), max(b, hi))
    outside = {k: [round(lo, 3), round(hi, 3)] for k, (lo, hi) in crack_ranges.items()
               if k in vrange and (lo < vrange[k][0] - 0.05 or hi > vrange[k][1] + 0.05)}
    bodies = []
    for (link, xyz, kg), name in zip(fit.extra, fit.extra_names):
        bodies.append({"link": link, "xyz": [round(float(v), 5) for v in xyz], "kg": round(float(kg), 4),
                       "from": name})
    params = {n: {"value": round(float(v), 5), "prior": round(float(m), 4), "prior_sd": round(float(s), 4),
                  "posterior_sd": None if not np.isfinite(p) else round(float(p), 5),
                  "identifiability": ("none: fixed to the prior" if n in fit.fixed else
                                      "individually" if fit.ratio.get(n, 1.0) < INDIV_RATIO else "only in combination")}
              for n, v, m, s, p in zip(fit.names, fit.theta, fit.prior_mean, fit.prior_sd, fit.post_sd)}
    return {
        "schema": gmod.SCHEMA,
        "created_utc": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "producer": "tools/fit_gravity_model.py",
        "urdf": {"sha256": ug.sha256, "file": ug.path.name,
                 "note": "the runner's ~/rebot_setup/model copy; demo_move.py --gravity-model refuses any other"},
        "bodies": bodies,
        "scale": {k: 1.0 for k in covered},
        "scale_note": "per-joint gravity scale the runner uses WITH this model (replaces 0.6/2.5/2.5); j1 is vertical",
        "parameters": params,
        "fit": {"date": dt.date.today().isoformat(), "runs": train, "show_runs_loro": show,
                "use_moving_samples": fit.use_moving, "n_obs": fit.n_obs, "fold_excluded_j3_above_rad": fold_j3,
                "ridge": {"mass_sd_kg": MASS_SIGMA, "moment_sd_kg_m": MOMENT_SIGMA, "camera_kg": [CAMERA_KG_PRIOR, CAMERA_KG_SIGMA],
                          "noise_nm": NOISE_NM},
                "condition_number": {"scaled_design": round(fit.cond, 1) if np.isfinite(fit.cond) else "inf",
                                     "rank": fit.rank, "of": len(fit.names),
                                     "identifiable_subspace": round(fit.cond_identifiable, 1),
                                     "fitted_with_ridge": round(fit.cond_ridge, 1)},
                "fixed_to_prior": fit.fixed, "friction_fitted": fit.friction, "static_friction_band": band},
        "residuals": residuals,
        "valid_range": vrange,
        "valid_range_note": "per-joint min/max of the fitted samples (holds and moving rows; a box, not their convex hull); "
                            "j1 is vertical (no gravity torque) and not listed",
        "extrapolated": bool(outside),
        "extrapolated_joints": outside,
        "extrapolation_note": ("the crack workspace (search/scan/trace) lies outside the logged poses on these joints: "
                               "the model is extrapolated there by geometry until the survey "
                               "(~/rebot_setup/trajectories/gravity_survey*.json) has been run and refitted")
        if outside else "every crack-trajectory pose lies inside the fitted range",
        "crack_feed_forward": {Path(k).name: {"refused_with_model": v["fit"]["refused"],
                                              "friction_limited_with_model": v["fit"]["friction_limited"],
                                              "peak_gravity_model_nm": {j: v["fit"][j]["peak_gravity_nm"] for j in ARM},
                                              "peak_ff_model_nm": {j: v["fit"][j]["peak_ff_nm"] for j in ARM},
                                              "peak_ff_scales_nm": {j: v["scales"][j]["peak_ff_nm"] for j in ARM},
                                              "sag_with_scales_if_model_right_max_mrad":
                                                  {j: x["max_abs"] for j, x in v["sag_with_scales_if_fit_is_right_mrad"].items()}}
                               for k, v in crack.items()},
    }


def summarise_loro(lo: dict) -> dict:
    """Per model, per joint: mean over held-out runs of |mean| and rms (path and holds), and the worst run."""
    out = {}
    for kind in ("urdf", "scales", "scales+bias", "fit"):
        out[kind] = {}
        for part in ("path", "path_off_rest", "holds"):
            out[kind][part] = {}
            for j in REPORT_JOINTS:
                vals = [r[kind][part][ARM[j]] for r in lo.values() if part in r[kind] and r[kind][part][ARM[j]]]
                if not vals:
                    continue
                out[kind][part][ARM[j]] = {
                    "mean_abs_mean_nm": round(float(np.mean([abs(v["mean_nm"]) for v in vals])), 3),
                    "worst_abs_mean_nm": round(float(np.max([abs(v["mean_nm"]) for v in vals])), 3),
                    "mean_rms_nm": round(float(np.mean([v["rms_nm"] for v in vals])), 3),
                    "worst_rms_nm": round(float(np.max([v["rms_nm"] for v in vals])), 3),
                    "mean_abs_mean_mrad": round(float(np.mean([abs(v["mean_mrad"]) for v in vals])), 1),
                    "mean_rms_mrad": round(float(np.mean([v["rms_mrad"] for v in vals])), 1),
                    "max_nm": round(float(np.max([v["max_nm"] for v in vals])), 3)}
    return out


def acceptance(lo: dict) -> dict:
    """The plan's offline acceptance on the leave-one-run-out results, per held-out show run and joint (J2-J4):
    |mean| < 0.3 Nm, rms below the scales + bias (path rows, with and without the rest pose), and on the hearts a
    predicted mean sag under 3 mrad."""
    out, ok = {}, {"mean_under_0.3": True, "rms_better_than_scales_bias": True,
                   "rms_better_than_scales_bias_off_rest": True, "heart_sag_under_3_mrad": True}
    for run, r in lo.items():
        row = {}
        for j in REPORT_JOINTS:
            k = ARM[j]
            f, b = r["fit"]["path"][k], r["scales+bias"]["path"][k]
            fo, bo = r["fit"]["path_off_rest"][k], r["scales+bias"]["path_off_rest"][k]
            c = {"fit_mean_nm": f["mean_nm"], "fit_rms_nm": f["rms_nm"], "scales_bias_rms_nm": b["rms_nm"],
                 "fit_off_rest_mean_nm": fo["mean_nm"], "fit_off_rest_rms_nm": fo["rms_nm"],
                 "scales_bias_off_rest_rms_nm": bo["rms_nm"], "fit_mean_mrad": f["mean_mrad"],
                 "mean_under_0.3": abs(f["mean_nm"]) < 0.3, "rms_better": f["rms_nm"] < b["rms_nm"],
                 "rms_better_off_rest": fo["rms_nm"] < bo["rms_nm"]}
            ok["mean_under_0.3"] &= c["mean_under_0.3"]
            ok["rms_better_than_scales_bias"] &= c["rms_better"]
            ok["rms_better_than_scales_bias_off_rest"] &= c["rms_better_off_rest"]
            if "heart" in run:
                c["heart_sag_under_3_mrad"] = abs(f["mean_mrad"]) < 3.0
                ok["heart_sag_under_3_mrad"] &= c["heart_sag_under_3_mrad"]
            row[k] = c
        out[run] = row
    return {"per_run": out, "all": ok}


def table(title: str, summary: dict, part: str) -> str:
    lines = [f"{title} ({part}): |mean| / rms / max, Nm  [mrad at kp 100/80/40]"]
    lines.append("  model         " + "".join(f"{ARM[j]:>36s}" for j in REPORT_JOINTS))
    for kind, d in summary.items():
        cells = []
        for j in REPORT_JOINTS:
            v = d.get(part, {}).get(ARM[j])
            cells.append("-" if v is None else f"{v['mean_abs_mean_nm']:.2f}/{v['mean_rms_nm']:.2f}/{v['max_nm']:.2f} "
                         f"[{v['mean_abs_mean_mrad']:.1f}/{v['mean_rms_mrad']:.1f}]")
        lines.append(f"  {kind:12s}  " + "".join(f"{c:>36s}" for c in cells))
    return "\n".join(lines)


def training_summary(data: dict, names: list[str], ug: UrdfGravity, fit: Fit) -> dict:
    per = {n: {k: evaluate(k, ug, data[n]["run"], data[n]["holds"], fit) for k in ("urdf", "scales", "scales+bias", "fit")}
           for n in names}
    return summarise_loro(per)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("--runs", nargs="*", type=Path, help="training runs besides the show runs (default: demo_show, "
                    "demo_wave, smoke_raise in ~/rebot_setup/runs)")
    ap.add_argument("--show-runs", nargs="*", type=Path, help="the show runs (leave-one-run-out; default: the 2026-10-23/25 "
                    "traj_figure8/heart/showreel/box)")
    ap.add_argument("--urdf", type=Path, default=gmod.DEFAULT_URDF)
    ap.add_argument("--moving", choices=("auto", "yes", "no"), default="auto",
                    help="use slow-moving samples (friction fitted); auto = whichever leave-one-run-out prefers")
    ap.add_argument("--write", type=Path, help="write the crackvision.gravity_model/1 yaml here")
    ap.add_argument("--report", type=Path, help="write the full JSON report here")
    ap.add_argument("--plots", type=Path, help="write gravity_model_residuals.png into this directory")
    ap.add_argument("--no-loro", action="store_true", help="skip leave-one-run-out (faster)")
    args = ap.parse_args(argv)
    root = gmod.ROOT
    ug = UrdfGravity(args.urdf)
    show_p, extra_p = default_runs()
    show_p = args.show_runs if args.show_runs is not None else show_p
    extra_p = args.runs if args.runs is not None else extra_p
    print(f"URDF {ug.path.name} sha256 {ug.sha256[:16]}...; {len(show_p)} show runs, {len(extra_p)} other runs")
    data = load_all(list(show_p) + list(extra_p))
    show = [p.name for p in show_p if p.name in data]
    names = list(data)
    for n in names:
        h, m = data[n]["holds"], data[n]["moving"]
        print(f"  {n:40s} holds {len(h.q):3d}  moving rows {len(m.q):5d}")
    bodies = default_bodies()
    variants = {"holds": False, "holds+moving": True} if args.moving == "auto" else \
        {"holds+moving" if args.moving == "yes" else "holds": args.moving == "yes"}
    report: dict = {"urdf_sha256": ug.sha256, "runs": names, "show_runs": show, "variants": {}}
    best, best_score = None, math.inf
    for vname, use_m in variants.items():
        fit = fit_on(data, names, ug, bodies, use_m)
        vrep = {"fixed_to_prior": fit.fixed, "n_obs": fit.n_obs, "cond": fit.cond, "cond_ridge": fit.cond_ridge,
                "rank": fit.rank, "cond_identifiable": fit.cond_identifiable, "post_over_prior_sd": fit.ratio,
                "params": dict(zip(fit.names, [round(float(v), 4) for v in fit.theta])),
                "post_sd": dict(zip(fit.names, [round(float(v), 4) for v in fit.post_sd])),
                "friction": fit.friction, "extra": fit.extra, "training": training_summary(data, names, ug, fit)}
        if not args.no_loro and show:
            lo = loro(data, show, ug, bodies, use_m)
            vrep["loro"] = lo
            vrep["loro_summary"] = summarise_loro(lo)
            vrep["acceptance"] = acceptance(lo)
            s = vrep["loro_summary"]["fit"]["path_off_rest"]
            score = sum(s[ARM[j]]["mean_rms_nm"] for j in REPORT_JOINTS if ARM[j] in s)
        else:
            s = vrep["training"]["fit"]["path_off_rest"]
            score = sum(s[ARM[j]]["mean_rms_nm"] for j in REPORT_JOINTS if ARM[j] in s)
        vrep["score_sum_rms_path_nm"] = round(score, 3)
        report["variants"][vname] = vrep
        print(f"\n=== variant {vname}: rank {fit.rank} of {len(fit.names)}; cond {fit.cond:.3g} (identifiable subspace "
              f"{fit.cond_identifiable:.3g}, with ridge {fit.cond_ridge:.3g}); obs {fit.n_obs}")
        print(f"  fixed to prior (data says nothing): {fit.fixed}")
        print("  post/prior sd: " + ", ".join(f"{k} {v:.2f}" for k, v in fit.ratio.items()))
        print("  bodies: " + "; ".join(f"{l} {np.round(x, 4).tolist()} {k:+.3f} kg" for l, x, k in fit.extra))
        if fit.friction:
            print("  friction (moving): " + ", ".join(f"{k} Fc {v['coulomb_nm']:+.2f} b {v['viscous_nm_s_rad']:+.2f}"
                                                     for k, v in fit.friction.items()))
        for part in ("path", "path_off_rest", "holds"):
            print(table("training (all runs)", vrep["training"], part))
        if "loro_summary" in vrep:
            for part in ("path", "path_off_rest", "holds"):
                print(table("LEAVE-ONE-RUN-OUT (show runs)", vrep["loro_summary"], part))
            print("  per held-out run, fit vs scales+bias (path rows): mean / rms [rms off the rest]")
            for run, row in vrep["acceptance"]["per_run"].items():
                print(f"    {run:38s} " + "  ".join(
                    f"{k} {c['fit_mean_nm']:+.2f}/{c['fit_rms_nm']:.2f} vs {c['scales_bias_rms_nm']:.2f} "
                    f"[{c['fit_off_rest_rms_nm']:.2f} vs {c['scales_bias_off_rest_rms_nm']:.2f}]" for k, c in row.items()))
            print(f"  acceptance: {vrep['acceptance']['all']}")
        if score < best_score:
            best, best_score = (vname, fit), score
    vname, fit = best
    report["chosen"] = vname
    print(f"\nchosen: {vname} (lowest summed off-rest path rms, leave-one-run-out)")
    band = friction_band(ug, concat_holds([data[n]["holds"] for n in names]), fit.extra)
    report["static_friction_band"] = band
    print("static friction from paired holds: " + ", ".join(
        f"{k} +-{v['half_band_nm']} (n={v['n_pairs']})" for k, v in band.items()))
    crack_paths = [root / p for p in CRACK_TRAJ] + ([newest_demo_trace(root)] if newest_demo_trace(root) else [])
    crack = crack_ff_report(ug, fit, [p for p in crack_paths if p and p.exists()])
    report["crack_feed_forward"] = crack
    for p, rep in crack.items():
        print(f"crack ff {Path(p).name}: " + ", ".join(
            f"{j} G {rep['fit'][j]['peak_gravity_nm']:.2f} G+Fc {rep['fit'][j]['peak_gravity_plus_coulomb_nm']:.2f} "
            f"ff {rep['fit'][j]['peak_ff_nm']:.2f}/{rep['fit'][j]['cap_nm']:g} (scales {rep['scales'][j]['peak_ff_nm']:.2f})"
            for j in ARM[1:4]) + f"; refused (fit): {rep['fit']['refused'] or 'none'}, friction-limited: "
            f"{rep['fit']['friction_limited'] or 'none'}; sag today if the fit is right (max mrad): "
            + ", ".join(f"{k} {v['max_abs']}" for k, v in rep["sag_with_scales_if_fit_is_right_mrad"].items()))
    vrange = valid_range(data, names)
    report["valid_range"] = vrange
    residuals = {"training": report["variants"][vname]["training"],
                 "leave_one_run_out": report["variants"][vname].get("loro_summary"),
                 "leave_one_run_out_acceptance": report["variants"][vname].get("acceptance"),
                 "variants_compared": {k: {"use_moving": k == "holds+moving",
                                           "loro_fit": v.get("loro_summary", {}).get("fit")}
                                       for k, v in report["variants"].items()},
                 "units": "Nm; mrad = Nm / kp (kp j2 100, j3 80, j4 40); 'path' = tau - (G + runner Coulomb ff) over "
                          "path rows (what the P/D terms supplied = the sag), 'holds' = tau - G at stationary windows "
                          "(includes +- static friction)"}
    doc = model_doc(fit, ug, data, names, show, residuals, crack, vrange, band, FOLD_J3)
    print(f"extrapolated: {doc['extrapolated']} {doc['extrapolated_joints']}")
    if args.plots:
        print(f"plot: {plot_residuals(ug, data, names, fit, args.plots)}")
    if args.write:
        # refuse to write a file the runner would refuse (demo_move.py load_gravity_model's checks, replicated)
        bad = gmod.runner_problems(json.loads(json.dumps(doc, default=float)))
        if bad:
            print(f"NOT writing {args.write}: the runner's load_gravity_model would refuse this model:\n  "
                  + "\n  ".join(bad))
            return 1
        args.write.parent.mkdir(parents=True, exist_ok=True)
        header = ("config/robot/gravity_model.yaml - fitted extra point masses for the B601-DM gravity feed-forward\n"
                  "(crackvision.gravity_model/1, written by tools/fit_gravity_model.py; docs/motion/GRAVITY_MODEL.md).\n"
                  "YAML in JSON style: the runner's environment has no PyYAML and reads it with json. Do not hand-edit:\n"
                  "demo_move.py logs this file's sha256, refit instead.\n"
                  "Used only when the operator passes ~/rebot_setup/demo_move.py --gravity-model THIS_FILE (per run).\n"
                  "The bodies are equivalent point masses: only each body's mass and first moment are meaningful.\n"
                  "EXTRAPOLATED in the crack workspace until the gravity survey has been run and refitted.")
        args.write.write_text(gmod.dump_model_text(json.loads(json.dumps(doc, default=float)), header))
        print(f"wrote {args.write}")
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
        print(f"wrote {args.report}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

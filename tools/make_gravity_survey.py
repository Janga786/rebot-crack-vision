"""tools/make_gravity_survey.py — the gravity survey: hold poses across the crack workspace, each reached from both sides.

The logged runs never put J2 below -1.0 or J4 above +0.61, but the crack pipeline works at J2 -1.13..-1.89,
J3 -1.04..-1.86, J4 +1.12..+1.36 (docs/motion/GRAVITY_MODEL_PLAN.md), so the fitted gravity model
(config/robot/gravity_model.yaml) is extrapolated there. This tool writes the trajectory that measures it: about 20
poses on a grid over J2 -0.6..-1.9, J3 -0.6..-1.9, J4 0..1.4 (J1, J5, J6 at 0), each visited twice, arriving once
from below and once from above in joint space (J2-J4 together, +-0.06 rad), held 3 s each time, so that every pose
gives an up/down pair (mean = gravity, half difference = static friction). Min-jerk moves at <= 60 % of the runner's
planned-speed caps (0.6 rad/s J1-J3, 1.2 rad/s J4-J6). Every pose and every sample of every move is checked:
the arm_guard joint windows with 0.05 rad margin, gripper_link >= 13.5 cm above the table, the camera housing and mount
>= 3 cm (crackvision.arm_ik.min_body_z), and the guard's other keep-outs (link4 >= 6 cm, link5/link6 >= 8 cm, the base
column, 6 cm from the upper arm) with 1 cm margin; a move that fails is routed through a logged, safe pose. The file
starts and ends at the fold, so the runner's own approach and return are empty. If the whole survey is longer than
--max-minutes at x1 it is split into two files, the poses nearer the logged data first.

  ./env.sh python tools/make_gravity_survey.py                            # -> ~/rebot_setup/trajectories/
  ./env.sh python tools/make_gravity_survey.py --out-dir DIR --n 20 --max-minutes 4
  then (operator, simulation): ~/rebot_setup/run-trajectory.sh --rehearse ~/rebot_setup/trajectories/gravity_survey_a.json 2
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from crackvision import gravity_model as gmod
from crackvision.arm_ik import ArmIK, IKError, check_motion_clearance, min_body_z, body_boxes, peak_speed_fraction, \
    trajectory_doc, RUNNER_SPEED_CAP_RAD_S

ROOT = gmod.ROOT
GUARD_TOML = ROOT / "config" / "robot" / "arm_guard.toml"
OUT_DIR = Path.home() / "rebot_setup" / "trajectories"
SPEED_FRACTION = 0.6
LIMIT_MARGIN = 0.05
GRIPPER_Z_MIN = 0.135
BODY_Z_MIN = 0.03
KEEPOUT_MARGIN = 0.01
DELTA = 0.06                         # rad: the approach offset on J2-J4 (moves enough to settle at a friction-band edge)
HOLD_S = 3.0
DT = 0.05
SHORT_MIN_S, TRANSIT_MIN_S = 1.0, 1.5
HUB = np.array([0.0, -0.9, -1.2, 0.35, 0.0, 0.0])   # the heart's start pose: logged 7 times, clear of everything
SIGN = np.array([0.0, 1.0, 1.0, 1.0, 0.0, 0.0])     # which joints flip their approach direction between the two visits
DATA_CENTRE = np.array([0.0, -0.85, -1.2, 0.3, 0.0, 0.0])
SAG_MAX_MRAD = 60.0                  # predicted hold error with today's feed-forward if the model is right (abort: 100)
SAG_CLEARANCE_FACTOR = 1.5           # each pose must also stay clear when sagged by this x the predicted error
MODEL_CAP_MARGIN = 0.5               # Nm: the model's gravity at a pose stays this far inside the feed-forward cap


def guard_limits(path: Path = GUARD_TOML) -> tuple[np.ndarray, np.ndarray, dict]:
    cfg = tomllib.loads(path.read_text())
    lo = np.array([cfg["joints"][f"j{i}"]["lower"] for i in range(1, 7)], dtype=float)
    hi = np.array([cfg["joints"][f"j{i}"]["upper"] for i in range(1, 7)], dtype=float)
    return lo, hi, cfg.get("keepout", {})


class Checker:
    """Pose checks against the guard limits (with margin), the table and the guard's keep-outs."""

    def __init__(self, ik: ArmIK | None = None, ug: gmod.UrdfGravity | None = None) -> None:
        self.ik = ik or ArmIK()
        self.ug = ug or gmod.UrdfGravity()
        self.lo, self.hi, self.keepout = guard_limits()
        self.boxes = body_boxes(self.ik)

    def problems(self, q: np.ndarray, margin: float = LIMIT_MARGIN) -> list[str]:
        """Why q is not acceptable ([] = fine). `margin` inside the guard's joint windows: LIMIT_MARGIN for the held
        poses; the moves (which must leave the fold, at J2 = J3 = 0 = the window's edge) are checked with 0."""
        q = np.asarray(q, dtype=float)
        out = []
        bad = (q < self.lo + margin - 1e-9) | (q > self.hi - margin + 1e-9)
        # the fold itself (all zero) is where every run starts and ends: allowed at the limit
        if bad.any() and np.abs(q).max() > 1e-9:
            out.append("joint window: " + ", ".join(f"j{i + 1} {q[i]:+.3f}" for i in np.nonzero(bad)[0]))
        fr = self.ug.fk(q[None])
        pos = {k: v[0] for k, v in fr.t.items()}
        if pos["gripper_link"][2] < GRIPPER_Z_MIN - 1e-9 and np.abs(q).max() > 1e-9:
            out.append(f"gripper_link z {pos['gripper_link'][2]:.3f} < {GRIPPER_Z_MIN}")
        z, box = min_body_z(self.ik, q, self.boxes)
        if z < BODY_Z_MIN:
            out.append(f"{box} z {z:.3f} < {BODY_Z_MIN}")
        for fl in self.keepout.get("floor", []):
            if fl["link"] != "gripper_link" and pos[fl["link"]][2] < fl["z_min"] + KEEPOUT_MARGIN:
                out.append(f"{fl['link']} z {pos[fl['link']][2]:.3f} < {fl['z_min']} + {KEEPOUT_MARGIN}")
        cyl = self.keepout.get("base_cylinder")
        if cyl:
            for ln in cyl["links"]:
                p = pos[ln]
                if math.hypot(p[0], p[1]) < cyl["radius"] + KEEPOUT_MARGIN and p[2] < cyl["z_max"] + KEEPOUT_MARGIN:
                    out.append(f"{ln} inside the base column")
        seg = self.keepout.get("segment")
        if seg:
            a, b = pos[seg["from_link"]], pos[seg["to_link"]]
            for ln in seg["links"]:
                p = pos[ln]
                u = np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-12), 0.0, 1.0)
                dist = float(np.linalg.norm(p - (a + u * (b - a))))
                if dist < seg["min_distance"] + KEEPOUT_MARGIN:
                    out.append(f"{ln} {dist:.3f} m from the upper arm")
        return out


def min_jerk(qa: np.ndarray, qb: np.ndarray, t_min: float) -> tuple[np.ndarray, float]:
    """Samples (DT apart, both ends included) of a min-jerk move timed so every joint peaks at <= SPEED_FRACTION of
    its runner cap (peak speed of a min-jerk move = 1.875 x distance / T)."""
    d = np.abs(qb - qa)
    T = max(t_min, float((1.875 * d / (SPEED_FRACTION * RUNNER_SPEED_CAP_RAD_S)).max()))
    T = math.ceil(T / DT) * DT
    s = np.linspace(0.0, 1.0, int(round(T / DT)) + 1)
    return qa + (qb - qa) * (10 * s**3 - 15 * s**4 + 6 * s**5)[:, None], T


@dataclass
class Piece:
    q: np.ndarray        # samples, first = the previous piece's last
    kind: str
    pose: int | None = None
    approach: int = 0    # +1 = the pose was reached moving up in angle (J2-J4), -1 = down


def move_ok(chk: Checker, qs: np.ndarray) -> list[str]:
    """[] if every sample of a move is clear (joint windows without margin), else the first problem."""
    for q in qs:
        p = chk.problems(q, margin=0.0)
        if p:
            return p
    return []


def route(chk: Checker, qa: np.ndarray, qb: np.ndarray, t_min: float) -> list[np.ndarray]:
    """A clear min-jerk move qa -> qb, or qa -> HUB -> qb; [] if neither is clear."""
    qs, _ = min_jerk(qa, qb, t_min)
    if not move_ok(chk, qs):
        return [qs]
    if not np.allclose(qa, HUB) and not np.allclose(qb, HUB):
        a, _ = min_jerk(qa, HUB, t_min)
        b, _ = min_jerk(HUB, qb, t_min)
        if not move_ok(chk, a) and not move_ok(chk, b):
            return [a, b]
    return []


def candidate_grid(n2: int = 7, n3: int = 7, n4: int = 4) -> np.ndarray:
    g = [np.array([0.0, a, b, c, 0.0, 0.0]) for a in np.linspace(-0.6, -1.9, n2) for b in np.linspace(-0.6, -1.9, n3)
         for c in np.linspace(0.0, 1.4, n4)]
    return np.array(g)


class Risk:
    """What the fitted model predicts at a pose: the hold error today's feed-forward (URDF x 0.6/2.5/2.5) would leave
    if the model is right, and whether the model's own gravity fits inside the caps (so the survey could also be
    run with --gravity-model). Without a model file nothing is predicted and nothing is filtered."""

    def __init__(self, model_path: Path | None) -> None:
        self.ug = gmod.UrdfGravity()
        self.extra = None
        if model_path is not None and Path(model_path).exists():
            self.extra = gmod.extra_bodies(gmod.load_model(model_path, urdf=self.ug.path))
        self.kp = np.array([gmod.KP[k] for k in gmod.ARM])
        self.cap = np.array([gmod.FF_CAP[k] for k in gmod.ARM])
        self.scale = np.array([gmod.RUNNER_SCALE[k] for k in gmod.ARM])

    def sag(self, q: np.ndarray) -> np.ndarray:
        """(6,) rad: commanded minus reached angle at a hold (0 without a model)."""
        if self.extra is None:
            return np.zeros(6)
        g_m = self.ug.hold_torques(q[None], self.extra)[0]
        g_s = self.ug.hold_torques(q[None])[0] * self.scale
        return (g_m - g_s) / self.kp

    def problems(self, chk: Checker, q: np.ndarray) -> list[str]:
        if self.extra is None:
            return []
        out = []
        e = self.sag(q)
        if np.abs(e[1:4]).max() * 1000 > SAG_MAX_MRAD:
            out.append(f"predicted hold error today {np.round(e[1:4] * 1000, 1).tolist()} mrad > {SAG_MAX_MRAD:g}")
        g_m = self.ug.hold_torques(q[None], self.extra)[0]
        if (np.abs(g_m) > self.cap - MODEL_CAP_MARGIN).any():
            out.append(f"model gravity {np.round(g_m[1:4], 2).tolist()} Nm within {MODEL_CAP_MARGIN} of the caps")
        sagged = chk.problems(q - SAG_CLEARANCE_FACTOR * e, margin=0.0)
        if sagged:
            out.append(f"sagged by {SAG_CLEARANCE_FACTOR:g}x the predicted error: {sagged[0]}")
        return out


def select_poses(chk: Checker, n: int, anchors: list[np.ndarray], risk: "Risk | None" = None) -> tuple[np.ndarray, list]:
    """Feasible grid poses (with their +-DELTA approach points; within the predicted-sag and cap bounds when a model
    is given), the nearest to each anchor first, then farthest-point sampling in J2-J4 for coverage."""
    rejected, ok = [], []
    for q in candidate_grid():
        why = chk.problems(q) or chk.problems(q - DELTA * SIGN) or chk.problems(q + DELTA * SIGN)
        if not why and risk is not None:
            why = risk.problems(chk, q) or risk.problems(chk, q - DELTA * SIGN) or risk.problems(chk, q + DELTA * SIGN)
        (rejected if why else ok).append((q, why))
    feas = np.array([q for q, _ in ok])
    chosen: list[int] = []
    for a in anchors:
        k = int(np.argmin(np.linalg.norm(feas[:, 1:4] - a[1:4], axis=1)))
        if k not in chosen:
            chosen.append(k)
    if not chosen and len(feas):
        chosen.append(int(np.argmin(np.linalg.norm(feas[:, 1:4] - DATA_CENTRE[1:4], axis=1))))
    while len(chosen) < min(n, len(feas)):
        d = np.min(np.linalg.norm(feas[:, None, 1:4] - feas[None, chosen, 1:4], axis=2), axis=1)
        d[chosen] = -1
        chosen.append(int(np.argmax(d)))
    return feas[chosen], [(np.round(q, 3).tolist(), why) for q, why in rejected]


def tour(poses: np.ndarray, start: np.ndarray) -> list[int]:
    left, out, cur = list(range(len(poses))), [], start
    while left:
        k = min(left, key=lambda i: float(np.abs(poses[i] - cur).max()))
        out.append(k)
        left.remove(k)
        cur = poses[k]
    return out


def build_file(chk: Checker, poses: np.ndarray, ids: list[int]) -> tuple[np.ndarray, np.ndarray, list[dict], list]:
    """fold -> for each pose: below -> pose (hold) -> above -> pose (hold) -> ... -> fold. Returns (t, q, holds,
    dropped poses)."""
    fold = np.zeros(6)
    pieces: list[Piece] = []
    holds, dropped, cur = [], [], fold
    for pid in ids:
        p = poses[pid]
        below, above = p - DELTA * SIGN, p + DELTA * SIGN
        legs = route(chk, cur, below, TRANSIT_MIN_S)
        if not legs:
            dropped.append((pid, "no clear transit"))
            continue
        for leg in legs:
            pieces.append(Piece(leg, "transit"))
        for approach, frm in ((+1, below), (-1, above)):
            if approach == -1:
                pieces.append(Piece(min_jerk(p, above, SHORT_MIN_S)[0], "offset"))
            pieces.append(Piece(min_jerk(frm, p, SHORT_MIN_S)[0], "approach", pid, approach))
            pieces.append(Piece(np.vstack([p, p]), "hold", pid, approach))
        cur = p
    legs = route(chk, cur, fold, TRANSIT_MIN_S)
    if not legs:
        raise IKError("no clear return to the fold")
    for leg in legs:
        pieces.append(Piece(leg, "transit"))
    t, q = [0.0], [fold]
    for pc in pieces:
        dt = HOLD_S if pc.kind == "hold" else DT
        for k, qq in enumerate(pc.q[1:]):
            t.append(t[-1] + dt)
            q.append(qq)
        if pc.kind == "hold":
            holds.append({"pose": int(pc.pose), "approach": "from below" if pc.approach > 0 else "from above",
                          "t0_s": round(t[-2], 3), "t1_s": round(t[-1], 3), "q": np.round(pc.q[0], 4).tolist()})
    return np.array(t), np.array(q), holds, dropped


def predictions(poses: np.ndarray, model_path: Path | None) -> list[dict]:
    """Per pose: gravity from the URDF x the runner's scales (today) and from the fitted model, and the hold error today's
    feed-forward would leave if the model is right ((G_model - G_scales) / kp)."""
    ug = gmod.UrdfGravity()
    g_sc = ug.hold_torques(poses) * np.array([gmod.RUNNER_SCALE[k] for k in gmod.ARM])
    out = [{"scales_nm": np.round(g_sc[i, 1:4], 2).tolist()} for i in range(len(poses))]
    if model_path is not None and model_path.exists():
        doc = gmod.load_model(model_path, urdf=ug.path)
        g_m = ug.hold_torques(poses, gmod.extra_bodies(doc))
        kp = np.array([gmod.KP[k] for k in ("j2", "j3", "j4")])
        cap = np.array([gmod.FF_CAP[k] for k in ("j2", "j3", "j4")]) - gmod.FF_LIMIT_MARGIN
        for i in range(len(poses)):
            out[i]["model_nm"] = np.round(g_m[i, 1:4], 2).tolist()
            out[i]["sag_today_if_model_right_mrad"] = np.round((g_m[i, 1:4] - g_sc[i, 1:4]) / kp * 1000, 1).tolist()
            out[i]["model_over_cap"] = [f"j{j + 2}" for j in range(3) if abs(g_m[i, j + 1]) > cap[j]]
    return out


def crack_anchors() -> list[np.ndarray]:
    """One pose per crack trajectory (the middle of its points), so the survey covers where the crack work happens."""
    out = []
    for p in sorted((ROOT / "data" / "crack_trace").glob("*.json")):
        try:
            d = json.loads(p.read_text())
        except (OSError, ValueError):
            continue
        if d.get("schema") == "crackvision.joint_trajectory/1" and d.get("points"):
            q = np.array([pt["positions"] for pt in d["points"]])
            out.append(np.median(q, axis=0))
    return out


def make(n: int = 20, max_minutes: float = 4.0, model_path: Path | None = gmod.DEFAULT_MODEL) -> dict:
    """Returns {"files": {name: doc}, "report": {...}}."""
    chk = Checker()
    anchors = crack_anchors()
    poses, rejected = select_poses(chk, n, anchors, Risk(model_path))
    pred = predictions(poses, model_path)
    # nearer the logged data first (if anything stops the run, those holds are already logged)
    order = sorted(range(len(poses)), key=lambda i: float(np.linalg.norm(poses[i, 1:4] - DATA_CENTRE[1:4])))
    full = [order[k] for k in tour(poses[order], np.zeros(6))]
    parts = {"gravity_survey": full}
    if build_file(chk, poses, full)[0][-1] > max_minutes * 60:
        half = len(order) // 2
        a, b = order[:half], order[half:]
        parts = {"gravity_survey_a": [a[k] for k in tour(poses[a], np.zeros(6))],
                 "gravity_survey_b": [b[k] for k in tour(poses[b], np.zeros(6))]}
    files, report = {}, {"n_poses": len(poses), "rejected_grid_poses": len(rejected), "rejected": rejected,
                         "poses": [{"id": i, "q": np.round(poses[i], 3).tolist()} | pred[i] for i in range(len(poses))],
                         "files": {}}
    for name, ids in parts.items():
        t, q, holds, dropped = build_file(chk, poses, ids)
        bad = move_ok(chk, q)
        if bad:
            raise IKError(f"{name}: {bad}")
        clear = check_motion_clearance(chk.ik, q, floor_z_m=BODY_Z_MIN)
        frac = peak_speed_fraction(t, q)
        if frac.max() > SPEED_FRACTION + 1e-6:
            raise IKError(f"{name}: peak speed {frac.max():.3f} of the cap > {SPEED_FRACTION}")
        gz = min(float(chk.ug.fk(q[i:i + 1]).t["gripper_link"][0, 2]) for i in range(len(q)) if np.abs(q[i]).max() > 1e-9)
        info = {"poses": ids, "duration_s": round(float(t[-1]), 1), "n_points": int(len(t)), "holds": holds,
                "dropped": dropped, "peak_speed_fraction_of_cap": np.round(frac, 3).tolist(),
                "min_body_z_m": clear["min_body_z_m"], "min_gripper_z_m": round(gz, 4),
                "joint_ranges": [[round(float(q[:, i].min()), 3), round(float(q[:, i].max()), 3)] for i in range(6)]}
        report["files"][name] = info
        doc = trajectory_doc(
            t, q, producer="tools/make_gravity_survey.py", purpose="test",
            notes=(f"gravity survey ({name}): {len(ids)} poses over J2 -0.6..-1.9, J3 -0.6..-1.9, J4 0..1.4 (J1/J5/J6 0), "
                   f"each reached from below and from above (J2-J4 +-{DELTA} rad) and held {HOLD_S:g} s; min-jerk moves "
                   f"at <= {SPEED_FRACTION:.0%} of the runner caps; starts and ends at the fold; arm_guard windows with "
                   f"{LIMIT_MARGIN} rad margin, gripper_link >= {GRIPPER_Z_MIN} m, camera/mount >= {BODY_Z_MIN} m, guard "
                   "keep-outs + 1 cm. Refit afterwards: tools/fit_gravity_model.py --runs ... (docs/motion/GRAVITY_MODEL.md)"),
            extra={"gravity_survey": {"part": name, "delta_rad": DELTA, "hold_s": HOLD_S, "speed_fraction": SPEED_FRACTION,
                                      "holds": holds, "poses": [report["poses"][i] for i in ids]}})
        files[name] = doc
    return {"files": files, "report": report}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    ap.add_argument("--n", type=int, default=20, help="number of poses")
    ap.add_argument("--max-minutes", type=float, default=4.0, help="split into two files above this duration at x1")
    ap.add_argument("--model", type=Path, default=gmod.DEFAULT_MODEL, help="gravity model for the per-pose predictions")
    ap.add_argument("--report", type=Path, help="write the JSON report here")
    args = ap.parse_args(argv)
    try:
        res = make(args.n, args.max_minutes, args.model)
    except IKError as e:
        print(f"REFUSED: {e}", file=sys.stderr)
        return 1
    args.out_dir.mkdir(parents=True, exist_ok=True)
    rep = res["report"]
    print(f"{rep['n_poses']} poses ({rep['rejected_grid_poses']} grid poses rejected by the checks)")
    for p in rep["poses"]:
        extra = (f" model {p['model_nm']} sag today if right {p['sag_today_if_model_right_mrad']} mrad"
                 + (f" OVER CAP {p['model_over_cap']}" if p["model_over_cap"] else "")) if "model_nm" in p else ""
        print(f"  pose {p['id']:2d} q {p['q'][1:4]} scales {p['scales_nm']}{extra}")
    for name, doc in res["files"].items():
        out = args.out_dir / f"{name}.json"
        out.write_text(json.dumps(doc, indent=1) + "\n")
        i = rep["files"][name]
        print(f"{name}: {len(i['poses'])} poses, {i['duration_s']} s ({i['duration_s'] / 60:.1f} min) at x1, peak/cap "
              + " ".join(f"{f:.2f}" for f in i["peak_speed_fraction_of_cap"])
              + f", min gripper z {i['min_gripper_z_m']} m, min camera/mount z {i['min_body_z_m']} m"
              + (f", dropped {i['dropped']}" if i["dropped"] else "") + f" -> {out}")
    if args.report:
        args.report.write_text(json.dumps(rep, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

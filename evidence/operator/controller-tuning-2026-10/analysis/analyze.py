#!/usr/bin/env python3
"""Regenerate every figure, the run index and the measured numbers for the 2026-10 controller tuning.

Reads only the real logs in ../runs (and runs the offline simulator in ../code for the sim-vs-real figure).
Writes ../figures/*.png, RUNS.md and metrics.json next to this file.

    python3 analyze.py            # needs numpy + matplotlib (system python3 is fine; no motorbridge needed)
"""
from __future__ import annotations

import csv
import glob
import json
import math
import os
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
RUNS = ROOT / "runs"
FIG = ROOT / "figures"
CODE = ROOT / "code"
sys.path.insert(0, str(CODE))
import gravity  # noqa: E402

FIG.mkdir(exist_ok=True)
METRICS: dict = {}
plt.rcParams.update({"figure.dpi": 130, "savefig.bbox": "tight", "axes.grid": True, "grid.alpha": 0.3,
                     "font.size": 9})
JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"]


# ---------------------------------------------------------------------------------------------------------- loading
def load_csv(name: str) -> dict[str, np.ndarray]:
    path = RUNS / (name if name.endswith(".csv") else name + ".csv")
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    out: dict[str, np.ndarray] = {}
    for k in rows[0]:
        vals = [r[k] for r in rows]
        try:
            out[k] = np.array([float(v) if v != "" else np.nan for v in vals])
        except ValueError:
            out[k] = np.array(vals)
    return out


def load_json(name: str) -> dict:
    return json.loads((RUNS / (name + ".json")).read_text())


def save(fig, name: str, title: str) -> None:
    fig.suptitle(title, fontsize=11)
    fig.savefig(FIG / name)
    plt.close(fig)
    print("wrote", name)


# ---------------------------------------------------------------------------------------------------------- index
# Session context the logs cannot carry (written from the operator session of 2026-10-14 / 2026-10-17).
NOTES = {
    "first_move_j1_20261014T163412Z": "first powered motion of the arm; kp 10 could not break J1 static friction",
    "first_move_j3_20261014T164229Z": "refused: start-at-rest check (later removed at the operator's request)",
    "first_move_j3_20261014T164348Z": "refused: start-at-rest check (later removed at the operator's request)",
    "first_move_j3_20261014T164445Z": "aborted: unpowered J4 swung 0.106 rad as J3 lifted (drift limit then 0.10, raised to 0.30)",
    "first_move_j6_20261014T164908Z": "refused: missed register read before enable (recurring, harmless); retried",
    "first_move_j7_20261014T165052Z": "aborted: J7 velocity reading 0.549 > 0.5 rad/s (single-sample spike)",
    "first_move_j7_20261017T184226Z": "aborted: J7 velocity spike 0.505 while really moving ~0.17 rad/s -> per-joint velocity abort",
    "first_move_j7_20261017T184241Z": "re-run after the code change WITHOUT a fresh operator go (process error, disclosed in session)",
    "first_move_j3_20261017T184744Z": "--tau-ff -2.8 requested but NOT sent: motor wrapper hard-coded tau=0 (bug, fixed next)",
    "first_move_j3_20261017T184845Z": "first run with --tau-ff -2.8 actually applied (hold err 0.050 -> 0.022)",
    "first_move_j3_20261017T190219Z": "aborted: held J4 velocity 0.564 at enable (operator's hand on the arm)",
    "first_move_j3_20261017T190527Z": "first all-held + URDF gravity run; J3 needed ~-7 Nm vs model -2.8 -> identification",
    "first_move_j3_20261017T190919Z": "aborted: full x2.5 feed-forward step at enable kicked J3 (0.535 rad/s) -> fade-in added",
    "first_move_j3_20261017T191042Z": "identification run: torque/position hysteresis -> gravity -6.6 Nm, friction +-2.3 Nm",
    "first_move_j4_20261017T191809Z": "aborted: J4 velocity spike 0.549 at enable -> DM4310 velocity abort 1.0 rad/s",
    "first_move_j1_20261017T192751Z": "first multi-joint move: --also j2,j3,j4 -0.10 with tuned gains",
}


def run_index() -> list[dict]:
    rows = []
    for jf in sorted(glob.glob(str(RUNS / "*.json"))):
        name = Path(jf).stem
        d = json.loads(Path(jf).read_text())
        r = {"run": name, "date": name.split("_")[-1][:8]}
        if name.startswith("first_move"):
            p, st = d.get("params", {}) or {}, d.get("stats", {}) or {}
            r["kind"] = "single joint" + (" sweep" if d.get("sweep") else "")
            r["joint"] = name.split("_")[2]
            gains = p.get("gains") or [[p.get("kp"), p.get("kd")]]
            r["gains"] = ", ".join(f"{g[0]:g}:{g[1]:g}" for g in gains if g[0] is not None)
            qs, qt = d.get("q_start"), d.get("q_target")
            r["move"] = f"{qt - qs:+.3f}" if isinstance(qs, (int, float)) and isinstance(qt, (int, float)) else ""
            ff = []
            if p.get("tau_ff"):
                ff.append(f"tau_ff {p['tau_ff']:+g}")
            if p.get("gravity"):
                sc = p.get("gravity_joint_scale") or {}
                ff.append("grav" + ("".join(f" {k}x{v:g}" for k, v in sc.items()) or " x1"))
            if p.get("friction_nm"):
                ff.append(f"fric {p['friction_nm']:g}")
            held = "held" if "held_at" in st else ""
            r["ff"] = "; ".join(ff) or "-"
            r["others"] = held or "disabled"
            r["outcome"] = d.get("outcome", "")
            trig = d.get("trigger") or {}
            r["trigger"] = (trig.get("detail") or d.get("refusal") or "")[:70]
            r["max_err"] = st.get("max_abs_err")
            r["hold_err"] = st.get("hold_mean_err_rad")
            r["max_tq"] = st.get("max_abs_torque")
            r["rate_hz"] = st.get("mean_rate_hz")
        elif name.startswith("demo"):
            st = d.get("stats", {})
            r.update(kind="all-joint demo", joint="all", gains="tuned", move=d.get("choreo"), ff="grav x tuned; fric",
                     others="all moving", outcome=d.get("outcome"), trigger=str((d.get("trigger") or {}).get("detail", ""))[:70],
                     max_err=max(st.get("max_err", {"x": float("nan")}).values()), hold_err=None, max_tq=None, rate_hz=None)
        else:
            r.update(kind=name.split("_2026")[0], joint="", gains="", move="", ff="", others="",
                     outcome=d.get("outcome", ""), trigger="", max_err=None, hold_err=None, max_tq=None, rate_hz=None)
        r["note"] = NOTES.get(name, "")
        rows.append(r)
    f = lambda v, s: "" if v is None or (isinstance(v, float) and math.isnan(v)) else format(v, s)  # noqa: E731
    lines = ["# Run index (generated by analyze.py from runs/*.json)", "",
             "Every real hardware run of 2026-10-14 and 2026-10-17, in time order. `move` = q_target - q_start (rad); "
             "errors in rad; `gains` = kp:kd (one pair, or the sweep list).", "",
             "| run | kind | joint | gains | move | feed-forward | others | outcome | max err | hold err | max Nm | Hz | trigger / refusal | note |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| `{r['run']}` | {r['kind']} | {r['joint']} | {r['gains']} | {r['move']} | {r['ff']} | {r['others']} | "
                     f"{r['outcome']} | {f(r['max_err'], '.4f')} | {f(r['hold_err'], '+.4f')} | {f(r['max_tq'], '.2f')} | "
                     f"{f(r['rate_hz'], '.0f')} | {r['trigger']} | {r['note']} |")
    (HERE / "RUNS.md").write_text("\n".join(lines) + "\n")
    METRICS["n_runs"] = len(rows)
    METRICS["n_runs_by_outcome"] = {o: sum(1 for r in rows if r["outcome"] == o) for o in sorted({r["outcome"] for r in rows})}
    print("wrote RUNS.md,", len(rows), "runs")
    return rows


# ---------------------------------------------------------------------------------------------------------- figures
def fig_min_jerk() -> None:
    s = np.linspace(0, 1, 400)
    q = 10 * s**3 - 15 * s**4 + 6 * s**5
    v = 30 * s**2 - 60 * s**3 + 30 * s**4
    a = 60 * s - 180 * s**2 + 120 * s**3
    fig, ax = plt.subplots(1, 3, figsize=(10, 2.8))
    for axi, y, lab in zip(ax, (q, v, a), ("position  q/Δ", "velocity  q̇·T/Δ  (peak 1.875)", "acceleration  q̈·T²/Δ  (peak ±5.77)")):
        axi.plot(s, y)
        axi.set_xlabel("s = t/T")
        axi.set_title(lab)
    save(fig, "f01_min_jerk_profile.png", "Min-jerk reference used for every move")


def fig_j3_progression() -> None:
    runs = [("first_move_j3_20261017T184426Z", "kp100, no feed-forward, others free"),
            ("first_move_j3_20261017T184845Z", "kp100 + tau_ff -2.8 (URDF), others free"),
            ("first_move_j3_20261017T190527Z", "all held, URDF gravity x1"),
            ("first_move_j3_20261017T191042Z", "all held, gravity j3 x2.5 (faded in)")]
    fig, ax = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)
    hold = []
    for name, lab in runs:
        d = load_csv(name)
        t = d["t_s"]
        q0 = d["target_rad"][0]
        l, = ax[0].plot(t, d["pos_rad"] - q0, label=lab)
        ax[1].plot(t, d["target_rad"] - d["pos_rad"], color=l.get_color())
        m = d["phase"] == "hold"
        hold.append((lab, float(np.mean(d["target_rad"][m] - d["pos_rad"][m]))))
    d = load_csv(runs[0][0])
    ax[0].plot(d["t_s"], d["target_rad"] - d["target_rad"][0], "k--", lw=1, label="reference")
    # friction compensated (sweep segment kp100 kd1)
    d = load_csv("first_move_j3_20261017T191215Z")
    m = d["seg"] == 3
    t = d["t_s"][m] - d["t_s"][m][0]
    ax[0].plot(t, d["pos_rad"][m] - d["target_rad"][m][0], label="+ friction ff 2 Nm (kp100 kd1, 1 s ramps)")
    ax[1].plot(t, d["target_rad"][m] - d["pos_rad"][m])
    mh = m & (d["phase"] == "hold")
    hold.append(("+ friction ff", float(np.mean(d["target_rad"][mh] - d["pos_rad"][mh]))))
    ax[0].set_ylabel("q - q_start (rad)")
    ax[1].set_ylabel("tracking error q_ref - q (rad)")
    ax[1].set_xlabel("t since enable (s)")
    ax[0].legend(fontsize=7)
    METRICS["j3_progression_hold_err"] = hold
    save(fig, "f02_j3_compensation_progression.png", "J3 lift (-0.10 rad): what each compensation step bought")


def fig_j3_hysteresis() -> None:
    d = load_csv("first_move_j3_20261017T191042Z")
    fig, ax = plt.subplots(figsize=(7.5, 5))
    for ph, c in (("ramp_up", "C0"), ("hold", "C1"), ("ramp_down", "C2"), ("final_hold", "C3")):
        m = d["phase"] == ph
        ax.plot(d["pos_rad"][m], d["torque_nm"][m], ".", ms=2, color=c, label=ph)
    hu = d["phase"] == "hold"
    hd = d["phase"] == "final_hold"
    up, dn = float(np.median(d["torque_nm"][hu])), float(np.median(d["torque_nm"][hd]))
    ff = float(np.median(d["tau_ff_nm"][hu]))
    centre, half = (up + dn) / 2, abs(up - dn) / 2
    for y, lab, st in ((up, f"stuck going up: {up:.2f} Nm", "--"), (dn, f"stuck coming down: {dn:.2f} Nm", "--"),
                       (centre, f"band centre = gravity ≈ {centre:.2f} Nm", "-"), (ff, f"feed-forward x2.5 = {ff:.2f} Nm", ":")):
        ax.axhline(y, ls=st, color="k", lw=0.9)
        ax.text(ax.get_xlim()[0] if False else np.min(d["pos_rad"]), y + 0.12, lab, fontsize=7)
    ax.set_xlabel("J3 position (rad)")
    ax.set_ylabel("J3 motor torque (Nm)")
    ax.legend(fontsize=7, loc="lower right")
    METRICS["j3_hysteresis"] = {"stick_up_nm": up, "stick_down_nm": dn, "gravity_est_nm": centre, "friction_est_nm": half,
                                "ff_nm": ff, "urdf_gravity_nm": gravity.GravityModel().hold_torques({"j3": -0.08})["j3"]}
    save(fig, "f03_j3_torque_position_hysteresis.png",
         f"J3 static identification: gravity ≈ {centre:.1f} Nm, Coulomb friction ≈ ±{half:.1f} Nm")


def fig_gravity_model() -> None:
    gm = gravity.GravityModel()
    ext = gravity.GravityModel(extra=(("gripper_link", (-0.0783, -0.0090, -0.0640), 0.10),)
                               + tuple((l, (0, 0, 0), 0.30) for l in ("link5", "link6", "gripper_link")))
    s = np.linspace(0, 1, 60)
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.6))
    for axi, j in zip(ax, ("j2", "j3", "j4")):
        q = [{"j2": -0.6 * x, "j3": -1.2 * x} for x in s]
        g = [gm.hold_torques(qq)[j] for qq in q]
        e = [ext.hold_torques(qq)[j] for qq in q]
        sc = {"j2": 0.6, "j3": 2.5, "j4": 2.5}[j]
        axi.plot(s, g, label="URDF")
        axi.plot(s, np.array(g) * sc, label=f"URDF x{sc} (used)")
        axi.plot(s, e, "--", label="URDF + camera 0.1 kg + 3x0.3 kg wrist motors (hypothesis)")
        axi.set_title(j)
        axi.set_xlabel("fraction of the 'rise' (j2 -0.6, j3 -1.2)")
        axi.set_ylabel("holding torque (Nm)")
    ax[0].legend(fontsize=6)
    save(fig, "f04_gravity_model_along_rise.png", "Gravity holding torque along the demo's rise: model, scaled model, extra-mass hypothesis")


SWEEPS = {
    "J1 (kp)": "first_move_j1_20261017T191629Z", "J2 (kp, gravity x1, fric 1.5)": "first_move_j2_20261017T191406Z",
    "J2 (gravity x0.6, fric 1.0)": "first_move_j2_20261017T191510Z", "J3 (kp/kd)": "first_move_j3_20261017T191215Z",
    "J4": "first_move_j4_20261017T191836Z", "J5": "first_move_j5_20261017T191928Z", "J6": "first_move_j6_20261017T191942Z",
    "J1 (kd 1/2/4)": "first_move_j1_20261017T192212Z", "J2 (kd 1/2/4)": "first_move_j2_20261017T192226Z",
    "J3 (kd 1/2/4)": "first_move_j3_20261017T192240Z",
}


def fig_sweeps() -> None:
    table = {}
    for label, name in SWEEPS.items():
        d, js = load_csv(name), load_json(name)
        sw = js["sweep"]
        table[label] = sw
        fig, ax = plt.subplots(1, 3, figsize=(13, 3.4), gridspec_kw={"width_ratios": [2, 2, 1.2]})
        for r in sw:
            m = d["seg"] == r["seg"]
            t = d["t_s"][m] - d["t_s"][m][0]
            ax[0].plot(t, d["pos_rad"][m] - d["target_rad"][m][0], label=f"kp {r['kp']:g} kd {r['kd']:g}")
            ax[1].plot(t, d["target_rad"][m] - d["pos_rad"][m])
        m0 = d["seg"] == 0
        ax[0].plot(d["t_s"][m0] - d["t_s"][m0][0], d["target_rad"][m0] - d["target_rad"][m0][0], "k--", lw=1, label="ref")
        ax[0].set_ylabel("q - q_start (rad)")
        ax[1].set_ylabel("q_ref - q (rad)")
        for a in ax[:2]:
            a.set_xlabel("t in segment (s)")
        ax[0].legend(fontsize=7)
        labs = [f"{r['kp']:g}:{r['kd']:g}" for r in sw]
        ax[2].bar(labs, [r["score"] for r in sw], color=["C%d" % i for i in range(len(sw))])
        ax[2].set_title("score (lower = better)")
        ax[2].set_xlabel("kp:kd")
        save(fig, f"f05_sweep_{name.split('_')[2]}_{name.split('_')[-1]}.png",
             f"Gain sweep {label} — {js['params']['gravity_joint_scale'] or ''} friction {js['params']['friction_nm']} Nm")
    METRICS["sweeps"] = table
    lines = ["# Gain sweeps (generated by analyze.py from runs/*.json `sweep` blocks)", "",
             "score = rms_track + |hold_err| + 2*overshoot + 0.01*osc_flips (rad); lower is better. Each pair = settle 0.5 s, "
             "min-jerk ramp, 1 s hold, ramp back, 1 s hold, all in one enable.", ""]
    for label, name in SWEEPS.items():
        p = load_json(name)["params"]
        lines += [f"## {label} — `{name}`", "",
                  f"gravity scales {p.get('gravity_joint_scale') or '{}'}, friction {p.get('friction_nm')} Nm, ramp {p.get('ramp_s')} s", "",
                  "| kp | kd | rms_track (rad) | hold_err (rad) | overshoot (rad) | osc | peak Nm | score |", "|---|---|---|---|---|---|---|---|"]
        best = min(table[label], key=lambda r: r["score"])
        for r in table[label]:
            b = " **best**" if r is best else ""
            lines.append(f"| {r['kp']:g} | {r['kd']:g} | {r['rms_track_rad']:.4f} | {r['hold_err_rad']:+.4f} | "
                         f"{r['overshoot_rad']:.4f} | {r['osc_flips']} | {r['peak_torque_nm']:.2f} | {r['score']:.4f}{b} |")
        lines.append("")
    (HERE / "SWEEPS.md").write_text("\n".join(lines))
    print("wrote SWEEPS.md")


def fig_sweep_summary() -> None:
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
    for label, sw in METRICS["sweeps"].items():
        kp = [r["kp"] for r in sw]
        if "kd 1/2/4" in label:
            ax[2].plot([r["kd"] for r in sw], [r["rms_track_rad"] for r in sw], "o-", label=label)
            continue
        if len({r["kd"] for r in sw}) == 1 or True:
            ax[0].plot(kp, [r["rms_track_rad"] for r in sw], "o-", label=label)
            ax[1].plot(kp, [abs(r["hold_err_rad"]) for r in sw], "o-", label=label)
    ax[0].set(xlabel="kp", ylabel="RMS tracking error during ramps (rad)", title="tracking vs kp")
    ax[1].set(xlabel="kp", ylabel="|mean hold error| (rad)", title="hold error vs kp")
    ax[2].set(xlabel="kd", ylabel="RMS tracking error (rad)", title="kd sweeps (J1-J3)")
    for a in ax:
        a.legend(fontsize=6)
    save(fig, "f06_sweep_summary.png", "All sweeps: stiffer kp tracks better; kd only helped J1")


def fig_velocity_spikes() -> None:
    fig, ax = plt.subplots(2, 1, figsize=(9, 5))
    d = load_csv("first_move_j7_20261017T184226Z")
    ax[0].plot(d["t_s"], d["target_vel_rad_s"], "k--", label="planned velocity")
    ax[0].plot(d["t_s"], d["vel_rad_s"], ".-", ms=2, lw=0.6, label="reported velocity (J7, DM4310)")
    ax[0].axhline(-0.5, color="r", lw=0.8)
    ax[0].text(d["t_s"][0], -0.48, "abort |v| > 0.5", color="r", fontsize=7)
    ax[0].set_ylabel("rad/s")
    ax[0].legend(fontsize=7)
    pos = d["pos_rad"]
    vdiff = np.gradient(pos, d["t_s"])
    ax[1].plot(d["t_s"], vdiff, label="d(position)/dt (numerical)")
    ax[1].plot(d["t_s"], d["vel_rad_s"], alpha=0.6, label="reported velocity")
    ax[1].set_ylabel("rad/s")
    ax[1].set_xlabel("t (s)")
    ax[1].legend(fontsize=7)
    save(fig, "f07_velocity_spikes_j7.png", "Single-sample velocity spikes: the J7 abort that tripped at 0.505 rad/s while really moving ~0.17 rad/s")


def fig_quantization() -> None:
    res = {}
    for j, name in (("j1", "first_move_j1_20261017T191629Z"), ("j3", "first_move_j3_20261017T191215Z"),
                    ("j4", "first_move_j4_20261017T191836Z"), ("j7", "first_move_j7_20261017T184241Z")):
        d = load_csv(name)
        out = {}
        for col in ("pos_rad", "vel_rad_s", "torque_nm"):
            u = np.unique(np.round(d[col], 7))
            steps = np.diff(u)
            steps = steps[steps > 1e-7]
            out[col] = float(np.min(steps)) if len(steps) else None
        res[j] = out
    METRICS["feedback_lsb_measured"] = res
    lines = ["| joint | motor | position LSB (rad) | velocity LSB (rad/s) | torque LSB (Nm) |", "|---|---|---|---|---|"]
    model = {"j1": "DM4340P", "j3": "DM4340P", "j4": "DM4310", "j7": "DM4310"}
    for j, o in res.items():
        lines.append(f"| {j} | {model[j]} | {o['pos_rad']:.6f} | {o['vel_rad_s']:.5f} | {o['torque_nm']:.4f} |")
    (HERE / "feedback_resolution.md").write_text("Smallest step between distinct reported values (measured from the logs):\n\n"
                                                 + "\n".join(lines) + "\n")
    print("wrote feedback_resolution.md")


def fk_gripper(gm, q: dict) -> tuple[float, float, float]:
    urdf_q = {gravity.ARM_JOINTS[k]: v for k, v in q.items() if k in gravity.ARM_JOINTS}
    T = {gm.base: [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]}
    for name, typ, p, c, org, ax in gm.order:
        Tj = gravity._mm(T[p], org)
        if typ in ("revolute", "continuous"):
            Tj = gravity._mm(Tj, gravity._axis_rot(ax, urdf_q.get(name, 0.0)))
        T[c] = Tj
    G = T["gripper_link"]
    return G[0][3], G[1][3], G[2][3]


def fig_demo(name: str, label: str) -> None:
    d = load_csv(name)
    t = d["t_s"]
    fig, ax = plt.subplots(7, 2, figsize=(12, 13), sharex=True)
    errs = {}
    for i, j in enumerate(JOINTS):
        ax[i, 0].plot(t, d[f"{j}_ref"], "k--", lw=0.8, label="reference")
        ax[i, 0].plot(t, d[f"{j}_pos"], lw=0.9, label="measured")
        ax[i, 0].set_ylabel(f"{j} (rad)")
        e = d[f"{j}_ref"] - d[f"{j}_pos"]
        ax[i, 1].plot(t, e, lw=0.7)
        ax[i, 1].set_ylabel(f"{j} err (rad)")
        errs[j] = {"max_abs": float(np.nanmax(np.abs(e))), "rms": float(np.sqrt(np.nanmean(e**2)))}
    ax[0, 0].legend(fontsize=7)
    ax[-1, 0].set_xlabel("t (s)")
    ax[-1, 1].set_xlabel("t (s)")
    METRICS[f"demo_{label}"] = errs
    save(fig, f"f09_demo_{label}_tracking.png", f"Demo '{label}': all seven joints, reference vs measured, and tracking error")
    # 3D gripper path (forward kinematics of the measured joints)
    gm = gravity.GravityModel()
    idx = np.arange(0, len(t), 10)
    pts = np.array([fk_gripper(gm, {j: d[f"{j}_pos"][k] for j in JOINTS[:6]}) for k in idx])
    ref = np.array([fk_gripper(gm, {j: d[f"{j}_ref"][k] for j in JOINTS[:6]}) for k in idx])
    fig = plt.figure(figsize=(7, 6))
    a3 = fig.add_subplot(111, projection="3d")
    a3.plot(ref[:, 0], ref[:, 1], ref[:, 2], "k--", lw=0.8, label="reference (FK)")
    sc = a3.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=t[idx], s=3, cmap="viridis", label="measured (FK)")
    fig.colorbar(sc, label="t (s)", shrink=0.6)
    a3.set_xlabel("x (m)")
    a3.set_ylabel("y (m)")
    a3.set_zlabel("z (m)")
    a3.legend(fontsize=7)
    dev = np.linalg.norm(pts - ref, axis=1)
    METRICS[f"demo_{label}_gripper_cartesian_dev_m"] = {"max": float(dev.max()), "rms": float(np.sqrt(np.mean(dev**2)))}
    save(fig, f"f10_demo_{label}_gripper_path_3d.png",
         f"Demo '{label}': gripper_link path from forward kinematics (max deviation {dev.max() * 1000:.1f} mm)")


def fig_loop_timing() -> None:
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.2))
    d = load_csv("first_move_j3_20261017T191215Z")
    g = d["cmd_gap_s"][1:] * 1000
    ax[0].hist(g, bins=80)
    ax[0].set(xlabel="time between command cycles (ms)", ylabel="count", title="J3 sweep, 7 motors live (target 5 ms)")
    ax[0].set_yscale("log")
    d = load_csv("demo_show_20261017T195627Z")
    g2 = np.diff(d["t_s"]) * 1000
    ax[1].hist(g2, bins=80, color="C1")
    ax[1].set(xlabel="cycle period (ms)", title="'show' demo, 7 motors moving")
    ax[1].set_yscale("log")
    METRICS["loop_timing_ms"] = {"sweep_mean": float(np.mean(g)), "sweep_p99": float(np.percentile(g, 99)), "sweep_max": float(np.max(g)),
                                 "demo_mean": float(np.mean(g2)), "demo_p99": float(np.percentile(g2, 99)), "demo_max": float(np.max(g2))}
    save(fig, "f11_loop_timing.png", "200 Hz loop over one 921600-baud serial bridge: cycle-period distribution")


def fig_j4_sag() -> None:
    fig, ax = plt.subplots(figsize=(8, 3.5))
    for name, lab in (("first_move_j3_20261017T190527Z", "J4 held kp20, URDF gravity x1"),
                      ("first_move_j3_20261017T191215Z", "J4 held kp30, gravity x2.5")):
        d = load_csv(name)
        ax.plot(d["t_s"], d["j4_pos"] - d["j4_pos"][0], label=lab)
    ax.set(xlabel="t (s)", ylabel="held J4 deviation from hold (rad)")
    ax.legend(fontsize=8)
    save(fig, "f12_held_j4_during_j3_lift.png", "Held wrist (J4) while J3 lifts: soft hold + low gravity scale let it sag")


def fig_friction_ff() -> None:
    v = np.linspace(-0.2, 0.2, 400)
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.2))
    for eps in (0.005, 0.02, 0.05):
        ax[0].plot(v, np.tanh(v / eps), label=f"ε = {eps}")
    ax[0].set(xlabel="commanded velocity q̇_ref (rad/s)", ylabel="τ_fric / F_c", title="smooth sign: tanh(q̇_ref/ε), ε = 0.02 used")
    ax[0].legend(fontsize=7)
    a = load_csv("first_move_j3_20261017T191042Z")
    b = load_csv("first_move_j3_20261017T191215Z")
    m = b["seg"] == 3
    ax[1].plot(a["t_s"] / a["t_s"][-1], a["target_rad"] - a["pos_rad"], label="gravity x2.5, no friction ff (2 s ramps)")
    tb = b["t_s"][m] - b["t_s"][m][0]
    ax[1].plot(tb / tb[-1], b["target_rad"][m] - b["pos_rad"][m], label="+ friction ff 2 Nm (1 s ramps)")
    ax[1].set(xlabel="normalised time", ylabel="J3 tracking error (rad)", title="effect on J3 (kp 100, kd 1)")
    ax[1].legend(fontsize=7)
    save(fig, "f13_friction_feedforward.png", "Coulomb friction feed-forward")


def fig_sim_vs_real() -> None:
    """Re-run the J3 sweep in the offline simulator (code/fakebus.py) with the identified load and friction."""
    import fakebus
    import first_move_j1 as fm
    bus = fakebus.FakeBus()
    for n, tq in gravity.GravityModel().hold_torques(dict(bus.rest)).items():
        bus.sims[n].load = -tq * (2.5 if n in ("j3", "j4") else 1.0)
    bus.sims["j3"].coulomb, bus.sims["j3"].inertia = 2.3, 0.3
    out = Path(tempfile.mkdtemp())
    rt = fm.Runtime(clock=bus.clock, sleep=bus.clock.sleep, tty_reader=lambda p: "ENABLE J1", out_dir=out, mb=bus.mod,
                    simulated=True, check_host=False, require_confirmation=False, install_signals=False, log=lambda s: None)
    fm.run(["--joint", "j3", "--move", "-0.10", "--ramp-s", "1", "--hold-others", "--gravity", "--gravity-joint-scale",
            "j3=2.5,j4=2.5", "--friction", "2.0", "--sweep", "60:0.5,80:1.0,100:0.5,100:1.0"], rt=rt, bus_factory=bus.factory)
    simf = next(out.glob("*.csv"))
    with open(simf) as fh:
        rows = list(csv.DictReader(fh))
    sim = {k: np.array([float(r[k]) if r[k] not in ("",) and k != "phase" else np.nan for r in rows]) for k in ("t_s", "seg", "target_rad", "pos_rad")}
    real = load_csv("first_move_j3_20261017T191215Z")
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6), sharey=True)
    for a, d, lab in ((ax[0], real, "real arm"), (ax[1], sim, "simulator (identified load + friction)")):
        for k in range(4):
            m = d["seg"] == k
            t = d["t_s"][m] - d["t_s"][m][0]
            a.plot(t, d["target_rad"][m] - d["pos_rad"][m], label=f"segment {k}")
        a.set(title=lab, xlabel="t in segment (s)")
    ax[0].set_ylabel("J3 tracking error (rad)")
    ax[0].legend(fontsize=7)
    save(fig, "f14_j3_sweep_sim_vs_real.png", "J3 sweep (kp:kd 60:0.5, 80:1, 100:0.5, 100:1): real arm vs offline simulator")


def fig_first_moves_1014() -> None:
    names = sorted(Path(p).stem for p in glob.glob(str(RUNS / "first_move_j*_20261014*.csv")))
    good = []
    for n in names:
        js = load_json(n)
        if js.get("outcome") in ("completed", "aborted"):
            good.append(n)
    fig, axs = plt.subplots(len(good) // 3 + 1, 3, figsize=(12, 2.4 * (len(good) // 3 + 1)))
    for a, n in zip(axs.flat, good):
        d = load_csv(n)
        js = load_json(n)
        a.plot(d["t_s"], d["target_rad"], "k--", lw=0.8)
        a.plot(d["t_s"], d["pos_rad"])
        p = js.get("params", {})
        a.set_title(f"{n.split('_')[2]} kp {p.get('kp')} — {js.get('outcome')}", fontsize=8)
    for a in list(axs.flat)[len(good):]:
        a.axis("off")
    save(fig, "f16_first_powered_moves_2026-10-14.png", "2026-10-14: first powered moves, one joint at a time, no feed-forward")


def main() -> None:
    run_index()
    fig_min_jerk()
    fig_j3_progression()
    fig_j3_hysteresis()
    fig_gravity_model()
    fig_sweeps()
    fig_sweep_summary()
    fig_velocity_spikes()
    fig_quantization()
    fig_demo("demo_wave_20261017T194533Z", "wave")
    fig_demo("demo_show_20261017T195627Z", "show")
    fig_loop_timing()
    fig_j4_sag()
    fig_friction_ff()
    fig_sim_vs_real()
    fig_first_moves_1014()
    (HERE / "metrics.json").write_text(json.dumps(METRICS, indent=2, default=float))
    print("wrote metrics.json")


if __name__ == "__main__":
    main()

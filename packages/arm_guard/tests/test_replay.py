"""Replay every real run logged on the arm (CONTROLLER_TUNING, 2026-10-14/17) through the rules.

Commands that really ran safely must pass (no false refusals). The one exception is by design: the "show" demo
turned J1 to -3.77 rad, beyond the URDF's +-2.8 (allowed only by that script's own override)."""

import csv
import glob
import json
import os

import pytest

from arm_guard import core
from conftest import REPO

RUNS = sorted(glob.glob(str(REPO / "evidence/operator/controller-tuning-2026-10/runs/*.csv")))
URDF = {f"j{i}": f"joint{i}" for i in range(1, 7)}


def _replay(cfg, path):
    meta = json.load(open(path[:-4] + ".json"))
    rows = list(csv.DictReader(open(path)))
    reasons = []
    if os.path.basename(path).startswith("demo"):
        prev = None
        for r in rows:
            t = float(r["t_s"])
            for n, j in cfg.joints.items():
                fb = core.Feedback(float(r[f"{n}_pos"]), float(r[f"{n}_vel"]), float(r[f"{n}_torq"]), 1, 0.0)
                ref = float(r[f"{n}_ref"])
                vel = 0.0 if prev is None or t == prev[0] else (ref - float(prev[1][f"{n}_ref"])) / (t - prev[0])
                kp, kd = meta["gains"][n]
                cmd = core.Command("mit", pos=ref, vel=vel, kp=kp, kd=kd, tau=float(r[f"{n}_ff"]))
                reasons += core.check_state(j, fb, cfg) + list(core.check_command(j, cmd, fb, cfg).reasons)
            qr = {URDF[n]: float(r[f"{n}_ref"]) for n in URDF}
            reasons += core.check_pose(cfg, qr, qr)
            prev = (t, r)
    else:
        n = os.path.basename(path).split("_")[2]
        j = cfg.joints[n]
        for r in rows:
            fb = core.Feedback(float(r["pos_rad"]), float(r["vel_rad_s"]), float(r["torque_nm"]), int(r["status"]), 0.0)
            cmd = core.Command("mit", pos=float(r["target_rad"]), vel=float(r["target_vel_rad_s"]),
                               kp=float(r.get("kp") or meta["params"]["kp"]),
                               kd=float(r.get("kd") or meta["params"]["kd"]), tau=float(r.get("tau_ff_nm") or 0.0))
            reasons += core.check_state(j, fb, cfg) + list(core.check_command(j, cmd, fb, cfg).reasons)
    return reasons


def test_all_runs_found():
    assert len(RUNS) == 34


@pytest.mark.parametrize("path", [p for p in RUNS if "demo_show" not in p], ids=os.path.basename)
def test_real_run_accepted(cfg, path):
    assert _replay(cfg, path) == []


def test_show_demo_refused_beyond_urdf(cfg):
    reasons = _replay(cfg, next(p for p in RUNS if "demo_show" in p))
    assert reasons and all(r.startswith("j1:") for r in reasons)
    assert any("-3.76" in r for r in reasons)

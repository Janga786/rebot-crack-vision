"""tools/analyze_run_gains.py: the error, torque-offset and ripple statistics on a synthetic run CSV."""

from __future__ import annotations

import csv
import json
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("analyze_run_gains", REPO / "tools" / "analyze_run_gains.py")
arg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(arg)


def write_run(path: Path, *, lag_rad: float, torque_offset: float, ripple: float, n: int = 4000) -> None:
    names = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"]
    cols = ["t_s", "mode", "seg"] + [f"{j}_{c}" for j in names for c in ("ref", "pos", "vel", "torq", "ff")]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for i in range(n):
            t = i * 0.005
            row = [f"{t:.4f}", "path", i * 3 // n]
            for j in names:
                ref = 0.3 * math.sin(0.5 * t)
                pos = ref - lag_rad + ripple * math.sin(2 * math.pi * 3.0 * t)
                ff = -2.0
                row += [f"{ref:.6f}", f"{pos:.6f}", "0", f"{ff + torque_offset:.3f}", f"{ff:.3f}"]
            w.writerow(row)


def test_statistics_recover_the_synthetic_lag_offset_and_ripple(tmp_path):
    p = tmp_path / "run.csv"
    write_run(p, lag_rad=0.012, torque_offset=-0.9, ripple=0.0005)
    r = arg.analyse(arg.load(p))["j3"]
    assert r["mean_err_mrad"] == 12.0
    assert abs(r["torque_minus_ff_mean_nm"] + 0.9) < 1e-6
    assert abs(r["ripple_hz"] - 3.0) < 0.5
    assert 0.30 < r["ripple_mrad"] < 0.40                      # 0.5 mrad amplitude -> 0.354 mrad rms, not attenuated at 3 Hz
    assert set(r["by_segment_rms_mrad"]) == {0, 1, 2}


def test_gzip_and_short_runs(tmp_path):
    import gzip
    p = tmp_path / "run.csv"
    write_run(p, lag_rad=0.0, torque_offset=0.0, ripple=0.0)
    gz = tmp_path / "run.csv.gz"
    gz.write_bytes(gzip.compress(p.read_bytes()))
    assert len(arg.load(gz)) == 4000
    assert arg.main([str(tmp_path / "run.csv")]) == 0


def test_band_rms_does_not_attenuate_low_frequencies():
    dt = 0.005
    t = np.arange(0, 20, dt)
    for f in (3.0, 10.0):
        rms, hz = arg.band_rms(0.001 * np.sin(2 * math.pi * f * t) + 0.2 * np.sin(2 * math.pi * 0.4 * t), dt)
        assert abs(rms - 0.001 / math.sqrt(2)) < 0.05e-3 and abs(hz - f) < 0.2


def write_posed_run(path: Path, model_extra, *, ff_kind: str, n: int = 3000) -> None:
    """A slow J2/J3/J4 sweep whose motor torque is exactly the gravity of `model_extra` (no friction while the
    commanded velocity is ~0 at the ends), logged with ff = URDF x the runner's scales ('scales') or the model."""
    from crackvision import gravity_model as gmod
    ug = gmod.UrdfGravity()
    names = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"]
    t = np.arange(n) * 0.005
    s = 0.5 - 0.5 * np.cos(2 * np.pi * t / t[-1])
    q = np.zeros((n, 6))
    q[:, 1], q[:, 2], q[:, 3] = -0.6 - 0.4 * s, -0.9 - 0.4 * s, 0.1 + 0.3 * s
    g_true = ug.hold_torques(q, model_extra)
    v = np.gradient(q, 0.005, axis=0)
    fric = gmod.runner_friction(v)
    ff = (ug.hold_torques(q) * np.array([gmod.RUNNER_SCALE[k] for k in gmod.ARM]) if ff_kind == "scales" else g_true) + fric
    tq = g_true + fric
    cols = ["t_s", "mode", "seg"] + [f"{j}_{c}" for j in names for c in ("ref", "pos", "vel", "torq", "ff")]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for i in range(n):
            row = [f"{t[i]:.4f}", "path", 1]
            for j in range(7):
                row += ([f"{q[i, j]:.6f}", f"{q[i, j]:.6f}", "0", f"{tq[i, j]:.4f}", f"{ff[i, j]:.4f}"] if j < 6
                        else ["0", "0", "0", "0", "0"])
            w.writerow(row)


EXTRA = [("link5", (0.0, 0.0, 0.03), 0.6), ("gripper_link", (-0.05, 0.0, 0.0), 0.3)]


def model_file(tmp_path: Path) -> Path:
    from crackvision import gravity_model as gmod
    doc = {"schema": gmod.SCHEMA, "urdf": {"sha256": gmod.file_sha256(gmod.DEFAULT_URDF)},
           "bodies": [{"link": l, "xyz": list(x), "kg": k} for l, x, k in EXTRA], "scale": {"j2": 1.0, "j3": 1.0, "j4": 1.0}}
    p = tmp_path / "gravity_model.yaml"
    p.write_text(gmod.dump_model_text(doc, "test"))
    return p


def test_gravity_model_rescore_shows_what_the_p_term_would_have_supplied(tmp_path, capsys):
    run = tmp_path / "run.csv"
    write_posed_run(run, EXTRA, ff_kind="scales")
    from crackvision import gravity_model as gmod
    r = arg.rescore(arg.load(run), gmod.load_model(model_file(tmp_path)))
    for j in ("j2", "j3", "j4"):
        assert abs(r[j]["model_mean_nm"]) < 0.01 and r[j]["model_rms_nm"] < 0.01      # the model is the truth here
    assert abs(r["j3"]["logged_mean_nm"]) > 0.5                                         # the scales were not
    assert arg.main([str(run), "--gravity-model", str(model_file(tmp_path))]) == 0
    assert "re-scored against" in capsys.readouterr().out


def test_compare_two_runs(tmp_path, capsys):
    a, b = tmp_path / "a.csv", tmp_path / "b.csv"
    write_run(a, lag_rad=0.010, torque_offset=-0.9, ripple=0.0)
    write_run(b, lag_rad=0.002, torque_offset=-0.1, ripple=0.0)
    c = arg.compare(arg.analyse(arg.load(a)), arg.analyse(arg.load(b)))
    assert c["j3"]["mean_err_mrad"]["B-A"] == -8.0
    assert abs(c["j2"]["torque_minus_ff_mean_nm"]["B-A"] - 0.8) < 1e-6
    assert arg.main(["--compare", str(a), str(b)]) == 0
    out = capsys.readouterr().out
    assert "A = a.csv" in out and "B = b.csv" in out


# ---- --friction-fit (docs/motion/J4_TUNING.md) ---------------------------------------------------------------------

def write_friction_run(path: Path, *, sgn_rad: float, const_rad: float, kp: float = 40.0, ff_coulomb: float = 0.5,
                       shared: float = 0.0012, n: int = 8000, seed: int = 1, summary: dict | None = None) -> None:
    """J4 sweeps +-0.15 rad (|v| <= 0.19 rad/s, both directions) in segment 1 with an error made of a constant, a known
    sign-of-velocity term and a 3.3 Hz ripple shared with J3 (+ independent noise); torq - ff = kp x e (quasi-static).
    J1 moves too, with only independent noise (no sign term, incoherent with J3)."""
    rng = np.random.default_rng(seed)
    names = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"]
    t = np.arange(n) * 0.005
    seg = np.where(t < 4.0, 0, np.where(t < t[-1] - 4.0, 1, 2))
    ripple = np.sin(2 * math.pi * 3.3 * t + 0.3)
    ref4 = 0.2 + 0.15 * np.sin(2 * math.pi * 0.2 * t)
    v4 = np.gradient(ref4, 0.005)
    e4 = const_rad + sgn_rad * np.tanh(v4 / 0.02) + shared * ripple + 0.0002 * rng.standard_normal(n)
    e3 = -0.002 + 0.0015 * ripple + 0.0002 * rng.standard_normal(n)
    ref1 = 0.1 * np.sin(2 * math.pi * 0.1 * t)
    e1 = 0.0005 * rng.standard_normal(n)
    cols = ["t_s", "mode", "seg"] + [f"{j}_{c}" for j in names for c in ("ref", "pos", "vel", "torq", "ff")]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for i in range(n):
            row = [f"{t[i]:.4f}", "path", int(seg[i])]
            for j in names:
                ref, e, k = {"j1": (ref1[i], e1[i], 100.0), "j3": (-1.2, e3[i], 80.0), "j4": (ref4[i], e4[i], kp)}.get(j, (0.0, 0.0, 40.0))
                ff = 0.5 + ff_coulomb * math.tanh(v4[i] / 0.02) if j == "j4" else 0.0
                row += [f"{ref:.6f}", f"{ref - e:.6f}", "0", f"{ff + k * e:.5f}", f"{ff:.5f}"]
            w.writerow(row)
    if summary is not None:
        path.with_suffix(".json").write_text(json.dumps(summary))


def test_friction_fit_recovers_a_known_sign_term_and_the_implied_coulomb(tmp_path):
    p = tmp_path / "run.csv"
    write_friction_run(p, sgn_rad=-0.008, const_rad=-0.005)
    ff = arg.friction_fit(arg.load(p), {j: 40.0 for j in arg.JOINTS}, {j: 0.5 for j in arg.JOINTS})
    d = ff["j4"]["segments"][1]
    assert abs(d["error"]["sgn_mrad"] + 8.0) < 0.2 and abs(d["error"]["const_mrad"] + 5.0) < 0.2
    assert abs(d["error"]["sgn_nm_at_kp"] + 0.32) < 0.01
    assert abs(d["torque"]["sgn_nm"] + 0.32) < 0.01                       # torq - ff = kp e here
    assert abs(d["implied_coulomb_nm"] - 0.18) < 0.01                     # 0.5 ff + (-0.32)
    up, down = d["implied_coulomb_up_down_nm"]
    assert abs(up - 0.18) < 0.02 and abs(down - 0.18) < 0.02             # symmetric by construction
    assert d["error"]["r2"] > 0.9 and abs(d["error"]["visc_mrad_per_rad_s"]) < 2.0 and not d["confounded"]
    assert ff["j1"]["segments"][1]["error"]["r2"] < 0.05                  # J1 has no sign term
    assert ff["j5"]["segments"][1].get("static")                          # J5 does not move: no fit


def test_coherence_with_j3_sees_the_shared_ripple_line(tmp_path):
    p = tmp_path / "run.csv"
    write_friction_run(p, sgn_rad=-0.008, const_rad=0.0)
    ff = arg.friction_fit(arg.load(p), {j: 40.0 for j in arg.JOINTS}, {j: 0.5 for j in arg.JOINTS})
    c4, c1 = ff["j4"]["segments"][1]["coherence_with_j3"], ff["j1"]["segments"][1]["coherence_with_j3"]
    assert c4["coherence"] > 0.8 and abs(c4["at_hz"] - 3.3) < 0.25
    assert c1["coherence"] < c1["zero_coherence_95"] + 0.1                # independent noise: near the zero level
    assert c4["shared_band_rms_mrad"] > c4["own_band_rms_mrad"]          # the 3.3 Hz line dominates J4's band here
    assert "coherence_with_j3" not in ff["j3"]["segments"][1]


def test_what_if_friction_removes_the_sign_term_and_kp_scales_the_rest(tmp_path):
    p = tmp_path / "run.csv"
    write_friction_run(p, sgn_rad=-0.008, const_rad=-0.005)
    rows = arg.load(p)
    kp, fr = {j: 40.0 for j in arg.JOINTS}, {j: 0.5 for j in arg.JOINTS}
    base = arg.friction_fit(rows, kp, fr)["j4"]["segments"][1]
    wi = arg.friction_fit(rows, kp, fr, {"j4": 0.18})["j4"]["segments"][1]["what_if"]
    assert abs(wi["sgn_mrad"]) < 0.3 and wi["rms_err_mrad"] < 0.4 * base["rms_err_mrad"]
    assert abs(wi["mean_err_mrad"] + 5.0) < 0.3                           # the constant (gravity) part stays
    both = arg.friction_fit(rows, kp, fr, {"j4": 0.18}, {"j4": 50.0})["j4"]["segments"][1]["what_if"]
    assert abs(both["rms_err_mrad"] - 0.8 * wi["rms_err_mrad"]) < 0.05 and both["kp"] == 50.0


def test_run_params_come_from_the_run_summary_and_cli(tmp_path, capsys):
    a, b = tmp_path / "a.csv", tmp_path / "b.csv"
    write_friction_run(a, sgn_rad=-0.008, const_rad=0.0)
    gains = {j: [40.0, 1.0] for j in ("j1", "j2", "j3", "j4", "j5", "j6", "j7")}
    write_friction_run(b, sgn_rad=-0.0005, const_rad=0.0, ff_coulomb=0.18, seed=2,
                       summary={"gains": gains, "friction": {"j1": 1.0, "j2": 1.0, "j3": 2.0, "j4": 0.18, "j5": 0.3}})
    assert arg.run_params(a)[2].startswith("tuned defaults (NO RUN SUMMARY") and arg.run_params(a)[1]["j4"] == 0.5
    kp, fr, src = arg.run_params(b)
    assert src == "run summary" and fr["j4"] == 0.18 and kp["j1"] == 40.0
    assert arg.main([str(a), str(b), "--friction-fit", "--what-if-friction", "j4=0.18", "--what-if-kp", "j4=50"]) == 0
    out = capsys.readouterr().out
    assert "friction fit" in out and "across 2 runs" in out and "what-if" in out
    with pytest.raises(SystemExit):
        arg.main([str(a), "--what-if-friction", "j4=0.18"])                # needs --friction-fit
    for bad in ("j4=4", "j9=0.1", "j4", "j4=-0.1"):
        with pytest.raises(ValueError):
            arg.parse_joint_values(bad, 0.0, 3.0, "Coulomb Nm")


def test_hold_flips_count_oscillation_only_while_holding():
    v_ref = np.r_[np.zeros(10), np.full(10, 0.3), np.zeros(10)]
    vel = np.r_[[0.1, -0.1, 0.1, 0.02, -0.02, -0.1, 0, 0, 0, 0], 0.3 * np.array([1, -1] * 5), [0.06, -0.06] + [0] * 8]
    assert arg.hold_flips(v_ref, vel) == 3 + 1                  # 3 in the first hold, none while moving, 1 in the last


def test_seg1_window_selects_file_time_scaled_by_the_runs_slow(tmp_path):
    p = tmp_path / "run.csv"
    write_friction_run(p, sgn_rad=-0.008, const_rad=0.0, summary={"slow": 2.0})
    rows = arg.load(p)
    t0 = float(next(x for x in rows if x["seg"] == "1")["t_s"])
    w = arg.seg1_window(rows, (1.0, 3.0), p)                   # file seconds 1..3 = run seconds 2..6 into segment 1
    assert {x["seg"] for x in w} == {"1"}
    assert abs(float(w[0]["t_s"]) - t0 - 2.0) < 0.006 and abs(float(w[-1]["t_s"]) - t0 - 6.0) < 0.006
    assert arg.main([str(p), "--friction-fit", "--seg1-window", "0:20"]) == 0
    with pytest.raises(SystemExit):
        arg.main([str(p), "--seg1-window", "5:1"])


# ---- audit 2026-10-29 items 4 / 5 ------------------------------------------------------------------------------------

def _relabel(path: Path, seg_map: dict) -> None:
    """Rewrite the seg column (e.g. a run whose zero-length approach was dropped: trajectory = segment 0)."""
    rows = list(csv.reader(open(path, newline="")))
    for r in rows[1:]:
        r[2] = str(seg_map[int(r[2])])
    with open(path, "w", newline="") as fh:
        csv.writer(fh).writerows(rows)


def test_trajectory_segment_is_found_by_kind_from_the_summary(tmp_path, capsys):
    """j4_tune.json / gravity_survey.json with --j1-here-is-zero: no approach, so the trajectory is segment 0 and the
    return is segment 1. The summary's path_kinds says so; --seg1-window and --friction-fit use segment 0."""
    p = tmp_path / "run.csv"
    write_friction_run(p, sgn_rad=-0.008, const_rad=0.0,
                       summary={"slow": 1.0, "choreo": "trajectory", "outcome": "completed",
                                "path_kinds": [["trajectory", 32.0], ["return", 4.0]]})
    _relabel(p, {0: 0, 1: 0, 2: 1})
    rows = arg.load(p)
    assert arg.trajectory_segment(p, rows)[:2] == (0, "run summary path_kinds")
    w = arg.seg1_window(rows, (1.0, 3.0), p)
    assert {x["seg"] for x in w} == {"0"} and abs(float(w[0]["t_s"]) - 1.0) < 0.006
    assert arg.main([str(p), "--friction-fit"]) == 0
    out = capsys.readouterr().out
    assert "segment 0 (the trajectory)" in out and "outcome completed" in out and "0 trajectory, 1 return" in out
    assert arg.main(["--compare", str(p), str(p), "--seg1-window", "1:20"]) == 0


def test_a_missing_summary_is_a_loud_warning_and_marks_the_output(tmp_path, capsys):
    p = tmp_path / "run.csv"
    write_friction_run(p, sgn_rad=-0.008, const_rad=0.0)
    assert arg.main([str(p), "--friction-fit"]) == 0
    cap = capsys.readouterr()
    assert "WARNING: no run summary run.json" in cap.err and "ASSUMING slow 1" in cap.err and "j4 40" in cap.err
    assert "NO RUN SUMMARY" in cap.out and "outcome unknown" in cap.out


def test_empty_cells_in_any_used_column_are_dropped_not_a_crash(tmp_path):
    p = tmp_path / "run.csv"
    write_run(p, lag_rad=0.0, torque_offset=0.0, ripple=0.0)
    rows = list(csv.reader(open(p, newline="")))
    hdr = rows[0]
    for k, col in ((10, "j4_pos"), (20, "j1_torq"), (30, "j6_ff"), (40, "j5_vel")):
        rows[k][hdr.index(col)] = ""
    with open(p, "w", newline="") as fh:
        csv.writer(fh).writerows(rows)
    assert len(arg.load(p)) == 4000 - 4
    assert arg.main([str(p), "--friction-fit"]) == 0


def test_compare_prints_ripple_to_a_hundredth_of_a_mrad(tmp_path, capsys):
    a, b = tmp_path / "a.csv", tmp_path / "b.csv"
    write_run(a, lag_rad=0.0, torque_offset=0.0, ripple=0.0005)
    write_run(b, lag_rad=0.0, torque_offset=0.0, ripple=0.0004)
    assert arg.main(["--compare", str(a), str(b)]) == 0
    out = capsys.readouterr().out
    assert " 0.35 ->  0.28 (-0.07)" in out

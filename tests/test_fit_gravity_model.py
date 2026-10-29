"""tools/fit_gravity_model.py + crackvision.gravity_model: a synthetic arm with known extra masses and friction.

The synthetic runs are written as demo_move.py run logs (the same CSV columns), with stationary holds reached from
both sides (torque = G + d * F_static) and slow moves (torque = G + F_c sign(v) + b v), so the whole path is tested:
loading, hold detection and pairing, the regressor, the ridge + Huber fit, the friction by-products, the file."""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from crackvision import gravity_model as gmod

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("fit_gravity_model", REPO / "tools" / "fit_gravity_model.py")
fg = importlib.util.module_from_spec(spec)
sys.modules["fit_gravity_model"] = fg
spec.loader.exec_module(fg)

UG = gmod.UrdfGravity()
TRUE_EXTRA = [("link4", (0.0, 0.0, 0.0), 0.30), ("link5", (0.01, 0.0, 0.03), 0.40), ("gripper_link", (-0.06, 0.0, 0.0), 0.25),
              ("gripper_link", fg.camera_xyz(), 0.10)]
F_STATIC = {"j2": 0.6, "j3": 0.5, "j4": 0.12, "j5": 0.04, "j6": 0.02}
F_COULOMB = {"j2": 0.9, "j3": 1.8, "j4": 0.15, "j5": 0.12, "j6": 0.08}


def mj(qa, qb, T, dt):
    s = np.arange(0, T, dt) / T
    return qa + (qb - qa) * (10 * s**3 - 15 * s**4 + 6 * s**5)[:, None]


def write_run(path: Path, poses: np.ndarray, rng, *, noise: float = 0.03, dt: float = 0.005) -> None:
    """fold -> for each pose: below -> pose (hold 1.5 s) -> above -> pose (hold 1.5 s) -> fold, as a run log."""
    sign = np.array([0, 1, 1, 1, 1, 1], dtype=float)
    ref = [np.zeros((int(1.5 / dt), 6))]
    cur = np.zeros(6)
    for p in poses:
        for d in (+1, -1):
            frm = p - d * 0.06 * sign
            ref += [mj(cur, frm, 2.0, dt), mj(frm, p, 1.0, dt), np.tile(p, (int(1.5 / dt), 1))]
            cur = p
    ref += [mj(cur, np.zeros(6), 3.0, dt), np.zeros((int(1.0 / dt), 6))]
    ref = np.vstack(ref)
    v = np.gradient(ref, dt, axis=0)
    G = UG.hold_torques(ref, TRUE_EXTRA)
    torq = G.copy()
    last = np.zeros(6)
    for i in range(len(ref)):
        for j in range(1, 6):
            k = f"j{j + 1}"
            if abs(v[i, j]) > 0.02:
                last[j] = np.sign(v[i, j])
                torq[i, j] += F_COULOMB[k] * np.sign(v[i, j]) + 0.2 * v[i, j]
            else:
                torq[i, j] += last[j] * F_STATIC[k]
    torq += rng.normal(0, noise, torq.shape)
    names = [f"j{i}" for i in range(1, 8)]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "mode", "seg"] + [f"{n}_{c}" for n in names for c in ("ref", "pos", "vel", "torq", "ff")])
        for i in range(len(ref)):
            row = [f"{i * dt:.4f}", "path", 1]
            for j in range(7):
                if j < 6:
                    row += [f"{ref[i, j]:+.5f}", f"{ref[i, j]:+.5f}", f"{v[i, j]:+.4f}", f"{torq[i, j]:+.3f}", "+0.000"]
                else:
                    row += ["0", "0", "0", "0", "0"]
            w.writerow(row)


def random_poses(rng, n):
    return np.column_stack([np.zeros(n), rng.uniform(-1.6, -0.3, n), rng.uniform(-1.8, -0.4, n), rng.uniform(0.0, 1.2, n),
                            rng.uniform(-0.6, 0.6, n), rng.uniform(-1.0, 1.0, n)])


@pytest.fixture(scope="module")
def synthetic(tmp_path_factory):
    rng = np.random.default_rng(3)
    d = tmp_path_factory.mktemp("runs")
    paths = []
    for k in range(3):
        p = d / f"traj_synthetic_{k}.csv"
        write_run(p, random_poses(rng, 8), rng)
        paths.append(p)
    data = fg.load_all(paths, log=lambda *a: None)
    return paths, data


def test_vectorised_gravity_matches_the_runners_gravity_py():
    # the runner's model (its evidence copy), loaded without writing bytecode into evidence/
    gspec = importlib.util.spec_from_file_location(
        "gravity_evidence_copy", REPO / "evidence" / "operator" / "controller-tuning-2026-10" / "code" / "gravity.py")
    gravity = importlib.util.module_from_spec(gspec)
    dont, sys.dont_write_bytecode = sys.dont_write_bytecode, True
    try:
        gspec.loader.exec_module(gravity)
    finally:
        sys.dont_write_bytecode = dont
    rng = np.random.default_rng(0)
    Q = rng.uniform(-1.5, 1.5, (20, 6))
    gm = gravity.GravityModel(urdf=UG.path, extra=TRUE_EXTRA)
    ref = np.array([[gm.hold_torques({f"j{i + 1}": q[i] for i in range(6)})[f"j{i + 1}"] for i in range(6)] for q in Q])
    assert np.abs(ref - UG.hold_torques(Q, TRUE_EXTRA)).max() < 1e-9
    assert UG.sha256 == gmod.file_sha256(UG.path)


def test_holds_are_detected_tagged_and_paired(synthetic):
    _, data = synthetic
    h = fg.concat_holds([v["holds"] for v in data.values()])
    # the 48 holds, plus the brief stops at the turn-around points above each pose (a real stop after moving up)
    assert 48 <= len(h.q) <= 72
    pairs, singles = fg.pair_holds(h)
    assert len(pairs[2]) == 24                                       # every pose gives an up/down pair on J3
    assert all(h.direction[a, 2] > 0 for a in singles[2])            # the unpaired ones are the turn-arounds
    assert set(np.unique(h.direction[:, 2])) == {-1.0, 1.0}


def test_known_extra_masses_are_recovered(synthetic):
    paths, data = synthetic
    names = list(data)
    for use_moving in (False, True):
        fit = fg.fit_on(data, names, UG, fg.default_bodies(), use_moving)
        q = random_poses(np.random.default_rng(11), 200)            # poses the fit never saw
        err = fit.predict(UG, q) - UG.hold_torques(q, TRUE_EXTRA)
        assert np.abs(err[:, 1:4]).max() < 0.12, (use_moving, np.abs(err).max(axis=0))
        assert np.sqrt((err[:, 1:4] ** 2).mean()) < 0.05
        assert "camera_mount.m" in fit.fixed                         # collinear with the gripper body: pinned to the prior
        assert fit.rank < len(fit.names)                             # some unknowns are not identifiable, as on the arm
    # a well-posed subset (one free body + the camera) is recovered parameter by parameter
    truth = [("link5", (0.02, 0.0, 0.05), 0.6)]
    rng = np.random.default_rng(5)
    hs = []
    for k in range(2):
        p = paths[0].parent / f"traj_one_{k}.csv"
        old = list(TRUE_EXTRA)
        TRUE_EXTRA[:] = truth
        try:
            write_run(p, random_poses(rng, 10), rng, noise=0.01)
        finally:
            TRUE_EXTRA[:] = old
        hs.append(p)
    d2 = fg.load_all(hs, log=lambda *a: None)
    fit = fg.solve(UG, [fg.Body("x", "link5", "free", kg_sigma=2.0)], fg.concat_holds([v["holds"] for v in d2.values()]),
                   auto_fix=False)
    m, mx, my, mz = fit.theta
    assert abs(m - 0.6) < 0.05 and abs(mx - 0.012) < 0.004 and abs(mz - 0.03) < 0.004


def test_friction_band_is_a_by_product_and_does_not_bias_the_fit(synthetic):
    _, data = synthetic
    names = list(data)
    fit = fg.fit_on(data, names, UG, fg.default_bodies(), True)
    band = fg.friction_band(UG, fg.concat_holds([data[n]["holds"] for n in names]), fit.extra)
    for k in ("j2", "j3", "j4"):
        assert band[k]["n_pairs"] > 10
        assert abs(band[k]["half_band_nm"] - F_STATIC[k]) < 0.05, (k, band[k])
    for k in ("j2", "j3"):
        assert abs(fit.friction[k]["coulomb_nm"] - F_COULOMB[k]) < 0.25, (k, fit.friction[k])
    # a hold-only fit with the pairs removed (singles biased by +-F_static) still lands within the band's noise
    h = fg.concat_holds([data[n]["holds"] for n in names])
    up = h.direction[:, 2] > 0
    one_side = fg.Holds(*[getattr(h, f)[up] for f in ("q", "tau", "direction", "run", "t", "dur", "mode")])
    f1 = fg.solve(UG, fg.default_bodies(), one_side)
    q = random_poses(np.random.default_rng(12), 100)
    err = f1.predict(UG, q) - UG.hold_torques(q, TRUE_EXTRA)
    assert np.abs(err[:, 2].mean()) < F_STATIC["j3"] + 0.1          # biased by at most about the band, never more


def test_model_file_roundtrip_and_urdf_sha256_check(tmp_path):
    doc = {"schema": gmod.SCHEMA, "urdf": {"sha256": UG.sha256}, "bodies": [{"link": "link5", "xyz": [0.01, 0.00001, 0.0],
                                                                             "kg": 0.4}],
           "scale": {"j2": 1.0, "j3": 1.0}, "residuals": {"tiny": 1e-07, "x": 0.123456}}
    p = tmp_path / "m.yaml"
    p.write_text(gmod.dump_model_text(doc, "a header\nline two"))
    text = p.read_text()
    assert text.startswith("# a header\n# line two\n{")
    import yaml
    assert yaml.safe_load(text) == json.loads("\n".join(l for l in text.splitlines() if not l.startswith("#")))
    back = gmod.load_model(p, urdf=UG.path)
    assert gmod.extra_bodies(back) == [("link5", (0.01, 0.0, 0.0), 0.4)]
    other = tmp_path / "other.urdf"
    other.write_text(UG.path.read_text().replace("reBot", "reBoT", 1))
    with pytest.raises(gmod.GravityModelError, match="URDF sha256"):
        gmod.load_model(p, urdf=other)
    for bad in ({"schema": "x"}, {"urdf": {}}, {"bodies": [{"link": "link5", "xyz": [0, 0, 5.0], "kg": 0.4}]},
                {"scale": {"j1": 1.0}}):
        q = tmp_path / "bad.yaml"
        q.write_text(gmod.dump_model_text(doc | bad, "bad"))
        with pytest.raises(gmod.GravityModelError):
            gmod.load_model(q)
    with pytest.raises(ValueError):
        gmod.dump_model_text(doc | {"big": 1e20}, "x")


def test_model_feed_forward_trims_friction_and_refuses_gravity_over_the_cap():
    grav = np.array([[0.0, 3.0, -8.3, -2.1, 0.0, 0.0], [0.0, 10.2, -1.0, 0.0, 0.0, 0.0]])
    fric = np.array([[0.0, 1.0, -2.0, -0.5, 0.0, 0.0], [0.0, 1.0, 2.0, 0.0, 0.0, 0.0]])
    ff, limited, refused = gmod.model_feed_forward(grav, fric)
    assert ff[0, 1] == 4.0 and not limited[0, 1]
    assert ff[0, 2] == pytest.approx(-9.95) and limited[0, 2]
    assert ff[0, 3] == pytest.approx(-2.45) and limited[0, 3]
    assert refused[1, 1] and not refused[1, 2] and ff[1, 2] == 1.0


def test_runner_samples_start_and_end_at_the_fold():
    doc = {"points": [{"t_s": 0.0, "positions": [0, -0.9, -1.2, 0.35, 0, 0]}, {"t_s": 5.0, "positions": [0, -1.0, -1.3, 0.4, 0, 0]}]}
    t, q, v = fg.runner_samples(doc)
    assert np.abs(q[0]).max() < 1e-9 and np.abs(q[-1]).max() < 0.01
    assert np.abs(v).max(axis=0)[1] <= 0.6 * 0.9 + 1e-6               # the approach is timed at 90 % of the caps


def test_cli_writes_a_loadable_model(tmp_path, synthetic):
    paths, _ = synthetic
    out = tmp_path / "gravity_model.yaml"
    rc = fg.main(["--show-runs", str(paths[0]), "--runs", *map(str, paths[1:]), "--write", str(out), "--moving", "no"])
    assert rc == 0
    doc = gmod.load_model(out, urdf=gmod.DEFAULT_URDF)
    assert doc["scale"] == {"j2": 1.0, "j3": 1.0, "j4": 1.0, "j5": 1.0, "j6": 1.0}
    assert doc["fit"]["runs"] and doc["valid_range"]["j2"][0] < -1.0
    assert "extrapolated" in doc and doc["urdf"]["sha256"] == UG.sha256
    assert set(doc["residuals"]["leave_one_run_out"]) == {"urdf", "scales", "scales+bias", "fit"}


# ---- audit 2026-10-29 item 6: split bodies stay inside the runner's limits; never write a file the runner refuses -----

def test_split_bodies_carry_the_exact_moment_within_half_a_metre():
    bodies = [fg.Body("a", "link4", "free"), fg.Body("b", "link5", "free"), fg.Body("c", "link6", "free")]
    #           near-massless, big moment     small mass, big moment         ordinary body
    theta = np.array([0.0, 0.06, 0.0, -0.02,   0.03, 0.04, 0.0, 0.0,     0.4, 0.004, 0.0, 0.012])
    extra = fg.bodies_to_extra(bodies, theta)
    for link, m_want, mr_want in (("link4", 0.0, [0.06, 0.0, -0.02]), ("link5", 0.03, [0.04, 0.0, 0.0]),
                                  ("link6", 0.4, [0.004, 0.0, 0.012])):
        pts = [(np.array(x), kg) for ln, x, kg in extra if ln == link]
        assert sum(kg for _, kg in pts) == pytest.approx(m_want, abs=1e-12)
        np.testing.assert_allclose(sum(kg * x for x, kg in pts), mr_want, atol=1e-12)    # exact first moment
        assert max(np.abs(x).max() for x, _ in pts) <= fg.SPLIT_MAX_XYZ_M + 1e-12
    assert len([1 for ln, *_ in extra if ln == "link6"]) == 1                            # not split
    # the moment that put the old 0.05 kg point 1.26 m out: now max(0.05, |mr|/0.5) = 0.126 kg at 0.5 m
    (_, xyz, kg), (_, origin, kg0) = fg.bodies_to_extra([fg.Body("a", "link4", "free")], np.array([0.0, 0.063, 0.0, 0.0]))
    assert kg == pytest.approx(0.126) and xyz == pytest.approx((0.5, 0.0, 0.0)) and kg0 == pytest.approx(-0.126)


def test_runner_problems_replicates_the_runners_loader():
    ok = {"schema": gmod.SCHEMA, "urdf": {"sha256": gmod.file_sha256(gmod.DEFAULT_URDF)},
          "bodies": [{"link": "link5", "xyz": [0.01, 0.0, 0.03], "kg": 0.4}], "scale": {"j2": 1.0, "j3": 1.0}}
    assert gmod.runner_problems(ok) == []
    cases = {"schema": {"schema": "x"}, "URDF": {"urdf": {"sha256": "0" * 64}}, "1..20": {"bodies": []},
             "<= 1 m": {"bodies": [{"link": "link5", "xyz": [1.26, 0.0, 0.0], "kg": 0.05}]},
             "kg in": {"bodies": [{"link": "link5", "xyz": [0.0, 0.0, 0.0], "kg": -0.6}]},
             "duplicate": {"bodies": [{"link": "link5", "xyz": [0, 0, 0], "kg": 0.1}] * 2},
             "moving link": {"bodies": [{"link": "base_link", "xyz": [0, 0, 0], "kg": 0.1}]},
             "> 5 kg": {"bodies": [{"link": f"link{i}", "xyz": [0, 0, 0], "kg": 2.0} for i in (3, 4, 5)]},
             "scale": {"scale": {"j1": 1.0}}, "bad body": {"bodies": [{"link": "link5", "xyz": [0, 0, True], "kg": 0.1}]}}
    for want, bad in cases.items():
        probs = gmod.runner_problems(ok | bad)
        assert probs and any(want in p for p in probs), (want, probs)
    assert gmod.runner_problems(ok | {"scale": {}}) and gmod.runner_problems(ok | {"scale": {"j2": 1.6}})


RUNNER = Path.home() / "rebot_setup"


@pytest.mark.skipif(not (RUNNER / "demo_move.py").exists(), reason="no ~/rebot_setup runner here")
def test_runner_problems_agrees_with_the_runners_own_loader(tmp_path):
    """The real load_gravity_model (in the runner's own interpreter, which has motorbridge) on the same files."""
    import subprocess
    py = Path.home() / "miniconda3" / "envs" / "rebot" / "bin" / "python"
    if not py.exists():
        pytest.skip("no rebot env")
    ok = {"schema": gmod.SCHEMA, "urdf": {"sha256": gmod.file_sha256(gmod.DEFAULT_URDF)},
          "bodies": [{"link": "link5", "xyz": [0.01, 0.0, 0.03], "kg": 0.4}], "scale": {"j2": 1.0, "j3": 1.0}}
    variants = [ok, ok | {"bodies": [{"link": "link5", "xyz": [1.26, 0.0, 0.0], "kg": 0.05}]},
                ok | {"bodies": [{"link": "base_link", "xyz": [0, 0, 0], "kg": 0.1}]}, ok | {"scale": {"j2": 1.6}},
                ok | {"bodies": [{"link": "link5", "xyz": [0, 0, 0], "kg": 0.1}] * 2}]
    files = []
    for k, doc in enumerate(variants):
        f = tmp_path / f"m{k}.yaml"
        f.write_text(gmod.dump_model_text(doc, "test"))
        files.append(str(f))
    code = ("import sys, demo_move as d\n"
            "for f in sys.argv[1:]:\n"
            "    try: d.load_gravity_model(f); print('ok')\n"
            "    except ValueError: print('refused')\n")
    env = {k: v for k, v in __import__("os").environ.items() if k != "PYTHONPATH"}
    out = subprocess.run([str(py), "-c", code, *files], cwd=RUNNER, env=env, capture_output=True, text=True,
                         timeout=120).stdout.split()
    assert out == ["ok" if not gmod.runner_problems(v) else "refused" for v in variants]


def test_cli_refuses_to_write_a_model_the_runner_would_refuse(tmp_path, synthetic, monkeypatch, capsys):
    paths, _ = synthetic
    real = fg.model_doc

    def bad_doc(*a, **k):
        doc = real(*a, **k)
        doc["bodies"].append({"link": "link5", "xyz": [1.26, 0.0, 0.0], "kg": 0.05})   # the audit's failure
        return doc
    monkeypatch.setattr(fg, "model_doc", bad_doc)
    out = tmp_path / "gravity_model.yaml"
    rc = fg.main(["--show-runs", str(paths[0]), "--runs", *map(str, paths[1:]), "--write", str(out), "--moving", "no"])
    assert rc == 1 and not out.exists()
    assert "would refuse" in capsys.readouterr().out

"""Offline tests for demo_move.py (fakebus, simulated time; no hardware)."""
from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import demo_move as d  # noqa: E402
import fakebus  # noqa: E402
import first_move_j1 as fm  # noqa: E402

TMP = tempfile.TemporaryDirectory(prefix="demo_tests_")


def run_sim(*argv, setup=None, choreo=None):
    logs: list[str] = []
    out = Path(tempfile.mkdtemp(dir=TMP.name))
    bus = fakebus.FakeBus()
    rt = fm.Runtime(clock=bus.clock, sleep=bus.clock.sleep, tty_reader=lambda p: None, out_dir=out, mb=bus.mod,
                    simulated=True, check_host=False, require_confirmation=False, install_signals=False,
                    log=logs.append)
    saved = dict(d.CHOREOS)
    if choreo is not None:
        d.CHOREOS["test"] = choreo
    try:
        rc = d.run(["--simulate", "--operator-go", "go", *argv] + (["--choreo", "test"] if choreo else []),
                   rt=rt, bus_factory=None, sim_setup=setup)
    finally:
        d.CHOREOS.clear()
        d.CHOREOS.update(saved)
    summary = json.loads(next(out.glob("*.json")).read_text())
    return rc, logs, summary


class Demo(unittest.TestCase):
    def test_wave_completes_and_everything_ends_disabled(self):
        rc, logs, s = run_sim()
        self.assertEqual(rc, fm.EXIT_OK, logs[-6:])
        self.assertEqual(s["outcome"], "completed")
        self.assertTrue(s["all_disabled_confirmed"])

    def test_too_fast_segment_is_refused_before_enable(self):
        rc, logs, s = run_sim(choreo=[(1.0, {"j3": -1.0}), (6.0, {"j3": d.S})])
        self.assertEqual(rc, fm.EXIT_REFUSED)
        self.assertTrue(any("peak" in p for p in s["refusal"]))

    def test_table_collision_is_refused_before_enable(self):
        rc, logs, s = run_sim(choreo=[(6.0, {"j2": -0.6}), (6.0, {"j2": d.S})])   # J2 alone dips the gripper
        self.assertEqual(rc, fm.EXIT_REFUSED)
        self.assertTrue(any("table" in p for p in s["refusal"]))

    def test_mid_path_fault_parks_back_to_start_then_disables(self):
        def bump(bus):   # j5 suddenly reads 0.4 rad off -> tracking abort while raised
            bus.faults.other_moves_at, bus.faults.other_moves_name = 8.0, "j5"
            bus.controller_t_enable_hack = True
        rc, logs, s = run_sim(setup=bump)
        self.assertEqual(rc, fm.EXIT_ABORTED)
        self.assertEqual(s["trigger"]["id"], "tracking")
        self.assertTrue(s["stats"]["parked"])
        self.assertTrue(s["all_disabled_confirmed"])
        self.assertTrue(any(l.startswith("PARK") for l in logs))

    def test_refuses_unless_starting_from_the_fold(self):
        def lifted(bus):
            bus.sims["j3"].pos = -0.3
        rc, logs, s = run_sim(setup=lifted)
        self.assertEqual(rc, fm.EXIT_REFUSED)
        self.assertIn("folded rest", s["refusal"])

    def test_motor_gains_are_locked(self):
        m = d.DemoMotor("j3", object())
        with self.assertRaises(PermissionError):
            m.send_mit(0.0, 0.0, 100.0, 1.0, 0.0)
        with self.assertRaises(PermissionError):
            m.write_register_u32(9, 0)


if __name__ == "__main__":
    unittest.main()


class WaitGo(unittest.TestCase):
    def test_goes_to_zero_holds_then_runs_on_go(self):
        go = Path(tempfile.mkdtemp(dir=TMP.name)) / "GO"

        def setup(bus):
            def drop_go(now):   # the "operator" says go 4 s after the zeroing finished
                c = bus.controller
                if c is not None and c.t_enable is not None and now - c.t_enable > 9.0 and not go.exists():
                    go.write_text("go")
            bus.clock.on_advance.append(drop_go)
        rc, logs, s = run_sim("--wait-go", str(go), setup=setup)
        self.assertEqual(rc, fm.EXIT_OK, logs[-5:])
        self.assertEqual(s["second_go"], "go")
        self.assertTrue(any("HOLDING; waiting" in l for l in logs))

    def test_no_go_times_out_and_disables(self):
        go = Path(tempfile.mkdtemp(dir=TMP.name)) / "GO"
        saved = d.MAX_WAIT_S
        d.MAX_WAIT_S = 2.0
        try:
            rc, logs, s = run_sim("--wait-go", str(go))
        finally:
            d.MAX_WAIT_S = saved
        self.assertEqual(s["outcome"], "no_go_timeout")
        self.assertTrue(s["all_disabled_confirmed"])

    def test_stale_go_file_is_cleared_before_enabling(self):
        go = Path(tempfile.mkdtemp(dir=TMP.name)) / "GO"
        go.write_text("old go")
        saved = d.MAX_WAIT_S
        d.MAX_WAIT_S = 1.0
        try:
            rc, logs, s = run_sim("--wait-go", str(go))
        finally:
            d.MAX_WAIT_S = saved
        self.assertEqual(s["outcome"], "no_go_timeout")   # the old file did not start the demo


class Show(unittest.TestCase):
    def test_show_turns_to_the_audience_and_completes(self):
        rc, logs, s = run_sim("--choreo", "show")
        self.assertEqual(rc, fm.EXIT_OK, logs[-5:])
        self.assertTrue(s["all_disabled_confirmed"])

    def test_show_can_start_already_facing_the_audience(self):
        def facing(bus):
            bus.sims["j1"].pos = d.AUDIENCE_J1 + 0.01
        rc, logs, s = run_sim("--choreo", "show", setup=facing)
        self.assertEqual(rc, fm.EXIT_OK, logs[-5:])

    def test_j1_beyond_the_demo_limit_is_still_refused(self):
        rc, logs, s = run_sim(choreo=[(20.0, {"j1": -4.2}), (20.0, {"j1": 0.0})])
        self.assertEqual(rc, fm.EXIT_REFUSED)


class Speed(unittest.TestCase):
    def test_show_at_1_5x_completes_with_scaled_aborts(self):
        rc, logs, s = run_sim("--choreo", "show", "--speed", "1.5")
        self.assertEqual(rc, fm.EXIT_OK, logs[-5:])
        self.assertAlmostEqual(s["vel_abort"]["4340P"], 1.5)
        self.assertTrue(all(v <= d.PLAN_VEL_MAX[fm.MODEL_OF[n]] * 1.5 + 1e-9 for n, v in s["planned_peak_rad_s"].items()))

    def test_speed_bounds(self):
        for bad in ("0.9", "2.5"):
            self.assertEqual(d.run(["--simulate", "--speed", bad]), fm.EXIT_USAGE, bad)


class J1Reference(unittest.TestCase):
    def test_j1_reading_becomes_zero_and_the_show_runs_relative_to_it(self):
        def powered_up_odd(bus):      # 2026-10-18: J1 read +3.03 after a power cycle, J2 drooped +0.40
            bus.sims["j1"].pos, bus.sims["j2"].pos = 3.0337, 0.40
        go = Path(tempfile.mkdtemp(dir=TMP.name)) / "GO"

        def setup(bus):
            powered_up_odd(bus)
            def drop_go(now):
                c = bus.controller
                if c is not None and c.t_enable is not None and now - c.t_enable > 8.0 and not go.exists():
                    go.write_text("go")
            bus.clock.on_advance.append(drop_go)
        rc, logs, s = run_sim("--choreo", "show", "--speed", "1.5", "--j1-here-is-zero", "--wait-go", str(go), setup=setup)
        self.assertEqual(rc, fm.EXIT_OK, logs[-6:])
        self.assertAlmostEqual(s["j1_offset"], 3.0337, places=3)
        rows = list(csv.DictReader(open(s["csv"])))
        j1 = [float(r["j1_ref"]) for r in rows]
        self.assertAlmostEqual(min(j1), 3.0337 + d.AUDIENCE_J1 - 0.1, delta=0.02)   # look-right, relative to the offset
        self.assertLess(abs(float(rows[-1]["j2_pos"])), 0.03)                      # J2 lifted out of the droop

    def test_without_the_flag_the_odd_reading_is_refused(self):
        def odd(bus):
            bus.sims["j1"].pos = 3.0337
        rc, logs, s = run_sim("--choreo", "show", setup=odd)
        self.assertEqual(rc, fm.EXIT_REFUSED)


class ZeroingNeverTurnsTheBase(unittest.TestCase):
    def test_with_the_j1_reference_the_zeroing_holds_j1_still(self):
        go = Path(tempfile.mkdtemp(dir=TMP.name)) / "GO"
        saved = d.MAX_WAIT_S
        d.MAX_WAIT_S = 1.0
        try:
            rc, logs, s = run_sim("--choreo", "show", "--j1-here-is-zero", "--wait-go", str(go),
                                  setup=lambda bus: setattr(bus.sims["j1"], "pos", 6.3354))
        finally:
            d.MAX_WAIT_S = saved
        rows = list(csv.DictReader(open(s["csv"])))
        j1 = [float(r["j1_ref"]) for r in rows]
        self.assertLess(max(j1) - min(j1), 1e-6)          # never commanded to move J1 before the second go
        self.assertEqual(s["outcome"], "no_go_timeout")


class J1OffsetReuse(unittest.TestCase):
    def test_starting_at_the_audience_with_a_known_offset_runs_at_2x(self):
        off = 6.3346
        go = Path(tempfile.mkdtemp(dir=TMP.name)) / "GO"

        def setup(bus):
            bus.sims["j1"].pos, bus.sims["j2"].pos = off + d.AUDIENCE_J1, 0.40
            def drop_go(now):
                c = bus.controller
                if c is not None and c.t_enable is not None and now - c.t_enable > 8.0 and not go.exists():
                    go.write_text("go")
            bus.clock.on_advance.append(drop_go)
        rc, logs, s = run_sim("--choreo", "show", "--speed", "2", "--j1-offset", str(off), "--wait-go", str(go), setup=setup)
        self.assertEqual(rc, fm.EXIT_OK, logs[-6:])
        rows = list(csv.DictReader(open(s["csv"])))
        j1 = [float(r["j1_ref"]) for r in rows]
        self.assertAlmostEqual(min(j1), off + d.AUDIENCE_J1 - 0.1, delta=0.02)   # never turned away from the audience
        self.assertLess(max(j1), off + d.AUDIENCE_J1 + 0.45)

    def test_wrong_offset_is_refused(self):
        rc, logs, s = run_sim("--choreo", "show", "--j1-offset", "1.0", setup=lambda b: setattr(b.sims["j1"], "pos", 2.57))
        self.assertEqual(rc, fm.EXIT_REFUSED)


class J1RefFile(unittest.TestCase):
    def test_saved_reference_reused_and_unknown_position_refused(self):
        ref = Path(tempfile.mkdtemp(dir=TMP.name)) / "j1_ref.json"
        # 1) reference at power-up: forward reads 6.3346 -> saved
        rc, logs, s = run_sim("--choreo", "show", "--speed", "2", "--j1-here-is-zero", "--j1-ref-file", str(ref),
                              setup=lambda b: setattr(b.sims["j1"], "pos", 6.3346))
        self.assertEqual(rc, fm.EXIT_OK, logs[-4:])
        self.assertAlmostEqual(json.loads(ref.read_text())["offset"], 6.3346, places=3)
        # 2) next run starting facing the audience: reused, show completes without turning away
        rc, logs, s = run_sim("--choreo", "show", "--speed", "2", "--j1-ref-file", str(ref),
                              setup=lambda b: setattr(b.sims["j1"], "pos", 6.3346 + d.AUDIENCE_J1))
        self.assertEqual(rc, fm.EXIT_OK, logs[-4:])
        # 3) after a "power cycle" J1 reads something else: refused before enabling
        rc, logs, s = run_sim("--choreo", "show", "--j1-ref-file", str(ref), setup=lambda b: setattr(b.sims["j1"], "pos", 0.02))
        self.assertEqual(rc, d.EXIT_J1_UNKNOWN)
        self.assertFalse(any("ENABLED" in l for l in logs))

    def test_missing_reference_file_is_refused(self):
        ref = Path(tempfile.mkdtemp(dir=TMP.name)) / "none.json"
        rc, logs, s = run_sim("--choreo", "show", "--j1-ref-file", str(ref))
        self.assertEqual(rc, d.EXIT_J1_UNKNOWN)


# ---- --gravity-model (2026-10-29) ---------------------------------------------------------------------------------
import hashlib  # noqa: E402
import importlib.util  # noqa: E402
import math  # noqa: E402

HERE = Path(__file__).resolve().parent
HEART = HERE / "trajectories" / "fancy" / "heart.json"
SMOKE = HERE / "trajectories" / "smoke_raise.json"
PRE_CHANGE = HERE / "backup_pre_gravity_model_2026-10-29" / "demo_move.py"
REPO_MODEL = Path.home() / "Projects" / "rebot_crack_vision" / "config" / "robot" / "gravity_model.yaml"
URDF_SHA = hashlib.sha256(d.gravity.URDF.read_bytes()).hexdigest()
# a model shaped like the 2026-10-29 fit: ~2 kg on the wrist links + the camera
BODIES = [{"link": "link3", "xyz": [-0.044, -0.37, 0.0], "kg": 0.25}, {"link": "link4", "xyz": [0.002, -0.013, 0.0], "kg": 0.47},
          {"link": "link5", "xyz": [-0.024, -0.011, -0.012], "kg": 0.50}, {"link": "link6", "xyz": [-0.003, 0.002, -0.027], "kg": 0.45},
          {"link": "gripper_link", "xyz": [-0.048, 0.004, 0.006], "kg": 0.26}, {"link": "gripper_link", "xyz": [-0.0887, 0.0, -0.0627], "kg": 0.1}]


def read_rows(path: str) -> list[dict]:
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def model_file(**over) -> str:
    doc = {"schema": "crackvision.gravity_model/1", "urdf": {"sha256": URDF_SHA}, "bodies": BODIES,
           "scale": {n: 1.0 for n in ("j2", "j3", "j4", "j5", "j6")}, "created_utc": "2026-10-29T00:00:00Z",
           "extrapolated": True} | over
    p = Path(tempfile.mkdtemp(dir=TMP.name)) / "gravity_model.yaml"
    p.write_text("# a JSON-style YAML file, as tools/fit_gravity_model.py writes it\n" + json.dumps(doc, indent=1) + "\n")
    return str(p)


def guarded(module, *argv, capture=None):
    """module.run like run-trajectory.sh --rehearse: --simulate --guard-sim, --wait-go + --sim-auto-go 2."""
    logs: list[str] = []
    out = Path(tempfile.mkdtemp(dir=TMP.name))
    bus = fakebus.FakeBus()
    rt = fm.Runtime(clock=bus.clock, sleep=bus.clock.sleep, tty_reader=lambda p: None, out_dir=out, mb=bus.mod,
                    simulated=True, check_host=False, require_confirmation=False, install_signals=False,
                    log=logs.append)

    def setup(b):
        if capture is not None:
            capture.append(b)
    rc = module.run(["--simulate", "--guard-sim", "--operator-go", "rehearsal (test)", *argv, "--wait-go", str(out / "GO"),
                     "--sim-auto-go", "2"], rt=rt, bus_factory=None, sim_setup=setup)
    summary = json.loads(next(out.glob("*.json")).read_text())
    return rc, logs, summary


class GravityModelFlag(unittest.TestCase):
    def test_loader_reads_a_model_and_the_repo_file(self):
        m = d.load_gravity_model(model_file())
        self.assertEqual(m["urdf_sha256"], URDF_SHA)
        self.assertEqual(len(m["extra"]), 6)
        self.assertEqual(m["scale"]["j3"], 1.0)
        self.assertEqual(len(m["sha256"]), 64)
        if REPO_MODEL.exists():                         # the fitted file from tools/fit_gravity_model.py
            r = d.load_gravity_model(str(REPO_MODEL))
            self.assertEqual(r["sha256"], hashlib.sha256(REPO_MODEL.read_bytes()).hexdigest())

    def test_urdf_sha_mismatch_and_bad_files_are_refused_before_anything(self):
        bad = [model_file(urdf={"sha256": "0" * 64}), model_file(schema="crackvision.gravity_model/2"),
               model_file(bodies=[{"link": "link9", "xyz": [0, 0, 0], "kg": 0.3}]),
               model_file(bodies=[{"link": "link5", "xyz": [0, 0, 2.0], "kg": 0.3}]),
               model_file(bodies=[{"link": "link5", "xyz": [0, 0, 0], "kg": 3.0}]),
               model_file(bodies=[{"link": "link5", "xyz": [0, 0, float("nan")], "kg": 0.3}]),
               model_file(scale={"j3": 2.5}), model_file(scale={"j1": 1.0}), model_file(scale={})]
        for f in bad:
            with self.assertRaises(ValueError, msg=f):
                d.load_gravity_model(f)
        with self.assertRaisesRegex(ValueError, "URDF sha256"):
            d.load_gravity_model(bad[0])
        bus = fakebus.FakeBus()
        rt = fm.Runtime(clock=bus.clock, sleep=bus.clock.sleep, tty_reader=lambda p: None,
                        out_dir=Path(tempfile.mkdtemp(dir=TMP.name)), mb=bus.mod, simulated=True, check_host=False,
                        require_confirmation=False, install_signals=False, log=lambda m: None)
        rc = d.run(["--simulate", "--operator-go", "go", "--gravity-model", bad[0]], rt=rt)
        self.assertEqual(rc, fm.EXIT_USAGE)              # refused at the argument check: the bus is never opened
        self.assertEqual(bus.calls, [])

    def test_feed_forward_with_the_model(self):
        m = d.load_gravity_model(model_file())
        gm, plain = d.gravity.GravityModel(extra=tuple(m["extra"])), d.gravity.GravityModel()
        q = {"j1": 0.0, "j2": -0.9, "j3": -1.2, "j4": 0.35, "j5": 0.0, "j6": 0.0, "j7": 0.0}
        saved = dict(d.GRAV_SCALE)
        try:
            d.GM_PLAIN = plain
            d.GM_COVERED.update(m["scale"])
            d.GRAV_SCALE.update(m["scale"])
            g, g0 = gm.hold_torques(q), plain.hold_torques(q)
            full, start = d.feed_forward(gm, q, {}, 1.0), d.feed_forward(gm, q, {}, 0.0)
            for n in ("j2", "j3", "j4"):
                self.assertAlmostEqual(full[n], g[n], places=9)     # scale 1.0 x the model once faded in
                self.assertAlmostEqual(start[n], g0[n], places=9)   # at enable: the plain URDF only
            self.assertEqual(full["j7"], 0.0)
            moving = d.feed_forward(gm, q, {"j4": 0.5}, 1.0)        # friction on the commanded velocity, as today
            self.assertAlmostEqual(moving["j4"] - full["j4"], d.FRICTION["j4"] * math.tanh(0.5 / fm.FRICTION_VEL_EPS))
            # near the fold J3 needs ~8 Nm: gravity + the 2 Nm Coulomb term would pass the 10 Nm cap -> trimmed
            fold = {n: 0.0 for n in d.NAMES}
            lift = d.feed_forward(gm, fold, {"j3": -0.2}, 1.0)
            self.assertLessEqual(abs(lift["j3"]), 10.0 - d.FF_LIMIT_MARGIN_NM + 1e-9)
            self.assertGreater(abs(lift["j3"]), abs(gm.hold_torques(fold)["j3"]))
            # gravity alone over the cap is not trimmed: the cap check still stops it
            heavy = d.gravity.GravityModel(extra=(("gripper_link", (-0.1, 0.0, 0.0), 2.0), ("link5", (0.0, 0.0, 0.0), 2.0)))
            self.assertGreater(abs(d.feed_forward(heavy, fold, {}, 1.0)["j3"]), 10.0)
        finally:
            d.GM_PLAIN = None
            d.GM_COVERED.clear()
            d.GRAV_SCALE.clear()
            d.GRAV_SCALE.update(saved)

    def test_run_applies_the_scales_records_the_file_and_restores_the_defaults(self):
        f = model_file()
        rc, logs, s = guarded(d, "--trajectory", str(SMOKE), "--gravity-model", f)
        self.assertEqual(rc, fm.EXIT_OK, logs[-8:])
        self.assertEqual(s["gravity_model"]["sha256"], hashlib.sha256(Path(f).read_bytes()).hexdigest())
        self.assertEqual(s["gravity_model"]["urdf_sha256"], URDF_SHA)
        self.assertEqual({n: s["gravity_scale"][n] for n in ("j2", "j3", "j4")}, {"j2": 1.0, "j3": 1.0, "j4": 1.0})
        self.assertTrue(any(line.startswith("gravity model for this run") for line in logs))
        self.assertIsNone(d.GM_PLAIN)                                # per run: the next run is the default again
        self.assertEqual(d.GRAV_SCALE["j3"], fm.TUNED["j3"][2])
        self.assertFalse(d.GM_COVERED)
        rows = read_rows(s["csv"])
        hold = [r for r in rows if r["mode"] == "wait_go"]
        gm = d.gravity.GravityModel(extra=tuple(d.load_gravity_model(f)["extra"]))
        q = {n: float(hold[-1][f"{n}_pos"]) for n in d.NAMES}
        self.assertAlmostEqual(float(hold[-1]["j3_ff"]), gm.hold_torques(q)["j3"], places=2)
        rc0, _, s0 = guarded(d, "--trajectory", str(SMOKE))
        self.assertIsNone(s0["gravity_model"])
        self.assertEqual(s0["gravity_scale"]["j3"], 2.5)

    def test_heart_rehearsal_with_the_model_completes_under_the_guard(self):
        """What run-trajectory.sh --rehearse heart.json would do with DEMO_EXTRA="--gravity-model FILE" (the wrapper's
        rehearse branch does not pass DEMO_EXTRA, so the flag is exercised here with the identical arguments)."""
        f = str(REPO_MODEL) if REPO_MODEL.exists() else model_file()
        rc, logs, s = guarded(d, "--trajectory", str(HEART), "--slow", "1", "--gravity-model", f)
        self.assertEqual(rc, fm.EXIT_OK, logs[-8:])
        self.assertEqual(s["outcome"], "completed")
        self.assertFalse([e for e in s["guard"]["events"] if e["kind"] == "fault"])
        self.assertFalse(s["guard_preflight"]["problems"])
        rows = read_rows(s["csv"])
        self.assertLessEqual(max(abs(float(r["j3_ff"])) for r in rows), 10.0 - d.FF_LIMIT_MARGIN_NM + 1e-6)

    @unittest.skipUnless(PRE_CHANGE.exists(), "no pre-change copy of demo_move.py")
    def test_without_the_flag_the_command_stream_is_byte_identical_to_the_pre_change_runner(self):
        spec = importlib.util.spec_from_file_location("demo_move_pre_gravity_model", PRE_CHANGE)
        pre = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(pre)
        streams = []
        for module in (pre, d):
            buses: list = []
            rc, logs, s = guarded(module, "--trajectory", str(SMOKE), capture=buses)
            self.assertEqual(rc, fm.EXIT_OK, logs[-6:])
            streams.append([c for c in buses[0].calls if c[1] in ("send_mit", "enable", "disable")])
        self.assertGreater(len(streams[0]), 10000)
        self.assertEqual(streams[0], streams[1])        # every enable, MIT command (pos, vel, kp, kd, tau), disable


# ---- --friction (2026-10-29.2) -------------------------------------------------------------------------------------
PRE_FRICTION = HERE / "backup_pre_j4_friction_2026-10-29" / "demo_move.py"
J4_TUNE = HERE / "trajectories" / "j4_tune.json"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class FrictionFlag(unittest.TestCase):
    def test_parse_and_bounds(self):
        self.assertEqual(d.parse_friction("j4=0.18"), {"j4": 0.18})
        self.assertEqual(d.parse_friction("j4=0.18, j5=0.12"), {"j4": 0.18, "j5": 0.12})
        self.assertEqual(d.parse_friction(f"j4=0,j1={fm.FRICTION_MAX_NM}"), {"j4": 0.0, "j1": fm.FRICTION_MAX_NM})
        for bad in ("j4=3.01", "j4=-0.1", "j7=0.1", "j9=0.1", "j4", "j4=nan", "j4=inf", "j4=0.1/2", "=0.2"):
            with self.assertRaises(ValueError, msg=bad):
                d.parse_friction(bad)

    def test_bad_value_is_a_usage_error_before_the_bus_is_opened(self):
        for bad in ("j4=3.5", "j7=0.1", "j4=-1"):
            bus = fakebus.FakeBus()
            rt = fm.Runtime(clock=bus.clock, sleep=bus.clock.sleep, tty_reader=lambda p: None,
                            out_dir=Path(tempfile.mkdtemp(dir=TMP.name)), mb=bus.mod, simulated=True, check_host=False,
                            require_confirmation=False, install_signals=False, log=lambda m: None)
            self.assertEqual(d.run(["--simulate", "--operator-go", "go", "--friction", bad], rt=rt), fm.EXIT_USAGE, bad)
            self.assertEqual(bus.calls, [])

    def test_feed_forward_uses_the_override_with_and_without_the_gravity_model(self):
        q = {"j1": 0.0, "j2": -0.9, "j3": -1.2, "j4": 0.35, "j5": 0.0, "j6": 0.0, "j7": 0.0}
        gm = d.gravity.GravityModel()
        saved = dict(d.FRICTION)
        try:
            base = d.feed_forward(gm, q, {"j4": 0.1}, 1.0)
            d.FRICTION["j4"] = 0.18
            new = d.feed_forward(gm, q, {"j4": 0.1}, 1.0)
            self.assertAlmostEqual(new["j4"] - base["j4"], (0.18 - saved["j4"]) * math.tanh(0.1 / fm.FRICTION_VEL_EPS))
            self.assertEqual({n: new[n] for n in d.NAMES if n != "j4"}, {n: base[n] for n in d.NAMES if n != "j4"})
            m = d.load_gravity_model(model_file())
            d.GM_PLAIN = d.gravity.GravityModel()
            d.GM_COVERED.update(m["scale"])
            d.GRAV_SCALE.update(m["scale"])
            gmm = d.gravity.GravityModel(extra=tuple(m["extra"]))
            still, moving = d.feed_forward(gmm, q, {}, 1.0), d.feed_forward(gmm, q, {"j4": 0.1}, 1.0)
            self.assertAlmostEqual(moving["j4"] - still["j4"], 0.18 * math.tanh(0.1 / fm.FRICTION_VEL_EPS))
        finally:
            d.FRICTION.clear()
            d.FRICTION.update(saved)
            d.GM_PLAIN = None
            d.GM_COVERED.clear()
            d.GRAV_SCALE.clear()
            d.GRAV_SCALE.update({n: fm.TUNED[n][2] for n in fm.TUNED} | {"j7": 0.0})

    def test_run_applies_logs_records_and_restores_the_override(self):
        rc, logs, s = guarded(d, "--trajectory", str(HEART), "--friction", "j4=0.18")
        self.assertEqual(rc, fm.EXIT_OK, logs[-8:])
        self.assertEqual(s["friction_override"], {"j4": 0.18})
        self.assertEqual(s["friction"]["j4"], 0.18)
        self.assertEqual(s["friction"]["j3"], fm.TUNED["j3"][3])
        self.assertTrue(any(line.startswith("Coulomb friction feed-forward for this run: j4 0.18 Nm (tuned 0.5)")
                            for line in logs))
        self.assertEqual(d.FRICTION, {n: fm.TUNED[n][3] for n in fm.TUNED} | {"j7": 0.0})   # restored
        # the logged J4 feed-forward = 2.5 x URDF gravity at the measured pose + 0.18 x tanh(v_ref / eps)
        gm = d.gravity.GravityModel()
        rows = [r for r in read_rows(s["csv"]) if r["mode"] == "path"]
        fric = [float(r["j4_ff"]) - d.GRAV_SCALE["j4"] * gm.hold_torques({n: float(r[f"{n}_pos"]) for n in d.NAMES})["j4"]
                for r in rows]
        self.assertAlmostEqual(max(abs(f) for f in fric), 0.18, delta=0.02)
        rc0, _, s0 = guarded(d, "--trajectory", str(HEART))
        self.assertEqual(s0["friction_override"], {})
        self.assertEqual(s0["friction"]["j4"], fm.TUNED["j4"][3])

    def test_without_the_flag_the_command_stream_is_byte_identical_to_both_pre_change_runners(self):
        copies = [p for p in (PRE_FRICTION, PRE_CHANGE) if p.exists()]
        if not copies:
            self.skipTest("no pre-change copies of demo_move.py")
        streams = []
        for k, module in enumerate([d] + [load_module(p, f"demo_move_pre_{k}") for k, p in enumerate(copies)]):
            buses: list = []
            rc, logs, s = guarded(module, "--trajectory", str(SMOKE), capture=buses)
            self.assertEqual(rc, fm.EXIT_OK, logs[-6:])
            streams.append([c for c in buses[0].calls if c[1] in ("send_mit", "enable", "disable")])
        self.assertGreater(len(streams[0]), 10000)
        for other in streams[1:]:
            self.assertEqual(streams[0], other)

    @unittest.skipUnless(J4_TUNE.exists(), "no trajectories/j4_tune.json (tools/make_j4_tuning_trajectory.py)")
    def test_j4_tune_rehearsals_with_the_ab_overrides_pass_the_guard(self):
        """Runs C and D of docs/motion/J4_TUNING.md §6 (B = C without the gains), in-process with the arguments of
        run-trajectory.sh --rehearse (whose rehearse branch does not pass DEMO_EXTRA)."""
        for extra in (["--friction", "j4=0.18", "--gains", "j4=40/3"], ["--friction", "j4=0.18", "--gains", "j4=50/3"],
                      ["--gains", "j4=50/3"]):
            rc, logs, s = guarded(d, "--trajectory", str(J4_TUNE), "--slow", "1", *extra)
            self.assertEqual(rc, fm.EXIT_OK, (extra, logs[-8:]))
            self.assertEqual(s["outcome"], "completed")
            self.assertFalse(s["guard_preflight"]["problems"], extra)
            self.assertFalse([e for e in s["guard"]["events"] if e["kind"] == "fault"], extra)
            self.assertEqual(tuple(s["gains"]["j4"]), (40.0, 3.0) if "j4=40/3" in extra else (50.0, 3.0))


# ---- audit fixes (2026-10-29.3) -------------------------------------------------------------------------------------
DEMO_TRACE = Path.home() / "Projects" / "rebot_crack_vision" / "data" / "crack_trace" / "demo_20261029T013007Z_trace.json"


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += max(dt, 1e-4)


def _fake_rt(clock):
    return fm.Runtime(clock=clock, sleep=clock.sleep, tty_reader=lambda p: None, out_dir=Path(TMP.name), mb=None,
                      simulated=False, check_host=False, require_confirmation=False, install_signals=False,
                      log=lambda s: None)


class AuditFixes(unittest.TestCase):
    def test_summary_records_the_segment_kinds_and_a_fold_to_fold_file_is_segment_0(self):
        """--j1-here-is-zero on a file that starts and ends at the fold: no approach, no return; path_kinds says so."""
        doc = {"schema": d.TRAJ_SCHEMA, "joint_names": d.TRAJ_JOINTS, "purpose": "test",
               "points": [{"t_s": 0.0, "positions": [0.0] * 6}, {"t_s": 3.0, "positions": [0.0, -0.4, -0.6, 0.1, 0.0, 0.0]},
                          {"t_s": 6.0, "positions": [0.0] * 6}]}
        f = Path(tempfile.mkdtemp(dir=TMP.name)) / "fold_to_fold.json"
        f.write_text(json.dumps(doc))
        rc, logs, s = guarded(d, "--trajectory", str(f), "--j1-here-is-zero")
        self.assertEqual(rc, fm.EXIT_OK, logs[-6:])
        self.assertEqual(s["path_kinds"], [["trajectory", 6.0]])
        self.assertEqual({r["seg"] for r in read_rows(s["csv"]) if r["mode"] == "path"}, {"0"})
        rc, logs, s = guarded(d, "--trajectory", str(SMOKE))
        self.assertEqual([k for k, _ in s["path_kinds"]], ["approach", "trajectory", "return"])

    def test_gravity_sag_check_names_the_joint_and_the_margin(self):
        class P:
            duration = 0.2

            def at(self, t):
                q = {n: 0.0 for n in d.NAMES} | {"j2": -1.0}
                return 0, q, {n: 0.0 for n in d.NAMES}
        saved = d.feed_forward
        try:
            for g0, bad in ((9.9, True), (9.0, False)):
                d.feed_forward = lambda gm, q, v, fade, bias=0.0, g0=g0: {n: 0.0 for n in d.NAMES} | {"j2": g0 - 3.0 * (q["j2"] + 1.0)}
                problems, worst = d.gravity_sag_check(P(), None)
                if bad:
                    self.assertEqual(len(problems), 1)
                    self.assertIn("j2 feed-forward gravity reaches 10.05 Nm with j2 -0.05", problems[0])
                    self.assertIn("margin -0.05 Nm", problems[0])
                else:
                    self.assertEqual(problems, [])
                    self.assertAlmostEqual(worst["j2"]["nm"], 9.15)
        finally:
            d.feed_forward = saved

    @unittest.skipUnless(REPO_MODEL.exists() and DEMO_TRACE.exists(), "no repo gravity model / crack demo trace")
    def test_the_crack_demo_trace_with_the_model_is_refused_up_front(self):
        rc, logs, s = guarded(d, "--trajectory", str(DEMO_TRACE), "--gravity-model", str(REPO_MODEL))
        self.assertEqual(rc, fm.EXIT_REFUSED, logs[-6:])
        self.assertEqual(s["outcome"], "refused")
        self.assertTrue(any("gravity margin: j2" in x for x in s["refusal"]), s["refusal"])
        self.assertFalse([c for c in s.get("guard", {}).get("events", []) if c.get("kind") == "fault"])

    def test_model_trim_also_applies_when_gravity_alone_is_inside_the_margin(self):
        saved = (d.GM_PLAIN, dict(d.GM_COVERED), dict(d.GRAV_SCALE))

        class G:
            def __init__(self, g):
                self.g = g

            def hold_torques(self, q):
                return dict(self.g)
        try:
            d.GM_PLAIN = G({"j2": 9.97})
            d.GM_COVERED.clear()
            d.GM_COVERED.update({"j2": 1.0})
            d.GRAV_SCALE["j2"] = 1.0
            ff = d.model_feed_forward(G({"j2": 9.97}), {}, {"j2": 0.5}, 1.0)
            self.assertAlmostEqual(ff["j2"], 10.0 - d.FF_LIMIT_MARGIN_NM)      # was 9.97 + 1.0 = 10.97 -> abort
            ff = d.model_feed_forward(G({"j2": 10.2}), {}, {"j2": 0.5}, 1.0)
            self.assertGreater(ff["j2"], 10.0)                                  # gravity over the cap: still aborts
        finally:
            d.GM_PLAIN, _, _ = saved
            d.GM_COVERED.clear()
            d.GM_COVERED.update(saved[1])
            d.GRAV_SCALE.clear()
            d.GRAV_SCALE.update(saved[2])

    def _ff_for(self):
        def ff_for(q, v_ref, fade, bias=0.0, clamp=False):
            out = {n: 0.0 for n in d.NAMES} | {"j2": 10.4, "j3": -10.2, "j4": 2.7}
            if clamp:
                return d.clamp_ff(out)
            raise d.Abort("gravity", "j2 feed-forward +10.40 Nm > 10.0", True)
        return ff_for

    def test_park_clamps_the_feed_forward_at_the_cap_instead_of_zeroing_it(self):
        clock = _Clock()
        sent, logs = [], []

        def cycle(q_ref, v_ref, ff, mode, seg, t, lim, t_prev):
            sent.append(dict(ff))
        q = {n: 0.0 for n in d.NAMES} | {"j2": -1.2, "j3": -1.1}
        out = d.park(_fake_rt(clock), None, q, {n: 0.0 for n in d.NAMES}, self._ff_for(), cycle, {}, {"n": 0}, logs.append)
        self.assertEqual(out, "aborted_parked")
        self.assertTrue(sent)
        for ff in sent:
            self.assertAlmostEqual(ff["j2"], 10.0 - d.FF_LIMIT_MARGIN_NM)
            self.assertAlmostEqual(ff["j3"], -(10.0 - d.FF_LIMIT_MARGIN_NM))
            self.assertAlmostEqual(ff["j4"], 2.5 - d.FF_LIMIT_MARGIN_NM)
        self.assertTrue(any("clamped" in x and "not zeroed" in x for x in logs))

    def test_guard_hold_clamps_target_and_feed_forward_and_survives_a_refused_command(self):
        clock = _Clock()
        rt = _fake_rt(clock)
        holds, calls = [], {"n": 0}

        def cycle(q_ref, v_ref, ff, mode, seg, t, lim, t_prev):
            calls["n"] += 1
            if mode == "guard_hold":
                holds.append((dict(q_ref), dict(ff)))
                if calls["n"] == 1:
                    raise PermissionError("j3: target outside URDF range")   # was unhandled -> motors disabled
                if calls["n"] == 3:
                    rt.stop_flag["why"] = "operator"
        q = {n: 0.0 for n in d.NAMES} | {"j2": -1.2, "j3": 0.12}           # J3 sagged past its 0.0 stop
        logs = []
        saved_guard = d.GUARD
        d.GUARD = None
        try:
            out = d.guard_recover(rt, None, q, {n: 0.0 for n in d.NAMES}, self._ff_for(), cycle, {}, {"n": 0}, logs.append)
        finally:
            d.GUARD = saved_guard
        self.assertEqual(out, "aborted_parked")
        self.assertEqual(holds[0][0]["j3"], d.limits("j3")[1])
        self.assertAlmostEqual(holds[0][1]["j2"], 10.0 - d.FF_LIMIT_MARGIN_NM)
        self.assertTrue(any("refused" in x and "still holding" in x for x in logs))


# ---- audit-2 fixes (2026-10-29.4) -----------------------------------------------------------------------------------
import os  # noqa: E402
import signal  # noqa: E402


class AuditFixes2(unittest.TestCase):
    def test_ctrl_c_during_the_bus_precheck_refuses_before_anything_is_enabled(self):
        """run() catches SIGINT from its first line (run-trajectory.sh starts it in the background, where SIGINT was
        ignored until control() installed its handlers): a Ctrl-C during the pre-check = refuse, nothing enabled."""
        logs: list[str] = []
        out = Path(tempfile.mkdtemp(dir=TMP.name))
        bus = fakebus.FakeBus()
        rt = fm.Runtime(clock=bus.clock, sleep=bus.clock.sleep, tty_reader=lambda p: None, out_dir=out, mb=bus.mod,
                        simulated=True, check_host=False, require_confirmation=False, install_signals=True,
                        log=logs.append)
        before = signal.getsignal(signal.SIGINT)
        buses = []

        def setup(b):                    # runs right before the bus is opened and pre-checked
            buses.append(b)
            os.kill(os.getpid(), signal.SIGINT)
        rc = d.run(["--simulate", "--operator-go", "go"], rt=rt, bus_factory=None, sim_setup=setup)
        s = json.loads(next(out.glob("*.json")).read_text())
        self.assertEqual(rc, fm.EXIT_REFUSED, logs[-4:])
        self.assertEqual(s["outcome"], "refused")
        self.assertIn("Ctrl-C", s["refusal"])
        self.assertTrue(any("REFUSED (nothing was enabled)" in x and "Ctrl-C" in x for x in logs))
        self.assertFalse([c for c in buses[0].calls if c[1] in ("enable", "send_mit")])
        self.assertIs(signal.getsignal(signal.SIGINT), before)          # the caller's handler is back

    def test_a_ctrl_c_flag_left_before_control_refuses_there_too(self):
        """A Ctrl-C between the last check in _run and control(): control() refuses before enabling."""
        saved = d.EARLY_STOP["n"]
        logs, summary = [], {}
        try:
            d.EARLY_STOP["n"] = 1
            rc = d.control(argparse_ns(), _fake_rt_log(logs), None, None, None,
                           {n: 0.0 for n in d.NAMES}, None, Path(tempfile.mkdtemp(dir=TMP.name)) / "x", summary)
        finally:
            d.EARLY_STOP["n"] = saved
        self.assertEqual(rc, fm.EXIT_REFUSED)
        self.assertEqual(summary["outcome"], "refused")

    def _ff_real(self):
        gm = d.gravity.GravityModel()
        return gm, (lambda q, v_ref, fade, bias=0.0, clamp=False: d.feed_forward(gm, q, v_ref, fade, bias))

    def test_park_uses_the_fade_it_is_given(self):
        clock = _Clock()
        sent = []

        def cycle(q_ref, v_ref, ff, mode, seg, t, lim, t_prev):
            sent.append((dict(v_ref), dict(ff)))
        gm, ff_for = self._ff_real()
        q = {n: 0.0 for n in d.NAMES} | {"j2": -0.3, "j3": -0.6}
        out = d.park(_fake_rt(clock), None, q, {n: 0.0 for n in d.NAMES}, ff_for, cycle, {}, {"n": 0}, lambda s: None,
                     fade=0.2)
        self.assertEqual(out, "aborted_parked")
        v0, ff0 = sent[0]
        self.assertAlmostEqual(ff0["j3"], d.feed_forward(gm, q, v0, 0.2)["j3"], delta=1e-6)
        self.assertGreater(abs(d.feed_forward(gm, q, v0, 1.0)["j3"] - ff0["j3"]), 0.5)   # the kick that is avoided

    def test_abort_0_2_s_after_enable_parks_with_the_faded_feed_forward(self):
        """audit 2: an abort in the first second after enable parked at fade 1.0, stepping J3 from 1x to 2.5x URDF
        gravity. Now park keeps min(1, t / SETTLE_S) of the abort."""
        logs: list[str] = []
        phase = {"p": "pre", "park_call_idx": None}
        out = Path(tempfile.mkdtemp(dir=TMP.name))
        bus = fakebus.FakeBus()
        buses = []                       # the fake bus run() builds for --simulate

        def log(line):
            logs.append(line)
            if line.startswith("all 7 motors ENABLED"):
                phase["p"] = "enabled"
            elif line.startswith("PARK:"):
                phase["p"], phase["park_call_idx"] = "park", len(buses[0].calls)
        rt = fm.Runtime(clock=bus.clock, sleep=bus.clock.sleep, tty_reader=lambda p: None, out_dir=out, mb=bus.mod,
                        simulated=True, check_host=False, require_confirmation=False, install_signals=False, log=log)
        calls = {"enabled": [], "park": []}
        real_ff = d.feed_forward

        def spy(gm, q, v_ref, fade, bias=0.0):
            res = real_ff(gm, q, v_ref, fade, bias)
            if phase["p"] in calls:
                calls[phase["p"]].append((dict(q), dict(v_ref), fade, res))
                if phase["p"] == "enabled" and fade >= 0.2 and not rt.stop_flag.get("why"):
                    rt.stop_flag["why"] = "test: stop 0.2 s after enable"
            return res
        d.feed_forward = spy
        try:
            rc = d.run(["--simulate", "--operator-go", "go"], rt=rt, bus_factory=None, sim_setup=buses.append)
        finally:
            d.feed_forward = real_ff
        s = json.loads(next(out.glob("*.json")).read_text())
        self.assertEqual(rc, fm.EXIT_ABORTED, logs[-6:])
        self.assertEqual(s["trigger"]["id"], "signal")
        self.assertEqual(s["outcome"], "aborted_parked")
        fade = s["stats"]["ff_fade_at_abort"]
        self.assertGreaterEqual(fade, 0.2)
        self.assertLess(fade, 0.21)
        self.assertEqual(fade, calls["enabled"][-1][2])          # the fade of the last command before the abort
        self.assertTrue(calls["park"])
        self.assertTrue(all(c[2] == fade for c in calls["park"]))
        # the first park command actually sent on j3 = the faded feed-forward, not the 2.5x one
        q, v, _, _ = calls["park"][0]
        j3_tau = next(c[2][4] for c in buses[0].calls[phase["park_call_idx"]:] if c[0] == "j3" and c[1] == "send_mit")
        expect = real_ff(d.gravity.GravityModel(), q, v, min(1.0, fade))["j3"]
        self.assertAlmostEqual(j3_tau, expect, delta=1e-6)
        self.assertGreater(abs(real_ff(d.gravity.GravityModel(), q, v, 1.0)["j3"] - j3_tau), 0.5)
        self.assertTrue(any("fade-in kept at" in x for x in logs))

    def _hold(self, raise_at: dict, stop_after: int):
        """guard_recover with a scripted cycle: raise_at = {call number: Abort}; 'stop' after stop_after calls."""
        clock = _Clock()
        rt = _fake_rt(clock)
        seen = []

        def cycle(q_ref, v_ref, ff, mode, seg, t, lim, t_prev):
            seen.append((mode, t_prev[0], clock()))
            if mode == "guard_hold":
                n = sum(1 for m, *_ in seen if m == "guard_hold")
                if n in raise_at:
                    raise raise_at[n]
                if n >= stop_after:
                    rt.stop_flag["why"] = "operator"
        logs, stats = [], {}
        saved_guard = d.GUARD
        d.GUARD = None
        try:
            _, ff_for = self._ff_real()
            out = d.guard_recover(rt, None, {n: 0.0 for n in d.NAMES} | {"j2": -0.5}, {n: 0.0 for n in d.NAMES},
                                  ff_for, cycle, stats, {"n": 0}, logs.append)
        finally:
            d.GUARD = saved_guard
        return out, seen, logs, stats

    def test_a_stream_gap_while_holding_after_a_guard_fault_warns_and_keeps_holding(self):
        out, seen, logs, stats = self._hold({2: d.Abort("stream", "120 ms between command cycles", True)}, 5)
        self.assertEqual(out, "aborted_parked")                  # held on, then the operator's stop parked it
        holds = [x for x in seen if x[0] == "guard_hold"]
        self.assertGreaterEqual(len(holds), 5)
        self.assertEqual(holds[2][1], holds[2][2])               # the hold was re-sent at once, gap clock restarted
        self.assertEqual(stats["hold_stream_gaps"], 1)
        self.assertTrue(any("WARNING" in x and "[stream]" in x and "still holding" in x for x in logs))
        self.assertFalse(any("cannot hold" in x for x in logs))

    def test_other_faults_while_holding_still_disable(self):
        for trig in ("stale", "status", "bus", "guard"):
            out, seen, logs, stats = self._hold({2: d.Abort(trig, "x", False)}, 99)
            self.assertEqual(out, "aborted_guard_hold_failed", trig)
            self.assertTrue(any("cannot hold, disabling" in x for x in logs), trig)


def argparse_ns():
    import argparse
    return argparse.Namespace(state_file=None, simulate=True)


def _fake_rt_log(logs):
    return fm.Runtime(clock=_Clock(), sleep=lambda dt: None, tty_reader=lambda p: None, out_dir=Path(TMP.name), mb=fakebus.FakeBus().mod,
                      simulated=True, check_host=False, require_confirmation=False, install_signals=False,
                      log=logs.append)

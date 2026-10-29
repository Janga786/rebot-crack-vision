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

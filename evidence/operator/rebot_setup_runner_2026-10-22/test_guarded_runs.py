"""Offline tests for demo_move.py under arm_guard (2026-10-21): --trajectory, the guard pre-flight, guard-fault
handling, the startup check / J1 whole turns, and the real-run guard requirement. fakebus + the real guard config
(--guard-sim), simulated time; no hardware.

  env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python -m unittest test_guarded_runs
"""
from __future__ import annotations

import copy
import csv
import json
import math
import random
import sys
import tempfile
import types
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import demo_move as d  # noqa: E402
import fakebus  # noqa: E402
import first_move_j1 as fm  # noqa: E402

TMP = tempfile.TemporaryDirectory(prefix="guarded_tests_")
SMOKE = HERE / "trajectories" / "smoke_raise.json"
FIRST = HERE / "trajectories" / "first_guarded_run.json"


def traj_file(points: list, **over) -> str:
    doc = json.loads(SMOKE.read_text()) | over
    doc["points"] = [{"t_s": t, "positions": q} for t, q in points]
    path = Path(tempfile.mkdtemp(dir=TMP.name)) / "t.json"
    path.write_text(json.dumps(doc))
    return str(path)


def run_sim(*argv, setup=None, guard=True, go_after=2.0, wait_go=True):
    """demo_move.run on fakebus; the "operator" gives the second go go_after s after the arm reached zero."""
    logs: list[str] = []
    out = Path(tempfile.mkdtemp(dir=TMP.name))
    bus = fakebus.FakeBus()
    rt = fm.Runtime(clock=bus.clock, sleep=bus.clock.sleep, tty_reader=lambda p: None, out_dir=out, mb=bus.mod,
                    simulated=True, check_host=False, require_confirmation=False, install_signals=False,
                    log=logs.append)
    args = ["--simulate", "--operator-go", "go", *argv]
    if guard:
        args.append("--guard-sim")
    if wait_go:
        args += ["--wait-go", str(out / "GO"), "--sim-auto-go", str(go_after)]
    rc = d.run(args, rt=rt, bus_factory=None, sim_setup=(lambda b: setup(b, rt)) if setup else None)
    summary = json.loads(next(out.glob("*.json")).read_text())
    return rc, logs, summary


def faults(summary) -> list:
    return [e for e in (summary.get("guard") or {}).get("events", []) if e["kind"] == "fault"]


class TrajectoryFile(unittest.TestCase):
    def test_repo_fixture_and_generated_file_load(self):
        for f in (SMOKE, FIRST):
            header, times, qs = d.load_trajectory(str(f))
            self.assertEqual(times[0], 0.0)
            self.assertEqual(len(qs[0]), 6)
            self.assertEqual(len(header["sha256"]), 64)

    def test_bad_files_are_usage_errors(self):
        bad = [
            traj_file([(0.0, [0] * 6), (1.0, [0] * 6)], schema="crackvision.joint_trajectory/2"),
            traj_file([(0.0, [0] * 6), (1.0, [0] * 6)], joint_names=["joint1"]),
            traj_file([(0.0, [0] * 6), (0.0, [0] * 6)]),
            traj_file([(0.5, [0] * 6), (1.0, [0] * 6)]),
            traj_file([(0.0, [0] * 6), (1.0, [0, 0, float("nan"), 0, 0, 0])]),
            traj_file([(0.0, [0] * 6)]),
        ]
        for f in bad:
            with self.assertRaises(ValueError, msg=f):
                d.load_trajectory(f)
            self.assertEqual(d.run(["--simulate", "--trajectory", f, "--wait-go", "/tmp/x"]), fm.EXIT_USAGE)

    def test_trajectory_needs_wait_go_and_no_speed(self):
        self.assertEqual(d.run(["--simulate", "--trajectory", str(SMOKE)]), fm.EXIT_USAGE)
        self.assertEqual(d.run(["--simulate", "--trajectory", str(SMOKE), "--wait-go", "/tmp/x", "--speed", "2"]),
                         fm.EXIT_USAGE)
        self.assertEqual(d.run(["--guard-sim", "--check-only"]), fm.EXIT_USAGE)


class MonotoneCubic(unittest.TestCase):
    def test_never_overshoots_and_rests_at_both_ends(self):
        rng = random.Random(5)
        for _ in range(200):
            n = rng.randint(2, 12)
            t = [0.0]
            for _ in range(n - 1):
                t.append(t[-1] + rng.uniform(0.05, 2.0))
            y = [rng.uniform(-2, 2) for _ in range(n)]
            m = d.pchip_slopes(t, y)
            self.assertEqual((m[0], m[-1]), (0.0, 0.0))
            for i in range(n - 1):
                h = t[i + 1] - t[i]
                lo, hi = min(y[i], y[i + 1]), max(y[i], y[i + 1])
                for k in range(21):
                    q, _ = d.hermite(y[i], y[i + 1], m[i], m[i + 1], h, k / 20)
                    self.assertTrue(lo - 1e-12 <= q <= hi + 1e-12)
                self.assertAlmostEqual(d.hermite(y[i], y[i + 1], m[i], m[i + 1], h, 0.0)[0], y[i])
                self.assertAlmostEqual(d.hermite(y[i], y[i + 1], m[i], m[i + 1], h, 1.0)[0], y[i + 1])


class GuardedTrajectoryRuns(unittest.TestCase):
    def test_both_trajectories_complete_with_no_guard_fault(self):
        for f, slow in ((SMOKE, "1"), (FIRST, "1"), (SMOKE, "2")):
            rc, logs, s = run_sim("--trajectory", str(f), "--slow", slow)
            self.assertEqual(rc, fm.EXIT_OK, logs[-8:])
            self.assertEqual(s["outcome"], "completed")
            self.assertTrue(s["all_disabled_confirmed"])
            self.assertEqual(faults(s), [])
            self.assertEqual(s["guard_preflight"]["problems"], [])
            self.assertTrue(any(e["kind"] == "startup" for e in s["guard"]["events"]))
            rows = list(csv.DictReader(open(s["csv"])))
            self.assertLess(max(abs(float(rows[-1][f"j{i}_pos"])) for i in (2, 3)), 0.04)   # ended folded

    def test_the_trajectory_is_followed(self):
        rc, logs, s = run_sim("--trajectory", str(SMOKE))
        rows = [r for r in csv.DictReader(open(s["csv"])) if r["mode"] == "path"]
        self.assertAlmostEqual(min(float(r["j3_ref"]) for r in rows), -0.9, places=3)   # reached P
        self.assertLess(max(abs(float(r["j3_ref"]) - float(r["j3_pos"])) for r in rows), 0.06)

    def test_guard_rejects_up_front_what_the_demo_limits_allow(self):
        f = traj_file([(0.0, [0] * 6), (8.0, [-3.0, 0, 0, 0, 0, 0])])   # inside the demo's J1 -3.9, past the URDF
        rc, logs, s = run_sim("--trajectory", f)
        self.assertEqual(rc, fm.EXIT_REFUSED)
        self.assertTrue(any("arm_guard pre-flight" in line for line in logs))
        self.assertTrue(s["refusal"][0].startswith("arm_guard: "))
        self.assertFalse(any("ENABLED" in line for line in logs))

    def test_keepout_the_demo_does_not_check_is_refused_up_front(self):
        # wrist bent back toward the upper arm: the gripper stays above the table (the demo's only clearance rule),
        # link6 ends 16 mm inside the guard's 60 mm upper-arm clearance
        f = traj_file([(0.0, [0] * 6), (8.0, [0, -0.03, -0.01, 0.61, -1.55, 0])])
        rc, logs, s = run_sim("--trajectory", f)
        self.assertEqual(rc, fm.EXIT_REFUSED, logs[-4:])
        self.assertTrue(any("keep-out" in r for r in s["refusal"]), s["refusal"])

    def test_the_show_demo_is_refused_by_the_guard_before_anything_moves(self):
        rc, logs, s = run_sim("--choreo", "show", "--speed", "2")
        self.assertEqual(rc, fm.EXIT_REFUSED)
        self.assertTrue(any("j1: target" in r for r in s["refusal"]))
        self.assertFalse(any("ENABLED" in line for line in logs))


class GuardFaultHandling(unittest.TestCase):
    def test_latch_mid_trajectory_holds_until_the_operator_then_parks(self):
        def setup(bus, rt):
            def trip(now):
                c = bus.controller
                if c is not None and c.t_enable is not None and now - c.t_enable > 9.0 and d.GUARD \
                        and not d.GUARD.latched and not getattr(trip, "done", False):
                    trip.done = True
                    jr = next(iter(d.GUARD.buses.values())).joints["j3"]
                    d.GUARD._trip(jr, ["test: injected fault"])
            bus.clock.on_advance.append(trip)
        rc, logs, s = run_sim("--trajectory", str(FIRST), setup=setup)
        self.assertEqual(s["outcome"], "aborted_parked", logs[-12:])
        self.assertEqual(s["stats"]["guard_faults"], 1)
        self.assertEqual(s["stats"]["guard_resets"], 1)
        self.assertTrue(s["all_disabled_confirmed"])
        kinds = [e["kind"] for e in s["guard"]["events"]]
        self.assertLess(kinds.index("fault"), kinds.index("reset"))
        rows = list(csv.DictReader(open(s["csv"])))
        held = [r for r in rows if r["mode"] == "guard_hold"]
        self.assertGreater(len(held), 100)                                # held ~3 s before the "operator" said stop
        j3 = [float(r["j3_pos"]) for r in held]
        self.assertLess(max(j3) - min(j3), 0.02)                          # and it really held still
        self.assertLess(float(rows[-1]["j3_pos"]), 0.0)
        self.assertGreater(float(rows[-1]["j3_pos"]), -0.05)              # parked back at the fold before disabling
        i_hold = rows.index(held[0])
        self.assertFalse(any(r["mode"] == "park" for r in rows[:i_hold])) # no park before the operator's stop

    def test_a_guard_fault_during_a_park_goes_back_to_holding(self):
        def setup(bus, rt):
            def spike(now):
                c = bus.controller
                if c is None or c.t_enable is None:
                    return
                t = now - c.t_enable
                bus.sims["j5"].torque_override = 7.5 if 10.0 <= t < 10.6 else None   # > the guard's 6.5 Nm abort
            bus.clock.on_advance.append(spike)
        rc, logs, s = run_sim("--trajectory", str(FIRST), setup=setup)
        self.assertTrue(s["all_disabled_confirmed"])
        self.assertIn(s["outcome"], ("aborted_parked",), logs[-12:])
        self.assertGreaterEqual(s["stats"].get("guard_faults", 0), 1)
        self.assertTrue(any("GUARD FAULT, HOLDING" in line for line in logs))

    def test_operator_stop_mid_run_parks_as_before_and_the_guard_stays_clear(self):
        def setup(bus, rt):
            def ctrl_c(now):
                c = bus.controller
                if c is not None and c.t_enable is not None and now - c.t_enable > 12.0 \
                        and not getattr(ctrl_c, "done", False):
                    ctrl_c.done = True
                    rt.stop_flag["why"] = "test Ctrl-C"
            bus.clock.on_advance.append(ctrl_c)
        rc, logs, s = run_sim("--trajectory", str(FIRST), setup=setup)
        self.assertEqual(s["trigger"]["id"], "signal")
        self.assertEqual(s["outcome"], "aborted_parked", logs[-6:])
        self.assertEqual(faults(s), [])
        self.assertTrue(s["all_disabled_confirmed"])

    def test_a_fault_that_persists_ends_with_the_operator_switching_off(self):
        """A stuck J5 reading (sensor fault): every park re-trips the guard; the arm keeps holding in between and only
        the operator's 'off' switches it off."""
        def setup(bus, rt):
            bus.faults.other_moves_at, bus.faults.other_moves_name = 12.0, "j5"
        rc, logs, s = run_sim("--trajectory", str(FIRST), setup=setup)
        self.assertEqual(s["outcome"], "aborted_operator_off", logs[-10:])
        self.assertGreaterEqual(s["stats"]["guard_faults"], d.SIM_MAX_PARKS + 1)
        self.assertTrue(s["all_disabled_confirmed"])
        self.assertTrue(any("switches off now" in line for line in logs))


class StartupAndWholeTurns(unittest.TestCase):
    def test_j1_one_turn_off_is_unwrapped_and_the_run_completes(self):
        ref = Path(tempfile.mkdtemp(dir=TMP.name)) / "ref.json"
        ref.write_text(json.dumps({"offset": 6.3346}))
        rc, logs, s = run_sim("--trajectory", str(SMOKE), "--j1-ref-file", str(ref),
                              setup=lambda b, rt: setattr(b.sims["j1"], "pos", 6.3354))
        self.assertEqual(rc, fm.EXIT_OK, logs[-8:])
        self.assertAlmostEqual(s["guard_preflight"]["offsets"]["j1"], 2 * math.pi)
        self.assertTrue(any("whole-turn offsets" in e["msg"] for e in s["guard"]["events"] if e["kind"] == "startup"))

    def test_j2_drooped_start_is_lifted_and_the_run_completes(self):
        ref = Path(tempfile.mkdtemp(dir=TMP.name)) / "ref.json"
        ref.write_text(json.dumps({"offset": 0.0}))

        def setup(b, rt):
            b.sims["j1"].pos, b.sims["j2"].pos = 0.01, 0.40
        rc, logs, s = run_sim("--trajectory", str(SMOKE), "--j1-ref-file", str(ref), setup=setup)
        self.assertEqual(rc, fm.EXIT_OK, logs[-8:])
        self.assertEqual(faults(s), [])

    def test_implausible_j1_reading_is_refused_by_the_guard(self):
        rc, logs, s = run_sim("--trajectory", str(SMOKE), "--j1-here-is-zero",
                              setup=lambda b, rt: setattr(b.sims["j1"], "pos", 3.0337))
        self.assertEqual(rc, fm.EXIT_REFUSED)
        self.assertTrue(any("startup: j1" in r for r in s["refusal"]), s["refusal"])
        self.assertFalse(any("ENABLED" in line for line in logs))


class GainsOverride(unittest.TestCase):
    def test_override_applies_to_this_run_only_and_is_recorded(self):
        before = dict(d.GAINS)
        rc, logs, s = run_sim("--trajectory", str(SMOKE), "--gains", "j2=100/4,j3=80/4")
        self.assertEqual(rc, fm.EXIT_OK, logs[-6:])
        self.assertEqual(s["gains_override"], {"j2": [100.0, 4.0], "j3": [80.0, 4.0]})
        self.assertEqual(s["gains"]["j2"], [100.0, 4.0])
        self.assertTrue(any("gains override for this run" in line for line in logs))
        self.assertEqual(d.GAINS, before)                                   # restored afterwards

    def test_bad_overrides_are_usage_errors(self):
        for spec in ("j2=100", "j9=100/4", "j2=150/4", "j3=80/6", "j2=x/4"):
            self.assertEqual(d.run(["--simulate", "--gains", spec]), fm.EXIT_USAGE, spec)


class RealRunNeedsTheGuard(unittest.TestCase):
    def test_guard_missing_reasons(self):
        saved = sys.modules.get("motorbridge.core")
        try:
            sys.modules["motorbridge.core"] = types.SimpleNamespace(Motor=type("M", (), {}),
                                                                    Controller=type("C", (), {}))
            self.assertIn("not patched", d.guard_missing())
        finally:
            if saved is None:
                sys.modules.pop("motorbridge.core", None)
            else:
                sys.modules["motorbridge.core"] = saved

    def test_real_run_without_guard_is_refused_before_the_bus(self):
        bus = fakebus.FakeBus()
        logs: list[str] = []
        rt = fm.Runtime(clock=bus.clock, sleep=bus.clock.sleep, tty_reader=lambda p: None,
                        out_dir=Path(tempfile.mkdtemp(dir=TMP.name)), mb=bus.mod, simulated=False, check_host=False,
                        require_confirmation=False, install_signals=False, log=logs.append)
        saved = sys.modules.get("motorbridge.core")
        sys.modules.pop("motorbridge.core", None)
        try:
            rc = d.run(["--operator-go", "go", "--trajectory", str(SMOKE), "--wait-go", "/tmp/x"], rt=rt,
                       bus_factory=bus.factory)
        finally:
            if saved is not None:
                sys.modules["motorbridge.core"] = saved
        self.assertEqual(rc, fm.EXIT_REFUSED)
        self.assertEqual([c for c in bus.calls if c[1] == "from_dm_serial"], [])   # the bus was never opened
        self.assertTrue(any("not patched" in line for line in logs))


if __name__ == "__main__":
    unittest.main()

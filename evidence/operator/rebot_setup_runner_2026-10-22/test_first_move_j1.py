"""Offline tests for first_move_j1.py against fakebus.py (no hardware, no motorbridge import).

Run (any Python >= 3.10, no third-party packages):
    cd ~/rebot_setup && env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python -m unittest -v test_first_move_j1
"""
from __future__ import annotations

import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import fakebus  # noqa: E402
import first_move_j1 as fm  # noqa: E402

FORBIDDEN = {"write_register_u32", "write_register_f32", "set_zero_position", "ensure_mode", "set_can_timeout_ms",
             "store_parameters", "clear_error", "send_pos_vel", "send_vel"}
OTHERS = ("j2", "j3", "j4", "j5", "j6", "j7")

# Every run's output goes under one temporary directory, removed when the module's tests finish.
_TMP_ROOT = tempfile.TemporaryDirectory(prefix="fm_j1_tests_")


def tearDownModule() -> None:
    _TMP_ROOT.cleanup()


class Harness:
    """One fake bus + runtime. Runs the real script code path (confirmation included) in simulated time."""

    def __init__(self, faults: fakebus.Faults | None = None, rest: dict | None = None, tty: str | None = "ENABLE J1",
                 version: str = "0.5.6") -> None:
        self.bus = fakebus.FakeBus(faults=faults or fakebus.Faults(),
                                   rest=dict(fakebus.REST_2026_10_10) if rest is None else rest)
        self.tmp = Path(tempfile.mkdtemp(prefix="fm_j1_", dir=_TMP_ROOT.name))
        self.logs: list[str] = []
        self.prompts: list[str] = []

        def tty_reader(prompt: str):
            self.prompts.append(prompt)
            return tty

        mod = self.bus.mod
        if version != "0.5.6":
            mod = type("M", (), {"CallError": fakebus.FakeCallError, "__version__": version,
                                 "Controller": self.bus.mod.Controller})
        self.rt = fm.Runtime(clock=self.bus.clock, sleep=self.bus.clock.sleep, tty_reader=tty_reader,
                             out_dir=self.tmp, mb=mod, simulated=False, check_host=False,
                             require_confirmation=True, install_signals=False, log=self.logs.append)

    def run(self, *argv: str) -> int:
        return fm.run(list(argv), rt=self.rt)

    # --- inspection helpers ---
    def calls(self, who: str | None = None, method: str | None = None) -> list:
        return [c for c in self.bus.calls if (who is None or c[0] == who) and (method is None or c[1] == method)]

    def summary(self) -> dict:
        files = sorted(self.tmp.glob("*.json"))
        assert len(files) == 1, files
        return json.loads(files[0].read_text())

    def rows(self) -> list[dict]:
        files = sorted(self.tmp.glob("*.csv"))
        assert len(files) == 1, files
        with open(files[0]) as fh:
            return list(csv.DictReader(fh))

    def after_enable(self, who: str, method: str) -> list:
        """Calls of `method` on `who` made after motor 1's enable."""
        out, seen = [], False
        for c in self.bus.calls:
            if c[0] == "j1" and c[1] == "enable":
                seen = True
                continue
            if seen and c[0] == who and c[1] == method:
                out.append(c)
        return out


class Preconditions(unittest.TestCase):
    def assert_nothing_energised(self, h: Harness) -> None:
        self.assertEqual(h.calls(method="enable"), [])
        self.assertEqual(h.calls(method="send_mit"), [])
        self.assertEqual(h.calls("ctrl", "enable_all"), [])

    def test_refuses_when_a_joint_is_not_at_rest(self):
        rest = dict(fakebus.REST_2026_10_10, j5=0.21)
        self.skipTest("2026-10-14: the at-rest requirement was removed on the operator's decision")
        h = Harness(rest=rest)
        self.assertEqual(h.run(), fm.EXIT_REFUSED)
        self.assert_nothing_energised(h)
        self.assertIn("j5", h.summary()["refusal"])

    def test_refuses_when_another_motor_is_enabled(self):
        h = Harness()
        h.bus.sims["j3"].status = 1
        self.assertEqual(h.run(), fm.EXIT_REFUSED)
        self.assert_nothing_energised(h)
        self.assertIn("j3: status ENABLED", h.summary()["refusal"])

    def test_refuses_when_a_motor_does_not_answer(self):
        h = Harness()
        h.bus.sims["j6"].answers = False
        self.assertEqual(h.run(), fm.EXIT_REFUSED)
        self.assert_nothing_energised(h)
        self.assertIn("j6", h.summary()["refusal"])

    def test_refuses_when_motor1_is_not_in_mit_mode(self):
        h = Harness()
        h.bus.sims["j1"].registers[10] = 2      # POS_VEL, e.g. left behind by the vendor driver
        self.assertEqual(h.run(), fm.EXIT_REFUSED)
        self.assert_nothing_energised(h)
        self.assertIn("CTRL_MODE 2", h.summary()["refusal"])

    def test_refuses_on_model_register_mismatch(self):
        h = Harness()
        h.bus.sims["j1"].registers[23] = 10.0   # a 4310 TMAX on motor 1
        self.assertEqual(h.run(), fm.EXIT_REFUSED)
        self.assert_nothing_energised(h)

    def test_refuses_other_motorbridge_versions(self):
        h = Harness(version="0.6.0")
        self.assertEqual(h.run(), fm.EXIT_REFUSED)
        self.assertEqual(h.bus.calls, [])       # did not even open the bus

    def test_refuses_without_the_exact_confirmation(self):
        for typed in ("enable j1", "ENABLE", "", "yes"):
            h = Harness(tty=typed)
            self.assertEqual(h.run(), fm.EXIT_REFUSED, typed)
            self.assert_nothing_energised(h)
            self.assertEqual(len(h.prompts), 1)

    def test_refuses_without_a_terminal(self):
        h = Harness(tty=None)
        self.assertEqual(h.run(), fm.EXIT_REFUSED)
        self.assert_nothing_energised(h)

    def test_typed_confirmation_is_recorded(self):
        h = Harness()
        self.assertEqual(h.run(), fm.EXIT_OK)
        self.assertEqual(h.summary()["confirmation"], {"method": "typed_on_dev_tty", "phrase": "ENABLE J1"})

    def test_operator_go_replaces_the_typed_phrase_and_is_recorded(self):
        for argv in ((), ("--estop-test", "--estop-wait-s", "3")):
            h = Harness(tty=None)                   # no terminal: the Claude session's situation
            h.run(*argv, "--operator-go", "go")
            self.assertEqual(h.prompts, [], argv)   # /dev/tty is never asked
            self.assertEqual(h.summary()["confirmation"], {"method": "operator_go_in_chat", "operator_go": "go"})
            self.assertEqual(len(h.calls("j1", "enable")), 1, argv)

    def test_operator_go_keeps_every_precondition(self):
        h = Harness(tty=None)                   # a goal outside joint2's URDF range [-3.14, 0]
        self.assertEqual(h.run("--operator-go", "go", "--joint", "j2", "--move", "0.10"), fm.EXIT_REFUSED)
        self.assert_nothing_energised(h)

    def test_blank_operator_go_is_a_usage_error(self):
        h = Harness()
        self.assertEqual(h.run("--operator-go", "  "), fm.EXIT_USAGE)
        self.assertEqual(h.bus.calls, [])

    def test_usage_errors_for_caps_and_timing(self):
        for argv in (["--kp", str(fm.KP_MAX + 0.5)], ["--kd", str(fm.KD_MAX + 0.01)], ["--kp", "0"], ["--ramp-s", str(fm.MIN_RAMP_S - 0.01)], ["--hold-s", "0.5"],
                     ["--target", "0.16"], ["--target", "-0.2"], ["--estop-wait-s", "61"]):
            h = Harness()
            self.assertEqual(h.run(*argv), fm.EXIT_USAGE, argv)
            self.assertEqual(h.bus.calls, [], argv)

    def test_check_only_never_energises_and_records_readings(self):
        h = Harness()
        self.assertEqual(h.run("--check-only"), fm.EXIT_OK)
        self.assert_nothing_energised(h)
        self.assertEqual(h.calls(method="disable"), [])
        s = h.summary()
        self.assertEqual(s["outcome"], "check_ok")
        self.assertAlmostEqual(s["precheck"]["j5"]["pos"], 0.03147)
        self.assertEqual(s["precheck"]["j1"]["ctrl_mode"], 1)
        self.assertEqual(h.prompts, [])         # no confirmation needed for a read-only check


class NominalRun(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.h = Harness()
        cls.rc = cls.h.run()
        cls.s = cls.h.summary()
        cls.r = cls.h.rows()

    def test_completes(self):
        self.assertEqual(self.rc, fm.EXIT_OK, self.h.logs[-5:])
        self.assertEqual(self.s["outcome"], "completed")
        self.assertIsNone(self.s["trigger"])
        self.assertTrue(self.s["stats"]["disabled_confirmed"])

    def test_only_motor1_is_ever_energised(self):
        for name in OTHERS:
            methods = {c[1] for c in self.h.calls(name)}
            self.assertLessEqual(methods, {"request_feedback", "get_state", "get_register_u32", "get_register_f32",
                                           "close"}, name)
        self.assertEqual(len(self.h.calls("j1", "enable")), 1)
        self.assertEqual(self.h.calls("ctrl", "enable_all"), [])
        self.assertEqual(self.h.calls("ctrl", "shutdown"), [])

    def test_no_register_writes_zeroing_or_mode_changes(self):
        bad = [c for c in self.h.bus.calls if c[1] in FORBIDDEN]
        self.assertEqual(bad, [])

    def test_first_command_after_enable_targets_the_measured_position(self):
        mits = self.h.calls("j1", "send_mit")
        first = mits[0][2]
        self.assertEqual(first[0], self.s["stats"]["first_cmd_rad"])
        self.assertAlmostEqual(first[0], fakebus.REST_2026_10_10["j1"], places=4)
        self.assertEqual(first[1], 0.0)

    def test_every_command_respects_the_caps_and_zero_torque(self):
        for _, _, (pos, vel, kp, kd, tau) in self.h.calls("j1", "send_mit"):
            self.assertLessEqual(kp, fm.KP_MAX)
            self.assertLessEqual(kd, fm.KD_MAX)
            self.assertEqual(tau, 0.0)
            self.assertLess(abs(vel), 0.1)
            self.assertLessEqual(pos, 0.10 + 1e-12)
            self.assertGreaterEqual(pos, fakebus.REST_2026_10_10["j1"] - 1e-12)

    def test_profile_reaches_target_holds_and_returns(self):
        targets = [float(x["target_rad"]) for x in self.r]
        self.assertAlmostEqual(max(targets), 0.10, places=6)
        self.assertAlmostEqual(targets[-1], self.s["q_start"], places=6)
        # >= 1 s at the target, >= 2 s for each ramp, >= 1 s final hold (from the logged phases)
        dur = {}
        for x in self.r:
            dur.setdefault(x["phase"], []).append(float(x["t_s"]))
        span = {k: max(v) - min(v) for k, v in dur.items()}
        self.assertGreaterEqual(span["ramp_up"], 2.0 - 0.02)
        self.assertGreaterEqual(span["hold"], 1.0 - 0.02)
        self.assertGreaterEqual(span["ramp_down"], 2.0 - 0.02)
        self.assertGreaterEqual(span["final_hold"], 1.0 - 0.02)
        # the simulated joint really moved there and back
        pos = [float(x["pos_rad"]) for x in self.r]
        # The fake has 0.3 Nm of Coulomb friction, so kp = 10 leaves up to 0.03 rad of steady-state error.
        self.assertGreater(max(pos), 0.065)
        self.assertLess(abs(pos[-1] - self.s["q_start"]), 0.035)
        self.assertLess(abs(self.s["stats"]["hold_mean_err_rad"]), fm.ARRIVAL_WARN_RAD + 0.005)

    def test_streams_at_least_100_hz(self):
        t = [float(x["t_s"]) for x in self.r]
        gaps = [b - a for a, b in zip(t, t[1:])]
        self.assertGreaterEqual(len(t) / (t[-1] - t[0]), 100.0)
        self.assertLessEqual(max(gaps), 0.010 + 1e-9)       # never slower than 100 Hz, even with heartbeats
        self.assertGreaterEqual(self.s["stats"]["mean_rate_hz"], 100.0)

    def test_csv_has_the_required_columns(self):
        for col in ("t_s", "target_rad", "pos_rad", "vel_rad_s", "torque_nm"):
            self.assertIn(col, self.r[0])

    def test_tracking_and_torque_stay_inside_the_abort_envelope(self):
        self.assertLess(self.s["stats"]["max_abs_err"], fm.TRACK_ABORT_RAD)
        self.assertLess(self.s["stats"]["max_abs_torque"], fm.torque_limit(fm.KP_MAX, fm.KD_MAX))

    def test_disable_is_the_last_command_to_motor1(self):
        j1 = [c[1] for c in self.h.calls("j1") if c[1] in ("enable", "send_mit", "disable")]
        self.assertEqual(j1[-1], "disable")
        self.assertEqual(j1.count("disable"), 1)
        self.assertEqual(self.h.bus.sims["j1"].status, 0)

    def test_heartbeats_cover_still_phases(self):
        # holds produce identical frames; the register round trip must keep feedback provably fresh
        self.assertGreater(self.s["stats"]["heartbeats_ok"], 0)
        self.assertEqual(self.s["stats"]["heartbeats_failed"], 0)
        self.assertLessEqual(max(float(x["fb_age_s"]) for x in self.r), fm.STALE_ABORT_S)


class Aborts(unittest.TestCase):
    def run_fault(self, **faults) -> Harness:
        h = Harness(faults=fakebus.Faults(**faults))
        h.rc = h.run()
        h.s = h.summary()
        return h

    def assert_hold_then_disable(self, h: Harness, trigger: str, source: str | None = None) -> None:
        self.assertEqual(h.rc, fm.EXIT_ABORTED, h.logs[-6:])
        self.assertEqual(h.s["outcome"], "aborted")
        self.assertEqual(h.s["trigger"]["id"], trigger)
        self.assertTrue(any(f"[{trigger}]" in line for line in h.logs))
        j1 = [c for c in h.calls("j1") if c[1] in ("send_mit", "disable")]
        self.assertEqual(j1[-1][1], "disable")
        if source is not None:
            self.assertEqual(h.s["stats"]["abort_hold_source"], source)
            hold = h.s["stats"]["abort_hold_rad"]
            tail = [c for c in j1 if c[1] == "send_mit"]
            held = [c for c in tail if c[2][0] == hold and c[2][1] == 0.0]
            # >= 0.5 s of hold commands at the loop rate
            self.assertGreaterEqual(len(held), int(fm.ABORT_HOLD_S * fm.LOOP_HZ) - 2)

    def test_tracking_abort_when_the_joint_jams(self):
        # A jam can only exceed the 0.10 rad threshold if the excursion is larger than 0.10 rad: start at -0.04
        # (inside the rest tolerance) so the +0.10 target is 0.14 rad away.
        h = Harness(faults=fakebus.Faults(stuck_from=1.0), rest=dict(fakebus.REST_2026_10_10, j1=-0.04))
        h.rc = h.run()
        h.s = h.summary()
        self.assert_hold_then_disable(h, "tracking", source="measured")
        # the hold is at the measured (jammed) position, not the last target
        self.assertLess(abs(h.s["stats"]["abort_hold_rad"] - h.bus.sims["j1"].pos), 1e-3)

    def test_torque_abort(self):
        h = self.run_fault(torque_spike_at=2.0, torque_spike_nm=12.0)
        self.assert_hold_then_disable(h, "torque", source="measured")

    def test_stale_abort_when_frames_stop_and_registers_go_silent(self):
        h = self.run_fault(freeze_at=2.0)
        self.assertEqual(h.s["trigger"]["id"], "stale")
        self.assertEqual(h.rc, fm.EXIT_ABORTED)
        rows = h.rows()
        self.assertLessEqual(float(rows[-1]["t_s"]), 2.0 + fm.STALE_ABORT_S + 0.05)
        self.assertEqual(h.s["stats"]["abort_hold_source"], "last_command")
        self.assertEqual(h.calls("j1", "disable")[-1][1], "disable")
        self.assertFalse(h.s["stats"]["disabled_confirmed"])
        self.assertTrue(any("CUT 24 V" in line for line in h.logs))

    def test_signal_abort(self):
        h = Harness()

        def on_adv(now):
            c = h.bus.controller
            if c is not None and c.t_enable is not None and now - c.t_enable >= 3.0:
                h.rt.stop_flag["why"] = "SIGINT"
        h.bus.clock.on_advance.append(on_adv)
        h.rc = h.run()
        h.s = h.summary()
        self.assert_hold_then_disable(h, "signal", source="measured")
        self.assertIn("SIGINT", h.s["trigger"]["detail"])

    def test_bus_error_skips_hold_and_still_disables(self):
        h = self.run_fault(bus_error_at=2.5)
        self.assertEqual(h.rc, fm.EXIT_ABORTED)
        self.assertEqual(h.s["trigger"]["id"], "bus")
        self.assertNotIn("abort_hold_rad", h.s["stats"])
        self.assertEqual(h.calls("j1", "disable")[-1][1], "disable")

    def test_disable_failure_falls_back_to_disable_all_and_says_cut_power(self):
        h = self.run_fault(bus_error_at=2.5, disable_raises=True)
        self.assertEqual(len(h.calls("ctrl", "disable_all")), 1)
        self.assertTrue(any("CUT 24 V" in line for line in h.logs))

    def test_another_motor_enabled_aborts_and_disables_all(self):
        h = self.run_fault(other_enabled_at=2.0)
        self.assert_hold_then_disable(h, "others", source="measured")
        self.assertEqual(len(h.calls("ctrl", "disable_all")), 1)

    def test_another_joint_moving_aborts(self):
        h = self.run_fault(other_moves_at=2.0)
        self.assert_hold_then_disable(h, "others", source="measured")
        self.assertEqual(h.calls("ctrl", "disable_all"), [])

    def test_internal_error_still_disables(self):
        h = Harness()
        orig = fm.Profile.at
        try:
            def boom(self, t):
                if t > 1.0:
                    raise ZeroDivisionError("bug")
                return orig(self, t)
            fm.Profile.at = boom
            rc = h.run()
        finally:
            fm.Profile.at = orig
        s = h.summary()
        self.assertEqual(rc, fm.EXIT_ABORTED)
        self.assertEqual(s["trigger"]["id"], "internal")
        self.assertEqual(h.calls("j1", "disable")[-1][1], "disable")


class SmallMoveJam(unittest.TestCase):
    def test_jam_on_a_small_move_completes_but_is_reported(self):
        h = Harness(faults=fakebus.Faults(stuck_from=0.4))      # jammed before the ramp starts
        rc = h.run()
        s = h.summary()
        self.assertEqual(rc, fm.EXIT_OK)                         # 0.083 rad < 0.10 rad: no abort is possible
        self.assertGreater(s["stats"]["hold_mean_err_rad"], fm.ARRIVAL_WARN_RAD)
        self.assertTrue(any("WARNING: during the hold" in line for line in h.logs))
        self.assertLess(s["stats"]["max_abs_torque"], fm.KP_MAX * 0.09)  # kp * 0.083 rad


class EstopTest(unittest.TestCase):
    def test_estop_observed_is_success(self):
        h = Harness(faults=fakebus.Faults(freeze_at=5.0))
        rc = h.run("--estop-test")
        s = h.summary()
        self.assertEqual(rc, fm.EXIT_OK, h.logs[-6:])
        self.assertEqual(s["outcome"], "estop_observed")
        self.assertEqual(s["trigger"]["id"], "stale")
        # holding only: every command is the measured start position, no motion
        poss = {c[2][0] for c in h.calls("j1", "send_mit")}
        self.assertEqual(poss, {s["q_start"]})

    def test_no_press_within_the_window_is_a_failure(self):
        h = Harness()
        rc = h.run("--estop-test", "--estop-wait-s", "3")
        s = h.summary()
        self.assertEqual(rc, fm.EXIT_ABORTED)
        self.assertEqual(s["trigger"]["id"], "estop_not_observed")
        self.assertTrue(s["stats"]["disabled_confirmed"])

    def test_identical_frames_from_a_live_motor_are_not_stale(self):
        # the motor is alive (registers answer) but every frame is byte-identical
        h = Harness(faults=fakebus.Faults(constant_frames=True))
        rc = h.run("--estop-test", "--estop-wait-s", "3")
        s = h.summary()
        self.assertEqual(s["trigger"]["id"], "estop_not_observed")    # not "stale"
        self.assertEqual(rc, fm.EXIT_ABORTED)
        self.assertGreater(s["stats"]["heartbeats_ok"], 40)


class Units(unittest.TestCase):
    def test_min_jerk_endpoints_and_peak(self):
        q0, q1, T = 0.017, 0.10, 2.0
        self.assertEqual(fm.min_jerk(q0, q1, T, 0.0), (q0, 0.0))
        self.assertEqual(fm.min_jerk(q0, q1, T, T), (q1, 0.0))
        peak = max(abs(fm.min_jerk(q0, q1, T, i * T / 2000)[1]) for i in range(2001))
        self.assertAlmostEqual(peak, 1.875 * (q1 - q0) / T, places=5)

    def test_torque_limit_value(self):
        self.assertAlmostEqual(fm.torque_limit(10.0, 1.0), 2.0)

    def test_proxies_block_everything_dangerous(self):
        bus = fakebus.FakeBus()
        c = bus.factory("/dev/null", 921600)
        ro = fm.ReadOnlyMotor("j2", c.add_damiao_motor(0x02, 0x12, "4340P"))
        for m in ("enable", "send_mit", "disable", "write_register_u32", "set_zero_position", "ensure_mode"):
            with self.assertRaises(PermissionError):
                getattr(ro, m)
        j1 = fm.J1Motor(c.add_damiao_motor(0x01, 0x11, "4340P"))
        for m in ("write_register_f32", "set_zero_position", "ensure_mode", "set_can_timeout_ms", "store_parameters"):
            with self.assertRaises(PermissionError):
                getattr(j1, m)
        with self.assertRaises(PermissionError):
            j1.send_mit(0.0, 0.0, fm.KP_MAX + 0.5, 1.0, 0.0)
        with self.assertRaises(PermissionError):
            j1.send_mit(0.0, 0.0, 10.0, 1.0, fm.FF_CAP_BY_MODEL["4340P"] + 0.5)
        with self.assertRaises(PermissionError):
            j1.send_mit(float("nan"), 0.0, 10.0, 1.0, 0.0)
        g = fm.GuardedController(c)
        for m in ("enable_all", "shutdown"):
            with self.assertRaises(PermissionError):
                getattr(g, m)

    def test_freshness_heartbeat_logic(self):
        clock = fakebus.FakeClock()
        f = fm.Freshness(clock, fakebus.FakeCallError)

        class M:
            ok = True

            def get_register_u32(self, rid, timeout_ms):
                if not self.ok:
                    clock.sleep(timeout_ms / 1000)
                    raise fakebus.FakeCallError("timeout")
                return 1
        m = M()
        st = fakebus.FakeMotorState(1, 0x11, 1, 0.1, 0.0, 0.0, 30.0, 28.0)
        f.observe(m, st)
        self.assertEqual(f.age(), 0.0)
        clock.sleep(0.06)
        f.observe(m, st)                      # unchanged frame, quiet > 50 ms -> heartbeat succeeds
        self.assertEqual(f.heartbeats_ok, 1)
        self.assertLess(f.age(), 0.01)
        m.ok = False
        for _ in range(30):
            clock.sleep(0.01)
            f.observe(m, st)
        self.assertGreater(f.age(), fm.STALE_ABORT_S)
        self.assertGreater(f.heartbeats_failed, 0)

    def test_simulate_does_not_import_motorbridge(self):
        sys.modules.pop("motorbridge", None)
        out = Path(tempfile.mkdtemp(prefix="fm_sim_", dir=_TMP_ROOT.name))
        rc = fm.run(["--simulate", "--out-dir", str(out)])
        self.assertEqual(rc, fm.EXIT_OK)
        self.assertNotIn("motorbridge", sys.modules)
        self.assertEqual(len(list(out.glob("*_SIM.csv"))), 1)


if __name__ == "__main__":
    unittest.main()


class HoldGravitySweep(unittest.TestCase):
    """2026-10-17: --hold-others, --gravity and --sweep."""

    def _harness_with_gravity(self) -> Harness:
        import gravity
        h = Harness()
        for n, tq in gravity.GravityModel().hold_torques(dict(h.bus.rest)).items():
            h.bus.sims[n].load = -tq
        return h

    def test_gravity_model_sign_at_the_fold(self):
        import gravity
        g = gravity.GravityModel().hold_torques({})
        self.assertAlmostEqual(g["j3"], -2.79, delta=0.05)   # a real j3 hold measured -5.1 Nm (with friction)
        self.assertLess(g["j2"], 0.0)

    def test_hold_others_enables_all_and_disables_all(self):
        h = Harness()
        self.assertEqual(h.run("--joint", "j3", "--move", "-0.10", "--hold-others"), fm.EXIT_OK)
        for n in ("j1", "j2", "j4", "j5", "j6", "j7"):
            self.assertEqual(len(h.calls(n, "enable")), 1, n)
            self.assertGreaterEqual(len(h.calls(n, "disable")), 1, n)
            self.assertEqual(h.bus.sims[n].status, 0, n)
        self.assertEqual(h.bus.sims["j3"].status, 0)

    def test_held_joint_bumped_aborts_and_disables_everything(self):
        h = Harness(faults=fakebus.Faults(other_moves_at=2.0))
        self.assertEqual(h.run("--joint", "j3", "--move", "-0.10", "--hold-others"), fm.EXIT_ABORTED)
        self.assertEqual(h.summary()["trigger"]["id"], "held")
        self.assertTrue(all(s.status == 0 for s in h.bus.sims.values()))

    def test_gravity_feed_forward_reaches_the_motors(self):
        h = self._harness_with_gravity()
        self.assertEqual(h.run("--joint", "j3", "--move", "-0.10", "--hold-others", "--gravity"), fm.EXIT_OK)
        taus = [c[2][4] for c in h.calls("j3", "send_mit")]
        self.assertTrue(all(-3.0 < t < -2.5 for t in taus), (min(taus), max(taus)))
        self.assertTrue(any(c[2][4] < -1.0 for c in h.calls("j2", "send_mit")))   # held j2 gets its gravity too

    def test_gravity_needs_hold_others_and_scale_bounds(self):
        h = Harness()
        self.assertEqual(h.run("--joint", "j3", "--move", "-0.10", "--gravity"), fm.EXIT_USAGE)
        self.assertEqual(h.run("--joint", "j3", "--move", "-0.10", "--hold-others", "--gravity",
                               "--gravity-scale", "2"), fm.EXIT_USAGE)
        self.assertEqual(h.calls(method="enable"), [])

    def test_sweep_runs_every_pair_in_one_enable_and_scores_them(self):
        h = self._harness_with_gravity()
        self.assertEqual(h.run("--joint", "j3", "--move", "-0.10", "--hold-others", "--gravity", "--ramp-s", "1",
                               "--sweep", "40:0.5,100:1.0"), fm.EXIT_OK)
        self.assertEqual(len(h.calls("j3", "enable")), 1)
        table = h.summary()["sweep"]
        self.assertEqual([(r["kp"], r["kd"]) for r in table], [(40.0, 0.5), (100.0, 1.0)])
        self.assertTrue(all(r["complete"] for r in table))
        self.assertLess(table[1]["rms_track_rad"], table[0]["rms_track_rad"])   # stiffer tracks closer
        kps = {c[2][2] for c in h.calls("j3", "send_mit")}
        self.assertEqual(kps, {40.0, 100.0})

    def test_sweep_usage_errors(self):
        h = Harness()
        for bad in ("40", "40:0.5,abc", "200:0.5", "40:9", ",".join(["40:0.5"] * (fm.SWEEP_MAX + 1))):
            self.assertEqual(h.run("--joint", "j3", "--move", "-0.10", "--sweep", bad), fm.EXIT_USAGE, bad)
        self.assertEqual(h.calls(method="enable"), [])


class MultiJoint(unittest.TestCase):
    """2026-10-17: --tuned and --also (several joints moving together)."""

    def test_also_moves_each_joint_with_its_tuned_gains_and_returns(self):
        import gravity
        h = Harness()
        for n, tq in gravity.GravityModel().hold_torques(dict(h.bus.rest)).items():
            h.bus.sims[n].load = -tq
        start = {n: h.bus.sims[n].pos for n in ("j2", "j3")}
        self.assertEqual(h.run("--joint", "j1", "--move", "0.10", "--also", "j2=-0.10,j3=-0.10", "--tuned",
                               "--hold-others", "--gravity"), fm.EXIT_OK)
        for n in ("j2", "j3"):
            gains = {(c[2][2], c[2][3]) for c in h.calls(n, "send_mit")}
            self.assertIn(fm.TUNED[n][:2], gains, n)
            targets = [c[2][0] for c in h.calls(n, "send_mit")]
            self.assertAlmostEqual(min(targets), start[n] - 0.10, delta=0.01)   # reached the goal...
            self.assertAlmostEqual(targets[-1], start[n], delta=0.01)          # ...and came back
        self.assertTrue(set(h.summary()["also"]) == {"j2", "j3"})
        self.assertTrue(all(s.status == 0 for s in h.bus.sims.values()))

    def test_also_usage_errors(self):
        h = Harness()
        for extra in (["--also", "j2=-0.10"],                                         # no --tuned/--hold-others
                      ["--also", "j1=0.1", "--tuned", "--hold-others"],               # the active joint
                      ["--also", "j2=-0.5", "--tuned", "--hold-others"],              # too far
                      ["--also", "j7=-0.1", "--tuned", "--hold-others"],              # gripper not allowed
                      ["--also", "j2=-0.1", "--tuned", "--hold-others", "--sweep", "40:1,60:1"]):
            self.assertEqual(h.run("--joint", "j1", "--move", "0.10", *extra), fm.EXIT_USAGE, extra)
        self.assertEqual(h.calls(method="enable"), [])

    def test_held_motor_refuses_tuned_gains_unless_it_moves(self):
        fm.ALSO = {}
        m = fm.HoldMotor("j2", object())
        with self.assertRaises(PermissionError):
            m.send_mit(0.0, 0.0, *fm.TUNED["j2"][:2], 0.0)
        with self.assertRaises(PermissionError):
            m.send_mit(0.0, 0.1, *fm.HOLD_GAINS["j2"], 0.0)

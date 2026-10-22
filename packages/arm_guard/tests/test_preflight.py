"""Pre-flight: the offline mirror of what the runtime will accept for a planned command stream."""

import math

from arm_guard import preflight
from conftest import make_rig

FOLD = {f"j{i}": 0.0 for i in range(1, 8)}
GAINS = {"j1": (100.0, 4.0), "j2": (100.0, 1.0), "j3": (80.0, 1.0), "j4": (40.0, 1.0), "j5": (40.0, 1.0),
         "j6": (40.0, 1.0), "j7": (40.0, 0.5)}


def ramp(target: dict, T=4.0, hz=200, start=FOLD, offset=None):
    """Min-jerk from start to target, as (t, {joint: (pos_raw, vel, kp, kd, tau)}) ticks."""
    offset = offset or {}
    n = int(T * hz)
    for k in range(n + 1):
        s = k / n
        sh = 10 * s**3 - 15 * s**4 + 6 * s**5
        dsh = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
        cmds = {}
        for name, q0 in start.items():
            q1 = target.get(name, q0)
            cmds[name] = (q0 + (q1 - q0) * sh + offset.get(name, 0.0), (q1 - q0) * dsh, *GAINS[name], 0.0)
        yield k / hz, cmds


def test_a_normal_raise_passes(cfg):
    r = preflight.check_plan(cfg, FOLD, ramp({"j2": -0.6, "j3": -0.9, "j1": 0.1, "j4": 0.1}))
    assert r["problems"] == [] and r["min_keepout_m"] > 0 and r["n_ticks"] == 801


def test_beyond_the_range_is_reported_once_per_rule_with_its_time(cfg):
    r = preflight.check_plan(cfg, FOLD, ramp({"j1": -3.76}, T=12.0))
    assert r["problems"] and all(p.startswith("t=") and " j1: " in p for p in r["problems"])
    assert sum("j1: target" in p for p in r["problems"]) == 1       # reported once, not on every tick
    assert any("stopping distance" in p for p in r["problems"])     # the speed bound fires first, near the limit


def test_too_fast_is_reported(cfg):
    r = preflight.check_plan(cfg, FOLD, ramp({"j5": 1.2}, T=0.8))     # min-jerk peak 2.8 rad/s > 2.0
    assert any("j5: vel" in p for p in r["problems"])


def test_a_dive_into_the_table_is_reported(cfg):
    r = preflight.check_plan(cfg, FOLD, ramp({"j2": -1.4, "j3": -0.2}, T=8.0))
    assert any("keep-out" in p for p in r["problems"])


def test_startup_and_turn_offset_are_mirrored(cfg):
    start = FOLD | {"j1": 2 * math.pi + 0.05}
    r = preflight.check_plan(cfg, start, ramp({"j1": 0.3}, start=FOLD | {"j1": 0.05},
                                              offset={"j1": 2 * math.pi}))
    assert r["problems"] == [] and math.isclose(r["offsets"]["j1"], 2 * math.pi)
    r = preflight.check_plan(cfg, FOLD | {"j3": -0.7}, ramp({}))
    assert r["problems"] and r["problems"][0].startswith("startup: j3")


def test_speed_near_the_abort_is_warned(cfg):
    r = preflight.check_plan(cfg, FOLD, ramp({"j1": 1.0}, T=1.6))      # peak 1.17 rad/s: legal, within noise
    assert r["problems"] == [] and any(w.startswith("j1:") for w in r["warnings"])


def test_preflight_agrees_with_the_runtime(cfg):
    """Whatever the pre-flight passes, the live guard (fake bus, tracking with a lag) passes too."""
    plan = list(ramp({"j2": -0.6, "j3": -1.0, "j5": 0.5, "j6": -0.8}, T=5.0))
    assert preflight.check_plan(cfg, FOLD, plan)["problems"] == []
    rig = make_rig(cfg)
    rig.enable_all()
    for t, cmds in plan:
        for name, (pos, vel, kp, kd, tau) in cmds.items():
            rig.m[name].send_mit(pos, vel, kp, kd, tau)
        rig.step(0.005)
    assert not rig.guard.latched

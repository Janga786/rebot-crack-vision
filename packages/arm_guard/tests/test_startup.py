"""S1 startup check, whole-turn offsets (J1), the droop tolerance (J2), the shrinking target window, and caller
register reads counting as liveness. Numbers come from the real starts logged 2026-10-14..21 (config comments)."""

import math

import pytest

from arm_guard import GuardFault, GuardRefused, core
from conftest import make_rig

TURN = 2 * math.pi


def tick(rig, n=1, dt=0.002):
    for _ in range(n):
        rig.step(dt)


# ---- core: startup_check ---------------------------------------------------------------------------------------

FOLD = {"j1": 0.012, "j2": 0.002, "j3": -0.010, "j4": 0.012, "j5": 0.037, "j6": 0.001, "j7": -0.002}


def test_fold_passes_without_offsets(cfg):
    offsets, problems = core.startup_check(cfg, FOLD)
    assert problems == [] and set(offsets.values()) == {0.0}


@pytest.mark.parametrize("j1, k", [(6.335, 1), (6.278, 1), (6.272, 1), (-6.20, -1), (-0.08, 0)])
def test_j1_unwrapped_by_whole_turns(cfg, j1, k):
    offsets, problems = core.startup_check(cfg, FOLD | {"j1": j1})
    assert problems == [] and offsets["j1"] == pytest.approx(k * TURN)


def test_j1_turned_by_hand_refused(cfg):
    _, problems = core.startup_check(cfg, FOLD | {"j1": 2.768})      # 2026-10-18 20:34Z: base turned while off
    assert len(problems) == 1 and problems[0].startswith("j1:")


def test_j1_two_turns_off_refused_by_mit_mapping(cfg):
    _, problems = core.startup_check(cfg, FOLD | {"j1": 2 * TURN + 0.03})
    assert problems and "MIT position mapping" in problems[0]


def test_only_turn_ambiguous_joints_are_unwrapped(cfg):
    _, problems = core.startup_check(cfg, FOLD | {"j6": TURN + 0.01})
    assert problems and problems[0].startswith("j6:")


@pytest.mark.parametrize("name, q", [("j2", 0.395), ("j3", -0.347), ("j6", -0.259), ("j4", 0.18), ("j7", -0.202)])
def test_every_logged_start_passes(cfg, name, q):
    assert core.startup_check(cfg, FOLD | {name: q})[1] == []


@pytest.mark.parametrize("name, q", [("j2", 0.6), ("j3", -0.5), ("j3", 0.08), ("j5", 1.0), ("j7", -1.0)])
def test_not_folded_refused(cfg, name, q):
    _, problems = core.startup_check(cfg, FOLD | {name: q})
    assert len(problems) == 1 and problems[0].startswith(f"{name}:")


def test_missing_reading_refused(cfg):
    _, problems = core.startup_check(cfg, FOLD | {"j4": None})
    assert problems == ["j4: no position reading"]


# ---- core: droop tolerance and the remembered window -------------------------------------------------------------

def test_j2_droop_is_a_healthy_state_but_j3_past_its_tolerance_is_not(cfg):
    j2, j3 = cfg.joints["j2"], cfg.joints["j3"]
    assert core.check_state(j2, core.Feedback(0.45, 0.0, 0.0, 0, 0.0), cfg) == []
    assert core.check_state(j3, core.Feedback(0.08, 0.0, 0.0, 0, 0.0), cfg)


def test_drooped_j2_may_hold_or_lift_never_push_further(cfg):
    j = cfg.joints["j2"]
    fb = core.Feedback(0.40, 0.0, 0.0, 1, 0.0)
    ok = lambda q: core.check_command(j, core.Command("mit", pos=q, kp=100, kd=1), fb, cfg).ok
    assert ok(0.40) and ok(0.41) and ok(0.30) and not ok(0.42)


def test_window_argument_is_the_runtime_allowance_bounded_by_rest_tolerance(cfg):
    j = cfg.joints["j2"]
    overshoot = core.Feedback(0.005, 0.0, 0.0, 1, 0.0)            # measured just past the limit
    cmd = core.Command("mit", pos=0.01, kp=100, kd=1)
    assert core.check_command(j, cmd, overshoot, cfg).ok              # no allowance: measured + slack
    assert not core.check_command(j, cmd, overshoot, cfg, window=(j.lower, 0.0)).ok
    assert core.check_command(j, core.Command("mit", pos=0.0, kp=100, kd=1), overshoot, cfg, window=(j.lower, 0.0)).ok
    absurd = (j.lower - 9.0, j.upper + 9.0)                           # never beyond rest_tolerance, whatever is passed
    assert not core.check_command(j, core.Command("mit", pos=0.6, kp=100, kd=1), overshoot, cfg, window=absurd).ok


def test_float_rounding_at_an_exact_limit_is_not_a_violation(cfg):
    j = cfg.joints["j3"]
    fb = core.Feedback(-0.01, 0.0, 0.0, 1, 0.0)
    assert core.check_command(j, core.Command("mit", pos=1.1e-16, kp=80, kd=1), fb, cfg).ok
    assert not core.check_command(j, core.Command("mit", pos=1e-6, kp=80, kd=1), fb, cfg).ok


def test_allowance_follows_targets_inward_with_a_fold_rest_floor(cfg):
    j = cfg.joints["j2"]
    a = core.initial_allowance(j, 0.40, cfg)                          # drooped start
    assert a[1] == pytest.approx(0.41) and a[3] == pytest.approx(0.05)
    for p in (0.40, 0.30, 0.10, 0.0, -0.3):
        a = core.shrink_allowance(a, p, cfg)
    assert a[1] == pytest.approx(0.05)                                # back into the droop: no; to a fold rest: yes
    b = core.initial_allowance(j, 0.002, cfg)                         # ordinary fold rest
    for p in (0.0, -0.6, 0.0):
        b = core.shrink_allowance(b, p, cfg)
    assert b[1] == pytest.approx(0.012)                               # may return to where it rested
    c = core.initial_allowance(j, -0.2, cfg)                          # started inside: never past the limit
    assert core.shrink_allowance(c, 0.0, cfg)[1] == 0.0


# ---- runtime: the startup check gates enable --------------------------------------------------------------------

def test_enable_refused_when_not_folded_and_nothing_enabled(cfg):
    r = make_rig(cfg, start={"j3": -0.6})
    with pytest.raises(GuardRefused, match="startup check failed"):
        r.m["j1"].enable()
    assert all(m.status == 0 for m in r.m.values())
    assert any(e["kind"] == "refused" and "startup" in e["msg"] for e in r.guard.events)


def test_enable_all_refused_when_not_folded(cfg):
    r = make_rig(cfg, start={"j1": 2.768})
    with pytest.raises(GuardRefused, match="j1: reads \\+2.768"):
        r.ctl.enable_all()
    assert all(m.status == 0 for m in r.m.values())


def test_j1_one_turn_off_runs_in_physical_coordinates(cfg):
    r = make_rig(cfg, start={"j1": TURN + 0.05})
    r.enable_all()
    assert r.guard.status()["turn_offsets"] == {"/dev/fake-arm": {"j1": pytest.approx(TURN)}}
    r.m["j1"].send_mit(TURN + 0.10, 0.0, 100.0, 4.0, 0.0)              # raw command, physical +0.10: fine
    assert r.m["j1"].sent[-1][1][0] == pytest.approx(TURN + 0.10)      # sent unchanged, in raw coordinates
    tick(r, 30)
    for k in range(1, 40):                                            # walk toward physical +2.9 (> 2.8)
        q = r.m["j1"].pos
        try:
            r.m["j1"].send_mit(q + 0.1, 0.0, 100.0, 4.0, 0.0)
        except GuardFault as e:
            assert "j1: target" in str(e) and "outside" in str(e)
            break
        tick(r, 30)
    else:
        pytest.fail("never refused")
    hold = r.m["j1"].sent[-1][1][0]
    assert TURN + 2.5 < hold <= TURN + 2.8 + 1e-9                       # the hold is sent in raw coordinates too
    assert r.m["j1"].pos <= TURN + 2.8 + 1e-6


def test_j2_drooped_start_lifts_but_cannot_be_ratcheted_down(cfg):
    r = make_rig(cfg, start={"j2": 0.40})
    r.enable_all()
    m = r.m["j2"]
    m.send_mit(0.40, 0.0, 100.0, 1.0, 0.0)                             # hold where it rests
    with pytest.raises(GuardFault):                                    # push further: refused, held, latched
        for _ in range(20):
            m.send_mit(m.pos + 0.009, 0.0, 100.0, 1.0, 0.0)
            tick(r, 15)
    assert m.pos <= 0.41 + 1e-9
    r.guard.reset("test: operator lifts the arm")
    for _ in range(200):                                               # lift back to the fold
        target = max(0.0, m.pos - 0.02)
        m.send_mit(target, -0.4, 100.0, 1.0, 0.0)
        tick(r, 5)
    assert m.pos == pytest.approx(0.0, abs=1e-3)
    tick(r, 5)
    with pytest.raises(GuardFault, match="j2: target"):               # lifted: no way back into the droop
        m.send_mit(0.08, 0.0, 100.0, 1.0, 0.0)


def test_no_outward_ratchet_from_an_overshoot(cfg):
    r = make_rig(cfg)
    r.enable_all()
    m = r.m["j3"]
    m.send_mit(0.0, 0.0, 80.0, 1.0, 0.0)
    m.pos = 0.004                                                       # the plant overshoots the limit a little
    with pytest.raises(GuardFault, match="j3: target"):
        m.send_mit(0.012, 0.0, 80.0, 1.0, 0.0)                         # old rule allowed measured + rest_slack


def test_measured_leading_the_reference_into_the_range_is_fine(cfg):
    """The fold return found by the 2026-10-21 rehearsal: J2 rests at +0.002, the reference comes down to 0 while
    the joint is already measured inside the limit."""
    r = make_rig(cfg, start={"j2": 0.002})
    r.enable_all()
    m = r.m["j2"]
    m.send_mit(0.002, 0.0, 100.0, 1.0, 0.0)
    m.pos = -0.004                                                      # the joint got there first
    for ref in (0.0015, 0.001, 0.0005, 1.1e-16, 0.0):
        m.send_mit(ref, 0.0, 100.0, 1.0, 0.0)
    assert not r.guard.latched


def test_new_session_rechecks_and_takes_a_new_offset(cfg):
    r = make_rig(cfg)
    r.enable_all()
    tick(r, 5)
    r.ctl.disable_all()
    r.clock.t += 5.0                                                   # 24 V cycled while the process kept running
    r.m["j1"].pos += TURN
    r.m["j1"].enable()
    assert r.guard.status()["turn_offsets"]["/dev/fake-arm"]["j1"] == pytest.approx(TURN)


def test_session_under_way_is_not_rechecked(cfg):
    r = make_rig(cfg)
    r.m["j1"].enable()
    r.m["j1"].send_mit(0.1, 0.0, 100.0, 4.0, 0.0)
    for _ in range(40):
        tick(r)
    r.m["j3"].pos = -0.6                                                # a raised joint while j1 is enabled
    r.m["j3"].enable()                                                  # no startup re-check mid-session


# ---- runtime: caller register reads keep frozen-but-alive feedback fresh -----------------------------------------

def test_callers_heartbeat_counts_and_guard_adds_none(rig, monkeypatch):
    rig.enable_all()
    m = rig.m["j4"]
    m.frozen = True
    reads = []
    orig = rig.guard.raw.get_register_u32
    monkeypatch.setattr(rig.guard.raw, "get_register_u32", lambda *a: reads.append(a) or orig(*a))
    for k in range(500):                                               # 1 s at 500 Hz, caller reads every 50 ms
        if k % 25 == 0:
            m.get_register_u32(10, 10)
        m.send_mit(0.0, 0.0, 40.0, 1.0, 0.0)
        tick(rig)
    assert not rig.guard.latched and reads == []

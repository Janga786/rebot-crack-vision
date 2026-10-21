"""Unit tests of the pure rules R1-R7 (arm_guard.core) against the real repo config."""

import math

import pytest

from arm_guard import core
from arm_guard.core import Command, Feedback

FB0 = Feedback(0.0, 0.0, 0.0, core.ST_ENABLED, 0.0)


def fb(pos, vel=0.0, torq=0.0, status=core.ST_ENABLED, age=0.0):
    return Feedback(pos, vel, torq, status, age)


def mit(pos, vel=0.0, kp=40.0, kd=1.0, tau=0.0):
    return Command("mit", pos=pos, vel=vel, kp=kp, kd=kd, tau=tau)


def ok(cfg, joint, cmd, feedback):
    return core.check_command(cfg.joints[joint], cmd, feedback, cfg)


# ---- R1 non-finite -----------------------------------------------------------------------------------------

@pytest.mark.parametrize("field", ["pos", "vel", "kp", "kd", "tau", "vlim", "ratio"])
@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_refused(cfg, field, bad):
    c = Command("mit", **{field: bad})
    assert not ok(cfg, "j4", c, FB0).ok


def test_no_measured_position_refused(cfg):
    assert not ok(cfg, "j4", mit(0.0), None).ok
    assert not ok(cfg, "j4", mit(0.0), fb(math.nan)).ok


def test_unknown_mode_refused(cfg):
    assert not ok(cfg, "j4", Command("torque", pos=0.0), FB0).ok


# ---- R2 position window ------------------------------------------------------------------------------------

def test_target_outside_limits_refused(cfg):
    j = cfg.joints["j1"]
    assert not ok(cfg, "j1", mit(j.upper + 0.001, kp=100), fb(j.upper - 0.05)).ok
    assert ok(cfg, "j1", mit(j.upper, kp=100), fb(j.upper - 0.05)).ok


def test_rest_pose_past_limit_can_be_held_but_not_pushed(cfg):
    # J2 rests at +0.0235 in the fold (upper limit 0.0), measured 2026-10-17
    rest = fb(0.0235)
    assert ok(cfg, "j2", mit(0.0235, kp=100), rest).ok                 # hold at the reading
    assert ok(cfg, "j2", mit(0.0300, kp=100), rest).ok                 # within rest_slack (sensor noise)
    assert not ok(cfg, "j2", mit(0.0400, kp=100), rest).ok             # pushing further out
    assert ok(cfg, "j2", mit(-0.05, kp=100), rest).ok                  # moving inward


def test_window_never_exceeds_rest_tolerance(cfg):
    j = cfg.joints["j2"]
    lo, hi = core.target_window(j, 0.2, cfg.rest_tolerance_rad, cfg.rest_slack_rad)
    assert hi == pytest.approx(j.upper + cfg.rest_tolerance_rad)


# ---- R3 step -----------------------------------------------------------------------------------------------

def test_step_beyond_max_step_refused(cfg):
    j = cfg.joints["j3"]
    assert ok(cfg, "j3", mit(-1.0 + j.max_step * 0.99, kp=80), fb(-1.0)).ok
    assert not ok(cfg, "j3", mit(-1.0 + j.max_step * 1.01, kp=80), fb(-1.0)).ok


def test_ik_wrist_flip_refused(cfg):
    # IK returns the other wrist branch: joint6 jumps by pi
    assert not ok(cfg, "j6", mit(-1.5), fb(1.5)).ok
    assert not ok(cfg, "j6", Command("pos_vel", pos=-1.5, vlim=1.0), fb(1.5)).ok


# ---- R4 speed / stopping distance --------------------------------------------------------------------------

def test_mit_speed_cap(cfg):
    j = cfg.joints["j4"]
    assert ok(cfg, "j4", mit(0.0, vel=j.vel_cap), FB0).ok
    assert not ok(cfg, "j4", mit(0.0, vel=j.vel_cap + 0.01), FB0).ok


def test_speed_toward_limit_must_be_stoppable(cfg):
    j = cfg.joints["j1"]
    near = j.upper - 0.01                          # 1 cm-ish from the limit
    v_ok = math.sqrt(2 * j.accel_stop * 0.01)
    assert ok(cfg, "j1", mit(near, vel=v_ok * 0.98, kp=100), fb(near)).ok
    assert not ok(cfg, "j1", mit(near, vel=v_ok * 1.05, kp=100), fb(near)).ok
    assert ok(cfg, "j1", mit(near, vel=-j.vel_cap, kp=100), fb(near)).ok     # away from the limit is fine


def test_pos_vel_vlim_lowered_to_cap(cfg):
    j = cfg.joints["j1"]
    v = ok(cfg, "j1", Command("pos_vel", pos=0.1, vlim=5.0), fb(0.0))        # the vendor default vlim
    assert v.ok and v.cmd.vlim == pytest.approx(j.vel_cap) and v.adjustments
    assert v.cmd.pos == 0.1


def test_pos_vel_vlim_lowered_near_limit(cfg):
    j = cfg.joints["j1"]
    v = ok(cfg, "j1", Command("pos_vel", pos=j.upper, vlim=5.0), fb(j.upper - 0.001))
    assert v.ok and v.cmd.vlim <= max(cfg.min_vlim, math.sqrt(2 * j.accel_stop * 0.001)) + 1e-9


def test_pos_vel_zero_vlim_refused(cfg):
    assert not ok(cfg, "j1", Command("pos_vel", pos=0.0, vlim=0.0), FB0).ok


def test_velocity_mode_refused_by_default(cfg):
    assert not ok(cfg, "j4", Command("vel", vel=0.1), FB0).ok


# ---- R5 gains / feed-forward -------------------------------------------------------------------------------

def test_kp_above_max_refused(cfg):
    j = cfg.joints["j4"]
    assert not ok(cfg, "j4", mit(0.0, kp=j.kp_max + 1), FB0).ok
    assert not ok(cfg, "j4", mit(0.0, kp=-1), FB0).ok


def test_kd_lowered_not_refused(cfg):
    v = ok(cfg, "j1", mit(0.0, kp=120, kd=8.0), FB0)                          # the vendor default kd
    assert v.ok and v.cmd.kd == cfg.joints["j1"].kd_max and v.cmd.kp == 120


def test_tau_ff_cap(cfg):
    j = cfg.joints["j3"]
    assert ok(cfg, "j3", mit(-1.0, kp=80, tau=j.tau_ff_max), fb(-1.0)).ok
    assert not ok(cfg, "j3", mit(-1.0, kp=80, tau=-j.tau_ff_max - 0.1), fb(-1.0)).ok


def test_force_pos_ratio(cfg):
    assert ok(cfg, "j7", Command("force_pos", pos=-0.5, vlim=1.0, ratio=0.3), fb(-0.5)).ok
    assert not ok(cfg, "j7", Command("force_pos", pos=-0.5, vlim=1.0, ratio=0.9), fb(-0.5)).ok


# ---- R6 keep-out -------------------------------------------------------------------------------------------

def q(**kw):
    out = {f"joint{i}": 0.0 for i in range(1, 7)}
    out.update({f"joint{k[1:]}": v for k, v in kw.items()})
    return out


def test_fold_is_clear(cfg):
    assert min(core.pose_margins(cfg, q()).values()) > 0


def test_shoulder_down_into_table_refused(cfg):
    # J2 = -1.57 alone points the arm straight down: gripper_link 0.22 m below the floor margin
    assert core.check_pose(cfg, q(j2=-1.57), q())


def test_raised_pose_clear(cfg):
    assert not core.check_pose(cfg, q(j2=-0.6, j3=-1.2), q())


def test_existing_violation_may_improve_not_worsen(cfg):
    bad = q(j2=-1.2)
    worse, better = q(j2=-1.3), q(j2=-1.1)
    assert core.check_pose(cfg, bad, bad) == []          # holding still is allowed
    assert core.check_pose(cfg, better, bad) == []
    assert core.check_pose(cfg, worse, bad)


# ---- R7 measured state -------------------------------------------------------------------------------------

@pytest.mark.parametrize("status", [0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE, 0x5])
def test_fault_status(cfg, status):
    assert core.check_state(cfg.joints["j1"], fb(0.0, status=status), cfg)


def test_state_limits(cfg):
    j = cfg.joints["j1"]
    assert core.check_state(j, fb(0.0), cfg) == []
    assert core.check_state(j, fb(-3.76), cfg)                                # J1 turned by hand to the audience
    assert core.check_state(j, fb(0.0, vel=j.vel_abort + 0.1), cfg)
    assert core.check_state(j, fb(0.0, torq=-j.torque_abort - 0.1), cfg)
    assert core.check_state(j, fb(0.0, age=cfg.stale_feedback_s + 0.01), cfg)
    assert core.check_state(j, None, cfg)
    assert core.check_state(j, fb(math.nan), cfg)


# ---- hold --------------------------------------------------------------------------------------------------

def test_hold_commands(cfg):
    j = cfg.joints["j3"]
    h = core.hold_command(j, "mit", -1.0, tau=50.0)
    assert (h.pos, h.vel, h.kp, h.kd) == (-1.0, 0.0, j.hold_kp, j.hold_kd) and h.tau == j.tau_ff_max
    assert core.hold_command(j, "pos_vel", -1.0).vlim == 0.2
    assert core.hold_command(j, "mit", -1.0, tau=math.nan).tau == 0.0

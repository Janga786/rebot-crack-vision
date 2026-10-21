"""Property tests (seeded random + adversarial generators; hypothesis isn't in the pinned envs).

P1  whatever goes in, an accepted command satisfies R1-R5 exactly (checked independently here).
P2  the only changes ever made are vlim and kd going down; position/velocity/kp/tau pass through untouched.
P3  the rules are deterministic.
P4  closed loop: a hostile "IK" sending garbage to every joint at 500 Hz through the patched motor classes into a
    simulated arm never drives a joint outside its limits (+ rest tolerance), never exceeds the speed caps, and
    keeps every keep-out margin above -KEEPOUT_SLACK_M.
"""

import math
import random

import pytest

from arm_guard import GuardFault, GuardRefused, core
from arm_guard.core import Command, Feedback
from conftest import make_rig

N_COMMANDS = 60_000
SEEDS = (1, 2, 3)
KEEPOUT_SLACK_M = 0.010   # checked endpoint-to-endpoint at 500 Hz; the in-between motion may dip this much


def _num(rng, scale):
    r = rng.random()
    if r < 0.03:
        return rng.choice([math.nan, math.inf, -math.inf, 1e300, -1e300, 0.0, -0.0])
    if r < 0.5:
        return rng.uniform(-scale, scale)
    return rng.gauss(0.0, scale / 10)


def _cmd(rng, j, q):
    mode = rng.choice(["mit", "mit", "pos_vel", "force_pos", "vel"])
    near = rng.random() < 0.5
    pos = q + rng.gauss(0, j.max_step / 2) if near else _num(rng, 2 * math.pi)
    if rng.random() < 0.1:
        pos = rng.choice([j.lower, j.upper, j.lower - 1e-6, j.upper + 1e-6])
    return Command(mode, pos=pos, vel=_num(rng, 3 * j.vel_cap), kp=_num(rng, 2 * j.kp_max), kd=_num(rng, 10),
                   tau=_num(rng, 2 * j.tau_ff_max), vlim=_num(rng, 10), ratio=_num(rng, 1.0))


def _fb(rng, j, rest_tol):
    if rng.random() < 0.3:
        q = rng.choice([j.lower, j.upper]) + rng.uniform(-rest_tol, rest_tol)
    else:
        q = rng.uniform(j.lower, j.upper)
    return Feedback(q, rng.uniform(-j.vel_cap, j.vel_cap), 0.0, core.ST_ENABLED, 0.0)


def _assert_rules(cfg, j, cmd_in, fb, out):
    lo, hi = core.target_window(j, fb.pos, cfg.rest_tolerance_rad, cfg.rest_slack_rad)
    for x in (out.pos, out.vel, out.kp, out.kd, out.tau, out.vlim, out.ratio):
        assert math.isfinite(x)
    if out.mode == "vel":
        assert cfg.allow_velocity_mode
        return
    assert lo <= out.pos <= hi
    assert j.lower - cfg.rest_tolerance_rad <= out.pos <= j.upper + cfg.rest_tolerance_rad
    assert abs(out.pos - fb.pos) <= j.max_step + 1e-12
    if out.mode == "mit":
        assert 0.0 <= out.kp <= j.kp_max and 0.0 <= out.kd <= j.kd_max and abs(out.tau) <= j.tau_ff_max
        assert abs(out.vel) <= j.vel_cap + 1e-12
        d = (hi - out.pos) if out.vel > 0 else (out.pos - lo)
        assert abs(out.vel) <= max(cfg.min_vlim, math.sqrt(2 * j.accel_stop * max(d, 0.0))) + 1e-9
    else:
        assert cfg.min_vlim * 0 < out.vlim <= j.vel_cap + 1e-12
        d = (hi - fb.pos) if out.pos > fb.pos else (fb.pos - lo)
        assert out.vlim <= max(cfg.min_vlim, math.sqrt(2 * j.accel_stop * max(d, 0.0))) + 1e-9
        if out.mode == "force_pos":
            assert 0.0 <= out.ratio <= cfg.force_pos_ratio_max
    # P2: only vlim/kd may change, and only downward
    assert (out.mode, out.pos, out.vel, out.kp, out.tau, out.ratio) == \
        (cmd_in.mode, cmd_in.pos, cmd_in.vel, cmd_in.kp, cmd_in.tau, cmd_in.ratio)
    assert out.kd <= cmd_in.kd and out.vlim <= cmd_in.vlim


@pytest.mark.parametrize("seed", SEEDS)
def test_accepted_commands_satisfy_every_rule(cfg, seed):
    rng = random.Random(seed)
    joints = list(cfg.joints.values())
    accepted = 0
    for _ in range(N_COMMANDS):
        j = rng.choice(joints)
        fb = _fb(rng, j, cfg.rest_tolerance_rad)
        c = _cmd(rng, j, fb.pos)
        v = core.check_command(j, c, fb, cfg)
        assert v.ok == (not v.reasons)
        if v.ok:
            accepted += 1
            _assert_rules(cfg, j, c, fb, v.cmd)
        assert core.check_command(j, c, fb, cfg) == v                    # P3
    assert accepted > N_COMMANDS * 0.03, "generator too hostile to exercise the accept path"


def test_random_poses_keepout_consistent(cfg):
    rng = random.Random(7)
    arm = cfg.arm_joints
    for _ in range(5_000):
        q_now = {j.urdf_joint: rng.uniform(j.lower, j.upper) for j in arm}
        q_new = dict(q_now)
        k = rng.choice(arm).urdf_joint
        q_new[k] += rng.uniform(-0.15, 0.15)
        reasons = core.check_pose(cfg, q_new, q_now)
        new, now = core.pose_margins(cfg, q_new), core.pose_margins(cfg, q_now)
        for name in new:
            if new[name] < 0 and new[name] < now[name] - 1e-9:
                assert any(name in r for r in reasons)
        if not reasons:
            assert all(new[n] >= 0 or new[n] >= now[n] - 1e-9 for n in new)


def _hostile_targets(rng, cfg, motors):
    """One tick of commands from a broken IK/planner: mostly plausible small moves, plus teleports to random IK
    branches, dives toward the table, NaN, absurd gains, overspeed."""
    out = {}
    for name, m in motors.items():
        j = cfg.joints[name]
        r = rng.random()
        q = m.pos
        if r < 0.70:
            pos = q + rng.uniform(-1, 1) * j.max_step * 0.6
        elif r < 0.80:
            pos = rng.uniform(-2 * math.pi, 2 * math.pi)                     # another IK branch
        elif r < 0.88:
            pos = q + (-1 if name in ("j2", "j3") else 1) * j.max_step * 0.9  # dive
        elif r < 0.90:
            pos = math.nan
        else:
            pos = q + rng.uniform(-1, 1) * j.max_step * 0.3
        kp = j.hold_kp if rng.random() > 0.03 else j.kp_max * 5
        vel = rng.uniform(-1, 1) * j.vel_cap * (1.0 if rng.random() > 0.05 else 4.0)
        tau = rng.uniform(-1, 1) * j.tau_ff_max * (1.0 if rng.random() > 0.03 else 3.0)
        out[name] = (pos, vel, kp, j.hold_kd, tau)
    return out


@pytest.mark.parametrize("seed", SEEDS)
def test_closed_loop_hostile_planner_never_breaks_limits(cfg, seed):
    rng = random.Random(seed)
    rig = make_rig(cfg)
    rig.enable_all()
    worst = {"margin": math.inf, "faults": 0, "accepted": 0}
    dt = 0.002
    for step in range(6_000):                                 # 12 s at 500 Hz
        for name, args in _hostile_targets(rng, cfg, rig.m).items():
            try:
                rig.m[name].send_mit(*args)
                worst["accepted"] += 1
            except GuardFault:
                worst["faults"] += 1
            except GuardRefused:
                pass
        rig.step(dt)
        for name, m in rig.m.items():
            j = cfg.joints[name]
            assert j.lower - cfg.rest_tolerance_rad - 1e-9 <= m.pos <= j.upper + cfg.rest_tolerance_rad + 1e-9, \
                (step, name, m.pos)
            assert abs(m.vel) <= max(j.vel_cap, j.max_step / 0.02) + 1e-9
        q = {cfg.joints[n].urdf_joint: m.pos for n, m in rig.m.items() if cfg.joints[n].arm}
        worst["margin"] = min(worst["margin"], min(core.pose_margins(cfg, q).values()))
        assert worst["margin"] >= -KEEPOUT_SLACK_M, (step, worst)
        if rig.guard.latched and rng.random() < 0.05:         # an operator resets now and then: keep exploring
            rig.guard.reset("fuzz: continue")
    assert worst["faults"] > 0 and worst["accepted"] > 1000, worst


@pytest.mark.parametrize("seed", SEEDS)
def test_closed_loop_pos_vel_hostile(cfg, seed):
    """Same in POS_VEL mode (the vendor driver's mode): the motor runs at vlim toward whatever target it got."""
    from motorbridge.models import Mode
    rng = random.Random(100 + seed)
    rig = make_rig(cfg)
    for m in rig.m.values():
        m.ensure_mode(Mode.POS_VEL)
    rig.enable_all()
    for step in range(4_000):
        for name, (pos, vel, kp, kd, tau) in _hostile_targets(rng, cfg, rig.m).items():
            try:
                rig.m[name].send_pos_vel(pos, rng.choice([5.0, 3.0, 0.5, abs(vel)]))
            except (GuardFault, GuardRefused):
                pass
        rig.step(0.002)
        for name, m in rig.m.items():
            j = cfg.joints[name]
            assert j.lower - cfg.rest_tolerance_rad - 1e-9 <= m.pos <= j.upper + cfg.rest_tolerance_rad + 1e-9
            assert abs(m.vel) <= j.vel_cap + 1e-9, (step, name, m.vel)
        q = {cfg.joints[n].urdf_joint: m.pos for n, m in rig.m.items() if cfg.joints[n].arm}
        assert min(core.pose_margins(cfg, q).values()) >= -KEEPOUT_SLACK_M
        if rig.guard.latched and rng.random() < 0.05:
            rig.guard.reset("fuzz: continue")


@pytest.mark.parametrize("seed", SEEDS)
def test_closed_loop_smooth_planner_into_obstacles(cfg, seed):
    """A well-behaved-looking planner (small steps, legal speeds) heading straight (in joint space) for goals that
    lie inside the table or the arm itself. The guard must stop every episode at the keep-out boundary."""
    rng = random.Random(200 + seed)
    arm = [n for n in cfg.joints if cfg.joints[n].arm]
    episodes, keepout_stops = 6, 0
    for episode in range(episodes):
        while True:
            goal = {n: rng.uniform(cfg.joints[n].lower, cfg.joints[n].upper) for n in cfg.joints}
            if min(core.pose_margins(cfg, {cfg.joints[n].urdf_joint: goal[n] for n in arm}).values()) < -0.03:
                break
        rig = make_rig(cfg, channel=f"/dev/fake-ep{seed}-{episode}")
        rig.enable_all()
        for step in range(4000):                                # up to 8 s at 500 Hz, 0.6 rad/s
            for name, m in rig.m.items():
                j = cfg.joints[name]
                d = max(-0.0012, min(0.0012, goal[name] - m.pos))
                try:
                    m.send_mit(m.pos + 8 * d, d / 0.002, j.hold_kp, j.hold_kd, 0.0)
                except (GuardFault, GuardRefused):
                    pass
            rig.step(0.002)
            q = {cfg.joints[n].urdf_joint: m.pos for n, m in rig.m.items() if cfg.joints[n].arm}
            assert min(core.pose_margins(cfg, q).values()) >= -KEEPOUT_SLACK_M, (episode, step)
            if rig.guard.latched:
                break
        assert rig.guard.latched, episode          # stopped (keep-out, or a joint limit reached first)
        keepout_stops += "keep-out" in rig.guard.latched
    assert keepout_stops >= 1

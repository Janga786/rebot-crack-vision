"""The guard installed on (fake) motorbridge classes: allowlist, enable gate, config writes, port lock, fault latch."""

import pytest

from arm_guard import GuardConfigError, GuardFault, GuardRefused, runtime
from arm_guard.core import ST_ENABLED
from arm_guard.patch import MOTOR_GUARDED, MOTOR_LIVENESS_READS, MOTOR_PASSTHROUGH
from conftest import make_rig
from motorbridge import CallError
from motorbridge.models import Mode


def tick(rig, n=1, dt=0.002):
    for _ in range(n):
        rig.step(dt)


# ---- allowlist ---------------------------------------------------------------------------------------------

def test_every_public_motor_method_is_classified(rig):
    for name in dir(rig.Motor):
        if name.startswith("_") or not callable(getattr(rig.Motor, name)):
            continue
        fn = getattr(rig.Motor, name)
        if name in MOTOR_LIVENESS_READS:     # pass-through, only noting that the link answered
            assert getattr(fn, "__wrapped__", None) and not getattr(fn, "_arm_guard_refuser", False), name
        elif name in MOTOR_PASSTHROUGH:
            assert not getattr(fn, "__wrapped__", None), name
        else:
            assert name in MOTOR_GUARDED or getattr(fn, "_arm_guard_refuser", False), name


def test_unlisted_methods_refused(rig):
    with pytest.raises(GuardRefused):
        rig.m["j1"].robstride_send_pos_vel_pp(0.0, 1.0, 1.0)
    with pytest.raises(GuardRefused):
        rig.m["j1"].robstride_write_param_f32(1, 1.0)
    with pytest.raises(GuardRefused):
        rig.ctl.add_robstride_motor(1, 1, "x")


def test_guard_errors_are_not_callerror():
    assert not issubclass(GuardFault, CallError) and not issubclass(GuardRefused, CallError)


def test_passthrough_works(rig):
    assert rig.m["j1"].get_state() is not None
    rig.m["j1"].disable()
    rig.ctl.disable_all()


def test_unknown_motor_id_refused(cfg):
    r = make_rig(cfg, joints=["j1"])
    with pytest.raises(GuardRefused):
        r.ctl.add_damiao_motor(0x09, 0x19, "4310")


def test_model_mismatch_refused(cfg):
    r = make_rig(cfg, joints=["j7"])
    with pytest.raises(GuardRefused):
        r.ctl.add_damiao_motor(0x01, 0x11, "4310")


def test_motor_not_from_guarded_controller_refused(rig):
    stray = rig.Motor(0x01, 0x11, "4340P")
    with pytest.raises(GuardRefused):
        stray.send_mit(0.0, 0.0, 10.0, 1.0, 0.0)


# ---- enable gate -------------------------------------------------------------------------------------------

def test_enable_ok_at_fold(rig):
    rig.enable_all()
    assert all(m.status == ST_ENABLED for m in rig.m.values())


def test_enable_all_ok(rig):
    rig.ctl.enable_all()
    assert all(m.status == ST_ENABLED for m in rig.m.values())


def test_enable_refused_outside_limits(cfg):
    r = make_rig(cfg, start={"j1": -3.76})               # J1 turned by hand past the URDF limit
    with pytest.raises(GuardRefused, match="outside"):
        r.m["j1"].enable()
    with pytest.raises(GuardRefused):
        r.ctl.enable_all()
    assert r.m["j1"].status == 0


def test_enable_refused_on_mapping_mismatch(rig):
    rig.m["j2"].regs[21] = 95.5                          # PMAX changed: MIT positions would be mis-scaled
    with pytest.raises(GuardRefused, match="PMAX"):
        rig.m["j2"].enable()


def test_enable_refused_when_link_dead(rig):
    rig.m["j3"].heartbeat_ok = False
    with pytest.raises(GuardRefused):
        rig.m["j3"].enable()


def test_enable_survives_one_dropped_register_reply(rig):
    """2026-10-22, first powered run: J3's CTRL_MODE reply went missing once and the guard refused to enable."""
    rig.m["j3"].reg_failures = 1
    rig.m["j3"].enable()
    assert rig.m["j3"].status == ST_ENABLED
    assert any(e["kind"] == "retry" and "j3" in e["msg"] for e in rig.guard.events)
    rig.m["j4"].reg_failures = 2                         # two misses in a row: the third attempt answers
    rig.m["j4"].enable()
    assert rig.m["j4"].status == ST_ENABLED


def test_enable_refused_when_every_attempt_fails(rig):
    rig.m["j5"].reg_failures = 3
    with pytest.raises(GuardRefused, match="register read failed"):
        rig.m["j5"].enable()
    assert rig.m["j5"].status != ST_ENABLED


def test_enable_refused_in_velocity_mode(rig):
    rig.m["j4"].regs[10] = int(Mode.VEL)
    with pytest.raises(GuardRefused, match="mode"):
        rig.m["j4"].enable()


def test_ensure_mode_velocity_refused(rig):
    with pytest.raises(GuardRefused):
        rig.m["j4"].ensure_mode(Mode.VEL, 1000)
    rig.m["j4"].ensure_mode(Mode.POS_VEL, 1000)
    assert rig.m["j4"].regs[10] == int(Mode.POS_VEL)


def test_command_in_wrong_mode_refused(rig):
    rig.enable_all()                                     # mode read back: MIT
    with pytest.raises(GuardRefused, match="mode"):
        rig.m["j1"].send_pos_vel(0.0, 0.5)


# ---- normal operation ----------------------------------------------------------------------------------------

def test_normal_mit_commands_pass_unchanged(rig):
    rig.enable_all()
    rig.m["j3"].send_mit(-0.05, -0.2, 80.0, 1.0, 3.0)
    assert rig.m["j3"].sent[-1] == ("mit", (-0.05, -0.2, 80.0, 1.0, 3.0))


def test_vendor_vlim_lowered(rig):
    for m in rig.m.values():
        m.ensure_mode(Mode.POS_VEL)
    rig.enable_all()
    rig.m["j1"].send_pos_vel(0.1, 5.0)
    assert rig.m["j1"].sent[-1] == ("pos_vel", (0.1, pytest.approx(1.2)))


def test_require_full_arm(cfg):
    r = make_rig(cfg, joints=["j1"])
    r.m["j1"].enable()
    with pytest.raises(GuardRefused, match="not registered"):
        r.m["j1"].send_mit(0.0, 0.0, 100.0, 4.0, 0.0)
    assert r.m["j1"].sent == []


def test_gripper_alone_allowed(cfg):
    r = make_rig(cfg, joints=["j7"], start={"j7": -0.2})
    r.m["j7"].enable()
    r.m["j7"].send_mit(-0.25, 0.0, 40.0, 0.5, 0.0)
    assert r.m["j7"].sent


# ---- fault latch ---------------------------------------------------------------------------------------------

def test_fault_sends_hold_then_raises_and_latches(rig):
    rig.enable_all()
    tick(rig, 5)
    with pytest.raises(GuardFault):
        rig.m["j6"].send_mit(3.0, 0.0, 40.0, 1.0, 0.0)          # IK flip: 3 rad away
    mode, (pos, vel, kp, kd, tau) = rig.m["j6"].sent[-1]
    assert mode == "mit" and abs(pos) < 1e-3 and vel == 0.0 and kp == 40.0
    assert rig.guard.latched
    # every later command on every joint becomes a hold at the latched position
    rig.m["j3"].send_mit(-0.1, 0.0, 80.0, 1.0, 2.0)
    assert rig.m["j3"].sent[-1][1][0] == pytest.approx(0.0, abs=1e-3)
    with pytest.raises(GuardRefused, match="latched"):
        rig.m["j1"].enable()
    rig.guard.reset("test: operator inspected")
    rig.m["j3"].send_mit(-0.1, 0.0, 80.0, 1.0, 2.0)
    assert rig.m["j3"].sent[-1][1][0] == -0.1


def test_reset_needs_reason(rig):
    with pytest.raises(GuardRefused):
        rig.guard.reset(" ")


def test_keepout_trips(rig):
    rig.enable_all()
    q = 0.0
    with pytest.raises(GuardFault, match="keep-out"):
        for _ in range(2000):                                     # walk J2 down toward the table
            q -= 0.002
            rig.m["j2"].send_mit(q, -1.0, 100.0, 1.0, 0.0)
            tick(rig)
    assert rig.m["j2"].pos > -1.5


def test_stale_feedback_trips(rig):
    rig.enable_all()
    m = rig.m["j1"]
    m.frozen = True
    m.heartbeat_ok = False
    with pytest.raises(GuardFault, match="stale"):
        for _ in range(400):
            m.send_mit(0.0, 0.0, 100.0, 4.0, 0.0)
            tick(rig)


def test_frozen_but_alive_link_is_ok(rig):
    rig.enable_all()
    m = rig.m["j1"]
    m.frozen = True                                               # identical frames, but registers answer
    for _ in range(400):
        m.send_mit(0.0, 0.0, 100.0, 4.0, 0.0)
        tick(rig)
    assert not rig.guard.latched


def test_motor_fault_status_trips(rig):
    rig.enable_all()
    rig.m["j2"].status = 0xA                                      # over-current
    with pytest.raises(GuardFault, match="OVER_CURRENT"):
        rig.m["j2"].send_mit(0.0, 0.0, 100.0, 1.0, 0.0)


def test_no_feedback_ever(rig):
    rig.m["j5"].no_feedback = True
    with pytest.raises(GuardRefused):
        rig.m["j5"].enable()


# ---- configuration writes ------------------------------------------------------------------------------------

def test_zero_and_store_refused_by_default(rig):
    with pytest.raises(GuardRefused):
        rig.m["j1"].set_zero_position()
    with pytest.raises(GuardRefused):
        rig.m["j1"].store_parameters()
    with pytest.raises(GuardRefused):
        rig.m["j1"].set_can_timeout_ms(200)
    assert rig.m["j1"].zeroed == 0 and rig.m["j1"].regs[9] == 0


def test_config_writes_context_motor_disabled_only(rig):
    with rig.guard.config_writes("test: set CAN timeout"):
        rig.m["j1"].set_can_timeout_ms(200)
        assert rig.m["j1"].regs[9] == 4000
        rig.m["j2"].enable()
        with pytest.raises(GuardRefused, match="DISABLED"):
            rig.m["j2"].set_zero_position()
    with pytest.raises(GuardRefused):
        rig.m["j1"].set_can_timeout_ms(0)


def test_config_writes_need_reason(rig):
    with pytest.raises(GuardRefused):
        with rig.guard.config_writes(""):
            pass


def test_vendor_loop_gain_writes_whitelisted(rig):
    rig.m["j1"].write_register_f32(27, 150.0)                    # vendor pos_kp on a POS_VEL switch
    assert rig.m["j1"].regs[27] == 150.0
    with pytest.raises(GuardRefused):
        rig.m["j1"].write_register_f32(27, 5000.0)
    with pytest.raises(GuardRefused):
        rig.m["j1"].write_register_f32(21, 50.0)                  # PMAX is never whitelisted


# ---- one process per bus --------------------------------------------------------------------------------------

def test_second_process_on_same_bus_refused(cfg):
    make_rig(cfg, channel="/dev/fake-shared")
    with pytest.raises(GuardRefused, match="already in use"):
        make_rig(cfg, channel="/dev/fake-shared")


def test_same_process_second_controller_shares_bus(rig):
    other = rig.Controller.from_dm_serial("/dev/fake-arm")
    assert other._arm_guard_bus is rig.ctl._arm_guard_bus


# ---- no config -> no motion ----------------------------------------------------------------------------------

def test_missing_config_refuses_everything_but_disable(cfg):
    from motorbridge import core as fake_core

    from arm_guard.patch import patch
    Controller, Motor = fake_core.make_classes()
    g = runtime.Guard(None, config_error="test: unapproved")
    patch(Controller, Motor, g)
    with pytest.raises(GuardConfigError):
        Controller.from_dm_serial("/dev/fake-noconfig")
    m = Motor(1, 0x11, "4340P")
    with pytest.raises(GuardConfigError):
        m.send_mit(0.0, 0.0, 1.0, 0.0, 0.0)
    with pytest.raises(GuardConfigError):
        m.enable()
    m.disable()


def test_events_logged(rig, tmp_path):
    rig.enable_all()
    with pytest.raises(GuardFault):
        rig.m["j1"].send_mit(1.0, 0.0, 100.0, 4.0, 0.0)
    log = (tmp_path / "events.jsonl").read_text()
    assert '"kind": "fault"' in log

"""arm_guard.patch — put the guard in front of motorbridge's Controller/Motor classes, in place.

Patching the classes (not wrapping instances) means every caller in the process is covered: the vendor SDK,
the vendor ROS driver, the LeRobot follower, operator scripts, an interactive shell. The method lists are an
allowlist: a public method that is neither pass-through (reads, disable, close) nor explicitly guarded is
replaced by a refusal, so a new motorbridge release cannot add an unguarded path by accident.

Bypasses that remain (documented in docs/motion/ARM_GUARD.md): calling motorbridge's C ABI directly
(`get_abi().lib.motor_handle_send_*`) and the Rust tools (`ws_gateway`, `motor_cli`) that never load Python.
"""

from __future__ import annotations

import functools
import inspect
import math

from .core import Command
from .errors import GuardFault, GuardRefused

MOTOR_PASSTHROUGH = {
    "close", "get_state", "request_feedback", "disable", "clear_error",
    "get_register_f32", "get_register_u32", "damiao_get_param_f32", "damiao_get_param_u32",
}
MOTOR_LIVENESS_READS = {"get_register_f32", "get_register_u32", "damiao_get_param_f32", "damiao_get_param_u32"}
MOTOR_SENDS = {"send_mit", "send_pos_vel", "send_vel", "send_force_pos"}
MOTOR_CONFIG_WRITES = {
    "set_zero_position", "store_parameters", "set_can_timeout_ms",
    "write_register_f32", "write_register_u32", "damiao_write_param_f32", "damiao_write_param_u32",
}
MOTOR_GUARDED = MOTOR_SENDS | MOTOR_CONFIG_WRITES | {"enable", "ensure_mode"}

CONTROLLER_PASSTHROUGH = {"close", "shutdown", "close_bus", "disable_all", "poll_feedback_once"}
CONTROLLER_OPENERS = {"from_socketcanfd", "from_dm_serial", "from_mcu_serial", "from_dm_device"}
CONTROLLER_GUARDED = CONTROLLER_OPENERS | {"add_damiao_motor", "enable_all"}

_MARK = "_arm_guard_patched"


class _Raw:
    """Unguarded calls, for the guard's own reads and hold commands."""

    def __init__(self, orig: dict) -> None:
        self.o = orig

    def get_state(self, m):
        return self.o["get_state"](m)

    def request_feedback(self, m):
        return self.o["request_feedback"](m)

    def get_register_u32(self, m, rid, timeout_ms):
        return self.o["get_register_u32"](m, rid, timeout_ms)

    def get_register_f32(self, m, rid, timeout_ms):
        return self.o["get_register_f32"](m, rid, timeout_ms)

    def send(self, m, c: Command):
        if c.mode == "mit":
            return self.o["send_mit"](m, c.pos, c.vel, c.kp, c.kd, c.tau)
        if c.mode == "pos_vel":
            return self.o["send_pos_vel"](m, c.pos, c.vlim)
        if c.mode == "vel":
            return self.o["send_vel"](m, c.vel)
        return self.o["send_force_pos"](m, c.pos, c.vlim, c.ratio)


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return math.nan


def _refuser(name: str):
    def refuse(self, *a, **k):
        raise GuardRefused(f"motorbridge {name}() is not allowed by arm_guard (not in its allowlist)")
    refuse.__name__ = name
    refuse._arm_guard_refuser = True
    return refuse


def patch(controller_cls, motor_cls, guard) -> None:
    """Patch both classes in place. Idempotent per class."""
    if getattr(motor_cls, _MARK, False):
        return
    orig = {n: getattr(motor_cls, n) for n in dir(motor_cls) if not n.startswith("_") and callable(getattr(motor_cls, n))}
    raw = _Raw(orig)
    guard.raw = raw

    def send_wrapper(name, build):
        @functools.wraps(orig[name])
        def w(self, *a, **k):
            cmd = build(*a, **k)
            out, fault = guard.filter(self, cmd)
            raw.send(self, out)
            if fault:
                raise GuardFault(fault)
        return w

    builders = {
        "send_mit": lambda pos, vel, kp, kd, tau: Command(
            "mit", pos=_f(pos), vel=_f(vel), kp=_f(kp), kd=_f(kd), tau=_f(tau)),
        "send_pos_vel": lambda pos, vlim: Command("pos_vel", pos=_f(pos), vlim=_f(vlim)),
        "send_vel": lambda vel: Command("vel", vel=_f(vel)),
        "send_force_pos": lambda pos, vlim, ratio: Command("force_pos", pos=_f(pos), vlim=_f(vlim), ratio=_f(ratio)),
    }
    wrappers = {n: send_wrapper(n, b) for n, b in builders.items() if n in orig}   # a class may lack some

    def enable(self):
        guard.check_enable(self)
        return orig["enable"](self)

    def ensure_mode(self, mode, timeout_ms=1000):
        guard.check_mode(self, mode)
        return orig["ensure_mode"](self, mode, timeout_ms)

    for n, fn in (("enable", enable), ("ensure_mode", ensure_mode)):
        if n in orig:
            wrappers[n] = functools.wraps(orig[n])(fn)

    def config_wrapper(name):
        @functools.wraps(orig[name])
        def w(self, *a, **k):
            rid = a[0] if name.startswith(("write_register", "damiao_write_param")) and a else None
            value = _f(a[1]) if rid is not None and len(a) > 1 else None
            guard.check_config_write(self, f"{name}{a}", rid=rid, value=value)
            return orig[name](self, *a, **k)
        return w

    for n in MOTOR_CONFIG_WRITES:
        if n in orig:
            wrappers[n] = config_wrapper(n)

    def liveness_read(name):
        @functools.wraps(orig[name])
        def w(self, *a, **k):
            out = orig[name](self, *a, **k)      # pass-through: never refused; a failure raises as before
            guard.note_link_alive(self)
            return out
        return w

    for n in orig:
        if n in MOTOR_LIVENESS_READS:
            setattr(motor_cls, n, liveness_read(n))
            continue
        if n in MOTOR_PASSTHROUGH:
            continue
        setattr(motor_cls, n, wrappers.get(n) or _refuser(n))
    setattr(motor_cls, _MARK, True)

    _patch_controller(controller_cls, guard)


def _patch_controller(cls, guard) -> None:
    if getattr(cls, _MARK, False):
        return
    names = [n for n in dir(cls) if not n.startswith("_") and callable(getattr(cls, n))]
    orig_init = cls.__init__

    def __init__(self, channel: str = "can0", *a, **k):
        self._arm_guard_bus = guard.open_bus(channel)
        orig_init(self, channel, *a, **k)

    cls.__init__ = functools.wraps(orig_init)(__init__)

    def opener(name):
        fn = getattr(cls, name).__func__

        @classmethod
        @functools.wraps(fn)
        def w(klass, *a, **k):
            bound = inspect.signature(fn).bind(klass, *a, **k)
            bound.apply_defaults()
            args = list(bound.arguments.values())[1:]
            channel = str(args[0]) if name != "from_dm_device" else "dm_device:" + ":".join(map(str, args))
            bus = guard.open_bus(channel)
            self = fn(klass, *a, **k)
            self._arm_guard_bus = bus
            return self
        return w

    def add_damiao_motor(self, motor_id, feedback_id, model):
        m = orig_add(self, motor_id, feedback_id, model)
        guard.register_motor(self._arm_guard_bus, m, motor_id, feedback_id, model)
        return m

    def enable_all(self):
        bus = self._arm_guard_bus
        for jr in list(bus.joints.values()):
            guard.check_enable(jr.motor)
        return orig_enable_all(self)

    orig_add = getattr(cls, "add_damiao_motor", None)
    orig_enable_all = getattr(cls, "enable_all", None)
    for n in names:
        if n in CONTROLLER_PASSTHROUGH:
            continue
        if n in CONTROLLER_OPENERS:
            setattr(cls, n, opener(n))
        elif n == "add_damiao_motor":
            setattr(cls, n, functools.wraps(orig_add)(add_damiao_motor))
        elif n == "enable_all":
            setattr(cls, n, functools.wraps(orig_enable_all)(enable_all))
        else:
            setattr(cls, n, _refuser(n))
    setattr(cls, _MARK, True)

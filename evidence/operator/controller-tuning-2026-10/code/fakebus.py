"""fakebus.py — an offline stand-in for motorbridge 0.5.6 (Controller / Motor / MotorState) on the B601-DM bus.

Used by test_first_move_j1.py and by `first_move_j1.py --simulate`. Nothing here touches hardware or imports
motorbridge.

What it models (enough to exercise every code path of first_move_j1.py):
  - the Python API surface of motorbridge 0.5.6 that the script may call (core.py: Controller.from_dm_serial,
    add_damiao_motor, enable_all, disable_all, close_bus, close, shutdown, poll_feedback_once; Motor.enable,
    disable, send_mit, request_feedback, get_state, get_register_u32/_f32, plus every write/zero/mode call so a
    test can prove it was never made);
  - Damiao semantics confirmed in the motorbridge v0.5.6 source (motor_vendors/damiao/src/{motor,protocol}.rs):
    every command (enable, disable, MIT, 0xCC refresh) is answered by a feedback frame; get_state() returns the
    last decoded frame with NO timestamp; register replies (0x33) go to a separate cache and never overwrite the
    feedback state; a disabled motor answers MIT frames without moving;
  - joint1's physics: MIT torque tau = kp*(p_des-p) + kd*(v_des-v) + tau_ff (clamped to TMAX), driving a
    rotational inertia with viscous + Coulomb friction. Joint1 carries no gravity load;
  - faults injected at a simulated time: a stuck joint, a torque-reading spike, frozen feedback with silent
    registers (= power cut / dead adapter), another motor reporting ENABLED, a bus error on send.

Every call is appended to FakeController.calls as (motor_name_or_'ctrl', method, args) for the tests.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Optional


class FakeCallError(RuntimeError):
    """Stands in for motorbridge.CallError (what the real SDK raises on a failed/timed-out call)."""


@dataclass(frozen=True)
class FakeMotorState:
    """Same fields as motorbridge.models.MotorState."""
    can_id: int
    arbitration_id: int
    status_code: int
    pos: float
    vel: float
    torq: float
    t_mos: float
    t_rotor: float


MODEL_LIMITS = {"4340P": (12.5, 10.0, 28.0), "4310": (12.5, 30.0, 10.0)}


class FakeClock:
    """Simulated monotonic time. sleep() advances it; nothing really waits."""

    def __init__(self, start: float = 1000.0) -> None:
        self.t = float(start)
        self.on_advance: list[Callable[[float], None]] = []

    def __call__(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        if dt > 0:
            self.t += dt
            for cb in list(self.on_advance):
                cb(self.t)


@dataclass
class Faults:
    """Faults keyed by simulated time measured from the first enable of motor 1 (None = never)."""
    stuck_from: Optional[float] = None           # joint1 stops moving (jam)
    torque_spike_at: Optional[float] = None      # joint1 torque reading jumps to torque_spike_nm
    torque_spike_nm: float = 5.0
    freeze_at: Optional[float] = None            # no more frames from motor 1, register reads time out
    other_enabled_at: Optional[float] = None     # motor `other_enabled_name` reports ENABLED
    other_enabled_name: str = "j4"
    other_moves_at: Optional[float] = None       # motor `other_moves_name` reads +0.4 rad (someone bumps it)
    other_moves_name: str = "j5"
    bus_error_at: Optional[float] = None         # send_mit raises from then on
    disable_raises: bool = False                 # motor.disable() raises (adapter gone)
    constant_frames: bool = False                # motor 1 frames never change (alive but identical)
    silent_registers: bool = False               # register reads always time out


class FakeMotorSim:
    """One Damiao motor: registers, cached feedback frame and (for joint1) simple rigid-body physics."""

    def __init__(self, name: str, motor_id: int, feedback_id: int, model: str, clock: FakeClock,
                 pos: float = 0.0, status: int = 0, inertia: float = 0.07, viscous: float = 0.5,
                 coulomb: float = 0.3) -> None:
        self.name, self.motor_id, self.feedback_id, self.model = name, motor_id, feedback_id, model
        self.clock = clock
        pmax, vmax, tmax = MODEL_LIMITS[model]
        self.registers: dict[int, float] = {10: 1, 9: 0, 21: pmax, 22: vmax, 23: tmax}
        self.pos, self.vel, self.torq = float(pos), 0.0, 0.0
        self.status = int(status)
        self.inertia, self.viscous, self.coulomb = inertia, viscous, coulomb
        self.mit: Optional[tuple[float, float, float, float, float]] = None
        self.t_phys = clock()
        self.answers = True                # False = silent (no frames, register timeouts)
        self.cached: Optional[FakeMotorState] = None
        self.torque_override: Optional[float] = None
        self.pos_override: Optional[float] = None
        self.status_override: Optional[int] = None
        self.frozen_frame = False
        self.load = 0.0                    # gravity torque on the joint while enabled (disabled: resting on its stop)

    def _step_physics(self) -> None:
        now = self.clock()
        dt_total = now - self.t_phys
        self.t_phys = now
        if dt_total <= 0:
            return
        n = max(1, int(math.ceil(dt_total / 0.0005)))
        h = dt_total / n
        tmax = self.registers[23]
        for _ in range(n):
            if self.status == 1 and self.mit is not None:
                p_des, v_des, kp, kd, tau_ff = self.mit
                tau = kp * (p_des - self.pos) + kd * (v_des - self.vel) + tau_ff
                tau = max(-tmax, min(tmax, tau))
            else:
                tau = 0.0
            self.torq = tau
            if self.status == 1:
                tau += self.load
            # viscous + Coulomb friction with stiction
            if abs(self.vel) < 1e-4 and abs(tau) <= self.coulomb:
                self.vel = 0.0
                continue
            fric = self.viscous * self.vel + self.coulomb * (1.0 if self.vel > 0 else -1.0 if self.vel < 0 else
                                                               (1.0 if tau > 0 else -1.0))
            acc = (tau - fric) / self.inertia
            self.vel += acc * h
            self.pos += self.vel * h

    def frame(self) -> None:
        """The motor answers a command: decode a fresh frame into the cache (unless silent/frozen)."""
        if not self.answers:
            return
        self._step_physics()
        if self.frozen_frame and self.cached is not None:
            return
        pos = self.pos if self.pos_override is None else self.pos_override
        torq = self.torq if self.torque_override is None else self.torque_override
        status = self.status if self.status_override is None else self.status_override
        self.cached = FakeMotorState(can_id=self.motor_id, arbitration_id=self.feedback_id, status_code=status,
                                     pos=pos, vel=self.vel, torq=torq, t_mos=30.0, t_rotor=28.0)


class FakeMotor:
    """The handle returned by FakeController.add_damiao_motor — same method names as motorbridge.Motor."""

    def __init__(self, ctrl: "FakeController", sim: FakeMotorSim) -> None:
        self._ctrl, self._sim = ctrl, sim

    def _log(self, method: str, *args) -> None:
        self._ctrl.calls.append((self._sim.name, method, args))

    # -- commands (each answered by a feedback frame) --
    def enable(self) -> None:
        self._log("enable")
        self._ctrl._maybe_bus_error()
        if self._sim.answers:
            self._sim._step_physics()   # integrate the disabled interval first (no load while disabled)
            self._sim.status = 1
            if self._sim.name == "j1" and self._ctrl.t_enable is None:
                self._ctrl.t_enable = self._ctrl.clock()
        self._sim.frame()

    def disable(self) -> None:
        self._log("disable")
        if self._ctrl.faults.disable_raises:
            raise FakeCallError("disable failed: serial write error")
        if self._sim.answers:
            self._sim.status = 0
            self._sim.mit = None
        self._sim.frame()

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        self._log("send_mit", pos, vel, kp, kd, tau)
        self._ctrl._maybe_bus_error()
        if self._sim.answers and self._sim.status == 1:
            self._sim._step_physics()
            self._sim.mit = (float(pos), float(vel), float(kp), float(kd), float(tau))
        self._sim.frame()

    def request_feedback(self) -> None:
        self._log("request_feedback")
        self._sim.frame()

    def get_state(self) -> Optional[FakeMotorState]:
        self._log("get_state")
        return self._sim.cached

    def get_register_u32(self, rid: int, timeout_ms: int = 1000) -> int:
        self._log("get_register_u32", rid, timeout_ms)
        if not self._sim.answers or self._ctrl.faults.silent_registers or rid not in self._sim.registers:
            self._ctrl.clock.sleep(timeout_ms / 1000.0)
            raise FakeCallError(f"get_register_u32 failed: register {rid} read timed out")
        self._ctrl.clock.sleep(0.002)
        return int(self._sim.registers[rid])

    def get_register_f32(self, rid: int, timeout_ms: int = 1000) -> float:
        self._log("get_register_f32", rid, timeout_ms)
        if not self._sim.answers or self._ctrl.faults.silent_registers or rid not in self._sim.registers:
            self._ctrl.clock.sleep(timeout_ms / 1000.0)
            raise FakeCallError(f"get_register_f32 failed: register {rid} read timed out")
        self._ctrl.clock.sleep(0.002)
        return float(self._sim.registers[rid])

    # -- calls the script must never make; present so a test can prove they were not --
    def write_register_u32(self, rid: int, value: int) -> None:
        self._log("write_register_u32", rid, value)

    def write_register_f32(self, rid: int, value: float) -> None:
        self._log("write_register_f32", rid, value)

    def set_zero_position(self) -> None:
        self._log("set_zero_position")

    def ensure_mode(self, mode, timeout_ms: int = 1000) -> None:
        self._log("ensure_mode", mode, timeout_ms)

    def set_can_timeout_ms(self, timeout_ms: int) -> None:
        self._log("set_can_timeout_ms", timeout_ms)

    def store_parameters(self) -> None:
        self._log("store_parameters")

    def clear_error(self) -> None:
        self._log("clear_error")

    def send_pos_vel(self, pos: float, vlim: float) -> None:
        self._log("send_pos_vel", pos, vlim)

    def send_vel(self, vel: float) -> None:
        self._log("send_vel", vel)

    def close(self) -> None:
        self._log("close")


class FakeController:
    """Same method names as motorbridge.Controller. Construct through FakeBus.factory()."""

    def __init__(self, bus: "FakeBus") -> None:
        self._bus = bus
        self.clock = bus.clock
        self.faults = bus.faults
        self.calls = bus.calls
        self.t_enable: Optional[float] = None
        self.motors: dict[str, FakeMotor] = {}

    def _maybe_bus_error(self) -> None:
        f = self.faults
        if f.bus_error_at is not None and self.t_enable is not None and \
                self.clock() - self.t_enable >= f.bus_error_at:
            raise FakeCallError("send failed: serial write error (device disconnected?)")

    def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str) -> FakeMotor:
        self.calls.append(("ctrl", "add_damiao_motor", (motor_id, feedback_id, model)))
        sim = self._bus.sims_by_id[motor_id]
        if sim.feedback_id != feedback_id or sim.model != model:
            raise FakeCallError("add_damiao_motor failed: config mismatch")
        m = FakeMotor(self, sim)
        self.motors[sim.name] = m
        return m

    def enable_all(self) -> None:
        self.calls.append(("ctrl", "enable_all", ()))

    def disable_all(self) -> None:
        self.calls.append(("ctrl", "disable_all", ()))
        for s in self._bus.sims_by_id.values():
            if s.answers:
                s.status = 0
                s.mit = None

    def shutdown(self) -> None:
        self.calls.append(("ctrl", "shutdown", ()))
        self.disable_all()

    def poll_feedback_once(self) -> None:
        self.calls.append(("ctrl", "poll_feedback_once", ()))

    def close_bus(self) -> None:
        self.calls.append(("ctrl", "close_bus", ()))

    def close(self) -> None:
        self.calls.append(("ctrl", "close", ()))


B601_MOTORS = (
    ("j1", 0x01, 0x11, "4340P"),
    ("j2", 0x02, 0x12, "4340P"),
    ("j3", 0x03, 0x13, "4340P"),
    ("j4", 0x04, 0x14, "4310"),
    ("j5", 0x05, 0x15, "4310"),
    ("j6", 0x06, 0x16, "4310"),
    ("j7", 0x07, 0x17, "4310"),
)

# Rest pose at the end of the 2026-10-10 session (evidence/operator/MOT-09/bringup_2026-10-10_summary.md).
REST_2026_10_10 = {"j1": 0.01698, "j2": 0.00172, "j3": -0.00134, "j4": -0.00210, "j5": 0.03147,
                   "j6": 0.00019, "j7": 0.00782}


@dataclass
class FakeBus:
    """Seven simulated motors on one fake dm-serial bus, plus the shared call log and fault plan."""
    clock: FakeClock = field(default_factory=FakeClock)
    faults: Faults = field(default_factory=Faults)
    rest: dict = field(default_factory=lambda: dict(REST_2026_10_10))
    calls: list = field(default_factory=list)

    def __post_init__(self) -> None:
        self.sims_by_id: dict[int, FakeMotorSim] = {}
        for name, mid, fid, model in B601_MOTORS:
            self.sims_by_id[mid] = FakeMotorSim(name, mid, fid, model, self.clock, pos=self.rest.get(name, 0.0))
        self.sims = {s.name: s for s in self.sims_by_id.values()}
        self.controller: Optional[FakeController] = None
        self.clock.on_advance.append(self._apply_faults)

    def factory(self, port: str, baud: int) -> FakeController:
        self.calls.append(("ctrl", "from_dm_serial", (port, baud)))
        self.controller = FakeController(self)
        return self.controller

    def _apply_faults(self, now: float) -> None:
        c = self.controller
        if c is None or c.t_enable is None:
            return
        t = now - c.t_enable
        f, j1 = self.faults, self.sims["j1"]
        if f.stuck_from is not None and t >= f.stuck_from:
            j1.inertia = 1e9          # effectively immovable: the joint is jammed
            j1.vel = 0.0
        if f.torque_spike_at is not None and t >= f.torque_spike_at:
            j1.torque_override = f.torque_spike_nm
        if f.freeze_at is not None and t >= f.freeze_at:
            j1.answers = False        # power cut / adapter gone: no frames, registers time out
            for s in self.sims.values():
                s.answers = False
        if f.other_enabled_at is not None and t >= f.other_enabled_at:
            self.sims[f.other_enabled_name].status_override = 1
        if f.other_moves_at is not None and t >= f.other_moves_at:
            self.sims[f.other_moves_name].pos_override = 0.4
        if f.constant_frames:
            j1.frozen_frame = True

    @property
    def mod(self):
        """Namespace standing in for the motorbridge module (only what first_move_j1.py uses)."""
        bus = self

        class _Mod:
            CallError = FakeCallError
            __version__ = "0.5.6"

            class Controller:
                @staticmethod
                def from_dm_serial(port: str, baud: int) -> FakeController:
                    return bus.factory(port, baud)
        return _Mod

"""A stand-in for motorbridge.core with the same public Controller/Motor surface (motorbridge 0.5.6) and a crude
joint simulation, so arm_guard can be exercised end to end without the native library or a bus.

`make_classes()` returns fresh classes each call (arm_guard patches classes in place, once)."""

from __future__ import annotations

from .errors import CallError
from .models import Mode, MotorState

MODEL_MAP = {"4340P": (12.5, 10.0, 28.0), "4310": (12.5, 30.0, 10.0)}


def make_classes():
    class Motor:
        def __init__(self, motor_id, feedback_id, model, start_pos=0.0):
            self.motor_id, self.feedback_id, self.model = motor_id, feedback_id, model
            self.pos, self.vel, self.torq = float(start_pos), 0.0, 0.0
            self.status = 0x0
            pmax, vmax, tmax = MODEL_MAP[model]
            self.regs = {9: 0, 10: int(Mode.MIT), 21: pmax, 22: vmax, 23: tmax, 25: 0.0, 26: 0.0, 27: 0.0, 28: 0.0}
            self.sent = []           # every raw command that reached the "bus"
            self.target = None
            self.frozen = False      # feedback stops changing
            self.heartbeat_ok = True
            self.reg_failures = 0    # the next N register reads time out (a dropped reply on the link)
            self.no_feedback = False
            self.zeroed = 0
            self.stored = 0
            self.closed = False
            self.tick = 0

        # -- simulation -------------------------------------------------------------------------------------
        def _sim_step(self, dt):
            """Advance the joint dt seconds toward its last command (MIT: tracks the target with a lag;
            POS_VEL/FORCE_POS: moves at up to vlim)."""
            self.tick += 1
            if self.status != 0x1 or self.target is None:
                self.vel = 0.0
                return
            mode, a = self.target
            if mode == "mit":
                q_t = a[0]
                new = self.pos + (q_t - self.pos) * min(1.0, dt / 0.02)
            elif mode in ("pos_vel", "force_pos"):
                q_t, vlim = a[0], a[1]
                d = max(-vlim * dt, min(vlim * dt, q_t - self.pos))
                new = self.pos + d
            else:
                new = self.pos + a[0] * dt
            self.vel = (new - self.pos) / dt
            self.pos = new
            self.torq = 0.0

        # -- motorbridge API --------------------------------------------------------------------------------
        def close(self):
            self.closed = True

        def enable(self):
            self.status = 0x1

        def disable(self):
            self.status = 0x0

        def clear_error(self):
            if self.status not in (0x0, 0x1):
                self.status = 0x0

        def set_zero_position(self):
            self.zeroed += 1
            self.pos = 0.0

        def ensure_mode(self, mode, timeout_ms=1000):
            self.regs[10] = int(mode)

        def send_mit(self, pos, vel, kp, kd, tau):
            self.sent.append(("mit", (pos, vel, kp, kd, tau)))
            self.target = ("mit", (pos, vel, kp, kd, tau))

        def send_pos_vel(self, pos, vlim):
            self.sent.append(("pos_vel", (pos, vlim)))
            self.target = ("pos_vel", (pos, vlim))

        def robstride_send_pos_vel_pp(self, pos, vel_max, acc_set):
            self.sent.append(("robstride_pp", (pos, vel_max, acc_set)))

        def robstride_send_pos_vel_csp(self, pos, vlim):
            self.sent.append(("robstride_csp", (pos, vlim)))

        def send_vel(self, vel):
            self.sent.append(("vel", (vel,)))
            self.target = ("vel", (vel,))

        def send_force_pos(self, pos, vlim, ratio):
            self.sent.append(("force_pos", (pos, vlim, ratio)))
            self.target = ("force_pos", (pos, vlim, ratio))

        def request_feedback(self):
            pass

        def set_can_timeout_ms(self, timeout_ms):
            self.regs[9] = int(timeout_ms) * 20

        def store_parameters(self):
            self.stored += 1

        def write_register_f32(self, rid, value):
            self.regs[rid] = float(value)

        def write_register_u32(self, rid, value):
            self.regs[rid] = int(value)

        def _reg_ok(self):
            if self.reg_failures > 0:
                self.reg_failures -= 1
                return False
            return self.heartbeat_ok

        def get_register_f32(self, rid, timeout_ms=1000):
            if not self._reg_ok():
                raise CallError("timeout")
            return float(self.regs[rid])

        def get_register_u32(self, rid, timeout_ms=1000):
            if not self._reg_ok():
                raise CallError("timeout")
            return int(self.regs[rid])

        def robstride_ping(self):
            return (0, 0)

        def robstride_ping_host_id(self, host_id, timeout_ms=500):
            return (0, 0)

        def robstride_get_param_f32_host_id(self, param_id, host_id, timeout_ms=1000):
            return 0.0

        def robstride_get_fault_report(self):
            return (0, 0)

        def robstride_set_device_id(self, new_device_id):
            pass

        def robstride_set_active_report(self, enabled):
            pass

        def robstride_write_param_i8(self, param_id, value):
            pass

        def robstride_write_param_u8(self, param_id, value):
            pass

        def robstride_write_param_u16(self, param_id, value):
            pass

        def robstride_write_param_u32(self, param_id, value):
            pass

        def robstride_write_param_f32(self, param_id, value):
            pass

        def robstride_get_param_i8(self, param_id, timeout_ms=1000):
            return 0

        def robstride_get_param_u8(self, param_id, timeout_ms=1000):
            return 0

        def robstride_get_param_u16(self, param_id, timeout_ms=1000):
            return 0

        def robstride_get_param_u32(self, param_id, timeout_ms=1000):
            return 0

        def robstride_get_param_f32(self, param_id, timeout_ms=1000):
            return 0.0

        def damiao_get_param_f32(self, param_id, timeout_ms=1000):
            return float(self.regs.get(param_id, 0.0))

        def damiao_get_param_u32(self, param_id, timeout_ms=1000):
            return int(self.regs.get(param_id, 0))

        def damiao_write_param_f32(self, param_id, value):
            self.regs[param_id] = float(value)

        def damiao_write_param_u32(self, param_id, value):
            self.regs[param_id] = int(value)

        def get_state(self):
            if self.no_feedback:
                return None
            # a real frame differs from the last one by tiny noise unless the link is frozen
            noise = 0.0 if self.frozen else 1e-7 * (self.tick % 7)
            return MotorState(self.motor_id, self.feedback_id, self.status, self.pos + noise, self.vel, self.torq,
                              30.0, 30.0)

    class Controller:
        start_positions: dict = {}

        def __init__(self, channel: str = "can0") -> None:
            self.channel = channel
            self.motors = []

        @classmethod
        def from_socketcanfd(cls, channel: str = "can0") -> "Controller":
            self = cls.__new__(cls)
            self.channel, self.motors = channel, []
            return self

        @classmethod
        def from_dm_serial(cls, serial_port: str = "/dev/ttyACM0", baud: int = 921600) -> "Controller":
            self = cls.__new__(cls)
            self.channel, self.motors = serial_port, []
            return self

        @classmethod
        def from_mcu_serial(cls, serial_port: str = "/dev/ttyUSB0", baud: int = 921600) -> "Controller":
            self = cls.__new__(cls)
            self.channel, self.motors = serial_port, []
            return self

        @classmethod
        def from_dm_device(cls, dm_device_type: str = "usb2canfd-dual", dm_channel: str = "0") -> "Controller":
            self = cls.__new__(cls)
            self.channel, self.motors = f"{dm_device_type}:{dm_channel}", []
            return self

        def close(self):
            pass

        def shutdown(self):
            for m in self.motors:
                m.disable()

        def close_bus(self):
            pass

        def enable_all(self):
            for m in self.motors:
                m.enable()

        def disable_all(self):
            for m in self.motors:
                m.disable()

        def poll_feedback_once(self):
            pass

        def add_damiao_motor(self, motor_id, feedback_id, model):
            m = Motor(motor_id, feedback_id, model, self.start_positions.get(motor_id, 0.0))
            self.motors.append(m)
            return m

        def add_hexfellow_motor(self, motor_id, feedback_id, model):
            return Motor(motor_id, feedback_id, "4310")

        def add_myactuator_motor(self, motor_id, feedback_id, model):
            return Motor(motor_id, feedback_id, "4310")

        def add_robstride_motor(self, motor_id, feedback_id, model):
            return Motor(motor_id, feedback_id, "4310")

        def add_hightorque_motor(self, motor_id, feedback_id, model):
            return Motor(motor_id, feedback_id, "4310")

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.shutdown()

    return Controller, Motor


Controller, Motor = make_classes()

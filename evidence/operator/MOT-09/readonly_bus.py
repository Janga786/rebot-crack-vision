"""Read-only access to the B601-DM motors over motorbridge (dm-serial).

Every motor handle is wrapped in ReadOnlyMotor, which exposes ONLY:
  request_feedback()  -> Damiao "refresh status" frame [id, 0, 0xCC, 0...] on 0x7FF (no state change)
  get_state()         -> last decoded feedback frame (local cache, sends nothing)
  get_register_u32/f32(rid) -> register READ [id, 0, 0x33, rid, 0...] on 0x7FF
Anything else (enable, send_mit, set_zero_position, write_register_*, ensure_mode, ...) raises.
The bus is closed with close_bus(), never shutdown() (shutdown() would send disable to every motor).
Verified against motorbridge v0.5.6 source: motor_vendors/damiao/src/{protocol,motor}.rs.
"""
from __future__ import annotations

from motorbridge import Controller

PORT = "/dev/ttyACM0"
BAUD = 921600

# (name, motor_id, feedback_id, motorbridge model)
MOTORS = [
    ("j1", 0x01, 0x11, "4340P"),
    ("j2", 0x02, 0x12, "4340P"),
    ("j3", 0x03, 0x13, "4340P"),
    ("j4", 0x04, 0x14, "4310"),
    ("j5", 0x05, 0x15, "4310"),
    ("j6", 0x06, 0x16, "4310"),
    ("j7", 0x07, 0x17, "4310"),  # gripper
]

STATUS = {0x0: "DISABLED", 0x1: "ENABLED", 0x8: "OVER_VOLTAGE", 0x9: "UNDER_VOLTAGE",
          0xA: "OVER_CURRENT", 0xB: "MOS_OVER_TEMP", 0xC: "ROTOR_OVER_TEMP",
          0xD: "LOST_COMM", 0xE: "OVERLOAD"}


class ReadOnlyMotor:
    _ALLOWED = ("request_feedback", "get_state", "get_register_u32", "get_register_f32")

    def __init__(self, motor):
        object.__setattr__(self, "_m", motor)

    def __getattr__(self, name):
        if name in self._ALLOWED:
            return getattr(self._m, name)
        raise PermissionError(f"ReadOnlyMotor: '{name}' is not a read-only call")

    def __setattr__(self, name, value):
        raise PermissionError("ReadOnlyMotor is immutable")

    def close(self):
        self._m.close()  # frees the local handle; sends nothing


def open_readonly():
    """Return (ctrl, [(name, id, fid, model, ReadOnlyMotor), ...])."""
    ctrl = Controller.from_dm_serial(PORT, BAUD)
    motors = []
    try:
        for name, mid, fid, model in MOTORS:
            motors.append((name, mid, fid, model, ReadOnlyMotor(ctrl.add_damiao_motor(mid, fid, model))))
    except Exception:
        close_readonly(ctrl, motors)
        raise
    return ctrl, motors


def close_readonly(ctrl, motors):
    for *_, m in motors:
        try:
            m.close()
        except Exception:
            pass
    try:
        ctrl.close_bus()  # NOT shutdown(): no disable frames
    finally:
        ctrl.close()

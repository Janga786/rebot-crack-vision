#!/usr/bin/env python3
"""first_move_j1.py — FIRST powered motion of the reBot B601-DM: joint1 only, low-gain MIT, operator-attended.

READ ALL OF THIS FILE BEFORE RUNNING IT. It was written overnight (2026-10-10/11) for review, has been tested only
offline against fakebus.py (test_first_move_j1.py), and has never been run on hardware.

What it does (MOT-09 step C1)
  1. Read-only pre-checks; refuses (exit 3) unless every one passes:
     - the port is accessible and no other process holds it;
     - motorbridge is 0.5.6, the version whose Damiao behaviour this script relies on;
     - all 7 motors answer, all report DISABLED, all within 0.05 rad of zero;
     - motor 1 is in CTRL_MODE 1 (MIT) and its PMAX/VMAX/TMAX match the 4340P mapping. This script never
       changes the mode or any register.
  2. Prints the plan and asks you to type ENABLE J1 on the terminal (/dev/tty). The one alternative is
     --operator-go TEXT (added 2026-10-14 on the operator's decision): the operator, at the arm with a hand on the
     24 V switch, says "go" in chat and the Claude session that runs this command passes those words. They are
     recorded verbatim in the JSON summary ("confirmation"), so the record shows how the go was given.
  3. Enables ONLY motor 1 (motors 2-7 are wrapped in a read-only proxy that cannot enable).
  4. The first MIT command after enable targets motor 1's measured position. Then:
       settle 0.5 s → minimum-jerk ramp to --target (default +0.10 rad) over >= 2 s
       → hold >= 1 s → minimum-jerk ramp back to the start over >= 2 s → hold >= 1 s → disable.
     Joint1 + = the arm swings to the ROBOT'S LEFT (sign check, 2026-10-10).
     Gains: kp <= 10, kd <= 1 (hard caps), feed-forward torque always 0.
     The MIT velocity reference is the profile's own velocity (<= 0.08 rad/s at defaults). That is a
     kd-damping reference, not torque feed-forward.
  5. Streams commands at 200 Hz (>= 100 Hz required) and logs t, target, position, velocity, torque (and more) to
     runs/first_move_j1_<UTC>.csv, plus a JSON summary.

Aborts (each one: hold the current position 0.5 s, then disable motor 1, print the trigger, exit 1)
  - tracking: |target − position| > 0.10 rad
  - torque:   |torque| > kp·0.10 + kd·0.5 + 0.5 Nm (= 2.0 Nm at kp 10, kd 1). Joint1 carries no gravity load, so the
              MIT law can only ask for kp·e + kd·Δv. Within the non-aborted envelope (e <= 0.10 rad, |v| <= 0.5 rad/s)
              that is at most 1.5 Nm. The 0.5 Nm margin covers the 12-bit torque feedback (≈0.014 Nm/LSB at the
              4340P's ±28 Nm mapping) and current-loop ripple. A higher reading means torque the controller did not
              ask for (contact, jam, misdecoded feedback). Expected joint1 friction is well under 1 Nm. 2.0 Nm is 7 %
              of the 4340P's 27 Nm URDF effort.
  - stale:    no proof of fresh feedback for > 0.2 s (see "Freshness" below)
  - velocity: |velocity| > 0.5 rad/s (the profile peaks at ≈ 0.08 rad/s)
  - status:   motor 1 not ENABLED, or reporting a fault code
  - others:   any of motors 2-7 not DISABLED, or moved > 0.10 rad from where it started (someone touched the arm)
  - stream:   a gap > 50 ms between two commands (the loop stalled)
  - bus:      any motorbridge error
  - Ctrl-C / SIGTERM / SIGHUP
  Hold target on abort: the last measured position if it is fresh, otherwise the last commanded target. On a stale
  abort the measured position is by definition not current, and freezing the setpoint keeps what the motor was
  already tracking. Joint1 has no gravity load, so disabling it at any angle cannot make the arm fall.
  If another motor reports ENABLED, something else is driving the bus. After disabling motor 1 the script then
  also sends disable_all(). Every cycle checked that motors 2-7 stayed within 0.10 rad of their folded rest, so
  joint2/joint3 still sit on their fold stop and nothing can drop.

Not an abort, but reported: a jam on a move this small (start ≈ +0.017, target +0.10: 0.083 rad) can never
  exceed the 0.10 rad tracking threshold, and kp·0.083 = 0.83 Nm is below the torque bound. The run then
  "completes". So the summary prints the mean error during the 1 s hold and warns when it exceeds 0.03 rad
  (friction above kp·0.03 = 0.3 Nm, or a jam). Read that line before going further.

Freshness (motorbridge exposes no feedback timestamp)
  motorbridge's get_state() returns the last decoded frame with no time attached (the Rust core keeps one, but the
  0.5.6 ABI does not expose it). This script treats feedback as fresh at time T only if one of these holds:
    (a) the decoded state tuple (status, pos, vel, torq, temperatures) changed at T, or
    (b) a register read of motor 1's CTRL_MODE (rid 10, a 0x33 request/reply, ~2 ms) completed at T within its 10 ms
        timeout. Each one is a round trip proving adapter, bus and motor are alive: motorbridge sends a new
        request and accepts only a reply timestamped after it (motor_vendors/damiao/src/motor.rs,
        wait_for_register_value). It also re-checks that the mode is still MIT. Register replies go to a separate
        cache and never overwrite feedback state (motor.rs, process_feedback_frame_impl).
  (b) runs only when (a) has not happened for 50 ms. So a still-but-alive motor costs one 2 ms round trip per 50 ms.
  If neither holds for 0.2 s, the script aborts as stale.

Watchdog
  Software only: try/finally always calls motor 1 disable(); if that fails it falls back to disable_all() and tells
  you to cut 24 V. No motor-side timeout register is written. TIMEOUT is 0 on every motor (2026-10-10), so if this
  process dies uncleanly (kill -9, USB unplug, power loss of the PC) motor 1 keeps executing its last MIT command
  (holding position). Joint1 carries no load, but your hand must be on the 24 V switch / e-stop.

Modes
  --check-only   do only the read-only pre-checks (no enable), write runs/rest_check_<UTC>.json
  --estop-test   enable motor 1 and hold its measured position for up to --estop-wait-s (default 30) while you
                 press the hardware e-stop. Expected: feedback lost → "e-stop observed" (exit 0). Release the
                 e-stop, then run --check-only. (MOT-09 step C3)
  --simulate     run everything against fakebus.py with simulated time (no hardware, no motorbridge import);
                 --simulate-fault demonstrates the abort paths. Output goes to runs_sim/ and is named *_SIM:
                 never file it as evidence.

Exit codes (repo convention): 0 completed / e-stop observed, 1 aborted, 2 usage, 3 precondition refused.

Run (after reading this file and running the offline tests):
  cd ~/rebot_setup
  env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python first_move_j1.py --check-only
  env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python first_move_j1.py
  # run by the Claude session after the operator's go in chat (no terminal on that side):
  env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python first_move_j1.py --operator-go "<the operator's words>"
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

SCRIPT_VERSION = "2026-10-17.1"

# ---- facts from the 2026-10-10 bring-up (repo: evidence/operator/MOT-09/bringup_2026-10-10_facts.yaml) ----------
PORT_DEFAULT = "/dev/ttyACM0"
BAUD = 921600
MOTORS = (  # (name, motor_id, feedback_id, motorbridge model)
    ("j1", 0x01, 0x11, "4340P"),
    ("j2", 0x02, 0x12, "4340P"),
    ("j3", 0x03, 0x13, "4340P"),
    ("j4", 0x04, 0x14, "4310"),
    ("j5", 0x05, 0x15, "4310"),
    ("j6", 0x06, 0x16, "4310"),
    ("j7", 0x07, 0x17, "4310"),  # gripper
)
MODEL_LIMITS = {"4340P": (12.5, 10.0, 28.0), "4310": (12.5, 30.0, 10.0)}  # PMAX, VMAX, TMAX
URDF_LIMITS = {"j1": (-2.8, 2.8), "j2": (-3.14, 0.0), "j3": (-3.14, 0.0), "j4": (-1.87, 1.57), "j5": (-1.57, 1.57),
               "j6": (-3.14, 3.14),
               "j7": (-1.8, 0.1)}   # gripper motor angle: open = negative; -1.776 was the widest seen 2026-10-10   # config/robot/b601_dm_limits.yaml
ACTIVE = "j1"     # the one motor this run may enable; set by --joint (2026-10-14: j2 added on the operator's go)
MOTORBRIDGE_VERSION = "0.5.6"
RID_TIMEOUT, RID_CTRL_MODE, RID_PMAX, RID_VMAX, RID_TMAX = 9, 10, 21, 22, 23
MODE_MIT = 1
ST_DISABLED, ST_ENABLED = 0x0, 0x1
STATUS_NAMES = {0x0: "DISABLED", 0x1: "ENABLED", 0x8: "OVER_VOLTAGE", 0x9: "UNDER_VOLTAGE",
                0xA: "OVER_CURRENT", 0xB: "MOS_OVER_TEMP", 0xC: "ROTOR_OVER_TEMP", 0xD: "LOST_COMM",
                0xE: "OVERLOAD"}
# /proc/<pid>/comm is truncated to 15 characters, so these are the names as the kernel reports them.
CONFLICTING_PROCS = ("ws_gateway", "reBotArmControl", "motorbridge-cli", "motorbridge-gat")

# ---- hard limits of this script (CLI values above these are refused, never clamped) --------------------------
KP_MAX = 100.0                    # raised from 10 on 2026-10-14 (operator): kp 10 did not break joint1 friction
KD_MAX = 5.0                      # was 1.0; 2026-10-17: kd 1.0 won every sweep (vendor uses 10 on J1-3, 2 on J4-6)
KD_DEFAULT = 1.0
REST_TOL_RAD = float("inf")        # 2026-10-14 operator: no near-zero start; the operator poses the arm by hand
OTHERS_REST_TOL_RAD = float("inf")
MAX_EXCURSION_RAD = 0.15
MAX_EXCURSION_BY_JOINT = {"j7": 1.2}   # 2026-10-14 operator: open the gripper (unloaded fingers)
MAX_ABS_TARGET_RAD = 0.15
MIN_RAMP_S = 1.0   # was 2.0; 2026-10-17 tuning sweeps (the velocity abort still bounds speed)
MIN_HOLD_S = 1.0
SETTLE_S = 0.5
ABORT_HOLD_S = 0.5
TRACK_ABORT_RAD = 0.10
VEL_ABORT_RAD_S = 0.5
TAU_FF_MAX = 4.0   # 2026-10-17: gravity feed-forward cap, Nm
VEL_ABORT_BY_JOINT = {"j4": 1.0, "j5": 1.0, "j6": 1.0, "j7": 1.0}   # 2026-10-17: DM4310 single-sample velocity spikes (j7, then j4 at enable)
TORQUE_MARGIN_NM = 0.5
# 2026-10-17 operator: --hold-others enables every other motor and holds it where it was measured at enable.
# Gains roughly half the vendor arm.yaml/gripper.yaml values (J1-3 120/10, J4-6 18/2, gripper 8/1), kd within KD_MAX.
HOLD_GAINS = {"j1": (60.0, 1.0), "j2": (60.0, 1.0), "j3": (60.0, 1.0), "j4": (30.0, 0.5), "j5": (30.0, 0.5),
              "j6": (30.0, 0.5), "j7": (8.0, 0.3)}
HOLD_TRACK_ABORT_RAD = 0.15
HOLD_VEL_ABORT_RAD_S = 1.0   # 2026-10-17: held j4 tripped 0.5 on its first frames; the 0.15 rad hold bound is the guard
HOLD_OTHERS = False   # set by --hold-others
# 2026-10-17: --gravity adds the URDF gravity holding torque (gravity.py) as each motor's MIT feed-forward.
FF_CAP_BY_MODEL = {"4340P": 10.0, "4310": 2.5}  # |tau_ff| per motor, Nm (rated 9 / 3, peak 27 / 7); above -> abort
# 4340P was 7.0; 2026-10-17 j3 needs ~6.8 gravity + ~2 friction
MODEL_OF = {n: model for n, _, _, model in MOTORS}
SWEEP_MAX = 12
# 2026-10-17 tuning sweeps (mirror of tuned_gains.yaml): (kp, kd, gravity_scale, friction_nm). --tuned / --also use it.
TUNED = {"j1": (100.0, 4.0, 1.0, 1.0), "j2": (100.0, 1.0, 0.6, 1.0), "j3": (80.0, 1.0, 2.5, 2.0),
         "j4": (40.0, 1.0, 2.5, 0.5), "j5": (40.0, 1.0, 1.0, 0.3), "j6": (40.0, 1.0, 1.0, 0.1)}
ALSO: dict = {}   # --also: extra joints that move with the active one, {name: relative move rad}
FRICTION_MAX_NM = 3.0         # --friction cap (j3 measured about +-2.3 Nm on 2026-10-17)
FRICTION_VEL_EPS = 0.02       # rad/s: tanh(v_ref / eps) is the smooth sign of the commanded velocity
FF_FADE_S = 1.0   # 2026-10-17: the extra (scale > 1) gravity fades in after enable (a j3 x2.5 step kicked it off its rest)
STALE_ABORT_S = 0.20
HEARTBEAT_AFTER_S = 0.05
REG_TIMEOUT_MS = 10
PRECHECK_REG_TIMEOUT_MS = 200
OTHERS_DRIFT_ABORT_RAD = 0.30   # was 0.10; 2026-10-14: the unpowered wrist (J4) swings when J3 lifts
LOOP_HZ = 200.0
STALL_ABORT_S = 0.05
ARRIVAL_WARN_RAD = 0.03          # report (not abort) if the hold sat further than this from the target
ENABLE_CONFIRM_S = 0.10
DISABLE_CONFIRM_S = 0.20
ESTOP_WAIT_DEFAULT_S = 30.0
ESTOP_WAIT_MAX_S = 60.0
CONFIRM_PHRASE = "ENABLE J1"

EXIT_OK, EXIT_ABORTED, EXIT_USAGE, EXIT_REFUSED = 0, 1, 2, 3


class Refused(Exception):
    """A precondition failed: nothing was enabled."""


class Abort(Exception):
    """A runtime trigger fired while motor 1 may be enabled."""

    def __init__(self, trigger: str, detail: str) -> None:
        super().__init__(f"{trigger}: {detail}")
        self.trigger, self.detail = trigger, detail


def torque_limit(kp: float, kd: float) -> float:
    """Largest |torque| the MIT law can legitimately ask for inside the abort envelope, plus margin (see docstring)."""
    return kp * TRACK_ABORT_RAD + kd * VEL_ABORT_RAD_S + TORQUE_MARGIN_NM


def min_jerk(q0: float, q1: float, duration: float, t: float) -> tuple[float, float]:
    """Position and velocity of a minimum-jerk move q0→q1 over `duration`, clamped outside [0, duration]."""
    if t <= 0.0:
        return q0, 0.0
    if t >= duration:
        return q1, 0.0
    s = t / duration
    d = q1 - q0
    return q0 + d * (10 * s**3 - 15 * s**4 + 6 * s**5), d * (30 * s**2 - 60 * s**3 + 30 * s**4) / duration


@dataclass
class Profile:
    q_start: float
    q_target: float
    ramp_s: float
    hold_s: float
    settle_s: float = SETTLE_S

    def phases(self) -> list[tuple[str, float]]:
        return [("settle", self.settle_s), ("ramp_up", self.ramp_s), ("hold", self.hold_s),
                ("ramp_down", self.ramp_s), ("final_hold", self.hold_s)]

    @property
    def duration(self) -> float:
        return sum(d for _, d in self.phases())

    def at(self, t: float) -> tuple[str, float, float]:
        """(phase, q_ref, v_ref) at time t since the first command."""
        t0 = 0.0
        for name, d in self.phases():
            if t < t0 + d:
                tau = t - t0
                if name == "ramp_up":
                    return (name, *min_jerk(self.q_start, self.q_target, d, tau))
                if name == "ramp_down":
                    return (name, *min_jerk(self.q_target, self.q_start, d, tau))
                return name, (self.q_target if name == "hold" else self.q_start), 0.0
            t0 += d
        return "done", self.q_start, 0.0

    @property
    def peak_speed(self) -> float:
        return 1.875 * abs(self.q_target - self.q_start) / self.ramp_s


# ---- guarded motor / controller proxies: the structural "only motor 1 can be enabled" ---------------------------
class ReadOnlyMotor:
    """Motors 2-7: only reads. Anything else raises before a frame can be sent."""
    _ALLOWED = ("request_feedback", "get_state", "get_register_u32", "get_register_f32")

    def __init__(self, name: str, motor: Any) -> None:
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "_m", motor)

    def __getattr__(self, attr: str) -> Any:
        if attr in ReadOnlyMotor._ALLOWED:
            return getattr(self._m, attr)
        raise PermissionError(f"{self.name}: '{attr}' is not allowed (motors 2-7 are read-only in this script)")

    def __setattr__(self, attr: str, value: Any) -> None:
        raise PermissionError("ReadOnlyMotor is immutable")


class J1Motor:
    """Motor 1: reads, enable, disable, and send_mit with the gain caps and zero torque enforced again here."""
    _ALLOWED = ("request_feedback", "get_state", "get_register_u32", "get_register_f32", "enable", "disable")

    def __init__(self, motor: Any, kp_cap: float = KP_MAX, kd_cap: float = KD_MAX) -> None:
        object.__setattr__(self, "name", ACTIVE)
        object.__setattr__(self, "_m", motor)
        object.__setattr__(self, "_kp_cap", kp_cap)
        object.__setattr__(self, "_kd_cap", kd_cap)

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        if not (0.0 <= kp <= self._kp_cap and 0.0 <= kd <= self._kd_cap):
            raise PermissionError(f"j1 send_mit refused: kp={kp} kd={kd} exceed caps {self._kp_cap}/{self._kd_cap}")
        if not abs(tau) <= FF_CAP_BY_MODEL[MODEL_OF[self.name]]:
            raise PermissionError(f"j1 send_mit refused: feed-forward torque |{tau}| > "
                                  f"{FF_CAP_BY_MODEL[MODEL_OF[self.name]]} Nm")
        if not all(math.isfinite(x) for x in (pos, vel)):
            raise PermissionError(f"j1 send_mit refused: non-finite pos/vel {pos}/{vel}")
        self._m.send_mit(float(pos), float(vel), float(kp), float(kd), float(tau))

    def __getattr__(self, attr: str) -> Any:
        if attr in J1Motor._ALLOWED:
            return getattr(self._m, attr)
        raise PermissionError(f"j1: '{attr}' is not allowed in this script (no register writes, zero or mode change)")

    def __setattr__(self, attr: str, value: Any) -> None:
        raise PermissionError("J1Motor is immutable")


class HoldMotor:
    """--hold-others: a non-active motor that may be enabled, disabled and sent MIT at its fixed HOLD_GAINS."""
    _ALLOWED = ("request_feedback", "get_state", "get_register_u32", "get_register_f32", "enable", "disable")

    def __init__(self, name: str, motor: Any) -> None:
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "_m", motor)

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        if (kp, kd) != HOLD_GAINS[self.name] and not (self.name in ALSO and (kp, kd) == TUNED[self.name][:2]):
            raise PermissionError(f"{self.name} send_mit refused: kp={kp} kd={kd} != hold gains {HOLD_GAINS[self.name]}")
        if (vel != 0.0 and self.name not in ALSO) or not abs(tau) <= FF_CAP_BY_MODEL[MODEL_OF[self.name]]:
            raise PermissionError(f"{self.name} send_mit refused: a held joint gets vel=0 and |tau| <= its cap")
        if not (math.isfinite(pos) and math.isfinite(vel)):
            raise PermissionError(f"{self.name} send_mit refused: non-finite pos/vel {pos}/{vel}")
        self._m.send_mit(float(pos), float(vel), float(kp), float(kd), float(tau))

    def __getattr__(self, attr: str) -> Any:
        if attr in HoldMotor._ALLOWED:
            return getattr(self._m, attr)
        raise PermissionError(f"{self.name}: '{attr}' is not allowed in this script")

    def __setattr__(self, attr: str, value: Any) -> None:
        raise PermissionError("HoldMotor is immutable")


class GuardedController:
    """The bus: close only, plus disable_all as the last-resort fallback. enable_all does not exist here."""
    _ALLOWED = ("close_bus", "close", "disable_all")

    def __init__(self, ctrl: Any) -> None:
        object.__setattr__(self, "_c", ctrl)

    def __getattr__(self, attr: str) -> Any:
        if attr in GuardedController._ALLOWED:
            return getattr(self._c, attr)
        raise PermissionError(f"controller: '{attr}' is not allowed in this script")

    def __setattr__(self, attr: str, value: Any) -> None:
        raise PermissionError("GuardedController is immutable")


# ---- freshness ----------------------------------------------------------------------------------------------------
def state_key(st: Any) -> Optional[tuple]:
    if st is None:
        return None
    return (st.status_code, st.pos, st.vel, st.torq, st.t_mos, st.t_rotor)


class Freshness:
    """Proof of fresh feedback without SDK timestamps: a changed decoded frame, or a successful register round trip."""

    def __init__(self, clock: Callable[[], float], call_error: type, heartbeat_after_s: float = HEARTBEAT_AFTER_S,
                 reg_timeout_ms: int = REG_TIMEOUT_MS) -> None:
        self.clock, self.call_error = clock, call_error
        self.heartbeat_after_s, self.reg_timeout_ms = heartbeat_after_s, reg_timeout_ms
        self.last_key: Optional[tuple] = None
        self.last_fresh: Optional[float] = None
        self.last_heartbeat_try: Optional[float] = None
        self.heartbeats_ok = 0
        self.heartbeats_failed = 0
        self.mode_seen: Optional[int] = None

    def observe(self, motor: Any, st: Any) -> None:
        now = self.clock()
        key = state_key(st)
        if key is not None and key != self.last_key:
            self.last_key, self.last_fresh = key, now
            return
        quiet_for = (now - self.last_fresh) if self.last_fresh is not None else math.inf
        since_try = (now - self.last_heartbeat_try) if self.last_heartbeat_try is not None else math.inf
        if quiet_for >= self.heartbeat_after_s and since_try >= self.heartbeat_after_s:
            self.last_heartbeat_try = now
            try:
                self.mode_seen = int(motor.get_register_u32(RID_CTRL_MODE, self.reg_timeout_ms))
            except self.call_error:
                self.heartbeats_failed += 1
                return
            self.heartbeats_ok += 1
            self.last_fresh = self.clock()

    def age(self) -> float:
        return math.inf if self.last_fresh is None else self.clock() - self.last_fresh


# ---- host pre-checks ----------------------------------------------------------------------------------------------
def processes_holding(path: str) -> list[str]:
    """PIDs (with names) of this user's processes that have `path` open — what `fuser` would print."""
    try:
        target = os.path.realpath(path)
    except OSError:
        return []
    found = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        fd_dir = f"/proc/{pid}/fd"
        try:
            for fd in os.listdir(fd_dir):
                try:
                    if os.path.realpath(os.path.join(fd_dir, fd)) == target:
                        with open(f"/proc/{pid}/comm") as fh:
                            found.append(f"{pid} ({fh.read().strip()})")
                        break
                except OSError:
                    continue
        except OSError:
            continue
    return found


def conflicting_processes() -> list[str]:
    out = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/comm") as fh:
                comm = fh.read().strip()
        except OSError:
            continue
        if comm in CONFLICTING_PROCS:
            out.append(f"{pid} ({comm})")
    return out


def host_prechecks(port: str) -> None:
    if not os.path.exists(port):
        raise Refused(f"{port} does not exist: plug in the USB2CAN adapter (journalctl -k should show 2e88:4603)")
    if not os.access(port, os.R_OK | os.W_OK):
        raise Refused(f"no read/write access to {port}: install HOST-04's udev rule (or re-login if you joined "
                      f"dialout), or as a stop-gap: sudo chmod 666 {port}")
    holders = processes_holding(port)
    if holders:
        raise Refused(f"{port} is already open by: {', '.join(holders)} (stop it first)")
    other = conflicting_processes()
    if other:
        raise Refused(f"another arm tool is running: {', '.join(other)} (stop it first)")


# ---- run context --------------------------------------------------------------------------------------------------
@dataclass
class Runtime:
    clock: Callable[[], float]
    sleep: Callable[[float], None]
    tty_reader: Callable[[str], Optional[str]]
    out_dir: Path
    mb: Any                                   # the motorbridge module (or the fake's stand-in)
    simulated: bool = False                   # names output files *_SIM; set with --simulate
    check_host: bool = True                   # port / process checks (off only for --simulate and tests)
    require_confirmation: bool = True         # typed ENABLE J1 on /dev/tty (off only for --simulate)
    install_signals: bool = True              # SIGINT/SIGTERM/SIGHUP -> abort (tests inject stop_flag instead)
    stop_flag: dict = field(default_factory=lambda: {"why": None})
    log: Callable[[str], None] = print


def read_dev_tty(prompt: str) -> Optional[str]:
    """Read one line from the controlling terminal itself (never stdin). None if there is no terminal."""
    try:
        with open("/dev/tty", "r+") as tty:
            tty.write(prompt)
            tty.flush()
            return tty.readline()
    except OSError:
        return None


def wait_state(m: Any, rt: Runtime, timeout_s: float, pred: Callable[[Any], bool] = lambda s: s is not None) -> Any:
    """Request a feedback frame and wait (bounded) until the cached state satisfies `pred`."""
    m.request_feedback()
    deadline = rt.clock() + timeout_s
    while True:
        st = m.get_state()
        if pred(st):
            return st
        if rt.clock() >= deadline:
            return st
        rt.sleep(0.002)


def precheck_bus(motors: dict[str, Any], rt: Runtime) -> dict[str, Any]:
    """Read-only: registers + one fresh frame per motor. Raises Refused with every problem found."""
    readings: dict[str, Any] = {}
    problems: list[str] = []
    CallError = rt.mb.CallError
    for name, mid, fid, model in MOTORS:
        m = motors[name]
        r: dict[str, Any] = {"motor_id": mid, "feedback_id": fid, "model": model}
        try:
            r["ctrl_mode"] = int(m.get_register_u32(RID_CTRL_MODE, PRECHECK_REG_TIMEOUT_MS))
            r["timeout_raw"] = int(m.get_register_u32(RID_TIMEOUT, PRECHECK_REG_TIMEOUT_MS))
            r["pmax_vmax_tmax"] = [round(float(m.get_register_f32(rid, PRECHECK_REG_TIMEOUT_MS)), 4)
                                   for rid in (RID_PMAX, RID_VMAX, RID_TMAX)]
        except CallError as e:
            problems.append(f"{name} (0x{mid:02X}) did not answer a register read: {e}")
            readings[name] = r
            continue
        exp = MODEL_LIMITS[model]
        if any(abs(a - b) > 0.01 for a, b in zip(r["pmax_vmax_tmax"], exp)):
            problems.append(f"{name}: PMAX/VMAX/TMAX {r['pmax_vmax_tmax']} != {model} {list(exp)}")
        st = wait_state(m, rt, 0.2)
        if st is None:
            problems.append(f"{name} (0x{mid:02X}) sent no feedback frame")
            readings[name] = r
            continue
        r.update(status=st.status_code, status_name=STATUS_NAMES.get(st.status_code, hex(st.status_code)),
                 pos=st.pos, vel=st.vel, torq=st.torq, t_mos=st.t_mos, t_rotor=st.t_rotor)
        if st.status_code != ST_DISABLED:
            problems.append(f"{name}: status {r['status_name']}, expected DISABLED")
        tol = REST_TOL_RAD if name == ACTIVE else OTHERS_REST_TOL_RAD
        if abs(st.pos) >= tol:
            problems.append(f"{name}: |pos| = {abs(st.pos):.4f} rad >= {tol} (not at rest at zero)")
        readings[name] = r
    if HOLD_OTHERS:
        for name, *_ in MOTORS:
            if name != ACTIVE and readings.get(name, {}).get("ctrl_mode") not in (None, MODE_MIT):
                problems.append(f"{name}: CTRL_MODE {readings[name]['ctrl_mode']} != 1 (MIT); --hold-others needs MIT")
    j1 = readings.get(ACTIVE, {})
    if j1.get("ctrl_mode") not in (None, MODE_MIT):
        problems.append(f"j1: CTRL_MODE {j1.get('ctrl_mode')} != 1 (MIT). This script never changes it. "
                        "If the vendor driver ran this power cycle, power-cycle the arm to restore the stored mode")
    if problems:
        raise Refused("; ".join(problems))
    return readings


def run(argv: list[str], rt: Optional[Runtime] = None, bus_factory: Optional[Callable] = None) -> int:
    global ACTIVE, HOLD_OTHERS, ALSO
    args = parse_args(argv)
    if isinstance(args, int):
        return args
    ACTIVE = args.joint
    HOLD_OTHERS = bool(getattr(args, "hold_others", False))
    ALSO = dict(getattr(args, "also", {}) or {})
    if rt is None:
        rt = make_runtime(args)
        if isinstance(rt, int):
            return rt
    rt.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    kind = "rest_check" if args.check_only else (f"estop_test_{ACTIVE}" if args.estop_test else f"first_move_{ACTIVE}")
    base = rt.out_dir / (f"{kind}_{stamp}" + ("_SIM" if rt.simulated else ""))
    summary: dict[str, Any] = {
        "tool": "first_move_j1.py", "version": SCRIPT_VERSION, "mode": kind, "simulated": rt.simulated,
        "started_utc": stamp, "port": args.port,
        "params": {"kp": args.kp, "kd": args.kd, "target_rad": args.target, "ramp_s": args.ramp_s,
                   "hold_s": args.hold_s, "settle_s": SETTLE_S, "loop_hz": LOOP_HZ,
                   "gains": args.gains, "gravity": args.gravity, "gravity_scale": args.gravity_scale, "gravity_joint_scale": args.joint_scale, "friction_nm": args.friction, "tau_ff": args.tau_ff, "track_abort_rad": TRACK_ABORT_RAD,
                   "stale_abort_s": STALE_ABORT_S, "vel_abort_rad_s": VEL_ABORT_RAD_S,
                   "others_drift_abort_rad": OTHERS_DRIFT_ABORT_RAD},
    }
    rc = EXIT_ABORTED
    try:
        rc = _run(args, rt, base, summary, bus_factory)
    except KeyboardInterrupt:
        # Only possible before control() installs its own handlers, i.e. before anything was enabled.
        summary["outcome"] = "interrupted_before_enable"
        rt.log("interrupted before anything was enabled; the bus was closed.")
        rc = EXIT_ABORTED
    finally:
        summary["exit_code"] = rc
        summary["finished_utc"] = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        with open(f"{base}.json", "w") as fh:
            json.dump(summary, fh, indent=2, default=str)
        rt.log(f"summary: {base}.json")
    return rc


def _run(args: argparse.Namespace, rt: Runtime, base: Path, summary: dict, bus_factory: Optional[Callable]) -> int:
    log = rt.log
    ver = getattr(rt.mb, "__version__", "?")
    if ver != MOTORBRIDGE_VERSION:
        summary["outcome"], summary["refusal"] = "refused", f"motorbridge {ver} != {MOTORBRIDGE_VERSION}"
        log(f"REFUSED: motorbridge {ver} installed; this script was checked against {MOTORBRIDGE_VERSION} only")
        return EXIT_REFUSED
    if rt.check_host:
        try:
            host_prechecks(args.port)
        except Refused as e:
            summary["outcome"], summary["refusal"] = "refused", str(e)
            log(f"REFUSED: {e}")
            return EXIT_REFUSED

    factory = bus_factory or rt.mb.Controller.from_dm_serial
    raw_ctrl = factory(args.port, BAUD)
    ctrl = GuardedController(raw_ctrl)
    raw_motors: dict[str, Any] = {}
    motors: dict[str, Any] = {}
    try:
        for name, mid, fid, model in MOTORS:
            raw_motors[name] = raw_ctrl.add_damiao_motor(mid, fid, model)
            motors[name] = J1Motor(raw_motors[name], max(g[0] for g in args.gains), max(g[1] for g in args.gains)) \
                if name == ACTIVE else \
                HoldMotor(name, raw_motors[name]) if HOLD_OTHERS else ReadOnlyMotor(name, raw_motors[name])
        try:
            readings = precheck_bus(motors, rt)
        except Refused as e:
            summary["outcome"], summary["refusal"] = "refused", str(e)
            log(f"REFUSED (nothing was enabled): {e}")
            return EXIT_REFUSED
        summary["precheck"] = readings
        q1 = readings[ACTIVE]["pos"]
        log("pre-checks OK: all 7 motors answer, DISABLED, |q| < "
            f"{REST_TOL_RAD} rad; j1 CTRL_MODE=1 (MIT); TIMEOUT registers = "
            f"{[readings[n]['timeout_raw'] for n, *_ in MOTORS]} (0 = no motor-side watchdog)")
        for name, *_ in MOTORS:
            r = readings[name]
            log(f"  {name}: {r['status_name']:9s} pos {r['pos']:+.4f} rad  torq {r['torq']:+.3f} Nm  "
                f"T {r['t_mos']:.0f}/{r['t_rotor']:.0f} C")
        if args.check_only:
            summary["outcome"] = "check_ok"
            log("check-only: nothing was enabled.")
            return EXIT_OK

        q_goal = q1 + args.move if args.move is not None else args.target
        lo, hi = URDF_LIMITS[ACTIVE]
        if not args.estop_test and not (lo <= q_goal <= hi):
            summary["outcome"], summary["refusal"] = "refused", f"goal {q_goal:+.3f} outside {ACTIVE} URDF [{lo}, {hi}]"
            log(f"REFUSED: goal {q_goal:+.3f} rad is outside {ACTIVE}'s URDF range [{lo}, {hi}]")
            return EXIT_REFUSED
        prof = Profile(q_start=q1, q_target=(q1 if args.estop_test else q_goal), ramp_s=args.ramp_s,
                       hold_s=args.hold_s)
        if not args.estop_test:
            exc = abs(prof.q_target - prof.q_start)
            max_exc = MAX_EXCURSION_BY_JOINT.get(ACTIVE, MAX_EXCURSION_RAD)
            if exc > max_exc:
                summary["outcome"], summary["refusal"] = "refused", f"excursion {exc:.3f} > {max_exc}"
                log(f"REFUSED: excursion {exc:.3f} rad > {max_exc}")
                return EXIT_REFUSED
        print_banner(args, prof, readings, log)
        if rt.require_confirmation and args.operator_go is not None:
            summary["confirmation"] = {"method": "operator_go_in_chat", "operator_go": args.operator_go}
            log(f"confirmation: the operator's go, given in chat and passed by the Claude session: {args.operator_go!r}")
        elif rt.require_confirmation:
            typed = rt.tty_reader(f"Type {CONFIRM_PHRASE} to energise motor 1 (anything else aborts): ")
            if typed is None:
                summary["outcome"], summary["refusal"] = "refused", "no controlling terminal for the confirmation"
                log("REFUSED: no controlling terminal (/dev/tty) for the typed confirmation")
                return EXIT_REFUSED
            if typed.strip() != CONFIRM_PHRASE:
                summary["outcome"], summary["refusal"] = "refused", "confirmation phrase not typed"
                log("REFUSED: confirmation phrase not typed; nothing was enabled")
                return EXIT_REFUSED
            summary["confirmation"] = {"method": "typed_on_dev_tty", "phrase": CONFIRM_PHRASE}
        # Re-check rest right before enabling (someone may have touched the arm while you read the banner).
        try:
            readings2 = precheck_bus(motors, rt)
        except Refused as e:
            summary["outcome"], summary["refusal"] = "refused", "re-check before enable: " + str(e)
            log(f"REFUSED (re-check before enable; nothing was enabled): {e}")
            return EXIT_REFUSED
        others_start = {n: readings2[n]["pos"] for n, *_ in MOTORS if n != ACTIVE}
        return control(args, rt, ctrl, motors, prof, others_start, base, summary)
    finally:
        for m in raw_motors.values():
            try:
                m.close()
            except Exception:  # noqa: BLE001 - freeing a local handle must not mask the real outcome
                pass
        try:
            ctrl.close_bus()
        except Exception as e:  # noqa: BLE001
            log(f"warning: close_bus failed: {e}")
        try:
            ctrl.close()
        except Exception:  # noqa: BLE001
            pass


def print_banner(args: argparse.Namespace, prof: Profile, readings: dict, log: Callable[[str], None]) -> None:
    deg = math.degrees(prof.q_target - prof.q_start)
    log("=" * 100)
    if args.estop_test:
        log("E-STOP TEST: joint1 only (motor 0x01). Motor 1 will be ENABLED and HOLD its measured position "
            f"{prof.q_start:+.4f} rad for up to {args.estop_wait_s:.0f} s.")
        log("  PRESS THE HARDWARE E-STOP during the hold. Expected: feedback lost -> 'e-stop observed'.")
    else:
        side = ("LEFT" if deg > 0 else "RIGHT") if ACTIVE == "j1" else ("negative direction, away from the fold stop" if deg < 0 else "positive direction")
        log(f"POWERED MOTION: {ACTIVE} moves. " + ("Every other motor is ENABLED, holding where it is." if HOLD_OTHERS else "Every other motor stays DISABLED."))
        log(f"  start {prof.q_start:+.4f} rad (measured) -> target {prof.q_target:+.4f} rad ({deg:+.1f} deg: the arm "
            f"swings to the ROBOT'S {side}) -> back to start")
        log(f"  settle {SETTLE_S:.1f} s, ramp {prof.ramp_s:.1f} s, hold {prof.hold_s:.1f} s, ramp {prof.ramp_s:.1f} s, "
            f"hold {prof.hold_s:.1f} s ({prof.duration:.1f} s); peak speed {prof.peak_speed:.3f} rad/s")
    if ALSO:
        log("  --also: moving together with " + ACTIVE + ": " + ", ".join(
            f"{n} {mv:+.3f} rad (kp {TUNED[n][0]:g} kd {TUNED[n][1]:g} friction {TUNED[n][3]:g})" for n, mv in ALSO.items()))
    if HOLD_OTHERS:
        log("  --hold-others: every other motor is ENABLED and holds where it is: "
            + ", ".join(f"{n} kp{g[0]:g}/kd{g[1]:g}" for n, g in HOLD_GAINS.items() if n != ACTIVE)
            + f"; abort if one moves > {HOLD_TRACK_ABORT_RAD} rad. On ANY abort/end ALL motors are disabled: "
              "a lifted arm WILL drop.")
    log(f"  MIT gains (kp:kd) {', '.join(f'{a:g}:{b:g}' for a, b in args.gains)}"
        + (f" (sweep: {len(args.gains)} back-and-forth moves in one enable)" if len(args.gains) > 1 else "")
        + f"; tau_ff {args.tau_ff:+g}" + (f" + URDF gravity x{args.gravity_scale:g} on every motor"
           + "".join(f", {k} x{v:g}" for k, v in args.joint_scale.items()) if args.gravity else "")
        + (f" + friction {args.friction:g} Nm along the commanded velocity" if args.friction else "")
        + f"; {LOOP_HZ:.0f} Hz command stream")
    log(f"  aborts: |target-pos|>{TRACK_ABORT_RAD} rad, |torque|>kp*{TRACK_ABORT_RAD}+kd*{VEL_ABORT_RAD_S}+{TORQUE_MARGIN_NM}+|tau_ff| Nm, feedback "
        f"stale>{STALE_ABORT_S} s, |vel|>{VEL_ABORT_RAD_S} rad/s, j1 not ENABLED/fault, motors 2-7 not DISABLED or "
        f"moved>{OTHERS_DRIFT_ABORT_RAD} rad, stream gap>{STALL_ABORT_S * 1000:.0f} ms, bus error, Ctrl-C")
    log(f"  on abort: hold the current position {ABORT_HOLD_S} s, then disable motor 1")
    log(f"  TIMEOUT register = {readings[ACTIVE]['timeout_raw']} on motor 1 (0 = no motor-side watchdog): if this "
        "process dies, the motor keeps its last command -> CUT 24 V")
    log("  E-STOP: put your hand on the hardware e-stop / 24 V switch NOW. Ctrl-C = software stop.")
    log("=" * 100)


def _other_cols(so: Any, ff: float) -> list[str]:
    if so is None:
        return ["", "", "", f"{ff:+.3f}"]
    return [f"{so.pos:+.5f}", f"{so.vel:+.4f}", f"{so.torq:+.3f}", f"{ff:+.3f}"]


def score_segments(rows: list[tuple], gains: list[tuple[float, float]], move_sign: float) -> list[dict]:
    """Per gain pair: RMS tracking error over the ramps, mean hold error, overshoot past the target during the hold,
    and an oscillation count (velocity sign changes above 0.05 rad/s while holding). Lower score = better."""
    out = []
    for k, (kp, kd) in enumerate(gains):
        seg = [r for r in rows if r[0] == k]
        if not seg:
            continue
        ramp = [r[2] for r in seg if r[1] in ("ramp_up", "ramp_down")]
        hold = [r for r in seg if r[1] == "hold"]
        still = [r for r in seg if r[1] in ("hold", "final_hold")]
        rms = math.sqrt(sum(e * e for e in ramp) / len(ramp)) if ramp else math.nan
        hold_err = sum(r[2] for r in hold) / len(hold) if hold else math.nan
        overshoot = max([0.0] + [-move_sign * r[2] for r in hold])
        flips, last = 0, 0.0
        for r in still:
            if abs(r[3]) > 0.05:
                if last and (r[3] > 0) != (last > 0):
                    flips += 1
                last = r[3]
        complete = any(r[1] == "final_hold" for r in seg)
        score = rms + abs(hold_err) + 2.0 * overshoot + 0.01 * flips if complete else math.inf
        out.append({"seg": k, "kp": kp, "kd": kd, "rms_track_rad": rms, "hold_err_rad": hold_err,
                    "overshoot_rad": overshoot, "osc_flips": flips, "peak_torque_nm": max(abs(r[4]) for r in seg),
                    "complete": complete, "score": score})
    return out


def control(args: argparse.Namespace, rt: Runtime, ctrl: GuardedController, motors: dict[str, Any], prof: Profile,
            others_start: dict[str, float], base: Path, summary: dict) -> int:
    """The only code path that energises anything. try/finally always disables motor 1."""
    log, clock, sleep = rt.log, rt.clock, rt.sleep
    CallError = rt.mb.CallError
    j1 = motors[ACTIVE]
    others = [n for n, *_ in MOTORS if n != ACTIVE]
    fresh = Freshness(clock, CallError)
    held: dict[str, float] = {}            # --hold-others: name -> hold position (measured at its enable)
    held_fresh = {n: Freshness(clock, CallError) for n in others}
    gm = None
    if args.gravity:
        import gravity  # local module next to this script
        gm = gravity.GravityModel()
    q_now = dict(others_start)
    ff_now: dict[str, float] = {n: 0.0 for n, *_ in MOTORS}

    def update_ff(fade: float = 1.0) -> None:
        """Feed-forward per motor from the latest measured pose: gravity (if --gravity) + --tau-ff on the active.
        The URDF gravity applies at once; only a --gravity-joint-scale above 1 fades in (`fade` 0..1 over FF_FADE_S)."""
        g = gm.hold_torques(q_now) if gm is not None else {}
        for n, *_ in MOTORS:
            js = args.joint_scale.get(n, 1.0)
            ff = (args.gravity_scale * (min(js, 1.0) + fade * max(js - 1.0, 0.0)) * g.get(n, 0.0)
                  + (args.tau_ff if n == ACTIVE else 0.0))
            cap = FF_CAP_BY_MODEL[MODEL_OF[n]]
            if abs(ff) > cap:
                raise Abort("gravity", f"{n} feed-forward {ff:+.2f} Nm exceeds {cap} Nm at this pose")
            ff_now[n] = ff

    n_seg = len(args.gains)

    def gains_at(t: float) -> tuple[int, float, float]:
        k = min(n_seg - 1, int(t // prof.duration)) if not args.estop_test else 0
        return (k, *args.gains[k])
    rows: list[tuple] = []        # (seg, phase, err, vel, torque) for the sweep score
    movers: dict[str, Profile] = {}   # --also joints: their own profile, same timing as the active one
    mstats: dict[str, dict] = {}
    mref: dict[str, tuple] = {}       # name -> (phase, q_ref, v_ref) this cycle
    period = 1.0 / LOOP_HZ
    window = (args.estop_wait_s if args.estop_test else prof.duration * n_seg)
    stats: dict[str, Any] = {"n_cmd": 0, "max_abs_err": 0.0, "max_abs_torque": 0.0, "max_abs_vel": 0.0,
                             "max_gap_s": 0.0, "peak_pos": None, "min_pos": None}
    trigger: Optional[Abort] = None
    outcome = "aborted"
    last_cmd: Optional[float] = None
    last_meas: Optional[float] = None
    last_meas_t: Optional[float] = None
    rr = 0
    csv_path = f"{base}.csv"
    fh = open(csv_path, "w", newline="")
    w = csv.writer(fh)
    w.writerow(["t_s", "phase", "target_rad", "target_vel_rad_s", "pos_rad", "vel_rad_s", "torque_nm", "status",
                "fb_age_s", "cmd_gap_s", "others_max_dev_rad", "seg", "kp", "kd", "tau_ff_nm"]
               + [f"{n}_{k}" for n in others for k in ("pos", "vel", "torq", "ff")])
    old_handlers = {}

    def on_signal(signum, _frame):  # noqa: ANN001 - signal handler signature
        rt.stop_flag["why"] = signal.Signals(signum).name

    if rt.install_signals:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old_handlers[sig] = signal.signal(sig, on_signal)

    t0 = None
    try:
        try:
            # ---- --hold-others: enable every other motor and hold it where it is ---------------------------------
            if HOLD_OTHERS:
                update_ff(0.0)
                for n in others:
                    kp_h, kd_h = HOLD_GAINS[n]
                    motors[n].enable()
                    held[n] = math.nan          # enabled: the finally must disable it even if confirmation fails
                    so = wait_state(motors[n], rt, ENABLE_CONFIRM_S,
                                    lambda s: s is not None and s.status_code == ST_ENABLED)
                    if so is None or so.status_code != ST_ENABLED:
                        raise Abort("enable", f"{n} did not report ENABLED within {ENABLE_CONFIRM_S} s")
                    held[n] = so.pos
                    q_now[n] = so.pos
                    motors[n].send_mit(so.pos, 0.0, kp_h, kd_h, ff_now[n])
                    held_fresh[n].observe(motors[n], so)
                stats["held_at"] = dict(held)
                for n, mv in ALSO.items():
                    lo, hi = URDF_LIMITS[n]
                    if not lo <= held[n] + mv <= hi:
                        raise Abort("excursion", f"--also {n}: goal {held[n] + mv:+.3f} outside its URDF range [{lo}, {hi}]")
                    movers[n] = Profile(q_start=held[n], q_target=held[n] + mv, ramp_s=prof.ramp_s, hold_s=prof.hold_s)
                    mstats[n] = {"max_abs_err": 0.0, "hold_err_sum": 0.0, "hold_err_n": 0}
                log("holding " + ", ".join(f"{n} {q:+.3f} (kp {HOLD_GAINS[n][0]:g}, ff {ff_now[n]:+.2f})"
                                           for n, q in held.items()))
            # ---- enable motor 1 and confirm it --------------------------------------------------------------------
            log("enabling motor 1 ...")
            j1.enable()
            st = wait_state(j1, rt, ENABLE_CONFIRM_S, lambda s: s is not None and s.status_code == ST_ENABLED)
            if st is None or st.status_code != ST_ENABLED:
                raise Abort("enable", f"motor 1 did not report ENABLED within {ENABLE_CONFIRM_S} s "
                                      f"(status {None if st is None else STATUS_NAMES.get(st.status_code)})")
            fresh.observe(j1, st)
            q_meas = st.pos
            q_now[ACTIVE] = q_meas
            update_ff(0.0)
            # The first command after enable targets the measured position.
            j1.send_mit(q_meas, 0.0, *args.gains[0], ff_now[ACTIVE])
            last_cmd, t0, t_prev = q_meas, clock(), clock()
            prof.q_start = q_meas
            if not args.estop_test:
                prof_note = abs(prof.q_target - q_meas)
                if prof_note > MAX_EXCURSION_BY_JOINT.get(ACTIVE, MAX_EXCURSION_RAD):
                    raise Abort("excursion", f"target {prof.q_target:+.4f} is {prof_note:.3f} rad from the "
                                             f"measured start {q_meas:+.4f}")
            else:
                prof.q_target = q_meas
            stats["first_cmd_rad"] = q_meas
            log(f"motor 1 ENABLED; first command = measured position {q_meas:+.5f} rad")
            next_t = t0 + period
            while True:
                if rt.stop_flag["why"]:
                    raise Abort("signal", f"{rt.stop_flag['why']} received")
                now = clock()
                t = now - t0
                if t >= window:
                    if args.estop_test:
                        raise Abort("estop_not_observed", f"no feedback loss within {window:.0f} s: the e-stop "
                                                          "was not pressed, or it did not remove motor power")
                    outcome = "completed"
                    break
                seg, kp_c, kd_c = gains_at(t)
                if args.estop_test:
                    phase, q_ref, v_ref = "estop_hold", prof.q_target, 0.0
                else:
                    phase, q_ref, v_ref = prof.at(t - seg * prof.duration)
                update_ff(min(1.0, t / FF_FADE_S))
                if args.friction:
                    ff_now[ACTIVE] += args.friction * math.tanh(v_ref / FRICTION_VEL_EPS)
                    if abs(ff_now[ACTIVE]) > FF_CAP_BY_MODEL[MODEL_OF[ACTIVE]]:
                        raise Abort("gravity", f"{ACTIVE} feed-forward {ff_now[ACTIVE]:+.2f} Nm (gravity + friction) "
                                               f"exceeds {FF_CAP_BY_MODEL[MODEL_OF[ACTIVE]]} Nm")
                tlim = torque_limit(kp_c, kd_c) + abs(ff_now[ACTIVE])
                gap = now - t_prev
                stats["max_gap_s"] = max(stats["max_gap_s"], gap)
                if gap > STALL_ABORT_S:
                    raise Abort("stream", f"{gap * 1000:.0f} ms between commands (> {STALL_ABORT_S * 1000:.0f} ms)")
                try:
                    j1.send_mit(q_ref, v_ref, kp_c, kd_c, ff_now[ACTIVE])
                except CallError as e:
                    raise Abort("bus", f"send_mit failed: {e}") from e
                t_prev = now
                last_cmd = q_ref
                stats["n_cmd"] += 1
                if HOLD_OTHERS:
                    for n in others:
                        try:
                            if n in movers:
                                mref[n] = movers[n].at(t - seg * prof.duration)
                                kp_m, kd_m, _, fr_m = TUNED[n]
                                ff_m = ff_now[n] + fr_m * math.tanh(mref[n][2] / FRICTION_VEL_EPS)
                                if abs(ff_m) > FF_CAP_BY_MODEL[MODEL_OF[n]]:
                                    raise Abort("gravity", f"{n} feed-forward {ff_m:+.2f} Nm exceeds its cap")
                                motors[n].send_mit(mref[n][1], mref[n][2], kp_m, kd_m, ff_m)
                                continue
                            motors[n].send_mit(held[n], 0.0, *HOLD_GAINS[n], ff_now[n])
                        except CallError as e:
                            raise Abort("bus", f"{n} hold send_mit failed: {e}") from e
                else:
                    # one other motor per cycle (round robin), so each is re-read ~33 times per second
                    name = others[rr % len(others)]
                    rr += 1
                    try:
                        motors[name].request_feedback()
                    except CallError as e:
                        raise Abort("bus", f"{name} request_feedback failed: {e}") from e
                st = j1.get_state()
                fresh.observe(j1, st)
                if fresh.mode_seen is not None and fresh.mode_seen != MODE_MIT:
                    raise Abort("mode", f"motor 1 CTRL_MODE changed to {fresh.mode_seen}")
                age = fresh.age()
                if st is not None and age <= STALE_ABORT_S:
                    last_meas, last_meas_t = st.pos, clock()
                    q_now[ACTIVE] = st.pos
                odev = 0.0
                held_abort: Optional[Abort] = None
                for n in (others if HOLD_OTHERS else ()):
                    so = motors[n].get_state()
                    held_fresh[n].observe(motors[n], so)
                    if held_fresh[n].age() > STALE_ABORT_S:
                        held_abort = held_abort or Abort("held", f"no fresh feedback from held {n} for {held_fresh[n].age():.3f} s")
                    if so is None:
                        continue
                    kp_h, kd_h = TUNED[n][:2] if n in movers else HOLD_GAINS[n]
                    q_now[n] = so.pos
                    if n in movers and n in mref:
                        dev = abs(mref[n][1] - so.pos)
                        ms = mstats[n]
                        ms["max_abs_err"] = max(ms["max_abs_err"], dev)
                        if mref[n][0] == "hold":
                            ms["hold_err_sum"] += mref[n][1] - so.pos
                            ms["hold_err_n"] += 1
                        if dev > TRACK_ABORT_RAD:
                            held_abort = held_abort or Abort("tracking", f"--also {n} |target-pos| = {dev:.3f} rad > {TRACK_ABORT_RAD}")
                    else:
                        dev = abs(so.pos - held[n])
                        odev = max(odev, dev)
                    if so.status_code != ST_ENABLED:
                        held_abort = held_abort or Abort("held", f"held {n} reports {STATUS_NAMES.get(so.status_code, so.status_code)}")
                    if dev > HOLD_TRACK_ABORT_RAD:
                        held_abort = held_abort or Abort("held", f"held {n} is {dev:.3f} rad from its hold (> {HOLD_TRACK_ABORT_RAD})")
                    tl_h = kp_h * HOLD_TRACK_ABORT_RAD + kd_h * VEL_ABORT_RAD_S + TORQUE_MARGIN_NM + abs(ff_now[n]) \
                        + (TUNED[n][3] if n in movers else 0.0)
                    if abs(so.torq) > tl_h:
                        held_abort = held_abort or Abort("held", f"held {n} |torque| = {abs(so.torq):.2f} Nm > {tl_h:.2f}")
                    vl_h = HOLD_VEL_ABORT_RAD_S
                    if abs(so.vel) > vl_h:
                        held_abort = held_abort or Abort("held", f"held {n} |vel| = {abs(so.vel):.3f} rad/s > {vl_h}")
                for n in (() if HOLD_OTHERS else others):
                    so = motors[n].get_state()
                    if so is None:
                        continue
                    if so.status_code != ST_DISABLED:
                        raise Abort("others", f"{n} reports {STATUS_NAMES.get(so.status_code, so.status_code)} "
                                              "(only motor 1 may be enabled)")
                    odev = max(odev, abs(so.pos - others_start[n]))
                if not HOLD_OTHERS and odev > OTHERS_DRIFT_ABORT_RAD:
                    raise Abort("others", f"a disabled joint moved {odev:.3f} rad (> {OTHERS_DRIFT_ABORT_RAD}): "
                                          "someone/something is moving the arm")
                if st is not None:
                    w.writerow([f"{t:.4f}", phase, f"{q_ref:+.6f}", f"{v_ref:+.6f}", f"{st.pos:+.6f}",
                                f"{st.vel:+.5f}", f"{st.torq:+.4f}", st.status_code, f"{age:.4f}", f"{gap:.4f}",
                                f"{odev:.4f}", seg, kp_c, kd_c, f"{ff_now[ACTIVE]:+.3f}"]
                               + [v for n in others for v in _other_cols(motors[n].get_state(), ff_now[n])])
                    rows.append((seg, phase, q_ref - st.pos, st.vel, st.torq))
                    err = abs(q_ref - st.pos)
                    stats["max_abs_err"] = max(stats["max_abs_err"], err)
                    stats["max_abs_torque"] = max(stats["max_abs_torque"], abs(st.torq))
                    stats["max_abs_vel"] = max(stats["max_abs_vel"], abs(st.vel))
                    if phase == "hold":
                        stats["hold_err_sum"] = stats.get("hold_err_sum", 0.0) + (q_ref - st.pos)
                        stats["hold_err_n"] = stats.get("hold_err_n", 0) + 1
                    stats["peak_pos"] = st.pos if stats["peak_pos"] is None else max(stats["peak_pos"], st.pos)
                    stats["min_pos"] = st.pos if stats["min_pos"] is None else min(stats["min_pos"], st.pos)
                if held_abort is not None:
                    raise held_abort
                if age > STALE_ABORT_S:
                    raise Abort("stale", f"no fresh feedback from motor 1 for {age:.3f} s (> {STALE_ABORT_S} s; "
                                         f"heartbeat failures {fresh.heartbeats_failed})")
                if st is None:
                    continue
                if st.status_code != ST_ENABLED:
                    raise Abort("status", f"motor 1 reports {STATUS_NAMES.get(st.status_code, st.status_code)}")
                if abs(q_ref - st.pos) > TRACK_ABORT_RAD:
                    raise Abort("tracking", f"|target-pos| = {abs(q_ref - st.pos):.3f} rad > {TRACK_ABORT_RAD} "
                                            f"(target {q_ref:+.4f}, pos {st.pos:+.4f})")
                if abs(st.torq) > tlim:
                    raise Abort("torque", f"|torque| = {abs(st.torq):.2f} Nm > {tlim:.2f} Nm")
                vlim = VEL_ABORT_BY_JOINT.get(ACTIVE, VEL_ABORT_RAD_S)
                if abs(st.vel) > vlim:
                    raise Abort("velocity", f"|vel| = {abs(st.vel):.3f} rad/s > {vlim}")
                # pace the loop
                slack = next_t - clock()
                if slack > 0:
                    sleep(slack)
                    next_t += period
                else:
                    next_t = clock() + period
        except Abort as a:
            trigger = a
        except CallError as e:
            trigger = Abort("bus", str(e))
        except Exception as e:  # noqa: BLE001 - any bug here must still end in disable (finally)
            trigger = Abort("internal", f"{type(e).__name__}: {e}")

        if trigger is not None:
            estop_seen = args.estop_test and trigger.trigger in ("stale", "bus")
            outcome = "estop_observed" if estop_seen else "aborted"
            log(("E-STOP OBSERVED: " if estop_seen else "ABORT: ") + f"[{trigger.trigger}] {trigger.detail}")
            # hold the current position for ABORT_HOLD_S, then disable (in finally)
            if t0 is not None and trigger.trigger not in ("bus",) and not estop_seen:
                fresh_meas = last_meas is not None and last_meas_t is not None and \
                    clock() - last_meas_t <= STALE_ABORT_S and trigger.trigger != "stale"
                hold = last_meas if fresh_meas else last_cmd
                stats["abort_hold_rad"] = hold
                stats["abort_hold_source"] = "measured" if fresh_meas else "last_command"
                log(f"holding {hold:+.5f} rad ({stats['abort_hold_source']}) for {ABORT_HOLD_S} s ...")
                t_end = clock() + ABORT_HOLD_S
                while clock() < t_end:
                    try:
                        j1.send_mit(hold, 0.0, *gains_at(clock() - t0)[1:], ff_now[ACTIVE])
                        for n, q in held.items():
                            if n in movers:
                                motors[n].send_mit(q_now[n], 0.0, *TUNED[n][:2], ff_now[n])
                            elif math.isfinite(q):
                                motors[n].send_mit(q, 0.0, *HOLD_GAINS[n], ff_now[n])
                    except Exception as e:  # noqa: BLE001
                        log(f"hold command failed ({e}); going straight to disable")
                        break
                    sleep(period)
    finally:
        # ---- ALWAYS: disable motor 1 ---------------------------------------------------------------------------
        disabled_ok = False
        try:
            j1.disable()
            disable_sent = True
        except Exception as e:  # noqa: BLE001
            disable_sent = False
            log(f"motor 1 disable() failed: {e}; trying disable_all() ...")
            try:
                ctrl.disable_all()
            except Exception as e2:  # noqa: BLE001
                log(f"disable_all() failed too: {e2}")
        if disable_sent:
            try:
                st = wait_state(j1, rt, DISABLE_CONFIRM_S, lambda s: s is not None and s.status_code == ST_DISABLED)
                disabled_ok = st is not None and st.status_code == ST_DISABLED
            except Exception as e:  # noqa: BLE001
                log(f"disable was sent but its confirmation could not be read: {e}")
        if held:
            # --hold-others: disable every motor this script enabled, and confirm each
            for n in held:
                try:
                    motors[n].disable()
                except Exception as e:  # noqa: BLE001
                    log(f"{n} disable() failed: {e}; trying disable_all() ...")
                    try:
                        ctrl.disable_all()
                    except Exception as e2:  # noqa: BLE001
                        log(f"disable_all() failed too: {e2}")
            not_off = []
            for n in held:
                try:
                    so = wait_state(motors[n], rt, DISABLE_CONFIRM_S,
                                    lambda s: s is not None and s.status_code == ST_DISABLED)
                    if so is None or so.status_code != ST_DISABLED:
                        not_off.append(n)
                except Exception:  # noqa: BLE001
                    not_off.append(n)
            stats["held_disabled_confirmed"] = not not_off
            if not_off:
                disabled_ok = False
                log(f"WARNING: held motors not confirmed DISABLED: {not_off} -> CUT 24 V NOW")
            else:
                log(f"held motors DISABLED (confirmed): {', '.join(held)}")
        if trigger is not None and trigger.trigger == "others" and "ENABLED" in trigger.detail:
            # Something other than this script enabled a motor. Every cycle checked that motors 2-7 stayed within
            # OTHERS_DRIFT_ABORT_RAD of their folded rest, so the arm is still in the fold, where joint2/joint3 rest
            # on their mechanical stop and disabling cannot drop anything.
            log("another motor reported ENABLED: sending disable_all() as well (the arm is in its folded rest)")
            try:
                ctrl.disable_all()
            except Exception as e:  # noqa: BLE001
                log(f"disable_all() failed: {e} -> CUT 24 V")
        if disabled_ok:
            log("motor 1 DISABLED (confirmed by its feedback frame).")
        elif outcome == "estop_observed":
            log("motor 1 cannot confirm DISABLED because it has no power (expected after the e-stop). Release the "
                "e-stop: Damiao motors power up DISABLED. Then run --check-only.")
        else:
            log("WARNING: could not confirm motor 1 is DISABLED -> CUT 24 V NOW, then investigate.")
        for sig, h in old_handlers.items():
            signal.signal(sig, h)
        fh.close()
        stats["disabled_confirmed"] = disabled_ok
        stats["heartbeats_ok"], stats["heartbeats_failed"] = fresh.heartbeats_ok, fresh.heartbeats_failed
        if t0 is not None and stats["n_cmd"]:
            stats["mean_rate_hz"] = stats["n_cmd"] / max(1e-9, (clock() - t0))
        summary.update(outcome=outcome, trigger=(None if trigger is None else
                                                 {"id": trigger.trigger, "detail": trigger.detail}),
                       stats=stats, csv=csv_path, q_start=prof.q_start, q_target=prof.q_target)
    if stats.get("hold_err_n"):
        stats["hold_mean_err_rad"] = stats["hold_err_sum"] / stats["hold_err_n"]
        summary["stats"] = stats
        if abs(stats["hold_mean_err_rad"]) > ARRIVAL_WARN_RAD:
            log(f"WARNING: during the hold joint1 sat {stats['hold_mean_err_rad']:+.3f} rad from the target (more than "
                f"{ARRIVAL_WARN_RAD}). Friction larger than kp can overcome, or a jam. A jam cannot trip the 0.10 rad "
                "abort on a move this small, so look at the CSV before going further.")
    for n, ms in mstats.items():
        ms["hold_mean_err_rad"] = ms["hold_err_sum"] / ms["hold_err_n"] if ms["hold_err_n"] else None
        log(f"--also {n}: max |target-pos| {ms['max_abs_err']:.4f} rad, hold err "
            + (f"{ms['hold_mean_err_rad']:+.4f}" if ms["hold_mean_err_rad"] is not None else "n/a"))
    if mstats:
        summary["also"] = mstats
    if len(args.gains) > 1 and rows:
        table = score_segments(rows, args.gains, 1.0 if prof.q_target >= prof.q_start else -1.0)
        summary["sweep"] = table
        log("sweep (lower score = better; incomplete pairs score inf):")
        log("   kp     kd   rms_track  hold_err  overshoot  osc  peak_Nm   score")
        for r in table:
            log(f"  {r['kp']:5g} {r['kd']:5g}   {r['rms_track_rad']:8.4f}  {r['hold_err_rad']:+8.4f}  "
                f"{r['overshoot_rad']:8.4f}  {r['osc_flips']:3d}  {r['peak_torque_nm']:6.2f}  {r['score']:7.4f}")
        best = min(table, key=lambda r: r["score"])
        if math.isfinite(best["score"]):
            log(f"best: kp {best['kp']:g} kd {best['kd']:g}")
    log(f"outcome: {outcome}" + ("" if trigger is None else f" (trigger {trigger.trigger}: {trigger.detail})"))
    log(f"max |target-pos| {stats['max_abs_err']:.4f} rad, max |torque| {stats['max_abs_torque']:.3f} Nm, "
        f"max |vel| {stats['max_abs_vel']:.3f} rad/s, max gap {stats['max_gap_s'] * 1000:.1f} ms, "
        f"commands {stats['n_cmd']}, csv: {csv_path}")
    return EXIT_OK if outcome in ("completed", "estop_observed") else EXIT_ABORTED


# ---- CLI ------------------------------------------------------------------------------------------------------------
def parse_args(argv: list[str]) -> argparse.Namespace | int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--check-only", action="store_true", help="read-only pre-checks; never enables anything")
    g.add_argument("--estop-test", action="store_true", help="enable motor 1, hold, wait for the hardware e-stop")
    ap.add_argument("--joint", choices=sorted(URDF_LIMITS), default="j1", help="the one joint to move")
    ap.add_argument("--port", default=PORT_DEFAULT)
    ap.add_argument("--kp", type=float, default=KP_MAX, help=f"MIT kp (<= {KP_MAX})")
    ap.add_argument("--kd", type=float, default=KD_DEFAULT, help=f"MIT kd (<= {KD_MAX})")
    ap.add_argument("--hold-others", action="store_true",
                    help="enable every other motor and hold it where it is (HOLD_GAINS); off = others stay DISABLED")
    ap.add_argument("--gravity", action="store_true", help="URDF gravity feed-forward on every motor (needs --hold-others)")
    ap.add_argument("--gravity-scale", type=float, default=1.0, help="multiply the URDF gravity torque (0..1.5)")
    ap.add_argument("--gravity-joint-scale", default="", metavar="jN=X,...",
                    help="extra per-joint gravity scale (0..3), e.g. j3=2.5 (2026-10-17: j3 measured ~2.5x the URDF)")
    ap.add_argument("--friction", type=float, default=0.0, metavar="NM",
                    help=f"Coulomb friction feed-forward on the moving joint, along the commanded velocity (0..{FRICTION_MAX_NM})")
    ap.add_argument("--tuned", action="store_true",
                    help="use TUNED (2026-10-17 sweeps) for the active joint's kp/kd/friction and every gravity scale")
    ap.add_argument("--also", default="", metavar="jN=MOVE,...",
                    help="extra joints (j1-j6) that move together with --joint, each by MOVE rad, with TUNED gains")
    ap.add_argument("--sweep", default=None, metavar="KP:KD,...",
                    help=f"run the move once per gain pair, back to back in one enable (<= {SWEEP_MAX} pairs)")
    ap.add_argument("--tau-ff", type=float, default=0.0, help=f"constant feed-forward torque, Nm (|x| <= {TAU_FF_MAX})")
    ap.add_argument("--target", type=float, default=0.10, help="joint1 target, rad (+ = robot's left)")
    ap.add_argument("--move", type=float, default=None,
                    help=f"move relative to the measured start, rad (|move| <= {MAX_EXCURSION_RAD}); overrides --target")
    ap.add_argument("--ramp-s", type=float, default=2.0, help=f"each ramp, s (>= {MIN_RAMP_S})")
    ap.add_argument("--hold-s", type=float, default=MIN_HOLD_S, help=f"each hold, s (>= {MIN_HOLD_S})")
    ap.add_argument("--estop-wait-s", type=float, default=ESTOP_WAIT_DEFAULT_S)
    ap.add_argument("--operator-go", default=None, metavar="TEXT",
                    help="the operator's go, given in chat to the Claude session running this command; replaces the "
                         "typed ENABLE J1 and is recorded verbatim in the JSON summary")
    ap.add_argument("--out-dir", default=None,
                    help="default runs/ next to this file; --simulate defaults to runs_sim/ so simulated files can "
                         "never be mistaken for, or filed as, hardware evidence")
    ap.add_argument("--simulate", action="store_true", help="run against fakebus.py in simulated time")
    ap.add_argument("--simulate-fault", default="none",
                    choices=["none", "stuck", "torque", "stale", "other_enabled", "other_moves", "bus_error",
                             "sigint", "estop"])
    try:
        args = ap.parse_args(argv)
    except SystemExit as e:
        return EXIT_USAGE if e.code else EXIT_OK
    problems = []
    if not (abs(args.tau_ff) <= TAU_FF_MAX):
        problems.append(f"--tau-ff must be within +-{TAU_FF_MAX} Nm")
    args.gains = [(args.kp, args.kd)]
    if args.sweep:
        try:
            args.gains = [tuple(float(x) for x in pair.split(":")) for pair in args.sweep.split(",")]
            assert all(len(g) == 2 for g in args.gains) and 1 <= len(args.gains) <= SWEEP_MAX
        except (ValueError, AssertionError):
            problems.append(f"--sweep must be 1-{SWEEP_MAX} KP:KD pairs, e.g. 40:0.5,60:1.0")
            args.gains = [(args.kp, args.kd)]
        for kp_s, kd_s in args.gains:
            if not (0.0 < kp_s <= KP_MAX and 0.0 <= kd_s <= KD_MAX):
                problems.append(f"--sweep pair {kp_s:g}:{kd_s:g} outside kp (0, {KP_MAX}], kd [0, {KD_MAX}]")
    args.joint_scale = {}
    if args.gravity_joint_scale:
        try:
            for item in args.gravity_joint_scale.split(","):
                k, v = item.split("=")
                assert k in ("j1", "j2", "j3", "j4", "j5", "j6") and 0.0 <= float(v) <= 3.0
                args.joint_scale[k] = float(v)
        except (ValueError, AssertionError):
            problems.append("--gravity-joint-scale must be jN=X pairs with N 1-6 and X in [0, 3], e.g. j3=2.5")
    if args.tuned and args.sweep:
        problems.append("--tuned sets the gains; it cannot be combined with --sweep")
    if args.tuned:
        if args.joint not in TUNED:
            problems.append(f"--tuned has no entry for {args.joint}")
        else:
            kp_t, kd_t, _, fr_t = TUNED[args.joint]
            args.kp, args.kd, args.friction = kp_t, kd_t, fr_t
            args.gains = [(kp_t, kd_t)]
            for n, (_, _, gs, _) in TUNED.items():
                if gs != 1.0 and n not in args.joint_scale:
                    args.joint_scale[n] = gs
    also_raw, args.also = args.also, {}
    if also_raw:
        try:
            for item in also_raw.split(","):
                k, v = item.split("=")
                assert k in TUNED and k != args.joint and k not in args.also
                args.also[k] = float(v)
                assert 0.0 < abs(args.also[k]) <= MAX_EXCURSION_RAD
        except (ValueError, AssertionError):
            problems.append(f"--also must be jN=MOVE pairs (j1-j6, not --joint, each once, 0 < |MOVE| <= "
                            f"{MAX_EXCURSION_RAD})")
            args.also = {}
        if not (args.hold_others and args.tuned):
            problems.append("--also needs --hold-others and --tuned")
        if len(args.gains) > 1:
            problems.append("--also cannot be combined with --sweep")
    if not (0.0 <= args.friction <= FRICTION_MAX_NM):
        problems.append(f"--friction must be in [0, {FRICTION_MAX_NM}] Nm")
    if args.gravity and not args.hold_others:
        problems.append("--gravity needs --hold-others (it uses every joint's measured position each cycle)")
    if not (0.0 <= args.gravity_scale <= 1.5):
        problems.append("--gravity-scale must be in [0, 1.5]")
    if not (0.0 < args.kp <= KP_MAX):
        problems.append(f"--kp must be in (0, {KP_MAX}]")
    if not (0.0 <= args.kd <= KD_MAX):
        problems.append(f"--kd must be in [0, {KD_MAX}]")
    if args.ramp_s < MIN_RAMP_S:
        problems.append(f"--ramp-s must be >= {MIN_RAMP_S}")
    if args.hold_s < MIN_HOLD_S:
        problems.append(f"--hold-s must be >= {MIN_HOLD_S}")
    lo, hi = URDF_LIMITS[args.joint]
    if args.move is None and not (abs(args.target) <= MAX_ABS_TARGET_RAD and lo <= args.target <= hi):
        problems.append(f"--target must be within ±{MAX_ABS_TARGET_RAD} rad and {args.joint}'s URDF range [{lo}, {hi}]")
    max_move = MAX_EXCURSION_BY_JOINT.get(args.joint, MAX_EXCURSION_RAD)
    if args.move is not None and not (0.0 < abs(args.move) <= max_move):
        problems.append(f"--move must be non-zero and within ±{max_move} rad for {args.joint}")
    if not (0.0 < args.estop_wait_s <= ESTOP_WAIT_MAX_S):
        problems.append(f"--estop-wait-s must be in (0, {ESTOP_WAIT_MAX_S}]")
    if args.operator_go is not None and not args.operator_go.strip():
        problems.append("--operator-go must quote the operator's words")
    if problems:
        print("usage error: " + "; ".join(problems), file=sys.stderr)
        return EXIT_USAGE
    return args


def make_runtime(args: argparse.Namespace) -> Runtime | int:
    here = Path(__file__).resolve().parent
    out_dir = Path(args.out_dir).expanduser() if args.out_dir else here / ("runs_sim" if args.simulate else "runs")
    if args.simulate:
        import fakebus  # local module next to this file; never motorbridge
        faults = fakebus.Faults()
        f = args.simulate_fault
        if f == "stuck":
            faults.stuck_from = 1.5
        elif f == "torque":
            faults.torque_spike_at = 2.0
        elif f in ("stale", "estop"):
            faults.freeze_at = 3.0
        elif f == "other_enabled":
            faults.other_enabled_at = 2.0
        elif f == "other_moves":
            faults.other_moves_at = 2.0
        elif f == "bus_error":
            faults.bus_error_at = 2.5
        bus = fakebus.FakeBus(faults=faults)
        if args.gravity:   # the simulated arm feels the URDF gravity at its rest pose (applied while enabled)
            import gravity
            hold = gravity.GravityModel().hold_torques(dict(bus.rest))
            for n, tq in hold.items():
                bus.sims[n].load = -tq
        rt = Runtime(clock=bus.clock, sleep=bus.clock.sleep, tty_reader=lambda p: CONFIRM_PHRASE, out_dir=out_dir,
                     mb=bus.mod, simulated=True, check_host=False, require_confirmation=False,
                     install_signals=False)
        if f == "sigint":
            def _sig(now: float) -> None:
                c = bus.controller
                if c is not None and c.t_enable is not None and now - c.t_enable >= 3.0:
                    rt.stop_flag["why"] = "SIGINT"
            bus.clock.on_advance.append(_sig)
        return rt
    try:
        import motorbridge as mb  # the real SDK, only when not simulating
    except Exception as e:  # noqa: BLE001
        print(f"cannot import motorbridge ({e}): run with ~/miniconda3/envs/rebot/bin/python and env -u PYTHONPATH",
              file=sys.stderr)
        return EXIT_REFUSED
    return Runtime(clock=time.monotonic, sleep=time.sleep, tty_reader=read_dev_tty, out_dir=out_dir, mb=mb)


def main() -> int:
    return run(sys.argv[1:])


if __name__ == "__main__":
    sys.exit(main())

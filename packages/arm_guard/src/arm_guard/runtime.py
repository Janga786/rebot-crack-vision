"""arm_guard.runtime — per-process guard state: which motor is which joint, fresh feedback, the fault latch.

The patch layer (patch.py) calls into one Guard instance for every motor operation. The Guard decides with the
pure functions in core.py and either returns the command to send, or raises. `raw` is the patch layer's handle
on the *unguarded* motorbridge calls (so the guard can read state and send holds without recursing into itself).
"""

from __future__ import annotations

import contextlib
import fcntl
import json
import logging
import math
import os
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path

from . import core
from .core import Command, Feedback
from .errors import GuardConfigError, GuardFault, GuardRefused

log = logging.getLogger("arm_guard")

MODE_NAMES = {1: "mit", 2: "pos_vel", 3: "vel", 4: "force_pos"}
RID_CTRL_MODE, RID_PMAX, RID_VMAX, RID_TMAX = 10, 21, 22, 23
LOCK_DIR = Path(os.environ.get("ARM_GUARD_LOCK_DIR", "/tmp/arm_guard_locks"))
EVENT_LOG = Path(os.environ.get("ARM_GUARD_EVENT_LOG", Path.home() / ".local/state/arm_guard/events.jsonl"))
HEARTBEAT_TIMEOUT_MS = 10
ENABLE_REG_TIMEOUT_MS = 200
ENABLE_REG_ATTEMPTS = 3      # the USB2CAN link occasionally drops one register reply (2026-10-17, 2026-10-22):
                             # one good round trip proves the link; a dead link fails every attempt
STARTUP_POLLS, STARTUP_POLL_S = 100, 0.002     # up to 0.2 s for each motor's feedback frame at the startup check
RESYNC_DEBOUNCE_S = 1.0                         # enable_all checks every motor in turn: one startup check for all


@dataclass
class JointRT:
    cfg: object
    bus: "Bus"
    motor: object = None
    mode: str = ""                      # last known control mode ("" = not yet read)
    fb_key: tuple = ()
    fb_changed_t: float = -math.inf
    hb_try_t: float = -math.inf         # last heartbeat attempt (a dead link must not stall every call)
    last_fb: Feedback = None
    cmd_pos: float = math.nan
    cmd_t: float = -math.inf
    mapping_verified: bool = False      # PMAX/VMAX/TMAX read back and match the model
    offset: float = 0.0                 # whole turns found by the startup check: raw = physical + offset
    allowance: tuple = None             # R2 (lo, hi, lo_floor, hi_floor) for this session; shrinks with targets


@dataclass
class Bus:
    channel: str
    lock_fd: int = -1
    joints: dict = field(default_factory=dict)   # name -> JointRT
    synced_t: float = -math.inf                  # last passed startup check (S1)


class Guard:
    def __init__(self, cfg=None, config_error: str = "", raw=None, clock=time.monotonic, sleep=time.sleep) -> None:
        self.cfg = cfg
        self.config_error = config_error
        self.raw = raw
        self.clock = clock
        self.sleep = sleep
        self.lock = threading.RLock()
        self.buses: dict = {}
        self.latched = ""
        self.latch_t = 0.0
        self.hold_pos: dict = {}
        self.events: deque = deque(maxlen=200)
        self._tls = threading.local()
        self._adj_logged: dict = {}

    # ---- bookkeeping -------------------------------------------------------------------------------------

    def _event(self, kind: str, msg: str, **extra) -> None:
        rec = {"t_wall": time.time(), "kind": kind, "msg": msg, "pid": os.getpid(), **extra}
        self.events.append(rec)
        (log.error if kind in ("fault", "refused") else log.info)("arm_guard %s: %s", kind, msg)
        if kind == "fault":
            print(f"[arm_guard] FAULT (latched, holding): {msg}", file=sys.stderr, flush=True)
        try:
            EVENT_LOG.parent.mkdir(parents=True, exist_ok=True)
            with EVENT_LOG.open("a") as f:
                f.write(json.dumps(rec) + "\n")
        except OSError:
            pass

    def _require_cfg(self) -> None:
        if self.cfg is None:
            raise GuardConfigError(f"arm_guard has no valid config, all motion refused: {self.config_error}")

    def status(self) -> dict:
        with self.lock:
            return {
                "config": None if self.cfg is None else {"path": self.cfg.path, "sha256": self.cfg.sha256},
                "config_error": self.config_error,
                "latched": self.latched,
                "buses": {c: sorted(b.joints) for c, b in self.buses.items()},
                "turn_offsets": {c: {n: o.offset for n, o in b.joints.items() if o.offset}
                                 for c, b in self.buses.items()},
                "recent_events": list(self.events)[-10:],
            }

    def reset(self, reason: str) -> None:
        """Clear the fault latch. A deliberate operator action: the reason is logged."""
        if not reason or not reason.strip():
            raise GuardRefused("reset() needs a reason")
        with self.lock:
            prev, self.latched, self.hold_pos = self.latched, "", {}
            self._event("reset", f"latch cleared ({reason}); was: {prev}")

    @contextlib.contextmanager
    def config_writes(self, reason: str):
        """Allow motor configuration writes (zero, store, TIMEOUT, registers) in this thread, motors disabled only."""
        if not reason or not reason.strip():
            raise GuardRefused("config_writes() needs a reason")
        self._event("config_writes", f"opened: {reason}")
        prev = getattr(self._tls, "config_reason", "")
        self._tls.config_reason = reason
        try:
            yield
        finally:
            self._tls.config_reason = prev
            self._event("config_writes", f"closed: {reason}")

    # ---- registration ------------------------------------------------------------------------------------

    @staticmethod
    def channel_key(channel: str) -> str:
        return os.path.realpath(channel) if channel.startswith("/") else channel

    def open_bus(self, channel: str) -> Bus:
        """Called before a Controller opens `channel`: one process per bus (exclusive flock)."""
        self._require_cfg()
        key = self.channel_key(channel)
        with self.lock:
            if key in self.buses:
                return self.buses[key]
            LOCK_DIR.mkdir(parents=True, exist_ok=True)
            path = LOCK_DIR / (re.sub(r"[^A-Za-z0-9_.-]", "_", key) + ".lock")
            fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                try:
                    holder = os.read(fd, 64).decode(errors="replace").strip()
                finally:
                    os.close(fd)
                raise GuardRefused(f"{key} is already in use by another guarded process ({holder or 'unknown'}); "
                                   f"only one process may drive the arm")
            os.ftruncate(fd, 0)
            os.write(fd, f"pid {os.getpid()} {' '.join(sys.argv)[:200]}".encode())
            bus = Bus(key, fd)
            self.buses[key] = bus
            self._event("bus", f"opened {key}")
            return bus

    def register_motor(self, bus: Bus, motor, motor_id: int, feedback_id: int, model: str) -> None:
        self._require_cfg()
        j = self.cfg.joint_by_motor_id(int(motor_id))
        if j is None:
            raise GuardRefused(f"motor id {motor_id:#x} is not in the guard config")
        if j.feedback_id != int(feedback_id) or j.model != model:
            raise GuardRefused(f"{j.name}: add_damiao_motor({motor_id:#x}, {feedback_id:#x}, {model!r}) does not match "
                               f"config ({j.motor_id:#x}, {j.feedback_id:#x}, {j.model!r})")
        with self.lock:
            jr = bus.joints.get(j.name) or JointRT(j, bus)
            jr.motor = motor
            bus.joints[j.name] = jr
            motor._arm_guard = jr

    def _jr(self, motor) -> JointRT:
        jr = getattr(motor, "_arm_guard", None)
        if jr is None:
            raise GuardRefused("motor was not created through a guarded Controller.add_damiao_motor; refused")
        return jr

    # ---- feedback ----------------------------------------------------------------------------------------

    def _feedback(self, jr: JointRT, now: float) -> Feedback:
        st = self.raw.get_state(jr.motor)
        if st is None:
            return None
        key = (st.pos, st.vel, st.torq, st.status_code, getattr(st, "t_mos", 0.0), getattr(st, "t_rotor", 0.0))
        if key != jr.fb_key:
            jr.fb_key, jr.fb_changed_t = key, now
        elif now - jr.fb_changed_t > self.cfg.stale_feedback_s / 2 and now - jr.hb_try_t > self.cfg.stale_feedback_s / 4:
            # identical frames happen while still (CONTROLLER_TUNING §4): a register read proves the link is alive
            jr.hb_try_t = now
            try:
                self.raw.get_register_u32(jr.motor, RID_CTRL_MODE, HEARTBEAT_TIMEOUT_MS)
                jr.fb_changed_t = now
            except Exception:
                pass
        fb = Feedback(float(st.pos) - jr.offset, float(st.vel), float(st.torq), int(st.status_code),
                      now - jr.fb_changed_t)
        jr.last_fb = fb
        return fb

    def note_link_alive(self, motor) -> None:
        """A caller's register read succeeded: the same proof of a live link as the guard's own heartbeat (so a
        caller that already sends heartbeats does not get a second set from the guard)."""
        jr = getattr(motor, "_arm_guard", None)
        if jr is not None:
            with self.lock:
                jr.fb_changed_t = self.clock()

    # ---- the startup check (S1) --------------------------------------------------------------------------

    def _state_now(self, jr: JointRT):
        try:
            self.raw.request_feedback(jr.motor)
        except Exception:
            pass
        for _ in range(STARTUP_POLLS):
            st = self.raw.get_state(jr.motor)
            if st is not None:
                return st
            self.sleep(STARTUP_POLL_S)
        return None

    def _startup(self, bus: Bus) -> None:
        """Before a session's first enable (every motor on the bus disabled, or never checked): every joint must
        read inside its startup window. Sets each joint's whole-turn offset for the session."""
        if not self.cfg.startup_required:
            return
        states = {name: self._state_now(o) for name, o in bus.joints.items()}
        all_disabled = all(st is not None and int(st.status_code) == core.ST_DISABLED for st in states.values())
        now = self.clock()
        if bus.synced_t > -math.inf and (not all_disabled or now - bus.synced_t < RESYNC_DEBOUNCE_S):
            return                     # a session is under way: keep its offsets
        raw_pos = {name: (None if st is None else float(st.pos)) for name, st in states.items()}
        offsets, problems = core.startup_check(self.cfg, raw_pos)
        if problems:
            msg = ("startup check failed, nothing enabled: " + "; ".join(problems) + ". Put the arm in its fold "
                   "(J1 facing forward) by hand; if it IS folded, an encoder or motor zero is off: stop and check")
            self._event("refused", msg, readings=raw_pos)
            raise GuardRefused(msg)
        for name, o in bus.joints.items():
            o.offset, o.allowance = offsets[name], None
        bus.synced_t = now
        turns = {n: round(off / (2 * math.pi)) for n, off in offsets.items() if off}
        self._event("startup", "startup check passed" + (f"; whole-turn offsets {turns}" if turns else ""),
                    readings=raw_pos)

    def _window(self, jr: JointRT, fb: Feedback) -> tuple:
        if jr.allowance is None:
            jr.allowance = core.initial_allowance(jr.cfg, fb.pos, self.cfg)
        return jr.allowance[:2]

    @staticmethod
    def _to_raw(jr: JointRT, c: Command) -> Command:
        return c if c.mode == "vel" or not jr.offset else replace(c, pos=c.pos + jr.offset)

    # ---- the fault latch ---------------------------------------------------------------------------------

    def _trip(self, jr: JointRT, reasons: list) -> None:
        msg = "; ".join(reasons)
        with self.lock:
            if not self.latched:
                self.latched, self.latch_t = msg, self.clock()
                for bus in self.buses.values():
                    for name, o in bus.joints.items():
                        if o.last_fb is not None and math.isfinite(o.last_fb.pos):
                            self.hold_pos[name] = o.last_fb.pos
                self._event("fault", msg, joint=jr.cfg.name, hold=dict(self.hold_pos))

    def _hold_for(self, jr: JointRT, cmd: Command) -> Command:
        q = self.hold_pos.get(jr.cfg.name)
        if q is None:
            q = jr.last_fb.pos if jr.last_fb is not None else None
        if q is None or not math.isfinite(q):
            return None
        ratio = min(max(cmd.ratio, 0.0), self.cfg.force_pos_ratio_max) if math.isfinite(cmd.ratio) else 0.0
        return self._to_raw(jr, core.hold_command(jr.cfg, cmd.mode, q, cmd.tau, ratio, self.cfg.hold_vlim))

    # ---- operations --------------------------------------------------------------------------------------

    def filter(self, motor, cmd: Command):
        """Return (command_to_send, fault_message). fault_message != "" means the caller must send the (hold)
        command and then raise GuardFault. Raises GuardRefused/GuardConfigError when nothing may be sent.
        Commands arrive and leave in raw motor coordinates; every check runs on physical ones (raw - offset)."""
        self._require_cfg()
        jr = self._jr(motor)
        with self.lock:
            now = self.clock()
            if jr.mode and cmd.mode != jr.mode:
                raise GuardRefused(f"{jr.cfg.name}: {cmd.mode} command while the motor is in {jr.mode} mode")
            if self.latched:
                hold = self._hold_for(jr, cmd)
                if hold is None:
                    raise GuardRefused(f"{jr.cfg.name}: latched and no position to hold")
                return hold, ""
            fb = self._feedback(jr, now)
            reasons = core.check_state(jr.cfg, fb, self.cfg)
            if not reasons:
                if jr.cfg.arm and self.cfg.require_full_arm:
                    missing = [j.name for j in self.cfg.arm_joints if j.name not in jr.bus.joints]
                    if missing:
                        raise GuardRefused(f"{jr.cfg.name}: arm joints {missing} not registered on {jr.bus.channel}; "
                                           f"the keep-out check needs the whole arm")
                phys = cmd if cmd.mode == "vel" else replace(cmd, pos=cmd.pos - jr.offset)
                v = core.check_command(jr.cfg, phys, fb, self.cfg, window=self._window(jr, fb))
                reasons = list(v.reasons)
                if v.ok and jr.cfg.arm:
                    reasons = self._pose_reasons(jr, phys.pos, now)
                if not reasons:
                    for a in v.adjustments:
                        self._note_adjustment(jr.cfg.name, a)
                    jr.cmd_pos, jr.cmd_t = v.cmd.pos, now
                    if cmd.mode != "vel":
                        jr.allowance = core.shrink_allowance(jr.allowance, v.cmd.pos, self.cfg)
                    return self._to_raw(jr, v.cmd), ""
            self._trip(jr, reasons)
            hold = self._hold_for(jr, cmd)
            if hold is None:
                raise GuardFault(f"{self.latched} (no position to hold; nothing sent)")
            return hold, self.latched

    def _pose_reasons(self, jr: JointRT, target: float, now: float) -> list:
        q_now, q_new = {}, {}
        for j in self.cfg.arm_joints:
            o = jr.bus.joints[j.name]
            fb = jr.last_fb if o is jr else self._feedback(o, now)
            if fb is None or not math.isfinite(fb.pos):
                return [f"keep-out: no feedback for {j.name}"]
            q_now[j.urdf_joint] = fb.pos
            recent = now - o.cmd_t <= self.cfg.recent_command_s and math.isfinite(o.cmd_pos)
            q_new[j.urdf_joint] = o.cmd_pos if recent else fb.pos
        q_new[jr.cfg.urdf_joint] = target
        return core.check_pose(self.cfg, q_new, q_now)

    def _note_adjustment(self, joint: str, msg: str) -> None:
        now = self.clock()
        if now - self._adj_logged.get(joint, -math.inf) > 5.0:
            self._adj_logged[joint] = now
            self._event("adjusted", msg)

    def _read_register(self, read, motor, rid: int):
        """A register read at enable time, retried: a single missed reply is not a dead link."""
        for attempt in range(1, ENABLE_REG_ATTEMPTS + 1):
            try:
                return read(motor, rid, ENABLE_REG_TIMEOUT_MS)
            except Exception as exc:  # noqa: BLE001 — motorbridge raises CallError on a timeout
                if attempt == ENABLE_REG_ATTEMPTS:
                    raise
                self._event("retry", f"{getattr(motor, '_arm_guard').cfg.name}: register {rid} read failed "
                                     f"({exc}); retry {attempt}/{ENABLE_REG_ATTEMPTS - 1}")

    def check_enable(self, motor) -> None:
        """Before enable(): known joint, no latch, live link, MIT mapping verified, sane mode, state within limits."""
        self._require_cfg()
        jr = self._jr(motor)
        with self.lock:
            if self.latched:
                raise GuardRefused(f"{jr.cfg.name}: enable refused, guard latched: {self.latched}")
            self._startup(jr.bus)
            try:
                mode = int(self._read_register(self.raw.get_register_u32, motor, RID_CTRL_MODE))
                if not jr.mapping_verified:
                    got = tuple(float(self._read_register(self.raw.get_register_f32, motor, rid))
                                for rid in (RID_PMAX, RID_VMAX, RID_TMAX))
                    want = self.cfg.models[jr.cfg.model]
                    if any(abs(a - b) > 1e-3 * max(1.0, abs(b)) for a, b in zip(got, want)):
                        raise GuardRefused(f"{jr.cfg.name}: PMAX/VMAX/TMAX {got} != {jr.cfg.model} {want}; "
                                           f"MIT commands would be mis-scaled")
                    jr.mapping_verified = True
            except GuardRefused:
                raise
            except Exception as e:
                raise GuardRefused(f"{jr.cfg.name}: enable refused, register read failed ({e})") from e
            name = MODE_NAMES.get(mode)
            if name is None or (name == "vel" and not self.cfg.allow_velocity_mode):
                raise GuardRefused(f"{jr.cfg.name}: enable refused in control mode {mode}")
            jr.mode = name
            try:
                self.raw.request_feedback(motor)
            except Exception:
                pass
            now = self.clock()
            fb = self._feedback(jr, now)
            jr.fb_changed_t = now   # the register reads above just proved the link is alive
            fb = fb and Feedback(fb.pos, fb.vel, fb.torq, fb.status, 0.0)
            reasons = core.check_state(jr.cfg, fb, self.cfg)
            if reasons:
                raise GuardRefused("enable refused: " + "; ".join(reasons))

    def check_mode(self, motor, mode) -> None:
        self._require_cfg()
        jr = self._jr(motor)
        name = MODE_NAMES.get(int(mode))
        if name is None or (name == "vel" and not self.cfg.allow_velocity_mode):
            raise GuardRefused(f"{jr.cfg.name}: control mode {int(mode)} refused")
        with self.lock:
            jr.mode = name

    def check_config_write(self, motor, what: str, rid=None, value=None) -> None:
        """Register/zero/store/timeout writes: refused unless whitelisted (vendor loop gains) or inside
        config_writes() with the motor disabled."""
        self._require_cfg()
        jr = self._jr(motor)
        if rid is not None and int(rid) in self.cfg.register_writes:
            lo, hi = self.cfg.register_writes[int(rid)]
            if isinstance(value, (int, float)) and math.isfinite(value) and lo <= value <= hi:
                return
        reason = getattr(self._tls, "config_reason", "")
        if not reason:
            raise GuardRefused(f"{jr.cfg.name}: {what} refused; motor configuration writes need "
                               f"`with arm_guard.config_writes('<reason>'):` (motors disabled)")
        st = self.raw.get_state(motor)
        if st is None or int(st.status_code) != core.ST_DISABLED:
            raise GuardRefused(f"{jr.cfg.name}: {what} refused; the motor must be DISABLED for configuration writes")
        self._event("config_write", f"{jr.cfg.name}: {what} ({reason})", rid=rid, value=value)

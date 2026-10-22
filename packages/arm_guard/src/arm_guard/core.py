"""arm_guard.core — the pure decision functions: (command, measured state, config) -> verdict.

Nothing here touches a bus, a clock or global state, so every rule can be tested exhaustively and fuzzed.
The runtime (runtime.py) feeds these functions and acts on their verdicts.

Rules (each is an invariant the tests assert for arbitrary input):
  R1  every number in a command is finite.
  R2  a position target lies inside [lower, upper]; if the joint already rests slightly outside (by at most its
      rest_tolerance, e.g. J2 at +0.02 in the fold, or drooped to +0.4 below it after a power cut) the target may
      hold there (+ rest_slack for sensor noise) or move inward, never further out. Within a session the runtime
      keeps that allowance and shrinks it with the accepted targets (never with the measured position, which may
      lead the reference into the range): no ratcheting outward, no return into a deep droop once lifted, but a
      joint may always go back to its fold rest reading (<= the global rest_tolerance past the limit).
  R3  |target - measured position| <= max_step (bounds the MIT spring torque kp * error and catches IK branch
      flips / wrist flips that teleport a joint).
  R4  speed (MIT vel, VEL-mode vel, POS_VEL/FORCE_POS vlim) <= min(vel_cap, max(min_vlim, sqrt(2 * accel_stop *
      distance to the limit ahead))): the joint can always stop before its limit (libfranka-style stopping-distance
      bound; min_vlim 0.05 rad/s stops within 1 mm).
  R5  MIT kp <= kp_max, |tau_ff| <= tau_ff_max; kd above kd_max is lowered to kd_max.
  R6  the pose with this target (other joints at their recent command or measured position) keeps the keep-out
      margins (floor/table plane, base cylinder, distance from the upper arm), or at least does not make an
      existing violation worse.
  R7  measured state: status ENABLED/DISABLED (no fault code), finite, position within limits + rest_tolerance,
      |velocity| <= vel_abort, |torque| <= torque_abort, feedback fresher than stale_feedback_s.
  S1  startup (before a session's first enable): every joint reads inside its startup window, i.e. the arm is in
      its fold and the encoders agree; a turn-ambiguous joint (J1) is unwrapped by whole turns into its window and
      every later check uses that offset (raw = physical + offset).

Only two adjustments are ever made, both toward "slower/softer": lowering vlim (R4) and kd (R5). A position
target is never moved: a target that breaks a rule is refused, and the runtime then holds the arm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

ST_DISABLED, ST_ENABLED = 0x0, 0x1
EPS_RAD = 1e-9            # float rounding at an exact limit (a min-jerk ending on 0.0 can give +1e-16)
STATUS_NAMES = {0x0: "DISABLED", 0x1: "ENABLED", 0x8: "OVER_VOLTAGE", 0x9: "UNDER_VOLTAGE", 0xA: "OVER_CURRENT",
                0xB: "MOS_OVER_TEMP", 0xC: "ROTOR_OVER_TEMP", 0xD: "LOST_COMM", 0xE: "OVERLOAD"}


@dataclass(frozen=True)
class Command:
    mode: str                 # "mit" | "pos_vel" | "vel" | "force_pos"
    pos: float = 0.0
    vel: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    tau: float = 0.0
    vlim: float = 0.0
    ratio: float = 0.0


@dataclass(frozen=True)
class Feedback:
    pos: float
    vel: float
    torq: float
    status: int
    age_s: float              # time since the feedback last changed (or a heartbeat read succeeded)


@dataclass(frozen=True)
class Verdict:
    ok: bool
    cmd: Command
    reasons: tuple = ()
    adjustments: tuple = ()


def _finite(*xs) -> bool:
    return all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in xs)


def target_window(j, q_meas: float, rest_tol: float, slack: float = 0.0) -> tuple:
    """R2: allowed target interval for a joint measured at q_meas. `slack` absorbs sensor noise when holding a
    rest pose that sits past a limit (the hold target is the rest reading, the measurement wanders around it)."""
    lo = j.lower if q_meas >= j.lower else max(q_meas - slack, j.lower - rest_tol)
    hi = j.upper if q_meas <= j.upper else min(q_meas + slack, j.upper + rest_tol)
    return lo, hi


def initial_allowance(j, q_meas: float, cfg) -> tuple:
    """R2 allowance at a joint's first command of a session: (lo, hi, lo_floor, hi_floor). The floors are how far
    it never shrinks: the fold rest band (global rest_tolerance) if the joint started out there, else the limit."""
    lo, hi = target_window(j, q_meas, j.rest_tolerance_rad, cfg.rest_slack_rad)
    return lo, hi, max(lo, j.lower - cfg.rest_tolerance_rad), min(hi, j.upper + cfg.rest_tolerance_rad)


def shrink_allowance(allow: tuple, target: float, cfg) -> tuple:
    """After an accepted target: the allowance past a limit follows the target inward and never grows back."""
    lo, hi, lof, hif = allow
    return (min(lof, max(lo, target - cfg.rest_slack_rad)), max(hif, min(hi, target + cfg.rest_slack_rad)), lof, hif)


def stop_speed(j, q: float, direction: float, lo: float, hi: float, floor: float = 0.0) -> float:
    """R4: highest speed from q toward `direction` that can still stop (at accel_stop) before the window edge.
    `floor` (e.g. 0.05 rad/s, stopping distance < 1 mm) keeps rounding/noise near the edge from tripping it."""
    if direction > 0:
        d = hi - q
    elif direction < 0:
        d = q - lo
    else:
        return j.vel_cap
    return min(j.vel_cap, max(floor, math.sqrt(2.0 * j.accel_stop * max(d, 0.0))))


def check_state(j, fb: Feedback, cfg) -> list:
    """R7. Returns fault reasons (empty = healthy)."""
    r = []
    if fb is None:
        return [f"{j.name}: no feedback"]
    if not _finite(fb.pos, fb.vel, fb.torq, fb.age_s):
        return [f"{j.name}: non-finite feedback"]
    if fb.status not in (ST_DISABLED, ST_ENABLED):
        r.append(f"{j.name}: motor status {STATUS_NAMES.get(fb.status, hex(fb.status))}")
    tol = j.rest_tolerance_rad
    if not (j.lower - tol <= fb.pos <= j.upper + tol):
        r.append(f"{j.name}: measured position {fb.pos:+.4f} outside [{j.lower}, {j.upper}] (+/-{tol})")
    if abs(fb.vel) > j.vel_abort:
        r.append(f"{j.name}: measured speed {fb.vel:+.3f} > {j.vel_abort} rad/s")
    if abs(fb.torq) > j.torque_abort:
        r.append(f"{j.name}: measured torque {fb.torq:+.2f} > {j.torque_abort} Nm")
    if fb.age_s > cfg.stale_feedback_s:
        r.append(f"{j.name}: feedback stale for {fb.age_s:.3f} s > {cfg.stale_feedback_s}")
    return r


def check_command(j, cmd: Command, fb: Feedback, cfg, window: tuple = None) -> Verdict:
    """R1-R5 for one joint. R6 (whole-pose keep-out) is checked separately by check_pose. `window` = the runtime's
    session allowance (lo, hi); without it the window is computed from the measured position alone. Either way it
    never reaches beyond the joint's rest_tolerance past its limits."""
    reasons, adj = [], []
    if fb is None or not _finite(fb.pos):
        return Verdict(False, cmd, (f"{j.name}: no usable measured position",))
    if cmd.mode not in ("mit", "pos_vel", "vel", "force_pos"):
        return Verdict(False, cmd, (f"{j.name}: unknown mode {cmd.mode!r}",))
    if not _finite(cmd.pos, cmd.vel, cmd.kp, cmd.kd, cmd.tau, cmd.vlim, cmd.ratio):
        return Verdict(False, cmd, (f"{j.name}: non-finite command {cmd}",))
    if window is None:
        lo, hi = target_window(j, fb.pos, j.rest_tolerance_rad, cfg.rest_slack_rad)
    else:
        lo, hi = max(window[0], j.lower - j.rest_tolerance_rad), min(window[1], j.upper + j.rest_tolerance_rad)
    out = cmd

    if cmd.mode == "vel":
        if not cfg.allow_velocity_mode:
            return Verdict(False, cmd, (f"{j.name}: velocity mode is disabled (keeps moving if commands stop)",))
        lim = stop_speed(j, fb.pos, cmd.vel, lo, hi, cfg.min_vlim)
        if abs(cmd.vel) > lim + 1e-12:
            reasons.append(f"{j.name}: vel {cmd.vel:+.3f} > {lim:.3f} rad/s (cap/stopping distance)")
        return Verdict(not reasons, out, tuple(reasons))

    if not (lo - EPS_RAD <= cmd.pos <= hi + EPS_RAD):
        reasons.append(f"{j.name}: target {cmd.pos:+.4f} outside [{lo:+.4f}, {hi:+.4f}]")
    if abs(cmd.pos - fb.pos) > j.max_step + 1e-12:
        reasons.append(f"{j.name}: target {cmd.pos:+.4f} is {abs(cmd.pos - fb.pos):.3f} rad from measured "
                       f"{fb.pos:+.4f} (> max_step {j.max_step})")

    if cmd.mode == "mit":
        if not 0.0 <= cmd.kp <= j.kp_max:
            reasons.append(f"{j.name}: kp {cmd.kp} outside [0, {j.kp_max}]")
        if cmd.kd < 0.0:
            reasons.append(f"{j.name}: kd {cmd.kd} < 0")
        elif cmd.kd > j.kd_max:
            adj.append(f"{j.name}: kd {cmd.kd} -> {j.kd_max}")
            out = replace(out, kd=j.kd_max)
        if abs(cmd.tau) > j.tau_ff_max:
            reasons.append(f"{j.name}: |tau_ff| {abs(cmd.tau):.2f} > {j.tau_ff_max} Nm")
        lim = stop_speed(j, min(max(cmd.pos, lo), hi), cmd.vel, lo, hi, cfg.min_vlim)
        if abs(cmd.vel) > lim + 1e-12:
            reasons.append(f"{j.name}: vel {cmd.vel:+.3f} > {lim:.3f} rad/s (cap/stopping distance)")
    else:  # pos_vel / force_pos: the motor runs toward pos at up to vlim
        if not cmd.vlim > 0.0:
            reasons.append(f"{j.name}: vlim {cmd.vlim} must be > 0")
        else:
            lim = stop_speed(j, fb.pos, cmd.pos - fb.pos, lo, hi, cfg.min_vlim)
            if cmd.vlim > lim:
                adj.append(f"{j.name}: vlim {cmd.vlim:.3f} -> {lim:.3f}")
                out = replace(out, vlim=lim)
        if cmd.mode == "force_pos" and not 0.0 <= cmd.ratio <= cfg.force_pos_ratio_max:
            reasons.append(f"{j.name}: force ratio {cmd.ratio} outside [0, {cfg.force_pos_ratio_max}]")
    if reasons:
        return Verdict(False, cmd, tuple(reasons), tuple(adj))
    return Verdict(True, out, (), tuple(adj))


def startup_check(cfg, raw_pos: dict) -> tuple:
    """S1. raw_pos: joint name -> position as the motor reports it (None = no reading), for every joint on the bus.
    Returns (offsets, problems): offsets[name] is the whole-turn offset (raw = physical + offset; 0 unless the joint
    is turn-ambiguous), problems lists every joint without a reading or outside its startup window."""
    offsets, problems = {}, []
    for name, r in raw_pos.items():
        j = cfg.joints[name]
        if not j.startup_window:
            offsets[name] = 0.0
            continue
        lo, hi = j.startup_window
        if r is None or not _finite(r):
            problems.append(f"{name}: no position reading")
            continue
        off = 0.0
        if j.turn_ambiguous:
            off = 2.0 * math.pi * round((r - (lo + hi) / 2.0) / (2.0 * math.pi))
        q = r - off
        turns = f" ({q:+.3f} plus {round(off / (2 * math.pi))} whole turns)" if off else ""
        if not lo <= q <= hi:
            problems.append(f"{name}: reads {r:+.3f}{turns}, outside its startup window [{lo:+.2f}, {hi:+.2f}]")
            continue
        pmax = cfg.models[j.model][0]
        if max(abs(j.lower - j.rest_tolerance_rad + off), abs(j.upper + j.rest_tolerance_rad + off)) >= pmax:
            problems.append(f"{name}: reads {r:+.3f}{turns}: its range would leave the {j.model} MIT position "
                            f"mapping (+/-{pmax}); power-cycle the arm")
            continue
        offsets[name] = off
    return offsets, problems


def _dist_point_segment(p, a, b) -> float:
    ab = [b[i] - a[i] for i in range(3)]
    ap = [p[i] - a[i] for i in range(3)]
    n2 = sum(c * c for c in ab)
    t = 0.0 if n2 == 0.0 else max(0.0, min(1.0, sum(ap[i] * ab[i] for i in range(3)) / n2))
    return math.sqrt(sum((ap[i] - t * ab[i]) ** 2 for i in range(3)))


def pose_margins(cfg, q_urdf: dict) -> dict:
    """R6 margins (metres, >= 0 is clear) of every keep-out constraint for URDF joint values q_urdf."""
    k = cfg.keepout
    pos = cfg.chain.link_positions(q_urdf)
    m = {}
    for f in k.floor:
        m[f"floor:{f.link}"] = pos[f.link][2] - f.z_min
    if k.cyl_radius > 0:
        for link in k.cyl_links:
            x, y, z = pos[link]
            # outside the cylinder if above it or radially clear of it
            m[f"base_cylinder:{link}"] = max(z - k.cyl_z_max, math.hypot(x, y) - k.cyl_radius)
    if k.seg_min_distance > 0:
        a, b = pos[k.seg_from], pos[k.seg_to]
        for link in k.seg_links:
            m[f"segment:{link}"] = _dist_point_segment(pos[link], a, b) - k.seg_min_distance
    return m


def check_pose(cfg, q_new: dict, q_now: dict) -> list:
    """R6: refuse a pose that violates a keep-out constraint, unless it is no worse than where the arm is now."""
    new = pose_margins(cfg, q_new)
    if min(new.values(), default=0.0) >= 0.0:
        return []
    now = pose_margins(cfg, q_now)
    return [f"keep-out {name}: margin {v * 1000:+.1f} mm" for name, v in new.items()
            if v < 0.0 and v < now[name] - 1e-9]


def hold_command(j, mode: str, q_hold: float, tau: float = 0.0, ratio: float = 0.0, hold_vlim: float = 0.2) -> Command:
    """The command that holds joint j at q_hold in the motor's current mode (used after a fault)."""
    tau = max(-j.tau_ff_max, min(j.tau_ff_max, tau if _finite(tau) else 0.0))
    if mode == "mit":
        return Command("mit", pos=q_hold, vel=0.0, kp=j.hold_kp, kd=j.hold_kd, tau=tau)
    if mode == "force_pos":
        return Command("force_pos", pos=q_hold, vlim=hold_vlim, ratio=ratio if _finite(ratio) else 0.0)
    if mode == "vel":
        return Command("vel", vel=0.0)
    return Command("pos_vel", pos=q_hold, vlim=hold_vlim)

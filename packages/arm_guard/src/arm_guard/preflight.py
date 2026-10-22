"""arm_guard.preflight — before anything is enabled: would the guard accept every command of a planned run?

A caller (e.g. ~/rebot_setup/demo_move.py) samples the exact MIT command stream it is about to send, at its loop
rate, and passes it here with the joints' current readings. This mirrors what the runtime will do on the bus: the
startup check (S1) and whole-turn offsets, the session allowance past a limit (R2), R1-R5 per joint, R6 on each
commanded pose. The measured position is taken to be the commanded one: the caller's own tracking abort (0.10 rad
in demo_move) keeps the real difference below max_step (0.15), so R3 cannot be what trips on the arm.

What it cannot check is the measured state (R7: speed/torque spikes, stale feedback). It reports how close the
planned speeds come to the guard's measured-speed abort, so a run that would sit within sensor noise of it is
flagged before it starts.
"""

from __future__ import annotations

import math

from . import core

SPEED_NOISE_RAD_S = 0.4     # DM4310 velocity readings spike by up to 26 LSB = 0.38 rad/s (CONTROLLER_TUNING)
MAX_PROBLEMS = 12


def check_plan(cfg, start_raw: dict, ticks) -> dict:
    """start_raw: joint -> position as read now (raw motor coordinates). ticks: iterable of (t_s, {joint: (pos_raw,
    vel, kp, kd, tau)}) in sending order. Returns {"problems": [...], "warnings": [...], "offsets": {...},
    "min_keepout_m": float, "peak_speed": {joint: rad/s}, "n_ticks": int}."""
    out = {"problems": [], "warnings": [], "offsets": {}, "min_keepout_m": math.inf, "peak_speed": {}, "n_ticks": 0}
    problems = out["problems"]
    offsets, s1 = core.startup_check(cfg, dict(start_raw))
    problems += [f"startup: {p}" for p in s1]
    if s1:
        return out
    out["offsets"] = offsets
    allowance = {}
    q_prev = None
    seen = set()

    def report(key, msg):
        if key not in seen and len(problems) < MAX_PROBLEMS:
            seen.add(key)
            problems.append(msg)

    for t, cmds in ticks:
        out["n_ticks"] += 1
        q_new = {}
        for name, (pos, vel, kp, kd, tau) in cmds.items():
            j = cfg.joints[name]
            q = pos - offsets.get(name, 0.0)
            fb = core.Feedback(q, vel, 0.0, core.ST_ENABLED, 0.0)
            if name not in allowance:
                allowance[name] = core.initial_allowance(j, q, cfg)
            v = core.check_command(j, core.Command("mit", pos=q, vel=vel, kp=kp, kd=kd, tau=tau), fb, cfg,
                                   window=allowance[name][:2])
            for r in v.reasons:
                report((name, r.split(" ")[1]), f"t={t:.2f} s: {r}")
            if v.ok:
                allowance[name] = core.shrink_allowance(allowance[name], q, cfg)
            out["peak_speed"][name] = max(out["peak_speed"].get(name, 0.0), abs(vel))
            if j.arm:
                q_new[j.urdf_joint] = q
        if len(q_new) == len(cfg.arm_joints):
            margins = core.pose_margins(cfg, q_new)
            out["min_keepout_m"] = min(out["min_keepout_m"], min(margins.values()))
            if q_prev is not None:
                for r in core.check_pose(cfg, q_new, q_prev):
                    report(("keepout", r.split(":")[0]), f"t={t:.2f} s: {r}")
            q_prev = q_new
    for name, v in out["peak_speed"].items():
        j = cfg.joints[name]
        if v > j.vel_abort - SPEED_NOISE_RAD_S:
            out["warnings"].append(f"{name}: planned {v:.2f} rad/s is within {SPEED_NOISE_RAD_S} of the guard's "
                                   f"measured-speed abort {j.vel_abort}: a sensor spike could trip it")
    return out

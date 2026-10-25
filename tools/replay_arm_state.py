"""tools/replay_arm_state.py — publish a trajectory's arm state in REAL time, as demo_move.py --state-file would.

Offline stand-in for the powered arm while testing crackvision.crack_trace capture / search-capture: it follows a
crackvision.joint_trajectory/1 file (fold -> first point -> the file -> fold, linear in time between points) and
writes the same rebot.arm_state/1 JSON, about 20 times a second. It never touches a bus or a motor.

  ./env.sh python tools/replay_arm_state.py TRAJECTORY.json STATE.json [--lead-s 2] [--slow 1]
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np

NAMES = [f"joint{i}" for i in range(1, 7)]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("trajectory", type=Path)
    ap.add_argument("state", type=Path)
    ap.add_argument("--lead-s", type=float, default=2.0, help="seconds at the fold before the approach")
    ap.add_argument("--approach-s", type=float, default=4.0)
    ap.add_argument("--slow", type=float, default=1.0)
    args = ap.parse_args(argv)
    pts = json.loads(args.trajectory.read_text())["points"]
    t = np.array([p["t_s"] for p in pts]) * args.slow
    q = np.array([p["positions"] for p in pts])
    fold = np.zeros(6)
    legs = [("approach", args.approach_s, fold, q[0]), ("trajectory", t[-1], None, None),
            ("return", args.approach_s, q[-1], fold)]
    t0 = time.monotonic() + args.lead_s
    total = args.lead_s + args.approach_s + t[-1] + args.approach_s + 1.0
    prev = None
    while (now := time.monotonic()) - t0 < total - args.lead_s:
        tau = now - t0
        if tau < 0:
            kind, qq, v = "wait_go", fold, np.zeros(6)
        elif tau < args.approach_s:
            u = tau / args.approach_s
            s = 10 * u**3 - 15 * u**4 + 6 * u**5
            kind, qq = "approach", fold + (q[0] - fold) * s
            v = (q[0] - fold) * (30 * u**2 - 60 * u**3 + 30 * u**4) / args.approach_s
        elif tau < args.approach_s + t[-1]:
            tt = tau - args.approach_s
            qq = np.array([np.interp(tt, t, q[:, j]) for j in range(6)])
            kind, v = "trajectory", (np.array([np.interp(tt + 0.02, t, q[:, j]) for j in range(6)]) - qq) / 0.02
        elif tau < args.approach_s * 2 + t[-1]:
            u = (tau - args.approach_s - t[-1]) / args.approach_s
            s = 10 * u**3 - 15 * u**4 + 6 * u**5
            kind, qq = "return", q[-1] + (fold - q[-1]) * s
            v = (fold - q[-1]) * (30 * u**2 - 60 * u**3 + 30 * u**4) / args.approach_s
        else:
            kind, qq, v = "final_hold", fold, np.zeros(6)
        doc = {"schema": "rebot.arm_state/1", "script_version": "replay_arm_state", "wall_time_ns": time.time_ns(),
               "simulated": True, "t_s": round(max(tau, 0), 3), "mode": "path" if kind in ("approach", "trajectory", "return") else kind,
               "seg": 0, "segment_kind": kind if kind in ("approach", "trajectory", "return") else None,
               "joint_names": NAMES, "positions_rad": [round(float(x), 6) for x in qq],
               "ref_rad": [round(float(x), 6) for x in qq], "velocity_rad_s": [round(float(x), 5) for x in v],
               "gripper_rad": 0.0}
        tmp = args.state.with_name(args.state.name + ".tmp")
        tmp.write_text(json.dumps(doc))
        os.replace(tmp, args.state)
        time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

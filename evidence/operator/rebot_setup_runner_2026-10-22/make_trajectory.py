#!/usr/bin/env python3
"""make_trajectory.py: write the hand-made trajectories in trajectories/ as crackvision.joint_trajectory/1 files.

Each is a list of joint-space waypoints (URDF joint1..joint6, rad) joined by minimum-jerk moves and sampled every
0.1 s, the way a planner's output looks (dense points, at rest at both ends). demo_move.py --trajectory plays it
back after zeroing: approach from the fold, the trajectory, return to the fold.

  env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python make_trajectory.py      # rewrites trajectories/*.json
"""
from __future__ import annotations

import datetime as dt
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
DT = 0.1
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
RAISED = [0.0, -0.6, -1.0, 0.0, 0.0, 0.0]          # the show demo's "rise" pose, a little less forearm

TRAJECTORIES = {
    # home -> raised, look left / right with base + wrist, nod, back to raised (the runner then folds the arm)
    "first_guarded_run": [
        (0.0, HOME),
        (4.0, RAISED),
        (7.0, [0.4, -0.6, -1.0, 0.3, 0.3, 0.0]),
        (11.0, [-0.4, -0.7, -1.1, 0.3, -0.3, 0.0]),
        (14.0, RAISED),
        (16.0, [0.0, -0.6, -1.0, 0.4, 0.0, 0.6]),
        (18.5, RAISED),
    ],
}


def min_jerk(a: list[float], b: list[float], s: float) -> list[float]:
    h = 10 * s**3 - 15 * s**4 + 6 * s**5
    return [x + (y - x) * h for x, y in zip(a, b)]


def build(name: str, waypoints: list) -> dict:
    points = []
    for (t0, q0), (t1, q1) in zip(waypoints, waypoints[1:]):
        n = round((t1 - t0) / DT)
        start = 0 if not points else 1
        for k in range(start, n + 1):
            points.append({"t_s": round(t0 + k * DT, 6), "positions": [round(x, 6) for x in min_jerk(q0, q1, k / n)]})
    return {
        "schema": "crackvision.joint_trajectory/1",
        "created_utc": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "producer": f"rebot_setup/make_trajectory.py:{name}",
        "purpose": "test",
        "planning_frame": "base_link",
        "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        "notes": "hand-made waypoints, min-jerk between them, sampled every 0.1 s; for the first guarded runs",
        "waypoints": [{"t_s": t, "positions": q} for t, q in waypoints],
        "points": points,
        "limits_file": {"path": "config/robot/b601_dm_limits.yaml", "sha256": None},
        "end_effector_config_sha256": None,
        "scene_config_sha256": None,
        "source": {"paths3d": None},
    }


def main() -> None:
    out = HERE / "trajectories"
    out.mkdir(exist_ok=True)
    for name, wps in TRAJECTORIES.items():
        path = out / f"{name}.json"
        path.write_text(json.dumps(build(name, wps), indent=1) + "\n")
        print(f"wrote {path}")


if __name__ == "__main__":
    main()

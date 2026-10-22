"""tools/make_box_trajectory.py — a joint trajectory that draws a box (or circle) with the gripper tip.

The shape lies in a vertical plane in front of the arm (x forward, y left, z up, base_link frame). The tip path
is solved with damped-least-squares inverse kinematics on the repo URDF (J1-J4 move, J5/J6 stay 0), warm-started
along the path so the joint solution never jumps branch. Each edge is a minimum-jerk move, so the tip stops
cleanly at every corner. The result is a crackvision.joint_trajectory/1 file (0.1 s points), played back by
~/rebot_setup/demo_move.py --trajectory (approach from the fold, the shape, return to the fold).

  ./env.sh python tools/make_box_trajectory.py --shape box --out OUT.json [--x 0.30 --zc 0.50 --side 0.15 --edge-s 3]
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "packages" / "arm_guard" / "src"))
from arm_guard import config as agc  # noqa: E402

NAMES = [f"joint{i}" for i in range(1, 7)]
FREE = [0, 1, 2, 3]                       # J1-J4 move; J5, J6 stay 0
SEED = np.array([0.0, -0.6, -1.0, 0.0, 0.0, 0.0])
DT = 0.1


def min_jerk(s: float) -> float:
    return 10 * s**3 - 15 * s**4 + 6 * s**5


class Solver:
    def __init__(self) -> None:
        self.cfg = agc.load(check_approval=False)
        self.chain = self.cfg.chain
        self.lo = np.array([j.lower for j in self.cfg.arm_joints])
        self.hi = np.array([j.upper for j in self.cfg.arm_joints])

    def tip(self, q: np.ndarray) -> np.ndarray:
        return np.array(self.chain.link_positions(dict(zip(NAMES, q)))["gripper_link"])

    def solve(self, target: np.ndarray, q0: np.ndarray, iters: int = 80) -> np.ndarray:
        q = q0.copy()
        for _ in range(iters):
            err = target - self.tip(q)
            if np.linalg.norm(err) < 5e-5:
                break
            J = np.zeros((3, len(FREE)))
            for k, i in enumerate(FREE):
                dq = q.copy()
                dq[i] += 1e-5
                J[:, k] = (self.tip(dq) - self.tip(q)) / 1e-5
            lam = 1e-3
            step = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
            # stay near the seed in the null space (J4 especially), so the wrist does not wander
            null = (np.eye(len(FREE)) - np.linalg.pinv(J) @ J) @ (0.2 * (SEED[FREE] - q[FREE]))
            q[FREE] += np.clip(step + null, -0.1, 0.1)
            q = np.clip(q, self.lo + 0.05, self.hi - 0.05)
        return q


def corners(shape: str, x: float, y0: float, zc: float, side: float, n_circle: int = 4) -> np.ndarray:
    h = side / 2
    if shape == "box":
        return np.array([[x, y0 - h, zc - h], [x, y0 + h, zc - h], [x, y0 + h, zc + h], [x, y0 - h, zc + h],
                         [x, y0 - h, zc - h]])
    raise ValueError(shape)


def path_points(shape: str, x: float, y0: float, zc: float, side: float, edge_s: float):
    """Yield (t, tip target) at DT spacing: each edge is a minimum-jerk move between corners; a circle is one
    smooth loop of the same total time."""
    if shape == "box":
        c = corners("box", x, y0, zc, side)
        t = 0.0
        yield t, c[0]
        for a, b in zip(c[:-1], c[1:]):
            n = round(edge_s / DT)
            for k in range(1, n + 1):
                yield t + k * DT, a + (b - a) * min_jerk(k / n)
            t += n * DT
    elif shape == "circle":
        T = 4 * edge_s
        n = round(T / DT)
        for k in range(n + 1):
            th = 2 * math.pi * min_jerk(k / n) - math.pi / 2
            yield k * DT, np.array([x, y0 + (side / 2) * math.cos(th), zc + (side / 2) * math.sin(th) + side / 2])
    else:
        raise ValueError(shape)


def build(args) -> dict:
    sv = Solver()
    q = SEED.copy()
    pts, tips = [], []
    for t, target in path_points(args.shape, args.x, args.y, args.zc, args.side, args.edge_s):
        q = sv.solve(target, q)
        err = float(np.linalg.norm(target - sv.tip(q)))
        if err > 1e-3:
            raise SystemExit(f"IK failed at t={t:.1f} s (target {np.round(target, 3)}, residual {err * 1000:.1f} mm): "
                             "move the shape closer to the arm or make it smaller")
        pts.append({"t_s": round(t, 6), "positions": [round(float(v), 6) for v in q]})
        tips.append(sv.tip(q))
    tips = np.array(tips)
    qs = np.array([p["positions"] for p in pts])
    dq = np.abs(np.diff(qs, axis=0)).max()
    print(f"{args.shape}: {len(pts)} points, {pts[-1]['t_s']:.1f} s; tip x {tips[:, 0].min():.3f}..{tips[:, 0].max():.3f}, "
          f"y {tips[:, 1].min():+.3f}..{tips[:, 1].max():+.3f}, z {tips[:, 2].min():.3f}..{tips[:, 2].max():.3f}; "
          f"max joint step {dq:.3f} rad per 0.1 s; joint ranges "
          + ", ".join(f"j{i + 1} {qs[:, i].min():+.2f}..{qs[:, i].max():+.2f}" for i in range(4)))
    return {
        "schema": "crackvision.joint_trajectory/1",
        "created_utc": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "producer": f"tools/make_box_trajectory.py:{args.shape}",
        "purpose": "test",
        "planning_frame": "base_link",
        "joint_names": NAMES,
        "notes": (f"gripper tip draws a {args.shape} (side {args.side} m) in the plane x={args.x} m, centred y={args.y} "
                  f"z={args.zc} m; DLS IK on the repo URDF, J5/J6 = 0; {args.edge_s} s per edge, min-jerk"),
        "points": pts,
        "limits_file": {"path": "config/robot/b601_dm_limits.yaml", "sha256": None},
        "end_effector_config_sha256": None,
        "scene_config_sha256": None,
        "source": {"paths3d": None},
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--shape", choices=("box", "circle"), default="box")
    ap.add_argument("--x", type=float, default=0.30, help="plane distance in front of the base, m")
    ap.add_argument("--y", type=float, default=0.0, help="centre, left of the arm, m")
    ap.add_argument("--zc", type=float, default=0.50, help="centre height, m")
    ap.add_argument("--side", type=float, default=0.15, help="box side / circle diameter, m")
    ap.add_argument("--edge-s", type=float, default=3.0, help="seconds per edge (circle: a quarter of the loop)")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    doc = build(args)
    args.out.write_text(json.dumps(doc, indent=1) + "\n")
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""tools/make_fancy_trajectory.py — show trajectories for the B601-DM: the gripper tip draws figures in the air.

Each shape is a tool-tip path in base_link (x forward, y left, z up), turned into a crackvision.joint_trajectory/1
file by crackvision.arm_ik (DLS IK on the repo URDF, warm-started so the joints never jump branch, the tool moving
at a set speed along the path with smooth starts and stops, then slowed until every joint stays under 70 % of the
runner's planned-speed caps). ~/rebot_setup/run-trajectory.sh plays the file: approach from the fold, the shape,
return to the fold.

Wall shapes are drawn in the vertical plane x = --x with the tool pointing forward (+x), as on a wall:
  figure8   lemniscate of Gerono, 20 x 10 cm                 star      5-point star, stops at every point
  heart     heart curve                                       spiral    Archimedean spiral out, then back in
  rose      3-petal rose                                      lissajous 3:2 Lissajous knot
  wave      sine wave whose amplitude swells and fades        helix     corkscrew moving toward the wall (3D)
Table shapes are drawn above the table with the tool pointing straight down (the inspection pose; the tip can
only point straight down between z 0.13 and 0.18 m):
  raster    lawnmower scan of a 16 x 10 cm patch              crack     a jagged crack-like line, the motion the
                                                                         camera pipeline produces
  showreel  figure8, star and heart in one run (wall)

  ./env.sh python tools/make_fancy_trajectory.py --shape figure8 --out OUT.json [--x 0.32 --zc 0.45 --size 0.16]
  ./env.sh python tools/make_fancy_trajectory.py --all --out-dir DIR
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np

from crackvision.arm_ik import ArmIK, IKError, Piece, build_trajectory, trajectory_doc

WALL = {"figure8", "star", "heart", "spiral", "rose", "lissajous", "wave", "helix", "showreel"}
TABLE = {"raster", "crack"}
SHAPES = sorted(WALL | TABLE)
FWD = np.array([1.0, 0.0, 0.0])
DOWN = np.array([0.0, 0.0, -1.0])
N = 600                                     # samples per closed curve (the IK follows them; ~1 mm apart)


def _wall(y: np.ndarray, z: np.ndarray, x: float, y0: float, zc: float) -> np.ndarray:
    return np.stack([np.full_like(y, x), y0 + y, zc + z], axis=1)


def _closed(f, n: int = N) -> tuple[np.ndarray, np.ndarray]:
    th = np.linspace(0.0, 2 * math.pi, n + 1)
    return f(th)


def wall_pieces(shape: str, x: float, y0: float, zc: float, size: float, speed: float) -> list[Piece]:
    h = size / 2

    def piece(y, z, stop=True, label=shape, xs=None, sp=speed):
        pts = _wall(y, z, x, y0, zc)
        if xs is not None:
            pts[:, 0] = xs
        return Piece(pts, np.tile(FWD, (len(pts), 1)), sp, stop, label)

    if shape == "figure8":                       # lemniscate of Gerono, starts and ends at the centre
        y, z = _closed(lambda t: (h * np.sin(t), h / 2 * np.sin(2 * t) * 1.0))
        return [piece(y * 1.25, z)]
    if shape == "star":                          # {5/2} star polygon, a full stop at every point
        ang = [math.pi / 2 + k * 4 * math.pi / 5 for k in range(6)]
        v = [(h * math.cos(a), h * math.sin(a)) for a in ang]
        out = []
        for (ya, za), (yb, zb) in zip(v[:-1], v[1:]):
            s = np.linspace(0, 1, 60)
            out.append(piece(ya + (yb - ya) * s, za + (zb - za) * s))
        return out
    if shape == "heart":
        y, z = _closed(lambda t: (16 * np.sin(t) ** 3,
                                  13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)))
        return [piece(y * h / 17, z * h / 17)]
    if shape == "spiral":                        # out over 3 turns, back in along the same spiral
        t = np.linspace(0, 6 * math.pi, 3 * N)
        r = h * t / t[-1]
        y, z = r * np.cos(t), r * np.sin(t)
        return [piece(y, z, stop=False), piece(y[::-1], z[::-1])]
    if shape == "rose":                          # r = cos(3t), three petals
        y, z = _closed(lambda t: (h * np.cos(3 * t) * np.cos(t), h * np.cos(3 * t) * np.sin(t)), n=900)
        return [piece(y, z)]
    if shape == "lissajous":
        y, z = _closed(lambda t: (h * np.sin(3 * t), h * 0.75 * np.sin(2 * t)), n=900)
        return [piece(y, z)]
    if shape == "wave":                          # left to right: 3 periods, amplitude swelling then fading
        s = np.linspace(-1, 1, 700)
        y, z = h * 1.3 * s, h * 0.6 * np.sin(3 * math.pi * s) * np.cos(math.pi * s / 2)
        return [piece(y, z, stop=False), piece(y[::-1], -z[::-1])]
    if shape == "helix":                         # 3 turns of radius h/2 while moving 8 cm toward the wall
        t = np.linspace(0, 6 * math.pi, 3 * N)
        y, z = h / 2 * np.sin(t), h / 2 * (1 - np.cos(t))
        xs = x - 0.04 + 0.08 * t / t[-1]
        return [piece(y, z - h / 2, xs=xs)]
    if shape == "showreel":
        out = []
        for sub, dy in (("figure8", 0.0), ("star", 0.0), ("heart", 0.0)):
            out += wall_pieces(sub, x, y0 + dy, zc, size, speed)
        # join the shapes with straight moves (each shape starts and ends where it began)
        joined = [out[0]]
        for pc in out[1:]:
            a, b = joined[-1].points[-1], pc.points[0]
            if np.linalg.norm(b - a) > 1e-6:
                s = np.linspace(0, 1, 40)[:, None]
                joined.append(Piece(a + (b - a) * s, np.tile(FWD, (40, 1)), speed, True, "transit"))
            joined.append(pc)
        return joined
    raise ValueError(shape)


def table_pieces(shape: str, x: float, y0: float, z: float, size: float, speed: float) -> list[Piece]:
    h = size / 2
    if shape == "raster":                        # 5 passes across y, 2.5 cm apart in x, tool pointing down
        rows = 5
        xs = x - 0.05 + 0.10 * np.arange(rows) / (rows - 1)
        out = []
        for k, xr in enumerate(xs):
            ys = np.linspace(-h, h, 120) * (1 if k % 2 == 0 else -1)
            out.append(Piece(np.stack([np.full_like(ys, xr), y0 + ys, np.full_like(ys, z)], 1),
                             np.tile(DOWN, (len(ys), 1)), speed, True, f"pass{k + 1}"))
            if k < rows - 1:
                xn = xs[k + 1]
                s = np.linspace(0, 1, 30)
                out.append(Piece(np.stack([xr + (xn - xr) * s, np.full_like(s, y0 + ys[-1]), np.full_like(s, z)], 1),
                                 np.tile(DOWN, (30, 1)), speed, True, "step"))
        return out
    if shape == "crack":                         # a deterministic jagged line, like a skeletonised crack
        rng = np.random.default_rng(7)
        k = 9
        ys = np.linspace(-h, h, k)
        xs = x + np.cumsum(rng.normal(0, 0.008, k))
        xs -= xs.mean() - x
        corner_y = np.concatenate([np.linspace(ys[i], ys[i + 1], 25, endpoint=False) for i in range(k - 1)] + [[ys[-1]]])
        corner_x = np.interp(corner_y, ys, xs)
        pts = np.stack([corner_x, y0 + corner_y, np.full_like(corner_y, z)], 1)
        return [Piece(pts, np.tile(DOWN, (len(pts), 1)), speed, True, "crack")]
    raise ValueError(shape)


DEFAULTS = {  # shape: (x, y, z-or-zc, size, tool speed m/s)
    "figure8": (0.32, 0.0, 0.45, 0.16, 0.05), "star": (0.32, 0.0, 0.45, 0.18, 0.05),
    "heart": (0.32, 0.0, 0.45, 0.16, 0.05), "spiral": (0.32, 0.0, 0.45, 0.16, 0.05),
    "rose": (0.32, 0.0, 0.45, 0.16, 0.05), "lissajous": (0.32, 0.0, 0.45, 0.16, 0.05),
    "wave": (0.32, 0.0, 0.45, 0.16, 0.05), "helix": (0.32, 0.0, 0.45, 0.14, 0.05),
    "showreel": (0.32, 0.0, 0.45, 0.14, 0.05),
    "raster": (0.30, 0.0, 0.15, 0.16, 0.04), "crack": (0.30, 0.0, 0.15, 0.16, 0.03),
}


def build(shape: str, x: float, y: float, z: float, size: float, speed: float, speed_fraction: float):
    ik = ArmIK()
    pieces = (wall_pieces if shape in WALL else table_pieces)(shape, x, y, z, size, speed)
    first = pieces[0]
    q0 = ik.solve_global(first.points[0], first.pointing[0]).q
    t, q, rep = build_trajectory(ik, pieces, q0, speed_fraction=speed_fraction)
    doc = trajectory_doc(
        t, q, producer=f"tools/make_fancy_trajectory.py:{shape}", purpose="test",
        notes=(f"{shape}: the tool tip ({ik.tool_value_status} tool_tip = gripper_link origin) draws it "
               + (f"in the plane x={x} m centred y={y} z={z} m, pointing +x" if shape in WALL else
                  f"at z={z} m around x={x} y={y} m, pointing down")
               + f"; size {size} m, tool speed <= {speed} m/s; crackvision.arm_ik DLS IK on the repo URDF; "
               f"joint peaks <= {speed_fraction:.0%} of the runner caps"),
        extra={"shape": {"name": shape, "x_m": x, "y_m": y, "z_m": z, "size_m": size, "tool_speed_m_s": speed},
               "report": asdict(rep)})
    return doc, rep


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--shape", choices=SHAPES)
    ap.add_argument("--all", action="store_true", help="every shape, into --out-dir as <shape>.json")
    ap.add_argument("--x", type=float, help="wall: plane distance in front of the base; table: centre x (m)")
    ap.add_argument("--y", type=float, help="centre, left of the arm (m)")
    ap.add_argument("--z", "--zc", dest="z", type=float, help="wall: centre height; table: tool height (m)")
    ap.add_argument("--size", type=float, help="overall size (m)")
    ap.add_argument("--speed", type=float, help="tool speed along the path (m/s)")
    ap.add_argument("--speed-fraction", type=float, default=0.7, help="joint peak speed / runner cap (<= 1)")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--out-dir", type=Path)
    args = ap.parse_args(argv)
    if not (0 < args.speed_fraction <= 1):
        ap.error("--speed-fraction must be in (0, 1]")
    shapes = SHAPES if args.all else [args.shape] if args.shape else ap.error("give --shape or --all")
    if args.all and not args.out_dir or not args.all and not args.out:
        ap.error("--all needs --out-dir; --shape needs --out")
    rc = 0
    for shape in shapes:
        d = DEFAULTS[shape]
        vals = [d[i] if v is None else v for i, v in enumerate((args.x, args.y, args.z, args.size, args.speed))]
        out = args.out_dir / f"{shape}.json" if args.all else args.out
        try:
            doc, rep = build(shape, *vals, args.speed_fraction)
        except IKError as exc:
            print(f"{shape}: REFUSED: {exc}", file=sys.stderr)
            rc = 1
            continue
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(doc, indent=1) + "\n")
        print(f"{shape:10s} {rep.n_points:4d} pts {rep.duration_s:6.1f} s  peak/cap "
              + " ".join(f"{f:.2f}" for f in rep.peak_speed_fraction_of_cap)
              + f"  slowed x{rep.slowed_by:.2f}  min gripper z {rep.min_gripper_z_m:.3f} m  tip err "
              f"{rep.max_tip_error_mm:.2f} mm -> {out}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())

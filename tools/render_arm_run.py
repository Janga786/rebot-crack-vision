"""tools/render_arm_run.py — watch a planned or recorded arm run: replay its joint angles on the B601-DM model in MuJoCo.

Input: a run CSV written by ~/rebot_setup/demo_move.py (simulated rehearsals in runs_sim/, real runs in runs/),
200 Hz rows of t_s, mode, and j1..j7 _ref (commanded) and _pos (measured). The six arm joints drive the
repo's URDF (presentation/sim/reBot_B601_DM_with_gripper.urdf, the one the planner and arm_guard use; joint
signs were checked against the real arm on 2026-10-10). Kinematic playback in real time: no physics, so the
video shows exactly the commanded motion (or, with --column pos, what the joints measured).

Needs an env with `mujoco` and `cv2` (on this workstation the navilaenv venv) plus ffmpeg:
    MUJOCO_GL=egl <python> tools/render_arm_run.py RUN.csv OUT.mp4 [--column ref|pos] [--title TEXT] [--trail]

--trail draws the gripper tip's path (gripper_link origin) as it goes and adds a third view facing the arm, so a
drawn figure (tools/make_fancy_trajectory.py) can be seen.
"""

from __future__ import annotations

import argparse
import csv
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
URDF = REPO / "presentation" / "sim" / "reBot_B601_DM_with_gripper.urdf"
ARM = [f"joint{i}" for i in range(1, 7)]
VIEWS = [  # (azimuth, elevation, distance, label)
    (135.0, -18.0, 1.05, "front-left"),
    (225.0, -18.0, 1.05, "front-right"),
]


def load_model(urdf: Path):
    import mujoco
    text = urdf.read_text()
    text = re.sub(r"(<robot[^>]*>)", r'\1<mujoco><compiler discardvisual="false" fusestatic="false" '
                                     r'balanceinertia="true"/></mujoco>', text, count=1)
    spec = mujoco.MjSpec.from_string(text)
    wb = spec.worldbody
    wb.add_light(pos=[0.6, -0.6, 1.8], dir=[-0.3, 0.3, -1.0], diffuse=[0.8, 0.8, 0.8])
    wb.add_light(pos=[-0.6, 0.6, 1.8], dir=[0.3, -0.3, -1.0], diffuse=[0.5, 0.5, 0.5])
    wb.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0.8, 0.8, 0.02], rgba=[0.82, 0.82, 0.80, 1.0])
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 1920, 1080   # offscreen render buffer
    return spec.compile()


def read_run(path: Path, column: str):
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r["t_s"]) for r in rows])
    q = np.array([[float(r[f"j{i}_{column}"] or "nan") for i in range(1, 7)] for r in rows])
    modes = [r["mode"] for r in rows]
    return t, q, modes


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--column", choices=("ref", "pos"), default="ref", help="commanded (ref) or measured (pos)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--size", default="640x540", help="per view")
    ap.add_argument("--title", default="")
    ap.add_argument("--trail", action="store_true", help="draw the gripper tip's path; add a view facing the arm")
    args = ap.parse_args(argv)
    views = VIEWS + ([(0.0, -5.0, 1.15, "facing the arm")] if args.trail else [])
    import cv2
    import mujoco

    model = load_model(URDF)
    data = mujoco.MjData(model)
    qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in ARM]
    w, h = (int(x) for x in args.size.split("x"))
    renderer = mujoco.Renderer(model, height=h, width=w)
    t, q, modes = read_run(args.csv, args.column)
    t = t - t[0]
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        print("ffmpeg is needed", file=sys.stderr)
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    proc = subprocess.Popen([ffmpeg, "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                             "-s", f"{w * len(views)}x{h}", "-r", str(args.fps), "-i", "-", "-c:v", "libx264",
                             "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                             str(args.out)], stdin=subprocess.PIPE)
    cams = []
    for az, el, dist, label in views:
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [0.15, 0.0, 0.32] if label == "facing the arm" else [0.0, 0.0, 0.22]
        cam.azimuth, cam.elevation, cam.distance = az, el, dist
        cams.append(cam)
    tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper_link")
    trail: list[np.ndarray] = []
    last = q[0]
    n_frames = int(t[-1] * args.fps) + 1
    for k in range(n_frames):
        i = min(int(np.searchsorted(t, k / args.fps)), len(t) - 1)
        if np.all(np.isfinite(q[i])):
            last = q[i]
        for adr, v in zip(qadr, last):
            data.qpos[adr] = v
        mujoco.mj_forward(model, data)
        if args.trail and (not trail or np.linalg.norm(data.xpos[tip_id] - trail[-1]) > 0.002):
            trail.append(data.xpos[tip_id].copy())
        tiles = []
        for cam, (_, _, _, label) in zip(cams, views):
            renderer.update_scene(data, camera=cam)
            scn = renderer.scene
            for p in trail[-(scn.maxgeom - scn.ngeom):]:
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.003, 0, 0]),
                                    p, np.eye(3).flatten(), np.array([0.85, 0.1, 0.1, 1.0], dtype=np.float32))
                scn.ngeom += 1
            img = renderer.render().copy()
            cv2.putText(img, label, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1, cv2.LINE_AA)
            tiles.append(img)
        frame = np.hstack(tiles)
        hud = (f"{args.title}  t={k / args.fps:5.1f} s  {modes[i]:<10}  "
               + " ".join(f"j{j + 1} {v:+.2f}" for j, v in enumerate(last)))
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 28), (255, 255, 255), -1)
        cv2.putText(frame, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
        proc.stdin.write(np.ascontiguousarray(frame).tobytes())
    proc.stdin.close()
    if proc.wait() != 0:
        return 1
    print(f"wrote {args.out} ({n_frames} frames, {n_frames / args.fps:.1f} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

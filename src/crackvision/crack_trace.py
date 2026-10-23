"""crackvision.crack_trace — from the wrist camera seeing a crack to a joint trajectory that traces it.

The B601-DM runner (~/rebot_setup/run-trajectory.sh -> demo_move.py, behind arm_guard) plays
crackvision.joint_trajectory/1 files: approach from the fold, the file, return to the fold. This module makes the two
files the crack pipeline needs and the capture in between (scripts/crack_pipeline.sh sequences them):

  scan-plan  the scan pose: the D405 looking straight at the target surface from --view-m away. Writes a
             trajectory that holds that pose for --dwell-s, during which the camera captures.
  capture    run while the scan trajectory holds (demo_move.py --state-file publishes the measured pose): waits
             until the arm is settled at the scan pose, grabs a burst, keeps the sharpest frame, and writes a
             normal case (RGB + aligned depth under data/camera/<case>/), the §9 capture record with the measured
             joint angles, then segment + paths (the accepted operator_segment_paths stages) and the overlays.
  plan       path3d (crack pixels -> base_link surface points) -> the trace: the tool tip follows each crack at
             --hover-m above the surface (raised automatically to keep gripper_link above arm_guard's 0.12 m
             floor and above 3 sigma of the calibration uncertainty), pointing into the surface (tilted up to
             --max-tilt-deg where the wrist cannot point straight down). Writes the trajectory, a report, and a
             preview: the planned path drawn back into the camera image over the crack.
  check      (synthetic cases) the planned surface path against tools/synth_scene3d.py's ground truth.

  ./env.sh python -m crackvision.crack_trace scan-plan --out data/crack_trace/scan.json [--target 0.30 0 0]
  ./env.sh python -m crackvision.crack_trace capture --case CASE --state-file ~/rebot_setup/runs/arm_state.json
  ./env.sh python -m crackvision.crack_trace plan --case CASE [--hover-m 0.05]

The camera mount and the tool tip are still nominal (CAD priors, end_effector.yaml): until GEOM-05/07 calibrate
them, the traced path can be off by a few centimetres. The hover margin is sized for that; nothing here touches
hardware.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import logging
import math
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw

from crackvision import capture_record, kinematics, naming
from crackvision.arm_ik import (GRIPPER_FLOOR_Z_M, ArmIK, IKError, Piece, build_trajectory, trajectory_doc)
from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE, setup_logging

STATE_SCHEMA = "rebot.arm_state/1"
PLAN_SCHEMA = "crackvision.crack_trace_plan/1"
SCAN_SCHEMA = "crackvision.crack_scan_pose/1"
UP = np.array([0.0, 0.0, 1.0])
TIP_Z_MIN_M = GRIPPER_FLOOR_Z_M + 0.015          # tool_tip == gripper_link origin (nominal): stay 15 mm above the floor


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def trace_dir(root: Path) -> Path:
    return Path(root) / "data" / "crack_trace"


def camera_frames(ee: kinematics.EndEffector) -> tuple[np.ndarray, np.ndarray]:
    """(T_gripper_link_camera_link, T_gripper_link_color_optical). camera_link +X is the optical axis."""
    T_cl = np.asarray(ee.T_gripper_link_camera_link, dtype=float)
    T_opt = T_cl @ kinematics.transform_from_xyz_rpy(kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME_XYZ,
                                                      kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME_RPY)
    return T_cl, T_opt


# ---- scan pose ---------------------------------------------------------------------------------------------------

def scan_pose(target: np.ndarray, normal: np.ndarray, view_m: float, ik: ArmIK | None = None) -> tuple[np.ndarray, dict]:
    """Joint angles that put the camera view_m from target along normal, looking back along it."""
    ik = ik or ArmIK()
    ee = kinematics.load_end_effector()
    T_cl, T_opt = camera_frames(ee)
    n = normal / np.linalg.norm(normal)
    r = ik.solve_global(target + view_m * n, -n, T_tip=T_cl)
    T_cam = ik.fk_frame(r.q, T_opt)
    return r.q, {"camera_position_m": T_cam[:3, 3].round(4).tolist(),
                 "optical_axis_base": T_cam[:3, 2].round(4).tolist(),
                 "gripper_link_z_m": round(float(ik.fk_frame(r.q, np.eye(4))[2, 3]), 4),
                 "wrist_camera_value_status": ee.wrist_camera_value_status}


def cmd_scan_plan(cfg: Config, args: argparse.Namespace, log: logging.Logger) -> int:
    target, normal = np.array(args.target, float), np.array(args.normal, float)
    try:
        q, info = scan_pose(target, normal, args.view_m)
    except IKError as exc:
        log.error("scan pose unreachable: %s", exc)
        return EXIT_PRECONDITION
    if info["gripper_link_z_m"] < TIP_Z_MIN_M:
        log.error("scan pose puts gripper_link at z %.3f m, below %.3f m: raise --view-m", info["gripper_link_z_m"],
                  TIP_Z_MIN_M)
        return EXIT_PRECONDITION
    t = np.array([0.0, args.dwell_s])
    doc = trajectory_doc(t, np.vstack([q, q]), producer="crackvision.crack_trace:scan-plan", purpose="test",
                         notes=(f"scan pose: the wrist camera {args.view_m} m from the surface point "
                                f"{target.round(3).tolist()} (normal {normal.round(3).tolist()}), held {args.dwell_s} s "
                                "for crack_trace capture; nominal camera mount (CAD prior)"),
                         extra={"scan": {"schema": SCAN_SCHEMA, "target_m": target.tolist(), "normal": normal.tolist(),
                                         "view_m": args.view_m, "dwell_s": args.dwell_s,
                                         "q_rad": q.round(6).tolist(), **info}})
    out = args.out or trace_dir(cfg.root) / "scan.json"
    if args.dry_run:
        log.info("dry-run: would write %s", out)
        return EXIT_OK
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, indent=1) + "\n")
    print(f"scan pose q = {np.round(q, 3).tolist()}  camera at {info['camera_position_m']} looking "
          f"{info['optical_axis_base']}; holds {args.dwell_s} s -> {out}")
    return EXIT_OK


# ---- capture -----------------------------------------------------------------------------------------------------

def read_state(path: Path) -> dict | None:
    try:
        d = json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return d if d.get("schema") == STATE_SCHEMA else None


def settled(states: list[dict], q_scan: np.ndarray, tol_rad: float, vel_tol: float) -> bool:
    """The last states all sit at the scan pose, inside the dwell, and do not move."""
    if not states:
        return False
    for s in states:
        q = np.array(s["positions_rad"])
        if s.get("segment_kind") != "trajectory" or np.abs(q - q_scan).max() > tol_rad:
            return False
        if np.abs(np.array(s["velocity_rad_s"])).max() > vel_tol:
            return False
    qs = np.array([s["positions_rad"] for s in states])
    return float(np.ptp(qs, axis=0).max()) < 0.003


def wait_settled(state_file: Path, q_scan: np.ndarray, timeout_s: float, *, tol_rad: float = 0.04,
                 vel_tol: float = 0.05, hold_s: float = 0.6, poll_s: float = 0.05, log=None) -> dict:
    """Poll the runner's state file until the arm has rested at q_scan for hold_s. Returns the last state."""
    t_end = time.monotonic() + timeout_s
    window: list[dict] = []
    last_wall = None
    said = None
    while time.monotonic() < t_end:
        s = read_state(state_file)
        if s is not None and s["wall_time_ns"] != last_wall:
            last_wall = s["wall_time_ns"]
            window = [w for w in window + [s] if s["wall_time_ns"] - w["wall_time_ns"] <= hold_s * 1e9]
            phase = f"{s['mode']}/{s.get('segment_kind')}"
            if log and phase != said:
                log.info("arm: %s", phase)
                said = phase
            if (s["wall_time_ns"] - window[0]["wall_time_ns"] >= (hold_s - 0.1) * 1e9
                    and settled(window, q_scan, tol_rad, vel_tol)):
                return s
            if s["mode"] in ("final_hold", "park", "guard_hold") or s.get("segment_kind") == "return":
                raise TimeoutError(f"the arm left the scan pose without settling there (now {phase})")
        time.sleep(poll_s)
    raise TimeoutError(f"the arm did not settle at the scan pose within {timeout_s:.0f} s (state file {state_file})")


def sharpness(rgb: np.ndarray) -> float:
    g = rgb.astype(np.float32).mean(axis=2)
    lap = g[1:-1, 1:-1] * 4 - g[:-2, 1:-1] - g[2:, 1:-1] - g[1:-1, :-2] - g[1:-1, 2:]
    return float(lap.var())


def write_case(cfg: Config, case_id: str, frame, info: dict, state: dict, capture_ns: int, source_kind: str,
               source_ref: str, synthetic: bool = False) -> Path:
    """RGB + aligned depth under data/camera/<case>/, and the §9 capture record (measured joint angles)."""
    from crackvision.recording import _write_depth_png
    case_dir = cfg.root / "data" / "camera" / case_id
    (case_dir / "input").mkdir(parents=True, exist_ok=True)
    src = case_dir / "input" / f"{case_id}.png"
    Image.fromarray(frame.color_rgb, mode="RGB").save(src)
    depth_path = case_dir / f"{case_id}_depth.png"
    _write_depth_png(frame.depth_u16, depth_path)
    ee = kinematics.load_end_effector()
    _xyz, quat = kinematics.transform_to_xyz_quat(kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME)
    urdf = kinematics.DEFAULT_URDF
    ci = dict(info["color_intrinsics"])
    doc = capture_record.build_record(
        case_id=case_id, synthetic=synthetic, image_height=frame.color_rgb.shape[0],
        image_width=frame.color_rgb.shape[1], color_intrinsics=ci,
        depth_file=depth_path.relative_to(cfg.root).as_posix(),
        depth_scale_m_per_unit=float(info["depth_scale_m_per_unit"]),
        depth_aligned_to=info.get("aligned_to", "color"), source_kind=source_kind, source_ref=source_ref,
        source_frame_index=int(frame.frame_index), capture_stamp_ns=int(capture_ns),
        joint_names=list(state["joint_names"]), positions_rad=list(state["positions_rad"]),
        robot_stamp_ns=int(state["wall_time_ns"]), robot_stamp_source="demo_move_state_file",
        kinematic_model_path=(urdf.relative_to(cfg.root).as_posix() if urdf.is_relative_to(cfg.root) else str(urdf)),
        kinematic_model_sha256=kinematics.file_sha256(urdf), end_effector_path="config/robot/end_effector.yaml",
        end_effector_sha256=ee.sha256, wrist_camera_value_status=ee.wrist_camera_value_status,
        tool_value_status=ee.tool_value_status, optical_xyz_m=list(kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME_XYZ),
        optical_quat_xyzw=list(quat), optical_source="nominal_d405")
    capture_record.write_record(doc, capture_record.capture_record_path(cfg.root, case_id))
    (case_dir / "crack_trace_capture.json").write_text(json.dumps(
        {"case_id": case_id, "state": state, "capture_stamp_ns": int(capture_ns), "source_kind": source_kind,
         "source_ref": source_ref, "image": src.relative_to(cfg.root).as_posix()}, indent=2) + "\n")
    return src


def run_perception(cfg: Config, case_id: str, src: Path, quiet: bool = True) -> int:
    """segment + paths (operator_segment_paths) and the mask/overlay renders, exactly as camera.py snap does."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("operator_segment_paths",
                                                  cfg.root / "scripts" / "operator_segment_paths.py")
    osp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(osp)
    q = ["-q"] if quiet else []
    rc = osp.main(["--root", str(cfg.root), "--case-id", case_id, "--source", str(src), "--stage", "both", "--force", *q])
    if rc == EXIT_OK:
        from crackvision import visualize
        rc = visualize.main(["--root", str(cfg.root), "--cases", case_id, *q])
    return rc


def cmd_capture(cfg: Config, args: argparse.Namespace, log: logging.Logger) -> int:
    scan = json.loads(Path(args.scan).read_text())["scan"]
    q_scan = np.array(scan["q_rad"])
    case_id = naming.derive_case_id(Path(f"{args.case}.png"))
    if args.dry_run:
        log.info("dry-run: would capture case %s at q %s", case_id, q_scan.round(3).tolist())
        return EXIT_OK
    from crackvision import camera_pipeline as cp
    from crackvision.d405_live import D405Error
    try:
        cam_ctx = cp.open_camera(cfg, args, log)
        cam = cam_ctx.__enter__()          # open (and warm up) the camera before the arm arrives
    except D405Error as exc:
        log.error("camera: %s", exc)
        return EXIT_PRECONDITION
    try:
        log.info("waiting for the arm to settle at the scan pose (state file %s)", args.state_file)
        try:
            wait_settled(Path(args.state_file), q_scan, args.timeout_s, log=log)
        except TimeoutError as exc:
            log.error("%s", exc)
            return EXIT_RUNTIME
        info = cam.session_info
        states, frames = [], []
        for f in cam.frames(args.burst):
            frames.append(f)
            s = read_state(Path(args.state_file))
            if s is not None:
                states.append(s)
        after = read_state(Path(args.state_file))
        states += [after] if after else []
    finally:
        cam_ctx.__exit__(None, None, None)
    if not frames or not states:
        log.error("no frames (%d) or no arm states (%d) during the burst", len(frames), len(states))
        return EXIT_RUNTIME
    if not settled(states, q_scan, 0.04, 0.05):
        log.error("the arm moved during the burst (or left the scan pose): not using this capture")
        return EXIT_RUNTIME
    scores = [sharpness(f.color_rgb) for f in frames]
    frame = frames[int(np.argmax(scores))]
    replay = bool(getattr(args, "replay", None))
    if replay:     # a recording's own clock is not today's: the arm was still for the whole burst, use its time
        state = states[-1]
        capture_ns = state["wall_time_ns"]
    else:
        capture_ns = time.time_ns() - (time.monotonic_ns() - frame.host_monotonic_ns)
        state = min(states, key=lambda s: abs(s["wall_time_ns"] - capture_ns))
    src = write_case(cfg, case_id, frame, info, state, capture_ns, "d405_replay" if replay else "d405_live",
                     str(args.replay) if replay else str(info.get("device", {}).get("serial", "d405")))
    print(f"captured case {case_id}: frame {int(np.argmax(scores)) + 1}/{len(frames)} (sharpest), arm at "
          f"{np.round(state['positions_rad'], 3).tolist()} (scan pose {np.round(q_scan, 3).tolist()})")
    rc = run_perception(cfg, case_id, src, quiet=args.quiet)
    if rc != EXIT_OK:
        log.error("segment/paths failed for %s (exit %d)", case_id, rc)
        return rc
    paths = json.loads((cfg.root / "data" / "paths" / f"{case_id}_paths.json").read_text())
    n = len(paths.get("components", []))
    print(f"perception: {n} crack component(s); overlay data/paths/{case_id}_paths_overlay.png"
          + ("" if n else "\n  no crack found in the view: nothing to trace"))
    return EXIT_OK if n else EXIT_PRECONDITION


# ---- plan --------------------------------------------------------------------------------------------------------

def _plane_fit(p: np.ndarray, up_hint: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    c = p.mean(axis=0)
    _, s, vt = np.linalg.svd(p - c)
    n = vt[2]
    if n @ up_hint < 0:
        n = -n
    rms = float(s[2] / math.sqrt(len(p))) if len(p) > 2 else 0.0
    return c, n, rms


def _smooth(p: np.ndarray, window: int) -> np.ndarray:
    if window < 2 or len(p) <= window:
        return p
    k = window // 2
    pad = np.vstack([np.repeat(p[:1], k, 0), p, np.repeat(p[-1:], k, 0)])
    out = np.array([pad[i:i + window].mean(axis=0) for i in range(len(p))])
    out[0], out[-1] = p[0], p[-1]
    return out


def crack_segments(doc: dict, min_length_m: float, all_paths: bool) -> list[dict]:
    """Trace segments from a paths3d document: surface points + normals of each segment's trace waypoints."""
    out = []
    for comp in doc.get("components", []):
        for pl in comp.get("polylines", []):
            if pl.get("kind") != "main" and not all_paths:
                continue
            for seg in pl.get("segments", []):
                wps = [w for w in seg.get("waypoints", []) if w["phase"] == "trace"]
                if len(wps) < 2:
                    continue
                pts = np.array([w["surface_point_m"] for w in wps])
                length = float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())
                if length < min_length_m:
                    continue
                out.append({"component": comp["component_id"], "kind": pl["kind"], "points": pts,
                            "normals": np.array([w["normal_base"] for w in wps]), "length_m": length,
                            "sigma_along_normal_m": max(w["sigma_along_normal_m"] for w in wps)})
    out.sort(key=lambda s: -s["length_m"])
    return out


def tilted(normal: np.ndarray, point: np.ndarray, tilt_deg: float) -> np.ndarray:
    """The tool's pointing: into the surface (-normal), tilted tilt_deg so the tool leans away from the base."""
    d = -normal
    r = np.array([point[0], point[1], 0.0])
    r = r - (r @ normal) * normal
    if np.linalg.norm(r) < 1e-9 or tilt_deg == 0:
        return d
    r /= np.linalg.norm(r)
    a = math.radians(tilt_deg)
    return math.cos(a) * d + math.sin(a) * r


def plan_trace(doc: dict, *, hover_m: float, approach_m: float, trace_speed: float, transit_speed: float,
               max_tilt_deg: float, min_length_m: float, all_paths: bool, max_segments: int, smooth: int,
               flat: bool, speed_fraction: float, log: logging.Logger) -> tuple[np.ndarray, np.ndarray, dict]:
    segs = crack_segments(doc, min_length_m, all_paths)[:max_segments]
    if not segs:
        raise IKError(f"no crack segment of at least {min_length_m * 1000:.0f} mm in the paths3d document")
    ik = ArmIK()
    pieces: list[Piece] = []
    planned = []
    sigma = max(s["sigma_along_normal_m"] for s in segs)
    hover = max(hover_m, 3 * sigma)
    for k, s in enumerate(segs):
        p, nrm = s["points"], s["normals"]
        if flat:
            c, n, rms = _plane_fit(p, nrm.mean(axis=0))
            p = p - np.outer((p - c) @ n, n)
            nrm = np.tile(n, (len(p), 1))
        p = _smooth(p, smooth)
        n_mean = nrm.mean(axis=0)
        n_mean /= np.linalg.norm(n_mean)
        # one tilt and one hover for the whole segment, so the tool's motion mirrors the crack; the whole segment
        # (approach, trace, retract) must be followable at that tilt
        line = lambda a, b, n=25: a + (b - a) * np.linspace(0, 1, n)[:, None]
        chosen = None
        for tilt in [0.0] + [t for t in (10.0, 20.0, 30.0, 40.0) if t <= max_tilt_deg]:
            h = hover
            while (p + h * nrm)[:, 2].min() < TIP_Z_MIN_M and h < 0.5:
                h += 0.005
            tips = p + h * nrm
            point = np.array([tilted(n, q, tilt) for n, q in zip(nrm, p)])
            above = p[0] + (h + approach_m) * nrm[0], p[-1] + (h + approach_m) * nrm[-1]
            seg_pieces = [Piece(line(above[0], tips[0]), np.tile(point[0], (25, 1)), transit_speed, True, f"approach{k}"),
                          Piece(tips, point, trace_speed, True, f"trace{k}"),
                          Piece(line(tips[-1], above[1]), np.tile(point[-1], (25, 1)), transit_speed, True,
                                f"retract{k}")]
            try:
                allp = np.vstack([pc.points for pc in seg_pieces])
                alln = np.vstack([pc.pointing for pc in seg_pieces])
                ik.follow(allp, alln, ik.solve_global(allp[0], alln[0]).q, max_joint_step_rad=0.15)
                chosen = (tilt, h, tips, point, above, seg_pieces)
                break
            except IKError as exc:
                log.info("segment %d: tilt %.0f deg, hover %.3f m: %s", k, tilt, h, exc)
        if chosen is None:
            raise IKError(f"segment {k} ({s['length_m'] * 1000:.0f} mm) cannot be traced at up to {max_tilt_deg} deg "
                          "tilt: move the target closer to the arm's front")
        tilt, h, tips, point, above, seg_pieces = chosen
        if pieces:      # transit from the previous retract to this approach point, turning the tool on the way
            prev, prev_d = pieces[-1].points[-1], pieces[-1].pointing[-1]
            u = np.linspace(0, 1, 25)[:, None]
            dirs = (1 - u) * prev_d + u * point[0]
            pieces.append(Piece(line(prev, above[0]), dirs / np.linalg.norm(dirs, axis=1, keepdims=True),
                                transit_speed, True, "transit"))
        pieces += seg_pieces
        planned.append({"component": s["component"], "kind": s["kind"], "length_m": round(s["length_m"], 4),
                        "hover_m": round(h, 4), "tilt_deg": tilt, "n_points": len(tips),
                        "surface_points_m": np.round(p, 5).tolist(), "tool_points_m": np.round(tips, 5).tolist(),
                        "sigma_along_normal_m": round(s["sigma_along_normal_m"], 4)})
    q0 = ik.solve_global(pieces[0].points[0], pieces[0].pointing[0]).q
    t, q, rep = build_trajectory(ik, pieces, q0, speed_fraction=speed_fraction, floor_z_m=TIP_Z_MIN_M - 0.005)
    return t, q, {"segments": planned, "report": asdict(rep), "hover_floor_3sigma_m": round(3 * sigma, 4)}


def project(T_base_opt: np.ndarray, intr: dict, pts: np.ndarray) -> np.ndarray:
    T = np.linalg.inv(T_base_opt)
    pc = (T[:3, :3] @ pts.T).T + T[:3, 3]
    return np.stack([intr["fx"] * pc[:, 0] / pc[:, 2] + intr["ppx"], intr["fy"] * pc[:, 1] / pc[:, 2] + intr["ppy"]], 1)


def write_preview(cfg: Config, case_id: str, record_doc: dict, planned: list[dict], out: Path) -> None:
    """The camera image, the model's crack mask in red, the planned surface path in green (start = circle)."""
    src = cfg.root / "data" / "camera" / case_id / "input" / f"{case_id}.png"
    if not src.is_file():
        cand = list((cfg.root / "data" / "synthetic").glob(f"{case_id}_color.png"))
        src = cand[0] if cand else None
    base = np.array(Image.open(src).convert("RGB")) if src else np.full(
        (record_doc["image"]["height"], record_doc["image"]["width"], 3), 255, np.uint8)
    mask_file = cfg.root / naming.mask_path(case_id)
    if mask_file.is_file():
        m = np.array(Image.open(mask_file)) > 0
        m = m.any(axis=-1) if m.ndim > 2 else m
        base[m] = (0.4 * base[m] + 0.6 * np.array([255, 0, 0])).astype(np.uint8)
    img = Image.fromarray(base)
    d = ImageDraw.Draw(img)
    ik = ArmIK()
    _, T_opt = camera_frames(kinematics.load_end_effector())
    T = ik.fk_frame(record_doc["robot"]["positions_rad"], T_opt)
    for s in planned:
        uv = project(T, record_doc["color_intrinsics"], np.array(s["surface_points_m"]))
        d.line([tuple(p) for p in uv], fill=(0, 200, 0), width=2)
        x, y = uv[0]
        d.ellipse([x - 5, y - 5, x + 5, y + 5], outline=(0, 200, 0), width=2)
    d.text((8, 8), f"{case_id}: red = crack mask, green = planned trace (circle = start)", fill=(255, 255, 0))
    img.save(out)


def cmd_plan(cfg: Config, args: argparse.Namespace, log: logging.Logger) -> int:
    case_id = args.case
    p3 = cfg.root / "data" / "paths3d" / f"{case_id}_paths3d.json"
    if not args.skip_path3d:
        from crackvision import path3d
        rc = path3d.main(["--root", str(cfg.root), "--cases", case_id, "-q"])
        if rc != EXIT_OK:
            log.error("path3d failed for %s (exit %d)", case_id, rc)
            return rc
    if not p3.is_file():
        log.error("%s missing", p3)
        return EXIT_PRECONDITION
    doc = json.loads(p3.read_text())
    if doc.get("status") == "failed" or "components" not in doc:
        log.error("path3d refused %s: %s", case_id, doc.get("reason") or doc.get("message"))
        return EXIT_PRECONDITION
    try:
        t, q, plan = plan_trace(doc, hover_m=args.hover_m, approach_m=args.approach_m, trace_speed=args.speed,
                                transit_speed=args.transit_speed, max_tilt_deg=args.max_tilt_deg,
                                min_length_m=args.min_length_m, all_paths=args.all_paths,
                                max_segments=args.max_segments, smooth=args.smooth, flat=not args.curved,
                                speed_fraction=args.speed_fraction, log=log)
    except IKError as exc:
        log.error("no trace: %s", exc)
        return EXIT_PRECONDITION
    out_dir = trace_dir(cfg.root)
    out = args.out or out_dir / f"{case_id}_trace.json"
    reasons = doc.get("ineligible_reasons", [])
    hov = ", ".join(f"{s['hover_m'] * 100:.1f} cm" for s in plan["segments"])
    traj = trajectory_doc(
        t, q, producer="crackvision.crack_trace:plan", purpose="crack_task",
        notes=(f"case {case_id}: the tool tip traces {len(plan['segments'])} crack segment(s) at {hov} above the "
               f"surface, {args.speed * 1000:.0f} mm/s; camera mount and tool are "
               f"{doc['calibration']['wrist_camera']['value_status']}/{doc['calibration']['tool']['value_status']}"
               + (f"; paths3d not execution-eligible: {', '.join(reasons)}" if reasons else "")),
        source={"paths3d": p3.relative_to(cfg.root).as_posix(), "paths3d_sha256": _sha256(p3),
                "execution_eligible": doc.get("execution_eligible"), "ineligible_reasons": reasons},
        extra={"crack_trace": {"schema": PLAN_SCHEMA, **{k: v for k, v in plan.items() if k != "segments"},
                               "segments": [{k: v for k, v in s.items() if not k.endswith("points_m")}
                                            for s in plan["segments"]]}})
    if args.dry_run:
        log.info("dry-run: would write %s", out)
        return EXIT_OK
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(traj, indent=1) + "\n")
    (out_dir / f"{case_id}_trace_plan.json").write_text(json.dumps(plan, indent=1) + "\n")
    rec = json.loads(capture_record.capture_record_path(cfg.root, case_id).read_text())
    prev = out_dir / f"{case_id}_trace_preview.png"
    write_preview(cfg, case_id, rec, plan["segments"], prev)
    r = plan["report"]
    print(f"trace for {case_id}: {len(plan['segments'])} segment(s) ("
          + "; ".join(f"{s['length_m'] * 1000:.0f} mm, hover {s['hover_m'] * 100:.1f} cm, tilt {s['tilt_deg']:.0f} deg"
                      for s in plan["segments"])
          + f"), {r['duration_s']:.1f} s, joint peak/cap {max(r['peak_speed_fraction_of_cap']):.2f}, "
          f"min gripper z {r['min_gripper_z_m']:.3f} m\n  trajectory {out}\n  preview    {prev}")
    if reasons:
        print(f"  NOTE paths3d is not execution-eligible ({', '.join(reasons)}): expect cm-level offsets until the "
              "camera and tool are calibrated (GEOM-05/07)")
    return EXIT_OK


# ---- check (synthetic) -------------------------------------------------------------------------------------------

def cmd_check(cfg: Config, args: argparse.Namespace, log: logging.Logger) -> int:
    gt = json.loads((cfg.root / "data" / "synthetic" / f"{args.case}_gt.json").read_text())
    plan = json.loads((trace_dir(cfg.root) / f"{args.case}_trace_plan.json").read_text())
    curve = np.array(gt["curve_points_base_m"])
    worst = 0.0
    for s in plan["segments"]:
        p = np.array(s["surface_points_m"])
        d = np.linalg.norm(p[:, None, :] - curve[None, :, :], axis=2).min(axis=1)
        tips = np.array(s["tool_points_m"])
        h = (tips - p) @ np.array(gt["normal_base"])
        worst = max(worst, float(d.max()))
        print(f"segment ({s['length_m'] * 1000:.0f} mm): planned surface path vs ground truth crack: mean "
              f"{d.mean() * 1000:.2f} mm, max {d.max() * 1000:.2f} mm; hover {h.min() * 100:.2f}..{h.max() * 100:.2f} cm")
    ok = worst <= args.tol_m
    print(f"{'PASS' if ok else 'FAIL'}: worst deviation {worst * 1000:.2f} mm (tolerance {args.tol_m * 1000:.1f} mm)")
    return EXIT_OK if ok else EXIT_RUNTIME


# ---- make-replay (offline testing) ------------------------------------------------------------------------------

def cmd_make_replay(cfg: Config, args: argparse.Namespace, log: logging.Logger) -> int:
    """A D405-format recording of what the camera sees at the scan pose when a photo, printed --print-width-m wide,
    lies flat on the surface at the scan target (ray/plane intersection; exact aligned depth). Feeds `capture
    --replay` with a scene that has a real crack in it, so the whole pipeline runs without the arm or the camera."""
    from crackvision import camera_pipeline as cp
    from crackvision.recording import CaptureFrame
    scan = json.loads(Path(args.scan).read_text())["scan"]
    info = json.loads((Path(args.like) / "session.json").read_text())
    ci = info["color_intrinsics"]
    W, H = int(ci["width"]), int(ci["height"])
    ik = ArmIK()
    _, T_opt = camera_frames(kinematics.load_end_effector())
    T = ik.fk_frame(scan["q_rad"], T_opt)
    target, n = np.array(scan["target_m"]), np.array(scan["normal"]) / np.linalg.norm(scan["normal"])
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays = np.stack([(u - ci["ppx"]) / ci["fx"], (v - ci["ppy"]) / ci["fy"], np.ones_like(u, float)], -1)
    rays_b = rays @ T[:3, :3].T
    lam = ((target - T[:3, 3]) @ n) / (rays_b @ n)                 # depth along optical z (ray z component = 1)
    pts = T[:3, 3] + rays_b * lam[..., None]
    # photo axes on the surface: x_img along base +y (left), y_img along base -x (toward the base) for a table
    e1 = np.cross(n, [1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.cross(n, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    photo = np.array(Image.open(args.image).convert("RGB"))
    ph, pw = photo.shape[:2]
    scale = pw / args.print_width_m                                 # photo px per metre
    a = ((pts - target) @ e1) * scale + pw / 2
    b = ((pts - target) @ e2) * scale + ph / 2
    inside = (a >= 0) & (a < pw - 1) & (b >= 0) & (b < ph - 1) & (lam > 0)
    rgb = np.full((H, W, 3), 205, np.uint8)
    rgb[inside] = photo[b[inside].astype(int), a[inside].astype(int)]
    depth = np.where(lam > 0, lam / float(info["depth_scale_m_per_unit"]), 0).round().astype(np.uint16)
    frames = [CaptureFrame(frame_index=k, color_rgb=rgb, depth_u16=depth, color_timestamp=1000.0 * k / 30,
                           color_timestamp_domain="system_time", depth_timestamp=1000.0 * k / 30,
                           depth_timestamp_domain="system_time", color_frame_number=k, depth_frame_number=k,
                           host_monotonic_ns=time.monotonic_ns(), extra={"rendered_from": str(args.image)})
             for k in range(args.frames)]
    info = dict(info, device=dict(info.get("device", {}), serial="rendered", name="crack_trace make-replay"))
    out = cp.write_recording(info, frames, Path(args.out))
    print(f"rendered {args.frames} frames ({100 * inside.mean():.0f} % of the view is the print, depth "
          f"{lam[inside].min():.3f}..{lam[inside].max():.3f} m) -> {out}")
    return EXIT_OK


# ---- CLI ---------------------------------------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="crack_trace", description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    sub = p.add_subparsers(dest="cmd", required=True)
    sp = sub.add_parser("scan-plan", help="scan pose trajectory (camera looking at the target)")
    sp.add_argument("--target", nargs=3, type=float, default=[0.30, 0.0, 0.0], metavar=("X", "Y", "Z"),
                    help="surface point to look at, base_link m (default: the table in front, 30 cm out)")
    sp.add_argument("--normal", nargs=3, type=float, default=[0.0, 0.0, 1.0], metavar=("NX", "NY", "NZ"))
    sp.add_argument("--view-m", type=float, default=0.25, help="camera distance from the surface (D405: 7-50 cm)")
    sp.add_argument("--dwell-s", type=float, default=15.0, help="how long the scan pose is held for the capture")
    sp.add_argument("--out", type=Path)
    cp = sub.add_parser("capture", help="capture at the scan pose, then segment + paths")
    cp.add_argument("--case", required=True)
    cp.add_argument("--scan", type=Path, default=None, help="scan-plan output (default data/crack_trace/scan.json)")
    cp.add_argument("--state-file", required=True, type=Path, help="demo_move.py --state-file")
    cp.add_argument("--timeout-s", type=float, default=600.0)
    cp.add_argument("--burst", type=int, default=10)
    cp.add_argument("--replay", type=Path, default=None, help="a recording instead of the live D405 (testing)")
    cp.add_argument("--serial", default=None)
    for a in ("--width", "--height", "--fps", "--warmup"):
        cp.add_argument(a, type=int, default=None)
    cp.add_argument("--hd", action="store_true")
    pp = sub.add_parser("plan", help="path3d + the trace trajectory + preview")
    pp.add_argument("--case", required=True)
    pp.add_argument("--hover-m", type=float, default=0.05, help="tool tip above the surface (raised as needed)")
    pp.add_argument("--approach-m", type=float, default=0.04, help="extra height for approach / retract")
    pp.add_argument("--speed", type=float, default=0.02, help="tool speed along the crack, m/s")
    pp.add_argument("--transit-speed", type=float, default=0.04)
    pp.add_argument("--max-tilt-deg", type=float, default=30.0)
    pp.add_argument("--min-length-m", type=float, default=0.01)
    pp.add_argument("--max-segments", type=int, default=3)
    pp.add_argument("--all-paths", action="store_true", help="branches too, not only each crack's main path")
    pp.add_argument("--smooth", type=int, default=7, help="moving-average window over the 2 mm samples")
    pp.add_argument("--curved", action="store_true", help="follow the measured surface, no plane fit")
    pp.add_argument("--speed-fraction", type=float, default=0.6)
    pp.add_argument("--skip-path3d", action="store_true", help="reuse data/paths3d/<case>_paths3d.json")
    pp.add_argument("--out", type=Path)
    mr = sub.add_parser("make-replay", help="render a printed crack photo under the scan pose (offline testing)")
    mr.add_argument("--image", required=True, type=Path)
    mr.add_argument("--scan", required=True, type=Path)
    mr.add_argument("--like", required=True, type=Path, help="a real D405 recording: intrinsics and depth scale")
    mr.add_argument("--print-width-m", type=float, default=0.20)
    mr.add_argument("--frames", type=int, default=10)
    mr.add_argument("--out", required=True, type=Path)
    ck = sub.add_parser("check", help="synthetic case: planned path vs ground truth")
    ck.add_argument("--case", required=True)
    ck.add_argument("--tol-m", type=float, default=0.002)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        cfg = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"crack_trace: {exc}", file=sys.stderr)
        return EXIT_USAGE
    log = setup_logging("crack_trace", cfg, verbose=args.verbose, quiet=args.quiet)
    if args.cmd == "capture" and args.scan is None:
        args.scan = trace_dir(cfg.root) / "scan.json"
    return {"scan-plan": cmd_scan_plan, "capture": cmd_capture, "plan": cmd_plan, "check": cmd_check,
            "make-replay": cmd_make_replay}[args.cmd](
        cfg, args, log)


if __name__ == "__main__":
    raise SystemExit(main())

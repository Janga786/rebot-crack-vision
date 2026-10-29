"""crackvision.crack_trace — from the wrist camera seeing a crack to a joint trajectory that traces it.

The B601-DM runner (~/rebot_setup/run-trajectory.sh -> demo_move.py, behind arm_guard) plays
crackvision.joint_trajectory/1 files: approach from the fold, the file, return to the fold. This module makes the two
files the crack pipeline needs and the capture in between (scripts/crack_pipeline.sh sequences them):

  search-plan   a small search before the scan: the camera, looking straight down, visits a grid of stops
                (default 2 x 3, 25 cm up) and rests at each. Writes the sweep trajectory and data/crack_trace/search.json.
  search-capture  while the sweep runs: at every stop waits for the arm to rest, grabs a frame, runs the crack model,
                and keeps the stop with the biggest crack that sits inside the image. The crack's centre becomes a
                3D target (depth + the measured pose) in data/crack_trace/<case>_search/search_result.json.
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

  ./env.sh python -m crackvision.crack_trace search-plan
  ./env.sh python -m crackvision.crack_trace search-capture --case CASE --state-file ~/rebot_setup/runs/arm_state.json
  ./env.sh python -m crackvision.crack_trace scan-plan --out data/crack_trace/scan.json [--target 0.30 0 0 | --from-search CASE]
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
from crackvision.arm_ik import (GRIPPER_FLOOR_Z_M, ArmIK, IKError, Piece, build_trajectory, check_motion_clearance,
                                trajectory_doc)
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
    if args.from_search:
        res_path = trace_dir(cfg.root) / f"{args.from_search}_search" / "search_result.json"
        try:
            res = json.loads(res_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            log.error("no search result %s: %s", res_path, exc)
            return EXIT_PRECONDITION
        if not res.get("found"):
            log.error("the search %s found no crack: nothing to scan", args.from_search)
            return EXIT_PRECONDITION
        target, normal = np.array(res["target_base_m"], float), np.array(res.get("normal_base", [0, 0, 1.0]), float)
        print(f"scan target from the search: crack centre at {target.round(3).tolist()} (stop {res['best_stop']})")
    try:
        q, info = scan_pose(target, normal, args.view_m)
    except IKError as exc:
        log.error("scan pose unreachable: %s", exc)
        return EXIT_PRECONDITION
    if info["gripper_link_z_m"] < TIP_Z_MIN_M:
        log.error("scan pose puts gripper_link at z %.3f m, below %.3f m: raise --view-m", info["gripper_link_z_m"],
                  TIP_Z_MIN_M)
        return EXIT_PRECONDITION
    try:
        clear = check_motion_clearance(ArmIK(), np.vstack([q, q]))
    except IKError as exc:
        log.error("scan pose refused: %s", exc)
        return EXIT_PRECONDITION
    t = np.array([0.0, args.dwell_s])
    doc = trajectory_doc(t, np.vstack([q, q]), producer="crackvision.crack_trace:scan-plan", purpose="test",
                         notes=(f"scan pose: the wrist camera {args.view_m} m from the surface point "
                                f"{target.round(3).tolist()} (normal {normal.round(3).tolist()}), held {args.dwell_s} s "
                                "for crack_trace capture; nominal camera mount (CAD prior)"),
                         extra={"scan": {"schema": SCAN_SCHEMA, "target_m": target.tolist(), "normal": normal.tolist(),
                                         "view_m": args.view_m, "dwell_s": args.dwell_s,
                                         "q_rad": q.round(6).tolist(), "clearance": clear, **info}})
    out = args.out or trace_dir(cfg.root) / "scan.json"
    if args.dry_run:
        log.info("dry-run: would write %s", out)
        return EXIT_OK
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, indent=1) + "\n")
    print(f"clearance over the whole run (fold -> scan pose -> fold): camera/mount at least "
          f"{clear['min_body_z_m'] * 1000:.0f} mm above the table ({clear['lowest_box']}), gripper_link at least "
          f"{clear['min_gripper_z_m'] * 1000:.0f} mm")
    print(f"scan pose q = {np.round(q, 3).tolist()}  camera at {info['camera_position_m']} looking "
          f"{info['optical_axis_base']}; holds {args.dwell_s} s -> {out}")
    return EXIT_OK


# ---- search ------------------------------------------------------------------------------------------------------

SEARCH_SCHEMA = "crackvision.crack_search/1"
BORDER_PX = 24                    # a crack closer than this to the image edge may be cut off by the view


def plan_search(xs: list[float], ys: list[float], view_m: float, dwell_s: float, *, speed_m_s: float = 0.06,
                ramp_s: float = 0.8, speed_fraction: float = 0.6, log: logging.Logger | None = None):
    """Camera stops over the table (looking straight down, view_m up), visited in a serpentine. Stops the arm cannot
    reach (camera pointing down), or that would put the camera/mount near the table, are dropped. Returns
    (t, q (N,6), stops [{q, camera_m, t0, t1}])."""
    from crackvision.arm_ik import peak_speed_fraction, slow_to_fit, time_pieces
    ik = ArmIK()
    T_cl, _ = camera_frames(kinematics.load_end_effector())
    down = np.array([0.0, 0.0, -1.0])
    cams = []
    for i, x in enumerate(xs):
        for y in (ys if i % 2 == 0 else ys[::-1]):
            cams.append(np.array([x, y, view_m]))
    stops, q_prev = [], None
    for c in cams:
        try:
            q = ik.solve_global(c, down, T_tip=T_cl).q if q_prev is None else ik.solve(c, q_prev, down, T_tip=T_cl)
            if q_prev is not None and not (isinstance(q, np.ndarray) or q.ok):
                raise IKError("no solution")
            q = q if isinstance(q, np.ndarray) else q.q
            check_motion_clearance(ik, np.vstack([q, q]))
        except IKError as exc:
            if log:
                log.warning("search stop %s dropped: %s", c.round(3).tolist(), exc)
            continue
        stops.append({"camera_m": c, "q": q})
        q_prev = q
    if len(stops) < 2:
        raise IKError(f"only {len(stops)} reachable search stop(s): move the search area closer to the arm")
    ts, qs = [np.array([0.0])], [stops[0]["q"][None]]
    t_now = 0.0
    dt = 0.05
    stops[0]["t0"] = 0.0
    n_hold = int(round(dwell_s / dt))
    ts.append(t_now + dt * np.arange(1, n_hold + 1)); qs.append(np.repeat(stops[0]["q"][None], n_hold, 0))
    t_now += n_hold * dt
    stops[0]["t1"] = t_now
    for a, b in zip(stops[:-1], stops[1:]):
        pts = a["camera_m"] + (b["camera_m"] - a["camera_m"]) * np.linspace(0, 1, 30)[:, None]
        pc = Piece(pts, np.tile(down, (30, 1)), speed_m_s, True, "leg")
        tl, pl, nl, _ = time_pieces([pc], ramp_s=ramp_s)
        ql = ArmIK.follow(ik, pl, nl, a["q"], max_joint_step_rad=0.15, T_tip=T_cl)
        tl, _f = slow_to_fit(tl, ql, speed_fraction)
        check_motion_clearance(ik, ql)
        b["q"] = ql[-1].copy()          # rest where the leg arrives: the roll about the optical axis is free, so a
                                        # separately solved stop pose differs by a few hundredths of a rad (a jump)
        ts.append(t_now + tl[1:]); qs.append(ql[1:])
        t_now += float(tl[-1])
        b["t0"] = t_now
        ts.append(t_now + dt * np.arange(1, n_hold + 1)); qs.append(np.repeat(b["q"][None], n_hold, 0))
        t_now += n_hold * dt
        b["t1"] = t_now
    return np.concatenate(ts), np.vstack(qs), stops


def cmd_search_plan(cfg: Config, args: argparse.Namespace, log: logging.Logger) -> int:
    xs = [float(v) for v in args.xs]
    ys = [float(v) for v in args.ys]
    try:
        t, q, stops = plan_search(xs, ys, args.view_m, args.dwell_s, log=log)
        clear = check_motion_clearance(ArmIK(), q)
    except IKError as exc:
        log.error("search refused: %s", exc)
        return EXIT_PRECONDITION
    doc = trajectory_doc(
        t, q, producer="crackvision.crack_trace:search-plan", purpose="test",
        notes=(f"search: the wrist camera looks straight down from {args.view_m} m and rests {args.dwell_s} s at each of "
               f"{len(stops)} stops over the table; crack_trace search-capture runs the crack model at every stop"),
        extra={"search": {"schema": SEARCH_SCHEMA, "view_m": args.view_m, "dwell_s": args.dwell_s, "clearance": clear,
                          "stops": [{"index": i, "camera_m": s["camera_m"].round(4).tolist(),
                                     "q_rad": s["q"].round(6).tolist(), "t0_s": round(s["t0"], 3),
                                     "t1_s": round(s["t1"], 3)} for i, s in enumerate(stops)]}})
    out = args.out or trace_dir(cfg.root) / "search.json"
    if args.dry_run:
        log.info("dry-run: would write %s", out)
        return EXIT_OK
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, indent=1) + "\n")
    print(f"search: {len(stops)} stops, camera {args.view_m} m up, {t[-1]:.0f} s; camera/mount >= "
          f"{clear['min_body_z_m'] * 1000:.0f} mm above the table over the whole run (incl. approach and return)\n"
          + "\n".join(f"  stop {i}: camera at {np.round(s['camera_m'], 3).tolist()}" for i, s in enumerate(stops))
          + f"\n  -> {out}")
    return EXIT_OK


def locate_crack(mask: np.ndarray, depth_u16: np.ndarray, depth_scale: float, intr: dict, T_base_opt: np.ndarray,
                 *, min_px: int = 150, component: np.ndarray | None = None,
                 valid_range_m: tuple[float, float] | None = None) -> dict | None:
    """Where the crack is, in base_link: its pixel centroid lifted with the surface depth around it (valid depth in
    a ring 4-14 px outside it and outside the whole mask, median) and the measured pose. With `component` (the best
    component crack_filter accepted) only that component is targeted and min_px does not apply (the filter's gates
    did); without it, the whole mask (the pre-filter behaviour). None if too small, no depth, or the surface depth
    is outside valid_range_m."""
    from scipy import ndimage
    m = (mask > 0) if component is None else (np.asarray(component) > 0)
    if component is None and m.sum() < min_px:
        return None
    if not m.any():
        return None
    ring = ndimage.binary_dilation(m, iterations=14) & ~ndimage.binary_dilation(m, iterations=4) & ~(mask > 0)
    d = depth_u16[ring & (depth_u16 > 0)].astype(float) * depth_scale
    if len(d) < 50:
        return None
    z = float(np.median(d))
    if valid_range_m is not None and not valid_range_m[0] <= z <= valid_range_m[1]:
        return None
    ys, xs = np.nonzero(m)
    u, v = xs.mean(), ys.mean()
    p_opt = np.array([(u - intr["ppx"]) / intr["fx"] * z, (v - intr["ppy"]) / intr["fy"] * z, z, 1.0])
    p = (T_base_opt @ p_opt)[:3]
    ext = []
    for uu, vv in ((xs.min(), ys.min()), (xs.max(), ys.max())):
        q_ = np.array([(uu - intr["ppx"]) / intr["fx"] * z, (vv - intr["ppy"]) / intr["fy"] * z, z, 1.0])
        ext.append((T_base_opt @ q_)[:3])
    h, w = m.shape
    touches = bool(xs.min() < BORDER_PX or ys.min() < BORDER_PX or xs.max() > w - BORDER_PX or ys.max() > h - BORDER_PX)
    return {"crack_px": int(m.sum()), "centroid_px": [round(float(u), 1), round(float(v), 1)], "depth_m": round(z, 4),
            "target_base_m": p.round(4).tolist(), "touches_border": touches,
            "extent_base_m": round(float(np.linalg.norm(ext[1] - ext[0])), 3)}


def _stop_source(args, cam, stop_index: int):
    """Frames for one stop: the live camera, or (testing) the stop's own rendered recording."""
    if getattr(args, "replay", None):
        from crackvision.camera_pipeline import _ReplayCamera
        return _ReplayCamera(Path(args.replay) / f"stop_{stop_index:02d}").frames(args.burst)
    return cam.frames(args.burst)


def cmd_search_capture(cfg: Config, args: argparse.Namespace, log: logging.Logger) -> int:
    plan = json.loads(Path(args.search).read_text())["search"]
    stops = plan["stops"]
    case = naming.derive_case_id(Path(f"{args.case}.png"))
    out = trace_dir(cfg.root) / f"{case}_search"
    if args.dry_run:
        log.info("dry-run: would search %d stops into %s", len(stops), out)
        return EXIT_OK
    from crackvision import camera_pipeline as cp
    from crackvision.d405_live import D405Error
    from crackvision.live_predictor import LivePredictor
    from crackvision.recording import _write_depth_png
    cam_ctx = cam = None
    if not args.replay:
        try:
            cam_ctx = cp.open_camera(cfg, args, log)
            cam = cam_ctx.__enter__()
        except D405Error as exc:
            log.error("camera: %s", exc)
            return EXIT_PRECONDITION
    from crackvision import crack_filter as cf
    use_filter = filter_enabled(cfg, args)
    params = cf.FilterParams.from_config(cfg)
    predictor = LivePredictor(cfg, tta=args.tta or cf.config_flag(cfg, "tta", False))
    warm_up(cfg, predictor, use_filter)
    out.mkdir(parents=True, exist_ok=True)
    ik = ArmIK()
    _, T_opt = camera_frames(kinematics.load_end_effector())
    results = []
    try:
        for st in stops:
            q_stop = np.array(st["q_rad"])
            log.info("stop %d/%d: waiting for the arm", st["index"] + 1, len(stops))
            try:
                wait_settled(Path(args.state_file), q_stop, args.timeout_s, tol_rad=0.05, hold_s=0.5, log=None)
            except TimeoutError as exc:
                log.error("stop %d skipped: %s", st["index"], exc)
                if "left the scan pose" in str(exc):
                    break
                continue
            s0 = read_state(Path(args.state_file))
            frames = list(_stop_source(args, cam, st["index"]))
            s1 = read_state(Path(args.state_file))
            if not frames or s0 is None or s1 is None or not settled([s0, s1], q_stop, 0.05, 0.05):
                log.error("stop %d skipped: the arm moved during the capture", st["index"])
                continue
            tail = frames[len(frames) // 2:]
            info = cam.session_info if cam is not None else json.loads((Path(args.replay) / "stop_00" / "session.json").read_text())
            T = ik.fk_frame(s1["positions_rad"], T_opt)
            tag = f"stop_{st['index']:02d}"
            ds, intr = float(info["depth_scale_m_per_unit"]), info["color_intrinsics"]
            if use_filter:      # burst vote + crack_filter; target the best accepted component, not the whole mask
                scores = [sharpness(fr.color_rgb) for fr in tail]
                k, prob, vm, binfo = cf.burst_probabilities(predictor, [fr.color_rgb for fr in tail], scores, params)
                f = tail[k]
                res = cf.analyse(prob, params, gray=cf.to_gray(f.color_rgb), depth_u16=f.depth_u16, depth_scale=ds,
                                 intrinsics=intr, vote_mask=vm)
                mask = res.accepted_mask.astype(np.uint8)
                best = res.best
                loc = (locate_crack(mask, f.depth_u16, ds, intr, T, component=res.component_mask(best["id"]),
                                    valid_range_m=params.working_range_m) if best else None)
                if loc:
                    loc |= {"component": best["id"], "score": best["score"], "length_mm": best["length_mm"],
                            "width_mm": best["width_mm"], "mean_p": best["mean_p"],
                            "touches_border": loc["touches_border"]}
                (out / f"{tag}_components.json").write_text(json.dumps(
                    cf.to_doc(res, params, case_id=f"{case}_{tag}", extra={"burst": binfo}), indent=1) + "\n")
                ov = cf.overlay(f.color_rgb, res)
                n_raw = res.summary["argmax_px"]
            else:
                f = max(tail, key=lambda fr: sharpness(fr.color_rgb))
                mask = predictor.predict(f.color_rgb)
                loc = locate_crack(mask, f.depth_u16, ds, intr, T, min_px=args.min_px)
                ov = f.color_rgb.copy()
                ov[mask > 0] = (255, 0, 0)
                res, n_raw = None, int(mask.sum())
            Image.fromarray(f.color_rgb).save(out / f"{tag}_color.png")
            _write_depth_png(f.depth_u16, out / f"{tag}_depth.png")
            Image.fromarray(ov).save(out / f"{tag}_overlay.png")
            r = {"stop": st["index"], "camera_m": st["camera_m"], "state": s1, "crack_px": int(mask.sum()),
                 "located": loc}
            if res is not None:
                r["crack_filter"] = res.summary
            results.append(r)
            if loc and res is not None:
                what = (f"crack #{loc['component']} ({loc['length_mm']:.0f} mm, p {loc['mean_p']:.2f}, score "
                        f"{loc['score']:.0f}; {res.summary['accepted_components']}/"
                        f"{res.summary['candidate_components']} accepted) at {loc['target_base_m']}")
            elif loc:
                what = f"crack {loc['crack_px']} px at {loc['target_base_m']}"
            elif res is not None:
                why = sorted({x for c in res.components for x in c["reject_reasons"]})
                what = (f"no crack accepted ({res.summary['candidate_components']} candidate(s), {n_raw} raw px"
                        + (f"; rejected: {', '.join(why)}" if why else "") + ")")
            else:
                what = f"no crack ({n_raw} px)"
            print(f"stop {st['index']}: camera {np.round(st['camera_m'], 3).tolist()}: " + what
                  + (" (touches the image edge)" if loc and loc["touches_border"] else ""))
    finally:
        if cam_ctx is not None:
            cam_ctx.__exit__(None, None, None)
    ok = [r for r in results if r["located"]]
    rank = ((lambda r: (not r["located"]["touches_border"], r["located"]["score"])) if use_filter else
            (lambda r: (not r["located"]["touches_border"], r["located"]["crack_px"])))
    best = max(ok, key=rank, default=None)
    res = {"schema": SEARCH_SCHEMA, "case": case, "stops_planned": len(stops), "stops_done": len(results),
           "found": best is not None, "results": [{k: v for k, v in r.items() if k != "state"} for r in results]}
    if best:
        res |= {"best_stop": best["stop"], "target_base_m": best["located"]["target_base_m"],
                "normal_base": [0.0, 0.0, 1.0], "crack_px": best["located"]["crack_px"],
                "extent_base_m": best["located"]["extent_base_m"],
                "touches_border": best["located"]["touches_border"],
                "crack_filter": use_filter,
                **({k: best["located"][k] for k in ("component", "score", "length_mm", "width_mm", "mean_p")}
                   if use_filter else {}),
                "note": ("target = centroid of the best accepted crack component (crack_filter), lifted with the "
                         "surface depth around it and the measured pose; stops ranked by that component's score"
                         if use_filter else "target = crack centroid lifted with the surface depth around it and the "
                         "measured pose") + "; the surface is assumed horizontal (table)"}
    (out / "search_result.json").write_text(json.dumps(res, indent=2) + "\n")
    if not best:
        print(f"search: no crack found at {len(results)}/{len(stops)} stops")
        return EXIT_PRECONDITION
    print(f"search: crack found at stop {best['stop']}, centre {res['target_base_m']} (extent {res['extent_base_m']} m); "
          f"next: scan-plan --from-search {case}")
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
    from crackvision import crack_filter as cf
    use_filter = filter_enabled(cfg, args)
    params = cf.FilterParams.from_config(cfg)
    predictor = None
    if use_filter:                         # load the network before the arm arrives
        from crackvision.live_predictor import LivePredictor
        predictor = LivePredictor(cfg, tta=args.tta or cf.config_flag(cfg, "tta", False))
        warm_up(cfg, predictor, True)
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
    if use_filter:
        k, prob, res, binfo = cp.filter_burst(predictor, frames, scores, info, params)
        rc = cp.filtered_perception(cfg, case_id, src, res, prob, params, quiet=args.quiet,
                                    extra={"source": "crack_trace capture", "burst": binfo,
                                           "frame_index": int(frames[k].frame_index)})
        cp.print_components(res, indent="")
    else:
        rc = run_perception(cfg, case_id, src, quiet=args.quiet)
    if rc != EXIT_OK:
        log.error("segment/paths failed for %s (exit %d)", case_id, rc)
        return rc
    paths = json.loads((cfg.root / "data" / "paths" / f"{case_id}_paths.json").read_text())
    n = len(paths.get("components", []))
    print(f"perception: {n} crack component(s); overlay data/paths/{case_id}_paths_overlay.png"
          + ("" if n else "\n  no crack found in the view: nothing to trace"))
    return EXIT_OK if n else EXIT_PRECONDITION


# ---- crack selection -------------------------------------------------------------------------------------------

def filter_enabled(cfg: Config, args: argparse.Namespace) -> bool:
    """crack_filter on the crack_trace path: config crack_filter.crack_trace (default on) unless --no-filter."""
    from crackvision.crack_filter import config_flag
    return config_flag(cfg, "crack_trace", True) and not getattr(args, "no_filter", False)


def warm_up(cfg: Config, predictor, probs: bool) -> None:
    """One pass on a blank frame of the configured size while the arm is still on its way: the first CUDA pass
    costs about 1.3 s, later ones 55 ms."""
    c = cfg.realsense.get("color", {})
    blank = np.zeros((int(c.get("height", 480)), int(c.get("width", 848)), 3), np.uint8)
    (predictor.predict_probs if probs else predictor.predict)(blank)


def select_crack(root: Path, case_id: str) -> dict | None:
    """The best accepted crack of a case, matched to its paths.json component: {paths_component, filter_component,
    score, ...}; None when the case has no crack_filter table (pre-filter cases, synthetic cases)."""
    from crackvision import crack_filter as cf
    table, labels = cf.find_case_table(root, case_id)
    pj = Path(root) / "data" / "paths" / f"{case_id}_paths.json"
    if table is None or labels is None or not pj.is_file():
        return None
    rows = {c["id"]: c for c in table.get("components", [])}
    best = None
    for comp in json.loads(pj.read_text()).get("components", []):
        pts = comp["main_path"]["dense"]
        ids = [int(labels[p["row"], p["col"]]) for p in pts]
        ids = [i for i in ids if i > 0]
        if not ids:
            continue
        fid = max(set(ids), key=ids.count)
        row = rows.get(fid)
        if row is None or not row.get("accepted"):
            continue
        cand = {"paths_component": comp["component_id"], "filter_component": fid, "score": row["score"],
                "score_units": row.get("score_units"), "length_mm": row.get("length_mm"),
                "width_mm": row.get("width_mm"), "mean_p": row["mean_p"]}
        if best is None or cand["score"] > best["score"]:
            best = cand
    return best


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


def crack_segments(doc: dict, min_length_m: float, all_paths: bool, component: int | None = None) -> list[dict]:
    """Trace segments from a paths3d document: surface points + normals of each segment's trace waypoints (only
    component `component` when given)."""
    out = []
    for comp in doc.get("components", []):
        if component is not None and comp["component_id"] != component:
            continue
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
               flat: bool, speed_fraction: float, log: logging.Logger,
               component: int | None = None) -> tuple[np.ndarray, np.ndarray, dict]:
    segs = crack_segments(doc, min_length_m, all_paths, component)[:max_segments]
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
    clear = check_motion_clearance(ik, q)
    return t, q, {"segments": planned, "report": asdict(rep), "clearance": clear, "hover_floor_3sigma_m": round(3 * sigma, 4)}


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
    from crackvision import crack_filter as cf
    table, labels = cf.find_case_table(cfg.root, case_id)
    rows = (table or {}).get("components", [])
    if labels is not None and labels.shape == base.shape[:2]:
        rej = np.isin(labels, [c["id"] for c in rows if not c["accepted"]])
        base[rej] = (0.45 * base[rej] + 0.55 * np.array([255, 220, 0])).astype(np.uint8)
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
    if table is not None:
        s = table.get("summary", {})
        acc = [c for c in rows if c["accepted"]]
        d.text((8, 22), f"crack filter: {len(acc)} accepted (red, numbered), {len(rows) - len(acc)} rejected (yellow); "
                        f"depth {s.get('depth')}", fill=(255, 255, 0))
        for c in acc:
            x, y = c["centroid_px"]
            d.text((x + 6, y + 6), f"#{c['id']}", fill=(255, 255, 255), stroke_width=2, stroke_fill=(0, 0, 0))
        h = base.shape[0]
        for i, c in enumerate(sorted(acc, key=lambda c: -c["score"])[:6]):
            d.text((8, h - 14 * (i + 1) - 4), cf.describe(c), fill=(255, 255, 255), stroke_width=2,
                   stroke_fill=(0, 0, 0))
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
    max_segments = args.max_segments if args.max_segments is not None else (3 if args.all_paths else 1)
    selection = None if args.all_paths else select_crack(cfg.root, case_id)
    try:
        t, q, plan = plan_trace(doc, hover_m=args.hover_m, approach_m=args.approach_m, trace_speed=args.speed,
                                transit_speed=args.transit_speed, max_tilt_deg=args.max_tilt_deg,
                                min_length_m=args.min_length_m, all_paths=args.all_paths,
                                max_segments=max_segments, smooth=args.smooth, flat=not args.curved,
                                speed_fraction=args.speed_fraction, log=log,
                                component=selection["paths_component"] if selection else None)
    except IKError as exc:
        log.error("no trace: %s", exc)
        return EXIT_PRECONDITION
    plan["selection"] = ({"mode": "all_paths", "max_segments": max_segments} if args.all_paths else
                         {"mode": "best_crack", "max_segments": max_segments, **selection} if selection else
                         {"mode": "longest_segment", "max_segments": max_segments,
                          "note": "no crack_filter table for this case, or no accepted component matched a path"})
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
    sel = plan["selection"]
    if sel["mode"] == "best_crack":
        print(f"crack selection: the best accepted crack, filter component #{sel['filter_component']} (score "
              f"{sel['score']:.0f}, {sel['length_mm']} mm, p {sel['mean_p']:.2f}); --all-paths traces the others too")
    print(f"trace for {case_id}: {len(plan['segments'])} segment(s) ("
          + "; ".join(f"{s['length_m'] * 1000:.0f} mm, hover {s['hover_m'] * 100:.1f} cm, tilt {s['tilt_deg']:.0f} deg"
                      for s in plan["segments"])
          + f"), {r['duration_s']:.1f} s, joint peak/cap {max(r['peak_speed_fraction_of_cap']):.2f}, "
          f"min gripper z {r['min_gripper_z_m']:.3f} m, camera/mount >= {plan['clearance']['min_body_z_m'] * 1000:.0f} mm "
          f"above the table (whole run incl. approach and return)\n  trajectory {out}\n  preview    {prev}")
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

def render_view(photo: np.ndarray, q, ci: dict, depth_scale: float, center: np.ndarray, normal: np.ndarray,
                print_width_m: float, ik: ArmIK | None = None) -> tuple[np.ndarray, np.ndarray, float]:
    """What the wrist camera sees at joint pose q of a photo printed print_width_m wide lying on the plane through
    `center` with `normal` (ray/plane intersection; exact aligned depth, the plane everywhere). Returns
    (rgb, depth_u16, fraction of the view covered by the print)."""
    ik = ik or ArmIK()
    W, H = int(ci["width"]), int(ci["height"])
    _, T_opt = camera_frames(kinematics.load_end_effector())
    T = ik.fk_frame(q, T_opt)
    n = normal / np.linalg.norm(normal)
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays = np.stack([(u - ci["ppx"]) / ci["fx"], (v - ci["ppy"]) / ci["fy"], np.ones_like(u, float)], -1)
    rays_b = rays @ T[:3, :3].T
    with np.errstate(divide="ignore", invalid="ignore"):
        lam = ((center - T[:3, 3]) @ n) / (rays_b @ n)             # depth along optical z (ray z component = 1)
    pts = T[:3, 3] + rays_b * lam[..., None]
    e1 = np.cross(n, [1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.cross(n, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    ph, pw = photo.shape[:2]
    scale = pw / print_width_m
    a = ((pts - center) @ e1) * scale + pw / 2
    b = ((pts - center) @ e2) * scale + ph / 2
    inside = (a >= 0) & (a < pw - 1) & (b >= 0) & (b < ph - 1) & (lam > 0)
    rgb = np.full((H, W, 3), 205, np.uint8)
    rgb[inside] = photo[b[inside].astype(int), a[inside].astype(int)]
    depth = np.where(lam > 0, lam / depth_scale, 0).round().astype(np.uint16)
    return rgb, depth, float(inside.mean())


def cmd_make_replay(cfg: Config, args: argparse.Namespace, log: logging.Logger) -> int:
    """D405-format recordings of what the camera sees when a photo, printed --print-width-m wide, lies flat on the
    table: at the scan pose (--scan), or at every stop of a search (--search, one stop_NN recording each, the print
    centred at --print-center). Feeds `capture --replay` / `search-capture --replay`: the whole pipeline runs without
    the arm or the camera."""
    from crackvision import camera_pipeline as cp
    from crackvision.recording import CaptureFrame
    info = json.loads((Path(args.like) / "session.json").read_text())
    ci = info["color_intrinsics"]
    ds = float(info["depth_scale_m_per_unit"])
    photo = np.array(Image.open(args.image).convert("RGB"))
    ik = ArmIK()
    info = dict(info, device=dict(info.get("device", {}), serial="rendered", name="crack_trace make-replay"))

    def write(rgb, depth, out: Path) -> Path:
        frames = [CaptureFrame(frame_index=k, color_rgb=rgb, depth_u16=depth, color_timestamp=1000.0 * k / 30,
                               color_timestamp_domain="system_time", depth_timestamp=1000.0 * k / 30,
                               depth_timestamp_domain="system_time", color_frame_number=k, depth_frame_number=k,
                               host_monotonic_ns=time.monotonic_ns(), extra={"rendered_from": str(args.image)})
                  for k in range(args.frames)]
        return cp.write_recording(info, frames, out)

    if args.search:
        stops = json.loads(Path(args.search).read_text())["search"]["stops"]
        center = np.array(args.print_center or [0.30, 0.0, 0.0], float)
        for st in stops:
            rgb, depth, cov = render_view(photo, st["q_rad"], ci, ds, center, np.array([0, 0, 1.0]), args.print_width_m, ik)
            write(rgb, depth, Path(args.out) / f"stop_{st['index']:02d}")
            print(f"stop {st['index']}: camera {st['camera_m']}: {100 * cov:.0f} % of the view is the print")
        print(f"rendered {len(stops)} stops, print centred at {center.tolist()} -> {args.out}")
        return EXIT_OK
    scan = json.loads(Path(args.scan).read_text())["scan"]
    center = np.array(args.print_center if args.print_center else scan["target_m"], float)
    rgb, depth, cov = render_view(photo, scan["q_rad"], ci, ds, center, np.array(scan["normal"]),
                                  args.print_width_m, ik)
    out = write(rgb, depth, Path(args.out))
    print(f"rendered {args.frames} frames ({100 * cov:.0f} % of the view is the print) -> {out}")
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
    sp.add_argument("--from-search", metavar="CASE", default=None,
                    help="centre the scan on the crack found by search-capture CASE (overrides --target)")
    sp.add_argument("--normal", nargs=3, type=float, default=[0.0, 0.0, 1.0], metavar=("NX", "NY", "NZ"))
    sp.add_argument("--view-m", type=float, default=0.25, help="camera distance from the surface (D405: 7-50 cm)")
    sp.add_argument("--dwell-s", type=float, default=15.0, help="how long the scan pose is held for the capture")
    sp.add_argument("--out", type=Path)
    sr = sub.add_parser("search-plan", help="sweep trajectory: the camera looks down at a grid of stops")
    sr.add_argument("--xs", nargs="+", type=float, default=[0.26, 0.36], help="stop rows, base x (m)")
    sr.add_argument("--ys", nargs="+", type=float, default=[-0.15, 0.0, 0.15], help="stops per row, base y (m)")
    sr.add_argument("--view-m", type=float, default=0.25)
    sr.add_argument("--dwell-s", type=float, default=3.0, help="rest at each stop")
    sr.add_argument("--out", type=Path)
    sc = sub.add_parser("search-capture", help="at every stop of the sweep: capture + model; locate the crack")
    sc.add_argument("--case", required=True)
    sc.add_argument("--search", type=Path, default=None, help="search-plan output (default data/crack_trace/search.json)")
    sc.add_argument("--state-file", required=True, type=Path)
    sc.add_argument("--timeout-s", type=float, default=300.0, help="per stop")
    sc.add_argument("--burst", type=int, default=8)
    sc.add_argument("--min-px", type=int, default=150, help="smallest crack (pixels) that counts (only with --no-filter)")
    sc.add_argument("--no-filter", action="store_true", help="raw argmax mask, whole-mask centroid, stops ranked by "
                                                              "pixel count (the pre-2026-10-28 behaviour)")
    sc.add_argument("--tta", action="store_true", help="mirroring TTA (slower)")
    sc.add_argument("--replay", type=Path, default=None, help="dir of stop_NN recordings (make-replay --search)")
    sc.add_argument("--serial", default=None)
    for a in ("--width", "--height", "--fps", "--warmup"):
        sc.add_argument(a, type=int, default=None)
    sc.add_argument("--hd", action="store_true")
    cp = sub.add_parser("capture", help="capture at the scan pose, then segment + paths")
    cp.add_argument("--case", required=True)
    cp.add_argument("--scan", type=Path, default=None, help="scan-plan output (default data/crack_trace/scan.json)")
    cp.add_argument("--state-file", required=True, type=Path, help="demo_move.py --state-file")
    cp.add_argument("--timeout-s", type=float, default=600.0)
    cp.add_argument("--burst", type=int, default=10)
    cp.add_argument("--no-filter", action="store_true", help="batch segment stage on the raw argmax mask (the "
                                                              "pre-2026-10-28 behaviour)")
    cp.add_argument("--tta", action="store_true", help="mirroring TTA for the in-process model (default: config "
                                                        "crack_filter.tta)")
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
    pp.add_argument("--max-segments", type=int, default=None,
                    help="default 1 (the best crack's longest main-path segment); 3 with --all-paths")
    pp.add_argument("--all-paths", action="store_true", help="every crack in the mask and its branches, not only the "
                                                             "best crack's main path")
    pp.add_argument("--smooth", type=int, default=7, help="moving-average window over the 2 mm samples")
    pp.add_argument("--curved", action="store_true", help="follow the measured surface, no plane fit")
    pp.add_argument("--speed-fraction", type=float, default=0.6)
    pp.add_argument("--skip-path3d", action="store_true", help="reuse data/paths3d/<case>_paths3d.json")
    pp.add_argument("--out", type=Path)
    mr = sub.add_parser("make-replay", help="render a printed crack photo under the scan pose (offline testing)")
    mr.add_argument("--image", required=True, type=Path)
    mr.add_argument("--scan", type=Path, default=None, help="render the view at this scan pose")
    mr.add_argument("--search", type=Path, default=None, help="or: one recording per stop of this search plan")
    mr.add_argument("--print-center", nargs=3, type=float, default=None, metavar=("X", "Y", "Z"),
                    help="where the print lies on the table (default: the scan target; with --search 0.30 0 0)")
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
    if args.cmd == "search-capture" and args.search is None:
        args.search = trace_dir(cfg.root) / "search.json"
    return {"scan-plan": cmd_scan_plan, "search-plan": cmd_search_plan, "search-capture": cmd_search_capture,
            "capture": cmd_capture, "plan": cmd_plan, "check": cmd_check,
            "make-replay": cmd_make_replay}[args.cmd](
        cfg, args, log)


if __name__ == "__main__":
    raise SystemExit(main())

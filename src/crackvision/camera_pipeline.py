"""crackvision.camera_pipeline — the physical D405 camera, end to end: connect, record, run the model.

    ./env.sh python scripts/camera.py check              # CAM-01: discovery + 30-frame recording + validation
    ./env.sh python scripts/camera.py record --frames 60 # a validated lossless RGB-D recording (CAM-04 sessions)
    ./env.sh python scripts/camera.py snap               # capture, sharpest frame, full crack pipeline, overlays
    ./env.sh python scripts/camera.py live               # model on every frame, viewed in a browser

`check` writes evidence/camera/d405_discovery.json (tracked in git): the device's serial, firmware
(and the SDK's recommended firmware), product line and USB link; its stream profiles and depth scale;
a 30-frame recording under data/recordings/ and `scripts/validate_recording.py`'s verdict on it.
A USB 2 link fails the check. Firmware is compared, never updated: that is an operator decision.

`snap` captures a short burst after auto-exposure warm-up, keeps the sharpest frame (variance of the
Laplacian), stores it as its own case under data/camera/<case_id>/ and runs the accepted single-case
pipeline (`scripts/operator_segment_paths.py --stage both`: prepare_inputs, nnU-Net inference,
skeleton, paths, overlays). With depth it also reports the working distance and the image scale
(mm per pixel), so crack path lengths come out in millimetres.

`live` keeps the network loaded (`crackvision.live_predictor`) and streams the camera with the crack
mask overlaid as MJPEG on http://127.0.0.1:8090 (this env's OpenCV is headless, so the browser is the
display). /snapshot saves the current frame; run `snap --image` on it for the full pipeline.

Sources: the live camera by default; `--replay DIR` (a crackvision.recording session) and, for
`snap`, `--image PNG` run the same code with no camera.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import logging
import platform
import re
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Iterator

import numpy as np
from PIL import Image

from crackvision import naming
from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.d405_live import (HD, LATEST_D405_FIRMWARE, MIN_GOOD_FIRMWARE, USB_HINT, D405Error, LiveD405Source,
                                   firmware_tuple, list_devices, supported_profiles, usb_is_3)
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME, EXIT_USAGE, RunSummary, setup_logging, utc_stamp
from crackvision.recording import CaptureFrame, RecordingWriter, ReplaySource, _write_depth_png

TOOL = "camera_pipeline"
REPO_ROOT = Path(__file__).resolve().parents[2]
EVIDENCE_SCHEMA = "crackvision.d405_discovery/1"
SNAP_SCHEMA = "crackvision.camera_snap/1"
MIN_CRACK_DEPTH_PX = 50        # fewer crack pixels with depth than this: measure the distance over the whole frame
D405_MAX_GOOD_Z_M = 0.5        # the D405's ideal range is 7-50 cm
CHECK_FRAMES = 30
DEFAULT_PORT = 8090


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _rel(path: Path, root: Path) -> str:
    try:
        return Path(path).resolve().relative_to(root).as_posix()
    except ValueError:
        return Path(path).resolve().as_posix()


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load_script(name: str):
    """Import scripts/<name>.py (the accepted validators/wrappers live there, not in the package)."""
    key = f"crackvision._script_{name}"
    if key in sys.modules:
        return sys.modules[key]
    spec = importlib.util.spec_from_file_location(key, REPO_ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[key] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def sharpness(rgb: np.ndarray) -> float:
    """Variance of the 4-neighbour Laplacian of the grey image: higher = sharper (less motion blur)."""
    g = rgb.astype(np.float32) @ np.array([0.299, 0.587, 0.114], dtype=np.float32)
    lap = -4.0 * g[1:-1, 1:-1] + g[:-2, 1:-1] + g[2:, 1:-1] + g[1:-1, :-2] + g[1:-1, 2:]
    return float(lap.var())


def safe_case_id(name: str) -> str:
    return naming.derive_case_id(Path(f"{name}.png"))


# ---- sources ---------------------------------------------------------------------------------------------

def _stream_args(cfg: Config, args: argparse.Namespace) -> tuple[int, int, int, int]:
    rs_cfg = cfg.realsense
    width, height = (HD if args.hd else (args.width or rs_cfg["color"]["width"], args.height or rs_cfg["color"]["height"]))
    fps = args.fps or rs_cfg["color"]["fps"]
    warmup = rs_cfg.get("warmup_frames", 30) if args.warmup is None else args.warmup
    return int(width), int(height), int(fps), int(warmup)


class _ReplayCamera:
    """ReplaySource behind the LiveD405Source interface (session_info + frames(n)); loops if asked."""

    def __init__(self, root: Path, loop: bool = False) -> None:
        self.src = ReplaySource(root)
        self.session_info = dict(self.src.session_info)
        self.loop = loop

    def __enter__(self) -> "_ReplayCamera":
        return self

    def __exit__(self, *exc: object) -> None:
        pass

    def frames(self, n: int | None = None) -> Iterator[CaptureFrame]:
        produced = 0
        while True:
            for frame in self.src:
                if n is not None and produced >= n:
                    return
                produced += 1
                yield frame
            if not self.loop or len(self.src) == 0:
                return


def open_camera(cfg: Config, args: argparse.Namespace, logger: logging.Logger, loop: bool = False):
    if getattr(args, "replay", None):
        return _ReplayCamera(Path(args.replay), loop=loop)
    width, height, fps, warmup = _stream_args(cfg, args)
    return LiveD405Source(width, height, fps, warmup=warmup, serial=args.serial, logger=logger)


RECORD_PNG_LEVEL = 1   # lossless at every level; 1 writes several times faster than PIL's default 6
RECORD_WRITE_WORKERS = 8   # PNG encoding threads when writing a captured recording


def _mem_available_bytes() -> int | None:
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) * 1024
    except OSError:
        pass
    return None


def check_memory(n: int, width: int, height: int) -> str:
    """'' if n frames fit comfortably in RAM (half of what is available), else why not."""
    need = n * width * height * 5            # RGB8 + Z16 per frame
    avail = _mem_available_bytes()
    if avail is not None and need > avail // 2:
        return (f"{n} frames at {width}x{height} need {need / 1e9:.1f} GB of RAM, more than half of the "
                f"{avail / 1e9:.1f} GB available: record fewer frames")
    return ""


def capture_frames(cam, n: int) -> tuple[list[CaptureFrame], float]:
    """Read n frames into memory at the camera's full rate (nothing slow inside the loop: writing PNGs between
    frames drops hardware frames, which validate_recording rightly fails). Returns (frames, wall seconds)."""
    t0 = time.monotonic()
    frames = list(cam.frames(n))
    return frames, time.monotonic() - t0


def write_recording(info: dict[str, Any], frames: list[CaptureFrame], out_dir: Path,
                    compress_level: int | None = RECORD_PNG_LEVEL) -> Path:
    device = {k: info["device"].get(k, "unknown") for k in ("serial", "firmware", "usb_type")} | info["device"]
    with RecordingWriter(out_dir, session=out_dir.name, device=device, color=info["color"], depth=info["depth"],
                         color_intrinsics=info["color_intrinsics"],
                         depth_scale_m_per_unit=info["depth_scale_m_per_unit"],
                         extrinsics_depth_to_color=info["extrinsics_depth_to_color"],
                         aligned_to=info.get("aligned_to", "color"), compress_level=compress_level) as writer:
        writer.write_frames(frames, workers=RECORD_WRITE_WORKERS)
    return out_dir / "session.json"


def record_session(cam, out_dir: Path, n: int) -> Path:
    frames, _ = capture_frames(cam, n)
    return write_recording(cam.session_info, frames, out_dir)


def measured_fps(frames: list[CaptureFrame], fallback: float = 30.0) -> float:
    if len(frames) < 2:
        return fallback
    span = frames[-1].color_timestamp - frames[0].color_timestamp
    return (len(frames) - 1) / span if span > 0 else fallback


def depth_colormap(depth_u16: np.ndarray, scale_m: float, near_m: float = 0.07, far_m: float = 0.5) -> np.ndarray:
    """Depth as RGB (turbo, near = red ... far = blue, over the D405's 7-50 cm sweet spot); no depth = black."""
    import cv2
    z = depth_u16.astype(np.float32) * float(scale_m)
    t = np.clip((far_m - z) / (far_m - near_m), 0.0, 1.0)
    rgb = cv2.applyColorMap((t * 255).astype(np.uint8), cv2.COLORMAP_TURBO)[:, :, ::-1]
    rgb[depth_u16 == 0] = 0
    return rgb


def write_video(frames: list[CaptureFrame], path: Path, fps: float, depth_scale_m: float | None = None) -> Path:
    """H.264 MP4 (ffmpeg; OpenCV's mp4v if ffmpeg is missing): colour, plus colourised depth beside it when a
    depth scale is given. For watching; the lossless recording stays the data."""
    if not frames:
        raise ValueError("no frames to encode")
    h, w = frames[0].color_rgb.shape[:2]
    with_depth = depth_scale_m is not None
    W = 2 * w if with_depth else w
    path.parent.mkdir(parents=True, exist_ok=True)

    def canvas(f: CaptureFrame) -> np.ndarray:
        return np.hstack([f.color_rgb, depth_colormap(f.depth_u16, depth_scale_m)]) if with_depth else f.color_rgb

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg:
        cmd = [ffmpeg, "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{h}",
               "-r", f"{fps:.3f}", "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", "18",
               "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)]
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        assert proc.stdin is not None
        for f in frames:
            proc.stdin.write(np.ascontiguousarray(canvas(f)).tobytes())
        proc.stdin.close()
        if proc.wait() != 0:
            raise RuntimeError(f"ffmpeg failed (exit {proc.returncode}) writing {path}")
        return path
    import cv2
    vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, h))
    for f in frames:
        vw.write(np.ascontiguousarray(canvas(f)[:, :, ::-1]))
    vw.release()
    return path


def validate(recording_dir: Path) -> dict[str, Any]:
    vr = _load_script("validate_recording")
    checks, code = vr.run_checks(Path(recording_dir))
    worst = "FAIL" if any(c.status == "FAIL" for c in checks) else "WARN" if any(c.status == "WARN" for c in checks) else "PASS"
    return {"status": worst, "exit_code": code,
            "checks": [{"name": c.name, "status": c.status, "value": c.value, "hint": getattr(c, "hint", "")}
                       for c in checks]}


# ---- check (CAM-01) ----------------------------------------------------------------------------------------

def _lsusb_sees_d405() -> bool | None:
    exe = shutil.which("lsusb")
    if not exe:
        return None
    out = subprocess.run([exe], capture_output=True, text=True).stdout.lower()
    return "8086:0b5b" in out


def _ros_librealsense_version() -> str | None:
    exe = shutil.which("dpkg-query")
    if not exe:
        return None
    out = subprocess.run([exe, "-W", "-f=${Version}", "ros-humble-librealsense2"], capture_output=True, text=True)
    return out.stdout.strip() or None


def _sdk_version() -> str:
    import pyrealsense2 as rs
    return getattr(rs, "__version__", "unknown")


def cmd_check(cfg: Config, args: argparse.Namespace, logger: logging.Logger) -> int:
    problems, warnings = [], []
    try:
        sdk = _sdk_version()
    except ImportError as exc:
        print(f"FAIL pyrealsense2 is not importable ({exc}): pip install -r requirements/requirements-realsense.txt")
        return EXIT_PRECONDITION
    devices = list_devices()
    if not devices:
        seen = _lsusb_sees_d405()
        hint = ("The USB bus sees a D405 (8086:0b5b) but librealsense cannot open it: the udev rule or plugdev "
                "membership is missing (docs/host/DEVICE_ACCESS.md), or another process holds the camera."
                if seen else "No D405 on the USB bus. " + USB_HINT)
        print(f"FAIL no RealSense device found (pyrealsense2 {sdk}). {hint}")
        return EXIT_PRECONDITION
    d405 = [d for d in devices if d.get("product_id", "").upper() == "0B5B"] or devices
    dev = d405[0]
    if dev.get("product_id", "").upper() != "0B5B":
        warnings.append(f"device {dev['name']} is not a D405 (product id {dev.get('product_id')})")
    if not usb_is_3(dev.get("usb_type", "")):
        problems.append(f"USB link is {dev.get('usb_type')}: the D405 needs USB 3. " + USB_HINT)
    fw, rec = firmware_tuple(dev.get("firmware", "")), firmware_tuple(dev.get("recommended_firmware", ""))
    if fw and rec:
        fw_status = "up_to_date" if fw == rec else "older_than_recommended" if fw < rec else "newer_than_recommended"
    elif fw:   # pyrealsense2 >= 2.58.2 bundles no firmware, so it recommends none: compare with the newest D405 FW
        fw_status = ("latest_d405" if fw == LATEST_D405_FIRMWARE else "older_than_latest_d405" if fw < LATEST_D405_FIRMWARE
                     else "newer_than_latest_known")
    else:
        fw_status = "unknown"
    if fw and fw < MIN_GOOD_FIRMWARE:
        warnings.append(f"firmware {dev['firmware']} predates 5.17.0.9, which fixed D405 auto-exposure (dark/frozen "
                        "colour); updating to 5.17.0.10 is recommended, as an operator decision (docs/camera/D405_SETUP.md §5)")
    elif fw_status == "older_than_recommended":
        warnings.append(f"firmware {dev['firmware']} is older than the SDK's recommended {dev['recommended_firmware']} "
                        "(updating is an operator decision: docs/camera/D405_SETUP.md §5)")
    profiles = supported_profiles(serial=dev.get("serial"))

    width, height, fps, warmup = _stream_args(cfg, args)
    session = args.session or f"cam01_{utc_stamp()}"
    rec_dir = cfg.root / "data" / "recordings" / session
    try:
        with LiveD405Source(width, height, fps, warmup=warmup, serial=dev.get("serial"), logger=logger) as cam:
            info = cam.session_info
            frames, capture_s = capture_frames(cam, args.frames)
        write_recording(info, frames, rec_dir)
        fps_measured = measured_fps(frames)
    except D405Error as exc:
        print(f"FAIL capture: {exc}")
        problems.append(f"capture failed: {exc}")
        info, capture_s, fps_measured = {}, 0.0, 0.0
    validation = validate(rec_dir) if (rec_dir / "session.json").is_file() else {"status": "FAIL", "checks": []}
    if validation["status"] == "FAIL":
        problems.append("validate_recording: " + "; ".join(f"{c['name']}: {c['value']}" for c in validation["checks"]
                                                           if c["status"] == "FAIL"))
    scale = info.get("depth_scale_m_per_unit")
    if scale is not None and not (5e-5 <= scale <= 2e-4):
        warnings.append(f"depth scale {scale} m/unit is not the D405's usual 1e-4")

    verdict = "FAIL" if problems else "WARN" if warnings or validation["status"] == "WARN" else "PASS"
    evidence = {
        "schema": EVIDENCE_SCHEMA,
        "created_utc": _utc_iso(),
        "verdict": verdict,
        "problems": problems,
        "warnings": warnings,
        "host": {"hostname": platform.node(), "kernel": platform.release()},
        "sdk": {"pyrealsense2": sdk, "ros_librealsense2": _ros_librealsense_version()},
        "device": dev,
        "usb_3": usb_is_3(dev.get("usb_type", "")),
        "firmware_status": fw_status,
        "all_devices": devices,
        "supported_profiles": profiles,
        "capture": {"width": width, "height": height, "fps": fps, "warmup_frames": warmup, "frames": args.frames,
                    "seconds": round(capture_s, 2), "measured_fps": round(fps_measured, 2),
                    "depth_scale_m_per_unit": scale,
                    "color_intrinsics": info.get("color_intrinsics"),
                    "extrinsics_depth_to_color": info.get("extrinsics_depth_to_color")},
        "recording": {"path": _rel(rec_dir, cfg.root),
                      "session_json_sha256": _sha256(rec_dir / "session.json") if (rec_dir / "session.json").is_file() else None},
        "validation": validation,
    }
    out = Path(args.evidence) if args.evidence else cfg.root / "evidence" / "camera" / "d405_discovery.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")

    print(f"{verdict}  D405 serial {dev.get('serial')}  firmware {dev.get('firmware')} ({fw_status})  "
          f"USB {dev.get('usb_type')}  pyrealsense2 {sdk}")
    print(f"      {args.frames} frames at {width}x{height}@{fps} in {capture_s:.1f} s ({fps_measured:.1f} fps measured) -> {_rel(rec_dir, cfg.root)}; "
          f"validate_recording: {validation['status']}; depth scale {scale}")
    for c in validation["checks"]:
        if c["status"] != "PASS":
            print(f"      {c['status']} {c['name']}: {c['value']}" + (f"  ({c['hint']})" if c.get("hint") else ""))
    for p in problems:
        print(f"      PROBLEM: {p}")
    for w in warnings:
        print(f"      WARNING: {w}")
    print(f"      evidence: {_rel(out, cfg.root)}")
    return EXIT_RUNTIME if problems else EXIT_OK


# ---- record ------------------------------------------------------------------------------------------------

def cmd_record(cfg: Config, args: argparse.Namespace, logger: logging.Logger) -> int:
    session = args.session or f"rec_{utc_stamp()}"
    rec_dir = cfg.root / "data" / "recordings" / session
    if rec_dir.exists():
        print(f"recording {rec_dir} already exists; pick another --session")
        return EXIT_USAGE
    if args.no_frames and not args.video:
        print("--no-frames only makes sense with --video")
        return EXIT_USAGE
    width, height, fps, _ = _stream_args(cfg, args)
    n = int(round(args.seconds * fps)) if args.seconds else args.frames
    if not args.replay and (why := check_memory(n, width, height)):
        print(f"FAIL {why}")
        return EXIT_USAGE
    try:
        with open_camera(cfg, args, logger) as cam:
            info = cam.session_info
            print(f"recording {n} frames ({n / fps:.1f} s at {fps} fps) ...", flush=True)
            frames, wall_s = capture_frames(cam, n)
    except D405Error as exc:
        print(f"FAIL {exc}")
        return EXIT_PRECONDITION
    fps_m = measured_fps(frames, fps)
    code = EXIT_OK
    rec_dir.mkdir(parents=True, exist_ok=True)
    if not args.no_frames:
        t0 = time.monotonic()
        write_recording(info, frames, rec_dir)
        v = validate(rec_dir)
        print(f"{v['status']}  {len(frames)} frames in {wall_s:.1f} s ({fps_m:.1f} fps measured), written in "
              f"{time.monotonic() - t0:.1f} s -> {_rel(rec_dir, cfg.root)}")
        for c in v["checks"]:
            if c["status"] != "PASS":
                print(f"      {c['status']} {c['name']}: {c['value']}" + (f"  ({c['hint']})" if c.get("hint") else ""))
        code = EXIT_RUNTIME if v["status"] == "FAIL" else EXIT_OK
    if args.video:
        out = rec_dir / "video.mp4"
        write_video(frames, out, fps_m, None if args.no_depth else info.get("depth_scale_m_per_unit"))
        print(f"      video: {_rel(out, cfg.root)} ({len(frames)} frames, {fps_m:.1f} fps, "
              f"{'colour' if args.no_depth else 'colour | depth'}, {out.stat().st_size / 1e6:.1f} MB)")
    return code


# ---- snap --------------------------------------------------------------------------------------------------

def _image_frame(path: Path) -> tuple[CaptureFrame, dict[str, Any]]:
    rgb = np.asarray(Image.open(path).convert("RGB"))
    frame = CaptureFrame(frame_index=0, color_rgb=rgb, depth_u16=np.zeros(rgb.shape[:2], np.uint16),
                         color_timestamp=0.0, color_timestamp_domain="none", depth_timestamp=0.0,
                         depth_timestamp_domain="none", color_frame_number=0, depth_frame_number=0,
                         host_monotonic_ns=time.monotonic_ns(), extra={"source_image": str(path)})
    return frame, {"device": {"name": "image file", "serial": "none"}, "color_intrinsics": None,
                   "depth_scale_m_per_unit": None}


def summarize_case(cfg: Config, case_id: str, frame: CaptureFrame, info: dict[str, Any]) -> dict[str, Any]:
    """Crack pixels, paths and (with depth) distance and scale for a finished case."""
    out: dict[str, Any] = {"case_id": case_id}
    pred_path = cfg.root / naming.prediction_path(case_id)
    if pred_path.is_file():
        mask = np.asarray(Image.open(pred_path)) > 0
        out["crack_pixels"] = int(mask.sum())
        out["crack_fraction"] = round(float(mask.mean()), 5)
    else:
        mask = None
    scale = info.get("depth_scale_m_per_unit")
    intr = info.get("color_intrinsics") or {}
    depth = frame.depth_u16.astype(np.float64) * scale if scale else None
    if depth is not None and (depth > 0).any():
        on_crack = mask is not None and int((mask & (depth > 0)).sum()) >= MIN_CRACK_DEPTH_PX
        z = float(np.median(depth[(depth > 0) & mask] if on_crack else depth[depth > 0]))
        out["distance_m"] = round(z, 4)
        out["distance_from"] = "crack pixels" if on_crack else "whole frame"
        if z > D405_MAX_GOOD_Z_M:
            out["distance_note"] = f"beyond the D405's {100 * D405_MAX_GOOD_Z_M:.0f} cm working range: scale is rough"
        out["valid_depth_fraction"] = round(float((depth > 0).mean()), 4)
        if intr.get("fx"):
            out["mm_per_px"] = round(1000.0 * z / ((intr["fx"] + intr["fy"]) / 2), 4)
    paths_json = cfg.root / "data" / "paths" / f"{case_id}_paths.json"
    if paths_json.is_file():
        doc = json.loads(paths_json.read_text())
        lengths = [c["main_path"]["length_px"] for c in doc.get("components", [])]
        out["crack_paths"] = len(lengths)
        out["path_lengths_px"] = lengths
        if out.get("mm_per_px"):
            out["path_lengths_mm"] = [round(L * out["mm_per_px"], 1) for L in lengths]
    out["files"] = {k: _rel(cfg.root / p, cfg.root) for k, p in {
        "overlay": naming.overlay_path(case_id), "comparison": naming.comparison_path(case_id),
        "skeleton_overlay": naming.skeleton_overlay_path(case_id),
        "paths_overlay": Path("data/paths") / f"{case_id}_paths_overlay.png",
        "prediction": naming.prediction_path(case_id)}.items() if (cfg.root / p).is_file()}
    return out


def cmd_snap(cfg: Config, args: argparse.Namespace, logger: logging.Logger) -> int:
    case_id = safe_case_id(args.name or f"d405_{utc_stamp()}")
    case_dir = cfg.root / "data" / "camera" / case_id
    if case_dir.exists() and not args.force:
        print(f"{case_dir} already exists; pick another --name or pass --force")
        return EXIT_USAGE
    scores: list[float] = []
    if args.image:
        frame, info = _image_frame(Path(args.image))
        scores = [sharpness(frame.color_rgb)]
    else:
        try:
            with open_camera(cfg, args, logger) as cam:
                info = cam.session_info
                burst = list(cam.frames(args.burst))
        except D405Error as exc:
            print(f"FAIL {exc}")
            return EXIT_PRECONDITION
        if not burst:
            print("FAIL the source delivered no frames")
            return EXIT_RUNTIME
        scores = [sharpness(f.color_rgb) for f in burst]
        frame = burst[int(np.argmax(scores))]

    (case_dir / "input").mkdir(parents=True, exist_ok=True)
    src = case_dir / "input" / f"{case_id}.png"   # alone in its folder: prepare_inputs converts the whole folder
    Image.fromarray(frame.color_rgb, mode="RGB").save(src)
    if frame.depth_u16.any():
        _write_depth_png(frame.depth_u16, case_dir / f"{case_id}_depth.png")
    record = {"schema": SNAP_SCHEMA, "created_utc": _utc_iso(), "case_id": case_id,
              "source": "image" if args.image else ("replay" if args.replay else "d405"),
              "session_info": info, "frame": {"frame_index": frame.frame_index,
                                              "color_timestamp": frame.color_timestamp,
                                              "color_frame_number": frame.color_frame_number, "extra": frame.extra},
              "burst_sharpness": [round(s, 1) for s in scores], "chosen": int(np.argmax(scores)),
              "image": _rel(src, cfg.root)}

    rc = EXIT_OK
    if not args.no_model:
        osp = _load_script("operator_segment_paths")
        t0 = time.monotonic()
        q = ["-q"] if args.quiet else []
        rc = osp.main(["--root", str(cfg.root), "--case-id", case_id, "--source", str(src), "--stage", "both",
                       "--force", *q])
        if rc == EXIT_OK:   # the human-viewable mask/overlay/comparison renders (run_test.sh's visualize stage)
            from crackvision import visualize
            rc = visualize.main(["--root", str(cfg.root), "--cases", case_id, *q])
        record["pipeline"] = {"exit_code": rc, "seconds": round(time.monotonic() - t0, 1)}
        record["result"] = summarize_case(cfg, case_id, frame, info)
    (case_dir / "snap.json").write_text(json.dumps(record, indent=2, default=str) + "\n", encoding="utf-8")

    res = record.get("result", {})
    print(f"{'OK' if rc == 0 else 'FAILED'}  case {case_id}: frame {record['chosen'] + 1}/{len(scores)} (sharpest)"
          + (f", pipeline {record['pipeline']['seconds']} s" if "pipeline" in record else ""))
    if res:
        line = f"      cracks: {res.get('crack_paths', 0)} path(s), {res.get('crack_pixels', 0)} px ({100 * res.get('crack_fraction', 0):.2f} %)"
        if "distance_m" in res:
            line += f"; distance {100 * res['distance_m']:.1f} cm ({res['distance_from']})"
            if "distance_note" in res:
                line += f" [{res['distance_note']}]"
        if "mm_per_px" in res:
            line += f", {res['mm_per_px']:.3f} mm/px"
        if res.get("path_lengths_mm"):
            line += f"; lengths {res['path_lengths_mm']} mm"
        print(line)
        for k, v in res.get("files", {}).items():
            print(f"      {k:<16} {v}")
    print(f"      record: {_rel(case_dir / 'snap.json', cfg.root)}")
    return rc


# ---- live --------------------------------------------------------------------------------------------------

def overlay(rgb: np.ndarray, mask: np.ndarray, color=(255, 0, 0), alpha: float = 0.5) -> np.ndarray:
    out = rgb.astype(np.float32)
    m = mask.astype(bool)
    out[m] = (1 - alpha) * out[m] + alpha * np.asarray(color, np.float32)
    return out.astype(np.uint8)


_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>D405 crack view</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px system-ui}header{padding:8px 12px}img{max-width:100%;display:block}
button{font:inherit;padding:4px 10px}</style></head><body><header>D405 + OpenCrack nnU-Net, live
<button onclick="fetch('/snapshot').then(r=>r.json()).then(j=>document.getElementById('s').textContent=JSON.stringify(j))">snapshot</button>
<span id="s"></span></header><img src="/stream"></body></html>"""


class _LiveState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.jpeg: bytes = b""
        self.frame: CaptureFrame | None = None
        self.mask: np.ndarray | None = None
        self.stats: dict[str, Any] = {}
        self.seq = 0


def _make_handler(state: _LiveState, cfg: Config, info: dict[str, Any]):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a: Any) -> None:
            pass

        def _send(self, code: int, ctype: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/":
                return self._send(200, "text/html", _PAGE.encode())
            if self.path == "/stats":
                with state.lock:
                    return self._send(200, "application/json", json.dumps(state.stats).encode())
            if self.path == "/snapshot":
                with state.lock:
                    frame, mask = state.frame, state.mask
                if frame is None:
                    return self._send(503, "application/json", b'{"error": "no frame yet"}')
                d = cfg.root / "data" / "camera" / f"live_{utc_stamp()}"
                d.mkdir(parents=True, exist_ok=True)
                Image.fromarray(frame.color_rgb, mode="RGB").save(d / "color.png")
                Image.fromarray((mask * 255).astype(np.uint8)).save(d / "live_mask.png")
                if frame.depth_u16.any():
                    _write_depth_png(frame.depth_u16, d / "depth.png")
                (d / "info.json").write_text(json.dumps({"session_info": info, "extra": frame.extra}, indent=2, default=str))
                body = {"saved": _rel(d, cfg.root),
                        "full_pipeline": f"./env.sh python scripts/camera.py snap --image {_rel(d / 'color.png', cfg.root)}"}
                return self._send(200, "application/json", json.dumps(body).encode())
            if self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last = -1
                try:
                    while True:
                        with state.lock:
                            seq, jpeg = state.seq, state.jpeg
                        if seq != last and jpeg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                            last = seq
                        time.sleep(0.01)
                except (BrokenPipeError, ConnectionResetError):
                    return
            return self._send(404, "text/plain", b"not found")
    return Handler


def cmd_live(cfg: Config, args: argparse.Namespace, logger: logging.Logger) -> int:
    import cv2
    from crackvision.live_predictor import LivePredictor

    print("loading the OpenCrack nnU-Net into memory ...", flush=True)
    predictor = LivePredictor(cfg, tta=args.tta)
    vis = cfg.visualization
    color, alpha = tuple(vis.get("crack_color", (255, 0, 0))), float(vis.get("overlay_alpha", 0.5))
    state = _LiveState()
    try:
        cam_cm = open_camera(cfg, args, logger, loop=True)
        cam = cam_cm.__enter__()
    except D405Error as exc:
        print(f"FAIL {exc}")
        return EXIT_PRECONDITION
    server = ThreadingHTTPServer((args.host, args.port), _make_handler(state, cfg, cam.session_info))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"live view: http://{args.host}:{args.port}/   (Ctrl-C to stop; 'snapshot' saves the current frame)",
          flush=True)
    n, t_start, fps_ema = 0, time.monotonic(), None
    try:
        for frame in cam.frames(args.max_frames):
            t0 = time.monotonic()
            mask = predictor.predict(frame.color_rgb)
            dt = time.monotonic() - t0
            img = overlay(frame.color_rgb, mask, color, alpha)
            frac = float(mask.mean())
            fps_ema = 1.0 / max(dt, 1e-3) if fps_ema is None else 0.9 * fps_ema + 0.1 / max(dt, 1e-3)
            hud = f"model {1000 * dt:.0f} ms ({fps_ema:.1f} fps)  crack {100 * frac:.2f}%"
            if frame.depth_u16.any() and cam.session_info.get("depth_scale_m_per_unit"):
                d = frame.depth_u16[frame.depth_u16 > 0]
                if d.size:
                    hud += f"  dist {100 * float(np.median(d)) * cam.session_info['depth_scale_m_per_unit']:.1f} cm"
            bgr = np.ascontiguousarray(img[:, :, ::-1])
            cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 34), (0, 0, 0), -1)
            cv2.putText(bgr, hud, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            ok, jpeg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            with state.lock:
                state.jpeg, state.frame, state.mask, state.seq = jpeg.tobytes() if ok else state.jpeg, frame, mask, state.seq + 1
                state.stats = {"frames": n + 1, "model_ms": round(1000 * dt, 1), "fps": round(fps_ema, 2),
                               "crack_fraction": round(frac, 5), "tta": args.tta}
            n += 1
    except KeyboardInterrupt:
        pass
    except D405Error as exc:
        print(f"camera stopped: {exc}")
        return EXIT_RUNTIME
    finally:
        server.shutdown()
        cam_cm.__exit__(None, None, None)
    print(f"stopped after {n} frames in {time.monotonic() - t_start:.1f} s")
    return EXIT_OK


# ---- CLI ---------------------------------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="camera.py", description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    sub = p.add_subparsers(dest="cmd", required=True)

    def stream_opts(sp: argparse.ArgumentParser, replay: bool = True) -> None:
        sp.add_argument("--width", type=int, default=None, help="default: config/project.yaml realsense.color")
        sp.add_argument("--height", type=int, default=None)
        sp.add_argument("--fps", type=int, default=None)
        sp.add_argument("--hd", action="store_true", help="1280x720, the D405's full resolution")
        sp.add_argument("--warmup", type=int, default=None, help="frames discarded after start (auto-exposure)")
        sp.add_argument("--serial", default=None, help="pick one camera by serial number")
        if replay:
            sp.add_argument("--replay", default=None, metavar="DIR", help="a crackvision.recording session instead "
                                                                           "of the camera")

    c = sub.add_parser("check", help="CAM-01: discovery, firmware, USB link, validated recording, evidence JSON")
    stream_opts(c, replay=False)
    c.add_argument("--frames", type=int, default=CHECK_FRAMES)
    c.add_argument("--session", default=None)
    c.add_argument("--evidence", default=None, help="default evidence/camera/d405_discovery.json")

    r = sub.add_parser("record", help="lossless RGB-D recording (data/recordings/<session>/), validated")
    stream_opts(r)
    r.add_argument("--frames", type=int, default=60)
    r.add_argument("--seconds", type=float, default=None, help="record this long instead of --frames")
    r.add_argument("--video", action="store_true", help="also write video.mp4 (H.264: colour | colourised depth)")
    r.add_argument("--no-depth", action="store_true", help="video: colour only")
    r.add_argument("--no-frames", action="store_true", help="with --video: skip the lossless PNG frames")
    r.add_argument("--session", default=None)

    s = sub.add_parser("snap", help="capture, keep the sharpest frame, run the full crack pipeline")
    stream_opts(s)
    s.add_argument("--image", default=None, help="run on an existing image instead of capturing")
    s.add_argument("--burst", type=int, default=10, help="frames captured after warm-up; the sharpest is kept")
    s.add_argument("--name", default=None, help="case id (default d405_<UTC>)")
    s.add_argument("--no-model", action="store_true", help="capture and store only")
    s.add_argument("--force", action="store_true", help="overwrite an existing case")

    lv = sub.add_parser("live", help="model on every frame, MJPEG view in a browser")
    stream_opts(lv)
    lv.add_argument("--tta", action="store_true", help="mirroring TTA (= the batch path; ~8x slower)")
    lv.add_argument("--host", default="127.0.0.1")
    lv.add_argument("--port", type=int, default=DEFAULT_PORT)
    lv.add_argument("--max-frames", type=int, default=None)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        cfg = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"{TOOL}: {exc}", file=sys.stderr)
        return EXIT_USAGE
    logger = setup_logging(TOOL, cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()
    code = {"check": cmd_check, "record": cmd_record, "snap": cmd_snap, "live": cmd_live}[args.cmd](cfg, args, logger)
    summary.write(cfg, f"{TOOL}_{args.cmd}", "ok" if code == EXIT_OK else "failed", code)
    return code


if __name__ == "__main__":
    raise SystemExit(main())

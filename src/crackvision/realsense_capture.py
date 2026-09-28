"""crackvision.realsense_capture — D405 depth-aligned RGB + raw-depth capture. Built by TC-013.

Writes synchronised colour/depth PNGs plus per-frame intrinsics/extrinsics metadata
(docs/INTERFACES.md §3.10). `rs.align(rs.stream.color)` runs by default so that `depth[r,c]`
corresponds to `color[r,c]` — the frame invariant (docs/INTERFACES.md §0.5) every later stage
depends on. `pyrealsense2` is imported lazily (see `_import_rs`) so this module, `--list-devices`,
`--synthetic` and every test here work on a machine with no RealSense stack and no camera, which is
the normal state of this development box.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import logging
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from crackvision import __version__
from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
    utc_stamp,
)

TOOL_NAME = "realsense_capture"
SCHEMA_VERSION = 1
INSTALL_HINT = "./env.sh pip install -r requirements/requirements-realsense.txt"
NO_DEVICE_MSG = (
    "No RealSense device found. Attach a D405 (USB-3), or use --synthetic to exercise the file format."
)
# A plausible D405-like depth scale for synthetic frames only — deliberately spelled in scientific
# notation so it can never be confused with (or grepped as) the hard-coded literal this module must
# never use on the real hardware path; the real path always reads depth_sensor.get_depth_scale().
SYNTHETIC_DEPTH_SCALE_M_PER_UNIT = 1e-4


def _import_rs():
    """Import pyrealsense2 lazily. Callers must catch ImportError themselves."""
    import pyrealsense2 as rs

    return rs


def _load_depth_png_helpers():
    """Load write_depth_png/read_depth_png from scripts/check_realsense.py (TC-012 owns that file)."""
    if "crackvision._check_realsense" in sys.modules:
        module = sys.modules["crackvision._check_realsense"]
        return module.write_depth_png, module.read_depth_png

    module_path = Path(__file__).resolve().parents[2] / "scripts" / "check_realsense.py"
    spec = importlib.util.spec_from_file_location("crackvision._check_realsense", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    # check_realsense.py uses @dataclass, which looks up sys.modules[cls.__module__] during
    # class creation — the module must be registered before exec_module runs, or that lookup
    # gets None and dataclass() crashes.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.write_depth_png, module.read_depth_png


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _relpath(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _device_field(device: Any, rs: Any, camera_info_name: str) -> str:
    key = getattr(rs.camera_info, camera_info_name)
    try:
        if device.supports(key):
            return device.get_info(key)
    except Exception:  # noqa: BLE001
        pass
    return "unknown"


def _device_info_dict(device: Any, rs: Any) -> dict[str, str]:
    return {
        "name": _device_field(device, rs, "name"),
        "serial": _device_field(device, rs, "serial_number"),
        "firmware": _device_field(device, rs, "firmware_version"),
    }


# ---------------------------------------------------------------------------
# Synthetic frame generation (no camera, no pyrealsense2 needed)
# ---------------------------------------------------------------------------


def synthetic_color_frame(width: int, height: int, seed: int) -> np.ndarray:
    """Noise plus a thin dark diagonal line standing in for a crack. 8-bit RGB, HxWx3."""
    rng = np.random.default_rng(1000 + seed)
    arr = rng.integers(60, 200, size=(height, width, 3), dtype=np.uint8)
    line_len = min(width, height)
    idx = np.arange(line_len)
    rows = np.clip(idx * height // line_len, 0, height - 1)
    cols = np.clip(idx * width // line_len, 0, width - 1)
    for dr in (-1, 0, 1):
        r = np.clip(rows + dr, 0, height - 1)
        arr[r, cols] = (20, 20, 20)
    return arr


def synthetic_depth_frame(width: int, height: int, seed: int) -> np.ndarray:
    """A smooth left-to-right ramp (raw z16 units) with a few zero "holes". uint16, HxW."""
    ramp = np.linspace(300, 3000, num=width, dtype=np.float64)
    depth = np.tile(ramp, (height, 1)).astype(np.uint16)
    rng = np.random.default_rng(2000 + seed)
    for _ in range(5):
        hr = int(rng.integers(0, height))
        hc = int(rng.integers(0, width))
        depth[max(0, hr - 3) : min(height, hr + 3), max(0, hc - 3) : min(width, hc + 3)] = 0
    return depth


def _synthetic_intrinsics(width: int, height: int) -> dict[str, Any]:
    return {
        "fx": round(width * 0.85, 3),
        "fy": round(width * 0.85, 3),
        "ppx": round(width / 2, 3),
        "ppy": round(height / 2, 3),
        "model": "synthetic",
        "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
    }


def _synthetic_extrinsics() -> dict[str, list[float]]:
    return {"rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], "translation": [0.0, 0.0, 0.0]}


# ---------------------------------------------------------------------------
# Real hardware intrinsics/extrinsics + retry helpers
# ---------------------------------------------------------------------------


def _intrinsics_dict(stream_profile: Any) -> dict[str, Any]:
    intr = stream_profile.as_video_stream_profile().get_intrinsics()
    return {
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "ppx": float(intr.ppx),
        "ppy": float(intr.ppy),
        "model": str(intr.model),
        "coeffs": [float(c) for c in intr.coeffs],
    }


def _extrinsics_dict(depth_profile: Any, color_profile: Any) -> dict[str, list[float]]:
    extr = depth_profile.get_extrinsics_to(color_profile)
    return {"rotation": [float(x) for x in extr.rotation], "translation": [float(x) for x in extr.translation]}


def _frame_metadata_value(frame: Any, rs: Any, key_name: str) -> float | None:
    key = getattr(rs.frame_metadata_value, key_name, None)
    if key is None:
        return None
    try:
        if frame.supports_frame_metadata(key):
            return frame.get_frame_metadata(key)
    except Exception:  # noqa: BLE001 — not all firmware exposes this; None is fine
        pass
    return None


def _wait_for_frames_with_retry(pipeline: Any, logger: logging.Logger, retries: int = 3, timeout_ms: int = 5000):
    last_exc: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            return pipeline.wait_for_frames(timeout_ms)
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            logger.warning("wait_for_frames failed (attempt %d/%d): %s: %s", attempt, retries, type(exc).__name__, exc)
    raise RuntimeError(
        f"wait_for_frames failed after {retries} attempts: {type(last_exc).__name__}: {last_exc}. "
        "Check the USB link — a USB-2 port or a poor-quality cable silently reduces bandwidth; use USB-3."
    )


def _print_supported_profiles(device: Any, rs: Any, logger: logging.Logger) -> None:
    for sensor in device.query_sensors():
        try:
            profiles = sensor.get_stream_profiles()
        except Exception:  # noqa: BLE001
            continue
        for profile in profiles:
            vprofile = profile.as_video_stream_profile() if profile.is_video_stream_profile() else None
            if vprofile is not None:
                logger.error(
                    "device supports: %s %dx%d@%d %s",
                    profile.stream_name(),
                    vprofile.width(),
                    vprofile.height(),
                    profile.fps(),
                    profile.format(),
                )
            else:
                logger.error("device supports: %s@%d", profile.stream_name(), profile.fps())


def _run_preview(logger: logging.Logger, color_bgr: np.ndarray, depth_colorized: np.ndarray) -> bool:
    """Show a side-by-side preview window. Returns False if the caller should stop (q pressed or unavailable)."""
    if not os.environ.get("DISPLAY"):
        logger.warning("--preview requested but $DISPLAY is unset; skipping preview window (no-op)")
        return False
    try:
        import cv2

        combined = np.hstack((color_bgr, depth_colorized))
        cv2.imshow("crackvision D405 preview (q to quit)", combined)
        key = cv2.waitKey(1) & 0xFF
        return key != ord("q")
    except Exception as exc:  # noqa: BLE001 — preview is diagnostic only, must never crash a capture
        logger.warning("preview unavailable: %s: %s", type(exc).__name__, exc)
        return False


# ---------------------------------------------------------------------------
# Argument parsing / config resolution
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="realsense_capture.py",
        description="Depth-aligned RGB + raw-depth capture from an Intel RealSense D405 (docs/INTERFACES.md §3.10).",
    )
    add_common_args(parser)
    parser.add_argument("--session", default=None, metavar="NAME", help="session name (default: UTC timestamp)")
    parser.add_argument(
        "--frames", type=int, default=1, metavar="N", help="frames to capture from a real device (default: 1)"
    )
    parser.add_argument(
        "--interval", type=float, default=0.0, metavar="S", help="seconds to sleep between saved frames (default: 0)"
    )
    parser.add_argument("--color-width", type=int, default=None, metavar="W")
    parser.add_argument("--color-height", type=int, default=None, metavar="H")
    parser.add_argument("--depth-width", type=int, default=None, metavar="W")
    parser.add_argument("--depth-height", type=int, default=None, metavar="H")
    parser.add_argument("--fps", type=int, default=None, metavar="F")
    parser.add_argument(
        "--no-align", action="store_true", help="disable depth-to-colour alignment (diagnostics only)"
    )
    parser.add_argument(
        "--preview", action="store_true", help="open an OpenCV preview window (no-op WARN if $DISPLAY is unset)"
    )
    parser.add_argument(
        "--warmup", type=int, default=None, metavar="N", help="frames to discard before saving (default: config/project.yaml)"
    )
    parser.add_argument("--list-devices", action="store_true", help="print attached RealSense devices and exit")
    parser.add_argument(
        "--synthetic", type=int, default=None, metavar="N", help="write N synthetic frames instead of using a real camera"
    )
    parser.add_argument(
        "--save-depth-preview",
        action="store_true",
        help="also write a colourised _depthviz.png (never a substitute for the raw depth PNG)",
    )
    return parser


def _resolve_stream_config(cfg: Config, args: argparse.Namespace) -> dict[str, int]:
    rs_cfg = cfg.realsense
    return {
        "cw": args.color_width or rs_cfg["color"]["width"],
        "ch": args.color_height or rs_cfg["color"]["height"],
        "dw": args.depth_width or rs_cfg["depth"]["width"],
        "dh": args.depth_height or rs_cfg["depth"]["height"],
        "fps": args.fps or rs_cfg["color"]["fps"],
        "warmup": args.warmup if args.warmup is not None else rs_cfg["warmup_frames"],
    }


def _d405_dirs(cfg: Config) -> tuple[Path, Path, Path]:
    d405_root = cfg.paths["d405"]
    color_dir, depth_dir, meta_dir = d405_root / "color", d405_root / "depth", d405_root / "metadata"
    for directory in (color_dir, depth_dir, meta_dir):
        directory.mkdir(parents=True, exist_ok=True)
    return color_dir, depth_dir, meta_dir


def _write_session_json(
    meta_dir: Path,
    filename_prefix: str,
    session: str,
    *,
    synthetic: bool,
    device_info: dict[str, str],
    resolved: dict[str, int],
    align_enabled: bool,
    started_utc: str,
    frame_count: int,
) -> Path:
    payload = {
        "schema_version": SCHEMA_VERSION,
        "session": session,
        "synthetic": synthetic,
        "device": device_info,
        "tool_version": __version__,
        "started_utc": started_utc,
        "finished_utc": _utc_iso(),
        "frame_count": frame_count,
        "config": {
            "color": {"width": resolved["cw"], "height": resolved["ch"], "fps": resolved["fps"], "format": "bgr8"},
            "depth": {"width": resolved["dw"], "height": resolved["dh"], "fps": resolved["fps"], "format": "z16"},
            "aligned_to": "color" if align_enabled else None,
            "warmup_frames": resolved["warmup"],
        },
    }
    path = meta_dir / f"{filename_prefix}{session}_session.json"
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return path


# ---------------------------------------------------------------------------
# Command implementations
# ---------------------------------------------------------------------------


def _cmd_list_devices(cfg: Config, logger: logging.Logger) -> int:
    try:
        rs = _import_rs()
    except ImportError:
        msg = f"pyrealsense2 is not installed. Run: {INSTALL_HINT}"
        logger.error(msg)
        print(msg, file=sys.stderr)
        return EXIT_PRECONDITION

    devices = list(rs.context().query_devices())
    if not devices:
        print("No RealSense devices found.")
    for device in devices:
        info = _device_info_dict(device, rs)
        usb = _device_field(device, rs, "usb_type_descriptor")
        line = f"name={info['name']} serial={info['serial']} firmware={info['firmware']} usb={usb}"
        print(line)
        logger.info(line)
    return EXIT_OK


def _cmd_synthetic(cfg: Config, args: argparse.Namespace, resolved: dict[str, int], logger: logging.Logger, summary: RunSummary) -> int:
    n = args.synthetic
    if n <= 0:
        msg = "--synthetic N requires N >= 1"
        logger.error(msg)
        print(msg, file=sys.stderr)
        return EXIT_USAGE

    write_depth_png, _read_depth_png = _load_depth_png_helpers()

    session = args.session or utc_stamp()
    align_enabled = not args.no_align
    if args.no_align:
        logger.warning("ALIGNMENT DISABLED (--no-align): saved depth is NOT pixel-aligned to colour")

    color_dir, depth_dir, meta_dir = _d405_dirs(cfg)
    device_info = {"name": "Synthetic RealSense D405", "serial": "SYNTHETIC", "firmware": "0.0.0-synthetic"}
    started_utc = _utc_iso()
    cw, ch, dw, dh, fps, warmup = resolved["cw"], resolved["ch"], resolved["dw"], resolved["dh"], resolved["fps"], resolved["warmup"]

    for idx in range(n):
        color_rgb = synthetic_color_frame(cw, ch, idx)
        depth_u16 = synthetic_depth_frame(dw, dh, idx)

        stem = f"synth_{session}_{idx:06d}"
        color_path = color_dir / f"{stem}_color.png"
        depth_path = depth_dir / f"{stem}_depth.png"
        meta_path = meta_dir / f"{stem}.json"

        Image.fromarray(color_rgb, mode="RGB").save(color_path)
        write_depth_png(depth_u16, depth_path)

        metadata = {
            "schema_version": SCHEMA_VERSION,
            "session": session,
            "frame_index": idx,
            "timestamp_utc": _utc_iso(),
            "device": device_info,
            "aligned_to": "color" if align_enabled else None,
            "warmup_frames": warmup,
            "depth_scale_m_per_unit": SYNTHETIC_DEPTH_SCALE_M_PER_UNIT,
            "color": {"width": cw, "height": ch, "fps": fps, "format": "bgr8", "intrinsics": _synthetic_intrinsics(cw, ch)},
            "depth": {"width": dw, "height": dh, "fps": fps, "format": "z16", "intrinsics": _synthetic_intrinsics(dw, dh)},
            "extrinsics_depth_to_color": _synthetic_extrinsics(),
            "exposure_us": None,
            "gain": None,
            "synthetic": True,
            "files": {"color": _relpath(color_path, cfg.root), "depth": _relpath(depth_path, cfg.root)},
        }
        meta_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        summary.increment("frames")
        logger.info("wrote synthetic frame %d/%d: %s", idx + 1, n, stem)

    _write_session_json(
        meta_dir,
        "synth_",
        session,
        synthetic=True,
        device_info=device_info,
        resolved=resolved,
        align_enabled=align_enabled,
        started_utc=started_utc,
        frame_count=n,
    )

    summary.write(cfg, TOOL_NAME, "ok", EXIT_OK)
    return EXIT_OK


def _cmd_real(cfg: Config, args: argparse.Namespace, resolved: dict[str, int], logger: logging.Logger, summary: RunSummary) -> int:
    try:
        rs = _import_rs()
    except ImportError:
        msg = f"pyrealsense2 is not installed. Run: {INSTALL_HINT}"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, TOOL_NAME, "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    devices = list(rs.context().query_devices())
    if not devices:
        logger.error(NO_DEVICE_MSG)
        print(NO_DEVICE_MSG, file=sys.stderr)
        summary.add_error(NO_DEVICE_MSG)
        summary.write(cfg, TOOL_NAME, "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    session = args.session or utc_stamp()
    align_enabled = not args.no_align
    if args.no_align:
        logger.warning("ALIGNMENT DISABLED (--no-align): saved depth is NOT pixel-aligned to colour")

    write_depth_png, _read_depth_png = _load_depth_png_helpers()
    color_dir, depth_dir, meta_dir = _d405_dirs(cfg)
    cw, ch, dw, dh, fps, warmup = resolved["cw"], resolved["ch"], resolved["dw"], resolved["dh"], resolved["fps"], resolved["warmup"]

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)

    try:
        profile = pipeline.start(config)
    except Exception as exc:  # noqa: BLE001 — unsupported resolution/fps for this device
        logger.error("failed to start RealSense pipeline: %s: %s", type(exc).__name__, exc)
        _print_supported_profiles(devices[0], rs, logger)
        summary.add_error(str(exc))
        summary.write(cfg, TOOL_NAME, "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    align = rs.align(rs.stream.color) if align_enabled else None
    device = profile.get_device()
    device_info = _device_info_dict(device, rs)

    try:
        depth_scale = device.first_depth_sensor().get_depth_scale()
    except Exception as exc:  # noqa: BLE001 — never substitute a guess (docs/RISKS.md R-09)
        logger.error("get_depth_scale() failed: %s: %s — refusing to guess, hard stop", type(exc).__name__, exc)
        summary.add_error(str(exc))
        pipeline.stop()
        summary.write(cfg, TOOL_NAME, "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    started_utc = _utc_iso()
    frames_captured = 0
    discarded = 0
    intrinsics_cache: dict[str, Any] | None = None
    preview_active = args.preview

    try:
        try:
            target = warmup + args.frames
            for i in range(target):
                try:
                    frameset = _wait_for_frames_with_retry(pipeline, logger)
                except RuntimeError as exc:
                    logger.error(str(exc))
                    summary.add_error(str(exc))
                    summary.write(cfg, TOOL_NAME, "failed", EXIT_RUNTIME)
                    return EXIT_RUNTIME

                if align is not None:
                    frameset = align.process(frameset)

                if i < warmup:
                    discarded += 1
                    continue

                color_frame = frameset.get_color_frame()
                depth_frame = frameset.get_depth_frame()
                if not color_frame or not depth_frame:
                    logger.warning("frame %d: missing colour or depth frame, skipping", i)
                    continue

                if intrinsics_cache is None:
                    intrinsics_cache = {
                        "color": _intrinsics_dict(color_frame.get_profile()),
                        "depth": _intrinsics_dict(depth_frame.get_profile()),
                        "extrinsics": _extrinsics_dict(depth_frame.get_profile(), color_frame.get_profile()),
                    }

                color_bgr = np.asanyarray(color_frame.get_data())
                color_rgb = color_bgr[:, :, ::-1]  # bgr8 stream -> RGB, exactly once
                depth_u16 = np.asanyarray(depth_frame.get_data()).astype(np.uint16)

                stem = f"{session}_{frames_captured:06d}"
                color_path = color_dir / f"{stem}_color.png"
                depth_path = depth_dir / f"{stem}_depth.png"
                meta_path = meta_dir / f"{stem}.json"

                Image.fromarray(color_rgb, mode="RGB").save(color_path)
                write_depth_png(depth_u16, depth_path)

                if args.save_depth_preview:
                    colorizer = rs.colorizer()
                    depth_viz = np.asanyarray(colorizer.colorize(depth_frame).get_data())
                    Image.fromarray(depth_viz, mode="RGB").save(depth_dir / f"{stem}_depthviz.png")

                exposure_us = _frame_metadata_value(color_frame, rs, "actual_exposure")
                gain = _frame_metadata_value(color_frame, rs, "gain_level")

                metadata = {
                    "schema_version": SCHEMA_VERSION,
                    "session": session,
                    "frame_index": frames_captured,
                    "timestamp_utc": _utc_iso(),
                    "device": device_info,
                    "aligned_to": "color" if align_enabled else None,
                    "warmup_frames": warmup,
                    "depth_scale_m_per_unit": depth_scale,
                    "color": {"width": cw, "height": ch, "fps": fps, "format": "bgr8", "intrinsics": intrinsics_cache["color"]},
                    "depth": {"width": dw, "height": dh, "fps": fps, "format": "z16", "intrinsics": intrinsics_cache["depth"]},
                    "extrinsics_depth_to_color": intrinsics_cache["extrinsics"],
                    "exposure_us": exposure_us,
                    "gain": gain,
                    "synthetic": False,
                    "files": {"color": _relpath(color_path, cfg.root), "depth": _relpath(depth_path, cfg.root)},
                }
                meta_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

                frames_captured += 1
                summary.increment("frames")
                logger.info("captured frame %d/%d: %s", frames_captured, args.frames, stem)

                if preview_active:
                    colorizer = rs.colorizer()
                    depth_viz = np.asanyarray(colorizer.colorize(depth_frame).get_data())
                    preview_active = _run_preview(logger, color_bgr, depth_viz)

                if args.interval:
                    time.sleep(args.interval)

            logger.info("discarded %d warm-up frame(s)", discarded)
        except KeyboardInterrupt:
            logger.warning("KeyboardInterrupt: stopping after %d frame(s) captured", frames_captured)
    finally:
        pipeline.stop()
        if args.preview:
            try:
                import cv2

                cv2.destroyAllWindows()
            except Exception:  # noqa: BLE001
                pass

    _write_session_json(
        meta_dir,
        "",
        session,
        synthetic=False,
        device_info=device_info,
        resolved=resolved,
        align_enabled=align_enabled,
        started_utc=started_utc,
        frame_count=frames_captured,
    )

    if frames_captured == 0:
        summary.write(cfg, TOOL_NAME, "failed", EXIT_RUNTIME)
        return EXIT_RUNTIME

    summary.write(cfg, TOOL_NAME, "ok", EXIT_OK)
    return EXIT_OK


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"realsense_capture: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging(TOOL_NAME, cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    if args.list_devices:
        return _cmd_list_devices(cfg, logger)

    resolved = _resolve_stream_config(cfg, args)

    if args.dry_run:
        logger.info(
            "dry-run: session=%s color=%dx%d@%d depth=%dx%d@%d warmup=%d align=%s synthetic=%s",
            args.session or "<auto>",
            resolved["cw"],
            resolved["ch"],
            resolved["fps"],
            resolved["dw"],
            resolved["dh"],
            resolved["fps"],
            resolved["warmup"],
            not args.no_align,
            args.synthetic is not None,
        )
        logger.info("dry-run: no files written")
        return EXIT_OK

    if args.synthetic is not None:
        return _cmd_synthetic(cfg, args, resolved, logger, summary)

    return _cmd_real(cfg, args, resolved, logger, summary)


if __name__ == "__main__":
    raise SystemExit(main())

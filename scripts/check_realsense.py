"""scripts/check_realsense.py — RealSense/D405 software validation. Built by TC-012.

Reports PASS/WARN/FAIL for the pyrealsense2 software stack and any attached device, and
succeeds with exit 0 even when no camera is attached — that is the normal case on this
box (docs/INTERFACES.md §3.9). Exit 3 only when pyrealsense2 itself is unimportable.
This script also hosts write_depth_png/read_depth_png, the 16-bit depth PNG round-trip
helpers that TC-013 imports.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_USAGE, RunSummary, setup_logging

STATUS_PASS = "PASS"
STATUS_WARN = "WARN"
STATUS_FAIL = "FAIL"

INSTALL_HINT = "./env.sh pip install -r requirements/requirements-realsense.txt"


def write_depth_png(arr: np.ndarray, path: Path) -> None:
    """Write a uint16 depth array as a 16-bit grayscale PNG, losslessly."""
    if arr.dtype != np.uint16:
        raise ValueError(f"expected uint16 array, got {arr.dtype}")
    img = Image.new("I;16", (arr.shape[1], arr.shape[0]))
    img.frombytes(arr.tobytes())
    img.save(path)


def read_depth_png(path: Path) -> np.ndarray:
    """Read a 16-bit grayscale PNG back into a uint16 array."""
    with Image.open(path) as img:
        return np.array(img.convert("I;16"), dtype=np.uint16)


@dataclass
class Check:
    name: str
    status: str
    value: str
    hint: str = ""

    def to_dict(self) -> dict[str, str]:
        payload = {"name": self.name, "status": self.status, "value": self.value}
        if self.hint:
            payload["hint"] = self.hint
        return payload


def _pyrealsense2_version(rs) -> str:
    version = getattr(rs, "__version__", None)
    if version:
        return version
    try:
        return importlib.metadata.version("pyrealsense2")
    except importlib.metadata.PackageNotFoundError:
        return "unknown"


def _device_info(device, key) -> str | None:
    try:
        if device.supports(key):
            return device.get_info(key)
    except Exception:  # noqa: BLE001
        pass
    return None


def run_checks() -> tuple[list[Check], int]:
    """Return (checks, exit_code). exit_code is EXIT_PRECONDITION only if pyrealsense2 is unimportable."""
    checks: list[Check] = []

    try:
        import pyrealsense2 as rs
    except Exception as exc:  # noqa: BLE001
        checks.append(Check("pyrealsense2_import", STATUS_FAIL, f"{type(exc).__name__}: {exc}", INSTALL_HINT))
        return checks, EXIT_PRECONDITION

    checks.append(Check("pyrealsense2_import", STATUS_PASS, "ok"))

    version = _pyrealsense2_version(rs)
    checks.append(Check("pyrealsense2_version", STATUS_PASS, version))

    runtime_module = getattr(rs, "pyrealsense2", rs)
    runtime_version = getattr(runtime_module, "__version__", None)
    if runtime_version:
        checks.append(Check("librealsense_runtime", STATUS_PASS, str(runtime_version)))
    else:
        checks.append(Check("librealsense_runtime", STATUS_WARN, "not exposed by this wheel"))

    try:
        context = rs.context()
        devices = list(context.query_devices())
    except Exception as exc:  # noqa: BLE001
        checks.append(
            Check(
                "devices_found",
                STATUS_WARN,
                f"{type(exc).__name__}: {exc}",
                "rs.context() raised — no udev rules or no permission; this is not fatal",
            )
        )
        devices = []
    else:
        if devices:
            checks.append(Check("devices_found", STATUS_PASS, str(len(devices))))
        else:
            checks.append(
                Check(
                    "devices_found",
                    STATUS_WARN,
                    "0",
                    "no RealSense device attached. The software stack is fine. Attach a D405 to a "
                    "USB-3 port, or use `--synthetic` with crackvision.realsense_capture (TC-013).",
                )
            )

    d405_device = None
    for device in devices:
        name = _device_info(device, rs.camera_info.name) or "unknown"
        serial = _device_info(device, rs.camera_info.serial_number) or "unknown"
        firmware = _device_info(device, rs.camera_info.firmware_version) or "unknown"
        port = _device_info(device, rs.camera_info.physical_port) or "unknown"
        usb = _device_info(device, rs.camera_info.usb_type_descriptor) or "unknown"
        checks.append(
            Check(
                "device_details",
                STATUS_PASS,
                f"name={name} serial={serial} firmware={firmware} port={port} usb={usb}",
            )
        )
        if "D405" in name and d405_device is None:
            d405_device = device
        if usb not in ("unknown",) and usb.strip().startswith("2"):
            checks.append(
                Check(
                    "usb_descriptor",
                    STATUS_WARN,
                    usb,
                    "USB-2 link — resolution/fps will be limited; use a USB-3 port and cable",
                )
            )

    if d405_device is not None:
        checks.append(Check("d405_present", STATUS_PASS, "D405 found"))
    else:
        checks.append(Check("d405_present", STATUS_WARN, "no D405 among attached devices"))

    if d405_device is not None:
        try:
            profile_names: list[str] = []
            for sensor in d405_device.query_sensors():
                for profile in sensor.get_stream_profiles():
                    vprofile = profile.as_video_stream_profile() if profile.is_video_stream_profile() else None
                    if vprofile is not None:
                        profile_names.append(
                            f"{profile.stream_name()} {vprofile.width()}x{vprofile.height()}@{profile.fps()}"
                        )
                    else:
                        profile_names.append(f"{profile.stream_name()}@{profile.fps()}")
            checks.append(Check("stream_profiles", STATUS_PASS, f"{len(profile_names)} profiles"))
        except Exception as exc:  # noqa: BLE001
            checks.append(Check("stream_profiles", STATUS_WARN, f"{type(exc).__name__}: {exc}"))

        try:
            depth_sensor = d405_device.first_depth_sensor()
            scale = depth_sensor.get_depth_scale()
            checks.append(
                Check("depth_scale", STATUS_PASS, f"{scale} m/unit (D405 is typically 0.0001 m/unit)")
            )
        except Exception as exc:  # noqa: BLE001
            checks.append(Check("depth_scale", STATUS_WARN, f"{type(exc).__name__}: {exc}"))
    else:
        checks.append(Check("stream_profiles", STATUS_WARN, "skipped — no device present"))
        checks.append(Check("depth_scale", STATUS_WARN, "skipped — no device present"))

    return checks, EXIT_OK


def _print_table(checks: list[Check]) -> None:
    name_width = max(len("CHECK"), *(len(c.name) for c in checks))
    status_width = max(len("STATUS"), 4)
    print(f"{'CHECK':<{name_width}}  {'STATUS':<{status_width}}  VALUE")
    for check in checks:
        print(f"{check.name:<{name_width}}  {check.status:<{status_width}}  {check.value}")
        if check.status != STATUS_PASS and check.hint:
            print(f"   hint: {check.hint}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="check_realsense.py",
        description="RealSense/D405 software validation (docs/INTERFACES.md §3.9).",
    )
    add_common_args(parser)
    parser.add_argument("--json", action="store_true", help="print the JSON summary instead of the table")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"check_realsense: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("check_realsense", cfg, verbose=args.verbose, quiet=args.quiet)

    checks, exit_code = run_checks()

    log_by_status = {STATUS_PASS: logger.info, STATUS_WARN: logger.warning, STATUS_FAIL: logger.error}
    for check in checks:
        log_by_status[check.status]("%-24s %-4s %s", check.name, check.status, check.value)

    if args.json:
        print(json.dumps({"checks": [c.to_dict() for c in checks]}, indent=2))
    else:
        _print_table(checks)

    status = "precondition" if exit_code != EXIT_OK else "ok"
    summary = RunSummary()
    for check in checks:
        summary.increment(check.status.lower())
        if check.status == STATUS_FAIL:
            summary.add_error(f"{check.name}: {check.value}")

    if args.dry_run:
        logger.info("dry-run: logs/check_realsense_latest.json not written")
        return exit_code

    latest_path = summary.write(cfg, "check_realsense", status, exit_code)
    data = json.loads(latest_path.read_text(encoding="utf-8"))
    data["checks"] = [c.to_dict() for c in checks]
    latest_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

"""scripts/validate_recording.py — recording validator with actionable diagnostics (CAM-03).

Runs a battery of checks against a `crackvision.recording` session (CAM-02, docs/INTERFACES.md
§3.14) and reports whether it is usable. Every failing check prints a cause and a concrete fix,
never just "FAIL". This script never imports `pyrealsense2` — it reads recordings, it never talks
to a camera.

Checks:
  - metadata completeness (session.json required keys, per §3.14)
  - depth scale present and plausible (D405 ≈ 1e-4 m/unit)
  - timestamp monotonicity (colour and depth streams each non-decreasing)
  - colour/depth timestamp delta distribution (mean/max |depth_timestamp - color_timestamp|)
  - dropped frames (sidecar count vs. session.json frame_count; hardware frame-number gaps)
  - USB 2 fallback (device.usb_type)
  - invalid-depth fraction per frame (depth_u16 == 0)
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)
from crackvision.recording import RecordingReader

STATUS_PASS = "PASS"
STATUS_WARN = "WARN"
STATUS_FAIL = "FAIL"

REQUIRED_DEVICE_KEYS = ("serial", "firmware", "usb_type")
REQUIRED_INTRINSICS_KEYS = ("fx", "fy", "ppx", "ppy", "model", "coeffs", "width", "height")
REQUIRED_EXTRINSICS_KEYS = ("rotation", "translation")
REQUIRED_SESSION_KEYS = (
    "device",
    "color",
    "depth",
    "color_intrinsics",
    "depth_scale_m_per_unit",
    "extrinsics_depth_to_color",
)

D405_DEPTH_SCALE_M_PER_UNIT = 1e-4
DEPTH_SCALE_PLAUSIBLE_LOW = 5e-5
DEPTH_SCALE_PLAUSIBLE_HIGH = 2e-4

TIMESTAMP_DELTA_WARN_S = 0.05
TIMESTAMP_DELTA_FAIL_S = 0.5

DROPPED_FRAME_WARN_FRACTION = 0.01
DROPPED_FRAME_FAIL_FRACTION = 0.05

INVALID_DEPTH_WARN_FRACTION = 0.2
INVALID_DEPTH_FAIL_FRACTION = 0.8


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


def _check_metadata_completeness(session_info: dict[str, Any]) -> list[Check]:
    checks: list[Check] = []

    missing = [key for key in REQUIRED_SESSION_KEYS if key not in session_info]
    if missing:
        checks.append(
            Check(
                "metadata_completeness",
                STATUS_FAIL,
                f"session.json missing key(s): {missing}",
                "recording was not closed by RecordingWriter.close(), or session.json was hand-"
                "edited — re-record with crackvision.recording.RecordingWriter",
            )
        )
        return checks

    for label, payload, required_keys in (
        ("device", session_info.get("device", {}), REQUIRED_DEVICE_KEYS),
        ("color_intrinsics", session_info.get("color_intrinsics", {}), REQUIRED_INTRINSICS_KEYS),
        (
            "extrinsics_depth_to_color",
            session_info.get("extrinsics_depth_to_color", {}),
            REQUIRED_EXTRINSICS_KEYS,
        ),
    ):
        sub_missing = [key for key in required_keys if key not in payload]
        if sub_missing:
            checks.append(
                Check(
                    "metadata_completeness",
                    STATUS_FAIL,
                    f"{label} missing key(s): {sub_missing}",
                    f"re-record so {label} is populated from the live D405 (pyrealsense2 device "
                    "info / intrinsics / extrinsics) before RecordingWriter.close()",
                )
            )

    if not checks:
        checks.append(Check("metadata_completeness", STATUS_PASS, "all required session.json keys present"))
    return checks


def _check_depth_scale(session_info: dict[str, Any]) -> Check:
    scale = session_info.get("depth_scale_m_per_unit")
    if scale is None:
        return Check(
            "depth_scale",
            STATUS_FAIL,
            "missing",
            "depth_scale_m_per_unit absent from session.json — re-record and pass the value read "
            "from depth_sensor.get_depth_scale() to RecordingWriter",
        )
    try:
        scale = float(scale)
    except (TypeError, ValueError):
        return Check(
            "depth_scale",
            STATUS_FAIL,
            f"non-numeric: {scale!r}",
            "session.json is corrupted — re-record",
        )
    if scale <= 0:
        return Check(
            "depth_scale",
            STATUS_FAIL,
            f"{scale} m/unit",
            "depth scale must be positive — re-record, this session.json is corrupted",
        )
    if DEPTH_SCALE_PLAUSIBLE_LOW <= scale <= DEPTH_SCALE_PLAUSIBLE_HIGH:
        return Check("depth_scale", STATUS_PASS, f"{scale} m/unit (D405 is typically {D405_DEPTH_SCALE_M_PER_UNIT})")
    return Check(
        "depth_scale",
        STATUS_WARN,
        f"{scale} m/unit — outside the plausible D405 range "
        f"[{DEPTH_SCALE_PLAUSIBLE_LOW}, {DEPTH_SCALE_PLAUSIBLE_HIGH}]",
        "confirm this recording came from a D405 and depth_scale_m_per_unit was read from "
        "depth_sensor.get_depth_scale(), not hard-coded",
    )


def _check_usb_type(session_info: dict[str, Any]) -> Check:
    usb_type = str(session_info.get("device", {}).get("usb_type", "unknown"))
    if usb_type == "unknown":
        return Check("usb_type", STATUS_WARN, "unknown", "device.usb_type absent from session.json")
    if usb_type.strip().startswith("2"):
        return Check(
            "usb_type",
            STATUS_WARN,
            usb_type,
            "recorded over a USB-2 link — expect reduced resolution/fps and higher drop rate; "
            "re-record over a USB-3 port and cable",
        )
    return Check("usb_type", STATUS_PASS, usb_type)


@dataclass
class _FrameStats:
    frame_indices: list[int]
    color_timestamps: list[float]
    depth_timestamps: list[float]
    color_frame_numbers: list[int]
    depth_frame_numbers: list[int]
    invalid_depth_fractions: list[float]


def _collect_frame_stats(reader: RecordingReader) -> _FrameStats:
    stats = _FrameStats([], [], [], [], [], [])
    for frame in reader:
        stats.frame_indices.append(frame.frame_index)
        stats.color_timestamps.append(frame.color_timestamp)
        stats.depth_timestamps.append(frame.depth_timestamp)
        stats.color_frame_numbers.append(frame.color_frame_number)
        stats.depth_frame_numbers.append(frame.depth_frame_number)
        invalid = float(np.count_nonzero(frame.depth_u16 == 0)) / frame.depth_u16.size
        stats.invalid_depth_fractions.append(invalid)
    return stats


def _check_monotonicity(stats: _FrameStats) -> list[Check]:
    checks: list[Check] = []
    for label, series in (("color_timestamp", stats.color_timestamps), ("depth_timestamp", stats.depth_timestamps)):
        violations = [i for i in range(1, len(series)) if series[i] < series[i - 1]]
        if violations:
            checks.append(
                Check(
                    f"{label}_monotonicity",
                    STATUS_FAIL,
                    f"non-monotonic at frame index(es) {violations[:5]}"
                    + (" (+more)" if len(violations) > 5 else ""),
                    "the capture clock jumped backwards or frames were written out of order — "
                    "re-record; if this is a merged/edited session, do not hand-edit sidecars",
                )
            )
        else:
            checks.append(Check(f"{label}_monotonicity", STATUS_PASS, f"{len(series)} timestamps, non-decreasing"))
    return checks


def _check_color_depth_delta(stats: _FrameStats) -> Check:
    if not stats.color_timestamps:
        return Check("color_depth_delta", STATUS_WARN, "no frames to check")
    deltas = np.abs(np.array(stats.depth_timestamps) - np.array(stats.color_timestamps))
    mean_delta = float(np.mean(deltas))
    max_delta = float(np.max(deltas))
    value = f"mean={mean_delta:.4f}s max={max_delta:.4f}s over {len(deltas)} frames"
    if max_delta > TIMESTAMP_DELTA_FAIL_S:
        return Check(
            "color_depth_delta",
            STATUS_FAIL,
            value,
            f"colour/depth timestamps drift by up to {max_delta:.3f}s — streams are not aligned in "
            "time; check the capture pipeline pairs frames from the same rs.composite_frame",
        )
    if max_delta > TIMESTAMP_DELTA_WARN_S:
        return Check(
            "color_depth_delta",
            STATUS_WARN,
            value,
            f"colour/depth timestamps drift by up to {max_delta:.3f}s — usable but worth checking "
            "the capture pipeline's frame pairing",
        )
    return Check("color_depth_delta", STATUS_PASS, value)


def _check_dropped_frames(session_info: dict[str, Any], stats: _FrameStats) -> list[Check]:
    checks: list[Check] = []

    declared_count = session_info.get("frame_count")
    actual_count = len(stats.frame_indices)
    if declared_count is not None and declared_count != actual_count:
        checks.append(
            Check(
                "frame_count_consistency",
                STATUS_FAIL,
                f"session.json declares frame_count={declared_count}, but {actual_count} sidecar(s) "
                "found on disk",
                "the recording is truncated or session.json was written before all frames were "
                "flushed — re-record, always closing the RecordingWriter after the last write_frame",
            )
        )
    else:
        checks.append(Check("frame_count_consistency", STATUS_PASS, f"{actual_count} frames"))

    if actual_count > 1:
        expected = list(range(stats.frame_indices[0], stats.frame_indices[0] + actual_count))
        if stats.frame_indices != expected:
            checks.append(
                Check(
                    "frame_index_gaps",
                    STATUS_FAIL,
                    "frame_index sequence has gaps or is out of order",
                    "sidecar frames/*.json were deleted, renamed or reordered out-of-band — "
                    "re-record; do not edit a recording directory by hand",
                )
            )
        else:
            checks.append(Check("frame_index_gaps", STATUS_PASS, "contiguous"))

    for label, numbers in (
        ("color_frame_number", stats.color_frame_numbers),
        ("depth_frame_number", stats.depth_frame_numbers),
    ):
        if len(numbers) < 2:
            continue
        gaps = sum(max(0, b - a - 1) for a, b in zip(numbers, numbers[1:]) if b >= a)
        dropped_fraction = gaps / max(1, len(numbers))
        value = f"{gaps} dropped hardware frame(s) inferred from {label} gaps ({dropped_fraction:.1%})"
        if dropped_fraction > DROPPED_FRAME_FAIL_FRACTION:
            checks.append(
                Check(
                    f"{label}_dropped_frames",
                    STATUS_FAIL,
                    value,
                    "USB bandwidth/CPU load is too high for the configured resolution/fps — use a "
                    "USB-3 port, lower fps/resolution, or reduce host load, then re-record",
                )
            )
        elif dropped_fraction > DROPPED_FRAME_WARN_FRACTION:
            checks.append(
                Check(
                    f"{label}_dropped_frames",
                    STATUS_WARN,
                    value,
                    "a small number of hardware frames were dropped — usable, but check the USB "
                    "link and host load if this recording matters",
                )
            )
        else:
            checks.append(Check(f"{label}_dropped_frames", STATUS_PASS, value))

    return checks


def _check_invalid_depth(stats: _FrameStats) -> Check:
    if not stats.invalid_depth_fractions:
        return Check("invalid_depth_fraction", STATUS_WARN, "no frames to check")
    fractions = np.array(stats.invalid_depth_fractions)
    mean_fraction = float(np.mean(fractions))
    max_fraction = float(np.max(fractions))
    worst_index = stats.frame_indices[int(np.argmax(fractions))]
    value = f"mean={mean_fraction:.1%} max={max_fraction:.1%} (worst frame_index={worst_index})"
    if max_fraction > INVALID_DEPTH_FAIL_FRACTION:
        return Check(
            "invalid_depth_fraction",
            STATUS_FAIL,
            value,
            "one or more frames are mostly invalid depth (depth_u16 == 0) — check D405 range, "
            "lighting, and IR interference/occlusion at capture time; re-record",
        )
    if max_fraction > INVALID_DEPTH_WARN_FRACTION:
        return Check(
            "invalid_depth_fraction",
            STATUS_WARN,
            value,
            "some frames have a high invalid-depth fraction — check subject distance/reflectivity",
        )
    return Check("invalid_depth_fraction", STATUS_PASS, value)


def run_checks(root: Path) -> tuple[list[Check], int]:
    """Return (checks, exit_code) for the recording under `root`."""
    root = Path(root)
    if not root.is_dir() or not (root / "session.json").is_file():
        return (
            [
                Check(
                    "recording_present",
                    STATUS_FAIL,
                    f"no session.json under {root}",
                    "point --recording at a directory written by crackvision.recording."
                    "RecordingWriter (docs/INTERFACES.md §3.14)",
                )
            ],
            EXIT_PRECONDITION,
        )

    try:
        reader = RecordingReader(root)
    except (FileNotFoundError, json.JSONDecodeError) as exc:
        return (
            [
                Check(
                    "recording_present",
                    STATUS_FAIL,
                    f"{type(exc).__name__}: {exc}",
                    "session.json is missing or malformed — re-record with RecordingWriter",
                )
            ],
            EXIT_PRECONDITION,
        )

    session_info = reader.session_info
    checks: list[Check] = []
    checks.extend(_check_metadata_completeness(session_info))
    checks.append(_check_depth_scale(session_info))
    checks.append(_check_usb_type(session_info))

    stats = _collect_frame_stats(reader)
    checks.extend(_check_monotonicity(stats))
    checks.append(_check_color_depth_delta(stats))
    checks.extend(_check_dropped_frames(session_info, stats))
    checks.append(_check_invalid_depth(stats))

    exit_code = EXIT_RUNTIME if any(c.status == STATUS_FAIL for c in checks) else EXIT_OK
    return checks, exit_code


def _print_table(checks: list[Check]) -> None:
    name_width = max(len("CHECK"), *(len(c.name) for c in checks))
    status_width = max(len("STATUS"), 4)
    print(f"{'CHECK':<{name_width}}  {'STATUS':<{status_width}}  VALUE")
    for check in checks:
        print(f"{check.name:<{name_width}}  {check.status:<{status_width}}  {check.value}")
        if check.status != STATUS_PASS and check.hint:
            print(f"   fix: {check.hint}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="validate_recording.py",
        description="Validate a crackvision.recording session with actionable diagnostics (CAM-03).",
    )
    parser.add_argument("recording", type=Path, help="path to a recording directory (session.json inside it)")
    add_common_args(parser)
    parser.add_argument("--json", action="store_true", help="print the JSON summary instead of the table")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"validate_recording: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging("validate_recording", cfg, verbose=args.verbose, quiet=args.quiet)

    checks, exit_code = run_checks(args.recording)

    log_by_status = {STATUS_PASS: logger.info, STATUS_WARN: logger.warning, STATUS_FAIL: logger.error}
    for check in checks:
        log_by_status[check.status]("%-28s %-4s %s", check.name, check.status, check.value)
        if check.status != STATUS_PASS and check.hint:
            logger.info("  fix: %s", check.hint)

    if args.json:
        print(json.dumps({"recording": str(args.recording), "checks": [c.to_dict() for c in checks]}, indent=2))
    else:
        _print_table(checks)

    if exit_code == EXIT_OK:
        status = "ok"
    elif exit_code == EXIT_PRECONDITION:
        status = "precondition"
    else:
        status = "failed"

    summary = RunSummary()
    for check in checks:
        summary.increment(check.status.lower())
        if check.status == STATUS_FAIL:
            summary.add_error(f"{check.name}: {check.value}")

    if args.dry_run:
        logger.info("dry-run: logs/validate_recording_latest.json not written")
        return exit_code

    latest_path = summary.write(cfg, "validate_recording", status, exit_code)
    data = json.loads(latest_path.read_text(encoding="utf-8"))
    data["recording"] = str(args.recording)
    data["checks"] = [c.to_dict() for c in checks]
    latest_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

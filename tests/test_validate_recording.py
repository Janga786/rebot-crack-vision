"""tests/test_validate_recording.py — diagnostics coverage for CAM-03."""

from __future__ import annotations

import importlib.util
import json
import sys
import time
from pathlib import Path

import numpy as np
import pytest

from crackvision.recording import CaptureFrame, RecordingWriter

_MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "validate_recording.py"
_spec = importlib.util.spec_from_file_location("validate_recording", _MODULE_PATH)
validate_recording = importlib.util.module_from_spec(_spec)
sys.modules["validate_recording"] = validate_recording
_spec.loader.exec_module(validate_recording)

from crackvision.logging_setup import EXIT_OK, EXIT_PRECONDITION, EXIT_RUNTIME  # noqa: E402

DEVICE = {"serial": "123456789012", "firmware": "5.16.0.1", "usb_type": "3.2"}
COLOR_CFG = {"width": 16, "height": 12, "fps": 30, "format": "rgb8"}
DEPTH_CFG = {"width": 16, "height": 12, "fps": 30, "format": "z16"}
COLOR_INTRINSICS = {
    "fx": 435.2,
    "fy": 435.2,
    "ppx": 8.0,
    "ppy": 6.0,
    "model": "Brown Conrady",
    "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
    "width": 16,
    "height": 12,
}
EXTRINSICS = {
    "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    "translation": [0.0148, 0.0, 0.0],
}


def _writer(root, *, device=None, depth_scale=0.0001):
    return RecordingWriter(
        root,
        session="sess01",
        device=device or DEVICE,
        color=COLOR_CFG,
        depth=DEPTH_CFG,
        color_intrinsics=COLOR_INTRINSICS,
        depth_scale_m_per_unit=depth_scale,
        extrinsics_depth_to_color=EXTRINSICS,
        aligned_to="color",
    )


def _frame(index, *, color_ts=None, depth_ts=None, color_fn=None, depth_fn=None, invalid_fraction=0.0):
    rng = np.random.default_rng(index)
    color = rng.integers(0, 256, size=(12, 16, 3), dtype=np.uint8)
    depth = rng.integers(1, 65535, size=(12, 16), dtype=np.uint16)
    n_invalid = int(round(invalid_fraction * depth.size))
    if n_invalid:
        flat = depth.reshape(-1)
        flat[:n_invalid] = 0
    return CaptureFrame(
        frame_index=index,
        color_rgb=color,
        depth_u16=depth,
        color_timestamp=1000.0 + index if color_ts is None else color_ts,
        color_timestamp_domain="hardware_clock",
        depth_timestamp=1000.0 + index if depth_ts is None else depth_ts,
        depth_timestamp_domain="hardware_clock",
        color_frame_number=index if color_fn is None else color_fn,
        depth_frame_number=index if depth_fn is None else depth_fn,
        host_monotonic_ns=time.monotonic_ns(),
        extra={},
    )


def _clean_recording(tmp_path: Path, n_frames: int = 5) -> Path:
    root = tmp_path / "sess01"
    with _writer(root) as writer:
        for i in range(n_frames):
            writer.write_frame(_frame(i))
    return root


# --- run_checks ---------------------------------------------------------------------------------


def test_clean_recording_passes_every_check(tmp_path):
    root = _clean_recording(tmp_path)
    checks, exit_code = validate_recording.run_checks(root)
    assert exit_code == EXIT_OK
    assert all(c.status != validate_recording.STATUS_FAIL for c in checks)


def test_missing_session_is_precondition(tmp_path):
    checks, exit_code = validate_recording.run_checks(tmp_path / "does_not_exist")
    assert exit_code == EXIT_PRECONDITION
    assert checks[0].status == validate_recording.STATUS_FAIL
    assert checks[0].hint


def test_missing_metadata_key_fails_with_hint(tmp_path):
    root = tmp_path / "sess01"
    with _writer(root) as writer:
        writer.write_frame(_frame(0))
    session_path = root / "session.json"
    payload = json.loads(session_path.read_text(encoding="utf-8"))
    del payload["depth_scale_m_per_unit"]
    session_path.write_text(json.dumps(payload), encoding="utf-8")

    checks, exit_code = validate_recording.run_checks(root)
    assert exit_code == EXIT_RUNTIME
    by_name = {c.name: c for c in checks}
    assert by_name["depth_scale"].status == validate_recording.STATUS_FAIL
    assert by_name["depth_scale"].hint


def test_implausible_depth_scale_warns(tmp_path):
    root = tmp_path / "sess01"
    with _writer(root, depth_scale=1.0) as writer:
        writer.write_frame(_frame(0))
    checks, exit_code = validate_recording.run_checks(root)
    by_name = {c.name: c for c in checks}
    assert by_name["depth_scale"].status == validate_recording.STATUS_WARN
    assert exit_code == EXIT_OK


def test_usb2_link_warns_with_fix(tmp_path):
    root = tmp_path / "sess01"
    usb2_device = dict(DEVICE, usb_type="2.1")
    with _writer(root, device=usb2_device) as writer:
        writer.write_frame(_frame(0))
    checks, _ = validate_recording.run_checks(root)
    by_name = {c.name: c for c in checks}
    assert by_name["usb_type"].status == validate_recording.STATUS_WARN
    assert "USB-3" in by_name["usb_type"].hint


def test_non_monotonic_timestamps_fail(tmp_path):
    root = tmp_path / "sess01"
    with _writer(root) as writer:
        writer.write_frame(_frame(0, color_ts=1000.0, depth_ts=1000.0))
        writer.write_frame(_frame(1, color_ts=999.0, depth_ts=1001.0))

    checks, exit_code = validate_recording.run_checks(root)
    assert exit_code == EXIT_RUNTIME
    by_name = {c.name: c for c in checks}
    assert by_name["color_timestamp_monotonicity"].status == validate_recording.STATUS_FAIL
    assert by_name["depth_timestamp_monotonicity"].status == validate_recording.STATUS_PASS


def test_large_color_depth_delta_fails(tmp_path):
    root = tmp_path / "sess01"
    with _writer(root) as writer:
        writer.write_frame(_frame(0, color_ts=1000.0, depth_ts=1000.0))
        writer.write_frame(_frame(1, color_ts=1001.0, depth_ts=1002.0))

    checks, exit_code = validate_recording.run_checks(root)
    assert exit_code == EXIT_RUNTIME
    by_name = {c.name: c for c in checks}
    assert by_name["color_depth_delta"].status == validate_recording.STATUS_FAIL
    assert "drift" in by_name["color_depth_delta"].hint


def test_dropped_hardware_frames_detected(tmp_path):
    root = tmp_path / "sess01"
    with _writer(root) as writer:
        writer.write_frame(_frame(0, color_fn=0, depth_fn=0))
        writer.write_frame(_frame(1, color_fn=1, depth_fn=1))
        writer.write_frame(_frame(2, color_fn=40, depth_fn=2))

    checks, exit_code = validate_recording.run_checks(root)
    assert exit_code == EXIT_RUNTIME
    by_name = {c.name: c for c in checks}
    assert by_name["color_frame_number_dropped_frames"].status == validate_recording.STATUS_FAIL


def test_frame_count_mismatch_fails(tmp_path):
    root = tmp_path / "sess01"
    with _writer(root) as writer:
        writer.write_frame(_frame(0))
        writer.write_frame(_frame(1))
    session_path = root / "session.json"
    payload = json.loads(session_path.read_text(encoding="utf-8"))
    payload["frame_count"] = 99
    session_path.write_text(json.dumps(payload), encoding="utf-8")

    checks, exit_code = validate_recording.run_checks(root)
    assert exit_code == EXIT_RUNTIME
    by_name = {c.name: c for c in checks}
    assert by_name["frame_count_consistency"].status == validate_recording.STATUS_FAIL


def test_invalid_depth_fraction_fails(tmp_path):
    root = tmp_path / "sess01"
    with _writer(root) as writer:
        writer.write_frame(_frame(0, invalid_fraction=0.95))

    checks, exit_code = validate_recording.run_checks(root)
    assert exit_code == EXIT_RUNTIME
    by_name = {c.name: c for c in checks}
    assert by_name["invalid_depth_fraction"].status == validate_recording.STATUS_FAIL


def test_invalid_depth_fraction_warns_below_fail_threshold(tmp_path):
    root = tmp_path / "sess01"
    with _writer(root) as writer:
        writer.write_frame(_frame(0, invalid_fraction=0.3))

    checks, exit_code = validate_recording.run_checks(root)
    by_name = {c.name: c for c in checks}
    assert by_name["invalid_depth_fraction"].status == validate_recording.STATUS_WARN
    assert exit_code == EXIT_OK


# --- main() CLI, exit codes and --json ----------------------------------------------------------


def test_main_exit_ok_on_clean_recording(tmp_project, tmp_path, capsys):
    root = _clean_recording(tmp_path)
    exit_code = validate_recording.main(["--root", str(tmp_project.root), str(root)])
    assert exit_code == EXIT_OK
    out = capsys.readouterr().out
    assert "PASS" in out


def test_main_exit_precondition_on_missing_recording(tmp_project, tmp_path, capsys):
    exit_code = validate_recording.main(["--root", str(tmp_project.root), str(tmp_path / "missing")])
    assert exit_code == EXIT_PRECONDITION


def test_main_json_output_is_parseable(tmp_project, tmp_path, capsys):
    root = _clean_recording(tmp_path)
    exit_code = validate_recording.main(["--root", str(tmp_project.root), "--json", str(root)])
    assert exit_code == EXIT_OK
    payload = json.loads(capsys.readouterr().out)
    assert payload["recording"] == str(root)
    assert all("name" in c and "status" in c for c in payload["checks"])


def test_main_writes_run_summary_json(tmp_project, tmp_path, capsys):
    root = _clean_recording(tmp_path)
    validate_recording.main(["--root", str(tmp_project.root), str(root)])
    latest = tmp_project.paths["logs"] / "validate_recording_latest.json"
    assert latest.is_file()
    summary = json.loads(latest.read_text(encoding="utf-8"))
    assert summary["status"] == "ok"
    assert summary["exit_code"] == EXIT_OK
    assert summary["checks"]


def test_main_dry_run_skips_summary(tmp_project, tmp_path, capsys):
    root = _clean_recording(tmp_path)
    latest = tmp_project.paths["logs"] / "validate_recording_latest.json"
    assert not latest.exists()
    validate_recording.main(["--root", str(tmp_project.root), "--dry-run", str(root)])
    assert not latest.exists()

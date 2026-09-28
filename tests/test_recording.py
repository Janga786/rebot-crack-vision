"""tests/test_recording.py — lossless round trip and replay behaviour for CAM-02."""

from __future__ import annotations

import time

import numpy as np
import pytest

from crackvision.recording import CaptureFrame, RecordingReader, RecordingWriter, ReplaySource

DEVICE = {"serial": "123456789012", "firmware": "5.16.0.1", "usb_type": "3.2"}
COLOR_CFG = {"width": 64, "height": 48, "fps": 30, "format": "rgb8"}
DEPTH_CFG = {"width": 64, "height": 48, "fps": 30, "format": "z16"}
COLOR_INTRINSICS = {
    "fx": 435.2,
    "fy": 435.2,
    "ppx": 32.0,
    "ppy": 24.0,
    "model": "Brown Conrady",
    "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
    "width": 64,
    "height": 48,
}
EXTRINSICS = {
    "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    "translation": [0.0148, 0.0, 0.0],
}


def _make_frame(index: int, seed: int, height: int = 48, width: int = 64) -> CaptureFrame:
    rng = np.random.default_rng(seed)
    color = rng.integers(0, 256, size=(height, width, 3), dtype=np.uint8)
    depth = rng.integers(0, 65535, size=(height, width), dtype=np.uint16)
    return CaptureFrame(
        frame_index=index,
        color_rgb=color,
        depth_u16=depth,
        color_timestamp=1000.0 + index,
        color_timestamp_domain="hardware_clock",
        depth_timestamp=1000.05 + index,
        depth_timestamp_domain="hardware_clock",
        color_frame_number=index,
        depth_frame_number=index,
        host_monotonic_ns=time.monotonic_ns(),
        extra={"exposure_us": 8000, "gain": 16},
    )


def _writer(root, session="sess01", *, depth_scale=0.0001):
    return RecordingWriter(
        root,
        session=session,
        device=DEVICE,
        color=COLOR_CFG,
        depth=DEPTH_CFG,
        color_intrinsics=COLOR_INTRINSICS,
        depth_scale_m_per_unit=depth_scale,
        extrinsics_depth_to_color=EXTRINSICS,
        aligned_to="color",
    )


def test_round_trip_is_bit_exact(tmp_path):
    root = tmp_path / "rec"
    writer = _writer(root)
    frames = [_make_frame(i, seed=100 + i) for i in range(3)]
    for frame in frames:
        writer.write_frame(frame)
    writer.close()

    reader = RecordingReader(root)
    assert len(reader) == 3
    for original, replayed in zip(frames, reader):
        assert replayed.color_rgb.dtype == np.uint8
        assert replayed.depth_u16.dtype == np.uint16
        assert np.array_equal(replayed.color_rgb, original.color_rgb)
        assert np.array_equal(replayed.depth_u16, original.depth_u16)
        assert replayed.color_rgb.shape == original.color_rgb.shape
        assert replayed.depth_u16.shape == original.depth_u16.shape


def test_context_manager_closes_and_writes_session_json(tmp_path):
    root = tmp_path / "rec"
    frames = [_make_frame(i, seed=200 + i) for i in range(2)]
    with _writer(root) as writer:
        for frame in frames:
            writer.write_frame(frame)
    assert (root / "session.json").is_file()


def test_session_json_has_required_fields(tmp_path):
    root = tmp_path / "rec"
    with _writer(root, depth_scale=0.0001) as writer:
        writer.write_frame(_make_frame(0, seed=300))
        writer.write_frame(_make_frame(1, seed=301))

    reader = RecordingReader(root)
    info = reader.session_info
    assert info["schema_version"] == 1
    assert info["session"] == "sess01"
    assert info["device"] == DEVICE
    assert info["color"] == COLOR_CFG
    assert info["depth"] == DEPTH_CFG
    assert info["color_intrinsics"] == COLOR_INTRINSICS
    assert info["depth_scale_m_per_unit"] == pytest.approx(0.0001)
    assert info["extrinsics_depth_to_color"] == EXTRINSICS
    assert info["aligned_to"] == "color"
    assert info["frame_count"] == 2


def test_depth_scale_is_read_not_hardcoded(tmp_path):
    root = tmp_path / "rec"
    with _writer(root, depth_scale=0.001) as writer:
        writer.write_frame(_make_frame(0, seed=400))
    info = RecordingReader(root).session_info
    assert info["depth_scale_m_per_unit"] == pytest.approx(0.001)


def test_per_frame_sidecar_has_required_fields(tmp_path):
    root = tmp_path / "rec"
    with _writer(root) as writer:
        writer.write_frame(_make_frame(0, seed=500))

    sidecar_path = root / "frames" / "000000.json"
    import json

    payload = json.loads(sidecar_path.read_text())
    required = {
        "frame_index",
        "color_timestamp",
        "color_timestamp_domain",
        "depth_timestamp",
        "depth_timestamp_domain",
        "color_frame_number",
        "depth_frame_number",
        "host_monotonic_ns",
    }
    assert required <= payload.keys()
    assert payload["frame_index"] == 0


def test_missing_required_device_key_raises(tmp_path):
    root = tmp_path / "rec"
    with pytest.raises(ValueError):
        RecordingWriter(
            root,
            session="bad",
            device={"serial": "x"},
            color=COLOR_CFG,
            depth=DEPTH_CFG,
            color_intrinsics=COLOR_INTRINSICS,
            depth_scale_m_per_unit=0.0001,
            extrinsics_depth_to_color=EXTRINSICS,
        )


def test_wrong_shape_frame_is_rejected_not_resized(tmp_path):
    root = tmp_path / "rec"
    writer = _writer(root)
    bad_frame = _make_frame(0, seed=600, height=32, width=32)
    with pytest.raises(ValueError):
        writer.write_frame(bad_frame)
    writer.close()
    assert len(RecordingReader(root)) == 0


def test_capture_frame_rejects_wrong_dtype():
    with pytest.raises(ValueError):
        CaptureFrame(
            frame_index=0,
            color_rgb=np.zeros((4, 4, 3), dtype=np.float32),
            depth_u16=np.zeros((4, 4), dtype=np.uint16),
            color_timestamp=0.0,
            color_timestamp_domain="hardware_clock",
            depth_timestamp=0.0,
            depth_timestamp_domain="hardware_clock",
            color_frame_number=0,
            depth_frame_number=0,
            host_monotonic_ns=0,
        )


def test_replay_source_yields_capture_frames_deterministically(tmp_path):
    root = tmp_path / "rec"
    frames = [_make_frame(i, seed=700 + i) for i in range(4)]
    with _writer(root) as writer:
        for frame in frames:
            writer.write_frame(frame)

    source = ReplaySource(root)
    assert len(source) == 4
    assert source.session_info["frame_count"] == 4

    first_pass = list(source)
    second_pass = list(source)
    assert len(first_pass) == len(second_pass) == 4
    for a, b in zip(first_pass, second_pass):
        assert isinstance(a, CaptureFrame) and isinstance(b, CaptureFrame)
        assert a.frame_index == b.frame_index
        assert np.array_equal(a.color_rgb, b.color_rgb)
        assert np.array_equal(a.depth_u16, b.depth_u16)

    for original, replayed in zip(frames, first_pass):
        assert np.array_equal(original.color_rgb, replayed.color_rgb)
        assert np.array_equal(original.depth_u16, replayed.depth_u16)


def test_no_pyrealsense2_import_required(monkeypatch):
    """recording.py must work standalone — no camera stack needed for file-format round trips."""
    import sys

    monkeypatch.setitem(sys.modules, "pyrealsense2", None)
    import importlib

    import crackvision.recording as recording_module

    importlib.reload(recording_module)
    assert "pyrealsense2" not in recording_module.__dict__

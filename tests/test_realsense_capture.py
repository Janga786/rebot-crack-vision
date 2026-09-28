"""tests/test_realsense_capture.py — synthetic-mode and CLI-exit behaviour for TC-013."""

from __future__ import annotations

import json
import sys

import numpy as np
import pytest

from crackvision import realsense_capture as rc

REQUIRED_TOP_KEYS = {
    "schema_version",
    "session",
    "frame_index",
    "timestamp_utc",
    "device",
    "aligned_to",
    "warmup_frames",
    "depth_scale_m_per_unit",
    "color",
    "depth",
    "extrinsics_depth_to_color",
    "exposure_us",
    "gain",
    "synthetic",
    "files",
}
REQUIRED_STREAM_KEYS = {"width", "height", "fps", "format", "intrinsics"}
REQUIRED_INTRINSICS_KEYS = {"fx", "fy", "ppx", "ppy", "model", "coeffs"}
REQUIRED_EXTRINSICS_KEYS = {"rotation", "translation"}


def _run(tmp_root, args):
    (tmp_root / "config").mkdir(exist_ok=True)
    return rc.main(["--root", str(tmp_root), *args])


def test_import_lazy_without_pyrealsense2(monkeypatch):
    monkeypatch.setitem(sys.modules, "pyrealsense2", None)
    sys.modules.pop("crackvision.realsense_capture", None)
    import importlib

    module = importlib.import_module("crackvision.realsense_capture")
    assert module is not None


def test_synthetic_writes_expected_files(tmp_path):
    exit_code = _run(tmp_path, ["--synthetic", "3", "--session", "cachk"])
    assert exit_code == 0

    color_dir = tmp_path / "data" / "d405" / "color"
    depth_dir = tmp_path / "data" / "d405" / "depth"
    meta_dir = tmp_path / "data" / "d405" / "metadata"

    color_files = sorted(color_dir.glob("synth_cachk_*_color.png"))
    depth_files = sorted(depth_dir.glob("synth_cachk_*_depth.png"))
    meta_files = sorted(p for p in meta_dir.glob("synth_cachk_*.json") if p.name != "synth_cachk_session.json")
    session_files = list(meta_dir.glob("synth_cachk_session.json"))

    assert len(color_files) == 3
    assert len(depth_files) == 3
    assert len(meta_files) == 3
    assert len(session_files) == 1

    for path in color_files + depth_files:
        assert path.name.startswith("synth_")


def test_synthetic_metadata_matches_schema_and_marks_synthetic(tmp_path):
    _run(tmp_path, ["--synthetic", "2", "--session", "schema"])
    meta_path = tmp_path / "data" / "d405" / "metadata" / "synth_schema_000000.json"
    payload = json.loads(meta_path.read_text())

    assert REQUIRED_TOP_KEYS <= payload.keys()
    assert payload["synthetic"] is True
    assert payload["aligned_to"] == "color"
    for stream_key in ("color", "depth"):
        stream = payload[stream_key]
        assert REQUIRED_STREAM_KEYS <= stream.keys()
        assert REQUIRED_INTRINSICS_KEYS <= stream["intrinsics"].keys()
    assert REQUIRED_EXTRINSICS_KEYS <= payload["extrinsics_depth_to_color"].keys()
    assert isinstance(payload["depth_scale_m_per_unit"], float)


def test_synthetic_depth_png_roundtrips_uint16(tmp_path):
    rng = np.random.default_rng(42)
    arr = rng.integers(0, 65535, size=(64, 96), dtype=np.uint16)
    write_depth_png, read_depth_png = rc._load_depth_png_helpers()
    path = tmp_path / "roundtrip.png"
    write_depth_png(arr, path)
    back = read_depth_png(path)
    assert back.dtype == np.uint16
    assert np.array_equal(back, arr)


def test_synthetic_color_is_rgb_not_bgr(tmp_path):
    _run(tmp_path, ["--synthetic", "1", "--session", "rgbcheck"])
    from PIL import Image

    color_path = tmp_path / "data" / "d405" / "color" / "synth_rgbcheck_000000_color.png"
    arr = np.array(Image.open(color_path))
    expected = rc.synthetic_color_frame(848, 480, 0)
    assert np.array_equal(arr, expected)


def test_no_device_and_not_synthetic_exits_3(tmp_path, monkeypatch):
    class _FakeContext:
        def query_devices(self):
            return []

    class _FakeRS:
        context = _FakeContext

    monkeypatch.setattr(rc, "_import_rs", lambda: _FakeRS())
    exit_code = _run(tmp_path, [])
    assert exit_code == 3


def test_dry_run_writes_zero_files(tmp_path):
    exit_code = _run(tmp_path, ["--dry-run"])
    assert exit_code == 0
    d405_dir = tmp_path / "data" / "d405"
    assert not d405_dir.exists() or not any(d405_dir.rglob("*"))


@pytest.mark.camera
def test_real_capture_requires_device():
    pytest.importorskip("pyrealsense2")
    pytest.skip("requires an attached RealSense D405")

"""tests/test_realsense_import.py — RealSense import-guard and depth PNG round-trip (TC-012)."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

_SPEC = importlib.util.spec_from_file_location(
    "check_realsense", Path(__file__).resolve().parents[1] / "scripts" / "check_realsense.py"
)
_check_realsense = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _check_realsense
_SPEC.loader.exec_module(_check_realsense)
read_depth_png = _check_realsense.read_depth_png
write_depth_png = _check_realsense.write_depth_png


def test_depth_png_roundtrip_uint16(tmp_path):
    rng = np.random.default_rng(0)
    arr = rng.integers(0, 65535, size=(64, 96), dtype=np.uint16)
    path = tmp_path / "d.png"
    write_depth_png(arr, path)
    back = read_depth_png(path)
    assert back.dtype == np.uint16
    assert np.array_equal(back, arr)


def test_realsense_capture_import_is_lazy(monkeypatch):
    """crackvision.realsense_capture (TC-013) must import cleanly with pyrealsense2 absent."""
    crackvision_realsense_capture = pytest.importorskip("crackvision.realsense_capture")
    monkeypatch.setitem(sys.modules, "pyrealsense2", None)
    sys.modules.pop("crackvision.realsense_capture", None)
    import importlib

    importlib.import_module("crackvision.realsense_capture")
    assert crackvision_realsense_capture is not None


@pytest.mark.camera
def test_realsense_device_enumeration():
    """Real-hardware smoke test; skips itself when no camera is attached (none on this box)."""
    rs = pytest.importorskip("pyrealsense2")
    devices = rs.context().query_devices()
    if len(devices) == 0:
        pytest.skip("no RealSense device attached")
    assert len(devices) >= 1

"""tests/test_d405_live.py — the live D405 source against a fake pyrealsense2 (no camera)."""

from __future__ import annotations

import numpy as np
import pytest

from crackvision import d405_live as dl
from crackvision.recording import RecordingReader, RecordingWriter
from fake_rs import FakeDevice, FakeRS


def test_frames_are_capture_frames_rgb_seconds_and_counted():
    rs = FakeRS()
    with dl.LiveD405Source(848, 480, 30, warmup=5, rs=rs) as cam:
        frames = list(cam.frames(4))
    assert [f.frame_index for f in frames] == [0, 1, 2, 3]
    f = frames[0]
    assert f.color_rgb.shape == (480, 848, 3) and f.color_rgb.dtype == np.uint8
    assert tuple(f.color_rgb[0, 0]) == (200, 0, 0)            # bgr8 converted to RGB exactly once
    assert f.depth_u16.dtype == np.uint16 and f.depth_u16[0, 0] == 2000
    assert 999.0 < f.color_timestamp < 1001.0                 # librealsense ms -> s
    assert f.color_timestamp_domain == "hardware_clock"
    assert frames[1].color_frame_number == frames[0].color_frame_number + 1
    assert f.extra == {"exposure_us": 1500, "gain": 16}
    assert rs.pipelines[0].stopped                              # the context manager stops the pipeline


def test_warmup_frames_are_discarded():
    rs = FakeRS()
    with dl.LiveD405Source(848, 480, 30, warmup=10, rs=rs) as cam:
        first = next(cam.frames(1))
    assert first.color_frame_number == 11                       # 10 warm-up + the one read for intrinsics


def test_session_info_feeds_recording_writer(tmp_path):
    with dl.LiveD405Source(848, 480, 30, warmup=0, rs=FakeRS()) as cam:
        info = cam.session_info
        with RecordingWriter(tmp_path / "s", session="s", device=info["device"], color=info["color"],
                             depth=info["depth"], color_intrinsics=info["color_intrinsics"],
                             depth_scale_m_per_unit=info["depth_scale_m_per_unit"],
                             extrinsics_depth_to_color=info["extrinsics_depth_to_color"]) as w:
            for frame in cam.frames(3):
                w.write_frame(frame)
    reader = RecordingReader(tmp_path / "s")
    assert len(reader) == 3
    assert reader.session_info["device"]["usb_type"] == "3.2"
    assert reader.session_info["depth_scale_m_per_unit"] == 1e-4
    assert info["device"]["recommended_firmware"] == "5.17.0.10"


def test_no_device_and_wrong_serial_are_clear_errors():
    with pytest.raises(dl.D405Error, match="Micro-B"):
        dl.LiveD405Source(848, 480, 30, rs=FakeRS(devices=[])).start()
    with pytest.raises(dl.D405Error, match="serial 999"):
        dl.LiveD405Source(848, 480, 30, serial="999", rs=FakeRS()).start()


def test_unsupported_mode_explains_usb2():
    with pytest.raises(dl.D405Error, match="USB 2"):
        dl.LiveD405Source(1920, 1080, 30, rs=FakeRS()).start()


def test_frames_stopping_is_a_d405_error():
    rs = FakeRS(dead_after=3)
    with dl.LiveD405Source(848, 480, 30, warmup=0, rs=rs) as cam:
        with pytest.raises(dl.D405Error, match="USB"):
            list(cam.frames(10))


def test_device_helpers():
    rs = FakeRS(devices=[FakeDevice(usb="2.1", firmware="5.16.0.1")])
    (d,) = dl.list_devices(rs)
    assert d["usb_type"] == "2.1" and d["product_id"] == "0B5B"
    assert not dl.usb_is_3(d["usb_type"]) and dl.usb_is_3("3.2") and not dl.usb_is_3("unknown")
    assert dl.firmware_tuple("5.17.0.10") > dl.firmware_tuple("5.17.0.9") > dl.firmware_tuple("5.16.0.1")
    profiles = dl.supported_profiles(rs)
    assert "1280x720@30 rgb8" in profiles["color"] and "848x480@90 z16" in profiles["depth"]

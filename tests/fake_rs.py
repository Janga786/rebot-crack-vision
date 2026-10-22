"""tests/fake_rs.py — a minimal stand-in for `pyrealsense2`, enough for crackvision.d405_live and
crackvision.camera_pipeline: one D405 that streams aligned colour/depth with hardware timestamps."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np


class _Enum(SimpleNamespace):
    def __getattr__(self, name):          # any member exists; its value is its own name
        return name


camera_info = _Enum()
stream = _Enum()
format = _Enum()   # noqa: A001 — mirrors pyrealsense2.format
frame_metadata_value = _Enum()


class FakeDevice:
    def __init__(self, serial="123456789012", firmware="5.17.0.10", usb="3.2", recommended="5.17.0.10",
                 product_id="0B5B", depth_scale=1e-4):
        self.info = {"name": "Intel RealSense D405", "serial_number": serial, "firmware_version": firmware,
                     "recommended_firmware_version": recommended, "product_line": "D400", "product_id": product_id,
                     "usb_type_descriptor": usb}
        self.depth_scale = depth_scale

    def supports(self, key):
        return key in self.info

    def get_info(self, key):
        return self.info[key]

    def first_depth_sensor(self):
        return SimpleNamespace(get_depth_scale=lambda: self.depth_scale)

    def query_sensors(self):
        profiles = []
        for kind, fmt in (("color", "rgb8"), ("depth", "z16")):
            for w, h, fps in ((1280, 720, 30), (848, 480, 30), (848, 480, 90)):
                profiles.append(_Profile(kind, w, h, fps, fmt))
        return [SimpleNamespace(get_stream_profiles=lambda: profiles)]


class _Intr(SimpleNamespace):
    pass


class _Profile:
    def __init__(self, kind, w, h, fps=30, fmt="z16"):
        self.kind, self.w, self.h, self._fps, self.fmt = kind, w, h, fps, fmt

    def stream_type(self):
        return f"stream.{self.kind}"

    def format(self):
        return f"format.{self.fmt}"

    def fps(self):
        return self._fps

    def width(self):
        return self.w

    def height(self):
        return self.h

    def as_video_stream_profile(self):
        return self

    def get_intrinsics(self):
        return _Intr(fx=430.0, fy=430.0, ppx=self.w / 2, ppy=self.h / 2, model="distortion.brown_conrady",
                     coeffs=[0.0] * 5)

    def get_extrinsics_to(self, other):
        return SimpleNamespace(rotation=[1, 0, 0, 0, 1, 0, 0, 0, 1], translation=[0.0, 0.0, 0.0])


class _Frame:
    def __init__(self, data, ts_ms, number, profile, meta=None):
        self._data, self._ts, self._n, self._p, self._meta = data, ts_ms, number, profile, meta or {}

    def __bool__(self):
        return True

    def get_data(self):
        return self._data

    def get_timestamp(self):
        return self._ts

    def get_frame_timestamp_domain(self):
        return "timestamp_domain.hardware_clock"

    def get_frame_number(self):
        return self._n

    def get_profile(self):
        return self._p

    def supports_frame_metadata(self, key):
        return key in self._meta

    def get_frame_metadata(self, key):
        return self._meta[key]


class _Frameset:
    def __init__(self, color, depth):
        self.c, self.d = color, depth

    def get_color_frame(self):
        return self.c

    def get_depth_frame(self):
        return self.d


class FakePipeline:
    def __init__(self, world):
        self.world, self.n, self.started, self.stopped = world, 0, False, False

    def start(self, config):
        if config.streams.get("color", (0, 0, 0))[:2] not in self.world.modes:
            raise RuntimeError("Couldn't resolve requests")
        self.started, self.cfg = True, config
        return SimpleNamespace(get_device=lambda: self.world.device)

    def stop(self):
        self.stopped = True

    def wait_for_frames(self, timeout_ms=5000):
        if self.world.dead_after is not None and self.n >= self.world.dead_after:
            raise RuntimeError("Frame didn't arrive within 5000")
        w, h, _ = self.cfg.streams["color"]
        self.n += 1
        bgr = np.full((h, w, 3), (10, 20, 30), np.uint8)      # B, G, R
        bgr[:, : w // 2] = (0, 0, 200)                         # left half red in RGB
        depth = np.full((h, w), 2000, np.uint16)
        ts = 1_000_000.0 + 33.333 * self.n                     # librealsense: milliseconds
        return _Frameset(_Frame(bgr, ts, self.n, _Profile("color", w, h), {"actual_exposure": 1500, "gain_level": 16}),
                         _Frame(depth, ts, self.n, _Profile("depth", w, h)))


class FakeConfig:
    def __init__(self):
        self.streams, self.device = {}, None

    def enable_device(self, serial):
        self.device = serial

    def enable_stream(self, kind, w, h, fmt, fps):
        self.streams[kind] = (w, h, fps)


class FakeRS:
    """`rs` replacement: FakeRS(devices=[FakeDevice()]). `modes` = resolutions the camera accepts."""

    camera_info = camera_info
    stream = stream
    format = format
    frame_metadata_value = frame_metadata_value

    def __init__(self, devices=None, modes=((1280, 720), (848, 480), (640, 480)), dead_after=None):
        self.devices = [FakeDevice()] if devices is None else devices
        self.device = self.devices[0] if self.devices else None
        self.modes, self.dead_after = set(modes), dead_after
        self.pipelines = []
        self.__version__ = "2.58.4"

    def context(self):
        return SimpleNamespace(query_devices=lambda: list(self.devices))

    def pipeline(self):
        p = FakePipeline(self)
        self.pipelines.append(p)
        return p

    def config(self):
        return FakeConfig()

    def align(self, to):
        return SimpleNamespace(process=lambda fs: fs)

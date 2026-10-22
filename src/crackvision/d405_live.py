"""crackvision.d405_live — the live D405 frame source: a real camera yielding `CaptureFrame`s.

`crackvision.recording` (CAM-02, docs/INTERFACES.md §3.14) defines `CaptureFrame` as the frame type
"every crackvision D405 source — live or recorded — yields", and ships the recorded side
(`RecordingWriter`, `RecordingReader`, `ReplaySource`). This module is the live side: it opens the
D405 with `pyrealsense2`, aligns depth to colour (`rs.align(rs.stream.color)`, the frame invariant
§0.5), and yields the same `CaptureFrame`s, so a live session can be written with `RecordingWriter`,
checked with `scripts/validate_recording.py`, and consumed by any code written against `ReplaySource`.

Frame timestamps are in seconds (librealsense reports milliseconds; `scripts/validate_recording.py`'s
thresholds are in seconds). `session_info` carries exactly what `RecordingWriter` needs (device incl. `usb_type`, stream configs,
colour intrinsics, the depth scale read from the device — never guessed, docs/RISKS.md R-09 — and the
depth->colour extrinsics), plus the facts CAM-01 records (product line, the SDK's recommended firmware).

`pyrealsense2` is imported lazily, and every call goes through an injectable `rs` module, so the
logic here is tested with a fake (tests/test_d405_live.py) on a machine with no camera.
"""

from __future__ import annotations

import logging
import time
from typing import Any, Iterator

import numpy as np

from crackvision.realsense_capture import (
    _device_field,
    _extrinsics_dict,
    _frame_metadata_value,
    _import_rs,
    _intrinsics_dict,
    _wait_for_frames_with_retry,
)
from crackvision.recording import CaptureFrame

D405_PRODUCT_ID = "0B5B"            # USB PID of the D405 (Intel VID 8086)
HD = (1280, 720)                    # the D405's full colour/depth resolution
USB_HINT = ("The D405 has a USB 3 Micro-B socket: use a USB-A to USB 3.0 Micro-B cable (2 m or shorter, ideally "
            "screw-locking; the camera-only kit ships without one) on a rear USB 3 port with no hub. Plug the camera "
            "end first, then push the host end in quickly and firmly (a slow insertion can enumerate as USB 2).")
NO_DEVICE_HINT = ("No RealSense device found. " + USB_HINT + " Then `lsusb -d 8086:0b5b`: if it is listed but this "
                  "still fails, the udev rule is missing (docs/host/DEVICE_ACCESS.md) or another process holds it.")
MIN_GOOD_FIRMWARE = (5, 17, 0, 9)   # D405 auto-exposure fixes (dark colour, AE region); 5.17.0.10 is the newest D405 FW


class D405Error(RuntimeError):
    """The camera is missing, refused the requested streams, or stopped delivering frames."""


def _timestamp_domain(frame: Any) -> str:
    try:
        return str(frame.get_frame_timestamp_domain()).split(".")[-1]
    except Exception:  # noqa: BLE001
        return "unknown"


class LiveD405Source:
    """Context manager over one D405: `with LiveD405Source(848, 480, 30) as cam: for f in cam.frames(30): ...`

    `warmup` frames are read and discarded after start (auto-exposure settles; docs/D405_TEST_PLAN.md).
    `serial` picks one camera when several are attached."""

    def __init__(
        self,
        width: int,
        height: int,
        fps: int,
        *,
        warmup: int = 30,
        align: bool = True,
        serial: str | None = None,
        rs: Any = None,
        logger: logging.Logger | None = None,
    ) -> None:
        self.width, self.height, self.fps = int(width), int(height), int(fps)
        self.warmup = max(0, int(warmup))
        self.align_enabled = align
        self.serial = serial
        self._rs = rs
        self.log = logger or logging.getLogger("d405_live")
        self._pipeline = None
        self._align = None
        self._started = False
        self._frame_index = 0
        self._pending = None                # the frameset start() read for its intrinsics, yielded first
        self.session_info: dict[str, Any] = {}

    # ---- lifecycle -----------------------------------------------------------------------------------

    def __enter__(self) -> "LiveD405Source":
        self.start()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.stop()

    def start(self) -> None:
        rs = self._rs = self._rs or _import_rs()
        devices = list(rs.context().query_devices())
        if self.serial:
            devices = [d for d in devices if _device_field(d, rs, "serial_number") == self.serial]
        if not devices:
            raise D405Error(NO_DEVICE_HINT if not self.serial else f"no RealSense device with serial {self.serial}")
        device = devices[0]
        serial = _device_field(device, rs, "serial_number")

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        try:
            profile = pipeline.start(config)
        except Exception as exc:  # noqa: BLE001 — unsupported resolution/fps, or a USB 2 link
            raise D405Error(
                f"the D405 refused {self.width}x{self.height}@{self.fps}: {type(exc).__name__}: {exc}. "
                "At 1280x720 it runs at 5/15/30 fps, at 848x480 and below up to 90. On a USB 2 link it only offers "
                "reduced modes: " + USB_HINT
            ) from exc
        self._pipeline = pipeline
        self._started = True
        dev = profile.get_device()
        try:
            depth_scale = float(dev.first_depth_sensor().get_depth_scale())
        except Exception as exc:  # noqa: BLE001 — never substitute a guess (docs/RISKS.md R-09)
            self.stop()
            raise D405Error(f"get_depth_scale() failed ({exc}); refusing to guess a depth scale") from exc
        self._align = rs.align(rs.stream.color) if self.align_enabled else None

        for _ in range(self.warmup):
            self._wait()

        frameset = self._wait()
        color, depth = frameset.get_color_frame(), frameset.get_depth_frame()
        if not color or not depth:
            self.stop()
            raise D405Error("the first frameset after warm-up had no colour or no depth frame")
        color_intr = _intrinsics_dict(color.get_profile()) | {"width": self.width, "height": self.height}
        self.session_info = {
            "device": {
                "name": _device_field(dev, rs, "name"),
                "serial": _device_field(dev, rs, "serial_number"),
                "firmware": _device_field(dev, rs, "firmware_version"),
                "recommended_firmware": _device_field(dev, rs, "recommended_firmware_version"),
                "product_line": _device_field(dev, rs, "product_line"),
                "product_id": _device_field(dev, rs, "product_id"),
                "usb_type": _device_field(dev, rs, "usb_type_descriptor"),
            },
            "color": {"width": self.width, "height": self.height, "fps": self.fps, "format": "rgb8"},
            "depth": {"width": self.width, "height": self.height, "fps": self.fps, "format": "z16"},
            "color_intrinsics": color_intr,
            "depth_scale_m_per_unit": depth_scale,
            "extrinsics_depth_to_color": _extrinsics_dict(depth.get_profile(), color.get_profile()),
            "aligned_to": "color" if self.align_enabled else None,
            "warmup_frames": self.warmup,
        }
        self._pending = frameset

    def stop(self) -> None:
        if self._pipeline is not None and self._started:
            try:
                self._pipeline.stop()
            except Exception:  # noqa: BLE001
                pass
        self._started = False

    # ---- frames --------------------------------------------------------------------------------------

    def _wait(self):
        try:
            frameset = _wait_for_frames_with_retry(self._pipeline, self.log)
        except RuntimeError as exc:
            raise D405Error(str(exc)) from exc
        return self._align.process(frameset) if self._align is not None else frameset

    def _to_capture_frame(self, frameset: Any) -> CaptureFrame | None:
        rs = self._rs
        color, depth = frameset.get_color_frame(), frameset.get_depth_frame()
        if not color or not depth:
            return None
        color_rgb = np.ascontiguousarray(np.asanyarray(color.get_data())[:, :, ::-1])   # bgr8 -> RGB, once
        depth_u16 = np.asanyarray(depth.get_data()).astype(np.uint16, copy=True)
        frame = CaptureFrame(
            frame_index=self._frame_index,
            color_rgb=color_rgb,
            depth_u16=depth_u16,
            color_timestamp=float(color.get_timestamp()) / 1000.0,     # librealsense ms -> s (validator units)
            color_timestamp_domain=_timestamp_domain(color),
            depth_timestamp=float(depth.get_timestamp()) / 1000.0,
            depth_timestamp_domain=_timestamp_domain(depth),
            color_frame_number=int(color.get_frame_number()),
            depth_frame_number=int(depth.get_frame_number()),
            host_monotonic_ns=time.monotonic_ns(),
            extra={"exposure_us": _frame_metadata_value(color, rs, "actual_exposure"),
                   "gain": _frame_metadata_value(color, rs, "gain_level")},
        )
        self._frame_index += 1
        return frame

    def frames(self, n: int | None = None) -> Iterator[CaptureFrame]:
        """Yield n aligned frames (forever if n is None). A frameset missing a stream is skipped."""
        if not self._started:
            raise D405Error("LiveD405Source is not started")
        produced = 0
        while n is None or produced < n:
            frameset, self._pending = (self._pending, None) if self._pending is not None else (self._wait(), None)
            frame = self._to_capture_frame(frameset)
            if frame is None:
                self.log.warning("frameset without colour or depth, skipped")
                continue
            produced += 1
            yield frame


def list_devices(rs: Any = None) -> list[dict[str, str]]:
    """Every attached RealSense device with the fields CAM-01 records (empty list if none)."""
    rs = rs or _import_rs()
    out = []
    for d in rs.context().query_devices():
        out.append({k: _device_field(d, rs, f) for k, f in (
            ("name", "name"), ("serial", "serial_number"), ("firmware", "firmware_version"),
            ("recommended_firmware", "recommended_firmware_version"), ("product_line", "product_line"),
            ("product_id", "product_id"), ("usb_type", "usb_type_descriptor"))})
    return out


def supported_profiles(rs: Any = None, serial: str | None = None) -> dict[str, list[str]]:
    """{"color": ["1280x720@30 bgr8", ...], "depth": [...]} for the first (or the given) device."""
    rs = rs or _import_rs()
    out: dict[str, list[str]] = {"color": [], "depth": []}
    for d in rs.context().query_devices():
        if serial and _device_field(d, rs, "serial_number") != serial:
            continue
        for sensor in d.query_sensors():
            for p in sensor.get_stream_profiles():
                kind = str(p.stream_type()).split(".")[-1]
                if kind not in out:
                    continue
                try:
                    v = p.as_video_stream_profile()
                    entry = f"{v.width()}x{v.height()}@{p.fps()} {str(p.format()).split('.')[-1]}"
                except Exception:  # noqa: BLE001
                    continue
                if entry not in out[kind]:
                    out[kind].append(entry)
        break
    return out


def usb_is_3(usb_type: str) -> bool:
    """'3.2' / '3.1' / '3.0' -> True; '2.1' / '2.0' / 'unknown' -> False."""
    try:
        return float(usb_type) >= 3.0
    except (TypeError, ValueError):
        return False


def firmware_tuple(version: str) -> tuple[int, ...]:
    try:
        return tuple(int(x) for x in str(version).split("."))
    except ValueError:
        return ()

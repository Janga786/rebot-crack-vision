"""crackvision.recording — lossless RGB-D recording + deterministic replay. Built by CAM-02.

`RecordingWriter` takes `CaptureFrame` objects (the frame type every crackvision D405 source — live
or recorded — yields) and writes them to a bit-exact on-disk session: one `session.json` (device,
stream configs, colour intrinsics, `depth_scale_m_per_unit`, depth->colour extrinsics, alignment
target, frame count — docs/INTERFACES.md #3.14) plus, per frame, a lossless colour PNG, a lossless
16-bit depth PNG and a JSON sidecar (timestamps, timestamp domains, frame numbers, host monotonic
clock). `RecordingReader` reads a session back frame-by-frame; `ReplaySource` wraps it as a
no-camera, deterministic drop-in for a live source — same `CaptureFrame` type, same order, every
run. This module never imports `pyrealsense2`, so it works on any machine with no RealSense stack
and no camera attached.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator

import numpy as np
from PIL import Image

SCHEMA_VERSION = 1
REQUIRED_DEVICE_KEYS = ("serial", "firmware", "usb_type")
REQUIRED_STREAM_KEYS = ("width", "height", "fps", "format")
REQUIRED_INTRINSICS_KEYS = ("fx", "fy", "ppx", "ppy", "model", "coeffs", "width", "height")
REQUIRED_EXTRINSICS_KEYS = ("rotation", "translation")


@dataclass(frozen=True)
class CaptureFrame:
    """One aligned colour/depth capture — the frame type every crackvision D405 source yields,
    live or replayed. `depth_u16[r, c]` corresponds to `color_rgb[r, c]` (docs/INTERFACES.md #0.5)."""

    frame_index: int
    color_rgb: np.ndarray  # uint8, (H, W, 3)
    depth_u16: np.ndarray  # uint16, (H, W), raw z16 device units
    color_timestamp: float
    color_timestamp_domain: str
    depth_timestamp: float
    depth_timestamp_domain: str
    color_frame_number: int
    depth_frame_number: int
    host_monotonic_ns: int
    extra: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.color_rgb.dtype != np.uint8 or self.color_rgb.ndim != 3 or self.color_rgb.shape[2] != 3:
            raise ValueError(
                f"color_rgb must be uint8 with shape (H, W, 3), got dtype={self.color_rgb.dtype} "
                f"shape={self.color_rgb.shape}"
            )
        if self.depth_u16.dtype != np.uint16 or self.depth_u16.ndim != 2:
            raise ValueError(
                f"depth_u16 must be uint16 with shape (H, W), got dtype={self.depth_u16.dtype} "
                f"shape={self.depth_u16.shape}"
            )


def _write_depth_png(arr: np.ndarray, path: Path) -> None:
    """Write a uint16 depth array as a lossless 16-bit grayscale PNG."""
    if arr.dtype != np.uint16:
        raise ValueError(f"expected uint16 array, got {arr.dtype}")
    img = Image.new("I;16", (arr.shape[1], arr.shape[0]))
    img.frombytes(arr.tobytes())
    img.save(path)


def _read_depth_png(path: Path) -> np.ndarray:
    """Read a 16-bit grayscale PNG back into a uint16 array."""
    with Image.open(path) as img:
        return np.array(img.convert("I;16"), dtype=np.uint16)


def _require_keys(payload: dict[str, Any], keys: tuple[str, ...], label: str) -> None:
    missing = [key for key in keys if key not in payload]
    if missing:
        raise ValueError(f"{label} is missing required key(s): {missing}")


class RecordingWriter:
    """Writes a `CaptureFrame` session losslessly to disk.

    Layout under `root`:
        session.json           -- schema_version, device, stream configs, colour intrinsics,
                                   depth_scale_m_per_unit, extrinsics_depth_to_color, aligned_to,
                                   frame_count (docs/INTERFACES.md #3.14)
        color/{NNNNNN}_color.png -- uint8 RGB, lossless PNG
        depth/{NNNNNN}_depth.png -- uint16 z16 units, lossless 16-bit PNG (never resized/scaled)
        frames/{NNNNNN}.json    -- per-frame sidecar
    """

    def __init__(
        self,
        root: Path,
        *,
        session: str,
        device: dict[str, str],
        color: dict[str, Any],
        depth: dict[str, Any],
        color_intrinsics: dict[str, Any],
        depth_scale_m_per_unit: float,
        extrinsics_depth_to_color: dict[str, list[float]],
        aligned_to: str | None = "color",
    ) -> None:
        _require_keys(device, REQUIRED_DEVICE_KEYS, "device")
        _require_keys(color, REQUIRED_STREAM_KEYS, "color config")
        _require_keys(depth, REQUIRED_STREAM_KEYS, "depth config")
        _require_keys(color_intrinsics, REQUIRED_INTRINSICS_KEYS, "color_intrinsics")
        _require_keys(extrinsics_depth_to_color, REQUIRED_EXTRINSICS_KEYS, "extrinsics_depth_to_color")

        self.root = Path(root)
        self.session = session
        self._device = dict(device)
        self._color = dict(color)
        self._depth = dict(depth)
        self._color_intrinsics = dict(color_intrinsics)
        self._depth_scale_m_per_unit = float(depth_scale_m_per_unit)
        self._extrinsics = dict(extrinsics_depth_to_color)
        self._aligned_to = aligned_to

        self._color_dir = self.root / "color"
        self._depth_dir = self.root / "depth"
        self._frames_dir = self.root / "frames"
        for directory in (self._color_dir, self._depth_dir, self._frames_dir):
            directory.mkdir(parents=True, exist_ok=True)

        self._frame_count = 0
        self._closed = False

    def write_frame(self, frame: CaptureFrame) -> Path:
        """Append one frame. Raises ValueError if its raster size doesn't match the session's
        declared stream configs (the frame invariant, docs/INTERFACES.md #0.5 — no resize/crop)."""
        if self._closed:
            raise RuntimeError("RecordingWriter is closed")

        expected_color_shape = (self._color["height"], self._color["width"])
        if frame.color_rgb.shape[:2] != expected_color_shape:
            raise ValueError(
                f"color frame shape {frame.color_rgb.shape[:2]} != session color config {expected_color_shape}"
            )
        expected_depth_shape = (self._depth["height"], self._depth["width"])
        if frame.depth_u16.shape != expected_depth_shape:
            raise ValueError(
                f"depth frame shape {frame.depth_u16.shape} != session depth config {expected_depth_shape}"
            )

        stem = f"{self._frame_count:06d}"
        color_path = self._color_dir / f"{stem}_color.png"
        depth_path = self._depth_dir / f"{stem}_depth.png"
        sidecar_path = self._frames_dir / f"{stem}.json"

        Image.fromarray(frame.color_rgb, mode="RGB").save(color_path)
        _write_depth_png(frame.depth_u16, depth_path)

        sidecar = {
            "schema_version": SCHEMA_VERSION,
            "frame_index": frame.frame_index,
            "color_timestamp": frame.color_timestamp,
            "color_timestamp_domain": frame.color_timestamp_domain,
            "depth_timestamp": frame.depth_timestamp,
            "depth_timestamp_domain": frame.depth_timestamp_domain,
            "color_frame_number": frame.color_frame_number,
            "depth_frame_number": frame.depth_frame_number,
            "host_monotonic_ns": frame.host_monotonic_ns,
            "extra": dict(frame.extra),
            "files": {
                "color": color_path.relative_to(self.root).as_posix(),
                "depth": depth_path.relative_to(self.root).as_posix(),
            },
        }
        sidecar_path.write_text(json.dumps(sidecar, indent=2) + "\n", encoding="utf-8")

        self._frame_count += 1
        return sidecar_path

    def close(self) -> Path:
        """Write session.json (idempotent) and return its path."""
        path = self.root / "session.json"
        if self._closed:
            return path
        payload = {
            "schema_version": SCHEMA_VERSION,
            "session": self.session,
            "device": self._device,
            "color": self._color,
            "depth": self._depth,
            "color_intrinsics": self._color_intrinsics,
            "depth_scale_m_per_unit": self._depth_scale_m_per_unit,
            "extrinsics_depth_to_color": self._extrinsics,
            "aligned_to": self._aligned_to,
            "frame_count": self._frame_count,
        }
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        self._closed = True
        return path

    def __enter__(self) -> "RecordingWriter":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()


class RecordingReader:
    """Reads a `RecordingWriter` session back, frame by frame, bit-exact."""

    def __init__(self, root: Path) -> None:
        self.root = Path(root)
        session_path = self.root / "session.json"
        if not session_path.is_file():
            raise FileNotFoundError(f"no session.json under {self.root}")
        self.session_info: dict[str, Any] = json.loads(session_path.read_text(encoding="utf-8"))
        self._frames_dir = self.root / "frames"
        self._sidecar_paths = sorted(self._frames_dir.glob("*.json"))

    def __len__(self) -> int:
        return len(self._sidecar_paths)

    def read_frame(self, index: int) -> CaptureFrame:
        sidecar = json.loads(self._sidecar_paths[index].read_text(encoding="utf-8"))
        color_path = self.root / sidecar["files"]["color"]
        depth_path = self.root / sidecar["files"]["depth"]
        color_rgb = np.array(Image.open(color_path).convert("RGB"), dtype=np.uint8)
        depth_u16 = _read_depth_png(depth_path)
        return CaptureFrame(
            frame_index=sidecar["frame_index"],
            color_rgb=color_rgb,
            depth_u16=depth_u16,
            color_timestamp=sidecar["color_timestamp"],
            color_timestamp_domain=sidecar["color_timestamp_domain"],
            depth_timestamp=sidecar["depth_timestamp"],
            depth_timestamp_domain=sidecar["depth_timestamp_domain"],
            color_frame_number=sidecar["color_frame_number"],
            depth_frame_number=sidecar["depth_frame_number"],
            host_monotonic_ns=sidecar["host_monotonic_ns"],
            extra=sidecar.get("extra", {}),
        )

    def __iter__(self) -> Iterator[CaptureFrame]:
        for index in range(len(self)):
            yield self.read_frame(index)


class ReplaySource:
    """A deterministic, no-camera stand-in for a live D405 source: replays a `RecordingWriter`
    session and yields the identical `CaptureFrame` type a live pipeline would, in recorded frame
    order, every time — so downstream code needs no branch between live and recorded input."""

    def __init__(self, root: Path) -> None:
        self._reader = RecordingReader(root)

    @property
    def session_info(self) -> dict[str, Any]:
        return self._reader.session_info

    def __len__(self) -> int:
        return len(self._reader)

    def __iter__(self) -> Iterator[CaptureFrame]:
        return iter(self._reader)

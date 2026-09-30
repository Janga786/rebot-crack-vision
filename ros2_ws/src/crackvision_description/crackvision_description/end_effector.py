"""end_effector — validation of config/robot/end_effector.yaml + the commissioning gate (ADR-014).

Stdlib + PyYAML + numpy only (no rclpy), so it runs under the system python3 without a colcon build
and is importable from launch files. The public names are normative:
`EndEffectorError`, `EndEffectorNotCalibratedError`, `SCHEMA_ID`, `load_config`, `config_sha256`,
`nominal_items`, `assert_commissioning_ready`, `transform`, `rpy_to_matrix`.

The file places three kinds of things on `gripper_link`, each tagged `value_status: nominal|measured`:
the tool tip (crack-facing tool reference), the wrist D405 `camera_link`, and conservative collision
proxies. MOT-05's commissioning-gated executor must call `assert_commissioning_ready` and refuse real
motion while anything is still nominal (an unmeasured tool tip or camera extrinsic means the path is
not where the robot thinks it is).
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any, Dict, List, Union

import numpy as np
import yaml

SCHEMA_ID = "crackvision.end_effector/1"
CAMERA_FRAME = "camera_link"  # realsense2_camera default camera_name "camera" -> camera_link

_TOP_KEYS = frozenset({"schema", "parent_link", "tool", "wrist_camera", "collision_padding_m",
                       "collision", "allowed_self_collisions"})
_TOOL_KEYS = frozenset({"frame", "xyz_m", "rpy_rad", "pointing_axis", "value_status", "provenance", "source"})
_CAMERA_KEYS = frozenset({"model", "frame", "xyz_m", "rpy_rad", "value_status", "provenance", "mount",
                          "source", "assumptions", "derived"})
_COLLISION_KEYS = frozenset({"id", "frame", "box_size_m", "xyz_m", "rpy_rad", "value_status", "source"})
_VALUE_STATUSES = frozenset({"nominal", "measured"})
_MAX_PADDING_M = 0.05


class EndEffectorError(ValueError):
    """Structurally or numerically invalid end-effector config."""


class EndEffectorNotCalibratedError(EndEffectorError):
    """Raised by assert_commissioning_ready while any block is still nominal."""


def _keys(d: Any, allowed: frozenset, ctx: str, required: frozenset | None = None) -> dict:
    if not isinstance(d, dict):
        raise EndEffectorError(f"'{ctx}' must be a mapping")
    unknown = set(d) - allowed
    if unknown:
        raise EndEffectorError(f"unknown key(s) in '{ctx}': {sorted(unknown)}")
    missing = (required if required is not None else allowed) - set(d)
    if missing:
        raise EndEffectorError(f"'{ctx}' is missing key(s): {sorted(missing)}")
    return d


def _name(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not v or any(c.isspace() for c in v):
        raise EndEffectorError(f"'{ctx}' must be a non-empty name without whitespace, got {v!r}")
    return v


def _text(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not v.strip():
        raise EndEffectorError(f"'{ctx}' must be a non-empty string")
    return v


def _vec3(v: Any, ctx: str, *, positive: bool = False) -> List[float]:
    if not (isinstance(v, (list, tuple)) and len(v) == 3
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v)):
        raise EndEffectorError(f"'{ctx}' must be a 3-element list of numbers")
    out = [float(x) for x in v]
    if not all(math.isfinite(x) for x in out):
        raise EndEffectorError(f"'{ctx}' must be finite, got {out}")
    if positive and any(x <= 0.0 for x in out):
        raise EndEffectorError(f"'{ctx}' must be all-positive, got {out}")
    return out


def _status(v: Any, ctx: str) -> str:
    if v not in _VALUE_STATUSES:
        raise EndEffectorError(f"'{ctx}.value_status' must be one of {sorted(_VALUE_STATUSES)}, got {v!r}")
    return v


def rpy_to_matrix(rpy: List[float]) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def transform(block: Dict[str, Any]) -> np.ndarray:
    """4x4 homogeneous transform of a validated block (T_<its parent>_<its frame>)."""
    t = np.eye(4)
    t[:3, :3] = rpy_to_matrix(block["rpy_rad"])
    t[:3, 3] = block["xyz_m"]
    return t


def load_config(path: Union[str, Path]) -> Dict[str, Any]:
    """Load and validate a crackvision.end_effector/1 file. Never invents a value."""
    path = Path(path)
    if not path.is_file():
        raise EndEffectorError(f"end-effector config not found: {path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    _keys(raw, _TOP_KEYS, "<root>")
    if raw["schema"] != SCHEMA_ID:
        raise EndEffectorError(f"'schema' must be {SCHEMA_ID!r}, got {raw['schema']!r}")
    parent = _name(raw["parent_link"], "parent_link")

    t = _keys(raw["tool"], _TOOL_KEYS, "tool")
    axis = np.array(_vec3(t["pointing_axis"], "tool.pointing_axis"))
    if np.linalg.norm(axis) < 1e-9:
        raise EndEffectorError("'tool.pointing_axis' must be non-zero")
    tool = {"frame": _name(t["frame"], "tool.frame"), "xyz_m": _vec3(t["xyz_m"], "tool.xyz_m"),
            "rpy_rad": _vec3(t["rpy_rad"], "tool.rpy_rad"),
            "pointing_axis": (axis / np.linalg.norm(axis)).tolist(),
            "value_status": _status(t["value_status"], "tool"),
            "provenance": _text(t["provenance"], "tool.provenance"), "source": _text(t["source"], "tool.source")}

    c = _keys(raw["wrist_camera"], _CAMERA_KEYS, "wrist_camera",
              required=_CAMERA_KEYS - {"assumptions", "derived"})
    frame = _name(c["frame"], "wrist_camera.frame")
    if frame != CAMERA_FRAME:
        raise EndEffectorError(f"'wrist_camera.frame' must be {CAMERA_FRAME!r} (realsense2_camera TF names), got {frame!r}")
    assumptions = c.get("assumptions", [])
    if not isinstance(assumptions, list) or not all(isinstance(a, str) and a.strip() for a in assumptions):
        raise EndEffectorError("'wrist_camera.assumptions' must be a list of non-empty strings")
    camera = {"model": _text(c["model"], "wrist_camera.model"), "frame": frame,
              "xyz_m": _vec3(c["xyz_m"], "wrist_camera.xyz_m"), "rpy_rad": _vec3(c["rpy_rad"], "wrist_camera.rpy_rad"),
              "value_status": _status(c["value_status"], "wrist_camera"),
              "provenance": _text(c["provenance"], "wrist_camera.provenance"),
              "mount": _text(c["mount"], "wrist_camera.mount"), "source": _text(c["source"], "wrist_camera.source"),
              "assumptions": list(assumptions), "derived": c.get("derived") or {}}
    if tool["frame"] in (parent, frame):
        raise EndEffectorError("'tool.frame' must differ from parent_link and the camera frame")

    pad = raw["collision_padding_m"]
    if isinstance(pad, bool) or not isinstance(pad, (int, float)) or not 0.0 <= float(pad) <= _MAX_PADDING_M:
        raise EndEffectorError(f"'collision_padding_m' must be a number in [0, {_MAX_PADDING_M}], got {pad!r}")

    if not isinstance(raw["collision"], list) or not raw["collision"]:
        raise EndEffectorError("'collision' must be a non-empty list")
    known_frames = {parent, frame, tool["frame"]}
    collision, ids = [], set()
    for i, item in enumerate(raw["collision"]):
        ctx = f"collision[{i}]"
        _keys(item, _COLLISION_KEYS, ctx)
        cid = _name(item["id"], f"{ctx}.id")
        if cid in ids or cid in known_frames:
            raise EndEffectorError(f"'{ctx}.id' {cid!r} duplicates another frame/link name")
        f = _name(item["frame"], f"{ctx}.frame")
        if f not in known_frames:
            raise EndEffectorError(f"'{ctx}.frame' must be one of {sorted(known_frames)}, got {f!r}")
        ids.add(cid)
        collision.append({"id": cid, "frame": f, "box_size_m": _vec3(item["box_size_m"], f"{ctx}.box_size_m", positive=True),
                          "xyz_m": _vec3(item["xyz_m"], f"{ctx}.xyz_m"), "rpy_rad": _vec3(item["rpy_rad"], f"{ctx}.rpy_rad"),
                          "value_status": _status(item["value_status"], ctx), "source": _text(item["source"], f"{ctx}.source")})

    pairs = raw["allowed_self_collisions"]
    if not isinstance(pairs, list):
        raise EndEffectorError("'allowed_self_collisions' must be a list")
    allowed, seen = [], set()
    for i, p in enumerate(pairs):
        ctx = f"allowed_self_collisions[{i}]"
        if not (isinstance(p, list) and len(p) == 3):
            raise EndEffectorError(f"'{ctx}' must be [link_a, link_b, reason]")
        a, b, why = _name(p[0], f"{ctx}[0]"), _name(p[1], f"{ctx}[1]"), _text(p[2], f"{ctx}[2]")
        if a == b:
            raise EndEffectorError(f"'{ctx}': the two links must differ")
        if a not in ids and b not in ids:
            raise EndEffectorError(f"'{ctx}': at least one side must be an overlay collision link {sorted(ids)}; "
                                   "vendor link pairs belong to the vendor SRDF")
        key = frozenset((a, b))
        if key in seen:
            raise EndEffectorError(f"'{ctx}': duplicate pair {sorted(key)}")
        seen.add(key)
        allowed.append([a, b, why])

    return {"schema": SCHEMA_ID, "parent_link": parent, "tool": tool, "wrist_camera": camera,
            "collision_padding_m": float(pad), "collision": collision, "allowed_self_collisions": allowed}


def config_sha256(path: Union[str, Path]) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def nominal_items(cfg: Dict[str, Any]) -> List[str]:
    """Sorted ids of every block still `nominal` (tool, wrist_camera, collision:<id>)."""
    items = [k for k in ("tool", "wrist_camera") if cfg[k]["value_status"] == "nominal"]
    items += [f"collision:{c['id']}" for c in cfg["collision"] if c["value_status"] == "nominal"]
    return sorted(items)


def assert_commissioning_ready(cfg: Dict[str, Any]) -> None:
    """Refuse (raise) unless every block is `measured`. MOT-05 calls this before any real-hardware run."""
    pending = nominal_items(cfg)
    if pending:
        raise EndEffectorNotCalibratedError(
            "end-effector geometry has nominal (unmeasured) value(s), refusing commissioning: " + ", ".join(pending))

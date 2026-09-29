"""scene_core — pure-Python collision-scene config validation + the commissioning gate (MOT-03).

Stdlib + PyYAML only -- no rclpy or *_msgs import anywhere in this file (mirrors
reachability_core.py), so it runs standalone under the system python3 (ROS Humble, py3.10)
without a colcon build and is unit-testable without a mock ROS stack. The public names below
(`SceneError`, `SceneNotCommissionedError`, `SCHEMA_ID`, `load_config`, `config_sha256`,
`nominal_items`, `all_measured`, `assert_commissioning_ready`) are normative: MOT-05's
commissioning-gated executor imports `assert_commissioning_ready` and must call it before any
real-hardware run, refusing whenever any scene value is still `nominal`.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any, Dict, List, Union

import yaml

SCHEMA_ID = "crackvision.scene_config/1"

_TOP_LEVEL_KEYS = frozenset({"schema", "frame", "objects", "allowed_collisions"})
_OBJECT_KEYS = frozenset({"id", "shape", "dimensions_m", "pose", "value_status", "source"})
_POSE_KEYS = frozenset({"frame", "position_m", "rpy_rad"})
_ACM_KEYS = frozenset({"link_a", "link_b", "reason", "value_status", "source"})

_VALUE_STATUSES = frozenset({"measured", "nominal"})
_SHAPES = frozenset({"box"})


class SceneError(ValueError):
    """Raised for any structurally or numerically invalid scene config."""


class SceneNotCommissionedError(SceneError):
    """Raised by assert_commissioning_ready when any scene value is still 'nominal'."""


def _check_keys(d: object, allowed: frozenset, ctx: str) -> None:
    if not isinstance(d, dict):
        raise SceneError(f"'{ctx}' must be a mapping")
    unknown = set(d.keys()) - allowed
    if unknown:
        raise SceneError(f"unknown key(s) in '{ctx}': {sorted(unknown)}")


def _require(d: dict, key: str, ctx: str) -> Any:
    if key not in d:
        raise SceneError(f"'{ctx}' is missing required key '{key}'")
    return d[key]


def _require_nonempty_str(d: dict, key: str, ctx: str) -> str:
    value = _require(d, key, ctx)
    if not isinstance(value, str) or not value:
        raise SceneError(f"'{ctx}.{key}' must be a non-empty string")
    return value


def _validate_value_status(value: Any, ctx: str) -> str:
    if value not in _VALUE_STATUSES:
        raise SceneError(f"'{ctx}.value_status' must be one of {sorted(_VALUE_STATUSES)}, got {value!r}")
    return value


def _validate_vec3(value: Any, ctx: str, name: str, *, positive: bool = False) -> List[float]:
    if not (isinstance(value, (list, tuple)) and len(value) == 3 and all(isinstance(v, (int, float)) for v in value)):
        raise SceneError(f"'{ctx}.{name}' must be a 3-element list of numbers")
    floats = [float(v) for v in value]
    if positive and any(v <= 0.0 for v in floats):
        raise SceneError(f"'{ctx}.{name}' must have all-positive components, got {floats}")
    return floats


def _validate_pose(pose: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(pose, _POSE_KEYS, ctx)
    frame = _require_nonempty_str(pose, "frame", ctx)
    position = _validate_vec3(_require(pose, "position_m", ctx), ctx, "position_m")
    rpy = _validate_vec3(_require(pose, "rpy_rad", ctx), ctx, "rpy_rad")
    return {"frame": frame, "position_m": position, "rpy_rad": rpy}


def _validate_object(obj: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(obj, _OBJECT_KEYS, ctx)
    obj_id = _require_nonempty_str(obj, "id", ctx)
    shape = _require(obj, "shape", ctx)
    if shape not in _SHAPES:
        raise SceneError(f"'{ctx}.shape' must be one of {sorted(_SHAPES)}, got {shape!r}")
    dimensions = _validate_vec3(_require(obj, "dimensions_m", ctx), ctx, "dimensions_m", positive=True)
    pose = _validate_pose(_require(obj, "pose", ctx), f"{ctx}.pose")
    value_status = _validate_value_status(_require(obj, "value_status", ctx), ctx)
    source = _require_nonempty_str(obj, "source", ctx)
    return {
        "id": obj_id,
        "shape": shape,
        "dimensions_m": dimensions,
        "pose": pose,
        "value_status": value_status,
        "source": source,
    }


def _validate_acm_entry(entry: Any, ctx: str) -> Dict[str, Any]:
    _check_keys(entry, _ACM_KEYS, ctx)
    link_a = _require_nonempty_str(entry, "link_a", ctx)
    link_b = _require_nonempty_str(entry, "link_b", ctx)
    if link_a == link_b:
        raise SceneError(f"'{ctx}': link_a and link_b must differ, both are {link_a!r}")
    reason = _require_nonempty_str(entry, "reason", ctx)
    value_status = _validate_value_status(_require(entry, "value_status", ctx), ctx)
    source = _require_nonempty_str(entry, "source", ctx)
    return {
        "link_a": link_a,
        "link_b": link_b,
        "reason": reason,
        "value_status": value_status,
        "source": source,
    }


def load_config(path: Union[str, Path]) -> Dict[str, Any]:
    """Load and validate a `crackvision.scene_config/1` YAML file. Raises SceneError on any
    structural or numeric problem. Never invents a value: every object and allowed_collisions
    entry must carry its own `value_status` + `source`."""
    path = Path(path)
    with path.open("r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    if not isinstance(raw, dict):
        raise SceneError("scene config must be a mapping at the top level")
    _check_keys(raw, _TOP_LEVEL_KEYS, "<root>")

    schema = _require(raw, "schema", "<root>")
    if schema != SCHEMA_ID:
        raise SceneError(f"'schema' must be {SCHEMA_ID!r}, got {schema!r}")

    frame = _require_nonempty_str(raw, "frame", "<root>")

    raw_objects = _require(raw, "objects", "<root>")
    if not isinstance(raw_objects, list) or not raw_objects:
        raise SceneError("'objects' must be a non-empty list")

    objects: List[Dict[str, Any]] = []
    seen_ids = set()
    for i, obj in enumerate(raw_objects):
        validated = _validate_object(obj, f"objects[{i}]")
        if validated["id"] in seen_ids:
            raise SceneError(f"duplicate object id {validated['id']!r}")
        seen_ids.add(validated["id"])
        objects.append(validated)

    raw_acm = raw.get("allowed_collisions", [])
    if not isinstance(raw_acm, list):
        raise SceneError("'allowed_collisions' must be a list")
    allowed_collisions = [_validate_acm_entry(entry, f"allowed_collisions[{i}]") for i, entry in enumerate(raw_acm)]

    return {
        "schema": schema,
        "frame": frame,
        "objects": objects,
        "allowed_collisions": allowed_collisions,
    }


def config_sha256(path: Union[str, Path]) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def nominal_items(config: Dict[str, Any]) -> List[str]:
    """Sorted human-readable ids of every object/allowed_collisions entry still 'nominal'."""
    items = [f"object:{obj['id']}" for obj in config["objects"] if obj["value_status"] == "nominal"]
    items += [
        f"allowed_collision:{entry['link_a']}~{entry['link_b']}"
        for entry in config["allowed_collisions"]
        if entry["value_status"] == "nominal"
    ]
    return sorted(items)


def all_measured(config: Dict[str, Any]) -> bool:
    return not nominal_items(config)


def assert_commissioning_ready(config: Dict[str, Any]) -> None:
    """Raise SceneNotCommissionedError unless every object and allowed_collisions entry in
    `config` is `value_status: measured`. MOT-05's commissioning-gated executor calls this before
    any real-hardware run: a scene built from unmeasured (nominal) values must never be trusted
    to gate a physical motion."""
    pending = nominal_items(config)
    if pending:
        raise SceneNotCommissionedError(
            "scene has nominal (unmeasured) value(s), refusing commissioning: " + ", ".join(pending)
        )

"""Reachability-map JSON and specimen-placement YAML: build, write, load, validate (MOT-04.2).

Stdlib + numpy + PyYAML only -- no rclpy or *_msgs import anywhere in this file, so it runs
standalone under the system python3 (ROS Humble, py3.10) without a colcon build. Schemas are
normative in `docs/INTERFACES.md` §7; this module is the reference implementation.
"""

from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Union

import yaml

SCHEMA = "crackvision.reachability_map/1"
PLACEMENT_SCHEMA = "crackvision.specimen_placement/1"

STATUSES = frozenset({"reachable", "unreachable", "prefiltered", "fk_mismatch", "error"})

_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))

_REACHABLE_EXTRA_KEYS = (
    "tilt_deg",
    "azimuth_rad",
    "roll_rad",
    "quat_xyzw",
    "joints",
    "min_joint_limit_margin_rad",
    "fk_position_error_m",
    "fk_axis_error_deg",
)

_PLACEMENT_KEYS = frozenset({
    "center_xy_m", "surface_z_m", "yaw_rad", "footprint_m", "tolerance_m", "standoffs_m",
    "score_min_joint_margin_rad",
})


class MapError(ValueError):
    """Raised for any structurally or numerically invalid reachability map."""


class PlacementError(ValueError):
    """Raised for any structurally or numerically invalid specimen-placement file."""


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _jsonify(value: Any) -> Any:
    """Recursively convert tuples (from reachability_core's validated config) to lists."""
    if isinstance(value, dict):
        return {k: _jsonify(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonify(v) for v in value]
    return value


def _format_surface_z_key(z: float) -> str:
    return format(float(z), ".6g")


# --------------------------------------------------------------------------------------
# building a map
# --------------------------------------------------------------------------------------

def new_map(
    cfg: Mapping[str, Any],
    config_path: Union[str, Path],
    config_sha256: str,
    robot_info: Mapping[str, Any],
    git_commit: str,
) -> Dict[str, Any]:
    """Start a fresh, empty reachability map from a validated reachability_core config dict."""
    return {
        "schema": SCHEMA,
        "created_utc": _utc_iso(),
        "git_commit": git_commit,
        "config_path": str(config_path),
        "config_sha256": config_sha256,
        "frame": cfg["frame"],
        "units": {"length": "m", "angle": "rad"},
        "quaternion_order": "xyzw",
        "robot": dict(robot_info),
        "boresight": {
            "axis_local": _jsonify(cfg["boresight"]["axis_local"]),
            "provenance": cfg["boresight"]["provenance"],
        },
        "grid": {
            "grid": _jsonify(cfg["grid"]),
            "orientation": _jsonify(cfg["orientation"]),
            "ik": _jsonify(cfg["ik"]),
            "surface_collision": _jsonify(cfg["surface_collision"]),
        },
        "complete": False,
        "duration_s": None,
        "targets": [],
        "summary": None,
    }


def add_target(
    map_dict: Dict[str, Any],
    target: Any,
    status: str,
    position: Sequence[float],
    *,
    ik_calls: int = 0,
    ik_time_s: float = 0.0,
    error: str = "",
    tilt_deg: Optional[float] = None,
    azimuth_rad: Optional[float] = None,
    roll_rad: Optional[float] = None,
    quat_xyzw: Optional[Sequence[float]] = None,
    joints: Optional[Mapping[str, float]] = None,
    min_joint_limit_margin_rad: Optional[float] = None,
    fk_position_error_m: Optional[float] = None,
    fk_axis_error_deg: Optional[float] = None,
) -> Dict[str, Any]:
    """Append one target entry to `map_dict["targets"]`. `target` duck-types reachability_core.Target."""
    if status not in STATUSES:
        raise MapError(f"unknown status {status!r}; must be one of {sorted(STATUSES)}")

    entry: Dict[str, Any] = {
        "target_id": target.target_id,
        "x": target.x,
        "y": target.y,
        "surface_z": target.surface_z,
        "standoff_m": target.standoff_m,
        "position": [float(v) for v in position],
        "status": status,
        "ik_calls": ik_calls,
        "ik_time_s": ik_time_s,
        "error": error,
    }
    if status == "reachable":
        if quat_xyzw is None or joints is None:
            raise MapError(f"target {target.target_id!r}: status 'reachable' requires quat_xyzw and joints")
        entry.update({
            "tilt_deg": tilt_deg,
            "azimuth_rad": azimuth_rad,
            "roll_rad": roll_rad,
            "quat_xyzw": [float(v) for v in quat_xyzw],
            "joints": {k: float(v) for k, v in joints.items()},
            "min_joint_limit_margin_rad": min_joint_limit_margin_rad,
            "fk_position_error_m": fk_position_error_m,
            "fk_axis_error_deg": fk_axis_error_deg,
        })
    map_dict["targets"].append(entry)
    return entry


def finalize(map_dict: Dict[str, Any], complete: bool, duration_s: float) -> Dict[str, Any]:
    """Set `complete`/`duration_s` and (re)compute `summary` from the current `targets` list."""
    map_dict["complete"] = complete
    map_dict["duration_s"] = duration_s

    counts_by_status: Dict[str, int] = {}
    per_z: Dict[str, List[int]] = {}
    for t in map_dict["targets"]:
        counts_by_status[t["status"]] = counts_by_status.get(t["status"], 0) + 1
        z_key = _format_surface_z_key(t["surface_z"])
        bucket = per_z.setdefault(z_key, [0, 0])
        bucket[1] += 1
        if t["status"] == "reachable":
            bucket[0] += 1

    map_dict["summary"] = {
        "counts_by_status": counts_by_status,
        "reachable_fraction_by_surface_z": {
            z: (reachable / total if total else 0.0) for z, (reachable, total) in per_z.items()
        },
    }
    return map_dict


# --------------------------------------------------------------------------------------
# I/O
# --------------------------------------------------------------------------------------

def write_map(map_dict: Mapping[str, Any], path: Union[str, Path]) -> None:
    """Write `map_dict` atomically (tmp file + os.replace), compact JSON, UTF-8."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(map_dict, indent=None, separators=(",", ":"), ensure_ascii=False)

    fd, tmp_name = tempfile.mkstemp(dir=str(path.parent), prefix=f".{path.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(payload)
        os.replace(tmp_name, path)
    except BaseException:
        if os.path.exists(tmp_name):
            os.remove(tmp_name)
        raise


def load_map(
    path: Union[str, Path],
    limits_path: Union[str, Path] = "config/robot/b601_dm_limits.yaml",
) -> Dict[str, Any]:
    """Load, then `validate_map` a reachability map JSON file. Raises MapError if invalid."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    limits = limits_from_yaml(limits_path)
    validate_map(data, limits)
    return data


def limits_from_yaml(path: Union[str, Path]) -> Dict[str, tuple]:
    """Parse config/robot/b601_dm_limits.yaml into {joint_name: (lower, upper)}."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    joints = raw["joints"]
    return {name: (float(spec["lower"]), float(spec["upper"])) for name, spec in joints.items()}


def joint_limit_margin(joints: Mapping[str, float], limits: Mapping[str, tuple]) -> float:
    """min over joint1..6 of min(q - lower, upper - q). Shared with the MOT-04.4 sweep node."""
    margins = []
    for name in _JOINT_NAMES:
        q = joints[name]
        lower, upper = limits[name]
        margins.append(min(q - lower, upper - q))
    return min(margins)


# --------------------------------------------------------------------------------------
# validate_map
# --------------------------------------------------------------------------------------

def validate_map(map_dict: Mapping[str, Any], limits: Mapping[str, tuple]) -> None:
    """Raise MapError if `map_dict` violates the crackvision.reachability_map/1 schema."""
    if map_dict.get("schema") != SCHEMA:
        raise MapError(f"unexpected schema {map_dict.get('schema')!r}, expected {SCHEMA!r}")

    targets = map_dict.get("targets")
    if not isinstance(targets, list):
        raise MapError("'targets' must be a list")

    seen_ids = set()
    for entry in targets:
        target_id = entry.get("target_id")
        if target_id in seen_ids:
            raise MapError(f"duplicate target_id {target_id!r}")
        seen_ids.add(target_id)

        status = entry.get("status")
        if status not in STATUSES:
            raise MapError(f"target {target_id!r}: unknown status {status!r}")

        if status == "reachable":
            quat = entry.get("quat_xyzw")
            if not isinstance(quat, list) or len(quat) != 4:
                raise MapError(f"target {target_id!r}: status 'reachable' requires a 4-element quat_xyzw")
            joints = entry.get("joints")
            if not isinstance(joints, dict) or any(name not in joints for name in _JOINT_NAMES):
                raise MapError(f"target {target_id!r}: status 'reachable' requires all 6 joint values")
            for name in _JOINT_NAMES:
                q = joints[name]
                lower, upper = limits[name]
                if q < lower or q > upper:
                    raise MapError(
                        f"target {target_id!r}: {name}={q} outside limits [{lower}, {upper}]"
                    )


# --------------------------------------------------------------------------------------
# specimen placement
# --------------------------------------------------------------------------------------

def _validate_placement_candidate(candidate: Mapping[str, Any], ctx: str) -> None:
    if not isinstance(candidate, dict):
        raise PlacementError(f"'{ctx}' must be a mapping")
    missing = _PLACEMENT_KEYS - set(candidate.keys())
    if missing:
        raise PlacementError(f"'{ctx}' is missing key(s): {sorted(missing)}")

    center = candidate["center_xy_m"]
    if not isinstance(center, (list, tuple)) or len(center) != 2:
        raise PlacementError(f"'{ctx}.center_xy_m' must be a 2-element list")
    footprint = candidate["footprint_m"]
    if not isinstance(footprint, (list, tuple)) or len(footprint) != 2:
        raise PlacementError(f"'{ctx}.footprint_m' must be a 2-element list")
    standoffs = candidate["standoffs_m"]
    if not isinstance(standoffs, (list, tuple)) or len(standoffs) == 0:
        raise PlacementError(f"'{ctx}.standoffs_m' must be a non-empty list")
    for key in ("surface_z_m", "yaw_rad", "tolerance_m", "score_min_joint_margin_rad"):
        if not isinstance(candidate[key], (int, float)) or isinstance(candidate[key], bool):
            raise PlacementError(f"'{ctx}.{key}' must be numeric")
    if candidate["tolerance_m"] < 0:
        raise PlacementError(f"'{ctx}.tolerance_m' must be >= 0")


def validate_placement(placement_dict: Mapping[str, Any]) -> None:
    """Raise PlacementError if `placement_dict` violates the crackvision.specimen_placement/1 schema."""
    if placement_dict.get("schema") != PLACEMENT_SCHEMA:
        raise PlacementError(f"unexpected schema {placement_dict.get('schema')!r}, expected {PLACEMENT_SCHEMA!r}")
    if placement_dict.get("value_status") != "nominal":
        raise PlacementError(f"'value_status' must be 'nominal', got {placement_dict.get('value_status')!r}")
    if placement_dict.get("frame") != "base_link":
        raise PlacementError(f"'frame' must be 'base_link', got {placement_dict.get('frame')!r}")

    feasible = placement_dict.get("feasible")
    if not isinstance(feasible, bool):
        raise PlacementError("'feasible' must be a bool")

    placement = placement_dict.get("placement")
    if feasible:
        if placement is None:
            raise PlacementError("'feasible' is true but 'placement' is null")
        _validate_placement_candidate(placement, "placement")
    else:
        if placement is not None:
            raise PlacementError("'feasible' is false but 'placement' is not null")
        max_square = placement_dict.get("max_feasible_square_m")
        if not isinstance(max_square, (int, float)) or isinstance(max_square, bool) or max_square < 0:
            raise PlacementError("'max_feasible_square_m' must be a non-negative number when infeasible")

    alternatives = placement_dict.get("alternatives")
    if not isinstance(alternatives, list):
        raise PlacementError("'alternatives' must be a list")
    for i, alt in enumerate(alternatives):
        _validate_placement_candidate(alt, f"alternatives[{i}]")

    for key in ("map_sha256", "config_sha256"):
        value = placement_dict.get(key)
        if not isinstance(value, str) or len(value) != 64:
            raise PlacementError(f"'{key}' must be a 64-character hex string")
        int(value, 16)

    if not isinstance(placement_dict.get("boresight_provenance"), str) or not placement_dict["boresight_provenance"]:
        raise PlacementError("'boresight_provenance' must be a non-empty string")

    caveats = placement_dict.get("caveats")
    if not isinstance(caveats, list) or not all(isinstance(c, str) for c in caveats):
        raise PlacementError("'caveats' must be a list of strings")

    if not isinstance(placement_dict.get("source"), str) or not placement_dict["source"]:
        raise PlacementError("'source' must be a non-empty string")


def load_placement(path: Union[str, Path]) -> Dict[str, Any]:
    """Load, then `validate_placement` a specimen_placement.yaml file. Raises PlacementError."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise PlacementError("placement file root must be a mapping")
    validate_placement(data)
    return data

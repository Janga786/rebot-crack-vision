"""Pure-Python reachability config validation and boresight-down pose geometry (MOT-04.1).

Stdlib + numpy + PyYAML only -- no rclpy or *_msgs import anywhere in this file, so it runs
standalone under the system python3 (ROS Humble, py3.10) without a colcon build. Later cards
(MOT-04.2+) import these names from the actual ROS sweep node; the public names below
(`ConfigError`, `Orientation`, `Target`, `load_config`, `config_sha256`, `axis_values`,
`orientations`, `target_position`, `enumerate_targets`, `reach_bound_from_urdf`,
`prefiltered`, `specimen_proxy_boxes`) are normative.

ADR-014 additions (all optional, backward compatible): `ik_link` may be any frame rigidly attached to
the planning chain tip (e.g. `tool_tip`, `camera_link` from crackvision_description) -- MoveIt's
/compute_ik resolves it; `surface_collision.model: specimen_block` replaces the thin grid-wide slab by
a block from the table (`floor_z_m`) up to the surface that excludes the robot base footprint
(`base_keepout_m`); `environment` names static workcell objects (e.g. the table) taken from a
crackvision.scene_config/1 file (MOT-03) and applied for the whole sweep.
"""

from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, NamedTuple, Sequence, Tuple, Union

import numpy as np
import yaml

_TOP_LEVEL_KEYS = frozenset({
    "frame", "group", "ik_link", "boresight", "grid", "orientation",
    "ik", "surface_collision", "prefilter", "placement", "value_status", "sources", "environment",
})
_BORESIGHT_KEYS = frozenset({"axis_local", "down_world", "provenance", "source"})
_AXIS_KEYS = frozenset({"min", "max", "step"})
_GRID_KEYS = frozenset({"x_m", "y_m", "surface_z_m", "standoffs_m"})
_ORIENTATION_KEYS = frozenset({"roll_samples", "tilt_deg", "tilt_azimuth_samples"})
_IK_KEYS = frozenset({"timeout_s", "avoid_collisions", "seed", "roll_search"})
_SURFACE_COLLISION_KEYS = frozenset({
    "enabled", "thickness_m", "margin_m", "allowed_links", "model", "floor_z_m", "base_keepout_m",
})
_SURFACE_MODELS = frozenset({"slab", "specimen_block"})
_ENVIRONMENT_KEYS = frozenset({"scene_config", "objects"})
_SURFACE_GAP_M = 0.001  # proxy top face sits this far below the inspected surface
_PREFILTER_KEYS = frozenset({"enabled"})
_PLACEMENT_KEYS = frozenset({"footprint_m", "yaw_candidates_rad", "tolerance_m", "top_k"})

_IK_SEEDS = frozenset({"neighbour", "home"})
_IK_ROLL_SEARCH = frozenset({"first", "best"})

_UNSUPPORTED_JOINT_TYPES = frozenset({"prismatic", "floating", "planar"})


class ConfigError(ValueError):
    """Raised for any structurally or numerically invalid reachability config."""


class Orientation(NamedTuple):
    tilt_deg: float
    azimuth_rad: float
    roll_rad: float
    d_world: np.ndarray
    quat_xyzw: np.ndarray


class Target(NamedTuple):
    target_id: str
    iz: int
    ix: int
    iy: int
    x: float
    y: float
    surface_z: float
    standoff_m: float


# --------------------------------------------------------------------------------------
# config validation
# --------------------------------------------------------------------------------------

def _check_keys(d: object, allowed: frozenset, ctx: str) -> None:
    if not isinstance(d, dict):
        raise ConfigError(f"'{ctx}' must be a mapping")
    unknown = set(d.keys()) - allowed
    if unknown:
        raise ConfigError(f"unknown key(s) in '{ctx}': {sorted(unknown)}")


def _require(d: dict, key: str, ctx: str):
    if key not in d:
        raise ConfigError(f"'{ctx}' is missing required key '{key}'")
    return d[key]


def _as_float(v: object, ctx: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ConfigError(f"'{ctx}' must be numeric, got {v!r}")
    return float(v)


def _as_int(v: object, ctx: str) -> int:
    if isinstance(v, bool) or not isinstance(v, int):
        raise ConfigError(f"'{ctx}' must be an int, got {v!r}")
    return v


def _as_bool(v: object, ctx: str) -> bool:
    if not isinstance(v, bool):
        raise ConfigError(f"'{ctx}' must be a bool, got {v!r}")
    return v


def _as_vec3(v: object, ctx: str) -> Tuple[float, float, float]:
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ConfigError(f"'{ctx}' must be a 3-element list")
    return (_as_float(v[0], ctx), _as_float(v[1], ctx), _as_float(v[2], ctx))


def _normalised(v: Tuple[float, float, float], ctx: str) -> Tuple[float, float, float]:
    arr = np.array(v, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-12:
        raise ConfigError(f"'{ctx}' must be non-zero")
    arr = arr / norm
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _validate_axis(cfg: object, ctx: str) -> Dict[str, float]:
    _check_keys(cfg, _AXIS_KEYS, ctx)
    axis_min = _as_float(_require(cfg, "min", ctx), f"{ctx}.min")
    axis_max = _as_float(_require(cfg, "max", ctx), f"{ctx}.max")
    step = _as_float(_require(cfg, "step", ctx), f"{ctx}.step")
    if step <= 0:
        raise ConfigError(f"'{ctx}.step' must be > 0, got {step}")
    if axis_max < axis_min:
        raise ConfigError(f"'{ctx}.max' ({axis_max}) must be >= '{ctx}.min' ({axis_min})")
    span = (axis_max - axis_min) / step
    if abs(span - round(span)) > 1e-6:
        raise ConfigError(f"'{ctx}': (max-min)/step = {span} is not integral within 1e-6")
    return {"min": axis_min, "max": axis_max, "step": step}


def axis_values(axis_cfg: Dict[str, float]) -> List[float]:
    """Inclusive sample list; count = round((max-min)/step)+1, no float accumulation."""
    axis_min = axis_cfg["min"]
    axis_max = axis_cfg["max"]
    step = axis_cfg["step"]
    count = round((axis_max - axis_min) / step) + 1
    return [axis_min + i * step for i in range(count)]


def load_config(path: Union[str, Path]) -> dict:
    raw = yaml.safe_load(Path(path).read_text())
    if not isinstance(raw, dict):
        raise ConfigError("config root must be a mapping")
    _check_keys(raw, _TOP_LEVEL_KEYS, "<root>")

    for key in ("frame", "group", "ik_link"):
        value = _require(raw, key, "<root>")
        if not isinstance(value, str) or not value:
            raise ConfigError(f"'{key}' must be a non-empty string")

    boresight_raw = _require(raw, "boresight", "<root>")
    _check_keys(boresight_raw, _BORESIGHT_KEYS, "boresight")
    axis_local = _normalised(
        _as_vec3(_require(boresight_raw, "axis_local", "boresight"), "boresight.axis_local"),
        "boresight.axis_local",
    )
    down_world = _normalised(
        _as_vec3(_require(boresight_raw, "down_world", "boresight"), "boresight.down_world"),
        "boresight.down_world",
    )
    provenance = _require(boresight_raw, "provenance", "boresight")
    source = _require(boresight_raw, "source", "boresight")
    if not isinstance(provenance, str) or not provenance:
        raise ConfigError("'boresight.provenance' must be a non-empty string")
    if not isinstance(source, str) or not source:
        raise ConfigError("'boresight.source' must be a non-empty string")
    boresight = {
        "axis_local": axis_local,
        "down_world": down_world,
        "provenance": provenance,
        "source": source,
    }

    grid_raw = _require(raw, "grid", "<root>")
    _check_keys(grid_raw, _GRID_KEYS, "grid")
    x_axis = _validate_axis(_require(grid_raw, "x_m", "grid"), "grid.x_m")
    y_axis = _validate_axis(_require(grid_raw, "y_m", "grid"), "grid.y_m")
    surface_z_raw = _require(grid_raw, "surface_z_m", "grid")
    if not isinstance(surface_z_raw, list) or len(surface_z_raw) == 0:
        raise ConfigError("'grid.surface_z_m' must be a non-empty list")
    surface_z_m = tuple(_as_float(v, "grid.surface_z_m") for v in surface_z_raw)
    standoffs_raw = _require(grid_raw, "standoffs_m", "grid")
    if not isinstance(standoffs_raw, list) or len(standoffs_raw) == 0:
        raise ConfigError("'grid.standoffs_m' must be a non-empty list")
    standoffs_m = tuple(_as_float(v, "grid.standoffs_m") for v in standoffs_raw)
    for s in standoffs_m:
        if s < 0:
            raise ConfigError(f"'grid.standoffs_m' entries must be >= 0, got {s}")
    grid = {
        "x_m": x_axis,
        "y_m": y_axis,
        "surface_z_m": surface_z_m,
        "standoffs_m": standoffs_m,
    }

    orientation_raw = _require(raw, "orientation", "<root>")
    _check_keys(orientation_raw, _ORIENTATION_KEYS, "orientation")
    roll_samples = _as_int(_require(orientation_raw, "roll_samples", "orientation"), "orientation.roll_samples")
    if roll_samples < 1:
        raise ConfigError(f"'orientation.roll_samples' must be >= 1, got {roll_samples}")
    tilt_deg_raw = _require(orientation_raw, "tilt_deg", "orientation")
    if not isinstance(tilt_deg_raw, list) or len(tilt_deg_raw) == 0:
        raise ConfigError("'orientation.tilt_deg' must be a non-empty list")
    tilt_deg = tuple(_as_float(v, "orientation.tilt_deg") for v in tilt_deg_raw)
    for t in tilt_deg:
        if not (0.0 <= t <= 60.0):
            raise ConfigError(f"'orientation.tilt_deg' entries must be in [0, 60], got {t}")
    tilt_azimuth_samples = _as_int(
        _require(orientation_raw, "tilt_azimuth_samples", "orientation"), "orientation.tilt_azimuth_samples"
    )
    if tilt_azimuth_samples < 1:
        raise ConfigError(
            f"'orientation.tilt_azimuth_samples' must be >= 1, got {tilt_azimuth_samples}"
        )
    orientation = {
        "roll_samples": roll_samples,
        "tilt_deg": tilt_deg,
        "tilt_azimuth_samples": tilt_azimuth_samples,
    }

    ik_raw = _require(raw, "ik", "<root>")
    _check_keys(ik_raw, _IK_KEYS, "ik")
    timeout_s = _as_float(_require(ik_raw, "timeout_s", "ik"), "ik.timeout_s")
    if timeout_s <= 0:
        raise ConfigError(f"'ik.timeout_s' must be > 0, got {timeout_s}")
    avoid_collisions = _as_bool(_require(ik_raw, "avoid_collisions", "ik"), "ik.avoid_collisions")
    seed = _require(ik_raw, "seed", "ik")
    if seed not in _IK_SEEDS:
        raise ConfigError(f"'ik.seed' must be one of {sorted(_IK_SEEDS)}, got {seed!r}")
    roll_search = _require(ik_raw, "roll_search", "ik")
    if roll_search not in _IK_ROLL_SEARCH:
        raise ConfigError(f"'ik.roll_search' must be one of {sorted(_IK_ROLL_SEARCH)}, got {roll_search!r}")
    ik = {
        "timeout_s": timeout_s,
        "avoid_collisions": avoid_collisions,
        "seed": seed,
        "roll_search": roll_search,
    }

    sc_raw = _require(raw, "surface_collision", "<root>")
    _check_keys(sc_raw, _SURFACE_COLLISION_KEYS, "surface_collision")
    sc_enabled = _as_bool(_require(sc_raw, "enabled", "surface_collision"), "surface_collision.enabled")
    thickness_m = _as_float(_require(sc_raw, "thickness_m", "surface_collision"), "surface_collision.thickness_m")
    if thickness_m <= 0:
        raise ConfigError(f"'surface_collision.thickness_m' must be > 0, got {thickness_m}")
    margin_m = _as_float(_require(sc_raw, "margin_m", "surface_collision"), "surface_collision.margin_m")
    if margin_m < 0:
        raise ConfigError(f"'surface_collision.margin_m' must be >= 0, got {margin_m}")
    allowed_links_raw = _require(sc_raw, "allowed_links", "surface_collision")
    if not isinstance(allowed_links_raw, list):
        raise ConfigError("'surface_collision.allowed_links' must be a list")
    allowed_links = tuple(str(v) for v in allowed_links_raw)
    model = sc_raw.get("model", "slab")
    if model not in _SURFACE_MODELS:
        raise ConfigError(f"'surface_collision.model' must be one of {sorted(_SURFACE_MODELS)}, got {model!r}")
    surface_collision = {
        "enabled": sc_enabled,
        "thickness_m": thickness_m,
        "margin_m": margin_m,
        "allowed_links": allowed_links,
        "model": model,
    }
    if model == "specimen_block":
        floor_z_m = _as_float(_require(sc_raw, "floor_z_m", "surface_collision"), "surface_collision.floor_z_m")
        keepout_raw = _require(sc_raw, "base_keepout_m", "surface_collision")
        if not isinstance(keepout_raw, list) or len(keepout_raw) != 4:
            raise ConfigError("'surface_collision.base_keepout_m' must be [x_min, x_max, y_min, y_max]")
        keepout = tuple(_as_float(v, "surface_collision.base_keepout_m") for v in keepout_raw)
        if not (keepout[0] < keepout[1] and keepout[2] < keepout[3]):
            raise ConfigError(f"'surface_collision.base_keepout_m' must have min < max, got {keepout}")
        for z in surface_z_m:
            if z < floor_z_m:
                raise ConfigError(f"'grid.surface_z_m' entry {z} lies below surface_collision.floor_z_m {floor_z_m}")
        for x in axis_values(x_axis):
            for y in axis_values(y_axis):
                if keepout[0] < x < keepout[1] and keepout[2] < y < keepout[3]:
                    raise ConfigError(
                        f"grid node ({x:.4f}, {y:.4f}) lies inside surface_collision.base_keepout_m {keepout}: "
                        "a specimen cannot be placed on the robot base"
                    )
        surface_collision["floor_z_m"] = floor_z_m
        surface_collision["base_keepout_m"] = keepout
    elif "floor_z_m" in sc_raw or "base_keepout_m" in sc_raw:
        raise ConfigError("'surface_collision.floor_z_m'/'base_keepout_m' only apply to model 'specimen_block'")

    prefilter_raw = _require(raw, "prefilter", "<root>")
    _check_keys(prefilter_raw, _PREFILTER_KEYS, "prefilter")
    prefilter_enabled = _as_bool(_require(prefilter_raw, "enabled", "prefilter"), "prefilter.enabled")
    prefilter = {"enabled": prefilter_enabled}

    placement_raw = _require(raw, "placement", "<root>")
    _check_keys(placement_raw, _PLACEMENT_KEYS, "placement")
    footprint_raw = _require(placement_raw, "footprint_m", "placement")
    if not isinstance(footprint_raw, list) or len(footprint_raw) != 2:
        raise ConfigError("'placement.footprint_m' must be a 2-element list")
    footprint_m = (
        _as_float(footprint_raw[0], "placement.footprint_m"),
        _as_float(footprint_raw[1], "placement.footprint_m"),
    )
    if footprint_m[0] <= 0 or footprint_m[1] <= 0:
        raise ConfigError(f"'placement.footprint_m' entries must be > 0, got {footprint_m}")
    yaw_raw = _require(placement_raw, "yaw_candidates_rad", "placement")
    if not isinstance(yaw_raw, list) or len(yaw_raw) == 0:
        raise ConfigError("'placement.yaw_candidates_rad' must be a non-empty list")
    yaw_candidates_rad = tuple(_as_float(v, "placement.yaw_candidates_rad") for v in yaw_raw)
    tolerance_m = _as_float(_require(placement_raw, "tolerance_m", "placement"), "placement.tolerance_m")
    min_tolerance = max(x_axis["step"], y_axis["step"]) / 2.0
    if tolerance_m < min_tolerance - 1e-12:
        raise ConfigError(
            f"'placement.tolerance_m' ({tolerance_m}) must be >= max(x.step, y.step)/2 ({min_tolerance})"
        )
    top_k = _as_int(_require(placement_raw, "top_k", "placement"), "placement.top_k")
    if top_k < 1:
        raise ConfigError(f"'placement.top_k' must be >= 1, got {top_k}")
    placement = {
        "footprint_m": footprint_m,
        "yaw_candidates_rad": yaw_candidates_rad,
        "tolerance_m": tolerance_m,
        "top_k": top_k,
    }

    value_status = _require(raw, "value_status", "<root>")
    if value_status != "nominal":
        raise ConfigError(f"'value_status' must be 'nominal', got {value_status!r}")

    sources = _require(raw, "sources", "<root>")
    if not isinstance(sources, dict) or not sources:
        raise ConfigError("'sources' must be a non-empty mapping")
    for k, v in sources.items():
        if not isinstance(v, str) or not v:
            raise ConfigError(f"'sources.{k}' must be a non-empty string")

    result = {
        "frame": raw["frame"],
        "group": raw["group"],
        "ik_link": raw["ik_link"],
        "boresight": boresight,
        "grid": grid,
        "orientation": orientation,
        "ik": ik,
        "surface_collision": surface_collision,
        "prefilter": prefilter,
        "placement": placement,
        "value_status": value_status,
        "sources": dict(sources),
    }
    if "environment" in raw:
        env_raw = raw["environment"]
        _check_keys(env_raw, _ENVIRONMENT_KEYS, "environment")
        scene_config = _require(env_raw, "scene_config", "environment")
        if not isinstance(scene_config, str) or not scene_config:
            raise ConfigError("'environment.scene_config' must be a non-empty (repo-relative) path")
        objects = _require(env_raw, "objects", "environment")
        if (not isinstance(objects, list) or not objects
                or not all(isinstance(o, str) and o for o in objects) or len(set(objects)) != len(objects)):
            raise ConfigError("'environment.objects' must be a non-empty list of unique object ids")
        result["environment"] = {"scene_config": scene_config, "objects": tuple(objects)}
    return result


def config_sha256(path: Union[str, Path]) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# --------------------------------------------------------------------------------------
# orientation / pose geometry
# --------------------------------------------------------------------------------------

def _rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation matrix for a right-handed rotation of `angle` about `axis`."""
    norm = np.linalg.norm(axis)
    axis = np.asarray(axis, dtype=float) / norm
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1.0 - c
    return np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [x * z * C - y * s, y * z * C + x * s, z * z * C + c],
    ])


def _shortest_arc_rotation(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Shortest-arc rotation matrix R such that R @ a == b, for unit-ish vectors a, b."""
    a = np.asarray(a, dtype=float) / np.linalg.norm(a)
    b = np.asarray(b, dtype=float) / np.linalg.norm(b)
    c = float(np.dot(a, b))
    if c > 1.0 - 1e-12:
        return np.eye(3)
    if c < -1.0 + 1e-12:
        # antiparallel: any axis perpendicular to a works, rotate by pi about it.
        perp = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(perp) < 1e-8:
            perp = np.cross(a, np.array([0.0, 1.0, 0.0]))
        return _rotation_matrix(perp, math.pi)
    v = np.cross(a, b)
    angle = math.acos(max(-1.0, min(1.0, c)))
    return _rotation_matrix(v, angle)


def _quat_from_matrix(m: np.ndarray) -> np.ndarray:
    """ROS (x, y, z, w) unit quaternion for rotation matrix m, canonicalised to w >= 0."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q = q / np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return q


def _perp_basis(n: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    arbitrary = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(n, arbitrary)
    e1 = e1 / np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    e2 = e2 / np.linalg.norm(e2)
    return e1, e2


def _tilted_direction(down_world: np.ndarray, tilt_deg: float, azimuth_rad: float) -> np.ndarray:
    n = np.asarray(down_world, dtype=float)
    if tilt_deg == 0.0:
        return n.copy()
    e1, e2 = _perp_basis(n)
    tilt_axis = math.cos(azimuth_rad) * e1 + math.sin(azimuth_rad) * e2
    return _rotation_matrix(tilt_axis, math.radians(tilt_deg)) @ n


def orientations(cfg: dict) -> List[Orientation]:
    """Enumerate boresight-down orientations, ordered tilt asc, azimuth asc, roll asc."""
    axis_local = np.array(cfg["boresight"]["axis_local"], dtype=float)
    down_world = np.array(cfg["boresight"]["down_world"], dtype=float)
    roll_samples = cfg["orientation"]["roll_samples"]
    tilt_azimuth_samples = cfg["orientation"]["tilt_azimuth_samples"]

    result: List[Orientation] = []
    for tilt_deg in cfg["orientation"]["tilt_deg"]:
        if tilt_deg == 0.0:
            azimuths = [0.0]
        else:
            azimuths = [2.0 * math.pi * k / tilt_azimuth_samples for k in range(tilt_azimuth_samples)]
        for azimuth_rad in azimuths:
            d_world = _tilted_direction(down_world, tilt_deg, azimuth_rad)
            r0 = _shortest_arc_rotation(axis_local, d_world)
            for k in range(roll_samples):
                roll_rad = 2.0 * math.pi * k / roll_samples
                r_roll = _rotation_matrix(d_world, roll_rad)
                r = r_roll @ r0
                quat = _quat_from_matrix(r)
                result.append(
                    Orientation(
                        tilt_deg=tilt_deg,
                        azimuth_rad=azimuth_rad,
                        roll_rad=roll_rad,
                        d_world=d_world,
                        quat_xyzw=quat,
                    )
                )
    return result


def target_position(surface_xyz: Sequence[float], standoff_m: float, d_world: Sequence[float]) -> np.ndarray:
    return np.array(surface_xyz, dtype=float) - standoff_m * np.array(d_world, dtype=float)


Box = Tuple[Tuple[float, float, float], Tuple[float, float, float]]  # (centre, size), axis-aligned, cfg frame


def specimen_proxy_boxes(cfg: dict, surface_z: float) -> List[Box]:
    """Axis-aligned boxes standing in for the inspected specimen at one surface height.

    Region = grid x/y extent dilated by margin_m. The top face is _SURFACE_GAP_M below `surface_z`.
    - model `slab` (MOT-04.4 behaviour): one box of thickness_m over the whole region.
    - model `specimen_block` (ADR-014): the region minus base_keepout_m (a specimen cannot overlap the
      robot base), as up to four boxes reaching down to floor_z_m (the table top the specimen stands
      on). Returns [] when the surface is at the floor (the table itself is the surface).
    """
    sc = cfg["surface_collision"]
    x_axis, y_axis = cfg["grid"]["x_m"], cfg["grid"]["y_m"]
    m = sc["margin_m"]
    x0, x1 = x_axis["min"] - m, x_axis["max"] + m
    y0, y1 = y_axis["min"] - m, y_axis["max"] + m
    top = surface_z - _SURFACE_GAP_M
    if sc.get("model", "slab") == "slab":
        t = sc["thickness_m"]
        return [(((x0 + x1) / 2.0, (y0 + y1) / 2.0, top - t / 2.0), (x1 - x0, y1 - y0, t))]
    bottom = sc["floor_z_m"]
    if top - bottom <= 1e-6:
        return []
    kx0, kx1, ky0, ky1 = sc["base_keepout_m"]
    rects = [
        (max(x0, kx1), x1, y0, y1),                               # in front of the base
        (x0, min(x1, kx0), y0, y1),                               # behind the base
        (max(x0, kx0), min(x1, kx1), max(y0, ky1), y1),           # beside the base, +y
        (max(x0, kx0), min(x1, kx1), y0, min(y1, ky0)),           # beside the base, -y
    ]
    boxes: List[Box] = []
    for rx0, rx1, ry0, ry1 in rects:
        if rx1 - rx0 > 1e-9 and ry1 - ry0 > 1e-9:
            boxes.append((((rx0 + rx1) / 2.0, (ry0 + ry1) / 2.0, (top + bottom) / 2.0),
                          (rx1 - rx0, ry1 - ry0, top - bottom)))
    return boxes


# --------------------------------------------------------------------------------------
# grid enumeration
# --------------------------------------------------------------------------------------

def enumerate_targets(cfg: dict) -> List[Target]:
    """Deterministic serpentine sweep: surface_z asc, x asc, y serpentine, standoff asc."""
    x_vals = axis_values(cfg["grid"]["x_m"])
    y_vals = axis_values(cfg["grid"]["y_m"])
    z_vals = cfg["grid"]["surface_z_m"]
    standoffs = cfg["grid"]["standoffs_m"]

    targets: List[Target] = []
    for iz, z in enumerate(z_vals):
        for ix, x in enumerate(x_vals):
            y_indices = range(len(y_vals)) if ix % 2 == 0 else range(len(y_vals) - 1, -1, -1)
            for iy in y_indices:
                y = y_vals[iy]
                for i_standoff, standoff in enumerate(standoffs):
                    target_id = f"z{iz:02d}_x{ix:03d}_y{iy:03d}_s{i_standoff:d}"
                    targets.append(Target(target_id, iz, ix, iy, x, y, z, standoff))
    return targets


# --------------------------------------------------------------------------------------
# reach bound from URDF
# --------------------------------------------------------------------------------------

def reach_bound_from_urdf(urdf_xml: str, base_link: str, tip_link: str) -> float:
    """Sum of joint-origin norms from `tip_link` back to `base_link`.

    A strict upper bound on ||p_tip|| in base_link, for any joint angles, by the triangle
    inequality: each joint contributes a vector whose norm equals its fixed origin.xyz norm
    (rotation preserves norm), so the norm of their sum can never exceed the sum of norms.
    """
    root = ET.fromstring(urdf_xml)
    joints_by_child: Dict[str, Tuple[str, np.ndarray, str]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_name = parent.get("link")
        child_name = child.get("link")
        joint_type = joint.get("type")
        origin_elem = joint.find("origin")
        xyz = (0.0, 0.0, 0.0)
        if origin_elem is not None and origin_elem.get("xyz"):
            xyz = tuple(float(v) for v in origin_elem.get("xyz").split())
        joints_by_child[child_name] = (joint_type, np.array(xyz, dtype=float), parent_name)

    total = 0.0
    current = tip_link
    visited = set()
    while current != base_link:
        if current in visited:
            raise ValueError(f"cycle detected in URDF joint chain at link '{current}'")
        visited.add(current)
        if current not in joints_by_child:
            raise ValueError(
                f"broken chain: no joint has child link '{current}' while walking to base_link '{base_link}'"
            )
        joint_type, xyz, parent_name = joints_by_child[current]
        if joint_type in _UNSUPPORTED_JOINT_TYPES:
            raise ValueError(
                f"joint chain contains unsupported joint type '{joint_type}' at child link '{current}'"
            )
        total += float(np.linalg.norm(xyz))
        current = parent_name
    return total


def prefiltered(position: Sequence[float], bound: float) -> bool:
    """True if `position` (base_link) is provably outside the conservative reach bound."""
    return float(np.linalg.norm(np.asarray(position, dtype=float))) > bound + 1e-6

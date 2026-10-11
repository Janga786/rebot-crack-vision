"""cartesian_core — pure-Python building blocks for Cartesian crack-path planning (MOT-06.3).

Stdlib + numpy + PyYAML only -- no rclpy or *_msgs import anywhere in this file (mirrors
reachability_core.py/scene_core.py), so it runs standalone under the system python3 without a
colcon build. Normative source: docs/INTERFACES.md §13 (schema
`crackvision.cartesian_planning_config/1`) and docs/adr/017-cartesian-crack-path-planning.md.

This module covers only the ROS-free pieces MOT-06.3 scopes:
  - `load_config` -- the §13.2 planning-config loader.
  - `load_paths3d` -- a §10 `paths3d` reader restricted to the subset the planner relies on
    (schema id, frame, segment/waypoint phase ordering, finite unit vectors/quaternions).
  - `segment_tasks` -- §13.3 segment iteration order, turning each segment's waypoints into a
    `SegmentTask`.
  - `candidates` -- the §13.4 direction x roll-offset candidate set, in its fixed enumeration/
    tie-break order.
  - `apply_candidate` -- the §13.4 pose transform (uniform roll about each pose's local +X;
    `reverse` reorders/relabels approach and retract).
  - `split_remainder` -- the §13.7 remainder sub-task boundary (no planning -- just the
    synthesised approach/retract geometry for the still-unreached trace waypoints).
  - `surface_scene_mismatch` -- the §13.9 pre-planning pure surface/scene check.

The actual IK/Cartesian/re-check planning loop, plan-file assembly and CLIs (§13.5-13.8,
§13.11-13.15) are later MOT-06 cards.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Dict, List, NamedTuple, Optional, Sequence, Tuple, Union

import numpy as np
import yaml

SCHEMA_ID = "crackvision.cartesian_planning_config/1"
PATHS3D_SCHEMA_ID = "crackvision.paths3d/1"
_FRAME = "base_link"

_MIN_REMAINDER_WAYPOINTS = 3

_TOP_LEVEL_KEYS = frozenset({
    "schema", "candidates", "cartesian", "recheck", "splitting", "end_state",
    "surface_object_id", "surface_object_tolerance_m", "service_timeout_s", "ik_solver_timeout_s",
    "view", "value_status", "sources",
})
_CANDIDATES_KEYS = frozenset({"roll_offsets_deg"})
_CARTESIAN_KEYS = frozenset({
    "max_step_m", "jump_threshold", "prismatic_jump_threshold", "revolute_jump_threshold",
})
_RECHECK_KEYS = frozenset({"fk_tolerance_m", "fk_tolerance_deg", "max_joint_step_rad"})
_SPLITTING_KEYS = frozenset({"max_splits_per_segment", "min_waypoints_per_remainder"})
_VIEW_KEYS = frozenset({
    "view_distance_m", "tilt_candidates_deg", "yaw_candidates_deg", "fov_deg", "fov_margin_deg",
    "footprint_m", "surface_z_m",
})
_FOV_DEG_KEYS = frozenset({"horizontal", "vertical"})
_END_STATES = frozenset({"start", "home", "none"})


class CartesianConfigError(ValueError):
    """Raised for any structurally or numerically invalid cartesian_planning.yaml."""


class Paths3dError(ValueError):
    """Raised for any structurally or numerically invalid paths3d (§10) document, restricted to
    the subset this planner relies on."""


class Pose(NamedTuple):
    position_m: Tuple[float, float, float]
    quat_xyzw: Tuple[float, float, float, float]


class SegmentTask(NamedTuple):
    segment_ref: Dict[str, Any]
    approach: Pose
    trace: Tuple[Pose, ...]
    retract: Pose
    normals: Tuple[Tuple[float, float, float], ...]
    surface_points: Tuple[Tuple[float, float, float], ...]
    trace_length_m: float
    clearances_m: Tuple[float, ...] = ()


class Candidate(NamedTuple):
    direction: str
    roll_offset_rad: float


# --------------------------------------------------------------------------------------
# config validation (§13.2)
# --------------------------------------------------------------------------------------

def _check_keys(d: object, allowed: frozenset, ctx: str) -> None:
    if not isinstance(d, dict):
        raise CartesianConfigError(f"'{ctx}' must be a mapping")
    unknown = set(d.keys()) - allowed
    if unknown:
        raise CartesianConfigError(f"unknown key(s) in '{ctx}': {sorted(unknown)}")


def _require(d: dict, key: str, ctx: str) -> Any:
    if key not in d:
        raise CartesianConfigError(f"'{ctx}' is missing required key '{key}'")
    return d[key]


def _as_float(v: object, ctx: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise CartesianConfigError(f"'{ctx}' must be numeric, got {v!r}")
    value = float(v)
    if not math.isfinite(value):
        raise CartesianConfigError(f"'{ctx}' must be finite, got {v!r}")
    return value


def _as_int(v: object, ctx: str) -> int:
    if isinstance(v, bool) or not isinstance(v, int):
        raise CartesianConfigError(f"'{ctx}' must be an int, got {v!r}")
    return v


def _as_nonempty_str(v: object, ctx: str) -> str:
    if not isinstance(v, str) or not v:
        raise CartesianConfigError(f"'{ctx}' must be a non-empty string, got {v!r}")
    return v


def _as_float_list(v: object, ctx: str, *, nonempty: bool = True) -> List[float]:
    if not isinstance(v, list) or (nonempty and not v):
        raise CartesianConfigError(f"'{ctx}' must be a non-empty list")
    return [_as_float(x, ctx) for x in v]


def _load_candidates(raw: Any) -> Dict[str, Any]:
    _check_keys(raw, _CANDIDATES_KEYS, "candidates")
    offsets_deg = _as_float_list(_require(raw, "roll_offsets_deg", "candidates"), "candidates.roll_offsets_deg")
    for v in offsets_deg:
        if not (-180.0 <= v <= 180.0):
            raise CartesianConfigError(
                f"'candidates.roll_offsets_deg' entries must lie in [-180, 180], got {v}"
            )
    if len(set(offsets_deg)) != len(offsets_deg):
        raise CartesianConfigError("'candidates.roll_offsets_deg' must not contain duplicate values")
    return {"roll_offsets_deg": offsets_deg}


def _load_cartesian(raw: Any) -> Dict[str, Any]:
    _check_keys(raw, _CARTESIAN_KEYS, "cartesian")
    max_step_m = _as_float(_require(raw, "max_step_m", "cartesian"), "cartesian.max_step_m")
    if max_step_m <= 0:
        raise CartesianConfigError(f"'cartesian.max_step_m' must be > 0, got {max_step_m}")
    result = {"max_step_m": max_step_m}
    for key in ("jump_threshold", "prismatic_jump_threshold", "revolute_jump_threshold"):
        value = _as_float(_require(raw, key, "cartesian"), f"cartesian.{key}")
        if value < 0:
            raise CartesianConfigError(f"'cartesian.{key}' must be >= 0, got {value}")
        result[key] = value
    return result


def _load_recheck(raw: Any) -> Dict[str, Any]:
    _check_keys(raw, _RECHECK_KEYS, "recheck")
    fk_tolerance_m = _as_float(_require(raw, "fk_tolerance_m", "recheck"), "recheck.fk_tolerance_m")
    if fk_tolerance_m <= 0:
        raise CartesianConfigError(f"'recheck.fk_tolerance_m' must be > 0, got {fk_tolerance_m}")
    fk_tolerance_deg = _as_float(_require(raw, "fk_tolerance_deg", "recheck"), "recheck.fk_tolerance_deg")
    if fk_tolerance_deg <= 0:
        raise CartesianConfigError(f"'recheck.fk_tolerance_deg' must be > 0, got {fk_tolerance_deg}")
    max_joint_step_rad = _as_float(
        _require(raw, "max_joint_step_rad", "recheck"), "recheck.max_joint_step_rad"
    )
    if max_joint_step_rad <= 0:
        raise CartesianConfigError(f"'recheck.max_joint_step_rad' must be > 0, got {max_joint_step_rad}")
    return {
        "fk_tolerance_m": fk_tolerance_m,
        "fk_tolerance_deg": fk_tolerance_deg,
        "max_joint_step_rad": max_joint_step_rad,
    }


def _load_splitting(raw: Any) -> Dict[str, Any]:
    _check_keys(raw, _SPLITTING_KEYS, "splitting")
    max_splits_per_segment = _as_int(
        _require(raw, "max_splits_per_segment", "splitting"), "splitting.max_splits_per_segment"
    )
    if max_splits_per_segment < 0:
        raise CartesianConfigError(
            f"'splitting.max_splits_per_segment' must be >= 0, got {max_splits_per_segment}"
        )
    min_waypoints_per_remainder = _as_int(
        _require(raw, "min_waypoints_per_remainder", "splitting"), "splitting.min_waypoints_per_remainder"
    )
    if min_waypoints_per_remainder < 1:
        raise CartesianConfigError(
            f"'splitting.min_waypoints_per_remainder' must be >= 1, got {min_waypoints_per_remainder}"
        )
    return {
        "max_splits_per_segment": max_splits_per_segment,
        "min_waypoints_per_remainder": min_waypoints_per_remainder,
    }


def _load_view(raw: Any) -> Dict[str, Any]:
    _check_keys(raw, _VIEW_KEYS, "view")
    view_distance_m = _as_float(_require(raw, "view_distance_m", "view"), "view.view_distance_m")
    if not (0.20 <= view_distance_m <= 0.30):
        raise CartesianConfigError(f"'view.view_distance_m' must lie in [0.20, 0.30], got {view_distance_m}")
    tilt_candidates_deg = _as_float_list(
        _require(raw, "tilt_candidates_deg", "view"), "view.tilt_candidates_deg"
    )
    for t in tilt_candidates_deg:
        if not (0.0 <= t <= 15.0):
            raise CartesianConfigError(f"'view.tilt_candidates_deg' entries must lie in [0, 15], got {t}")
    yaw_candidates_deg = _as_float_list(_require(raw, "yaw_candidates_deg", "view"), "view.yaw_candidates_deg")
    fov_raw = _require(raw, "fov_deg", "view")
    _check_keys(fov_raw, _FOV_DEG_KEYS, "view.fov_deg")
    horizontal = _as_float(_require(fov_raw, "horizontal", "view.fov_deg"), "view.fov_deg.horizontal")
    vertical = _as_float(_require(fov_raw, "vertical", "view.fov_deg"), "view.fov_deg.vertical")
    if not (0.0 < horizontal <= 360.0):
        raise CartesianConfigError(f"'view.fov_deg.horizontal' must lie in (0, 360], got {horizontal}")
    if not (0.0 < vertical <= 360.0):
        raise CartesianConfigError(f"'view.fov_deg.vertical' must lie in (0, 360], got {vertical}")
    fov_margin_deg = _as_float(_require(raw, "fov_margin_deg", "view"), "view.fov_margin_deg")
    if fov_margin_deg < 0:
        raise CartesianConfigError(f"'view.fov_margin_deg' must be >= 0, got {fov_margin_deg}")
    footprint_raw = _require(raw, "footprint_m", "view")
    if not isinstance(footprint_raw, list) or len(footprint_raw) != 2:
        raise CartesianConfigError("'view.footprint_m' must be a 2-element list")
    footprint_m = (
        _as_float(footprint_raw[0], "view.footprint_m"),
        _as_float(footprint_raw[1], "view.footprint_m"),
    )
    if footprint_m[0] <= 0 or footprint_m[1] <= 0:
        raise CartesianConfigError(f"'view.footprint_m' entries must be > 0, got {footprint_m}")
    surface_z_m = _as_float(_require(raw, "surface_z_m", "view"), "view.surface_z_m")
    return {
        "view_distance_m": view_distance_m,
        "tilt_candidates_deg": tilt_candidates_deg,
        "yaw_candidates_deg": yaw_candidates_deg,
        "fov_deg": {"horizontal": horizontal, "vertical": vertical},
        "fov_margin_deg": fov_margin_deg,
        "footprint_m": footprint_m,
        "surface_z_m": surface_z_m,
    }


def load_config(path: Union[str, Path]) -> Dict[str, Any]:
    """Load and validate a `crackvision.cartesian_planning_config/1` YAML file (§13.2). Raises
    `CartesianConfigError` on any unknown/missing key or out-of-range value."""
    raw = yaml.safe_load(Path(path).read_text())
    if not isinstance(raw, dict):
        raise CartesianConfigError("config root must be a mapping")
    _check_keys(raw, _TOP_LEVEL_KEYS, "<root>")

    schema = _require(raw, "schema", "<root>")
    if schema != SCHEMA_ID:
        raise CartesianConfigError(f"'schema' must be {SCHEMA_ID!r}, got {schema!r}")

    candidates_cfg = _load_candidates(_require(raw, "candidates", "<root>"))
    cartesian_cfg = _load_cartesian(_require(raw, "cartesian", "<root>"))
    recheck_cfg = _load_recheck(_require(raw, "recheck", "<root>"))
    splitting_cfg = _load_splitting(_require(raw, "splitting", "<root>"))

    end_state = _require(raw, "end_state", "<root>")
    if end_state not in _END_STATES:
        raise CartesianConfigError(f"'end_state' must be one of {sorted(_END_STATES)}, got {end_state!r}")

    surface_object_id = _as_nonempty_str(_require(raw, "surface_object_id", "<root>"), "surface_object_id")
    surface_object_tolerance_m = _as_float(
        _require(raw, "surface_object_tolerance_m", "<root>"), "surface_object_tolerance_m"
    )
    if surface_object_tolerance_m < 0:
        raise CartesianConfigError(
            f"'surface_object_tolerance_m' must be >= 0, got {surface_object_tolerance_m}"
        )

    service_timeout_s = _as_float(_require(raw, "service_timeout_s", "<root>"), "service_timeout_s")
    if service_timeout_s <= 0:
        raise CartesianConfigError(f"'service_timeout_s' must be > 0, got {service_timeout_s}")

    ik_solver_timeout_s = _as_float(
        _require(raw, "ik_solver_timeout_s", "<root>"), "ik_solver_timeout_s"
    )
    if ik_solver_timeout_s <= 0:
        raise CartesianConfigError(f"'ik_solver_timeout_s' must be > 0, got {ik_solver_timeout_s}")

    view_cfg = _load_view(_require(raw, "view", "<root>"))

    value_status = _require(raw, "value_status", "<root>")
    if value_status != "nominal":
        raise CartesianConfigError(f"'value_status' must be 'nominal', got {value_status!r}")

    sources = _require(raw, "sources", "<root>")
    if not isinstance(sources, dict) or not sources:
        raise CartesianConfigError("'sources' must be a non-empty mapping")
    for k, v in sources.items():
        if not isinstance(v, str) or not v:
            raise CartesianConfigError(f"'sources.{k}' must be a non-empty string")

    return {
        "schema": schema,
        "candidates": candidates_cfg,
        "cartesian": cartesian_cfg,
        "recheck": recheck_cfg,
        "splitting": splitting_cfg,
        "end_state": end_state,
        "surface_object_id": surface_object_id,
        "surface_object_tolerance_m": surface_object_tolerance_m,
        "service_timeout_s": service_timeout_s,
        "ik_solver_timeout_s": ik_solver_timeout_s,
        "view": view_cfg,
        "value_status": value_status,
        "sources": dict(sources),
    }


def config_sha256(path: Union[str, Path]) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# --------------------------------------------------------------------------------------
# paths3d loading (§10 subset)
# --------------------------------------------------------------------------------------

def _finite_vec(v: Any, ctx: str, n: int) -> None:
    if not (isinstance(v, (list, tuple)) and len(v) == n):
        raise Paths3dError(f"'{ctx}' must be a {n}-element list")
    for x in v:
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
            raise Paths3dError(f"'{ctx}' entries must be finite numbers, got {v!r}")


def _finite_unit_vec3(v: Any, ctx: str, tol: float = 1e-3) -> None:
    _finite_vec(v, ctx, 3)
    norm = math.sqrt(sum(float(x) * float(x) for x in v))
    if abs(norm - 1.0) > tol:
        raise Paths3dError(f"'{ctx}' must be a unit vector, got norm {norm}")


def _finite_unit_quat(v: Any, ctx: str, tol: float = 1e-3) -> None:
    _finite_vec(v, ctx, 4)
    norm = math.sqrt(sum(float(x) * float(x) for x in v))
    if abs(norm - 1.0) > tol:
        raise Paths3dError(f"'{ctx}' must be a unit quaternion, got norm {norm}")


def _validate_waypoint(wp: Any, ctx: str) -> None:
    if not isinstance(wp, dict):
        raise Paths3dError(f"'{ctx}' must be a mapping")
    phase = wp.get("phase")
    if phase not in ("approach", "trace", "retract"):
        raise Paths3dError(f"'{ctx}.phase' must be one of approach/trace/retract, got {phase!r}")
    _finite_vec(wp.get("position_m"), f"{ctx}.position_m", 3)
    _finite_unit_quat(wp.get("quat_xyzw"), f"{ctx}.quat_xyzw")
    _finite_vec(wp.get("surface_point_m"), f"{ctx}.surface_point_m", 3)
    _finite_unit_vec3(wp.get("normal_base"), f"{ctx}.normal_base")
    clearance_m = wp.get("clearance_m")
    if isinstance(clearance_m, bool) or not isinstance(clearance_m, (int, float)) \
            or not math.isfinite(clearance_m) or clearance_m <= 0:
        raise Paths3dError(f"'{ctx}.clearance_m' must be a finite positive number, got {clearance_m!r}")


def _validate_segment(segment: Any, ctx: str) -> None:
    if not isinstance(segment, dict):
        raise Paths3dError(f"'{ctx}' must be a mapping")
    waypoints = segment.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) < 3:
        raise Paths3dError(
            f"'{ctx}.waypoints' must be a list of at least 3 (approach, >=1 trace, retract)"
        )
    for wi, wp in enumerate(waypoints):
        _validate_waypoint(wp, f"{ctx}.waypoints[{wi}]")
    phases = [wp["phase"] for wp in waypoints]
    if phases[0] != "approach":
        raise Paths3dError(f"'{ctx}.waypoints[0].phase' must be 'approach', got {phases[0]!r}")
    if phases[-1] != "retract":
        raise Paths3dError(f"'{ctx}.waypoints[-1].phase' must be 'retract', got {phases[-1]!r}")
    if any(p != "trace" for p in phases[1:-1]):
        raise Paths3dError(f"'{ctx}.waypoints' middle entries must all be phase 'trace'")


def _validate_paths3d(doc: Any) -> None:
    if not isinstance(doc, dict):
        raise Paths3dError("paths3d document must be a mapping at the top level")
    schema = doc.get("schema")
    if schema != PATHS3D_SCHEMA_ID:
        raise Paths3dError(f"'schema' must be {PATHS3D_SCHEMA_ID!r}, got {schema!r}")
    frame = doc.get("frame")
    if frame != _FRAME:
        raise Paths3dError(f"'frame' must be {_FRAME!r}, got {frame!r}")

    components = doc.get("components")
    if not isinstance(components, list) or not components:
        raise Paths3dError("'components' must be a non-empty list")

    seen_order_index = set()
    for ci, comp in enumerate(components):
        ctx = f"components[{ci}]"
        if not isinstance(comp, dict):
            raise Paths3dError(f"'{ctx}' must be a mapping")
        order_index = comp.get("order_index")
        if isinstance(order_index, bool) or not isinstance(order_index, int):
            raise Paths3dError(f"'{ctx}.order_index' must be an int")
        if order_index in seen_order_index:
            raise Paths3dError(f"duplicate 'order_index' {order_index} in 'components'")
        seen_order_index.add(order_index)
        component_id = comp.get("component_id")
        if not isinstance(component_id, str) or not component_id:
            raise Paths3dError(f"'{ctx}.component_id' must be a non-empty string")

        polylines = comp.get("polylines")
        if not isinstance(polylines, list) or not polylines:
            raise Paths3dError(f"'{ctx}.polylines' must be a non-empty list")
        seen_main = False
        seen_branch_indices = set()
        for pi, polyline in enumerate(polylines):
            pctx = f"{ctx}.polylines[{pi}]"
            if not isinstance(polyline, dict):
                raise Paths3dError(f"'{pctx}' must be a mapping")
            kind = polyline.get("kind")
            if kind not in ("main", "branch"):
                raise Paths3dError(f"'{pctx}.kind' must be 'main' or 'branch', got {kind!r}")
            branch_index = polyline.get("branch_index")
            if kind == "main":
                if branch_index is not None:
                    raise Paths3dError(f"'{pctx}.branch_index' must be null for kind 'main'")
                if seen_main:
                    raise Paths3dError(f"'{ctx}.polylines' has more than one 'main' polyline")
                seen_main = True
            else:
                if isinstance(branch_index, bool) or not isinstance(branch_index, int) or branch_index < 0:
                    raise Paths3dError(
                        f"'{pctx}.branch_index' must be a non-negative int for kind 'branch'"
                    )
                if branch_index in seen_branch_indices:
                    raise Paths3dError(f"duplicate 'branch_index' {branch_index} in '{ctx}.polylines'")
                seen_branch_indices.add(branch_index)

            segments = polyline.get("segments")
            if not isinstance(segments, list):
                raise Paths3dError(f"'{pctx}.segments' must be a list")
            for si, segment in enumerate(segments):
                _validate_segment(segment, f"{pctx}.segments[{si}]")

        if not seen_main:
            raise Paths3dError(f"'{ctx}.polylines' has no 'main' polyline")


def load_paths3d(path: Union[str, Path]) -> Tuple[Dict[str, Any], str]:
    """Load + validate a `crackvision.paths3d/1` document (§10), restricted to the subset the
    planner relies on. Returns `(doc, sha256)`. Raises `Paths3dError` on any structural problem
    (never a bare `KeyError`)."""
    path = Path(path)
    try:
        raw_bytes = path.read_bytes()
    except OSError as exc:
        raise Paths3dError(f"could not read paths3d file '{path}': {exc}") from exc
    sha256 = hashlib.sha256(raw_bytes).hexdigest()
    try:
        doc = json.loads(raw_bytes.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise Paths3dError(f"malformed JSON in '{path}': {exc}") from exc
    _validate_paths3d(doc)
    return doc, sha256


# --------------------------------------------------------------------------------------
# segment iteration (§13.3)
# --------------------------------------------------------------------------------------

def _polyline_length_m(points: Sequence[Sequence[float]]) -> float:
    if len(points) < 2:
        return 0.0
    arr = np.asarray(points, dtype=float)
    return float(np.sum(np.linalg.norm(arr[1:] - arr[:-1], axis=1)))


def _segment_task(comp: Dict[str, Any], polyline: Dict[str, Any], seg_index: int, segment: Dict[str, Any]) -> SegmentTask:
    waypoints = segment["waypoints"]
    approach_wp = waypoints[0]
    retract_wp = waypoints[-1]
    trace_wps = waypoints[1:-1]

    approach = Pose(tuple(float(x) for x in approach_wp["position_m"]), tuple(float(x) for x in approach_wp["quat_xyzw"]))
    retract = Pose(tuple(float(x) for x in retract_wp["position_m"]), tuple(float(x) for x in retract_wp["quat_xyzw"]))
    trace = tuple(
        Pose(tuple(float(x) for x in wp["position_m"]), tuple(float(x) for x in wp["quat_xyzw"]))
        for wp in trace_wps
    )
    normals = tuple(tuple(float(x) for x in wp["normal_base"]) for wp in trace_wps)
    surface_points = tuple(tuple(float(x) for x in wp["surface_point_m"]) for wp in trace_wps)
    clearances_m = tuple(float(wp["clearance_m"]) for wp in trace_wps)
    trace_length_m = _polyline_length_m([wp["position_m"] for wp in trace_wps])

    segment_ref = {
        "component_id": comp["component_id"],
        "polyline_kind": polyline["kind"],
        "branch_index": polyline["branch_index"],
        "segment_index": seg_index,
        "sub_segment": None,
    }
    return SegmentTask(
        segment_ref, approach, trace, retract, normals, surface_points, trace_length_m, clearances_m,
    )


def segment_tasks(doc: Dict[str, Any]) -> List[SegmentTask]:
    """Ordered `SegmentTask`s per §13.3: `components[]` by `order_index` ascending, `main` before
    `branch` (by `branch_index` ascending) within a component, `segments[]` in file order within
    a polyline."""
    tasks: List[SegmentTask] = []
    components = sorted(doc["components"], key=lambda c: c["order_index"])
    for comp in components:
        polylines = sorted(
            comp["polylines"],
            key=lambda p: (0, 0) if p["kind"] == "main" else (1, p["branch_index"]),
        )
        for polyline in polylines:
            for seg_index, segment in enumerate(polyline["segments"]):
                tasks.append(_segment_task(comp, polyline, seg_index, segment))
    return tasks


# --------------------------------------------------------------------------------------
# candidate set + pose transform (§13.4)
# --------------------------------------------------------------------------------------

def candidates(cfg: Dict[str, Any]) -> List[Candidate]:
    """Deterministic `{forward, reverse} x roll_offset` candidate set (§13.4): roll magnitude
    ascending, positive before negative at equal magnitude, forward before reverse."""
    offsets_deg = cfg["candidates"]["roll_offsets_deg"]
    ordered_degs = sorted(offsets_deg, key=lambda v: (abs(v), 0 if v >= 0 else 1))
    result: List[Candidate] = []
    for deg in ordered_degs:
        roll_rad = math.radians(deg)
        for direction in ("forward", "reverse"):
            result.append(Candidate(direction, roll_rad))
    return result


def _quat_mul(q1: Sequence[float], q2: Sequence[float]) -> Tuple[float, float, float, float]:
    """xyzw quaternion product q1 ⊗ q2."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


def _apply_roll(pose: Pose, roll_rad: float) -> Pose:
    """Intrinsic rotation by `roll_rad` about `pose`'s own local +X (§13.4): position unchanged."""
    qx = (math.sin(roll_rad / 2.0), 0.0, 0.0, math.cos(roll_rad / 2.0))
    return Pose(pose.position_m, _quat_mul(pose.quat_xyzw, qx))


def apply_candidate(task: SegmentTask, direction: str, roll_rad: float) -> SegmentTask:
    """Apply a (direction, roll) candidate to `task` (§13.4). `reverse` reorders the pose
    sequence to `[retract, reverse(trace), approach]` and swaps the approach/retract labels
    (every pose's `position_m`/base `quat_xyzw` is unchanged by this); the roll offset then
    rotates every pose about its own local +X, leaving `position_m` unchanged."""
    if direction not in ("forward", "reverse"):
        raise ValueError(f"direction must be 'forward' or 'reverse', got {direction!r}")

    if direction == "reverse":
        approach, retract = task.retract, task.approach
        trace = tuple(reversed(task.trace))
        normals = tuple(reversed(task.normals))
        surface_points = tuple(reversed(task.surface_points))
        clearances_m = tuple(reversed(task.clearances_m))
    else:
        approach, retract, trace, normals, surface_points, clearances_m = (
            task.approach, task.retract, task.trace, task.normals, task.surface_points, task.clearances_m
        )

    approach = _apply_roll(approach, roll_rad)
    retract = _apply_roll(retract, roll_rad)
    trace = tuple(_apply_roll(p, roll_rad) for p in trace)

    return SegmentTask(
        task.segment_ref, approach, trace, retract, normals, surface_points, task.trace_length_m, clearances_m,
    )


# --------------------------------------------------------------------------------------
# partial-trace split (§13.7, geometry only)
# --------------------------------------------------------------------------------------

def split_remainder(task: SegmentTask, last_reached_index: int) -> Optional[SegmentTask]:
    """The still-unreached remainder of `task.trace` strictly after `last_reached_index`, with a
    freshly synthesised approach/retract (§10.5 point 4, offset from the remainder's own end
    normals, same orientation as the bounding trace waypoint), or `None` when fewer than 3
    waypoints remain (§13.7)."""
    remainder_trace = task.trace[last_reached_index + 1:]
    remainder_normals = task.normals[last_reached_index + 1:]
    remainder_surface_points = task.surface_points[last_reached_index + 1:]
    remainder_clearances_m = task.clearances_m[last_reached_index + 1:]

    if len(remainder_trace) < _MIN_REMAINDER_WAYPOINTS:
        return None

    offset_m = float(
        np.linalg.norm(np.asarray(task.approach.position_m) - np.asarray(task.surface_points[0]))
    )
    approach = Pose(
        tuple(np.asarray(remainder_surface_points[0]) + offset_m * np.asarray(remainder_normals[0])),
        remainder_trace[0].quat_xyzw,
    )
    retract = Pose(
        tuple(np.asarray(remainder_surface_points[-1]) + offset_m * np.asarray(remainder_normals[-1])),
        remainder_trace[-1].quat_xyzw,
    )

    sub_segment = 0 if task.segment_ref["sub_segment"] is None else task.segment_ref["sub_segment"] + 1
    segment_ref = dict(task.segment_ref)
    segment_ref["sub_segment"] = sub_segment

    trace_length_m = _polyline_length_m([p.position_m for p in remainder_trace])
    return SegmentTask(
        segment_ref, approach, remainder_trace, retract, remainder_normals, remainder_surface_points,
        trace_length_m, remainder_clearances_m,
    )


# --------------------------------------------------------------------------------------
# surface/scene consistency check (§13.9, pre-planning pure)
# --------------------------------------------------------------------------------------

def _rotation_from_rpy(rpy: Sequence[float]) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll), matching scene_apply._quat_from_rpy's convention."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def _box_signed_distance(point: Sequence[float], pose: Dict[str, Any], dimensions_m: Sequence[float]) -> float:
    """Exact Euclidean distance to a rotated box's surface; negative inside (§13.9)."""
    p = np.asarray(point, dtype=float)
    position = np.asarray(pose["position_m"], dtype=float)
    rotation = _rotation_from_rpy(pose["rpy_rad"])
    local = rotation.T @ (p - position)
    half = np.asarray(dimensions_m, dtype=float) / 2.0
    q = np.abs(local) - half
    outside = np.maximum(q, 0.0)
    outside_dist = float(np.linalg.norm(outside))
    if outside_dist > 0.0:
        return outside_dist
    return float(np.max(q))


def surface_scene_mismatch(
    task: SegmentTask, scene_cfg: Dict[str, Any], surface_object_id: str, surface_object_tolerance_m: float,
) -> List[str]:
    """Every trace waypoint position closer than its own `clearances_m` entry to, or inside, any
    `scene_cfg` object's (possibly rotated) box (§13.9). Each waypoint sits exactly `clearance_m`
    from `surface_object_id` by construction (§10.5 point 2), so that one object is checked against
    the relaxed bound `clearance_m - surface_object_tolerance_m` instead of the full `clearance_m`;
    every other object uses the waypoint's full `clearance_m`. Independent of direction/roll --
    `position_m` is unaffected by either (§13.4). Returns `[]` when every trace waypoint clears
    every object."""
    reasons: List[str] = []
    for i, pose in enumerate(task.trace):
        clearance_m = task.clearances_m[i]
        for obj in scene_cfg["objects"]:
            distance_m = _box_signed_distance(pose.position_m, obj["pose"], obj["dimensions_m"])
            bound_m = (
                clearance_m - surface_object_tolerance_m if obj["id"] == surface_object_id else clearance_m
            )
            if distance_m < bound_m:
                reasons.append(
                    f"trace waypoint {i} is {distance_m:.4f} m from scene object '{obj['id']}', "
                    f"less than the required clearance {bound_m:.4f} m"
                )
    return reasons

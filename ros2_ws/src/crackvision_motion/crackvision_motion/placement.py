"""Specimen placement scoring, recommendation and verification-grid emission (MOT-04.3).

Stdlib-only -- no numpy/rclpy import anywhere in this file, so it runs standalone under the
system python3 (ROS Humble, py3.10) without a colcon build. The scoring rule implemented here is
normative in `docs/INTERFACES.md` §7.3 ("Scoring rule"); this module is the reference
implementation. Grid geometry (x/y/surface_z/standoffs) is always drawn from the *map* being
scored, never from the `cfg` argument's own `grid` block -- only `cfg["placement"]` (footprint,
tolerance, yaw candidates, top_k) and `cfg`'s other top-level blocks (for `verification_config`)
are read from the reachability config.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .reachability_core import axis_values

_EPS = 1e-9
_SCORE_DECIMALS = 6

SCHEMA = "crackvision.specimen_placement/1"
SOURCE = "recommend_placement (MOT-04.3)"

NOMINAL_DISCLAIMER = (
    "Nominal recommendation only; the operator places the specimen and MOT-10 measures and "
    "supersedes this file with the real, as-placed pose."
)
CAVEATS = [
    NOMINAL_DISCLAIMER,
    "The boresight is prior evidence until GEOM-07.",
    "End-of-arm geometry (tool_tip, wrist D405 camera_link and mount collision proxies) is the nominal "
    "CAD/URDF prior in config/robot/end_effector.yaml (ADR-014); re-run after GEOM-07 (tool tip) and "
    "GEOM-05 (hand-eye) replace it.",
    "The values are nominal until MOT-10.",
]


def caveats_for(map_dict: Mapping[str, Any]) -> List[str]:
    """CAVEATS plus the map-specific task frame and collision model the recommendation rests on."""
    ik_link = map_dict.get("robot", {}).get("ik_link", "?")
    grid = map_dict.get("grid", {})
    model = grid.get("surface_collision", {}).get("model", "slab")
    env = grid.get("environment")
    scene = (f"; environment objects {list(env['objects'])} from {env['scene_config']}" if env
             else "; no workcell objects (table) in the scene")
    return list(CAVEATS) + [
        f"Targets are poses of '{ik_link}' at the map's standoffs along its boresight.",
        f"Collision model: self-collision (incl. the camera proxies) + specimen proxy '{model}'{scene}.",
    ]


def _round_key(v: float) -> float:
    return round(float(v), 9)


def _build_lookup(map_dict: Mapping[str, Any]) -> Dict[Tuple[float, float, float, float], dict]:
    lookup: Dict[Tuple[float, float, float, float], dict] = {}
    for t in map_dict["targets"]:
        key = (_round_key(t["surface_z"]), _round_key(t["x"]), _round_key(t["y"]), _round_key(t["standoff_m"]))
        lookup[key] = t
    return lookup


def _map_grid(map_dict: Mapping[str, Any]) -> Tuple[dict, dict, List[float], List[float], Sequence[float], Sequence[float]]:
    grid = map_dict["grid"]["grid"]
    x_axis = grid["x_m"]
    y_axis = grid["y_m"]
    x_vals = axis_values(x_axis)
    y_vals = axis_values(y_axis)
    return x_axis, y_axis, x_vals, y_vals, grid["surface_z_m"], grid["standoffs_m"]


def _half_extents(footprint_m: Sequence[float], tolerance_m: float) -> Tuple[float, float]:
    return footprint_m[0] / 2.0 + tolerance_m, footprint_m[1] / 2.0 + tolerance_m


def _rect_corners(cx: float, cy: float, hw: float, hd: float, yaw: float) -> List[Tuple[float, float]]:
    c, s = math.cos(yaw), math.sin(yaw)
    local = ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd))
    return [(cx + lx * c - ly * s, cy + lx * s + ly * c) for lx, ly in local]


def _out_of_grid(corners: Sequence[Tuple[float, float]], x_axis: Mapping[str, float], y_axis: Mapping[str, float]) -> bool:
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    return (
        min(xs) < x_axis["min"] - _EPS or max(xs) > x_axis["max"] + _EPS
        or min(ys) < y_axis["min"] - _EPS or max(ys) > y_axis["max"] + _EPS
    )


def _inside_dilated_rect(px: float, py: float, cx: float, cy: float, hw: float, hd: float, yaw: float) -> bool:
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = px - cx, py - cy
    local_x = dx * c + dy * s
    local_y = -dx * s + dy * c
    return abs(local_x) <= hw + _EPS and abs(local_y) <= hd + _EPS


def _evaluate_candidate(
    cx: float,
    cy: float,
    surface_z: float,
    yaw: float,
    footprint_m: Sequence[float],
    tolerance_m: float,
    standoffs: Sequence[float],
    x_axis: Mapping[str, float],
    y_axis: Mapping[str, float],
    x_vals: Sequence[float],
    y_vals: Sequence[float],
    lookup: Mapping[Tuple[float, float, float, float], dict],
) -> Optional[float]:
    """Return the candidate's score, or None if infeasible (out-of-grid or any node unreachable)."""
    hw, hd = _half_extents(footprint_m, tolerance_m)
    corners = _rect_corners(cx, cy, hw, hd, yaw)
    if _out_of_grid(corners, x_axis, y_axis):
        return None

    margins: List[float] = []
    for x in x_vals:
        for y in y_vals:
            if not _inside_dilated_rect(x, y, cx, cy, hw, hd, yaw):
                continue
            for standoff in standoffs:
                entry = lookup.get((_round_key(surface_z), _round_key(x), _round_key(y), _round_key(standoff)))
                if entry is None or entry["status"] != "reachable":
                    return None
                margins.append(entry["min_joint_limit_margin_rad"])
    if not margins:
        return None
    return min(margins)


def _rank_key(candidate: Mapping[str, Any]) -> Tuple[float, float, float, float, float]:
    return (
        -round(candidate["score_min_joint_margin_rad"], _SCORE_DECIMALS),
        abs(candidate["center_xy_m"][1]),
        candidate["yaw_rad"],
        candidate["center_xy_m"][0],
        candidate["surface_z_m"],
    )


def score_candidates(map_dict: Mapping[str, Any], cfg: Mapping[str, Any]) -> List[Dict[str, Any]]:
    """Every feasible placement candidate over the map's grid, ranked best-first (§7.3)."""
    x_axis, y_axis, x_vals, y_vals, surface_z_vals, standoffs = _map_grid(map_dict)
    footprint_m = cfg["placement"]["footprint_m"]
    tolerance_m = cfg["placement"]["tolerance_m"]
    yaw_candidates = cfg["placement"]["yaw_candidates_rad"]

    lookup = _build_lookup(map_dict)

    candidates: List[Dict[str, Any]] = []
    for surface_z in surface_z_vals:
        for yaw in yaw_candidates:
            for cx in x_vals:
                for cy in y_vals:
                    score = _evaluate_candidate(
                        cx, cy, surface_z, yaw, footprint_m, tolerance_m, standoffs,
                        x_axis, y_axis, x_vals, y_vals, lookup,
                    )
                    if score is None:
                        continue
                    candidates.append({
                        "center_xy_m": [_round_key(cx), _round_key(cy)],
                        "surface_z_m": surface_z,
                        "yaw_rad": yaw,
                        "footprint_m": list(footprint_m),
                        "tolerance_m": tolerance_m,
                        "standoffs_m": list(standoffs),
                        "score_min_joint_margin_rad": score,
                    })

    candidates.sort(key=_rank_key)
    return candidates


def _max_feasible_square(map_dict: Mapping[str, Any], cfg: Mapping[str, Any]) -> float:
    x_axis, y_axis, x_vals, y_vals, surface_z_vals, standoffs = _map_grid(map_dict)
    tolerance_m = cfg["placement"]["tolerance_m"]
    footprint_m = cfg["placement"]["footprint_m"]
    step = min(x_axis["step"], y_axis["step"])
    lookup = _build_lookup(map_dict)

    s0 = min(footprint_m)
    k = 0
    while True:
        s = s0 - k * step
        if s <= _EPS:
            return 0.0
        for surface_z in surface_z_vals:
            for cx in x_vals:
                for cy in y_vals:
                    score = _evaluate_candidate(
                        cx, cy, surface_z, 0.0, (s, s), tolerance_m, standoffs,
                        x_axis, y_axis, x_vals, y_vals, lookup,
                    )
                    if score is not None:
                        return round(s, 9)
        k += 1


def recommend(
    map_dict: Mapping[str, Any],
    cfg: Mapping[str, Any],
    map_sha256: str,
    config_sha256: str,
) -> Dict[str, Any]:
    """Build a full crackvision.specimen_placement/1 dict (§7.3) from a scored map."""
    candidates = score_candidates(map_dict, cfg)
    top_k = cfg["placement"]["top_k"]
    x_axis, y_axis, _, _, _, _ = _map_grid(map_dict)
    x_step, y_step = x_axis["step"], y_axis["step"]

    result: Dict[str, Any] = {
        "schema": SCHEMA,
        "value_status": "nominal",
        "feasible": False,
        "frame": cfg["frame"],
        "placement": None,
        "alternatives": [],
        "max_feasible_square_m": None,
        "map_sha256": map_sha256,
        "config_sha256": config_sha256,
        "boresight_provenance": map_dict["boresight"]["provenance"],
        "caveats": caveats_for(map_dict),
        "source": SOURCE,
    }

    if not candidates:
        result["max_feasible_square_m"] = _max_feasible_square(map_dict, cfg)
        return result

    selected: List[Dict[str, Any]] = [candidates[0]]
    for candidate in candidates[1:]:
        if len(selected) >= top_k:
            break
        too_close = any(
            abs(candidate["center_xy_m"][0] - s["center_xy_m"][0]) <= x_step + _EPS
            and abs(candidate["center_xy_m"][1] - s["center_xy_m"][1]) <= y_step + _EPS
            for s in selected
        )
        if too_close:
            continue
        selected.append(candidate)

    result["feasible"] = True
    result["placement"] = selected[0]
    result["alternatives"] = selected[1:]
    return result


def _snap_axis(lo: float, hi: float, step: float) -> Tuple[float, float]:
    n = max(1, math.ceil((hi - lo) / step - _EPS))
    return lo, lo + n * step


def _jsonify(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: _jsonify(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonify(v) for v in value]
    return value


def verification_config(placement: Mapping[str, Any], cfg: Mapping[str, Any]) -> Dict[str, Any]:
    """A crackvision.reachability_config/1 dict re-sampling the winning placement's dilated
    footprint at half the sweep grid step, for an independent re-check of the recommendation."""
    cx, cy = placement["center_xy_m"]
    yaw = placement["yaw_rad"]
    hw, hd = _half_extents(placement["footprint_m"], placement["tolerance_m"])
    corners = _rect_corners(cx, cy, hw, hd, yaw)
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]

    x_step = cfg["grid"]["x_m"]["step"] / 2.0
    y_step = cfg["grid"]["y_m"]["step"] / 2.0
    x_lo, x_hi = _snap_axis(min(xs), max(xs), x_step)
    y_lo, y_hi = _snap_axis(min(ys), max(ys), y_step)

    sources = dict(cfg["sources"])
    sources["placement_verification"] = (
        "Emitted by recommend_placement (MOT-04.3) to independently re-check the winning "
        "placement's dilated footprint at half the sweep grid step -- points the original sweep "
        "never sampled."
    )

    out = {
        "frame": cfg["frame"],
        "group": cfg["group"],
        "ik_link": cfg["ik_link"],
        "boresight": _jsonify(cfg["boresight"]),
        "grid": {
            "x_m": {"min": x_lo, "max": x_hi, "step": x_step},
            "y_m": {"min": y_lo, "max": y_hi, "step": y_step},
            "surface_z_m": [placement["surface_z_m"]],
            "standoffs_m": list(placement["standoffs_m"]),
        },
        "orientation": _jsonify(cfg["orientation"]),
        "ik": _jsonify(cfg["ik"]),
        "surface_collision": _jsonify(cfg["surface_collision"]),
        "prefilter": {"enabled": True},
        "placement": _jsonify(cfg["placement"]),
        "value_status": "nominal",
        "sources": sources,
    }
    if cfg.get("environment"):
        out["environment"] = _jsonify(cfg["environment"])
    return out


VIEW_DISTANCE_M = 0.25
VIEW_TILT_DEG = (0.0, 15.0)


def view_verification_config(
    placement: Mapping[str, Any],
    cfg: Mapping[str, Any],
    distance_m: float = VIEW_DISTANCE_M,
    tilt_deg: Sequence[float] = VIEW_TILT_DEG,
) -> Dict[str, Any]:
    """A crackvision.reachability_config/1 dict checking that the wrist D405 can look at the placement
    (ADR-014 view phase): `camera_link` (+x = optical axis) at `distance_m` above the footprint centre,
    optical axis along the surface anti-normal or tilted up to max(tilt_deg), any roll. The specimen
    proxy margin is widened to cover the whole dilated footprint."""
    cx, cy = placement["center_xy_m"]
    fw, fd = placement["footprint_m"]
    sc = _jsonify(cfg["surface_collision"])
    sc["margin_m"] = max(fw, fd) / 2.0 + placement["tolerance_m"]
    sources = dict(cfg["sources"])
    sources["view_verification"] = (
        f"Emitted by recommend_placement: wrist-camera view check (ADR-014 view phase) at the placement "
        f"centre, viewing distance {distance_m} m (nominal: inside the D405 0.07-0.50 m ideal band, and "
        f"far enough for a {max(fw, fd)} m specimen to fit the 58 deg vertical FOV), tilt <= {max(tilt_deg)} deg."
    )
    out = {
        "frame": cfg["frame"],
        "group": cfg["group"],
        "ik_link": "camera_link",
        "boresight": {
            "axis_local": [1.0, 0.0, 0.0],
            "down_world": [0.0, 0.0, -1.0],
            "provenance": "cad_prior",
            "source": "camera_link +x is the D405 optical axis (realsense-ros); its pose on the gripper is the "
                      "nominal CAD prior in config/robot/end_effector.yaml until GEOM-05 (ADR-014)",
        },
        "grid": {
            "x_m": {"min": cx, "max": cx, "step": cfg["grid"]["x_m"]["step"]},
            "y_m": {"min": cy, "max": cy, "step": cfg["grid"]["y_m"]["step"]},
            "surface_z_m": [placement["surface_z_m"]],
            "standoffs_m": [float(distance_m)],
        },
        "orientation": {
            "roll_samples": cfg["orientation"]["roll_samples"],
            "tilt_deg": [float(t) for t in tilt_deg],
            "tilt_azimuth_samples": 4,
        },
        "ik": dict(_jsonify(cfg["ik"]), roll_search="first"),
        "surface_collision": sc,
        "prefilter": {"enabled": True},
        "placement": _jsonify(cfg["placement"]),
        "value_status": "nominal",
        "sources": sources,
    }
    if cfg.get("environment"):
        out["environment"] = _jsonify(cfg["environment"])
    return out

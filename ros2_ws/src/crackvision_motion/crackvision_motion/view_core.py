"""view_core — pure-Python §13.14 view-phase geometry (MOT-06.8): camera_link view-pose
candidates over the specimen and the nominal D405 field-of-view coverage check.

Stdlib + numpy only -- no rclpy/*_msgs import anywhere in this file (mirrors
reachability_core.py/cartesian_core.py), so it runs standalone under the system python3 without a
colcon build. Normative source: docs/INTERFACES.md §13.14 (schema `crackvision.view_plan/1`) and
docs/adr/014-end-effector-frames-and-task-phases.md §8.2/§8.3.

This module covers only the ROS-free geometry MOT-06.8 scopes:
  - `specimen_frame` -- the specimen top-face centre, outward normal and footprint corners, from
    either the scene's `specimen` object or an explicit `--center`.
  - `view_candidates` -- the `(tilt_deg, yaw_deg)` camera_link pose candidate set, in ascending
    tilt x ascending yaw order (§13.14).
  - `fov_check` -- the per-corner nominal D405 field-of-view coverage check (§13.14).

IK/reachability, the independent re-check, candidate selection and the `plan_view` CLI/plan-file
assembly (§13.14's own selection rule, §13.15) are later MOT-06 cards.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

from .cartesian_core import _rotation_from_rpy
from .reachability_core import _perp_basis, _quat_from_matrix, _rotation_matrix

_SPECIMEN_OBJECT_ID = "specimen"
_CORNER_SIGNS: Tuple[Tuple[float, float], ...] = ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0))


class ViewCoreError(ValueError):
    """Raised when the specimen centre/normal/footprint cannot be resolved (§13.14): no
    `--center` (with its own footprint/surface height) and no scene `specimen` object."""


class Pose(NamedTuple):
    position_m: Tuple[float, float, float]
    quat_xyzw: Tuple[float, float, float, float]


class SpecimenFrame(NamedTuple):
    centre_top_m: Tuple[float, float, float]
    normal_out: Tuple[float, float, float]
    # 4 top-face corners, local-frame sign order (-x,-y), (-x,+y), (+x,-y), (+x,+y) -- _CORNER_SIGNS
    corners_m: Tuple[Tuple[float, float, float], ...]


class ViewCandidate(NamedTuple):
    tilt_deg: float
    yaw_deg: float
    pose: Pose


# --------------------------------------------------------------------------------------
# specimen frame (§13.14 "Geometry")
# --------------------------------------------------------------------------------------

def specimen_frame(
    scene_cfg: Optional[Dict[str, Any]] = None,
    center: Optional[Sequence[float]] = None,
    footprint_m: Optional[Sequence[float]] = None,
    surface_z_m: Optional[float] = None,
) -> SpecimenFrame:
    """The specimen top-face centre, outward normal and 4 footprint corners (§13.14).

    `center=(x, y)` (with `footprint_m`/`surface_z_m`, both required together with it) always
    takes precedence over the scene's `specimen` object, even if one is also present: the
    specimen is then assumed axis-aligned (`normal_out = +Z`), with corners at `z = surface_z_m`.

    Without `center`, the centre/normal/footprint come from the scene's `specimen` object's own
    (possibly rotated) pose: `centre_top_m` is that box's top-face centre, `normal_out` is its
    pose's own `+Z`, and the 4 corners are the (rotation-included) top-face corners. A missing
    scene `specimen` object in this path is a `ViewCoreError`.
    """
    if center is not None:
        if footprint_m is None or surface_z_m is None:
            raise ViewCoreError("'center' requires both 'footprint_m' and 'surface_z_m'")
        cx, cy = float(center[0]), float(center[1])
        z_top = float(surface_z_m)
        fx, fy = float(footprint_m[0]), float(footprint_m[1])
        centre_top = (cx, cy, z_top)
        normal_out = (0.0, 0.0, 1.0)
        corners = tuple(
            (cx + sx * fx / 2.0, cy + sy * fy / 2.0, z_top) for sx, sy in _CORNER_SIGNS
        )
        return SpecimenFrame(centre_top, normal_out, corners)

    objects = (scene_cfg or {}).get("objects", [])
    specimen = next((o for o in objects if o.get("id") == _SPECIMEN_OBJECT_ID), None)
    if specimen is None:
        raise ViewCoreError(
            "no 'center' was given and the scene has no 'specimen' object (§13.14): cannot "
            "resolve the specimen centre/normal/footprint"
        )

    position = np.asarray(specimen["pose"]["position_m"], dtype=float)
    rotation = _rotation_from_rpy(specimen["pose"]["rpy_rad"])
    half = np.asarray(specimen["dimensions_m"], dtype=float) / 2.0

    centre_top = tuple(float(v) for v in position + rotation @ np.array([0.0, 0.0, half[2]]))
    normal_out = tuple(float(v) for v in rotation @ np.array([0.0, 0.0, 1.0]))
    corners = tuple(
        tuple(float(v) for v in position + rotation @ np.array([sx * half[0], sy * half[1], half[2]]))
        for sx, sy in _CORNER_SIGNS
    )
    return SpecimenFrame(centre_top, normal_out, corners)


# --------------------------------------------------------------------------------------
# view-pose candidates (§13.14 "Geometry")
# --------------------------------------------------------------------------------------

def _axes_from_quat(q: Sequence[float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`(+X, +Y, +Z)` world-frame unit axes for a xyzw quaternion (inverse of `view_candidates`'s
    own column-stacked rotation-matrix construction)."""
    x, y, z, w = (float(v) for v in q)
    r = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ])
    return r[:, 0], r[:, 1], r[:, 2]


def view_candidates(
    cfg: Dict[str, Any], centre_top_m: Sequence[float], normal_out: Sequence[float],
) -> List[ViewCandidate]:
    """The `(tilt_deg, yaw_deg)` camera_link pose candidate set (§13.14), in ascending
    `cfg['view']['tilt_candidates_deg']` x ascending `cfg['view']['yaw_candidates_deg']` order --
    this is both the enumeration order and the (later) selection tie-break rule.

    Camera position: `centre_top_m - view_distance_m * d_world`, where `d_world` is
    `-normal_out` rotated `tilt_deg` about the azimuth axis `a = cos(yaw) e1 + sin(yaw) e2`
    (right-hand rule; `e1, e2` span the plane perpendicular to `normal_out`). This keeps the
    camera on the sphere of radius `view_distance_m` centred on `centre_top_m`, so the optical
    axis (`camera_link +X := d_world`) passes exactly through the centre at every tilt -- the
    same geometry `recommend_placement --emit-view-config` (MOT-04.6) uses via
    `reachability_core.orientations`/`target_position`. `+Z := a`, `+Y := +Z x +X` (§13.14).
    """
    view_distance_m = cfg["view"]["view_distance_m"]
    n = np.asarray(normal_out, dtype=float)
    n = n / np.linalg.norm(n)
    down_into = -n
    e1, e2 = _perp_basis(down_into)
    centre = np.asarray(centre_top_m, dtype=float)

    result: List[ViewCandidate] = []
    for tilt_deg in sorted(float(t) for t in cfg["view"]["tilt_candidates_deg"]):
        for yaw_deg in sorted(float(y) for y in cfg["view"]["yaw_candidates_deg"]):
            yaw_rad = math.radians(yaw_deg)
            a = math.cos(yaw_rad) * e1 + math.sin(yaw_rad) * e2
            if tilt_deg == 0.0:
                d_world = down_into
            else:
                d_world = _rotation_matrix(a, math.radians(tilt_deg)) @ down_into
            position = centre - view_distance_m * d_world

            x_axis = d_world
            z_axis = a
            y_axis = np.cross(z_axis, x_axis)
            rotation = np.column_stack((x_axis, y_axis, z_axis))
            quat = _quat_from_matrix(rotation)

            pose = Pose(tuple(float(v) for v in position), tuple(float(v) for v in quat))
            result.append(ViewCandidate(tilt_deg, yaw_deg, pose))
    return result


# --------------------------------------------------------------------------------------
# FOV coverage check (§13.14 "FOV check")
# --------------------------------------------------------------------------------------

def fov_check(
    pose: Pose, corners_m: Sequence[Tuple[float, float, float]], cfg: Dict[str, Any],
) -> Tuple[bool, Dict[str, Any]]:
    """Per-corner nominal D405 field-of-view coverage check (§13.14): projects every
    `corners_m` point into `pose`'s camera_link frame and compares the horizontal/vertical angle
    against the configured half-FOV minus margin. Passes (`ok`) iff every corner clears both
    bounds. `details` reports every corner's own angle and signed margin (positive = pass)."""
    position = np.asarray(pose.position_m, dtype=float)
    x_axis, y_axis, z_axis = _axes_from_quat(pose.quat_xyzw)

    half_h_deg = cfg["view"]["fov_deg"]["horizontal"] / 2.0 - cfg["view"]["fov_margin_deg"]
    half_v_deg = cfg["view"]["fov_deg"]["vertical"] / 2.0 - cfg["view"]["fov_margin_deg"]

    horizontal_angles_deg: List[float] = []
    vertical_angles_deg: List[float] = []
    horizontal_margins_deg: List[float] = []
    vertical_margins_deg: List[float] = []
    for corner in corners_m:
        v = np.asarray(corner, dtype=float) - position
        h_angle_deg = math.degrees(math.atan2(float(np.dot(v, y_axis)), float(np.dot(v, x_axis))))
        v_angle_deg = math.degrees(math.atan2(float(np.dot(v, z_axis)), float(np.dot(v, x_axis))))
        horizontal_angles_deg.append(h_angle_deg)
        vertical_angles_deg.append(v_angle_deg)
        horizontal_margins_deg.append(half_h_deg - abs(h_angle_deg))
        vertical_margins_deg.append(half_v_deg - abs(v_angle_deg))

    ok = all(m >= 0.0 for m in horizontal_margins_deg) and all(m >= 0.0 for m in vertical_margins_deg)
    details = {
        "horizontal_angles_deg": horizontal_angles_deg,
        "vertical_angles_deg": vertical_angles_deg,
        "horizontal_margins_deg": horizontal_margins_deg,
        "vertical_margins_deg": vertical_margins_deg,
    }
    return ok, details

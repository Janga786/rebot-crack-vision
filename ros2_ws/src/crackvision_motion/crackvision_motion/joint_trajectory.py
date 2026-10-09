"""Pure-Python `crackvision.joint_trajectory/1` file library (MOT-05.2).

Stdlib + PyYAML only -- no rclpy or *_msgs import anywhere in this file, so it runs standalone
under the system python3 (ROS Humble, py3.10) without a colcon build, exactly like
`reachability_core.py` / `scene_core.py`. Schema is normative in `docs/INTERFACES.md` §11.2;
this module is the reference implementation of its validation, the §11.6 position-margin
policy, limit checking (§11.7 `G-LIMITS`) and the §11.7/§11.11 time-scaling and densification
helpers. Public names (`TrajectoryError`, `JOINT_NAMES`, `load_trajectory`, `file_sha256`,
`load_limits`, `Violation`, `check_limits`, `scale_time`, `densify`, `max_abs_velocity`) are
normative for MOT-05.3/.4.

`reachability_map.limits_from_yaml` already parses `config/robot/b601_dm_limits.yaml`, but only
into `{joint: (lower, upper)}` -- it has no velocity/acceleration, which §11 limit checking needs
for every point. Rather than partially reuse it and re-open the same file for the rest, this
module reads `b601_dm_limits.yaml` once via its own `load_limits`, which returns the full
lower/upper/velocity/acceleration shape; nothing here duplicates `limits_from_yaml`'s margin or
validation logic.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

import yaml

SCHEMA = "crackvision.joint_trajectory/1"

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))

_TOP_LEVEL_KEYS = frozenset({
    "schema", "created_utc", "producer", "purpose", "planning_frame", "joint_names", "points",
    "limits_file", "end_effector_config_sha256", "scene_config_sha256", "source",
    "notes",  # non-normative, human documentation only; ignored by every consumer
})
_REQUIRED_TOP_LEVEL_KEYS = (
    "schema", "created_utc", "producer", "purpose", "planning_frame", "joint_names", "points",
    "limits_file", "end_effector_config_sha256", "scene_config_sha256", "source",
)
_POINT_KEYS = frozenset({"t_s", "positions", "velocities", "accelerations"})
_PURPOSE_VALUES = frozenset({"crack_task", "test"})
_SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")

_N_JOINTS = len(JOINT_NAMES)


class TrajectoryError(ValueError):
    """Raised for any structurally or numerically invalid joint_trajectory file."""


# --------------------------------------------------------------------------------------
# load_trajectory
# --------------------------------------------------------------------------------------

def _check_sha_or_null(value: object, ctx: str) -> Optional[str]:
    if value is None:
        return None
    if not isinstance(value, str) or not _SHA256_RE.match(value):
        raise TrajectoryError(f"'{ctx}' must be a 64-hex-char sha256 digest or null, got {value!r}")
    return value.lower()


def _validate_vector(v: object, ctx: str) -> List[float]:
    if not isinstance(v, list) or len(v) != _N_JOINTS:
        raise TrajectoryError(f"'{ctx}' must be a list of length {_N_JOINTS}, got {v!r}")
    out: List[float] = []
    for item in v:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise TrajectoryError(f"'{ctx}' entries must be numeric, got {item!r}")
        fv = float(item)
        if not math.isfinite(fv):
            raise TrajectoryError(f"'{ctx}' entries must be finite, got {item!r}")
        out.append(fv)
    return out


def load_trajectory(path: Union[str, Path]) -> dict:
    """Load and validate a `crackvision.joint_trajectory/1` file. Raises TrajectoryError."""
    raw_text = Path(path).read_text()
    try:
        raw = json.loads(raw_text)
    except json.JSONDecodeError as exc:
        raise TrajectoryError(f"invalid JSON in {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise TrajectoryError("trajectory root must be a JSON object")

    unknown = set(raw) - _TOP_LEVEL_KEYS
    if unknown:
        raise TrajectoryError(f"unknown top-level key(s): {sorted(unknown)}")
    for key in _REQUIRED_TOP_LEVEL_KEYS:
        if key not in raw:
            raise TrajectoryError(f"missing required top-level key '{key}'")

    if raw["schema"] != SCHEMA:
        raise TrajectoryError(f"'schema' must be {SCHEMA!r}, got {raw['schema']!r}")
    if not isinstance(raw["created_utc"], str) or not raw["created_utc"]:
        raise TrajectoryError("'created_utc' must be a non-empty string")
    if not isinstance(raw["producer"], str) or not raw["producer"]:
        raise TrajectoryError("'producer' must be a non-empty string")
    if raw["purpose"] not in _PURPOSE_VALUES:
        raise TrajectoryError(f"'purpose' must be one of {sorted(_PURPOSE_VALUES)}, got {raw['purpose']!r}")
    if raw["planning_frame"] != "base_link":
        raise TrajectoryError(f"'planning_frame' must be 'base_link', got {raw['planning_frame']!r}")

    joint_names = raw["joint_names"]
    if not isinstance(joint_names, list) or joint_names != list(JOINT_NAMES):
        raise TrajectoryError(f"'joint_names' must be exactly {list(JOINT_NAMES)} in order, got {joint_names!r}")

    points_raw = raw["points"]
    if not isinstance(points_raw, list) or len(points_raw) < 2:
        n = len(points_raw) if isinstance(points_raw, list) else points_raw
        raise TrajectoryError(f"'points' must be a list with at least 2 entries, got {n!r}")

    points: List[Dict[str, Any]] = []
    prev_t: Optional[float] = None
    for i, p in enumerate(points_raw):
        ctx = f"points[{i}]"
        if not isinstance(p, dict):
            raise TrajectoryError(f"'{ctx}' must be an object, got {p!r}")
        unknown_p = set(p) - _POINT_KEYS
        if unknown_p:
            raise TrajectoryError(f"unknown key(s) in '{ctx}': {sorted(unknown_p)}")
        if "t_s" not in p or "positions" not in p:
            raise TrajectoryError(f"'{ctx}' is missing required key 't_s' and/or 'positions'")

        t_s_raw = p["t_s"]
        if isinstance(t_s_raw, bool) or not isinstance(t_s_raw, (int, float)) or not math.isfinite(float(t_s_raw)):
            raise TrajectoryError(f"'{ctx}.t_s' must be a finite number, got {t_s_raw!r}")
        t_s = float(t_s_raw)
        if i == 0 and t_s != 0.0:
            raise TrajectoryError(f"first point's t_s must be 0, got {t_s!r}")
        if prev_t is not None and t_s <= prev_t:
            raise TrajectoryError(f"'{ctx}.t_s' must be strictly increasing: {prev_t!r} -> {t_s!r}")
        prev_t = t_s

        point: Dict[str, Any] = {
            "t_s": t_s,
            "positions": _validate_vector(p["positions"], f"{ctx}.positions"),
        }
        if "velocities" in p:
            point["velocities"] = _validate_vector(p["velocities"], f"{ctx}.velocities")
        if "accelerations" in p:
            point["accelerations"] = _validate_vector(p["accelerations"], f"{ctx}.accelerations")
        points.append(point)

    limits_file_raw = raw["limits_file"]
    if not isinstance(limits_file_raw, dict) or set(limits_file_raw) != {"path", "sha256"}:
        raise TrajectoryError("'limits_file' must be an object with exactly 'path' and 'sha256'")
    if not isinstance(limits_file_raw["path"], str) or not limits_file_raw["path"]:
        raise TrajectoryError("'limits_file.path' must be a non-empty string")
    limits_file = {
        "path": limits_file_raw["path"],
        "sha256": _check_sha_or_null(limits_file_raw["sha256"], "limits_file.sha256"),
    }

    end_effector_sha = _check_sha_or_null(raw["end_effector_config_sha256"], "end_effector_config_sha256")
    scene_sha = _check_sha_or_null(raw["scene_config_sha256"], "scene_config_sha256")

    source_raw = raw["source"]
    if not isinstance(source_raw, dict) or set(source_raw) != {"paths3d"}:
        raise TrajectoryError("'source' must be an object with exactly key 'paths3d'")
    paths3d_raw = source_raw["paths3d"]
    if paths3d_raw is None:
        source: Dict[str, Any] = {"paths3d": None}
    else:
        if not isinstance(paths3d_raw, dict) or set(paths3d_raw) != {"path", "sha256", "execution_eligible"}:
            raise TrajectoryError(
                "'source.paths3d' must be null or an object with 'path', 'sha256', 'execution_eligible'"
            )
        if not isinstance(paths3d_raw["path"], str) or not paths3d_raw["path"]:
            raise TrajectoryError("'source.paths3d.path' must be a non-empty string")
        sha = paths3d_raw["sha256"]
        if not isinstance(sha, str) or not _SHA256_RE.match(sha):
            raise TrajectoryError(f"'source.paths3d.sha256' must be a 64-hex-char sha256 digest, got {sha!r}")
        eligible = paths3d_raw["execution_eligible"]
        if eligible is not None and not isinstance(eligible, bool):
            raise TrajectoryError(f"'source.paths3d.execution_eligible' must be a bool or null, got {eligible!r}")
        source = {
            "paths3d": {"path": paths3d_raw["path"], "sha256": sha.lower(), "execution_eligible": eligible}
        }

    result: Dict[str, Any] = {
        "schema": raw["schema"],
        "created_utc": raw["created_utc"],
        "producer": raw["producer"],
        "purpose": raw["purpose"],
        "planning_frame": raw["planning_frame"],
        "joint_names": list(JOINT_NAMES),
        "points": points,
        "limits_file": limits_file,
        "end_effector_config_sha256": end_effector_sha,
        "scene_config_sha256": scene_sha,
        "source": source,
        "_path": str(Path(path)),
    }
    if "notes" in raw:
        result["notes"] = raw["notes"]
    return result


def file_sha256(path: Union[str, Path]) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# --------------------------------------------------------------------------------------
# limits
# --------------------------------------------------------------------------------------

def load_limits(path: Union[str, Path]) -> Dict[str, Dict[str, float]]:
    """Parse `b601_dm_limits.yaml` into `{joint: {lower, upper, velocity, acceleration}}`."""
    raw = yaml.safe_load(Path(path).read_text())
    joints = raw["joints"]
    result: Dict[str, Dict[str, float]] = {}
    for name in JOINT_NAMES:
        spec = joints[name]
        result[name] = {
            "lower": float(spec["lower"]),
            "upper": float(spec["upper"]),
            "velocity": float(spec["velocity"]),
            "acceleration": float(spec["acceleration"]),
        }
    return result


# --------------------------------------------------------------------------------------
# limit checking (§11.6 position-margin policy + §11.7 G-LIMITS)
# --------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Violation:
    index: int
    joint: str
    kind: str  # "position" | "position_margin" | "velocity" | "acceleration"
    value: float
    bound: float


def _margin_violations_for_joint(
    joint: str,
    positions: List[float],
    t_s: List[float],
    explicit_vels: List[Optional[float]],
    lower: float,
    upper: float,
    margin: float,
    eps: float,
) -> List[Violation]:
    """§11.6: toward-bound speed must not increase at a near-limit point.

    `seg_v[i]` is the forward finite-difference velocity of segment i -> i+1. For a near-limit
    interior point p (1 <= p <= n-2), the segment *arriving* at p is seg_v[p-1] and the segment
    *leaving* p is seg_v[p]; a violation is arriving speeding-toward-bound (> 0) and leaving even
    faster. The first point has no arriving segment and the last has no leaving segment, so
    neither is ever flagged by the finite-difference pass -- matching §11.6's home-approach and
    counter-example derivations exactly. The explicit-velocity pass instead compares each point's
    own velocity directly against the next point's (both exist per point, not per segment), plus
    a separate last-point-only rule (a last point is a violation only if it carries an explicit
    velocity still heading into the bound).
    """
    n = len(positions)
    seg_v = [(positions[i + 1] - positions[i]) / (t_s[i + 1] - t_s[i]) for i in range(n - 1)]
    violations: List[Violation] = []

    for bound_name, sign in (("upper", 1.0), ("lower", -1.0)):
        if bound_name == "upper":
            near = [(upper - q) <= margin for q in positions]
        else:
            near = [(q - lower) <= margin for q in positions]

        we = [(sign * v if v is not None else None) for v in explicit_vels]
        for i in range(n - 1):
            if (
                near[i] and we[i] is not None and we[i] > eps
                and we[i + 1] is not None and we[i + 1] > we[i] + eps
            ):
                violations.append(Violation(i, joint, "position_margin", we[i + 1], we[i]))
        last = n - 1
        if near[last] and we[last] is not None and we[last] > eps:
            violations.append(Violation(last, joint, "position_margin", we[last], 0.0))

        wf = [sign * v for v in seg_v]
        for p in range(1, n - 1):
            w_in, w_out = wf[p - 1], wf[p]
            if near[p] and w_in > eps and w_out > w_in + eps:
                violations.append(Violation(p, joint, "position_margin", w_out, w_in))

    return violations


def _vel_acc_violations_for_joint(
    joint: str,
    positions: List[float],
    t_s: List[float],
    explicit_vels: List[Optional[float]],
    explicit_accs: List[Optional[float]],
    velocity_limit: float,
    acceleration_limit: float,
    eps: float,
) -> List[Violation]:
    n = len(positions)
    violations: List[Violation] = []

    for i, v in enumerate(explicit_vels):
        if v is not None and abs(v) > velocity_limit + eps:
            violations.append(Violation(i, joint, "velocity", v, velocity_limit))

    seg_v = [(positions[i + 1] - positions[i]) / (t_s[i + 1] - t_s[i]) for i in range(n - 1)]
    for i, v in enumerate(seg_v):
        if abs(v) > velocity_limit + eps:
            violations.append(Violation(i, joint, "velocity", v, velocity_limit))

    for i, a in enumerate(explicit_accs):
        if a is not None and abs(a) > acceleration_limit + eps:
            violations.append(Violation(i, joint, "acceleration", a, acceleration_limit))

    for i in range(1, n - 1):
        dt = (t_s[i + 1] - t_s[i - 1]) / 2.0
        a = (seg_v[i] - seg_v[i - 1]) / dt
        if abs(a) > acceleration_limit + eps:
            violations.append(Violation(i, joint, "acceleration", a, acceleration_limit))

    return violations


def check_limits(
    traj: dict,
    limits: Dict[str, Dict[str, float]],
    *,
    position_margin_rad: float,
    start_tolerance_rad: float = 1e-9,
) -> List[Violation]:
    """Every §11.7 `G-LIMITS` violation in `traj`, never stopping at the first.

    `start_tolerance_rad` is floating-point slack for the otherwise-strict comparisons (the
    inclusive position bound, and the margin rule's strict "speeding up" ordering); it defaults
    to a tiny epsilon rather than `execution.yaml`'s 0.02 rad start-state tolerance, since widening
    the hard position bound by a visible amount would contradict §11.6 ("never narrowed").
    """
    points = traj["points"]
    t_s = [p["t_s"] for p in points]
    violations: List[Violation] = []

    for jidx, jname in enumerate(JOINT_NAMES):
        spec = limits[jname]
        positions = [p["positions"][jidx] for p in points]
        explicit_vels = [p["velocities"][jidx] if p.get("velocities") is not None else None for p in points]
        explicit_accs = [p["accelerations"][jidx] if p.get("accelerations") is not None else None for p in points]

        for i, q in enumerate(positions):
            if q < spec["lower"] - start_tolerance_rad:
                violations.append(Violation(i, jname, "position", q, spec["lower"]))
            elif q > spec["upper"] + start_tolerance_rad:
                violations.append(Violation(i, jname, "position", q, spec["upper"]))

        violations.extend(_vel_acc_violations_for_joint(
            jname, positions, t_s, explicit_vels, explicit_accs,
            spec["velocity"], spec["acceleration"], start_tolerance_rad,
        ))
        violations.extend(_margin_violations_for_joint(
            jname, positions, t_s, explicit_vels, spec["lower"], spec["upper"],
            position_margin_rad, start_tolerance_rad,
        ))

    return violations


# --------------------------------------------------------------------------------------
# §11.7 G-SPEED time scaling + §11.11 densification
# --------------------------------------------------------------------------------------

def scale_time(traj: dict, speed_scale: float) -> dict:
    """New dict with uniform retiming t/s, v*s, a*s^2; never mutates `traj`."""
    if isinstance(speed_scale, bool) or not isinstance(speed_scale, (int, float)) or not (0.0 < speed_scale <= 1.0):
        raise TrajectoryError(f"speed_scale must be in (0, 1], got {speed_scale!r}")
    speed_scale = float(speed_scale)

    new_points = []
    for p in traj["points"]:
        new_p: Dict[str, Any] = {
            "t_s": p["t_s"] / speed_scale,
            "positions": list(p["positions"]),
        }
        if p.get("velocities") is not None:
            new_p["velocities"] = [v * speed_scale for v in p["velocities"]]
        if p.get("accelerations") is not None:
            new_p["accelerations"] = [a * speed_scale * speed_scale for a in p["accelerations"]]
        new_points.append(new_p)

    new_traj = dict(traj)
    new_traj["points"] = new_points
    return new_traj


def densify(traj: dict, max_step_rad: float) -> List[List[float]]:
    """Linearly interpolated configs so no joint step exceeds `max_step_rad`; keeps originals."""
    if max_step_rad <= 0:
        raise TrajectoryError(f"max_step_rad must be > 0, got {max_step_rad!r}")
    points = traj["points"]
    configs: List[List[float]] = [list(points[0]["positions"])]
    for prev, cur in zip(points, points[1:]):
        q0, q1 = prev["positions"], cur["positions"]
        step = max(abs(b - a) for a, b in zip(q0, q1))
        n_segments = max(1, math.ceil(step / max_step_rad))
        for k in range(1, n_segments + 1):
            frac = k / n_segments
            configs.append([a + frac * (b - a) for a, b in zip(q0, q1)])
    return configs


def max_abs_velocity(traj: dict) -> List[float]:
    """Per-joint max |finite-difference velocity| over the whole trajectory."""
    points = traj["points"]
    result = []
    for jidx in range(_N_JOINTS):
        vmax = 0.0
        for i in range(len(points) - 1):
            dt = points[i + 1]["t_s"] - points[i]["t_s"]
            v = (points[i + 1]["positions"][jidx] - points[i]["positions"][jidx]) / dt
            vmax = max(vmax, abs(v))
        result.append(vmax)
    return result

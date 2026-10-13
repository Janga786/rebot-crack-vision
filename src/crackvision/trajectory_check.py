"""crackvision.trajectory_check — independent `crackvision.trajectory_check/1` checker
(docs/INTERFACES.md §16.5, ADR-019).

A separate implementation from `crackvision_motion`'s `retime_plan`/`joint_trajectory`/
`retime_core`: built from the §11.2 (`joint_trajectory`), §11.6 (position-margin policy) and
§13.11 (`cartesian_plan`) contract text alone, sharing no code with `crackvision_motion`. Runs
under the conda interpreter (`./env.sh python -m crackvision.trajectory_check`), never imports
`crackvision_motion` or `rclpy`, and never reads `retime_plan`'s own report except where C-REPORT
explicitly cross-checks it.

MOT-07.6 implemented C-SCHEMA, C-PROVENANCE, C-LIMITS, C-MARGIN, C-REST and C-SAMPLING. This part
(MOT-07.7) adds C-PATH, C-TOOLSPEED and C-REPORT: path fidelity against the plan polyline, constant
tool-tip speed on trace legs (forward kinematics from `crackvision.kinematics`, never
`crackvision_motion.tool_fk`) and cross-checking of the `retime_report` sidecar's own claims.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.kinematics import EndEffector, KinematicChain, KinematicsError, load_chain, load_end_effector
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)

TOOL = "trajectory_check"
SCHEMA = "crackvision.trajectory_check/1"
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]

_FLOAT_TOL = 1e-9

# §16.5 C-PATH tolerance (perpendicular distance to the nearest plan-polyline segment).
_PATH_TOL_RAD = 1e-6

# Mirrors `crackvision_motion.cartesian_plan.CONTINUITY_TOL_RAD`, not importable here by design
# (this checker never imports `crackvision_motion`); kept in sync by hand against §16.2 Step 1.
_CONTINUITY_TOL_RAD = 1e-6

# §16.5 C-REPORT tolerances.
_REPORT_DURATION_TOL_S = 1e-6
_REPORT_SPEED_TOL_M_S = 1e-3

# G-TOOLSPEED's vertex-dip amendment (MOT-07.11, ADR-019 addendum): the fixed joint-space turn angle
# above which an interior plan vertex counts as a real corner. Mirrors
# `crackvision_motion.retime._VERTEX_DIR_EPS_RAD` by hand (this checker never imports
# `crackvision_motion`), kept in sync against §16.2/§16.5's own text.
_VERTEX_DIR_EPS_RAD = 1e-9


class UsageError(Exception):
    """A CLI/config input could not even be parsed or opened (§0.2 exit 2)."""


def file_sha256(path: Path | str) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _iso_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _load_json(path: Path) -> Any:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise UsageError(f"cannot open {path}: {exc}") from exc
    try:
        return json.loads(text)
    except json.JSONDecodeError as exc:
        raise UsageError(f"malformed JSON in {path}: {exc}") from exc


def _load_yaml(path: Path) -> dict:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise UsageError(f"cannot open {path}: {exc}") from exc
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise UsageError(f"malformed YAML in {path}: {exc}") from exc
    if not isinstance(doc, dict):
        raise UsageError(f"{path} must contain a mapping at the top level")
    return doc


def _is_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _is_number_list(value: Any, length: int) -> bool:
    return isinstance(value, list) and len(value) == length and all(_is_number(v) for v in value)


def _is_sha256_hex(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value.lower())


def _view_plan_source_errors(view_plan: Any) -> list[str]:
    """§17.1: `source.view_plan` for a `view_move` trajectory must be exactly
    `{"path": non-empty str, "sha256": 64-hex str}` (no `execution_eligible` or other key)."""
    if not isinstance(view_plan, dict):
        return ["trajectory.source.view_plan must be {path: str, sha256: 64-hex str}"]
    errors: list[str] = []
    extra = set(view_plan) - {"path", "sha256"}
    if extra:
        errors.append(f"trajectory.source.view_plan has unexpected key(s) {sorted(extra)!r}")
    if not isinstance(view_plan.get("path"), str) or not view_plan.get("path"):
        errors.append("trajectory.source.view_plan.path must be a non-empty string")
    if not _is_sha256_hex(view_plan.get("sha256")):
        errors.append("trajectory.source.view_plan.sha256 must be a 64-hex string")
    return errors


# ---------------------------------------------------------------------------
# C-SCHEMA (§11.2 / §13.11.1)
# ---------------------------------------------------------------------------


def validate_trajectory_schema(doc: Any) -> list[str]:
    """§11.2 `crackvision.joint_trajectory/1`. Returns a list of error strings (empty = valid)."""
    if not isinstance(doc, dict):
        return ["trajectory is not a JSON object"]

    errors: list[str] = []

    if doc.get("schema") != "crackvision.joint_trajectory/1":
        errors.append(
            f"trajectory.schema is {doc.get('schema')!r}, expected 'crackvision.joint_trajectory/1'"
        )
    for key in ("created_utc", "producer"):
        if not isinstance(doc.get(key), str) or not doc.get(key):
            errors.append(f"trajectory.{key} must be a non-empty string")
    purpose = doc.get("purpose")
    if purpose not in ("crack_task", "test", "view_move"):
        errors.append(
            f"trajectory.purpose is {purpose!r}, expected 'crack_task', 'test' or 'view_move'"
        )
    if doc.get("planning_frame") != "base_link":
        errors.append(f"trajectory.planning_frame is {doc.get('planning_frame')!r}, expected 'base_link'")
    if doc.get("joint_names") != JOINT_NAMES:
        errors.append(f"trajectory.joint_names is {doc.get('joint_names')!r}, expected {JOINT_NAMES!r}")

    points = doc.get("points")
    if not isinstance(points, list) or not points:
        errors.append("trajectory.points must be a non-empty list")
    else:
        prev_t: float | None = None
        for idx, pt in enumerate(points):
            if not isinstance(pt, dict):
                errors.append(f"trajectory.points[{idx}] is not an object")
                continue
            t_s = pt.get("t_s")
            if not _is_number(t_s):
                errors.append(f"trajectory.points[{idx}].t_s must be numeric")
            else:
                t_s = float(t_s)
                if idx == 0 and abs(t_s) > _FLOAT_TOL:
                    errors.append(f"trajectory.points[0].t_s must be 0, got {t_s}")
                if prev_t is not None and not (t_s > prev_t):
                    errors.append(
                        f"trajectory.points[{idx}].t_s ({t_s}) is not strictly greater than "
                        f"points[{idx - 1}].t_s ({prev_t})"
                    )
                prev_t = t_s
            if not _is_number_list(pt.get("positions"), 6):
                errors.append(f"trajectory.points[{idx}].positions must be a list of 6 numbers")
            for optional_key in ("velocities", "accelerations"):
                if pt.get(optional_key) is not None and not _is_number_list(pt[optional_key], 6):
                    errors.append(
                        f"trajectory.points[{idx}].{optional_key} must be a list of 6 numbers when present"
                    )

    limits_file = doc.get("limits_file")
    if (
        not isinstance(limits_file, dict)
        or not isinstance(limits_file.get("path"), str)
        or not (limits_file.get("sha256") is None or isinstance(limits_file.get("sha256"), str))
    ):
        errors.append("trajectory.limits_file must be {path: str, sha256: str|null}")

    for key in ("end_effector_config_sha256", "scene_config_sha256"):
        value = doc.get(key)
        if value is not None and not isinstance(value, str):
            errors.append(f"trajectory.{key} must be a string or null")

    source = doc.get("source")
    if not isinstance(source, dict):
        errors.append("trajectory.source must be an object")
    elif purpose == "view_move":
        if "paths3d" not in source or source.get("paths3d") is not None:
            errors.append(
                "trajectory.source.paths3d must be null for purpose 'view_move', got "
                f"{source.get('paths3d')!r}"
            )
        extra = set(source) - {"paths3d", "view_plan"}
        if extra:
            errors.append(f"trajectory.source has unexpected key(s) {sorted(extra)!r} for purpose 'view_move'")
        if "view_plan" not in source:
            errors.append("trajectory.source.view_plan must be present for purpose 'view_move'")
        else:
            errors.extend(_view_plan_source_errors(source.get("view_plan")))
    else:
        if "view_plan" in source:
            errors.append(
                f"trajectory.source.view_plan must not be present when purpose is {purpose!r}"
            )
        if "paths3d" not in source:
            errors.append("trajectory.source.paths3d must be present (object or null)")
        else:
            p3d = source["paths3d"]
            if p3d is not None and (
                not isinstance(p3d, dict)
                or not isinstance(p3d.get("path"), str)
                or not isinstance(p3d.get("sha256"), str)
                or not (p3d.get("execution_eligible") is None or isinstance(p3d.get("execution_eligible"), bool))
            ):
                errors.append(
                    "trajectory.source.paths3d must be {path: str, sha256: str, execution_eligible: bool|null} or null"
                )

    return errors


def validate_plan_schema(doc: Any) -> list[str]:
    """§17.4: `--plan` is dispatched on its own `schema` field between
    `crackvision.cartesian_plan/1` (§13.11.1) and `crackvision.view_plan/1` (§13.14.1,
    minimal structure). Returns a list of error strings (empty = valid)."""
    if isinstance(doc, dict) and doc.get("schema") == "crackvision.view_plan/1":
        return validate_view_plan_schema(doc)
    return validate_cartesian_plan_schema(doc)


def validate_view_plan_schema(doc: Any) -> list[str]:
    """§17.4's minimal `crackvision.view_plan/1` structure sufficient for C-SCHEMA to pass on the
    plan side. Returns a list of error strings (empty = valid)."""
    if not isinstance(doc, dict):
        return ["plan is not a JSON object"]

    errors: list[str] = []
    if doc.get("schema") != "crackvision.view_plan/1":
        errors.append(f"plan.schema is {doc.get('schema')!r}, expected 'crackvision.view_plan/1'")

    sources = doc.get("sources")
    if not isinstance(sources, dict):
        errors.append("plan.sources must be an object")
    else:
        for name in ("scene", "end_effector", "limits", "config"):
            block = sources.get(name)
            if not isinstance(block, dict) or not isinstance(block.get("path"), str) or not isinstance(
                block.get("sha256"), str
            ):
                errors.append(f"plan.sources.{name} must be {{path: str, sha256: str}}")

    chosen = doc.get("chosen")
    if not isinstance(chosen, dict):
        errors.append("plan.chosen must be a non-null object")

    transit_leg = doc.get("transit_leg")
    if not isinstance(transit_leg, dict):
        errors.append("plan.transit_leg must be a non-null object")
    else:
        if transit_leg.get("kind") != "transit":
            errors.append(f"plan.transit_leg.kind is {transit_leg.get('kind')!r}, expected 'transit'")
        positions = transit_leg.get("positions")
        if not isinstance(positions, list) or not positions or not all(
            _is_number_list(p, 6) for p in positions
        ):
            errors.append(
                "plan.transit_leg.positions must be a non-empty list of 6-element lists of finite numbers"
            )

    return errors


def validate_cartesian_plan_schema(doc: Any) -> list[str]:
    """§13.11.1 `crackvision.cartesian_plan/1`. Returns a list of error strings (empty = valid)."""
    if not isinstance(doc, dict):
        return ["plan is not a JSON object"]

    errors: list[str] = []

    if doc.get("schema") != "crackvision.cartesian_plan/1":
        errors.append(f"plan.schema is {doc.get('schema')!r}, expected 'crackvision.cartesian_plan/1'")
    for key in ("case_id", "created_utc", "producer"):
        if not isinstance(doc.get(key), str) or not doc.get(key):
            errors.append(f"plan.{key} must be a non-empty string")
    if doc.get("planning_frame") != "base_link":
        errors.append(f"plan.planning_frame is {doc.get('planning_frame')!r}, expected 'base_link'")
    if doc.get("joint_names") != JOINT_NAMES:
        errors.append(f"plan.joint_names is {doc.get('joint_names')!r}, expected {JOINT_NAMES!r}")

    sources = doc.get("sources")
    if not isinstance(sources, dict):
        errors.append("plan.sources must be an object")
    else:
        p3d = sources.get("paths3d")
        if not isinstance(p3d, dict) or not isinstance(p3d.get("path"), str) or not isinstance(
            p3d.get("sha256"), str
        ):
            errors.append("plan.sources.paths3d must be {path: str, sha256: str, execution_eligible: bool|null}")
        for name in ("scene", "end_effector", "limits", "config"):
            block = sources.get(name)
            if not isinstance(block, dict) or not isinstance(block.get("path"), str) or not isinstance(
                block.get("sha256"), str
            ):
                errors.append(f"plan.sources.{name} must be {{path: str, sha256: str}}")

    for key in ("parameters", "start_state", "end_state", "coverage"):
        if not isinstance(doc.get(key), dict):
            errors.append(f"plan.{key} must be an object")

    legs = doc.get("legs")
    if not isinstance(legs, list) or not legs:
        errors.append("plan.legs must be a non-empty list")
    else:
        for idx, leg in enumerate(legs):
            if not isinstance(leg, dict):
                errors.append(f"plan.legs[{idx}] is not an object")
                continue
            if not isinstance(leg.get("leg_index"), int):
                errors.append(f"plan.legs[{idx}].leg_index must be an int")
            if leg.get("kind") not in ("transit", "approach", "trace", "retract"):
                errors.append(f"plan.legs[{idx}].kind is invalid: {leg.get('kind')!r}")
            positions = leg.get("positions")
            if not isinstance(positions, list) or not positions or not all(
                _is_number_list(p, 6) for p in positions
            ):
                errors.append(
                    f"plan.legs[{idx}].positions must be a non-empty list of 6-element lists of finite numbers"
                )

    for key in ("segments", "caveats"):
        if not isinstance(doc.get(key), list):
            errors.append(f"plan.{key} must be a list")

    return errors


# ---------------------------------------------------------------------------
# C-PROVENANCE (§16.3 / §16.5)
# ---------------------------------------------------------------------------


def check_provenance(traj: dict, plan: dict, files: dict) -> dict:
    """§16.5/§17.4: a `view_move` trajectory requires its `--plan` to be a `view_plan`, and a
    `crack_task`/`test` trajectory requires its `--plan` to be a `cartesian_plan`; a mismatch
    either way is a provenance `fail` rather than a crash against the wrong-shaped plan."""
    is_view_move = traj.get("purpose") == "view_move"
    expected_plan_schema = "crackvision.view_plan/1" if is_view_move else "crackvision.cartesian_plan/1"
    plan_schema = plan.get("schema")
    if plan_schema != expected_plan_schema:
        detail = (
            f"trajectory.purpose {traj.get('purpose')!r} requires --plan schema "
            f"{expected_plan_schema!r}, got {plan_schema!r}"
        )
        numbers = {
            "trajectory": {"purpose": traj.get("purpose")},
            "plan": {"schema": plan_schema},
        }
        return {"check": "C-PROVENANCE", "outcome": "fail", "detail": detail, "numbers": numbers}

    if is_view_move:
        return _check_provenance_view_move(traj, plan, files)
    return _check_provenance_cartesian(traj, plan, files)


def _check_provenance_view_move(traj: dict, plan: dict, files: dict) -> dict:
    traj_limits_sha = (traj.get("limits_file") or {}).get("sha256")
    traj_ee_sha = traj.get("end_effector_config_sha256")
    traj_scene_sha = traj.get("scene_config_sha256")
    source = traj.get("source") or {}
    traj_paths3d = source.get("paths3d")
    traj_view_plan = source.get("view_plan") or {}
    traj_view_plan_sha = traj_view_plan.get("sha256")

    view_sources = plan.get("sources") or {}
    plan_limits_sha = (view_sources.get("limits") or {}).get("sha256")
    plan_ee_sha = (view_sources.get("end_effector") or {}).get("sha256")
    plan_scene_sha = (view_sources.get("scene") or {}).get("sha256")

    cli_limits_sha = (files.get("limits") or {}).get("sha256")
    cli_ee_sha = (files.get("end_effector") or {}).get("sha256")
    plan_file_sha = files.get("plan_sha256")

    errors: list[str] = []
    if traj_paths3d is not None:
        errors.append(f"trajectory.source.paths3d must be null for purpose 'view_move', got {traj_paths3d!r}")
    if traj_view_plan_sha != plan_file_sha:
        errors.append(
            f"trajectory.source.view_plan.sha256 ({traj_view_plan_sha!r}) != --plan file sha256 "
            f"({plan_file_sha!r})"
        )
    if traj_limits_sha != cli_limits_sha:
        errors.append(
            f"trajectory.limits_file.sha256 ({traj_limits_sha!r}) != --limits file sha256 ({cli_limits_sha!r})"
        )
    if traj_limits_sha != plan_limits_sha:
        errors.append(
            f"trajectory.limits_file.sha256 ({traj_limits_sha!r}) != "
            f"view_plan.sources.limits.sha256 ({plan_limits_sha!r})"
        )
    if traj_ee_sha != cli_ee_sha:
        errors.append(
            f"trajectory.end_effector_config_sha256 ({traj_ee_sha!r}) != "
            f"--end-effector-config file sha256 ({cli_ee_sha!r})"
        )
    if traj_ee_sha != plan_ee_sha:
        errors.append(
            f"trajectory.end_effector_config_sha256 ({traj_ee_sha!r}) != "
            f"view_plan.sources.end_effector.sha256 ({plan_ee_sha!r})"
        )
    if traj_scene_sha != plan_scene_sha:
        errors.append(
            f"trajectory.scene_config_sha256 ({traj_scene_sha!r}) != "
            f"view_plan.sources.scene.sha256 ({plan_scene_sha!r})"
        )

    numbers = {
        "trajectory": {
            "limits_sha256": traj_limits_sha,
            "end_effector_config_sha256": traj_ee_sha,
            "scene_config_sha256": traj_scene_sha,
            "paths3d": traj_paths3d,
            "view_plan_sha256": traj_view_plan_sha,
        },
        "view_plan": {
            "limits_sha256": plan_limits_sha,
            "end_effector_sha256": plan_ee_sha,
            "scene_sha256": plan_scene_sha,
        },
        "cli_files": {
            "limits_sha256": cli_limits_sha,
            "end_effector_sha256": cli_ee_sha,
            "plan_sha256": plan_file_sha,
            "fk_files": files.get("fk_files"),
        },
    }

    outcome = "fail" if errors else "pass"
    detail = "; ".join(errors) if errors else "view_move trajectory provenance matches view plan and command-line files"
    return {"check": "C-PROVENANCE", "outcome": outcome, "detail": detail, "numbers": numbers}


def _check_provenance_cartesian(traj: dict, plan: dict, files: dict) -> dict:
    traj_limits_sha = (traj.get("limits_file") or {}).get("sha256")
    traj_ee_sha = traj.get("end_effector_config_sha256")
    traj_scene_sha = traj.get("scene_config_sha256")
    traj_paths3d = (traj.get("source") or {}).get("paths3d")

    plan_sources = plan.get("sources") or {}
    plan_limits_sha = (plan_sources.get("limits") or {}).get("sha256")
    plan_ee_sha = (plan_sources.get("end_effector") or {}).get("sha256")
    plan_scene_sha = (plan_sources.get("scene") or {}).get("sha256")
    plan_paths3d = plan_sources.get("paths3d")

    cli_limits_sha = (files.get("limits") or {}).get("sha256")
    cli_ee_sha = (files.get("end_effector") or {}).get("sha256")

    errors: list[str] = []
    if traj_limits_sha != cli_limits_sha:
        errors.append(
            f"trajectory.limits_file.sha256 ({traj_limits_sha!r}) != --limits file sha256 ({cli_limits_sha!r})"
        )
    if traj_limits_sha != plan_limits_sha:
        errors.append(
            f"trajectory.limits_file.sha256 ({traj_limits_sha!r}) != plan.sources.limits.sha256 ({plan_limits_sha!r})"
        )
    if traj_ee_sha != cli_ee_sha:
        errors.append(
            f"trajectory.end_effector_config_sha256 ({traj_ee_sha!r}) != "
            f"--end-effector-config file sha256 ({cli_ee_sha!r})"
        )
    if traj_ee_sha != plan_ee_sha:
        errors.append(
            f"trajectory.end_effector_config_sha256 ({traj_ee_sha!r}) != "
            f"plan.sources.end_effector.sha256 ({plan_ee_sha!r})"
        )
    if traj_scene_sha != plan_scene_sha:
        errors.append(
            f"trajectory.scene_config_sha256 ({traj_scene_sha!r}) != plan.sources.scene.sha256 ({plan_scene_sha!r})"
        )
    if traj_paths3d != plan_paths3d:
        errors.append(f"trajectory.source.paths3d ({traj_paths3d!r}) != plan.sources.paths3d ({plan_paths3d!r})")

    traj_points: list[dict] = traj.get("points") or []
    plan_legs: list[dict] = plan.get("legs") or []
    plan_start: list[float] | None = None
    plan_end: list[float] | None = None
    if plan_legs:
        first_leg = min(plan_legs, key=lambda leg: leg["leg_index"])
        last_leg = max(plan_legs, key=lambda leg: leg["leg_index"])
        plan_start = first_leg["positions"][0]
        plan_end = last_leg["positions"][-1]

    traj_start = traj_points[0]["positions"] if traj_points else None
    traj_end = traj_points[-1]["positions"] if traj_points else None

    if plan_start is not None and traj_start is not None and list(traj_start) != list(plan_start):
        errors.append(
            f"trajectory.points[0].positions ({traj_start!r}) != plan's lowest-leg_index leg's first "
            f"configuration ({plan_start!r})"
        )
    if plan_end is not None and traj_end is not None and list(traj_end) != list(plan_end):
        errors.append(
            f"trajectory.points[-1].positions ({traj_end!r}) != plan's last leg's last configuration "
            f"({plan_end!r})"
        )

    numbers = {
        "trajectory": {
            "limits_sha256": traj_limits_sha,
            "end_effector_config_sha256": traj_ee_sha,
            "scene_config_sha256": traj_scene_sha,
            "paths3d": traj_paths3d,
            "start_positions": traj_start,
            "end_positions": traj_end,
        },
        "plan": {
            "limits_sha256": plan_limits_sha,
            "end_effector_sha256": plan_ee_sha,
            "scene_sha256": plan_scene_sha,
            "paths3d": plan_paths3d,
            "start_positions": plan_start,
            "end_positions": plan_end,
        },
        "cli_files": {
            "limits_sha256": cli_limits_sha,
            "end_effector_sha256": cli_ee_sha,
            "fk_files": files.get("fk_files"),
        },
    }

    outcome = "fail" if errors else "pass"
    detail = "; ".join(errors) if errors else "trajectory provenance matches plan and command-line files"
    return {"check": "C-PROVENANCE", "outcome": outcome, "detail": detail, "numbers": numbers}


# ---------------------------------------------------------------------------
# C-LIMITS (§16.2 G-VEL/G-ACC formulas, at raw limits)
# ---------------------------------------------------------------------------


def check_limits(traj: dict, limits: dict) -> dict:
    joint_names: list[str] = traj["joint_names"]
    points: list[dict] = traj["points"]
    n = len(points)
    joints_cfg = limits.get("joints", {})
    t = [float(p["t_s"]) for p in points]

    violations: list[str] = []
    peak_velocity_fraction: dict[str, float] = {}
    peak_acceleration_fraction: dict[str, float] = {}

    for j_idx, name in enumerate(joint_names):
        jcfg = joints_cfg.get(name)
        if jcfg is None:
            violations.append(f"limits file has no entry for joint {name!r}")
            continue
        V = float(jcfg["velocity"])
        A = float(jcfg["acceleration"])
        q = [float(p["positions"][j_idx]) for p in points]

        v = [(q[i + 1] - q[i]) / (t[i + 1] - t[i]) for i in range(n - 1)]
        max_v = max((abs(x) for x in v), default=0.0)
        peak_velocity_fraction[name] = max_v / V if V > 0 else float("inf")
        for i, vi in enumerate(v):
            if abs(vi) > V + _FLOAT_TOL:
                violations.append(f"joint {name} segment {i} FD velocity {vi:.6f} exceeds limit {V}")

        a = []
        for i in range(1, n - 1):
            denom = ((t[i] - t[i - 1]) + (t[i + 1] - t[i])) / 2.0
            a.append((v[i] - v[i - 1]) / denom)
        max_a = max((abs(x) for x in a), default=0.0)
        peak_acceleration_fraction[name] = max_a / A if A > 0 else float("inf")
        for i, ai in enumerate(a):
            if abs(ai) > A + _FLOAT_TOL:
                violations.append(f"joint {name} point {i + 1} FD acceleration {ai:.6f} exceeds limit {A}")

    outcome = "fail" if violations else "pass"
    detail = "; ".join(violations) if violations else "FD velocity/acceleration within raw b601_dm_limits.yaml bounds"
    numbers = {
        "peak_velocity_fraction": peak_velocity_fraction,
        "peak_acceleration_fraction": peak_acceleration_fraction,
        "violations": violations,
    }
    return {"check": "C-LIMITS", "outcome": outcome, "detail": detail, "numbers": numbers}


# ---------------------------------------------------------------------------
# C-MARGIN (§11.6 position-margin policy, FD + explicit-velocity passes)
# ---------------------------------------------------------------------------


def check_margin(traj: dict, limits: dict, margin_rad: float) -> dict:
    joint_names: list[str] = traj["joint_names"]
    points: list[dict] = traj["points"]
    n = len(points)
    joints_cfg = limits.get("joints", {})
    m = float(margin_rad)
    t = [float(p["t_s"]) for p in points]

    violations: list[str] = []

    for j_idx, name in enumerate(joint_names):
        jcfg = joints_cfg.get(name)
        if jcfg is None:
            violations.append(f"limits file has no entry for joint {name!r}")
            continue
        lower = float(jcfg["lower"])
        upper = float(jcfg["upper"])
        q = [float(p["positions"][j_idx]) for p in points]

        def fd_w(i: int, sign: float) -> float:
            if i < n - 1:
                return sign * ((q[i + 1] - q[i]) / (t[i + 1] - t[i]))
            vel = points[i].get("velocities")
            return sign * float(vel[j_idx]) if vel is not None else 0.0

        def explicit_w(i: int, sign: float) -> float | None:
            vel = points[i].get("velocities")
            return sign * float(vel[j_idx]) if vel is not None else None

        for sign, bound_name in ((1.0, "upper"), (-1.0, "lower")):
            for i in range(n):
                qi = q[i]
                near = (upper - qi <= m) if sign > 0 else (qi - lower <= m)
                if not near:
                    continue
                if i == n - 1:
                    w_last = explicit_w(i, sign)
                    if w_last is not None and w_last > 0:
                        violations.append(
                            f"joint {name} last point near {bound_name} with explicit w={w_last:.6f} > 0"
                        )
                    continue
                w_i = fd_w(i, sign)
                if w_i > 0:
                    w_next = fd_w(i + 1, sign)
                    if w_next > w_i + _FLOAT_TOL:
                        violations.append(
                            f"joint {name} point {i} near {bound_name}: FD w={w_i:.6f} speeds up to {w_next:.6f}"
                        )
                w_i_explicit = explicit_w(i, sign)
                if w_i_explicit is not None and w_i_explicit > 0:
                    w_next_explicit = explicit_w(i + 1, sign)
                    if w_next_explicit is not None and w_next_explicit > w_i_explicit + _FLOAT_TOL:
                        violations.append(
                            f"joint {name} point {i} near {bound_name}: explicit w={w_i_explicit:.6f} "
                            f"speeds up to {w_next_explicit:.6f}"
                        )

    outcome = "fail" if violations else "pass"
    detail = (
        "; ".join(violations)
        if violations
        else "§11.6 position-margin rule holds (FD pass, and explicit-velocity pass where present)"
    )
    numbers = {"violations": violations, "position_margin_rad": m}
    return {"check": "C-MARGIN", "outcome": outcome, "detail": detail, "numbers": numbers}


# ---------------------------------------------------------------------------
# C-REST (§16.2 G-REST)
# ---------------------------------------------------------------------------


def check_rest(traj: dict, limits: dict, retime_cfg: dict) -> dict:
    points: list[dict] = traj["points"]
    n = len(points)
    if n < 2:
        return {
            "check": "C-REST",
            "outcome": "skip",
            "detail": "fewer than 2 points: no first/last segment to evaluate",
            "numbers": {},
        }

    joint_names: list[str] = traj["joint_names"]
    joints_cfg = limits.get("joints", {})
    f_a = float(retime_cfg["limit_fraction"]["acceleration"])
    t = [float(p["t_s"]) for p in points]
    tau0 = t[1] - t[0]
    tau_last = t[n - 1] - t[n - 2]

    violations: list[str] = []
    numbers: dict[str, Any] = {}

    for j_idx, name in enumerate(joint_names):
        jcfg = joints_cfg.get(name)
        if jcfg is None:
            violations.append(f"limits file has no entry for joint {name!r}")
            continue
        A = float(jcfg["acceleration"])
        q0 = float(points[0]["positions"][j_idx])
        q1 = float(points[1]["positions"][j_idx])
        q_pen = float(points[n - 2]["positions"][j_idx])
        q_last = float(points[n - 1]["positions"][j_idx])

        v0 = (q1 - q0) / tau0
        v_last = (q_last - q_pen) / tau_last
        bound0 = f_a * A * tau0
        bound_last = f_a * A * tau_last
        numbers[name] = {"v0": v0, "bound0": bound0, "v_last": v_last, "bound_last": bound_last}

        if abs(v0) > bound0 + _FLOAT_TOL:
            violations.append(
                f"joint {name} first-segment FD velocity {v0:.6f} exceeds rest-start bound {bound0:.6f}"
            )
        if abs(v_last) > bound_last + _FLOAT_TOL:
            violations.append(
                f"joint {name} last-segment FD velocity {v_last:.6f} exceeds rest-end bound {bound_last:.6f}"
            )

    outcome = "fail" if violations else "pass"
    detail = (
        "; ".join(violations)
        if violations
        else "G-REST holds: first/last segment FD velocity within the accel-from-rest bound"
    )
    return {"check": "C-REST", "outcome": outcome, "detail": detail, "numbers": numbers}


# ---------------------------------------------------------------------------
# C-SAMPLING (§16.2 G-STEP/G-PERIOD)
# ---------------------------------------------------------------------------


def check_sampling(traj: dict, retime_cfg: dict) -> dict:
    points: list[dict] = traj["points"]
    n = len(points)
    if n < 2:
        return {
            "check": "C-SAMPLING",
            "outcome": "skip",
            "detail": "fewer than 2 points: no segments to evaluate",
            "numbers": {},
        }

    max_step = float(retime_cfg["sampling"]["max_joint_step_rad"])
    max_period = float(retime_cfg["sampling"]["max_period_s"])
    t = [float(p["t_s"]) for p in points]

    violations: list[str] = []
    max_step_seen = 0.0
    max_period_seen = 0.0

    for i in range(n - 1):
        tau = t[i + 1] - t[i]
        max_period_seen = max(max_period_seen, tau)
        if tau > max_period + _FLOAT_TOL:
            violations.append(f"segment {i} duration {tau:.6f}s exceeds max_period_s {max_period}")

        pos_i = points[i]["positions"]
        pos_next = points[i + 1]["positions"]
        step = max(abs(float(b) - float(a)) for a, b in zip(pos_i, pos_next))
        max_step_seen = max(max_step_seen, step)
        if step > max_step + _FLOAT_TOL:
            violations.append(f"segment {i} max joint step {step:.6f} exceeds max_joint_step_rad {max_step}")

    outcome = "fail" if violations else "pass"
    detail = "; ".join(violations) if violations else "G-STEP/G-PERIOD hold"
    numbers = {
        "max_joint_step_rad_seen": max_step_seen,
        "max_period_s_seen": max_period_seen,
        "violations": violations,
    }
    return {"check": "C-SAMPLING", "outcome": outcome, "detail": detail, "numbers": numbers}


# ---------------------------------------------------------------------------
# Plan polyline (§16.2 Step 1) and point-to-polyline projection (shared by C-PATH, C-TOOLSPEED,
# C-REPORT — "leg membership comes from the projection, never from the report", §16.5)
# ---------------------------------------------------------------------------


def _within_continuity(a: list[float], b: list[float]) -> bool:
    return max(abs(float(x) - float(y)) for x, y in zip(a, b)) <= _CONTINUITY_TOL_RAD


def _bit_exact(a: list[float], b: list[float]) -> bool:
    return len(a) == len(b) and all(float(x) == float(y) for x, y in zip(a, b))


def build_plan_polyline(plan: dict) -> tuple[list[list[float]], list[float], list[dict]]:
    """§16.2 Step 1: concatenate `plan["legs"][].positions` in `leg_index` order, dropping each
    leg's first configuration when within `_CONTINUITY_TOL_RAD` of the previous leg's last.

    Returns `(P, cumlen, leg_info)`: `P` the plan polyline (bit-exact copies), `cumlen[k]` the
    Euclidean arc length from `P[0]` to `P[k]`, and one `{leg_index, kind, p_first, p_last}` per
    leg (inclusive indices into `P`, sharing one index at a dropped boundary, §16.4).

    §17.4: a `view_plan` (`plan["schema"] == "crackvision.view_plan/1"`) has no `legs[]`; its single
    polyline is `legs=[transit_leg]` (one leg, no concatenation), normalised here once so
    `check_path`/`check_toolspeed`/`check_report` run unchanged downstream.
    """
    raw_legs = plan["transit_leg"] if "legs" not in plan else plan["legs"]
    if isinstance(raw_legs, dict):
        raw_legs = [raw_legs]
    legs = sorted(raw_legs, key=lambda leg: leg["leg_index"])
    P: list[list[float]] = []
    cumlen: list[float] = []
    leg_info: list[dict] = []
    for leg in legs:
        positions = [[float(x) for x in pos] for pos in leg["positions"]]
        start = 0
        if P and _within_continuity(positions[0], P[-1]):
            start = 1
            p_first = len(P) - 1
        else:
            p_first = len(P)
        for pos in positions[start:]:
            if P:
                seg_len = math.sqrt(sum((a - b) ** 2 for a, b in zip(pos, P[-1])))
                cumlen.append(cumlen[-1] + seg_len)
            else:
                cumlen.append(0.0)
            P.append(pos)
        p_last = len(P) - 1
        leg_info.append(
            {"leg_index": leg["leg_index"], "kind": leg["kind"], "p_first": p_first, "p_last": p_last}
        )
    return P, cumlen, leg_info


def _point_to_segment(q: np.ndarray, a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-18:
        s = 0.0
    else:
        s = float(np.dot(q - a, ab) / denom)
        s = min(1.0, max(0.0, s))
    proj = a + s * ab
    dist = float(np.linalg.norm(q - proj))
    return s, dist


def project_polyline(positions: list[list[float]], P: list[list[float]]) -> list[dict]:
    """Project each `positions[i]` onto the nearest segment of polyline `P`, walking a window
    around the previous match and falling back to a global search when the window result exceeds
    the path tolerance (keeps this O(N) in the common, monotone-progress case).

    When several segments tie for nearest (within `_PATH_TOL_RAD`) -- which happens whenever the
    polyline revisits an earlier configuration, e.g. a retract leg that ends back at an earlier
    leg's vertex -- the tie is broken by preferring the candidate with the smallest progress that
    is still `>= ` the previous trajectory point's progress, so progress stays monotone and a
    revisited vertex is not reassigned to the earlier leg it also touches."""
    P_arr = [np.asarray(p, dtype=float) for p in P]
    n_segs = len(P_arr) - 1
    seg_len = [float(np.linalg.norm(P_arr[k + 1] - P_arr[k])) for k in range(n_segs)]
    cumlen = [0.0]
    for length in seg_len:
        cumlen.append(cumlen[-1] + length)

    def eval_range(q: np.ndarray, lo: int, hi: int, prev_progress: float | None):
        candidates = []
        for k in range(lo, hi):
            s, d = _point_to_segment(q, P_arr[k], P_arr[k + 1])
            progress = cumlen[k] + s * seg_len[k]
            candidates.append((d, k, s, progress))
        if not candidates:
            return None
        d_min = min(c[0] for c in candidates)
        tied = [c for c in candidates if c[0] <= d_min + _PATH_TOL_RAD]
        if prev_progress is not None:
            forward = [c for c in tied if c[3] >= prev_progress - _FLOAT_TOL]
            if forward:
                return min(forward, key=lambda c: c[3])
        return min(tied, key=lambda c: c[3])

    results: list[dict] = []
    cursor = 0
    window = 4
    prev_progress: float | None = None
    for pos in positions:
        q = np.asarray(pos, dtype=float)
        if n_segs == 0:
            dist = float(np.linalg.norm(q - P_arr[0]))
            results.append({"segment": 0, "s": 0.0, "distance": dist, "progress": 0.0})
            prev_progress = 0.0
            continue
        lo = max(0, cursor - window)
        hi = min(n_segs, cursor + window + 1)
        best = eval_range(q, lo, hi, prev_progress)
        if best is None or best[0] > _PATH_TOL_RAD:
            global_best = eval_range(q, 0, n_segs, prev_progress)
            if global_best is not None and (best is None or global_best[0] < best[0]):
                best = global_best
        dist, k, s, progress = best
        cursor = k
        prev_progress = progress
        results.append({"segment": k, "s": s, "distance": dist, "progress": progress})
    return results


def _build_projection(traj_doc: dict, plan_doc: dict) -> dict:
    P, cumlen, leg_info = build_plan_polyline(plan_doc)
    positions = [[float(x) for x in p["positions"]] for p in traj_doc["points"]]
    points = project_polyline(positions, P)
    return {"P": P, "cumlen": cumlen, "leg_info": leg_info, "points": points, "positions": positions}


def _interior_vertices_with_turn(P_slice: list[list[float]]) -> list[list[float]]:
    """G-TOOLSPEED's vertex-dip amendment (§16.2/§16.5, MOT-07.11): this leg's own interior plan
    vertices (`P_slice`, pre-refine, §16.2 Step 1) whose joint-space direction genuinely changes
    (excludes a redundant, collinear waypoint, `_VERTEX_DIR_EPS_RAD`). Independently reimplemented
    from `crackvision_motion.retime._interior_vertices_with_turn` by hand -- this checker never
    imports `crackvision_motion`."""
    vertices: list[list[float]] = []
    for k in range(1, len(P_slice) - 1):
        prev = np.asarray(P_slice[k], dtype=float) - np.asarray(P_slice[k - 1], dtype=float)
        nxt = np.asarray(P_slice[k + 1], dtype=float) - np.asarray(P_slice[k], dtype=float)
        n_prev = float(np.linalg.norm(prev))
        n_next = float(np.linalg.norm(nxt))
        if n_prev <= _VERTEX_DIR_EPS_RAD or n_next <= _VERTEX_DIR_EPS_RAD:
            continue
        u = prev / n_prev
        w = nxt / n_next
        turn = 2.0 * math.atan2(float(np.linalg.norm(u - w)), float(np.linalg.norm(u + w)))
        if turn > _VERTEX_DIR_EPS_RAD:
            vertices.append(P_slice[k])
    return vertices


def _vertex_segments_for_leg(vertices: list[list[float]], leg_positions: list[list[float]]) -> frozenset[int]:
    """Segment indices local to one trace leg's own trajectory point range (`leg_positions`) that
    are adjacent to a bit-exact match of one of `vertices` -- the set of segments G-TOOLSPEED's
    vertex-dip amendment may excuse (mirrors `crackvision_motion.retime._vertex_segments_for_leg`
    by hand, on the trajectory's own fixed points rather than a bisection trial's output mesh)."""
    n_points = len(leg_positions)
    segments: set[int] = set()
    for vertex in vertices:
        for m, pos in enumerate(leg_positions):
            if _bit_exact(pos, vertex):
                if m - 1 >= 0:
                    segments.add(m - 1)
                if m <= n_points - 2:
                    segments.add(m)
    return frozenset(segments)


def _leg_vertex_segments(projection: dict, leg: dict, first: int, last: int) -> frozenset[int]:
    plan_vertices = _interior_vertices_with_turn(projection["P"][leg["p_first"]:leg["p_last"] + 1])
    leg_positions = projection["positions"][first:last + 1]
    return _vertex_segments_for_leg(plan_vertices, leg_positions)


def _leg_point_ranges(projection: dict) -> dict[int, tuple[int | None, int | None, str]]:
    """Map each plan leg to `[first, last]` trajectory point indices, derived purely from the
    projection's arc-length progress against each leg's own `[p_first, p_last]` span on the plan
    polyline — never from any report field (§16.5)."""
    cumlen = projection["cumlen"]
    points = projection["points"]
    # Float-noise slack only (MOT-07.11): a looser, path-tolerance-sized slack (formerly 1e-5 rad)
    # wrongly also assigned to the next leg a point lying on this leg's own last segment just short
    # of a rest boundary (the fixture's retract->transit vertex has a point 9.4e-6 rad before it).
    eps = _FLOAT_TOL
    ranges: dict[int, tuple[int | None, int | None, str]] = {}
    for leg in projection["leg_info"]:
        lo = cumlen[leg["p_first"]]
        hi = cumlen[leg["p_last"]]
        idxs = [i for i, p in enumerate(points) if lo - eps <= p["progress"] <= hi + eps]
        if idxs:
            ranges[leg["leg_index"]] = (min(idxs), max(idxs), leg["kind"])
        else:
            ranges[leg["leg_index"]] = (None, None, leg["kind"])
    return ranges


# ---------------------------------------------------------------------------
# C-PATH (§16.2 G-PATH, independently recomputed)
# ---------------------------------------------------------------------------


def check_path(traj: dict, plan: dict, projection: dict | None = None) -> dict:
    if projection is None:
        projection = _build_projection(traj, plan)

    P = projection["P"]
    positions = projection["positions"]
    points = projection["points"]

    if not P:
        return {"check": "C-PATH", "outcome": "fail", "detail": "plan polyline is empty", "numbers": {}}

    violations: list[str] = []
    max_dist = 0.0
    for i, proj in enumerate(points):
        max_dist = max(max_dist, proj["distance"])
        if proj["distance"] > _PATH_TOL_RAD:
            violations.append(
                f"trajectory point {i} is {proj['distance']:.9f} rad from the nearest plan-polyline "
                f"segment, exceeding the {_PATH_TOL_RAD} rad tolerance"
            )

    for i in range(1, len(points)):
        if points[i]["progress"] < points[i - 1]["progress"] - _FLOAT_TOL:
            violations.append(
                f"trajectory point {i} polyline progress ({points[i]['progress']:.9f}) regresses "
                f"from point {i - 1}'s ({points[i - 1]['progress']:.9f})"
            )

    match_idx = 0
    missing_vertices: list[int] = []
    for k, p_k in enumerate(P):
        found = False
        for i in range(match_idx, len(positions)):
            if _bit_exact(positions[i], p_k):
                match_idx = i + 1
                found = True
                break
        if not found:
            missing_vertices.append(k)
    if missing_vertices:
        violations.append(
            f"plan configuration(s) at plan-polyline index {missing_vertices} not found bit-exact, "
            "in order, in the trajectory"
        )

    if not positions or not _bit_exact(positions[0], P[0]):
        violations.append("trajectory's first point does not equal the plan polyline's first configuration")
    if not positions or not _bit_exact(positions[-1], P[-1]):
        violations.append("trajectory's last point does not equal the plan polyline's last configuration")

    outcome = "fail" if violations else "pass"
    detail = "; ".join(violations) if violations else "trajectory follows the plan polyline exactly (G-PATH)"
    numbers = {
        "path_tolerance_rad": _PATH_TOL_RAD,
        "max_distance_rad": max_dist,
        "n_plan_vertices": len(P),
        "n_trajectory_points": len(positions),
        "violations": violations,
    }
    return {"check": "C-PATH", "outcome": outcome, "detail": detail, "numbers": numbers}


# ---------------------------------------------------------------------------
# C-TOOLSPEED (§16.2 G-TOOLSPEED, §16.5 "v_c inference")
# ---------------------------------------------------------------------------


def _toolspeed_rule23_ok(
    speeds: list[float], taus: list[float], tol: float, v_c: float, max_ramp_m: float,
    vertex_segments: frozenset[int] = frozenset(),
) -> bool:
    """G-TOOLSPEED rules 2-3 (§16.2 Step 4) for one trace leg's fixed segment speeds at a candidate
    `v_c`. Rule 2 amendment (MOT-07.11, ADR-019 addendum): besides the leading/trailing ramp, an
    interior maximal below-band run (a vertex-tied dip) is also permitted when it overlaps a segment
    in `vertex_segments` (a genuine interior plan vertex) and is bounded (rule 3): its own tool arc
    length is `<= max_ramp_m`, and all vertex-tied dips together cover at most `tol` of the leg's
    tool arc length. Any below-band run that is neither a ramp nor vertex-tied still fails, exactly
    as before this amendment."""
    n = len(speeds)
    below = [s < v_c * (1.0 - tol) for s in speeds]
    good_idx = [i for i in range(n) if not below[i]]
    if not good_idx:
        ramp_len = sum(s * t for s, t in zip(speeds, taus))
        return ramp_len <= max_ramp_m + _FLOAT_TOL
    start, end = min(good_idx), max(good_idx)

    i = start
    dip_total = 0.0
    while i <= end:
        if not below[i]:
            i += 1
            continue
        j = i
        while j <= end and below[j]:
            j += 1
        if not any(seg in vertex_segments for seg in range(i, j)):
            return False
        run_len = sum(s * t for s, t in zip(speeds[i:j], taus[i:j]))
        if run_len > max_ramp_m + _FLOAT_TOL:
            return False
        dip_total += run_len
        i = j
    leg_len = sum(s * t for s, t in zip(speeds, taus))
    if dip_total > tol * leg_len + _FLOAT_TOL:
        return False

    ramp1_len = sum(s * t for s, t in zip(speeds[:start], taus[:start]))
    ramp2_len = sum(s * t for s, t in zip(speeds[end + 1 :], taus[end + 1 :]))
    return ramp1_len <= max_ramp_m + _FLOAT_TOL and ramp2_len <= max_ramp_m + _FLOAT_TOL


def _merge_intervals(intervals: list[list[float]]) -> list[list[float]]:
    if not intervals:
        return []
    ordered = sorted(intervals, key=lambda iv: iv[0])
    merged = [list(ordered[0])]
    for lo, hi in ordered[1:]:
        if lo <= merged[-1][1] + 1e-12:
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])
    return merged


def _toolspeed_feasible_set(
    speeds: list[float], taus: list[float], tol: float, v_min: float, v_max: float, max_ramp_m: float,
    vertex_segments: frozenset[int] = frozenset(),
) -> tuple[list[list[float]], float | None, float | None]:
    """§16.5 "C-TOOLSPEED v_c inference": the feasible set `F` of constant tool speeds `v_c` for
    which this leg's fixed segment speeds satisfy G-TOOLSPEED rules 1-3 (rule 2's vertex-dip
    amendment per `vertex_segments`, MOT-07.11). Returns `(F, L, v_0)` -- `L`/`v_0` are `None` for
    an `n = 0` leg, for which rule 1 (and so `L`) is not defined."""
    n = len(speeds)
    if n == 0:
        return [[v_min, v_max]], None, None
    L = max(speeds) / (1.0 + tol)
    v0 = max(L, v_min)
    if v0 > v_max + _FLOAT_TOL:
        return [], L, v0

    breakpoints = {v0, v_max}
    for s in speeds:
        bp = s / (1.0 - tol)
        if v0 < bp < v_max:
            breakpoints.add(bp)
    ordered = sorted(breakpoints)

    pieces: list[tuple[float, float]] = [(u, u) for u in ordered]
    pieces += [(a, b) for a, b in zip(ordered[:-1], ordered[1:])]

    passing: list[list[float]] = []
    for lo, hi in pieces:
        rep = lo if lo == hi else (lo + hi) / 2.0
        if _toolspeed_rule23_ok(speeds, taus, tol, rep, max_ramp_m, vertex_segments):
            passing.append([lo, hi])
    return _merge_intervals(passing), L, v0


def _distance_to_intervals(v: float, intervals: list[list[float]]) -> float:
    if not intervals:
        return float("inf")
    best = float("inf")
    for lo, hi in intervals:
        if lo <= v <= hi:
            return 0.0
        best = min(best, (lo - v) if v < lo else (v - hi))
    return best


def _tool_tip_positions(
    point_idxs: list[int], points: list[dict], chain: KinematicChain, end_effector: EndEffector
) -> list[np.ndarray]:
    tool_T = end_effector.T_gripper_link_tool_tip
    return [(chain.fk(points[i]["positions"]) @ tool_T)[:3, 3] for i in point_idxs]


def _leg_speeds(
    first: int, last: int, points: list[dict], chain: KinematicChain, end_effector: EndEffector
) -> tuple[list[float], list[float]]:
    idxs = list(range(first, last + 1))
    tool_pos = _tool_tip_positions(idxs, points, chain, end_effector)
    t = [float(points[i]["t_s"]) for i in idxs]
    speeds: list[float] = []
    taus: list[float] = []
    for j in range(len(idxs) - 1):
        tau = t[j + 1] - t[j]
        dist = float(np.linalg.norm(tool_pos[j + 1] - tool_pos[j]))
        taus.append(tau)
        speeds.append(dist / tau if tau > 0 else float("inf"))
    return speeds, taus


def check_toolspeed(
    traj: dict,
    plan: dict,
    retime_cfg: dict,
    chain: KinematicChain | None,
    end_effector: EndEffector | None,
    projection: dict | None = None,
) -> dict:
    if projection is None:
        projection = _build_projection(traj, plan)

    trace_legs = [leg for leg in projection["leg_info"] if leg["kind"] == "trace"]
    if not trace_legs:
        if plan.get("schema") == "crackvision.view_plan/1":
            # §17.4: a view-move polyline is `kind == "transit"` only, so there is no leg for
            # G-TOOLSPEED to evaluate -- a vacuous pass, not a skip (C-SCHEMA, its precondition,
            # did run and succeed).
            return {
                "check": "C-TOOLSPEED",
                "outcome": "pass",
                "detail": "view-move polyline has no trace legs: G-TOOLSPEED is vacuously satisfied",
                "numbers": {"trace_legs": []},
            }
        return {"check": "C-TOOLSPEED", "outcome": "skip", "detail": "plan has no trace legs", "numbers": {}}

    trace_cfg = retime_cfg["trace"]
    v_max_cfg = float(trace_cfg["tool_speed_m_s"])
    v_min_cfg = float(trace_cfg["min_tool_speed_m_s"])
    tol = float(trace_cfg["speed_tolerance"])
    max_ramp_m = float(trace_cfg["max_ramp_m"])

    points = traj["points"]
    leg_ranges = _leg_point_ranges(projection)

    violations: list[str] = []
    leg_numbers: dict[int, dict] = {}
    for leg in trace_legs:
        leg_index = leg["leg_index"]
        first, last, _kind = leg_ranges.get(leg_index, (None, None, "trace"))
        if first is None or last is None or last <= first:
            leg_numbers[leg_index] = {
                "outcome": "skip",
                "reason": "fewer than 2 trajectory points were assigned to this leg by the projection",
            }
            continue
        speeds, taus = _leg_speeds(first, last, points, chain, end_effector)
        vertex_segments = _leg_vertex_segments(projection, leg, first, last)
        F, L, v0 = _toolspeed_feasible_set(speeds, taus, tol, v_min_cfg, v_max_cfg, max_ramp_m, vertex_segments)
        leg_numbers[leg_index] = {
            "point_range": [first, last],
            "speeds": speeds,
            "min_speed": min(speeds) if speeds else None,
            "max_speed": max(speeds) if speeds else None,
            "L": L,
            "v_0": v0,
            "v_max": v_max_cfg,
            "F": F,
            "vertex_segments": sorted(vertex_segments),
        }
        if not F:
            violations.append(
                f"trace leg {leg_index}: no constant tool speed v_c in "
                f"[{v_min_cfg}, {v_max_cfg}] m/s satisfies G-TOOLSPEED (feasible set F is empty)"
            )

    outcome = "fail" if violations else "pass"
    detail = (
        "; ".join(violations)
        if violations
        else "every trace leg has a non-empty G-TOOLSPEED feasible set of constant tool speeds"
    )
    return {"check": "C-TOOLSPEED", "outcome": outcome, "detail": detail, "numbers": {"legs": leg_numbers}}


# ---------------------------------------------------------------------------
# C-REPORT (§16.4 retime_report cross-check, §16.5)
# ---------------------------------------------------------------------------


def check_report(
    traj: dict,
    plan: dict,
    retime_cfg: dict,
    chain: KinematicChain | None,
    end_effector: EndEffector | None,
    report: dict | None,
    projection: dict | None = None,
) -> dict:
    if report is None:
        return {"check": "C-REPORT", "outcome": "skip", "detail": "no --report given", "numbers": {}}
    if projection is None:
        projection = _build_projection(traj, plan)

    points = traj["points"]
    t = [float(p["t_s"]) for p in points]
    leg_ranges = _leg_point_ranges(projection)
    report_legs = {leg["leg_index"]: leg for leg in (report.get("legs") or []) if isinstance(leg, dict)}
    trace_cfg = retime_cfg.get("trace") or {}

    violations: list[str] = []
    leg_numbers: dict[int, dict] = {}
    for leg in projection["leg_info"]:
        leg_index = leg["leg_index"]
        first, last, kind = leg_ranges.get(leg_index, (None, None, leg["kind"]))
        report_leg = report_legs.get(leg_index)
        if report_leg is None:
            violations.append(f"report has no entry for leg {leg_index}")
            continue
        if first is None or last is None:
            violations.append(f"leg {leg_index}: the projection assigned it no trajectory points")
            continue

        rep_range = report_leg.get("point_range")
        if not isinstance(rep_range, list):
            violations.append(f"leg {leg_index} report point_range must be a list, got {rep_range!r}")
        elif rep_range != [first, last]:
            violations.append(f"leg {leg_index} report point_range {rep_range!r} != recomputed [{first}, {last}]")

        recomputed_duration = t[last] - t[first]
        rep_duration = report_leg.get("duration_s")
        if (
            not isinstance(rep_duration, (int, float))
            or abs(float(rep_duration) - recomputed_duration) > _REPORT_DURATION_TOL_S
        ):
            violations.append(
                f"leg {leg_index} report duration_s {rep_duration!r} != recomputed "
                f"{recomputed_duration:.9f}s (tol {_REPORT_DURATION_TOL_S}s)"
            )

        leg_number: dict[str, Any] = {"point_range": [first, last], "duration_s": recomputed_duration}

        if kind == "trace":
            v_max_cfg = float(trace_cfg["tool_speed_m_s"])
            v_min_cfg = float(trace_cfg["min_tool_speed_m_s"])
            tol = float(trace_cfg["speed_tolerance"])
            max_ramp_m = float(trace_cfg["max_ramp_m"])
            speeds, taus = _leg_speeds(first, last, points, chain, end_effector)
            vertex_segments = _leg_vertex_segments(projection, leg, first, last)
            F, _L, _v0 = _toolspeed_feasible_set(speeds, taus, tol, v_min_cfg, v_max_cfg, max_ramp_m, vertex_segments)
            observed_min = min(speeds) if speeds else None
            observed_max = max(speeds) if speeds else None
            leg_number.update({"F": F, "observed_min": observed_min, "observed_max": observed_max})

            rep_tool_speed = report_leg.get("tool_speed")
            if not isinstance(rep_tool_speed, dict):
                violations.append(f"leg {leg_index} report tool_speed must be an object, got {rep_tool_speed!r}")
                rep_tool_speed = {}
            rep_constant = rep_tool_speed.get("constant")
            if not isinstance(rep_constant, (int, float)):
                violations.append(f"leg {leg_index} report tool_speed.constant missing or non-numeric")
            else:
                dist_to_F = _distance_to_intervals(float(rep_constant), F)
                leg_number["constant_distance_to_F_m_s"] = dist_to_F
                if dist_to_F > _REPORT_SPEED_TOL_M_S + _FLOAT_TOL:
                    violations.append(
                        f"leg {leg_index} report tool_speed.constant {rep_constant} is {dist_to_F:.6f} m/s "
                        f"from the feasible set F (tol {_REPORT_SPEED_TOL_M_S} m/s)"
                    )
            for bound_name, observed in (("min", observed_min), ("max", observed_max)):
                rep_val = rep_tool_speed.get(bound_name)
                if (
                    not isinstance(rep_val, (int, float))
                    or observed is None
                    or abs(float(rep_val) - observed) > _REPORT_SPEED_TOL_M_S + _FLOAT_TOL
                ):
                    violations.append(
                        f"leg {leg_index} report tool_speed.{bound_name} {rep_val!r} != recomputed "
                        f"{observed!r} (tol {_REPORT_SPEED_TOL_M_S} m/s)"
                    )
        leg_numbers[leg_index] = leg_number

    outcome = "fail" if violations else "pass"
    detail = (
        "; ".join(violations)
        if violations
        else "report's per-leg point_range/duration_s/tool_speed agree with recomputed values"
    )
    return {"check": "C-REPORT", "outcome": outcome, "detail": detail, "numbers": {"legs": leg_numbers}}


# ---------------------------------------------------------------------------
# Report assembly (§16.5)
# ---------------------------------------------------------------------------

_SKIPPED_ON_SCHEMA_FAIL = (
    "C-PROVENANCE",
    "C-LIMITS",
    "C-MARGIN",
    "C-REST",
    "C-SAMPLING",
    "C-PATH",
    "C-TOOLSPEED",
    "C-REPORT",
)


def run_checks(
    traj_doc: dict,
    plan_doc: dict,
    *,
    limits: dict,
    margin_rad: float,
    retime_cfg: dict,
    files: dict,
    extended: bool = False,
    chain: KinematicChain | None = None,
    end_effector: EndEffector | None = None,
    report_doc: dict | None = None,
) -> dict:
    """Run C-SCHEMA, C-PROVENANCE, C-LIMITS, C-MARGIN, C-REST, C-SAMPLING (§16.5), plus — when
    `extended` is true — C-PATH, C-TOOLSPEED and C-REPORT (MOT-07.7). `extended` defaults to false
    so a caller that predates MOT-07.7 (passing none of `chain`/`end_effector`/`report_doc`) gets
    exactly the MOT-07.6 check set, unchanged."""
    traj_errors = validate_trajectory_schema(traj_doc)
    plan_errors = validate_plan_schema(plan_doc)
    schema_errors = traj_errors + plan_errors
    schema_outcome = "fail" if schema_errors else "pass"
    plan_schema_name = (
        plan_doc.get("schema") if isinstance(plan_doc, dict) else None
    ) or "crackvision.cartesian_plan/1"
    checks = [
        {
            "check": "C-SCHEMA",
            "outcome": schema_outcome,
            "detail": (
                "; ".join(schema_errors)
                if schema_errors
                else f"trajectory parses as crackvision.joint_trajectory/1 and plan as {plan_schema_name}"
            ),
            "numbers": {"trajectory_errors": traj_errors, "plan_errors": plan_errors},
        }
    ]

    if schema_outcome == "fail":
        for name in _SKIPPED_ON_SCHEMA_FAIL:
            checks.append(
                {
                    "check": name,
                    "outcome": "skip",
                    "detail": "C-SCHEMA failed: trajectory/plan did not parse into the required structures",
                    "numbers": {},
                }
            )
    else:
        checks.append(check_provenance(traj_doc, plan_doc, files))
        checks.append(check_limits(traj_doc, limits))
        checks.append(check_margin(traj_doc, limits, margin_rad))
        checks.append(check_rest(traj_doc, limits, retime_cfg))
        checks.append(check_sampling(traj_doc, retime_cfg))
        if extended:
            projection = _build_projection(traj_doc, plan_doc)
            checks.append(check_path(traj_doc, plan_doc, projection=projection))
            checks.append(
                check_toolspeed(traj_doc, plan_doc, retime_cfg, chain, end_effector, projection=projection)
            )
            checks.append(
                check_report(traj_doc, plan_doc, retime_cfg, chain, end_effector, report_doc, projection=projection)
            )

    return {
        "schema": SCHEMA,
        "created_utc": _iso_now(),
        "trajectory_sha256": files.get("trajectory_sha256"),
        "checks": checks,
    }


# ---------------------------------------------------------------------------
# CLI (§16.6, §0)
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="trajectory_check.py",
        description="Independent crackvision.trajectory_check/1 checker (docs/INTERFACES.md §16.5).",
    )
    add_common_args(parser)
    parser.add_argument("--trajectory", type=Path, required=True, help="crackvision.joint_trajectory/1 file")
    parser.add_argument(
        "--plan", type=Path, required=True,
        help="crackvision.cartesian_plan/1 or crackvision.view_plan/1 file (dispatched by schema, §17.4)",
    )
    parser.add_argument(
        "--report", type=Path, default=None,
        help="crackvision.retime_report/1 sidecar; enables C-REPORT when given",
    )
    parser.add_argument("--limits", type=Path, default=None, help="default config/robot/b601_dm_limits.yaml")
    parser.add_argument(
        "--end-effector-config", type=Path, default=None, help="default config/robot/end_effector.yaml"
    )
    parser.add_argument(
        "--execution-config", type=Path, default=None, help="default config/motion/execution.yaml"
    )
    parser.add_argument("--retime-config", type=Path, default=None, help="default config/motion/retime.yaml")
    parser.add_argument(
        "--urdf", type=Path, default=None,
        help="default presentation/sim/reBot_B601_DM_with_gripper.urdf (only read when the plan has a trace leg)",
    )
    parser.add_argument("--out", type=Path, default=None, help="default <trajectory>.check.json")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"{TOOL}: {exc}", file=sys.stderr)
        return EXIT_USAGE

    logger = setup_logging(TOOL, cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    limits_path = args.limits or (cfg.root / "config" / "robot" / "b601_dm_limits.yaml")
    end_effector_path = args.end_effector_config or (cfg.root / "config" / "robot" / "end_effector.yaml")
    execution_path = args.execution_config or (cfg.root / "config" / "motion" / "execution.yaml")
    retime_path = args.retime_config or (cfg.root / "config" / "motion" / "retime.yaml")
    urdf_path = args.urdf or (cfg.root / "presentation" / "sim" / "reBot_B601_DM_with_gripper.urdf")
    out_path = args.out or Path(str(args.trajectory) + ".check.json")

    if args.dry_run:
        logger.info(
            "dry-run: would check %s against plan %s, writing a report to %s", args.trajectory, args.plan, out_path
        )
        summary.write(cfg, TOOL, "ok", EXIT_OK)
        return EXIT_OK

    if args.skip_existing and out_path.is_file():
        logger.info("skip-existing: %s already exists", out_path)
        summary.write(cfg, TOOL, "ok", EXIT_OK)
        return EXIT_OK

    try:
        traj_doc = _load_json(Path(args.trajectory))
        plan_doc = _load_json(Path(args.plan))
        limits_doc = _load_yaml(limits_path)
        execution_doc = _load_yaml(execution_path)
        retime_doc = _load_yaml(retime_path)
        report_doc = _load_json(Path(args.report)) if args.report is not None else None
        if not end_effector_path.is_file():
            raise UsageError(f"cannot open {end_effector_path}")

        for name in JOINT_NAMES:
            jcfg = limits_doc["joints"][name]
            float(jcfg["lower"])
            float(jcfg["upper"])
            float(jcfg["velocity"])
            float(jcfg["acceleration"])
        margin_rad = float(execution_doc["position_margin_rad"])
        float(retime_doc["limit_fraction"]["acceleration"])
        float(retime_doc["sampling"]["max_joint_step_rad"])
        float(retime_doc["sampling"]["max_period_s"])

        trajectory_sha256 = file_sha256(args.trajectory)
        limits_sha256 = file_sha256(limits_path)
        end_effector_sha256 = file_sha256(end_effector_path)
        plan_sha256 = file_sha256(args.plan)

        # FK (crackvision.kinematics, never crackvision_motion.tool_fk, §16.5) is only needed, and
        # only loaded, when the plan actually has a trace leg -- C-TOOLSPEED/C-REPORT's own
        # precondition. A view_plan (§17.4) never has a trace leg.
        needs_fk = isinstance(plan_doc, dict) and any(
            isinstance(leg, dict) and leg.get("kind") == "trace" for leg in (plan_doc.get("legs") or [])
        )
        is_view_plan = isinstance(plan_doc, dict) and plan_doc.get("schema") == "crackvision.view_plan/1"
        chain = end_effector = urdf_sha256 = None
        if needs_fk:
            chain = load_chain(urdf_path=urdf_path)
            end_effector = load_end_effector(end_effector_path)
            urdf_sha256 = file_sha256(urdf_path)
    except (UsageError, KeyError, TypeError, ValueError, OSError, KinematicsError) as exc:
        msg = f"{TOOL}: usage/config error: {exc}"
        logger.error(msg)
        summary.add_error(str(exc))
        summary.write(cfg, TOOL, "failed", EXIT_USAGE)
        return EXIT_USAGE

    files = {
        "trajectory_sha256": trajectory_sha256,
        "limits": {"path": str(limits_path), "sha256": limits_sha256},
        "end_effector": {"path": str(end_effector_path), "sha256": end_effector_sha256},
        "fk_files": {"urdf_sha256": urdf_sha256, "end_effector_sha256": end_effector_sha256},
        "plan_sha256": plan_sha256,
    }

    try:
        report = run_checks(
            traj_doc, plan_doc,
            limits=limits_doc, margin_rad=margin_rad, retime_cfg=retime_doc, files=files,
            # C-PATH/C-TOOLSPEED/C-REPORT (MOT-07.7) run whenever the plan has a trace leg --
            # exactly `needs_fk`'s own condition, since a plan with no trace leg has nothing for
            # C-TOOLSPEED/C-REPORT to evaluate (§16.5's own "skip when nothing to evaluate"
            # precedent) and C-PATH piggybacks on the same gate rather than running against a
            # plan whose only legs are untraced coarse moves -- plus, always, a view_plan
            # (§17.4: C-PATH/C-TOOLSPEED/C-REPORT run on the single transit-leg polyline even
            # though it has no trace leg).
            extended=needs_fk or is_view_plan, chain=chain, end_effector=end_effector, report_doc=report_doc,
        )
    except Exception as exc:  # unexpected failure while a check is running (§16.5 exit 1)
        logger.exception("unexpected error while running checks")
        summary.add_error(str(exc))
        summary.write(cfg, TOOL, "failed", EXIT_RUNTIME)
        return EXIT_RUNTIME

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    for check in report["checks"]:
        summary.increment(check["outcome"])

    any_fail = any(check["outcome"] == "fail" for check in report["checks"])
    if any_fail:
        logger.error("trajectory_check FAIL: report written to %s", out_path)
        summary.write(cfg, TOOL, "failed", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    logger.info("trajectory_check PASS: report written to %s", out_path)
    summary.write(cfg, TOOL, "ok", EXIT_OK)
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())

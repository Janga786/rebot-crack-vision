"""crackvision.trajectory_check — independent `crackvision.trajectory_check/1` checker
(docs/INTERFACES.md §16.5, ADR-019).

A separate implementation from `crackvision_motion`'s `retime_plan`/`joint_trajectory`/
`retime_core`: built from the §11.2 (`joint_trajectory`), §11.6 (position-margin policy) and
§13.11 (`cartesian_plan`) contract text alone, sharing no code with `crackvision_motion`. Runs
under the conda interpreter (`./env.sh python -m crackvision.trajectory_check`), never imports
`crackvision_motion` or `rclpy`, and never reads `retime_plan`'s own report except where a future
C-REPORT check explicitly cross-checks it (not yet implemented — MOT-07.7).

This part (MOT-07.6) implements C-SCHEMA, C-PROVENANCE, C-LIMITS, C-MARGIN, C-REST and
C-SAMPLING. C-PATH, C-TOOLSPEED and C-REPORT are added by MOT-07.7 and are omitted from the
report until then, as §16.5 allows.
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

import yaml

from crackvision.config import Config, ConfigError, add_common_args, load_config
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
    if doc.get("purpose") not in ("crack_task", "test"):
        errors.append(f"trajectory.purpose is {doc.get('purpose')!r}, expected 'crack_task' or 'test'")
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
    if not isinstance(source, dict) or "paths3d" not in source:
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
        "cli_files": {"limits_sha256": cli_limits_sha, "end_effector_sha256": cli_ee_sha},
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
# Report assembly (§16.5)
# ---------------------------------------------------------------------------

_SKIPPED_ON_SCHEMA_FAIL = ("C-PROVENANCE", "C-LIMITS", "C-MARGIN", "C-REST", "C-SAMPLING")


def run_checks(
    traj_doc: dict,
    plan_doc: dict,
    *,
    limits: dict,
    margin_rad: float,
    retime_cfg: dict,
    files: dict,
) -> dict:
    """Run C-SCHEMA, C-PROVENANCE, C-LIMITS, C-MARGIN, C-REST, C-SAMPLING (§16.5)."""
    traj_errors = validate_trajectory_schema(traj_doc)
    plan_errors = validate_plan_schema(plan_doc)
    schema_errors = traj_errors + plan_errors
    schema_outcome = "fail" if schema_errors else "pass"
    checks = [
        {
            "check": "C-SCHEMA",
            "outcome": schema_outcome,
            "detail": (
                "; ".join(schema_errors)
                if schema_errors
                else "trajectory parses as crackvision.joint_trajectory/1 and plan as crackvision.cartesian_plan/1"
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
    parser.add_argument("--plan", type=Path, required=True, help="crackvision.cartesian_plan/1 file")
    parser.add_argument(
        "--report", type=Path, default=None,
        help="crackvision.retime_report/1 sidecar; C-REPORT is not implemented yet (MOT-07.7)",
    )
    parser.add_argument("--limits", type=Path, default=None, help="default config/robot/b601_dm_limits.yaml")
    parser.add_argument(
        "--end-effector-config", type=Path, default=None, help="default config/robot/end_effector.yaml"
    )
    parser.add_argument(
        "--execution-config", type=Path, default=None, help="default config/motion/execution.yaml"
    )
    parser.add_argument("--retime-config", type=Path, default=None, help="default config/motion/retime.yaml")
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
    out_path = args.out or Path(str(args.trajectory) + ".check.json")

    if args.report is not None:
        logger.warning("--report given but C-REPORT is not implemented until MOT-07.7; ignoring")

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
    except (UsageError, KeyError, TypeError, ValueError, OSError) as exc:
        msg = f"{TOOL}: usage/config error: {exc}"
        logger.error(msg)
        summary.add_error(str(exc))
        summary.write(cfg, TOOL, "failed", EXIT_USAGE)
        return EXIT_USAGE

    files = {
        "trajectory_sha256": trajectory_sha256,
        "limits": {"path": str(limits_path), "sha256": limits_sha256},
        "end_effector": {"path": str(end_effector_path), "sha256": end_effector_sha256},
    }

    try:
        report = run_checks(
            traj_doc, plan_doc,
            limits=limits_doc, margin_rad=margin_rad, retime_cfg=retime_doc, files=files,
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

"""crackvision.preview_core — pure core for `crackvision.trajectory_preview/1` (docs/INTERFACES.md
§18, ADR-020).

Validates and sha-binds the four preview inputs (trajectory, plan, retime report, check report),
computes the §18.1.2 summary metrics and tool-tip path via `crackvision.kinematics`, and builds,
validates and loads a `crackvision.trajectory_preview/1` manifest. No plotting, no ROS, no
`crackvision_motion`: this module is the shared core a later CLI card (MOT-08.3+) wraps, and is
exercised directly by its own tests.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from crackvision.kinematics import EndEffector, KinematicChain
from crackvision.trajectory_check import (
    validate_cartesian_plan_schema,
    validate_trajectory_schema,
    validate_view_plan_schema,
)

MANIFEST_SCHEMA = "crackvision.trajectory_preview/1"
_PLAN_SCHEMAS = ("crackvision.cartesian_plan/1", "crackvision.view_plan/1")
_PURPOSES = ("crack_task", "test", "view_move")
_ARTIFACT_KINDS = ("joints", "toolpath", "toolspeed")


class PreviewRefusal(Exception):
    """A §18 refusal rule fired: a sha-binding mismatch, an invalid input, or an invalid manifest."""


class PreviewUsageError(Exception):
    """A required input file could not be opened or did not parse as JSON/YAML at all."""


def file_sha256(path: Path | str) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _is_sha256_hex(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value.lower())


def _repo_relative(path: Path, root: Path) -> str:
    resolved = Path(path).resolve()
    try:
        return resolved.relative_to(Path(root).resolve()).as_posix()
    except ValueError:
        raise PreviewUsageError(f"{path} does not resolve under --root {root}") from None


def _load_json(path: Path) -> Any:
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        raise PreviewUsageError(f"cannot open {path}: {exc}") from exc
    try:
        return json.loads(text)
    except json.JSONDecodeError as exc:
        raise PreviewUsageError(f"malformed JSON in {path}: {exc}") from exc


def load_limits_yaml(path: Path | str) -> dict:
    """Mirrors `trajectory_check`'s own `b601_dm_limits.yaml` reading (§16.5)."""
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        raise PreviewUsageError(f"cannot open {path}: {exc}") from exc
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise PreviewUsageError(f"malformed YAML in {path}: {exc}") from exc
    if not isinstance(doc, dict):
        raise PreviewUsageError(f"{path} must contain a mapping at the top level")
    return doc


# ---------------------------------------------------------------------------
# Minimal retime_report/check_report schema checks (§16.4/§16.5), just enough for §18's sha-binding
# and leg-range reads -- the full schemas are `retime_plan`/`trajectory_check`'s own business.
# ---------------------------------------------------------------------------


def _validate_retime_report_schema(doc: Any) -> list[str]:
    if not isinstance(doc, dict):
        return ["retime report is not a JSON object"]
    errors: list[str] = []
    if doc.get("schema") != "crackvision.retime_report/1":
        errors.append(f"retime_report.schema is {doc.get('schema')!r}, expected 'crackvision.retime_report/1'")
    if not isinstance(doc.get("trajectory_sha256"), str) or not doc.get("trajectory_sha256"):
        errors.append("retime_report.trajectory_sha256 must be a non-empty string")
    legs = doc.get("legs")
    if not isinstance(legs, list) or not legs:
        errors.append("retime_report.legs must be a non-empty list")
    else:
        for idx, leg in enumerate(legs):
            if not isinstance(leg, dict):
                errors.append(f"retime_report.legs[{idx}] is not an object")
                continue
            if not isinstance(leg.get("leg_index"), int) or isinstance(leg.get("leg_index"), bool):
                errors.append(f"retime_report.legs[{idx}].leg_index must be an int")
            if not isinstance(leg.get("kind"), str) or not leg.get("kind"):
                errors.append(f"retime_report.legs[{idx}].kind must be a non-empty string")
            point_range = leg.get("point_range")
            if not (
                isinstance(point_range, list)
                and len(point_range) == 2
                and all(isinstance(x, int) and not isinstance(x, bool) for x in point_range)
            ):
                errors.append(f"retime_report.legs[{idx}].point_range must be a 2-element list of ints")
            if not _is_finite_number(leg.get("duration_s")):
                errors.append(f"retime_report.legs[{idx}].duration_s must be a finite number")
    return errors


def _validate_check_report_schema(doc: Any) -> list[str]:
    if not isinstance(doc, dict):
        return ["check report is not a JSON object"]
    errors: list[str] = []
    if doc.get("schema") != "crackvision.trajectory_check/1":
        errors.append(f"check_report.schema is {doc.get('schema')!r}, expected 'crackvision.trajectory_check/1'")
    if not isinstance(doc.get("trajectory_sha256"), str) or not doc.get("trajectory_sha256"):
        errors.append("check_report.trajectory_sha256 must be a non-empty string")
    checks = doc.get("checks")
    if not isinstance(checks, list):
        errors.append("check_report.checks must be a list")
    else:
        for idx, check in enumerate(checks):
            if (
                not isinstance(check, dict)
                or not isinstance(check.get("check"), str)
                or check.get("outcome") not in ("pass", "fail", "skip")
            ):
                errors.append(f"check_report.checks[{idx}] must be {{check: str, outcome: pass|fail|skip, ...}}")
    return errors


# ---------------------------------------------------------------------------
# bind_inputs (§18.1 refusal rules)
# ---------------------------------------------------------------------------


def bind_inputs(
    trajectory_path: Path | str,
    plan_path: Path | str,
    retime_report_path: Path | str,
    check_report_path: Path | str,
    *,
    root: Path | str,
) -> dict:
    """Load and validate the four §18.1 inputs, enforce the sha-binding refusals, and return their
    repo-relative paths/shas/docs plus the derived `check_report.all_pass`/`failed_checks`."""
    root = Path(root)
    trajectory_path = Path(trajectory_path)
    plan_path = Path(plan_path)
    retime_report_path = Path(retime_report_path)
    check_report_path = Path(check_report_path)

    traj_doc = _load_json(trajectory_path)
    plan_doc = _load_json(plan_path)
    retime_doc = _load_json(retime_report_path)
    check_doc = _load_json(check_report_path)

    traj_errors = validate_trajectory_schema(traj_doc)
    if traj_errors:
        raise PreviewRefusal(
            "trajectory does not parse as crackvision.joint_trajectory/1: " + "; ".join(traj_errors)
        )

    plan_schema = plan_doc.get("schema") if isinstance(plan_doc, dict) else None
    if plan_schema == "crackvision.view_plan/1":
        plan_errors = validate_view_plan_schema(plan_doc)
    else:
        plan_errors = validate_cartesian_plan_schema(plan_doc)
    if plan_errors:
        raise PreviewRefusal(
            "plan does not parse as crackvision.cartesian_plan/1 or crackvision.view_plan/1: "
            + "; ".join(plan_errors)
        )

    retime_errors = _validate_retime_report_schema(retime_doc)
    if retime_errors:
        raise PreviewRefusal(
            "retime report does not parse as crackvision.retime_report/1: " + "; ".join(retime_errors)
        )

    check_errors = _validate_check_report_schema(check_doc)
    if check_errors:
        raise PreviewRefusal(
            "check report does not parse as crackvision.trajectory_check/1: " + "; ".join(check_errors)
        )

    trajectory_sha256 = file_sha256(trajectory_path)
    if retime_doc["trajectory_sha256"] != trajectory_sha256:
        raise PreviewRefusal(
            f"retime report trajectory_sha256 {retime_doc['trajectory_sha256']!r} != "
            f"current trajectory sha256 {trajectory_sha256!r}"
        )
    if check_doc["trajectory_sha256"] != trajectory_sha256:
        raise PreviewRefusal(
            f"check report trajectory_sha256 {check_doc['trajectory_sha256']!r} != "
            f"current trajectory sha256 {trajectory_sha256!r}"
        )

    failed_checks = [c["check"] for c in check_doc["checks"] if c.get("outcome") == "fail"]
    all_pass = not failed_checks

    return {
        "trajectory": {
            "path": _repo_relative(trajectory_path, root),
            "sha256": trajectory_sha256,
            "doc": traj_doc,
        },
        "plan": {
            "path": _repo_relative(plan_path, root),
            "sha256": file_sha256(plan_path),
            "schema": plan_schema,
            "doc": plan_doc,
        },
        "retime_report": {
            "path": _repo_relative(retime_report_path, root),
            "sha256": file_sha256(retime_report_path),
            "doc": retime_doc,
        },
        "check_report": {
            "path": _repo_relative(check_report_path, root),
            "sha256": file_sha256(check_report_path),
            "all_pass": all_pass,
            "failed_checks": failed_checks,
            "doc": check_doc,
        },
    }


# ---------------------------------------------------------------------------
# Tool-tip FK path (§18.1, same composition as `trajectory_check._tool_tip_positions`)
# ---------------------------------------------------------------------------


def tool_tip_path(traj: dict, chain: KinematicChain, end_effector: EndEffector) -> np.ndarray:
    """`(N, 3)` tool-tip positions in `base_link`, one per `traj["points"]`, via
    `chain.fk(q) @ end_effector.T_gripper_link_tool_tip` -- never `crackvision_motion.tool_fk`."""
    tool_T = end_effector.T_gripper_link_tool_tip
    positions = [p["positions"] for p in traj["points"]]
    return np.array([(chain.fk(q) @ tool_T)[:3, 3] for q in positions])


# ---------------------------------------------------------------------------
# compute_summary (§18.1.2)
# ---------------------------------------------------------------------------


def compute_summary(
    traj: dict, limits: dict, tool_xyz: np.ndarray, retime_report: dict, preview_speed_scale: float
) -> dict:
    """§18.1.2 `summary`: per-joint FD velocity/acceleration against `b601_dm_limits.yaml`, and the
    tool-tip path/speed metrics, scaled per §18.1.2/§18.1.2.2's `duration_s / s` and `speed * s`
    identities. `preview_speed_scale` is carried in the returned dict (`build_manifest`'s own job
    to split it back out to the manifest's top-level field, §18.1.1)."""
    points = traj["points"]
    n = len(points)
    joint_names = traj["joint_names"]
    t = [float(p["t_s"]) for p in points]
    duration_s = t[-1] - t[0]
    duration_scaled_s = duration_s / preview_speed_scale

    joints_cfg = limits["joints"]
    per_joint = []
    for j, name in enumerate(joint_names):
        jcfg = joints_cfg[name]
        lower = float(jcfg["lower"])
        upper = float(jcfg["upper"])
        v_limit = float(jcfg["velocity"])
        a_limit = float(jcfg["acceleration"])
        q = [float(p["positions"][j]) for p in points]

        min_headroom_rad = min(min(qi - lower, upper - qi) for qi in q)

        v = [(q[i + 1] - q[i]) / (t[i + 1] - t[i]) for i in range(n - 1)]
        max_abs_v = max((abs(x) for x in v), default=0.0)
        v_ratio = max_abs_v / v_limit if v_limit > 0 else float("inf")

        a = []
        for i in range(1, n - 1):
            denom = ((t[i] - t[i - 1]) + (t[i + 1] - t[i])) / 2.0
            a.append((v[i] - v[i - 1]) / denom)
        max_abs_a = max((abs(x) for x in a), default=0.0)
        a_ratio = max_abs_a / a_limit if a_limit > 0 else float("inf")

        per_joint.append(
            {
                "joint": name,
                "min_q": min(q),
                "max_q": max(q),
                "lower": lower,
                "upper": upper,
                "min_headroom_rad": min_headroom_rad,
                "max_abs_v": max_abs_v,
                "v_limit": v_limit,
                "v_ratio": v_ratio,
                "max_abs_a": max_abs_a,
                "a_limit": a_limit,
                "a_ratio": a_ratio,
            }
        )

    seg_len = [float(np.linalg.norm(tool_xyz[i + 1] - tool_xyz[i])) for i in range(n - 1)]
    cumlen = [0.0]
    for length in seg_len:
        cumlen.append(cumlen[-1] + length)
    path_length_m = cumlen[-1] if cumlen else 0.0

    speeds = [seg_len[i] / (t[i + 1] - t[i]) if (t[i + 1] - t[i]) > 0 else float("inf") for i in range(n - 1)]
    max_speed_m_s = max(speeds, default=0.0)
    max_speed_scaled_m_s = max_speed_m_s * preview_speed_scale
    min_z_m = float(min(tool_xyz[:, 2])) if n else 0.0

    legs = []
    for leg in retime_report.get("legs") or []:
        first, last = leg["point_range"]
        length_m = cumlen[last] - cumlen[first]
        leg_duration_s = float(leg["duration_s"])
        mean_speed_m_s = length_m / leg_duration_s if leg_duration_s > 0 else 0.0
        legs.append(
            {
                "leg_index": leg["leg_index"],
                "kind": leg["kind"],
                "point_range": [first, last],
                "duration_s": leg_duration_s,
                "length_m": length_m,
                "mean_speed_m_s": mean_speed_m_s,
            }
        )

    return {
        "n_points": n,
        "duration_s": duration_s,
        "duration_scaled_s": duration_scaled_s,
        "per_joint": per_joint,
        "tool": {
            "path_length_m": path_length_m,
            "max_speed_m_s": max_speed_m_s,
            "max_speed_scaled_m_s": max_speed_scaled_m_s,
            "min_z_m": min_z_m,
            "legs": legs,
        },
        "preview_speed_scale": preview_speed_scale,
    }


# ---------------------------------------------------------------------------
# build_manifest / validate_manifest / load_manifest (§18.1.1)
# ---------------------------------------------------------------------------


def build_manifest(bound: dict, summary: dict, artifacts: list[dict], *, created_utc: str, producer: str) -> dict:
    """Assembles a `crackvision.trajectory_preview/1` manifest (§18.1.1) from `bind_inputs`'s own
    return value, `compute_summary`'s return value (`preview_speed_scale` is split back out to the
    manifest's own top-level field here) and the caller's already-rendered `artifacts[]`."""
    summary = dict(summary)
    preview_speed_scale = summary.pop("preview_speed_scale")

    traj_doc = bound["trajectory"]["doc"]
    plan_doc = bound["plan"]["doc"]
    plan_schema = bound["plan"]["schema"]
    case_id = plan_doc.get("case_id") if plan_schema == "crackvision.cartesian_plan/1" else None

    limits_sha = (traj_doc.get("limits_file") or {}).get("sha256")
    end_effector_sha = traj_doc.get("end_effector_config_sha256")
    scene_sha = traj_doc.get("scene_config_sha256")
    paths3d = (traj_doc.get("source") or {}).get("paths3d")
    paths3d_sha = paths3d.get("sha256") if isinstance(paths3d, dict) else None

    bindings = {
        "limits_file_sha256": limits_sha,
        "end_effector_config_sha256": end_effector_sha,
        "scene_config_sha256": scene_sha,
        "paths3d_sha256": paths3d_sha,
    }

    failed_checks = bound["check_report"]["failed_checks"]
    warnings: list[str] = []
    if not bound["check_report"]["all_pass"]:
        warnings.append(
            f"check report has {len(failed_checks)} failing check(s): {', '.join(failed_checks)}"
        )
    for key in ("limits_file_sha256", "end_effector_config_sha256", "scene_config_sha256", "paths3d_sha256"):
        if bindings[key] is None:
            warnings.append(f"bindings.{key} is null")

    return {
        "schema": MANIFEST_SCHEMA,
        "created_utc": created_utc,
        "producer": producer,
        "case_id": case_id,
        "trajectory": {
            "path": bound["trajectory"]["path"],
            "sha256": bound["trajectory"]["sha256"],
            "purpose": traj_doc.get("purpose"),
        },
        "plan": {
            "path": bound["plan"]["path"],
            "sha256": bound["plan"]["sha256"],
            "schema": plan_schema,
        },
        "retime_report": {
            "path": bound["retime_report"]["path"],
            "sha256": bound["retime_report"]["sha256"],
        },
        "check_report": {
            "path": bound["check_report"]["path"],
            "sha256": bound["check_report"]["sha256"],
            "all_pass": bound["check_report"]["all_pass"],
            "failed_checks": failed_checks,
        },
        "bindings": bindings,
        "preview_speed_scale": preview_speed_scale,
        "summary": summary,
        "artifacts": list(artifacts),
        "warnings": warnings,
    }


def _validate_summary(summary: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(summary, dict):
        return ["manifest.summary must be an object"]
    required = {"n_points", "duration_s", "duration_scaled_s", "per_joint", "tool"}
    extra = set(summary) - required
    if extra:
        errors.append(f"manifest.summary has unexpected key(s) {sorted(extra)!r}")
    missing = required - set(summary)
    if missing:
        errors.append(f"manifest.summary is missing key(s) {sorted(missing)!r}")
        return errors

    if not isinstance(summary["n_points"], int) or isinstance(summary["n_points"], bool):
        errors.append("manifest.summary.n_points must be an int")
    for key in ("duration_s", "duration_scaled_s"):
        if not _is_finite_number(summary[key]):
            errors.append(f"manifest.summary.{key} must be a finite number")

    per_joint = summary["per_joint"]
    joint_keys = {
        "joint", "min_q", "max_q", "lower", "upper", "min_headroom_rad",
        "max_abs_v", "v_limit", "v_ratio", "max_abs_a", "a_limit", "a_ratio",
    }
    if not isinstance(per_joint, list) or len(per_joint) != 6:
        errors.append("manifest.summary.per_joint must be a list of 6 entries")
    else:
        for idx, entry in enumerate(per_joint):
            if not isinstance(entry, dict):
                errors.append(f"manifest.summary.per_joint[{idx}] must be an object")
                continue
            extra = set(entry) - joint_keys
            if extra:
                errors.append(f"manifest.summary.per_joint[{idx}] has unexpected key(s) {sorted(extra)!r}")
            missing = joint_keys - set(entry)
            if missing:
                errors.append(f"manifest.summary.per_joint[{idx}] is missing key(s) {sorted(missing)!r}")
                continue
            if not isinstance(entry["joint"], str) or not entry["joint"]:
                errors.append(f"manifest.summary.per_joint[{idx}].joint must be a non-empty string")
            for key in joint_keys - {"joint"}:
                if not _is_finite_number(entry[key]):
                    errors.append(f"manifest.summary.per_joint[{idx}].{key} must be a finite number")

    tool = summary["tool"]
    tool_keys = {"path_length_m", "max_speed_m_s", "max_speed_scaled_m_s", "min_z_m", "legs"}
    if not isinstance(tool, dict):
        errors.append("manifest.summary.tool must be an object")
    else:
        extra = set(tool) - tool_keys
        if extra:
            errors.append(f"manifest.summary.tool has unexpected key(s) {sorted(extra)!r}")
        missing = tool_keys - set(tool)
        if missing:
            errors.append(f"manifest.summary.tool is missing key(s) {sorted(missing)!r}")
        else:
            for key in ("path_length_m", "max_speed_m_s", "max_speed_scaled_m_s", "min_z_m"):
                if not _is_finite_number(tool[key]):
                    errors.append(f"manifest.summary.tool.{key} must be a finite number")
            legs = tool["legs"]
            leg_keys = {"leg_index", "kind", "point_range", "duration_s", "length_m", "mean_speed_m_s"}
            if not isinstance(legs, list):
                errors.append("manifest.summary.tool.legs must be a list")
            else:
                for idx, leg in enumerate(legs):
                    if not isinstance(leg, dict):
                        errors.append(f"manifest.summary.tool.legs[{idx}] must be an object")
                        continue
                    extra = set(leg) - leg_keys
                    if extra:
                        errors.append(f"manifest.summary.tool.legs[{idx}] has unexpected key(s) {sorted(extra)!r}")
                    missing = leg_keys - set(leg)
                    if missing:
                        errors.append(f"manifest.summary.tool.legs[{idx}] is missing key(s) {sorted(missing)!r}")
                        continue
                    if not isinstance(leg["leg_index"], int) or isinstance(leg["leg_index"], bool):
                        errors.append(f"manifest.summary.tool.legs[{idx}].leg_index must be an int")
                    if not isinstance(leg["kind"], str) or not leg["kind"]:
                        errors.append(f"manifest.summary.tool.legs[{idx}].kind must be a non-empty string")
                    point_range = leg["point_range"]
                    if not (
                        isinstance(point_range, list)
                        and len(point_range) == 2
                        and all(isinstance(x, int) and not isinstance(x, bool) for x in point_range)
                    ):
                        errors.append(
                            f"manifest.summary.tool.legs[{idx}].point_range must be a 2-element list of ints"
                        )
                    for key in ("duration_s", "length_m", "mean_speed_m_s"):
                        if not _is_finite_number(leg[key]):
                            errors.append(f"manifest.summary.tool.legs[{idx}].{key} must be a finite number")

    return errors


def validate_manifest(doc: Any) -> list[str]:
    """Strict §18.1.1 validation: any unknown or missing key at any level, or a malformed sha256
    string, is reported. Returns a list of error strings (empty = valid)."""
    if not isinstance(doc, dict):
        return ["manifest is not a JSON object"]

    errors: list[str] = []
    top_keys = {
        "schema", "created_utc", "producer", "case_id", "trajectory", "plan",
        "retime_report", "check_report", "bindings", "preview_speed_scale",
        "summary", "artifacts", "warnings",
    }
    extra = set(doc) - top_keys
    if extra:
        errors.append(f"manifest has unexpected key(s) {sorted(extra)!r}")
    missing = top_keys - set(doc)
    if missing:
        errors.append(f"manifest is missing key(s) {sorted(missing)!r}")
        return errors

    if doc["schema"] != MANIFEST_SCHEMA:
        errors.append(f"manifest.schema is {doc['schema']!r}, expected {MANIFEST_SCHEMA!r}")
    if not isinstance(doc["created_utc"], str) or not doc["created_utc"]:
        errors.append("manifest.created_utc must be a non-empty string")
    if not isinstance(doc["producer"], str) or not doc["producer"]:
        errors.append("manifest.producer must be a non-empty string")
    case_id = doc["case_id"]
    if case_id is not None and not isinstance(case_id, str):
        errors.append("manifest.case_id must be a string or null")

    traj = doc["trajectory"]
    traj_keys = {"path", "sha256", "purpose"}
    if not isinstance(traj, dict):
        errors.append("manifest.trajectory must be an object")
    else:
        extra = set(traj) - traj_keys
        if extra:
            errors.append(f"manifest.trajectory has unexpected key(s) {sorted(extra)!r}")
        missing = traj_keys - set(traj)
        if missing:
            errors.append(f"manifest.trajectory is missing key(s) {sorted(missing)!r}")
        else:
            if not isinstance(traj["path"], str) or not traj["path"]:
                errors.append("manifest.trajectory.path must be a non-empty string")
            if not _is_sha256_hex(traj["sha256"]):
                errors.append("manifest.trajectory.sha256 must be a 64-hex string")
            if traj["purpose"] not in _PURPOSES:
                errors.append(f"manifest.trajectory.purpose is {traj['purpose']!r}, expected one of {_PURPOSES!r}")

    plan = doc["plan"]
    plan_keys = {"path", "sha256", "schema"}
    if not isinstance(plan, dict):
        errors.append("manifest.plan must be an object")
    else:
        extra = set(plan) - plan_keys
        if extra:
            errors.append(f"manifest.plan has unexpected key(s) {sorted(extra)!r}")
        missing = plan_keys - set(plan)
        if missing:
            errors.append(f"manifest.plan is missing key(s) {sorted(missing)!r}")
        else:
            if not isinstance(plan["path"], str) or not plan["path"]:
                errors.append("manifest.plan.path must be a non-empty string")
            if not _is_sha256_hex(plan["sha256"]):
                errors.append("manifest.plan.sha256 must be a 64-hex string")
            if plan["schema"] not in _PLAN_SCHEMAS:
                errors.append(f"manifest.plan.schema is {plan['schema']!r}, expected one of {_PLAN_SCHEMAS!r}")

    for name in ("retime_report", "check_report"):
        block = doc[name]
        block_keys = {"path", "sha256"} | ({"all_pass", "failed_checks"} if name == "check_report" else set())
        if not isinstance(block, dict):
            errors.append(f"manifest.{name} must be an object")
            continue
        extra = set(block) - block_keys
        if extra:
            errors.append(f"manifest.{name} has unexpected key(s) {sorted(extra)!r}")
        missing = block_keys - set(block)
        if missing:
            errors.append(f"manifest.{name} is missing key(s) {sorted(missing)!r}")
            continue
        if not isinstance(block["path"], str) or not block["path"]:
            errors.append(f"manifest.{name}.path must be a non-empty string")
        if not _is_sha256_hex(block["sha256"]):
            errors.append(f"manifest.{name}.sha256 must be a 64-hex string")
        if name == "check_report":
            if not isinstance(block["all_pass"], bool):
                errors.append("manifest.check_report.all_pass must be a bool")
            failed = block["failed_checks"]
            if not isinstance(failed, list) or not all(isinstance(x, str) for x in failed):
                errors.append("manifest.check_report.failed_checks must be a list of strings")

    bindings = doc["bindings"]
    binding_keys = {"limits_file_sha256", "end_effector_config_sha256", "scene_config_sha256", "paths3d_sha256"}
    if not isinstance(bindings, dict):
        errors.append("manifest.bindings must be an object")
    else:
        extra = set(bindings) - binding_keys
        if extra:
            errors.append(f"manifest.bindings has unexpected key(s) {sorted(extra)!r}")
        missing = binding_keys - set(bindings)
        if missing:
            errors.append(f"manifest.bindings is missing key(s) {sorted(missing)!r}")
        for key in binding_keys & set(bindings):
            value = bindings[key]
            if value is not None and not _is_sha256_hex(value):
                errors.append(f"manifest.bindings.{key} must be a 64-hex string or null")

    scale = doc["preview_speed_scale"]
    if not _is_finite_number(scale) or scale <= 0:
        errors.append("manifest.preview_speed_scale must be a positive finite number")

    errors.extend(_validate_summary(doc["summary"]))

    artifacts = doc["artifacts"]
    if not isinstance(artifacts, list) or len(artifacts) != 3:
        errors.append("manifest.artifacts must be a list of exactly 3 entries")
    else:
        for idx, (entry, expected_kind) in enumerate(zip(artifacts, _ARTIFACT_KINDS)):
            if not isinstance(entry, dict):
                errors.append(f"manifest.artifacts[{idx}] must be an object")
                continue
            entry_keys = {"kind", "path", "sha256"}
            extra = set(entry) - entry_keys
            if extra:
                errors.append(f"manifest.artifacts[{idx}] has unexpected key(s) {sorted(extra)!r}")
            missing = entry_keys - set(entry)
            if missing:
                errors.append(f"manifest.artifacts[{idx}] is missing key(s) {sorted(missing)!r}")
                continue
            if entry["kind"] != expected_kind:
                errors.append(f"manifest.artifacts[{idx}].kind is {entry['kind']!r}, expected {expected_kind!r}")
            if not isinstance(entry["path"], str) or not entry["path"]:
                errors.append(f"manifest.artifacts[{idx}].path must be a non-empty string")
            if not _is_sha256_hex(entry["sha256"]):
                errors.append(f"manifest.artifacts[{idx}].sha256 must be a 64-hex string")

    warnings = doc["warnings"]
    if not isinstance(warnings, list) or not all(isinstance(w, str) for w in warnings):
        errors.append("manifest.warnings must be a list of strings")

    return errors


def load_manifest(path: Path | str) -> dict:
    """Loads and strictly validates a `crackvision.trajectory_preview/1` manifest, raising
    `PreviewUsageError` if the file cannot be opened or parsed, `PreviewRefusal` if it parses but
    fails `validate_manifest`."""
    doc = _load_json(Path(path))
    errors = validate_manifest(doc)
    if errors:
        raise PreviewRefusal(f"invalid crackvision.trajectory_preview/1 manifest at {path}: " + "; ".join(errors))
    return doc

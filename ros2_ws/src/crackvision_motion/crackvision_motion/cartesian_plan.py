"""cartesian_plan — pure-Python assembly, validation and I/O for `crackvision.cartesian_plan/1`
and `crackvision.view_plan/1` documents (MOT-06.4).

Stdlib + PyYAML only (via `joint_trajectory.load_limits`) -- no rclpy or `*_msgs` import anywhere
in this file, mirroring `cartesian_core.py`/`joint_trajectory.py`. Normative source:
`docs/INTERFACES.md` §13.11-13.14.

This is the only module that builds or reads §13 plan/view-plan documents. It does not run the
IK/Cartesian/re-check planning loop itself (§13.5-13.9, ROS-dependent, a later MOT-06 card) --
callers build a plan incrementally with `new_plan`/`add_leg`/`add_segment_report`, then `finalize`
computes coverage and validates the whole document before `write_plan` persists it. `load_plan`
re-validates on the way back in, so MOT-07 never has to re-check what this module already proved.
"""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Union

from .joint_trajectory import JOINT_NAMES, load_limits

PLAN_SCHEMA = "crackvision.cartesian_plan/1"
VIEW_SCHEMA = "crackvision.view_plan/1"
_FRAME = "base_link"
_SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
_N_JOINTS = len(JOINT_NAMES)

_NOMINAL_DISCLAIMER = (
    "Nominal recommendation only; the operator places the specimen and MOT-10 measures and "
    "supersedes this file with the real, as-placed pose."
)
_UNTIMED_NOTE = (
    "This file is untimed geometric output, never executable directly; MOT-07 retimes it into a "
    "joint_trajectory file before execution."
)
_INELIGIBLE_NOTE = (
    "Built from an execution-ineligible paths3d file; valid for sim/visualisation only (§10.6)."
)

# continuity tolerance (§ acceptance criteria "beyond a stated tolerance"): floating-point slack,
# not a physical bound -- the previous leg's chosen last configuration and the next leg's first
# configuration must be numerically identical (same joint configuration, re-emitted).
CONTINUITY_TOL_RAD = 1e-6


class CartesianPlanError(ValueError):
    """Raised for any structurally or numerically invalid cartesian_plan/view_plan document."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# --------------------------------------------------------------------------------------
# generic field helpers
# --------------------------------------------------------------------------------------

def _check_exact_keys(d: object, required: frozenset, ctx: str) -> None:
    if not isinstance(d, dict):
        raise CartesianPlanError(f"'{ctx}' must be an object, got {d!r}")
    present = set(d.keys())
    missing = required - present
    extra = present - required
    if missing:
        raise CartesianPlanError(f"'{ctx}' is missing required key(s): {sorted(missing)}")
    if extra:
        raise CartesianPlanError(f"'{ctx}' has unknown key(s): {sorted(extra)}")


def _as_str(v: object, ctx: str, *, nonempty: bool = True) -> str:
    if not isinstance(v, str) or (nonempty and not v):
        raise CartesianPlanError(f"'{ctx}' must be a non-empty string, got {v!r}")
    return v


def _as_sha256(v: object, ctx: str) -> str:
    if not isinstance(v, str) or not _SHA256_RE.match(v):
        raise CartesianPlanError(f"'{ctx}' must be a 64-hex-char sha256 digest, got {v!r}")
    return v.lower()


def _as_bool(v: object, ctx: str) -> bool:
    if not isinstance(v, bool):
        raise CartesianPlanError(f"'{ctx}' must be a bool, got {v!r}")
    return v


def _as_finite(v: object, ctx: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise CartesianPlanError(f"'{ctx}' must be numeric, got {v!r}")
    value = float(v)
    if not math.isfinite(value):
        raise CartesianPlanError(f"'{ctx}' must be finite, got {v!r}")
    return value


def _as_nonneg_int(v: object, ctx: str) -> int:
    if isinstance(v, bool) or not isinstance(v, int) or v < 0:
        raise CartesianPlanError(f"'{ctx}' must be a non-negative int, got {v!r}")
    return v


def _as_finite_vec(v: object, ctx: str, n: int) -> List[float]:
    if not isinstance(v, list) or len(v) != n:
        raise CartesianPlanError(f"'{ctx}' must be a list of length {n}, got {v!r}")
    return [_as_finite(x, f"{ctx}[{i}]") for i, x in enumerate(v)]


def _file_source(v: object, ctx: str) -> Dict[str, str]:
    _check_exact_keys(v, frozenset({"path", "sha256"}), ctx)
    return {"path": _as_str(v["path"], f"{ctx}.path"), "sha256": _as_sha256(v["sha256"], f"{ctx}.sha256")}


def _paths3d_source(v: object, ctx: str) -> Dict[str, Any]:
    """The exact §11.2 `source.paths3d` shape: `{path, sha256, execution_eligible}`."""
    _check_exact_keys(v, frozenset({"path", "sha256", "execution_eligible"}), ctx)
    eligible = v["execution_eligible"]
    if eligible is not None and not isinstance(eligible, bool):
        raise CartesianPlanError(f"'{ctx}.execution_eligible' must be a bool or null, got {eligible!r}")
    return {
        "path": _as_str(v["path"], f"{ctx}.path"),
        "sha256": _as_sha256(v["sha256"], f"{ctx}.sha256"),
        "execution_eligible": eligible,
    }


# --------------------------------------------------------------------------------------
# caveats (§13.13)
# --------------------------------------------------------------------------------------

def build_caveats(task_frame_note: str, execution_eligible: Optional[bool]) -> List[str]:
    """§13.13: the §7.3 nominal disclaimer verbatim first, then the task frame/collision model
    note, then the untimed-output note, then (only when `execution_eligible is False`) the
    ineligible-paths3d note."""
    caveats = [_NOMINAL_DISCLAIMER, _as_str(task_frame_note, "task_frame_note"), _UNTIMED_NOTE]
    if execution_eligible is False:
        caveats.append(_INELIGIBLE_NOTE)
    return caveats


def _validate_caveats(v: object, ctx: str) -> List[str]:
    if not isinstance(v, list) or not v:
        raise CartesianPlanError(f"'{ctx}' must be a non-empty list")
    for i, c in enumerate(v):
        _as_str(c, f"{ctx}[{i}]")
    if v[0] != _NOMINAL_DISCLAIMER:
        raise CartesianPlanError(f"'{ctx}[0]' must be the §13.13 nominal disclaimer verbatim, got {v[0]!r}")
    return list(v)


# --------------------------------------------------------------------------------------
# legs (§13.11.2) -- shared by cartesian_plan.legs[] and view_plan.transit_leg
# --------------------------------------------------------------------------------------

_LEG_KEYS = frozenset({
    "leg_index", "kind", "segment_ref", "direction", "roll_offset_rad", "positions",
    "waypoint_index_map", "recheck",
})
_LEG_KINDS = frozenset({"transit", "approach", "trace", "retract"})
_LEG_RECHECK_KEYS = frozenset({
    "max_pos_err_m", "max_ang_err_rad", "all_valid", "within_limits", "max_joint_step_rad",
})
_SEGMENT_REF_KEYS = frozenset({"component_id", "polyline_kind", "branch_index", "segment_index", "sub_segment"})
_DIRECTIONS = frozenset({"forward", "reverse"})


def _validate_segment_ref(v: object, ctx: str) -> Dict[str, Any]:
    _check_exact_keys(v, _SEGMENT_REF_KEYS, ctx)
    component_id = _as_str(v["component_id"], f"{ctx}.component_id")
    polyline_kind = v["polyline_kind"]
    if polyline_kind not in ("main", "branch"):
        raise CartesianPlanError(f"'{ctx}.polyline_kind' must be 'main' or 'branch', got {polyline_kind!r}")
    branch_index = v["branch_index"]
    if polyline_kind == "main":
        if branch_index is not None:
            raise CartesianPlanError(f"'{ctx}.branch_index' must be null for polyline_kind 'main'")
    else:
        if isinstance(branch_index, bool) or not isinstance(branch_index, int) or branch_index < 0:
            raise CartesianPlanError(f"'{ctx}.branch_index' must be a non-negative int for 'branch'")
    segment_index = _as_nonneg_int(v["segment_index"], f"{ctx}.segment_index")
    sub_segment = v["sub_segment"]
    if sub_segment is not None:
        sub_segment = _as_nonneg_int(sub_segment, f"{ctx}.sub_segment")
    return {
        "component_id": component_id, "polyline_kind": polyline_kind, "branch_index": branch_index,
        "segment_index": segment_index, "sub_segment": sub_segment,
    }


def _ref_key(ref: Dict[str, Any]):
    return (ref["component_id"], ref["polyline_kind"], ref["branch_index"], ref["segment_index"], ref["sub_segment"])


def _new_leg(
    leg_index: int, kind: str, segment_ref: Optional[Dict[str, Any]], direction: Optional[str],
    roll_offset_rad: Optional[float], positions: Sequence[Sequence[float]],
    waypoint_index_map: Sequence[Optional[int]], recheck: Dict[str, Any],
) -> Dict[str, Any]:
    if kind not in _LEG_KINDS:
        raise CartesianPlanError(f"leg 'kind' must be one of {sorted(_LEG_KINDS)}, got {kind!r}")
    if (segment_ref is None) != (direction is None) or (direction is None) != (roll_offset_rad is None):
        raise CartesianPlanError(
            "leg 'segment_ref'/'direction'/'roll_offset_rad' must be null together (trailing "
            "end_state/view transit leg) or non-null together"
        )
    positions_out = [_as_finite_vec(p, f"leg.positions[{i}]", _N_JOINTS) for i, p in enumerate(positions)]
    if not positions_out:
        raise CartesianPlanError("leg 'positions' must be non-empty")
    if len(waypoint_index_map) != len(positions_out):
        raise CartesianPlanError("leg 'waypoint_index_map' must have the same length as 'positions'")
    wmap_out: List[Optional[int]] = []
    for i, w in enumerate(waypoint_index_map):
        wmap_out.append(None if w is None else _as_nonneg_int(w, f"leg.waypoint_index_map[{i}]"))
    _check_exact_keys(recheck, _LEG_RECHECK_KEYS, "leg.recheck")
    recheck_out = {
        "max_pos_err_m": _as_finite(recheck["max_pos_err_m"], "leg.recheck.max_pos_err_m"),
        "max_ang_err_rad": _as_finite(recheck["max_ang_err_rad"], "leg.recheck.max_ang_err_rad"),
        "all_valid": _as_bool(recheck["all_valid"], "leg.recheck.all_valid"),
        "within_limits": _as_bool(recheck["within_limits"], "leg.recheck.within_limits"),
        "max_joint_step_rad": _as_finite(recheck["max_joint_step_rad"], "leg.recheck.max_joint_step_rad"),
    }
    if recheck_out["max_pos_err_m"] < 0 or recheck_out["max_ang_err_rad"] < 0 or recheck_out["max_joint_step_rad"] < 0:
        raise CartesianPlanError("leg 'recheck' numeric fields must be >= 0")

    if direction is not None and direction not in _DIRECTIONS:
        raise CartesianPlanError(f"leg 'direction' must be one of {sorted(_DIRECTIONS)}, got {direction!r}")

    return {
        "leg_index": leg_index,
        "kind": kind,
        "segment_ref": None if segment_ref is None else _validate_segment_ref(segment_ref, "leg.segment_ref"),
        "direction": direction,
        "roll_offset_rad": None if roll_offset_rad is None else _as_finite(roll_offset_rad, "leg.roll_offset_rad"),
        "positions": positions_out,
        "waypoint_index_map": wmap_out,
        "recheck": recheck_out,
    }


def _validate_leg_doc(v: object, ctx: str, *, limits: Dict[str, Dict[str, float]]) -> Dict[str, Any]:
    _check_exact_keys(v, _LEG_KEYS, ctx)
    leg_index = _as_nonneg_int(v["leg_index"], f"{ctx}.leg_index")
    kind = v["kind"]
    if kind not in _LEG_KINDS:
        raise CartesianPlanError(f"'{ctx}.kind' must be one of {sorted(_LEG_KINDS)}, got {kind!r}")
    segment_ref_raw, direction, roll_offset_rad = v["segment_ref"], v["direction"], v["roll_offset_rad"]
    if (segment_ref_raw is None) != (direction is None) or (direction is None) != (roll_offset_rad is None):
        raise CartesianPlanError(
            f"'{ctx}' segment_ref/direction/roll_offset_rad must be null together or non-null together"
        )
    segment_ref = None if segment_ref_raw is None else _validate_segment_ref(segment_ref_raw, f"{ctx}.segment_ref")
    if direction is not None and direction not in _DIRECTIONS:
        raise CartesianPlanError(f"'{ctx}.direction' must be one of {sorted(_DIRECTIONS)}, got {direction!r}")
    if roll_offset_rad is not None:
        roll_offset_rad = _as_finite(roll_offset_rad, f"{ctx}.roll_offset_rad")

    positions_raw = v["positions"]
    if not isinstance(positions_raw, list) or not positions_raw:
        raise CartesianPlanError(f"'{ctx}.positions' must be a non-empty list")
    positions = [_as_finite_vec(p, f"{ctx}.positions[{i}]", _N_JOINTS) for i, p in enumerate(positions_raw)]
    for pi, pos in enumerate(positions):
        for jidx, jname in enumerate(JOINT_NAMES):
            q = pos[jidx]
            spec = limits[jname]
            if q < spec["lower"] or q > spec["upper"]:
                raise CartesianPlanError(
                    f"'{ctx}.positions[{pi}]' joint '{jname}' value {q} outside inclusive bound "
                    f"[{spec['lower']}, {spec['upper']}]"
                )

    wmap_raw = v["waypoint_index_map"]
    if not isinstance(wmap_raw, list) or len(wmap_raw) != len(positions):
        raise CartesianPlanError(f"'{ctx}.waypoint_index_map' must be a list the same length as 'positions'")
    waypoint_index_map = [
        None if w is None else _as_nonneg_int(w, f"{ctx}.waypoint_index_map[{i}]") for i, w in enumerate(wmap_raw)
    ]

    recheck_raw = v["recheck"]
    _check_exact_keys(recheck_raw, _LEG_RECHECK_KEYS, f"{ctx}.recheck")
    recheck = {
        "max_pos_err_m": _as_finite(recheck_raw["max_pos_err_m"], f"{ctx}.recheck.max_pos_err_m"),
        "max_ang_err_rad": _as_finite(recheck_raw["max_ang_err_rad"], f"{ctx}.recheck.max_ang_err_rad"),
        "all_valid": _as_bool(recheck_raw["all_valid"], f"{ctx}.recheck.all_valid"),
        "within_limits": _as_bool(recheck_raw["within_limits"], f"{ctx}.recheck.within_limits"),
        "max_joint_step_rad": _as_finite(recheck_raw["max_joint_step_rad"], f"{ctx}.recheck.max_joint_step_rad"),
    }

    return {
        "leg_index": leg_index, "kind": kind, "segment_ref": segment_ref, "direction": direction,
        "roll_offset_rad": roll_offset_rad, "positions": positions, "waypoint_index_map": waypoint_index_map,
        "recheck": recheck,
    }


def _check_leg_continuity(legs: Sequence[Dict[str, Any]], ctx: str) -> None:
    for i in range(1, len(legs)):
        prev_last = legs[i - 1]["positions"][-1]
        cur_first = legs[i]["positions"][0]
        max_diff = max(abs(a - b) for a, b in zip(prev_last, cur_first))
        if max_diff > CONTINUITY_TOL_RAD:
            raise CartesianPlanError(
                f"'{ctx}[{i}]' is not continuous with the previous leg: max joint difference "
                f"{max_diff} rad > tolerance {CONTINUITY_TOL_RAD} rad"
            )


# --------------------------------------------------------------------------------------
# cartesian_plan: incremental builder (§13.11)
# --------------------------------------------------------------------------------------

_PLAN_TOP_KEYS = frozenset({
    "schema", "case_id", "created_utc", "producer", "planning_frame", "joint_names", "sources",
    "parameters", "start_state", "end_state", "legs", "segments", "coverage", "caveats",
})
_PLAN_SOURCES_KEYS = frozenset({"paths3d", "scene", "end_effector", "limits", "config"})
_START_STATE_KEYS = frozenset({"joint_names", "positions_rad", "source"})
_START_STATE_SOURCES = frozenset({"capture_record", "start_joints_override"})
_END_STATE_KEYS = frozenset({"mode", "positions_rad"})
_END_STATE_MODES = frozenset({"start", "home", "none"})

_SEGMENT_REPORT_KEYS = frozenset({
    "segment_ref", "status", "reasons", "candidates_tried", "chosen", "trace_waypoints", "trace_length_m",
})
_SEGMENT_STATUSES = frozenset({"full", "partial", "failed", "surface_scene_mismatch"})
_CHOSEN_KEYS = frozenset({"direction", "roll_offset_rad"})
_COUNT_PAIR_KEYS = frozenset({"total", "planned"})

_COVERAGE_KEYS = frozenset({
    "trace_waypoints_total", "trace_waypoints_planned", "length_m_total", "length_m_planned",
    "fraction_by_length", "complete", "upstream",
})
_UPSTREAM_KEYS = frozenset({"points", "valid", "invalid_by_reason"})


def new_plan(
    case_id: str, sources: Dict[str, Any], parameters: Dict[str, Any], start_state: Dict[str, Any],
    end_state: Dict[str, Any],
) -> Dict[str, Any]:
    """A fresh, empty `crackvision.cartesian_plan/1` document (§13.11.1) ready for `add_leg`/
    `add_segment_report`. `sources`/`parameters`/`start_state`/`end_state` are the caller-resolved
    §13.1/§13.2 values; shape is checked by `finalize`/`validate_plan`, not here, so a plan under
    construction can be built up field by field."""
    return {
        "schema": PLAN_SCHEMA,
        "case_id": case_id,
        "created_utc": _utc_now(),
        "producer": "plan_crack_path",
        "planning_frame": _FRAME,
        "joint_names": list(JOINT_NAMES),
        "sources": sources,
        "parameters": parameters,
        "start_state": start_state,
        "end_state": end_state,
        "legs": [],
        "segments": [],
        "coverage": {},
        "caveats": [],
    }


def add_leg(
    plan: Dict[str, Any], kind: str, segment_ref: Optional[Dict[str, Any]], direction: Optional[str],
    roll_offset_rad: Optional[float], positions: Sequence[Sequence[float]],
    waypoint_index_map: Sequence[Optional[int]], recheck: Dict[str, Any],
) -> Dict[str, Any]:
    """Append one §13.11.2 leg to `plan["legs"]` (`leg_index` auto-assigned) and return it."""
    leg = _new_leg(len(plan["legs"]), kind, segment_ref, direction, roll_offset_rad, positions, waypoint_index_map, recheck)
    plan["legs"].append(leg)
    return leg


def build_leg(
    kind: str, segment_ref: Optional[Dict[str, Any]], direction: Optional[str],
    roll_offset_rad: Optional[float], positions: Sequence[Sequence[float]],
    waypoint_index_map: Sequence[Optional[int]], recheck: Dict[str, Any],
) -> Dict[str, Any]:
    """Build one standalone §13.11.2-shaped leg (`leg_index: 0`), for `view_plan.transit_leg`
    (§13.14.1), which is not part of a `legs[]` list."""
    return _new_leg(0, kind, segment_ref, direction, roll_offset_rad, positions, waypoint_index_map, recheck)


def add_segment_report(
    plan: Dict[str, Any], segment_ref: Dict[str, Any], reasons: Sequence[str], candidates_tried: int,
    chosen: Optional[Dict[str, Any]], trace_waypoints_total: int, trace_waypoints_planned: int,
    trace_length_total: float, trace_length_planned: float, *, mismatch: bool = False,
) -> Dict[str, Any]:
    """Append one §13.11.3 `segments[]` entry, deriving `status` per §13.11.3's own table from
    `mismatch` (§13.9, never attempted) and the waypoint counts. Returns the appended entry."""
    segment_ref = _validate_segment_ref(segment_ref, "segment_ref")
    reasons_list = [_as_str(r, "reasons[]") for r in reasons]
    candidates_tried = _as_nonneg_int(candidates_tried, "candidates_tried")
    trace_waypoints_total = _as_nonneg_int(trace_waypoints_total, "trace_waypoints_total")
    trace_waypoints_planned = _as_nonneg_int(trace_waypoints_planned, "trace_waypoints_planned")
    if trace_waypoints_planned > trace_waypoints_total:
        raise CartesianPlanError("trace_waypoints_planned must be <= trace_waypoints_total")
    trace_length_total = _as_finite(trace_length_total, "trace_length_total")
    trace_length_planned = _as_finite(trace_length_planned, "trace_length_planned")
    if trace_length_total < 0 or trace_length_planned < 0:
        raise CartesianPlanError("trace_length_total/planned must be >= 0")
    if trace_length_planned > trace_length_total + 1e-9:
        raise CartesianPlanError("trace_length_planned must be <= trace_length_total")

    chosen_out: Optional[Dict[str, Any]] = None
    if chosen is not None:
        _check_exact_keys(chosen, _CHOSEN_KEYS, "chosen")
        if chosen["direction"] not in _DIRECTIONS:
            raise CartesianPlanError(f"'chosen.direction' must be one of {sorted(_DIRECTIONS)}")
        chosen_out = {
            "direction": chosen["direction"],
            "roll_offset_rad": _as_finite(chosen["roll_offset_rad"], "chosen.roll_offset_rad"),
        }

    if mismatch:
        if candidates_tried != 0 or chosen_out is not None or trace_waypoints_planned != 0:
            raise CartesianPlanError(
                "surface_scene_mismatch requires candidates_tried == 0, chosen == null, planned == 0"
            )
        status = "surface_scene_mismatch"
    elif trace_waypoints_planned == trace_waypoints_total:
        if chosen_out is None:
            raise CartesianPlanError("status 'full' requires chosen != null")
        status = "full"
    elif trace_waypoints_planned == 0:
        if candidates_tried == 0:
            raise CartesianPlanError("status 'failed' requires candidates_tried > 0 (else it is a mismatch)")
        if chosen_out is not None:
            raise CartesianPlanError("status 'failed' requires chosen == null")
        if not reasons_list:
            raise CartesianPlanError("status 'failed' requires at least one reason")
        status = "failed"
    else:
        if chosen_out is None:
            raise CartesianPlanError("status 'partial' requires chosen != null")
        status = "partial"

    entry = {
        "segment_ref": segment_ref,
        "status": status,
        "reasons": reasons_list,
        "candidates_tried": candidates_tried,
        "chosen": chosen_out,
        "trace_waypoints": {"total": trace_waypoints_total, "planned": trace_waypoints_planned},
        "trace_length_m": {"total": trace_length_total, "planned": trace_length_planned},
    }
    plan["segments"].append(entry)
    return entry


def _compute_coverage(segments: Sequence[Dict[str, Any]], upstream_counts: Dict[str, Any]) -> Dict[str, Any]:
    originals = [s for s in segments if s["segment_ref"]["sub_segment"] is None]
    trace_waypoints_total = sum(s["trace_waypoints"]["total"] for s in originals)
    trace_waypoints_planned = sum(s["trace_waypoints"]["planned"] for s in segments)
    length_m_total = sum(s["trace_length_m"]["total"] for s in originals)
    length_m_planned = sum(s["trace_length_m"]["planned"] for s in segments)
    fraction_by_length = 1.0 if length_m_total == 0 else length_m_planned / length_m_total
    complete = trace_waypoints_planned == trace_waypoints_total
    return {
        "trace_waypoints_total": trace_waypoints_total,
        "trace_waypoints_planned": trace_waypoints_planned,
        "length_m_total": length_m_total,
        "length_m_planned": length_m_planned,
        "fraction_by_length": fraction_by_length,
        "complete": complete,
        "upstream": {
            "points": upstream_counts["points"],
            "valid": upstream_counts["valid"],
            "invalid_by_reason": dict(upstream_counts["invalid_by_reason"]),
        },
    }


def finalize(
    plan: Dict[str, Any], paths3d_doc: Dict[str, Any], segment_tasks: Sequence[Any],
    limits: Dict[str, Dict[str, float]], *, task_frame_note: str = "tool_tip; production scene (§13.1)",
) -> Dict[str, Any]:
    """Compute `coverage` (§13.12) from `plan["segments"]` and `paths3d_doc.counts`, check that
    every original `segment_tasks[].segment_ref` is present in `segments[]`, build `caveats[]`
    (§13.13) and validate the whole document (raises `CartesianPlanError`). Mutates and returns
    `plan`."""
    original_refs = {_ref_key(task.segment_ref) for task in segment_tasks}
    present_refs = {
        _ref_key(s["segment_ref"]) for s in plan["segments"] if s["segment_ref"]["sub_segment"] is None
    }
    missing = original_refs - present_refs
    if missing:
        raise CartesianPlanError(f"segments[] is missing original segment(s): {sorted(missing)}")

    plan["coverage"] = _compute_coverage(plan["segments"], paths3d_doc["counts"])

    doc_eligible = paths3d_doc["execution_eligible"]
    if not isinstance(doc_eligible, bool):
        raise CartesianPlanError(f"paths3d_doc 'execution_eligible' must be a bool, got {doc_eligible!r}")
    sources_paths3d = plan["sources"]["paths3d"]
    existing_eligible = sources_paths3d.get("execution_eligible")
    if existing_eligible is not None and existing_eligible != doc_eligible:
        raise CartesianPlanError(
            f"'sources.paths3d.execution_eligible' ({existing_eligible!r}) does not match the "
            f"paths3d file's 'execution_eligible' ({doc_eligible!r})"
        )
    sources_paths3d["execution_eligible"] = doc_eligible
    plan["caveats"] = build_caveats(task_frame_note, doc_eligible)

    validate_plan(plan, limits)
    return plan


def _validate_segment_report(v: object, ctx: str) -> Dict[str, Any]:
    _check_exact_keys(v, _SEGMENT_REPORT_KEYS, ctx)
    segment_ref = _validate_segment_ref(v["segment_ref"], f"{ctx}.segment_ref")
    status = v["status"]
    if status not in _SEGMENT_STATUSES:
        raise CartesianPlanError(f"'{ctx}.status' must be one of {sorted(_SEGMENT_STATUSES)}, got {status!r}")
    reasons_raw = v["reasons"]
    if not isinstance(reasons_raw, list):
        raise CartesianPlanError(f"'{ctx}.reasons' must be a list")
    reasons = [_as_str(r, f"{ctx}.reasons[{i}]") for i, r in enumerate(reasons_raw)]
    candidates_tried = _as_nonneg_int(v["candidates_tried"], f"{ctx}.candidates_tried")

    chosen_raw = v["chosen"]
    chosen = None
    if chosen_raw is not None:
        _check_exact_keys(chosen_raw, _CHOSEN_KEYS, f"{ctx}.chosen")
        if chosen_raw["direction"] not in _DIRECTIONS:
            raise CartesianPlanError(f"'{ctx}.chosen.direction' must be one of {sorted(_DIRECTIONS)}")
        chosen = {
            "direction": chosen_raw["direction"],
            "roll_offset_rad": _as_finite(chosen_raw["roll_offset_rad"], f"{ctx}.chosen.roll_offset_rad"),
        }

    tw_raw = v["trace_waypoints"]
    _check_exact_keys(tw_raw, _COUNT_PAIR_KEYS, f"{ctx}.trace_waypoints")
    tw_total = _as_nonneg_int(tw_raw["total"], f"{ctx}.trace_waypoints.total")
    tw_planned = _as_nonneg_int(tw_raw["planned"], f"{ctx}.trace_waypoints.planned")
    if tw_planned > tw_total:
        raise CartesianPlanError(f"'{ctx}.trace_waypoints.planned' must be <= 'total'")

    tl_raw = v["trace_length_m"]
    _check_exact_keys(tl_raw, _COUNT_PAIR_KEYS, f"{ctx}.trace_length_m")
    tl_total = _as_finite(tl_raw["total"], f"{ctx}.trace_length_m.total")
    tl_planned = _as_finite(tl_raw["planned"], f"{ctx}.trace_length_m.planned")
    if tl_total < 0 or tl_planned < 0 or tl_planned > tl_total + 1e-9:
        raise CartesianPlanError(f"'{ctx}.trace_length_m' must have 0 <= planned <= total")

    if status == "surface_scene_mismatch":
        if candidates_tried != 0 or chosen is not None or tw_planned != 0:
            raise CartesianPlanError(f"'{ctx}' status 'surface_scene_mismatch' requires candidates_tried == 0, chosen == null, planned == 0")
    elif status == "full":
        if tw_planned != tw_total:
            raise CartesianPlanError(f"'{ctx}' status 'full' requires trace_waypoints.planned == total")
        if chosen is None:
            raise CartesianPlanError(f"'{ctx}' status 'full' requires chosen != null")
    elif status == "failed":
        if tw_planned != 0 or candidates_tried == 0:
            raise CartesianPlanError(f"'{ctx}' status 'failed' requires planned == 0 and candidates_tried > 0")
        if chosen is not None:
            raise CartesianPlanError(f"'{ctx}' status 'failed' requires chosen == null")
        if not reasons:
            raise CartesianPlanError(f"'{ctx}' status 'failed' requires at least one reason")
    else:  # partial
        if not (0 < tw_planned < tw_total):
            raise CartesianPlanError(f"'{ctx}' status 'partial' requires 0 < planned < total")
        if chosen is None:
            raise CartesianPlanError(f"'{ctx}' status 'partial' requires chosen != null")

    return {
        "segment_ref": segment_ref, "status": status, "reasons": reasons, "candidates_tried": candidates_tried,
        "chosen": chosen, "trace_waypoints": {"total": tw_total, "planned": tw_planned},
        "trace_length_m": {"total": tl_total, "planned": tl_planned},
    }


def _validate_coverage(v: object, segments: Sequence[Dict[str, Any]], ctx: str) -> Dict[str, Any]:
    _check_exact_keys(v, _COVERAGE_KEYS, ctx)
    tw_total = _as_nonneg_int(v["trace_waypoints_total"], f"{ctx}.trace_waypoints_total")
    tw_planned = _as_nonneg_int(v["trace_waypoints_planned"], f"{ctx}.trace_waypoints_planned")
    length_total = _as_finite(v["length_m_total"], f"{ctx}.length_m_total")
    length_planned = _as_finite(v["length_m_planned"], f"{ctx}.length_m_planned")
    fraction_by_length = _as_finite(v["fraction_by_length"], f"{ctx}.fraction_by_length")
    complete = _as_bool(v["complete"], f"{ctx}.complete")
    upstream_raw = v["upstream"]
    _check_exact_keys(upstream_raw, _UPSTREAM_KEYS, f"{ctx}.upstream")
    points = _as_nonneg_int(upstream_raw["points"], f"{ctx}.upstream.points")
    valid = _as_nonneg_int(upstream_raw["valid"], f"{ctx}.upstream.valid")
    invalid_by_reason_raw = upstream_raw["invalid_by_reason"]
    if not isinstance(invalid_by_reason_raw, dict):
        raise CartesianPlanError(f"'{ctx}.upstream.invalid_by_reason' must be an object")
    invalid_by_reason = {
        _as_str(k, f"{ctx}.upstream.invalid_by_reason key"): _as_nonneg_int(n, f"{ctx}.upstream.invalid_by_reason[{k!r}]")
        for k, n in invalid_by_reason_raw.items()
    }

    recomputed = _compute_coverage(segments, {"points": points, "valid": valid, "invalid_by_reason": invalid_by_reason})
    if tw_total != recomputed["trace_waypoints_total"]:
        raise CartesianPlanError(f"'{ctx}.trace_waypoints_total' inconsistent with segments[]")
    if tw_planned != recomputed["trace_waypoints_planned"]:
        raise CartesianPlanError(f"'{ctx}.trace_waypoints_planned' inconsistent with segments[]")
    if abs(length_total - recomputed["length_m_total"]) > 1e-9:
        raise CartesianPlanError(f"'{ctx}.length_m_total' inconsistent with segments[]")
    if abs(length_planned - recomputed["length_m_planned"]) > 1e-9:
        raise CartesianPlanError(f"'{ctx}.length_m_planned' inconsistent with segments[]")
    if abs(fraction_by_length - recomputed["fraction_by_length"]) > 1e-9:
        raise CartesianPlanError(f"'{ctx}.fraction_by_length' inconsistent with segments[]")
    if complete != recomputed["complete"]:
        raise CartesianPlanError(f"'{ctx}.complete' inconsistent with segments[]")

    return {
        "trace_waypoints_total": tw_total, "trace_waypoints_planned": tw_planned,
        "length_m_total": length_total, "length_m_planned": length_planned,
        "fraction_by_length": fraction_by_length, "complete": complete,
        "upstream": {"points": points, "valid": valid, "invalid_by_reason": invalid_by_reason},
    }


def validate_plan(doc: object, limits: Dict[str, Dict[str, float]]) -> Dict[str, Any]:
    """Strictly validate a `crackvision.cartesian_plan/1` document per §13.11-13.13. Raises
    `CartesianPlanError` on any missing/extra field, wrong `joint_names`, non-finite position,
    out-of-limits position, broken leg continuity, or coverage inconsistent with `segments[]`."""
    _check_exact_keys(doc, _PLAN_TOP_KEYS, "<root>")
    if doc["schema"] != PLAN_SCHEMA:
        raise CartesianPlanError(f"'schema' must be {PLAN_SCHEMA!r}, got {doc['schema']!r}")
    case_id = _as_str(doc["case_id"], "case_id")
    created_utc = _as_str(doc["created_utc"], "created_utc")
    producer = _as_str(doc["producer"], "producer")
    if doc["planning_frame"] != _FRAME:
        raise CartesianPlanError(f"'planning_frame' must be {_FRAME!r}, got {doc['planning_frame']!r}")
    joint_names = doc["joint_names"]
    if joint_names != list(JOINT_NAMES):
        raise CartesianPlanError(f"'joint_names' must be exactly {list(JOINT_NAMES)}, got {joint_names!r}")

    sources_raw = doc["sources"]
    _check_exact_keys(sources_raw, _PLAN_SOURCES_KEYS, "sources")
    sources = {
        "paths3d": _paths3d_source(sources_raw["paths3d"], "sources.paths3d"),
        "scene": _file_source(sources_raw["scene"], "sources.scene"),
        "end_effector": _file_source(sources_raw["end_effector"], "sources.end_effector"),
        "limits": _file_source(sources_raw["limits"], "sources.limits"),
        "config": _file_source(sources_raw["config"], "sources.config"),
    }

    parameters = doc["parameters"]
    if not isinstance(parameters, dict) or not parameters:
        raise CartesianPlanError("'parameters' must be a non-empty object")

    start_state_raw = doc["start_state"]
    _check_exact_keys(start_state_raw, _START_STATE_KEYS, "start_state")
    if start_state_raw["joint_names"] != list(JOINT_NAMES):
        raise CartesianPlanError("'start_state.joint_names' must be exactly the §11.2 joint_names")
    start_positions = _as_finite_vec(start_state_raw["positions_rad"], "start_state.positions_rad", _N_JOINTS)
    if start_state_raw["source"] not in _START_STATE_SOURCES:
        raise CartesianPlanError(f"'start_state.source' must be one of {sorted(_START_STATE_SOURCES)}")
    start_state = {
        "joint_names": list(JOINT_NAMES), "positions_rad": start_positions, "source": start_state_raw["source"],
    }

    end_state_raw = doc["end_state"]
    _check_exact_keys(end_state_raw, _END_STATE_KEYS, "end_state")
    mode = end_state_raw["mode"]
    if mode not in _END_STATE_MODES:
        raise CartesianPlanError(f"'end_state.mode' must be one of {sorted(_END_STATE_MODES)}")
    positions_rad = end_state_raw["positions_rad"]
    if mode == "none":
        if positions_rad is not None:
            raise CartesianPlanError("'end_state.positions_rad' must be null when mode is 'none'")
    else:
        positions_rad = _as_finite_vec(positions_rad, "end_state.positions_rad", _N_JOINTS)
    end_state = {"mode": mode, "positions_rad": positions_rad}

    legs_raw = doc["legs"]
    if not isinstance(legs_raw, list):
        raise CartesianPlanError("'legs' must be a list")
    legs = [_validate_leg_doc(leg, f"legs[{i}]", limits=limits) for i, leg in enumerate(legs_raw)]
    for i, leg in enumerate(legs):
        if leg["leg_index"] != i:
            raise CartesianPlanError(f"'legs[{i}].leg_index' must be {i}, got {leg['leg_index']}")
    _check_leg_continuity(legs, "legs")

    segments_raw = doc["segments"]
    if not isinstance(segments_raw, list):
        raise CartesianPlanError("'segments' must be a list")
    segments = [_validate_segment_report(s, f"segments[{i}]") for i, s in enumerate(segments_raw)]

    coverage = _validate_coverage(doc["coverage"], segments, "coverage")
    caveats = _validate_caveats(doc["caveats"], "caveats")

    return {
        "schema": PLAN_SCHEMA, "case_id": case_id, "created_utc": created_utc, "producer": producer,
        "planning_frame": _FRAME, "joint_names": list(JOINT_NAMES), "sources": sources,
        "parameters": parameters, "start_state": start_state, "end_state": end_state, "legs": legs,
        "segments": segments, "coverage": coverage, "caveats": caveats,
    }


# --------------------------------------------------------------------------------------
# atomic write + I/O (shared by plan and view_plan)
# --------------------------------------------------------------------------------------

def _atomic_write_json(doc: Dict[str, Any], path: Union[str, Path]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(doc, indent=2, sort_keys=False, ensure_ascii=False)
    fd, tmp_name = tempfile.mkstemp(dir=str(path.parent), prefix=f".{path.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(payload)
        os.replace(tmp_name, path)
    except BaseException:
        if os.path.exists(tmp_name):
            os.remove(tmp_name)
        raise


def write_plan(plan: Dict[str, Any], path: Union[str, Path]) -> None:
    """Write `plan` atomically (temp file + `os.replace`)."""
    _atomic_write_json(plan, path)


def load_plan(path: Union[str, Path], limits_path: Union[str, Path]) -> Dict[str, Any]:
    """Load + `validate_plan` a `crackvision.cartesian_plan/1` file. Raises `CartesianPlanError`."""
    try:
        raw_text = Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        raise CartesianPlanError(f"could not read plan file '{path}': {exc}") from exc
    try:
        doc = json.loads(raw_text)
    except json.JSONDecodeError as exc:
        raise CartesianPlanError(f"invalid JSON in '{path}': {exc}") from exc
    limits = load_limits(limits_path)
    return validate_plan(doc, limits)


# --------------------------------------------------------------------------------------
# view_plan: incremental builder (§13.14)
# --------------------------------------------------------------------------------------

_VIEW_TOP_KEYS = frozenset({
    "schema", "created_utc", "producer", "planning_frame", "sources", "parameters", "center_source",
    "center_xy_m", "surface_z_m", "footprint_m", "candidates", "chosen", "transit_leg", "caveats",
})
_VIEW_SOURCES_KEYS = frozenset({"scene", "end_effector", "limits", "config"})
_VIEW_CENTER_SOURCES = frozenset({"scene.specimen", "--center"})
_VIEW_CANDIDATE_KEYS = frozenset({"tilt_deg", "yaw_deg", "camera_pose", "fov_check", "reachable", "recheck_passed"})
_POSE_KEYS = frozenset({"position_m", "quat_xyzw"})
_FOV_CHECK_KEYS = frozenset({"passed", "horizontal_margins_deg", "vertical_margins_deg"})
_VIEW_CHOSEN_KEYS = frozenset({"tilt_deg", "yaw_deg"})


def new_view_plan(
    sources: Dict[str, Any], parameters: Dict[str, Any], center_source: str, center_xy_m: Sequence[float],
    surface_z_m: float, footprint_m: Sequence[float],
) -> Dict[str, Any]:
    """A fresh, empty `crackvision.view_plan/1` document (§13.14.1) ready for `add_view_candidate`."""
    return {
        "schema": VIEW_SCHEMA,
        "created_utc": _utc_now(),
        "producer": "plan_view",
        "planning_frame": _FRAME,
        "sources": sources,
        "parameters": parameters,
        "center_source": center_source,
        "center_xy_m": list(center_xy_m),
        "surface_z_m": surface_z_m,
        "footprint_m": list(footprint_m),
        "candidates": [],
        "chosen": None,
        "transit_leg": None,
        "caveats": [],
    }


def add_view_candidate(
    plan: Dict[str, Any], tilt_deg: float, yaw_deg: float, camera_pose: Dict[str, Any],
    fov_check: Dict[str, Any], reachable: bool, recheck_passed: bool,
) -> Dict[str, Any]:
    """Append one §13.14.1 `candidates[]` entry and return it."""
    entry = {
        "tilt_deg": _as_finite(tilt_deg, "tilt_deg"),
        "yaw_deg": _as_finite(yaw_deg, "yaw_deg"),
        "camera_pose": {
            "position_m": _as_finite_vec(camera_pose["position_m"], "camera_pose.position_m", 3),
            "quat_xyzw": _as_finite_vec(camera_pose["quat_xyzw"], "camera_pose.quat_xyzw", 4),
        },
        "fov_check": {
            "passed": _as_bool(fov_check["passed"], "fov_check.passed"),
            "horizontal_margins_deg": _as_finite_vec(fov_check["horizontal_margins_deg"], "fov_check.horizontal_margins_deg", 4),
            "vertical_margins_deg": _as_finite_vec(fov_check["vertical_margins_deg"], "fov_check.vertical_margins_deg", 4),
        },
        "reachable": _as_bool(reachable, "reachable"),
        "recheck_passed": _as_bool(recheck_passed, "recheck_passed"),
    }
    plan["candidates"].append(entry)
    return entry


def finalize_view(
    plan: Dict[str, Any], chosen: Optional[Dict[str, Any]], transit_leg: Optional[Dict[str, Any]],
    limits: Dict[str, Dict[str, float]], *, task_frame_note: str = "camera_link; production scene (§13.1)",
) -> Dict[str, Any]:
    """Set `chosen`/`transit_leg`, build `caveats[]` and validate the whole document. Mutates and
    returns `plan`."""
    if chosen is not None:
        _check_exact_keys(chosen, _VIEW_CHOSEN_KEYS, "chosen")
        plan["chosen"] = {
            "tilt_deg": _as_finite(chosen["tilt_deg"], "chosen.tilt_deg"),
            "yaw_deg": _as_finite(chosen["yaw_deg"], "chosen.yaw_deg"),
        }
    else:
        plan["chosen"] = None
    plan["transit_leg"] = None if transit_leg is None else transit_leg
    plan["caveats"] = build_caveats(task_frame_note, execution_eligible=None)

    validate_view_plan(plan, limits)
    return plan


def _validate_view_candidate(v: object, ctx: str) -> Dict[str, Any]:
    _check_exact_keys(v, _VIEW_CANDIDATE_KEYS, ctx)
    tilt_deg = _as_finite(v["tilt_deg"], f"{ctx}.tilt_deg")
    yaw_deg = _as_finite(v["yaw_deg"], f"{ctx}.yaw_deg")
    pose_raw = v["camera_pose"]
    _check_exact_keys(pose_raw, _POSE_KEYS, f"{ctx}.camera_pose")
    camera_pose = {
        "position_m": _as_finite_vec(pose_raw["position_m"], f"{ctx}.camera_pose.position_m", 3),
        "quat_xyzw": _as_finite_vec(pose_raw["quat_xyzw"], f"{ctx}.camera_pose.quat_xyzw", 4),
    }
    fov_raw = v["fov_check"]
    _check_exact_keys(fov_raw, _FOV_CHECK_KEYS, f"{ctx}.fov_check")
    fov_check = {
        "passed": _as_bool(fov_raw["passed"], f"{ctx}.fov_check.passed"),
        "horizontal_margins_deg": _as_finite_vec(fov_raw["horizontal_margins_deg"], f"{ctx}.fov_check.horizontal_margins_deg", 4),
        "vertical_margins_deg": _as_finite_vec(fov_raw["vertical_margins_deg"], f"{ctx}.fov_check.vertical_margins_deg", 4),
    }
    reachable = _as_bool(v["reachable"], f"{ctx}.reachable")
    recheck_passed = _as_bool(v["recheck_passed"], f"{ctx}.recheck_passed")
    return {
        "tilt_deg": tilt_deg, "yaw_deg": yaw_deg, "camera_pose": camera_pose, "fov_check": fov_check,
        "reachable": reachable, "recheck_passed": recheck_passed,
    }


def validate_view_plan(doc: object, limits: Dict[str, Dict[str, float]]) -> Dict[str, Any]:
    """Strictly validate a `crackvision.view_plan/1` document per §13.14.1. Raises
    `CartesianPlanError`."""
    _check_exact_keys(doc, _VIEW_TOP_KEYS, "<root>")
    if doc["schema"] != VIEW_SCHEMA:
        raise CartesianPlanError(f"'schema' must be {VIEW_SCHEMA!r}, got {doc['schema']!r}")
    created_utc = _as_str(doc["created_utc"], "created_utc")
    producer = _as_str(doc["producer"], "producer")
    if doc["planning_frame"] != _FRAME:
        raise CartesianPlanError(f"'planning_frame' must be {_FRAME!r}, got {doc['planning_frame']!r}")

    sources_raw = doc["sources"]
    _check_exact_keys(sources_raw, _VIEW_SOURCES_KEYS, "sources")
    sources = {k: _file_source(sources_raw[k], f"sources.{k}") for k in _VIEW_SOURCES_KEYS}

    parameters = doc["parameters"]
    if not isinstance(parameters, dict) or not parameters:
        raise CartesianPlanError("'parameters' must be a non-empty object")

    center_source = doc["center_source"]
    if center_source not in _VIEW_CENTER_SOURCES:
        raise CartesianPlanError(f"'center_source' must be one of {sorted(_VIEW_CENTER_SOURCES)}")
    center_xy_m = _as_finite_vec(doc["center_xy_m"], "center_xy_m", 2)
    surface_z_m = _as_finite(doc["surface_z_m"], "surface_z_m")
    footprint_m = _as_finite_vec(doc["footprint_m"], "footprint_m", 2)
    if footprint_m[0] <= 0 or footprint_m[1] <= 0:
        raise CartesianPlanError("'footprint_m' entries must be > 0")

    candidates_raw = doc["candidates"]
    if not isinstance(candidates_raw, list):
        raise CartesianPlanError("'candidates' must be a list")
    candidates = [_validate_view_candidate(c, f"candidates[{i}]") for i, c in enumerate(candidates_raw)]

    chosen_raw = doc["chosen"]
    chosen = None
    if chosen_raw is not None:
        _check_exact_keys(chosen_raw, _VIEW_CHOSEN_KEYS, "chosen")
        chosen = {
            "tilt_deg": _as_finite(chosen_raw["tilt_deg"], "chosen.tilt_deg"),
            "yaw_deg": _as_finite(chosen_raw["yaw_deg"], "chosen.yaw_deg"),
        }
        matches = [
            c for c in candidates
            if c["tilt_deg"] == chosen["tilt_deg"] and c["yaw_deg"] == chosen["yaw_deg"]
        ]
        if len(matches) != 1:
            raise CartesianPlanError("'chosen' must match exactly one candidates[] entry")
        if not (matches[0]["fov_check"]["passed"] and matches[0]["reachable"] and matches[0]["recheck_passed"]):
            raise CartesianPlanError("'chosen' candidate must have passed fov_check/reachable/recheck_passed")

    transit_leg_raw = doc["transit_leg"]
    if (chosen is None) != (transit_leg_raw is None):
        raise CartesianPlanError("'chosen' and 'transit_leg' must be null together or non-null together")
    transit_leg = None
    if transit_leg_raw is not None:
        transit_leg = _validate_leg_doc(transit_leg_raw, "transit_leg", limits=limits)
        if transit_leg["kind"] != "transit" or transit_leg["segment_ref"] is not None \
                or transit_leg["direction"] is not None or transit_leg["roll_offset_rad"] is not None:
            raise CartesianPlanError(
                "'transit_leg' must have kind 'transit' and null segment_ref/direction/roll_offset_rad"
            )

    caveats = _validate_caveats(doc["caveats"], "caveats")

    return {
        "schema": VIEW_SCHEMA, "created_utc": created_utc, "producer": producer, "planning_frame": _FRAME,
        "sources": sources, "parameters": parameters, "center_source": center_source,
        "center_xy_m": center_xy_m, "surface_z_m": surface_z_m, "footprint_m": footprint_m,
        "candidates": candidates, "chosen": chosen, "transit_leg": transit_leg, "caveats": caveats,
    }


def write_view_plan(plan: Dict[str, Any], path: Union[str, Path]) -> None:
    """Write `plan` atomically (temp file + `os.replace`)."""
    _atomic_write_json(plan, path)


def load_view_plan(path: Union[str, Path], limits_path: Union[str, Path]) -> Dict[str, Any]:
    """Load + `validate_view_plan` a `crackvision.view_plan/1` file. Raises `CartesianPlanError`."""
    try:
        raw_text = Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        raise CartesianPlanError(f"could not read view_plan file '{path}': {exc}") from exc
    try:
        doc = json.loads(raw_text)
    except json.JSONDecodeError as exc:
        raise CartesianPlanError(f"invalid JSON in '{path}': {exc}") from exc
    limits = load_limits(limits_path)
    return validate_view_plan(doc, limits)

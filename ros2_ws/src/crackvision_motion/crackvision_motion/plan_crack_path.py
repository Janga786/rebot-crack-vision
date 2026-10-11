"""plan_crack_path -- Cartesian crack-path planning node/CLI (MOT-06.6).

console_script: `ros2 run crackvision_motion plan_crack_path`. Runs under the system python3 (ROS
Humble, py3.10). See docs/INTERFACES.md §13 (normative source: docs/adr/017-cartesian-crack-path-
planning.md) for the full contract this file implements: §13.3 segment order, §13.4 candidate set,
§13.5 per-candidate procedure, §13.6 selection, §13.7 partial-trace splitting, §13.8 independent
re-check, §13.9 surface/scene mismatch, §13.11-13.13 the plan file, §13.15 CLI/exit codes.

Only ever calls `/compute_ik`, `/compute_fk`, `/compute_cartesian_path`, `/plan_kinematic_path`,
`/check_state_validity`, `/get_planning_scene` and `/apply_planning_scene` (the last two via
`scene_apply.apply_scene`). There is no action client anywhere in this file -- nothing here can
command or execute a trajectory; §13's own preamble is explicit that its output is untimed
geometric data, never a valid `execute_trajectory --trajectory` input.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Set, Tuple

import rclpy
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node

from .cartesian_core import (
    CartesianConfigError,
    Candidate,
    Paths3dError,
    Pose,
    SegmentTask,
    apply_candidate,
    candidates,
    config_sha256,
    load_config,
    load_paths3d,
    segment_tasks,
    split_remainder,
    surface_scene_mismatch,
)
from .cartesian_moveit import (
    MoveItClients,
    build_cartesian_request,
    build_fk_request,
    build_ik_request,
    build_motion_plan_request,
    build_validity_request,
    ik_call_timeout_s,
    parse_cartesian,
    parse_fk,
    parse_ik,
    parse_motion_plan,
)
from .cartesian_plan import add_leg, add_segment_report, finalize, new_plan, write_plan
from .cli_common import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_USAGE,
    PreconditionError,
    RunSummary,
    add_common_args,
    find_root,
    run_cli,
    setup_logging,
)
from .joint_trajectory import JOINT_NAMES, load_limits
from .reachability_core import ConfigError
from .scene_apply import apply_scene
from .scene_core import SceneError, config_sha256 as scene_config_sha256, load_config as load_scene_config

TOOL = "plan_crack_path"
LINK = "tool_tip"

DEFAULT_PLANNING_CONFIG = "config/motion/cartesian_planning.yaml"
DEFAULT_SCENE_CONFIG = "config/scene/scene.yaml"
DEFAULT_END_EFFECTOR_CONFIG = "config/robot/end_effector.yaml"
DEFAULT_LIMITS = "config/robot/b601_dm_limits.yaml"

_N_JOINTS = len(JOINT_NAMES)

_SCENE_SERVICE_SPECS = (
    ("get_scene", GetPlanningScene, "/get_planning_scene"),
    ("apply_scene", ApplyPlanningScene, "/apply_planning_scene"),
)


# --------------------------------------------------------------------------------------
# small pure helpers (joints, FK errors, limits, joint step)
# --------------------------------------------------------------------------------------

def _joint_dict(values: Sequence[float]) -> Dict[str, float]:
    return dict(zip(JOINT_NAMES, (float(v) for v in values)))


def _within_limits(configs: Sequence[Sequence[float]], limits: Dict[str, Dict[str, float]]) -> bool:
    for vals in configs:
        for name, v in zip(JOINT_NAMES, vals):
            spec = limits[name]
            if v < spec["lower"] or v > spec["upper"]:
                return False
    return True

def _min_joint_margin(configs: Sequence[Sequence[float]], limits: Dict[str, Dict[str, float]]) -> float:
    best = math.inf
    for vals in configs:
        for name, v in zip(JOINT_NAMES, vals):
            spec = limits[name]
            best = min(best, v - spec["lower"], spec["upper"] - v)
    return best


def _max_joint_step(configs: Sequence[Sequence[float]]) -> float:
    if len(configs) < 2:
        return 0.0
    return max(
        max(abs(a - b) for a, b in zip(configs[i], configs[i + 1]))
        for i in range(len(configs) - 1)
    )


def _quat_angle_deg(q1: Sequence[float], q2: Sequence[float]) -> float:
    dot = max(-1.0, min(1.0, abs(sum(a * b for a, b in zip(q1, q2)))))
    return math.degrees(2.0 * math.acos(dot))


def _pose_errors(pose: Pose, target: Pose) -> Tuple[float, float]:
    pos_err = math.sqrt(sum((a - b) ** 2 for a, b in zip(pose.position_m, target.position_m)))
    return pos_err, _quat_angle_deg(pose.quat_xyzw, target.quat_xyzw)


def _index_map_and_errors(
    fk_poses: Sequence[Pose], targets: Sequence[Pose], tol_m: float, tol_deg: float,
) -> Tuple[List[Optional[int]], float, float]:
    """§13.7 FK-matching pointer walk (mirrors cartesian_moveit.match_reached_waypoints), but also
    returns the full per-config `waypoint_index_map` (§13.11.2) that function does not expose, plus
    the max position/angle error over only the FK-*matched* configurations (§13.11.2's `recheck`
    is scoped to FK-checked configurations, i.e. those that got a `waypoint_index_map` entry) --
    an unmatched intermediate/resampled configuration has no target of its own to be judged
    against, so it is never allowed to inflate this diagnostic."""
    pointer = 0
    index_map: List[Optional[int]] = []
    max_pos_err = 0.0
    max_ang_err = 0.0
    n = len(targets)
    for pose in fk_poses:
        if pointer >= n:
            index_map.append(None)
            continue
        target = targets[pointer]
        pos_err, ang_err = _pose_errors(pose, target)
        if pos_err <= tol_m and ang_err <= tol_deg:
            index_map.append(pointer)
            max_pos_err = max(max_pos_err, pos_err)
            max_ang_err = max(max_ang_err, ang_err)
            pointer += 1
        else:
            index_map.append(None)
    return index_map, max_pos_err, max_ang_err


def _last_reached(index_map: Sequence[Optional[int]]) -> int:
    return max((v for v in index_map if v is not None), default=-1)


def _recheck_dict(
    max_pos_err_m: float, max_ang_err_deg: float, all_valid: bool, within_limits_ok: bool,
    max_joint_step_rad: float,
) -> Dict[str, Any]:
    return {
        "max_pos_err_m": max_pos_err_m,
        "max_ang_err_rad": math.radians(max_ang_err_deg),
        "all_valid": all_valid,
        "within_limits": within_limits_ok,
        "max_joint_step_rad": max_joint_step_rad,
    }


def _prefix_length_m(trace: Sequence[Pose], count: int) -> float:
    if count <= 1:
        return 0.0
    pts = [p.position_m for p in trace[:count]]
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def _synthesized_retract_pose(rolled_task: SegmentTask, last_idx: int) -> Pose:
    """§13.7's synthesised retract target: offset `approach_retract_offset_m` (recovered here as
    the distance already baked into the task's own approach pose, §10.5 point 4) along the
    last-reached trace waypoint's own normal, same orientation as that waypoint."""
    offset_m = math.dist(rolled_task.approach.position_m, rolled_task.surface_points[0])
    surface_point = rolled_task.surface_points[last_idx]
    normal = rolled_task.normals[last_idx]
    position = tuple(s + offset_m * n for s, n in zip(surface_point, normal))
    return Pose(position, rolled_task.trace[last_idx].quat_xyzw)


# --------------------------------------------------------------------------------------
# candidate attempt / leg result (ROS-produced, but structurally plain)
# --------------------------------------------------------------------------------------

@dataclass
class LegResult:
    kind: str
    positions: List[List[float]]
    waypoint_index_map: List[Optional[int]]
    recheck: Dict[str, Any]


@dataclass
class CandidateAttempt:
    candidate: Candidate
    rolled_task: SegmentTask
    legs: List[LegResult] = field(default_factory=list)
    last_reached_index: int = -1
    full: bool = False
    margin: Optional[float] = None
    final_positions: Optional[Dict[str, float]] = None
    reasons: Set[str] = field(default_factory=set)


# --------------------------------------------------------------------------------------
# pure selection logic (§13.6/§13.7) -- unit-tested with fake CandidateAttempt lists
# --------------------------------------------------------------------------------------

def select_full(attempts: Sequence[CandidateAttempt]) -> Optional[CandidateAttempt]:
    """§13.6: among fully-completing candidates, the max joint margin wins; ties (rounded to 1e-6)
    keep the first (earliest in §13.4 order) -- `attempts` must already be in that order."""
    best: Optional[CandidateAttempt] = None
    for a in attempts:
        if not a.full:
            continue
        margin = round(a.margin, 6) if a.margin is not None else float("-inf")
        if best is None or margin > round(best.margin, 6):
            best = a
    return best


def select_fallback(attempts: Sequence[CandidateAttempt]) -> Optional[CandidateAttempt]:
    """§13.7: among all attempted candidates, the largest `last_reached_index` wins; `None` if the
    best index is -1 (the segment fails, no split possible). Ties within the largest-index group
    are broken exactly as `select_full`."""
    best_idx = max((a.last_reached_index for a in attempts), default=-1)
    if best_idx < 0:
        return None
    group = [a for a in attempts if a.last_reached_index == best_idx]
    best: Optional[CandidateAttempt] = None
    for a in group:
        margin = round(a.margin, 6) if a.margin is not None else float("-inf")
        if best is None or margin > round(best.margin, 6):
            best = a
    return best


def aggregate_reasons(attempts: Sequence[CandidateAttempt]) -> List[str]:
    out: Set[str] = set()
    for a in attempts:
        out.update(a.reasons)
    return sorted(out)


def _chosen_dict(candidate: Optional[Candidate]) -> Optional[Dict[str, float]]:
    if candidate is None:
        return None
    return {"direction": candidate.direction, "roll_offset_rad": candidate.roll_offset_rad}


def _build_report_spec(
    segment_ref: Dict[str, Any], candidate: Optional[Candidate], reasons: Sequence[str],
    candidates_tried: int, total_count: int, planned_count: int, total_len: float, planned_len: float,
) -> Dict[str, Any]:
    return {
        "segment_ref": segment_ref,
        "reasons": list(reasons),
        "candidates_tried": candidates_tried,
        "chosen": _chosen_dict(candidate),
        "trace_waypoints_total": total_count,
        "trace_waypoints_planned": planned_count,
        "trace_length_total": total_len,
        "trace_length_planned": planned_len,
    }


def _flatten_legs(attempt: CandidateAttempt, segment_ref: Dict[str, Any]) -> List[Dict[str, Any]]:
    return [
        {
            "kind": leg.kind,
            "segment_ref": segment_ref,
            "direction": attempt.candidate.direction,
            "roll_offset_rad": attempt.candidate.roll_offset_rad,
            "positions": leg.positions,
            "waypoint_index_map": leg.waypoint_index_map,
            "recheck": leg.recheck,
        }
        for leg in attempt.legs
    ]


AttemptFn = Callable[[SegmentTask, Dict[str, float]], List[CandidateAttempt]]
SynthRetractFn = Callable[[SegmentTask, int, Dict[str, float]], Optional[LegResult]]
SplitFn = Callable[[SegmentTask, int], Optional[SegmentTask]]


def run_segment_splits(
    task: SegmentTask,
    start_positions: Dict[str, float],
    max_splits: int,
    attempt_fn: AttemptFn,
    synth_retract_fn: SynthRetractFn,
    split_fn: SplitFn = split_remainder,
    *,
    min_waypoints_per_remainder: int,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], Dict[str, float]]:
    """§13.5-13.7 orchestration for one original segment, including every split retry. Pure with
    respect to ROS: `attempt_fn`/`synth_retract_fn`/`split_fn` are injected, so this is directly
    unit-testable with fake planner results (first-full-wins, the margin tie-break, the
    partial->split path and the all-fail path), with no ROS/MoveIt involved.

    Returns `(leg_specs, report_specs, final_positions)`: `leg_specs` are ready for
    `cartesian_plan.add_leg(plan, **spec)`, `report_specs` for
    `cartesian_plan.add_segment_report(plan, **spec)`, in commit order.
    """
    leg_specs: List[Dict[str, Any]] = []
    report_specs: List[Dict[str, Any]] = []
    cur_task = task
    cur_positions = start_positions
    splits_used = 0

    while True:
        attempts = attempt_fn(cur_task, cur_positions)
        total = len(cur_task.trace)

        full = select_full(attempts)
        if full is not None:
            leg_specs.extend(_flatten_legs(full, cur_task.segment_ref))
            report_specs.append(
                _build_report_spec(
                    cur_task.segment_ref, full.candidate, [], len(attempts), total, total,
                    cur_task.trace_length_m, cur_task.trace_length_m,
                )
            )
            cur_positions = full.final_positions
            break

        reasons = aggregate_reasons(attempts)
        fallback = select_fallback(attempts)
        if fallback is None:
            report_specs.append(
                _build_report_spec(cur_task.segment_ref, None, reasons, len(attempts), total, 0, cur_task.trace_length_m, 0.0)
            )
            break

        planned = fallback.last_reached_index + 1
        retract_leg = synth_retract_fn(fallback.rolled_task, fallback.last_reached_index, fallback.final_positions)
        leg_specs.extend(_flatten_legs(fallback, cur_task.segment_ref))
        final_positions = fallback.final_positions
        if retract_leg is not None:
            leg_specs.append(
                {
                    "kind": retract_leg.kind, "segment_ref": cur_task.segment_ref,
                    "direction": fallback.candidate.direction, "roll_offset_rad": fallback.candidate.roll_offset_rad,
                    "positions": retract_leg.positions, "waypoint_index_map": retract_leg.waypoint_index_map,
                    "recheck": retract_leg.recheck,
                }
            )
            final_positions = _joint_dict(retract_leg.positions[-1])
        else:
            reasons = sorted(set(reasons) | {"cartesian_incomplete"})
        cur_positions = final_positions

        planned_len = cur_task.trace_length_m if planned == total else _prefix_length_m(fallback.rolled_task.trace, planned)
        report_specs.append(
            _build_report_spec(
                cur_task.segment_ref, fallback.candidate, reasons, len(attempts), total, planned,
                cur_task.trace_length_m, planned_len,
            )
        )

        if planned == total:
            break
        if splits_used >= max_splits:
            break
        if total - planned < min_waypoints_per_remainder:
            report_specs[-1]["reasons"] = sorted(set(report_specs[-1]["reasons"]) | {"too_short"})
            break
        # §13.7's remainder is retried "from scratch" against paths3d's own base poses, never the
        # previous candidate's rolled/reversed ones -- only this round's own direction (not its
        # roll) carries over, since it fixes which end of `cur_task` the remainder continues from.
        traversal_task = apply_candidate(cur_task, fallback.candidate.direction, 0.0)
        remainder = split_fn(traversal_task, fallback.last_reached_index)
        if remainder is None:
            report_specs[-1]["reasons"] = sorted(set(report_specs[-1]["reasons"]) | {"too_short"})
            break
        splits_used += 1
        cur_task = remainder

    return leg_specs, report_specs, cur_positions


# --------------------------------------------------------------------------------------
# ROS-calling helpers
# --------------------------------------------------------------------------------------

def _check_all_valid(moveit: MoveItClients, configs: Sequence[Sequence[float]]) -> bool:
    for vals in configs:
        resp = moveit.call(moveit.clients["validity"], build_validity_request(_joint_dict(vals)))
        if resp is None or not resp.valid:
            return False
    return True


def _fk_poses(moveit: MoveItClients, configs: Sequence[Sequence[float]]) -> List[Pose]:
    poses: List[Pose] = []
    for vals in configs:
        resp = moveit.call(moveit.clients["fk"], build_fk_request(_joint_dict(vals), [LINK]))
        pose, err = parse_fk(resp, LINK)
        if err is not None:
            raise RuntimeError(f"/compute_fk failed: {err}")
        poses.append(pose)
    return poses


def _classify_shortfall(
    moveit: MoveItClients, cfg: Dict[str, Any], from_positions: Dict[str, float], target: Pose,
) -> str:
    """§13.8's bounded, best-effort attribution for a `fraction < 1` approach/trace/retract
    shortfall (CARTESIAN_PROBE.md consequence 5: the service itself gives no cause): one extra IK
    call, seeded from `from_positions` (the last-reached, or leg-start, configuration), to
    `target` (the next unreached waypoint); IK failure leaves the generic `cartesian_incomplete`.
    On an IK solution, `/check_state_validity` -> `collision` if invalid; otherwise the per-joint
    step from `from_positions` to that solution against `recheck.max_joint_step_rad` ->
    `joint_jump` if over, else the generic `cartesian_incomplete` stands."""
    ik_resp = moveit.call(
        moveit.clients["ik"], build_ik_request(target, from_positions, LINK, cfg),
        timeout_s=ik_call_timeout_s(cfg),
    )
    ik_joints, err = parse_ik(ik_resp)
    if err is not None:
        return "cartesian_incomplete"
    ik_vals = [ik_joints[n] for n in JOINT_NAMES]
    if not _check_all_valid(moveit, [ik_vals]):
        return "collision"
    from_vals = [from_positions[n] for n in JOINT_NAMES]
    if _max_joint_step([from_vals, ik_vals]) > cfg["recheck"]["max_joint_step_rad"]:
        return "joint_jump"
    return "cartesian_incomplete"


def _attempt_candidate(
    moveit: MoveItClients, cfg: Dict[str, Any], limits: Dict[str, Dict[str, float]],
    start_positions: Dict[str, float], task: SegmentTask, candidate: Candidate,
) -> CandidateAttempt:
    """§13.5 steps 1-5 for one (direction, roll) candidate, with the §13.8 independent re-check
    applied to every leg before it is committed. Never accepts a leg on MoveIt's own fraction/
    error-code alone (CARTESIAN_PROBE.md consequence 5/7)."""
    rolled = apply_candidate(task, candidate.direction, candidate.roll_offset_rad)
    reasons: Set[str] = set()
    tol_m = cfg["recheck"]["fk_tolerance_m"]
    tol_deg = cfg["recheck"]["fk_tolerance_deg"]
    max_step_rad = cfg["recheck"]["max_joint_step_rad"]

    def fail(reason: str) -> CandidateAttempt:
        reasons.add(reason)
        return CandidateAttempt(candidate, rolled, [], -1, False, None, None, reasons)

    # Step 1: IK for the (rolled) approach pose.
    ik_resp = moveit.call(
        moveit.clients["ik"], build_ik_request(rolled.approach, start_positions, LINK, cfg),
        timeout_s=ik_call_timeout_s(cfg),
    )
    approach_joints, err = parse_ik(ik_resp)
    if err is not None:
        if err.startswith("ik_error:"):
            raise RuntimeError(f"/compute_ik {err}")
        return fail(err)

    # Step 2: transit (joint-space) from the current config to the IK solution.
    mp_resp = moveit.call(moveit.clients["motion_plan"], build_motion_plan_request(start_positions, approach_joints, cfg))
    transit_configs, err = parse_motion_plan(mp_resp)
    if err is not None:
        return fail(err)
    final_pose = _fk_poses(moveit, [transit_configs[-1]])[0]
    pos_err, ang_err = _pose_errors(final_pose, rolled.approach)
    if pos_err > tol_m or ang_err > tol_deg:
        return fail("fk_mismatch")
    if not _check_all_valid(moveit, transit_configs):
        return fail("collision")
    if not _within_limits(transit_configs, limits):
        return fail("joint_limit")
    step = _max_joint_step(transit_configs)
    if step > max_step_rad:
        return fail("joint_jump")
    transit_map: List[Optional[int]] = [None] * (len(transit_configs) - 1) + [0]
    legs = [LegResult("transit", transit_configs, transit_map, _recheck_dict(pos_err, ang_err, True, True, step))]
    cur_positions = _joint_dict(transit_configs[-1])

    # Step 3: Cartesian approach to the first (rolled) trace pose.
    approach_leg_start = cur_positions
    cart_resp = moveit.call(moveit.clients["cartesian"], build_cartesian_request(cur_positions, [rolled.trace[0]], LINK, cfg))
    approach_configs, fraction, err = parse_cartesian(cart_resp)
    if err is not None or fraction < 1.0:
        return fail(_classify_shortfall(moveit, cfg, approach_leg_start, rolled.trace[0]))
    fk_poses = _fk_poses(moveit, approach_configs)
    idx_map, pos_err, ang_err = _index_map_and_errors(fk_poses, [rolled.trace[0]], tol_m, tol_deg)
    if _last_reached(idx_map) != 0:
        return fail("fk_mismatch")
    if not _check_all_valid(moveit, approach_configs):
        return fail("collision")
    if not _within_limits(approach_configs, limits):
        return fail("joint_limit")
    step = _max_joint_step(approach_configs)
    if step > max_step_rad:
        return fail("joint_jump")
    legs.append(LegResult("approach", approach_configs, idx_map, _recheck_dict(pos_err, ang_err, True, True, step)))
    cur_positions = _joint_dict(approach_configs[-1])

    # Step 4: Cartesian trace across every (rolled) trace pose; partial fallback is §13.7's job,
    # so any recheck failure on the reached prefix conservatively drops the whole prefix (never
    # over-claims coverage) rather than guessing where within it to truncate.
    trace_leg_start = cur_positions
    cart_resp = moveit.call(moveit.clients["cartesian"], build_cartesian_request(cur_positions, rolled.trace, LINK, cfg))
    trace_configs, fraction, _cart_err = parse_cartesian(cart_resp)
    if trace_configs:
        fk_poses = _fk_poses(moveit, trace_configs)
        trace_map, pos_err, ang_err = _index_map_and_errors(fk_poses, rolled.trace, tol_m, tol_deg)
    else:
        trace_map, pos_err, ang_err = [], 0.0, 0.0
    cutoff = max((i for i, v in enumerate(trace_map) if v is not None), default=-1)
    committed_configs = trace_configs[: cutoff + 1] if cutoff >= 0 else []
    committed_map = trace_map[: cutoff + 1] if cutoff >= 0 else []

    if committed_configs:
        ok = (
            _check_all_valid(moveit, committed_configs)
            and _within_limits(committed_configs, limits)
            and _max_joint_step(committed_configs) <= max_step_rad
        )
        if not ok:
            committed_configs, committed_map = [], []

    last_reached = _last_reached(committed_map)
    full_trace = last_reached == len(rolled.trace) - 1

    if not full_trace:
        # §13.8's fraction<1 classification, from the last FK-matched config (or this leg's own
        # start if none matched) to the next still-unreached trace waypoint.
        shortfall_start = _joint_dict(committed_configs[-1]) if committed_configs else trace_leg_start
        reasons.add(_classify_shortfall(moveit, cfg, shortfall_start, rolled.trace[last_reached + 1]))

    if not committed_configs:
        return CandidateAttempt(candidate, rolled, legs, -1, False, _min_joint_margin(approach_configs, limits), cur_positions, reasons)

    trace_step = _max_joint_step(committed_configs)
    legs.append(LegResult("trace", committed_configs, committed_map, _recheck_dict(pos_err, ang_err, True, True, trace_step)))
    cur_positions = _joint_dict(committed_configs[-1])
    margin = _min_joint_margin(approach_configs + committed_configs, limits)

    if not full_trace:
        return CandidateAttempt(candidate, rolled, legs, last_reached, False, margin, cur_positions, reasons)

    # Step 5: Cartesian retract (only attempted once the whole trace is reached).
    retract_leg_start = cur_positions
    cart_resp = moveit.call(moveit.clients["cartesian"], build_cartesian_request(cur_positions, [rolled.retract], LINK, cfg))
    retract_configs, fraction, err = parse_cartesian(cart_resp)
    if err is not None or fraction < 1.0:
        reason = _classify_shortfall(moveit, cfg, retract_leg_start, rolled.retract)
        return CandidateAttempt(candidate, rolled, legs, last_reached, False, margin, cur_positions, reasons | {reason})
    fk_poses = _fk_poses(moveit, retract_configs)
    r_idx_map, r_pos_err, r_ang_err = _index_map_and_errors(fk_poses, [rolled.retract], tol_m, tol_deg)
    if _last_reached(r_idx_map) != 0:
        return CandidateAttempt(candidate, rolled, legs, last_reached, False, margin, cur_positions, reasons | {"fk_mismatch"})
    if not _check_all_valid(moveit, retract_configs):
        return CandidateAttempt(candidate, rolled, legs, last_reached, False, margin, cur_positions, reasons | {"collision"})
    if not _within_limits(retract_configs, limits):
        return CandidateAttempt(candidate, rolled, legs, last_reached, False, margin, cur_positions, reasons | {"joint_limit"})
    r_step = _max_joint_step(retract_configs)
    if r_step > max_step_rad:
        return CandidateAttempt(candidate, rolled, legs, last_reached, False, margin, cur_positions, reasons | {"joint_jump"})

    legs.append(LegResult("retract", retract_configs, r_idx_map, _recheck_dict(r_pos_err, r_ang_err, True, True, r_step)))
    final_positions = _joint_dict(retract_configs[-1])
    full_margin = _min_joint_margin(approach_configs + committed_configs + retract_configs, limits)
    return CandidateAttempt(candidate, rolled, legs, last_reached, True, full_margin, final_positions, reasons)


def _synth_retract(
    moveit: MoveItClients, cfg: Dict[str, Any], limits: Dict[str, Dict[str, float]],
    rolled_task: SegmentTask, last_idx: int, start_positions: Dict[str, float],
) -> Optional[LegResult]:
    """§13.7's synthesised retract leg, from the last-reached configuration to the offset pose
    built from that waypoint's own normal/orientation. Independently re-checked like any other
    leg; `None` (never committed) if it fails, in which case the caller still has every leg up to
    the last-reached trace configuration."""
    target = _synthesized_retract_pose(rolled_task, last_idx)
    resp = moveit.call(moveit.clients["cartesian"], build_cartesian_request(start_positions, [target], LINK, cfg))
    configs, fraction, err = parse_cartesian(resp)
    if err is not None or fraction < 1.0 or not configs:
        return None
    fk_poses = _fk_poses(moveit, configs)
    idx_map, pos_err, ang_err = _index_map_and_errors(
        fk_poses, [target], cfg["recheck"]["fk_tolerance_m"], cfg["recheck"]["fk_tolerance_deg"]
    )
    if _last_reached(idx_map) != 0:
        return None
    if not _check_all_valid(moveit, configs) or not _within_limits(configs, limits):
        return None
    step = _max_joint_step(configs)
    if step > cfg["recheck"]["max_joint_step_rad"]:
        return None
    return LegResult("retract", configs, idx_map, _recheck_dict(pos_err, ang_err, True, True, step))


def _plan_end_transit(
    moveit: MoveItClients, cfg: Dict[str, Any], limits: Dict[str, Dict[str, float]],
    start_positions: Dict[str, float], target_positions: Dict[str, float],
) -> Optional[LegResult]:
    """The trailing `end_state` transit leg (§13.1/§13.8): FK-exempt (no Cartesian pose target),
    still subject to collision/limits/joint-step."""
    resp = moveit.call(moveit.clients["motion_plan"], build_motion_plan_request(start_positions, target_positions, cfg))
    configs, err = parse_motion_plan(resp)
    if err is not None:
        return None
    if not _check_all_valid(moveit, configs) or not _within_limits(configs, limits):
        return None
    step = _max_joint_step(configs)
    if step > cfg["recheck"]["max_joint_step_rad"]:
        return None
    return LegResult("transit", configs, [None] * len(configs), _recheck_dict(0.0, 0.0, True, True, step))


def _wait_for_scene_services(node: Node, timeout_s: float) -> Dict[str, Any]:
    clients: Dict[str, Any] = {}
    deadline = time.monotonic() + timeout_s
    for key, srv_type, name in _SCENE_SERVICE_SPECS:
        client = node.create_client(srv_type, name)
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=max(remaining, 0.0)):
            raise PreconditionError(f"service '{name}' did not become available within {timeout_s}s")
        clients[key] = client
    return clients


# --------------------------------------------------------------------------------------
# CLI plumbing
# --------------------------------------------------------------------------------------

def _resolve(root: Path, value: Optional[str], default_rel: str) -> Path:
    if value is not None:
        p = Path(value).expanduser()
        return p.resolve() if p.is_absolute() else (root / p).resolve()
    return (root / default_rel).resolve()


def _resolve_paths(args: argparse.Namespace, root: Path) -> Dict[str, Path]:
    return {
        "planning_config": _resolve(root, args.planning_config, DEFAULT_PLANNING_CONFIG),
        "scene_config": _resolve(root, args.scene_config, DEFAULT_SCENE_CONFIG),
        "end_effector": (root / DEFAULT_END_EFFECTOR_CONFIG).resolve(),
        "limits": (root / DEFAULT_LIMITS).resolve(),
    }


def _relpath(path: Path, root: Path) -> str:
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path)


def _load_planning_config(path: Path) -> Dict[str, Any]:
    """`cartesian_core.load_config`, with a missing/unreadable file folded into the same §0.2
    usage/config-error outcome as a malformed one (it does not raise `CartesianConfigError` on
    its own -- a bare `Path.read_text` failure would otherwise surface as a generic exit 1)."""
    try:
        return load_config(path)
    except OSError as exc:
        raise CartesianConfigError(f"cannot read planning config '{path}': {exc}") from exc


def _load_scene_cfg(path: Path) -> Dict[str, Any]:
    try:
        return load_scene_config(path)
    except OSError as exc:
        raise SceneError(f"cannot read scene config '{path}': {exc}") from exc


def _load_capture_joint_state(root: Path, paths3d_doc: Dict[str, Any]) -> Dict[str, float]:
    """§13.1 start state: the `paths3d` file's own `sources.capture_record`, independently
    re-read and sha-checked against `sources.capture_record_sha256` (§13.15's "sha mismatch
    against a resolved input" refusal) -- never trusted from `paths3d` alone."""
    sources = paths3d_doc.get("sources")
    if not isinstance(sources, dict):
        raise PreconditionError("paths3d file has no 'sources' block")
    rel_path = sources.get("capture_record")
    recorded_sha = sources.get("capture_record_sha256")
    if not isinstance(rel_path, str) or not rel_path or not isinstance(recorded_sha, str) or not recorded_sha:
        raise PreconditionError("paths3d file is missing sources.capture_record / sources.capture_record_sha256")

    capture_path = (root / rel_path).resolve()
    try:
        raw_bytes = capture_path.read_bytes()
    except OSError as exc:
        raise PreconditionError(f"capture_record '{capture_path}' unreadable: {exc}") from exc
    actual_sha = hashlib.sha256(raw_bytes).hexdigest()
    if actual_sha.lower() != recorded_sha.lower():
        raise PreconditionError(
            f"capture_record '{capture_path}' sha256 {actual_sha} does not match paths3d's recorded "
            f"sources.capture_record_sha256 {recorded_sha}"
        )
    try:
        capture_doc = json.loads(raw_bytes.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise PreconditionError(f"capture_record '{capture_path}' is not valid JSON: {exc}") from exc

    robot = capture_doc.get("robot") if isinstance(capture_doc, dict) else None
    if not isinstance(robot, dict):
        raise PreconditionError(f"capture_record '{capture_path}' has no 'robot' block (2D-only capture, §9.4)")
    joint_names = robot.get("joint_names")
    positions = robot.get("positions_rad")
    if joint_names != list(JOINT_NAMES) or not isinstance(positions, list) or len(positions) != _N_JOINTS:
        raise PreconditionError(f"capture_record '{capture_path}' 'robot' block is malformed")
    try:
        values = [float(v) for v in positions]
    except (TypeError, ValueError) as exc:
        raise PreconditionError(f"capture_record '{capture_path}' robot.positions_rad is not all numeric: {exc}") from exc
    if not all(math.isfinite(v) for v in values):
        raise PreconditionError(f"capture_record '{capture_path}' robot.positions_rad has a non-finite value")
    return _joint_dict(values)


def _extra_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--paths3d", required=True, help="crackvision.paths3d/1 file to plan (§13.1)")
    parser.add_argument("--out", default=None, help="plan file to write (default: data/plans/{case}_cartesian_plan.json)")
    parser.add_argument(
        "--start-joints", type=float, nargs=6, default=None, metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="override the start state (radians); default: paths3d's own capture_record joint state",
    )
    parser.add_argument("--scene-config", default=None, help=f"default: {DEFAULT_SCENE_CONFIG} under --root")
    parser.add_argument("--planning-config", default=None, help=f"default: {DEFAULT_PLANNING_CONFIG} under --root")
    parser.add_argument("--require-full-coverage", action="store_true", help="exit 3 unless coverage.complete is true")


def _dry_run(args: argparse.Namespace) -> int:
    """§13.15: validate every input (paths/shas/schema) and print the segment/candidate plan --
    no ROS service call, no `rclpy.init`, no file write, exit 0 on success."""
    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    log_dir = root / "logs"
    logger = setup_logging(TOOL, log_dir, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()
    status, exit_code = "ok", EXIT_OK
    try:
        paths = _resolve_paths(args, root)
        cfg = _load_planning_config(paths["planning_config"])
        scene_cfg = _load_scene_cfg(paths["scene_config"])
        if not paths["end_effector"].is_file():
            raise PreconditionError(f"end_effector config not found: {paths['end_effector']}")
        if not paths["limits"].is_file():
            raise PreconditionError(f"limits file not found: {paths['limits']}")
        load_limits(paths["limits"])
        doc, _sha = load_paths3d(args.paths3d)

        tasks = segment_tasks(doc)
        cand_count = len(candidates(cfg))
        logger.info(
            "--dry-run: paths3d=%s segments=%d candidates_per_segment=%d", args.paths3d, len(tasks), cand_count,
        )
        for i, task in enumerate(tasks):
            mismatch = surface_scene_mismatch(task, scene_cfg, cfg["surface_object_id"], cfg["surface_object_tolerance_m"])
            ref = task.segment_ref
            n_cand = 0 if mismatch else cand_count
            logger.info(
                "  segment[%d] component=%s polyline=%s branch=%s segment_index=%d trace_waypoints=%d candidates=%d%s",
                i, ref["component_id"], ref["polyline_kind"], ref["branch_index"], ref["segment_index"],
                len(task.trace), n_cand, " (surface_scene_mismatch)" if mismatch else "",
            )
        summary.counts.update({"segments": len(tasks), "candidates_per_segment": cand_count})
    except (CartesianConfigError, SceneError) as exc:
        logger.error("%s", exc)
        summary.add_error(str(exc))
        status, exit_code = "failed", EXIT_USAGE
    except (Paths3dError, PreconditionError) as exc:
        logger.error("%s", exc)
        summary.add_error(str(exc))
        status, exit_code = "precondition", EXIT_PRECONDITION
    summary.write(log_dir, TOOL, status, exit_code)
    return exit_code


def _run(args: argparse.Namespace, log, summary: RunSummary) -> Dict[str, str]:
    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    paths = _resolve_paths(args, root)

    try:
        cfg = _load_planning_config(paths["planning_config"])
    except CartesianConfigError as exc:
        raise ConfigError(f"cartesian planning config invalid: {exc}") from exc
    cfg_sha = config_sha256(paths["planning_config"])

    try:
        scene_cfg = _load_scene_cfg(paths["scene_config"])
    except SceneError as exc:
        raise ConfigError(f"scene config invalid: {exc}") from exc
    scene_sha = scene_config_sha256(paths["scene_config"])

    if not paths["end_effector"].is_file():
        raise PreconditionError(f"end_effector config not found: {paths['end_effector']}")
    ee_sha = hashlib.sha256(paths["end_effector"].read_bytes()).hexdigest()

    if not paths["limits"].is_file():
        raise PreconditionError(f"limits file not found: {paths['limits']}")
    limits = load_limits(paths["limits"])
    limits_sha = hashlib.sha256(paths["limits"].read_bytes()).hexdigest()

    try:
        paths3d_doc, paths3d_sha = load_paths3d(args.paths3d)
    except Paths3dError as exc:
        raise PreconditionError(f"paths3d file invalid: {exc}") from exc

    case_id = paths3d_doc["case_id"]
    out_path = (
        Path(args.out).expanduser().resolve() if args.out
        else (root / "data" / "plans" / f"{case_id}_cartesian_plan.json")
    )
    if args.skip_existing and out_path.exists():
        log.info("--skip-existing: %s already exists, skipping", out_path)
        summary.increment("skipped")
        return {"status": "ok"}

    if args.start_joints is not None:
        start_positions = _joint_dict(args.start_joints)
        if not all(math.isfinite(v) for v in start_positions.values()):
            raise PreconditionError("--start-joints values must be finite")
        if not _within_limits([args.start_joints], limits):
            raise PreconditionError("--start-joints values lie outside b601_dm_limits.yaml's inclusive bound")
        start_source = "start_joints_override"
    else:
        start_positions = _load_capture_joint_state(root, paths3d_doc)
        start_source = "capture_record"

    end_mode = cfg["end_state"]
    if end_mode == "start":
        end_positions: Optional[List[float]] = [start_positions[n] for n in JOINT_NAMES]
    elif end_mode == "home":
        end_positions = [0.0] * _N_JOINTS
    else:
        end_positions = None

    tasks = segment_tasks(paths3d_doc)
    cand_list = candidates(cfg)

    paths3d_abs = Path(args.paths3d).expanduser().resolve()
    paths3d_rel = _relpath(paths3d_abs, root)

    plan = new_plan(
        case_id=case_id,
        sources={
            "paths3d": {"path": paths3d_rel, "sha256": paths3d_sha, "execution_eligible": None},
            "scene": {"path": _relpath(paths["scene_config"], root), "sha256": scene_sha},
            "end_effector": {"path": _relpath(paths["end_effector"], root), "sha256": ee_sha},
            "limits": {"path": _relpath(paths["limits"], root), "sha256": limits_sha},
            "config": {"path": _relpath(paths["planning_config"], root), "sha256": cfg_sha},
        },
        parameters=cfg,
        start_state={"joint_names": list(JOINT_NAMES), "positions_rad": [start_positions[n] for n in JOINT_NAMES], "source": start_source},
        end_state={"mode": end_mode, "positions_rad": end_positions},
    )

    rclpy.init(args=None)
    node = Node(TOOL)
    try:
        moveit = MoveItClients(node, cfg["service_timeout_s"])
        moveit.wait_for_services()
        scene_clients = _wait_for_scene_services(node, cfg["service_timeout_s"])
        apply_scene(node, scene_clients["get_scene"], scene_clients["apply_scene"], scene_cfg)
        log.info("applied scene from %s", paths["scene_config"])

        cur_positions = dict(start_positions)
        for task in tasks:
            mismatch_reasons = surface_scene_mismatch(task, scene_cfg, cfg["surface_object_id"], cfg["surface_object_tolerance_m"])
            if mismatch_reasons:
                add_segment_report(
                    plan, task.segment_ref, reasons=mismatch_reasons, candidates_tried=0, chosen=None,
                    trace_waypoints_total=len(task.trace), trace_waypoints_planned=0,
                    trace_length_total=task.trace_length_m, trace_length_planned=0.0, mismatch=True,
                )
                log.warning("segment %s flagged surface_scene_mismatch, not planned", task.segment_ref)
                continue

            def attempt_fn(t: SegmentTask, positions: Dict[str, float]) -> List[CandidateAttempt]:
                return [_attempt_candidate(moveit, cfg, limits, positions, t, cand) for cand in cand_list]

            def synth_retract_fn(rolled_task: SegmentTask, last_idx: int, positions: Dict[str, float]) -> Optional[LegResult]:
                return _synth_retract(moveit, cfg, limits, rolled_task, last_idx, positions)

            leg_specs, report_specs, cur_positions = run_segment_splits(
                task, cur_positions, cfg["splitting"]["max_splits_per_segment"], attempt_fn, synth_retract_fn,
                min_waypoints_per_remainder=cfg["splitting"]["min_waypoints_per_remainder"],
            )
            for spec in leg_specs:
                add_leg(plan, **spec)
            for spec in report_specs:
                add_segment_report(plan, **spec)
            planned = sum(s["trace_waypoints_planned"] for s in report_specs)
            log.info("segment %s: planned %d/%d trace waypoints", task.segment_ref, planned, len(task.trace))

        if end_mode != "none":
            target = _joint_dict(end_positions)
            end_leg = _plan_end_transit(moveit, cfg, limits, cur_positions, target)
            if end_leg is None:
                raise RuntimeError(f"end_state ({end_mode!r}) transit failed or failed its independent re-check")
            add_leg(plan, end_leg.kind, None, None, None, end_leg.positions, end_leg.waypoint_index_map, end_leg.recheck)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    finalize(plan, paths3d_doc, tasks, limits)

    if plan["coverage"]["trace_waypoints_planned"] == 0:
        raise PreconditionError("zero trace waypoints planned across the whole paths3d file")
    if args.require_full_coverage and not plan["coverage"]["complete"]:
        raise PreconditionError("--require-full-coverage was given but coverage.complete is false")

    write_plan(plan, out_path)
    cov = plan["coverage"]
    log.info(
        "wrote %s: coverage %d/%d waypoints (%.1f%% by length)",
        out_path, cov["trace_waypoints_planned"], cov["trace_waypoints_total"], 100.0 * cov["fraction_by_length"],
    )
    summary.counts.update({
        "segments_total": len(tasks),
        "trace_waypoints_total": cov["trace_waypoints_total"],
        "trace_waypoints_planned": cov["trace_waypoints_planned"],
    })
    return {"status": "ok"}


def main(argv=None) -> int:
    argv = list(argv) if argv is not None else sys.argv[1:]
    parser = argparse.ArgumentParser(prog=TOOL)
    add_common_args(parser)
    _extra_args(parser)
    args = parser.parse_args(argv)
    if args.dry_run:
        return _dry_run(args)
    return run_cli(TOOL, _run, argv, extra_args=_extra_args)


if __name__ == "__main__":
    sys.exit(main())

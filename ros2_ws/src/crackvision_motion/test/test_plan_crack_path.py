"""Unit tests for plan_crack_path (MOT-06.6). Runs without a colcon build or a running ROS graph
-- the selection/splitting logic is tested with fake planner results (`attempt_fn`/`synth_retract_fn`/
`split_fn` injected), never a real MoveIt call; `_classify_shortfall` is tested against canned
`moveit_msgs` response messages via a `MoveItClients`-shaped fake (mirrors test_cartesian_moveit.py
-- still no `rclpy.init`, no running graph); the CLI-level tests only ever exercise `--dry-run`
(no `rclpy.init`, no file write).

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py -q -p no:cacheprovider
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List

import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
from trajectory_msgs.msg import JointTrajectoryPoint

from crackvision_motion.cartesian_core import Candidate, Pose, SegmentTask, apply_candidate, load_config
from crackvision_motion.cli_common import EXIT_OK, EXIT_PRECONDITION, EXIT_USAGE
from crackvision_motion.joint_trajectory import JOINT_NAMES
from crackvision_motion.plan_crack_path import (
    CandidateAttempt,
    LegResult,
    _attempt_candidate,
    _classify_shortfall,
    _quat_angle_deg,
    _index_map_and_errors,
    _joint_dict,
    _max_joint_step,
    _min_joint_margin,
    _nearest_point_on_segment,
    _pose_errors,
    _polyline_index_map_and_errors,
    _prefix_length_m,
    _slerp_quat,
    _synthesized_retract_pose,
    _within_limits,
    aggregate_reasons,
    main,
    run_segment_splits,
    select_fallback,
    select_full,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures" / "cartesian"
MULTI_COMPONENT_FIXTURE = FIXTURES_DIR / "paths3d_multi_component.json"
PRODUCTION_PLANNING_CONFIG = REPO_ROOT / "config" / "motion" / "cartesian_planning.yaml"

_LIMITS = {n: {"lower": -3.0, "upper": 3.0} for n in JOINT_NAMES}
_Q = (0.0, 0.0, 0.0, 1.0)


def _pose(x=0.0, y=0.0, z=0.0, quat=_Q) -> Pose:
    return Pose((x, y, z), quat)


def _zeros6():
    return [0.0] * 6


def _task(trace_xs, segment_ref=None, sub_segment=None) -> SegmentTask:
    """A minimal real `SegmentTask`: `trace_xs` are the x-coordinates of each trace waypoint,
    spaced 0.01 m apart along +x, every other field a plausible constant."""
    ref = segment_ref or {
        "component_id": "crackA", "polyline_kind": "main", "branch_index": None, "segment_index": 0,
        "sub_segment": sub_segment,
    }
    trace = tuple(_pose(x=x) for x in trace_xs)
    normals = tuple((0.0, 0.0, 1.0) for _ in trace_xs)
    surface_points = tuple((x, 0.0, 0.0) for x in trace_xs)
    approach = _pose(x=trace_xs[0], z=0.04)
    retract = _pose(x=trace_xs[-1], z=0.04)
    length = sum(abs(trace_xs[i + 1] - trace_xs[i]) for i in range(len(trace_xs) - 1))
    return SegmentTask(ref, approach, trace, retract, normals, surface_points, length, tuple(0.01 for _ in trace_xs))


def _candidate(direction="forward", roll_deg=0.0) -> Candidate:
    return Candidate(direction, math.radians(roll_deg))


def _leg(kind="trace", n=2, positions=None) -> LegResult:
    positions = positions or [_zeros6() for _ in range(n)]
    return LegResult(kind, positions, [None] * len(positions), {
        "max_pos_err_m": 0.0, "max_ang_err_rad": 0.0, "all_valid": True, "within_limits": True,
        "max_joint_step_rad": 0.0,
    })


def _full_attempt(task, candidate, margin, final=None) -> CandidateAttempt:
    final = final or _zeros6()
    return CandidateAttempt(
        candidate=candidate, rolled_task=task, legs=[_leg("transit"), _leg("approach", 1), _leg("trace"), _leg("retract", 1)],
        last_reached_index=len(task.trace) - 1, full=True, margin=margin, final_positions=_joint_dict(final), reasons=set(),
    )


def _partial_attempt(task, candidate, last_idx, margin, reasons=frozenset(), final=None) -> CandidateAttempt:
    final = final or _zeros6()
    legs = [_leg("transit"), _leg("approach", 1), _leg("trace", last_idx + 1)] if last_idx >= 0 else []
    return CandidateAttempt(
        candidate=candidate, rolled_task=task, legs=legs, last_reached_index=last_idx, full=False,
        margin=margin if last_idx >= 0 else None, final_positions=_joint_dict(final) if last_idx >= 0 else None,
        reasons=set(reasons),
    )


def _failed_attempt(task, candidate, reason) -> CandidateAttempt:
    return CandidateAttempt(candidate=candidate, rolled_task=task, legs=[], last_reached_index=-1, full=False, margin=None, final_positions=None, reasons={reason})


# --------------------------------------------------------------------------------------
# select_full / select_fallback / aggregate_reasons (§13.6/§13.7 selection, pure)
# --------------------------------------------------------------------------------------

def test_select_full_none_when_nothing_fully_completes():
    task = _task([0.0, 0.01, 0.02])
    attempts = [_partial_attempt(task, _candidate(), 1, 0.5), _failed_attempt(task, _candidate("reverse"), "ik_failed")]
    assert select_full(attempts) is None


def test_select_full_first_full_wins_on_equal_margin():
    """Two fully-completing candidates with the same (rounded) margin: the earlier one in
    candidate order wins, never the later one."""
    task = _task([0.0, 0.01, 0.02])
    first = _full_attempt(task, _candidate("forward", 0.0), margin=0.5)
    second = _full_attempt(task, _candidate("reverse", 0.0), margin=0.5)
    winner = select_full([first, second])
    assert winner is first


def test_select_full_margin_tie_break_picks_larger_margin():
    task = _task([0.0, 0.01, 0.02])
    low = _full_attempt(task, _candidate("forward", 0.0), margin=0.2)
    high = _full_attempt(task, _candidate("forward", 30.0), margin=0.9)
    assert select_full([low, high]) is high
    assert select_full([high, low]) is high


def test_select_fallback_prefers_largest_last_reached_index():
    task = _task([0.0, 0.01, 0.02, 0.03])
    worse = _partial_attempt(task, _candidate("forward", 0.0), last_idx=0, margin=0.9)
    better = _partial_attempt(task, _candidate("forward", 30.0), last_idx=2, margin=0.1)
    assert select_fallback([worse, better]) is better


def test_select_fallback_margin_tie_break_within_largest_index_group():
    task = _task([0.0, 0.01, 0.02, 0.03])
    low = _partial_attempt(task, _candidate("forward", 0.0), last_idx=2, margin=0.1)
    high = _partial_attempt(task, _candidate("forward", 30.0), last_idx=2, margin=0.8)
    assert select_fallback([low, high]) is high


def test_select_fallback_none_when_all_fail_path():
    task = _task([0.0, 0.01, 0.02])
    attempts = [_failed_attempt(task, _candidate(), "ik_failed"), _failed_attempt(task, _candidate("reverse"), "transit_failed")]
    assert select_fallback(attempts) is None


def test_aggregate_reasons_is_deduplicated_and_sorted():
    task = _task([0.0, 0.01])
    attempts = [_failed_attempt(task, _candidate(), "ik_failed"), _failed_attempt(task, _candidate("reverse"), "ik_failed"), _failed_attempt(task, _candidate("reverse", 30), "transit_failed")]
    assert aggregate_reasons(attempts) == ["ik_failed", "transit_failed"]


# --------------------------------------------------------------------------------------
# run_segment_splits (§13.5-13.7 orchestration; attempt_fn/synth_retract_fn/split_fn injected)
# --------------------------------------------------------------------------------------

def test_run_segment_splits_full_completion_first_round():
    task = _task([0.0, 0.01, 0.02])
    cand = _candidate()

    def attempt_fn(t, positions):
        return [_full_attempt(t, cand, margin=0.5)]

    def synth_retract_fn(rolled, idx, positions):
        raise AssertionError("must not be called on a full first-round completion")

    leg_specs, report_specs, final = run_segment_splits(
        task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, min_waypoints_per_remainder=3,
    )
    assert len(report_specs) == 1
    r = report_specs[0]
    assert r["trace_waypoints_total"] == 3
    assert r["trace_waypoints_planned"] == 3
    assert r["chosen"] == {"direction": "forward", "roll_offset_rad": 0.0}
    assert r["reasons"] == []
    assert [spec["kind"] for spec in leg_specs] == ["transit", "approach", "trace", "retract"]
    assert final == _joint_dict(_zeros6())


def test_run_segment_splits_all_fail_path():
    task = _task([0.0, 0.01, 0.02])

    def attempt_fn(t, positions):
        return [_failed_attempt(t, _candidate(), "ik_failed"), _failed_attempt(t, _candidate("reverse"), "transit_failed")]

    def synth_retract_fn(rolled, idx, positions):
        raise AssertionError("must not be called when no candidate reaches anything")

    leg_specs, report_specs, final = run_segment_splits(
        task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, min_waypoints_per_remainder=3,
    )
    assert leg_specs == []
    assert len(report_specs) == 1
    r = report_specs[0]
    assert r["trace_waypoints_planned"] == 0
    assert r["chosen"] is None
    assert r["reasons"] == ["ik_failed", "transit_failed"]
    assert final == _joint_dict(_zeros6())


def test_run_segment_splits_partial_then_split_then_full():
    """A partial first round (reaches waypoint 0 of 3) synthesises a retract, splits off the
    2-waypoint remainder (passing `min_waypoints_per_remainder=1` here so the 2-waypoint
    remainder clears the gate; the injected `split_fn` itself is what actually builds it), and
    the second round fully completes it."""
    task = _task([0.0, 0.01, 0.02])
    remainder = _task([0.01, 0.02], sub_segment=0)
    cand = _candidate()

    def attempt_fn(t, positions):
        if t is task:
            return [_partial_attempt(t, cand, last_idx=0, margin=0.3, reasons={"collision"})]
        assert t is remainder
        return [_full_attempt(t, cand, margin=0.6)]

    retract_leg = _leg("retract", 1)

    def synth_retract_fn(rolled, idx, positions):
        assert idx == 0
        return retract_leg

    def split_fn(rolled, idx):
        assert idx == 0
        return remainder

    leg_specs, report_specs, final = run_segment_splits(
        task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, split_fn, min_waypoints_per_remainder=1,
    )

    assert len(report_specs) == 2
    first, second = report_specs
    assert first["trace_waypoints_total"] == 3
    assert first["trace_waypoints_planned"] == 1
    assert first["reasons"] == ["collision"]
    assert second["segment_ref"]["sub_segment"] == 0
    assert second["trace_waypoints_total"] == 2
    assert second["trace_waypoints_planned"] == 2
    kinds = [spec["kind"] for spec in leg_specs]
    assert kinds == ["transit", "approach", "trace", "retract", "transit", "approach", "trace", "retract"]


def test_run_segment_splits_too_short_remainder_not_attempted():
    """The 2-waypoint remainder (3 total, waypoint 0 reached) is below the default
    `min_waypoints_per_remainder` of 3, so `split_fn` is never even consulted."""
    task = _task([0.0, 0.01, 0.02])
    cand = _candidate()

    def attempt_fn(t, positions):
        return [_partial_attempt(t, cand, last_idx=0, margin=0.3)]

    def synth_retract_fn(rolled, idx, positions):
        return _leg("retract", 1)

    def split_fn(rolled, idx):
        raise AssertionError("must not be called: the remainder is shorter than min_waypoints_per_remainder")

    leg_specs, report_specs, final = run_segment_splits(
        task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, split_fn, min_waypoints_per_remainder=3,
    )

    assert len(report_specs) == 1
    assert "too_short" in report_specs[0]["reasons"]
    assert report_specs[0]["trace_waypoints_planned"] == 1


def test_run_segment_splits_stops_at_max_splits_without_too_short():
    task = _task([0.0, 0.01, 0.02])
    cand = _candidate()
    split_calls = []

    def attempt_fn(t, positions):
        return [_partial_attempt(t, cand, last_idx=0, margin=0.3)]

    def synth_retract_fn(rolled, idx, positions):
        return _leg("retract", 1)

    def split_fn(rolled, idx):
        split_calls.append(idx)
        return _task([0.01, 0.02])

    leg_specs, report_specs, final = run_segment_splits(
        task, _joint_dict(_zeros6()), 0, attempt_fn, synth_retract_fn, split_fn, min_waypoints_per_remainder=3,
    )

    assert split_calls == []  # max_splits_per_segment=0: never even tries to split
    assert len(report_specs) == 1
    assert "too_short" not in report_specs[0]["reasons"]


def test_run_segment_splits_synth_retract_failure_degrades_reason_and_skips_leg():
    task = _task([0.0, 0.01, 0.02])
    cand = _candidate()

    def attempt_fn(t, positions):
        return [_partial_attempt(t, cand, last_idx=0, margin=0.3)]

    def synth_retract_fn(rolled, idx, positions):
        return None  # the synthesised retract itself failed its own re-check

    def split_fn(rolled, idx):
        return None

    leg_specs, report_specs, final = run_segment_splits(
        task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, split_fn, min_waypoints_per_remainder=1,
    )

    assert "retract" not in [spec["kind"] for spec in leg_specs]
    assert "cartesian_incomplete" in report_specs[0]["reasons"]


def test_run_segment_splits_too_short_uses_cfg_min_waypoints_per_remainder():
    """Finding 2d382ffda7d1d567: the too_short decision must use the caller's own
    `splitting.min_waypoints_per_remainder`, never `cartesian_core`'s hard-coded minimum (3) --
    here a non-default value of 4 rejects the same 3-waypoint remainder a value of 3 accepts."""
    task = _task([0.0, 0.01, 0.02, 0.03, 0.04])  # 5 waypoints; reaching index 1 leaves 3
    cand = _candidate()

    def attempt_fn(t, positions):
        return [_partial_attempt(t, cand, last_idx=1, margin=0.3)]

    def synth_retract_fn(rolled, idx, positions):
        return _leg("retract", 1)

    split_calls = []

    def split_fn(rolled, idx):
        split_calls.append(idx)
        return _task([0.03, 0.04, 0.05], sub_segment=0)

    run_segment_splits(
        task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, split_fn, min_waypoints_per_remainder=3,
    )
    assert split_calls == [1]

    split_calls.clear()
    _leg_specs, report_specs, _final = run_segment_splits(
        task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, split_fn, min_waypoints_per_remainder=4,
    )
    assert split_calls == []  # the 3-waypoint remainder is below this non-default min of 4
    assert "too_short" in report_specs[0]["reasons"]


def test_run_segment_splits_retry_remainder_uses_unrolled_base_quaternions():
    """Finding 11ab2dba7b6b1849: a split retry's remainder must be built from paths3d's own base
    poses, never the previous round's rolled/reversed ones -- a roll-0 forward candidate applied
    to the remainder must reproduce the original task's own base quaternions exactly, not the
    30 deg the first round's candidate actually rolled by."""
    task = _task([0.0, 0.01, 0.02, 0.03])
    roll_30 = _candidate("forward", 30.0)
    roll_0 = _candidate("forward", 0.0)
    rolled_30 = apply_candidate(task, roll_30.direction, roll_30.roll_offset_rad)
    checked_quats = []

    def attempt_fn(t, positions):
        if t is task:
            return [
                CandidateAttempt(
                    candidate=roll_30, rolled_task=rolled_30,
                    legs=[_leg("transit"), _leg("approach", 1), _leg("trace", 1)],
                    last_reached_index=0, full=False, margin=0.3,
                    final_positions=_joint_dict(_zeros6()), reasons=set(),
                )
            ]
        rolled = apply_candidate(t, roll_0.direction, roll_0.roll_offset_rad)
        checked_quats.append(rolled.trace[0].quat_xyzw)
        return [_full_attempt(t, roll_0, margin=0.6)]

    def synth_retract_fn(rolled, idx, positions):
        return _leg("retract", 1)

    leg_specs, report_specs, final = run_segment_splits(
        task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, min_waypoints_per_remainder=1,
    )

    assert len(report_specs) == 2
    assert checked_quats == [pytest.approx(task.trace[1].quat_xyzw)]


def test_run_segment_splits_retry_after_reverse_partial_uses_paths3d_order():
    """Finding c6af65a5745b06c4: after a reverse (rolled) partial reaching index k of the reversed
    traversal, the remainder is the original prefix trace[0:N-1-k] in paths3d order, un-rolled --
    the retry's forward roll-0 candidate must reproduce exactly those paths3d base poses, and the
    retry's reported `chosen` direction is relative to paths3d."""
    base = _task([0.0, 0.01, 0.02, 0.03, 0.04])
    # distinct per-waypoint base orientations (yaw about +z), so both order and roll are observable
    quats = tuple((0.0, 0.0, math.sin(math.radians(10 * i) / 2), math.cos(math.radians(10 * i) / 2)) for i in range(5))
    task = base._replace(trace=tuple(Pose(p.position_m, q) for p, q in zip(base.trace, quats)))
    rev_30 = _candidate("reverse", 30.0)
    fwd_0 = _candidate("forward", 0.0)
    rolled = apply_candidate(task, rev_30.direction, rev_30.roll_offset_rad)
    k = 0
    seen = []

    def attempt_fn(t, positions):
        if t is task:
            return [
                CandidateAttempt(
                    candidate=rev_30, rolled_task=rolled,
                    legs=[_leg("transit"), _leg("approach", 1), _leg("trace", k + 1)],
                    last_reached_index=k, full=False, margin=0.3,
                    final_positions=_joint_dict(_zeros6()), reasons=set(),
                )
            ]
        seen.append(apply_candidate(t, fwd_0.direction, fwd_0.roll_offset_rad))
        return [_full_attempt(t, fwd_0, margin=0.6)]

    def synth_retract_fn(rolled_task, idx, positions):
        return _leg("retract", 1)

    _, report_specs, _ = run_segment_splits(
        task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, min_waypoints_per_remainder=1,
    )

    expected = task.trace[0:len(task.trace) - 1 - k]
    assert len(seen) == 1
    retry = seen[0]
    assert len(retry.trace) == len(expected) == 4
    for got, want in zip(retry.trace, expected):
        assert got.position_m == pytest.approx(want.position_m)
        assert got.quat_xyzw == pytest.approx(want.quat_xyzw)
    assert retry.approach.position_m[0] == pytest.approx(expected[0].position_m[0])
    assert retry.retract.position_m[0] == pytest.approx(expected[-1].position_m[0])
    assert len(report_specs) == 2
    assert report_specs[1]["segment_ref"]["sub_segment"] == 0
    assert report_specs[1]["chosen"]["direction"] == "forward"
    assert report_specs[1]["chosen"]["roll_offset_rad"] == pytest.approx(0.0)


# --------------------------------------------------------------------------------------
# small pure helpers
# --------------------------------------------------------------------------------------

def test_within_limits():
    assert _within_limits([_zeros6()], _LIMITS)
    bad = [10.0] + [0.0] * 5
    assert not _within_limits([bad], _LIMITS)


def test_min_joint_margin():
    limits = {n: {"lower": -1.0, "upper": 1.0} for n in JOINT_NAMES}
    configs = [[0.5] * 6, [0.9] * 6]
    assert _min_joint_margin(configs, limits) == pytest.approx(0.1)


def test_max_joint_step():
    configs = [[0.0] * 6, [0.0, 0.2, 0.0, 0.0, 0.0, 0.0], [0.0, 0.2, 0.0, 0.0, 0.0, 0.5]]
    assert _max_joint_step(configs) == pytest.approx(0.5)
    assert _max_joint_step([[0.0] * 6]) == 0.0


def test_pose_errors_zero_for_identical_poses():
    p = _pose(0.1, 0.2, 0.3)
    pos_err, ang_err = _pose_errors(p, p)
    assert pos_err == pytest.approx(0.0)
    assert ang_err == pytest.approx(0.0)


def test_index_map_and_errors_matches_in_order():
    targets = [_pose(0.0), _pose(0.01), _pose(0.02)]
    fk_poses = [targets[0], targets[1], _pose(5.0), targets[2]]
    index_map, max_pos_err, max_ang_err = _index_map_and_errors(fk_poses, targets, tol_m=0.001, tol_deg=1.0)
    assert index_map == [0, 1, None, 2]
    # Finding 8e9a7e822fe1b8a8: the far-off, unmatched config must never inflate the diagnostic
    # max -- only FK-matched (and therefore within-tolerance) configurations count.
    assert max_pos_err <= 0.001
    assert max_ang_err <= 1.0


def test_index_map_and_errors_approach_leg_interpolated_intermediates_within_tolerance():
    """Finding 8e9a7e822fe1b8a8's own scenario: an approach leg interpolated in from z=0.04 down
    to the trace pose at z=0.01 must never record a recheck error above the FK tolerance -- the
    interpolated intermediates never match (and so never count), only the final configuration
    that actually reaches the target does."""
    target = [_pose(z=0.01)]
    fk_poses = [_pose(z=0.04), _pose(z=0.025), _pose(z=0.01)]
    index_map, max_pos_err, max_ang_err = _index_map_and_errors(fk_poses, target, tol_m=0.001, tol_deg=1.0)
    assert index_map == [None, None, 0]
    assert max_pos_err <= 0.001
    assert max_ang_err <= 1.0


def test_prefix_length_m():
    trace = [_pose(0.0), _pose(0.01), _pose(0.03)]
    assert _prefix_length_m(trace, 0) == 0.0
    assert _prefix_length_m(trace, 1) == 0.0
    assert _prefix_length_m(trace, 3) == pytest.approx(0.03)


# --------------------------------------------------------------------------------------
# _polyline_index_map_and_errors (docs/INTERFACES.md §15, ADR-017 addendum, MOT-06.11): the
# fraction==1.0 full-trace FK rule, replacing the §13.7 pointer walk for that case only.
# --------------------------------------------------------------------------------------

def test_nearest_point_on_segment_midpoint():
    d, t = _nearest_point_on_segment((0.005, 0.001, 0.0), (0.0, 0.0, 0.0), (0.01, 0.0, 0.0))
    assert d == pytest.approx(0.001)
    assert t == pytest.approx(0.5)


def test_nearest_point_on_segment_clamps_past_endpoints():
    d, t = _nearest_point_on_segment((-0.01, 0.0, 0.0), (0.0, 0.0, 0.0), (0.01, 0.0, 0.0))
    assert t == pytest.approx(0.0)
    assert d == pytest.approx(0.01)


def test_slerp_quat_endpoints_and_midpoint():
    q0 = (0.0, 0.0, 0.0, 1.0)
    q1 = (0.0, 0.0, 1.0, 0.0)  # 180 deg about Z
    assert _slerp_quat(q0, q1, 0.0) == pytest.approx(q0)
    assert _slerp_quat(q0, q1, 1.0) == pytest.approx(q1)
    mid = _slerp_quat(q0, q1, 0.5)
    assert _quat_angle_deg(mid, q0) == pytest.approx(90.0, abs=1e-6)


def test_polyline_index_map_full_trace_with_widely_spaced_resampled_configs():
    """The regression this card exists to fix: a fraction==1.0 trace whose returned
    configurations are resampled far more sparsely than the requested waypoints (consecutive
    configs here are 0.008 m apart, 8x `2*fk_tolerance_m`) must still be judged `full` -- every
    configuration lies on the requested polyline within tolerance, which is all this rule checks."""
    targets = [_pose(x=x) for x in (0.0, 0.002, 0.004, 0.006, 0.008)]
    fk_poses = [_pose(x=0.0), _pose(x=0.008)]  # 2 configs for 5 requested waypoints
    index_map, max_pos_err, max_ang_err, ok = _polyline_index_map_and_errors(
        fk_poses, targets, tol_m=0.001, tol_deg=1.0,
    )
    assert ok is True
    assert index_map == [0, 4]  # nearest-vertex diagnostic; last forced to the final waypoint
    assert max_pos_err == pytest.approx(0.0, abs=1e-9)
    assert max_ang_err == pytest.approx(0.0, abs=1e-9)


def test_polyline_index_map_mid_segment_config_passes_on_the_line():
    """A config that lands strictly between two requested waypoints (not at either vertex) is
    still judged by its perpendicular distance to the segment, not forced to match one vertex."""
    targets = [_pose(x=0.0), _pose(x=0.01), _pose(x=0.02)]
    fk_poses = [_pose(x=0.005), _pose(x=0.02)]  # mid-segment config, then the final waypoint
    index_map, max_pos_err, _max_ang_err, ok = _polyline_index_map_and_errors(
        fk_poses, targets, tol_m=0.001, tol_deg=1.0,
    )
    assert ok is True
    assert max_pos_err == pytest.approx(0.0, abs=1e-9)
    assert index_map == [1, 2]


def test_polyline_index_map_single_trace_waypoint_degenerates_to_point_check():
    targets = [_pose(x=0.0)]
    fk_poses = [_pose(x=0.0005)]
    index_map, max_pos_err, _max_ang_err, ok = _polyline_index_map_and_errors(
        fk_poses, targets, tol_m=0.001, tol_deg=1.0,
    )
    assert ok is True
    assert index_map == [0]
    assert max_pos_err == pytest.approx(0.0005)


def test_polyline_index_map_deviating_config_fails():
    targets = [_pose(x=x) for x in (0.0, 0.002, 0.004, 0.006, 0.008)]
    fk_poses = [_pose(x=0.0), _pose(x=0.004, y=0.05), _pose(x=0.008)]  # middle config far off the line
    _index_map, _max_pos_err, _max_ang_err, ok = _polyline_index_map_and_errors(
        fk_poses, targets, tol_m=0.001, tol_deg=1.0,
    )
    assert ok is False


def test_polyline_index_map_final_config_must_match_last_waypoint():
    """Even if every config lies on the polyline somewhere, the final one must land on the *last*
    waypoint specifically -- landing short (e.g. only reaching the second-to-last vertex) fails."""
    targets = [_pose(x=x) for x in (0.0, 0.002, 0.004)]
    fk_poses = [_pose(x=0.0), _pose(x=0.002)]  # never reaches x=0.004
    _index_map, _max_pos_err, _max_ang_err, ok = _polyline_index_map_and_errors(
        fk_poses, targets, tol_m=0.001, tol_deg=1.0,
    )
    assert ok is False


def test_synthesized_retract_pose_offset_matches_approach_standoff():
    task = _task([0.0, 0.01, 0.02])
    pose = _synthesized_retract_pose(task, last_idx=1)
    standoff = math.dist(task.approach.position_m, task.surface_points[0])
    assert pose.position_m[2] == pytest.approx(standoff)
    assert pose.quat_xyzw == task.trace[1].quat_xyzw


# --------------------------------------------------------------------------------------
# _classify_shortfall (§13.8's fraction<1 collision/joint_jump attribution), fake service results
# --------------------------------------------------------------------------------------

@pytest.fixture()
def cfg():
    return load_config(PRODUCTION_PLANNING_CONFIG)


class _FakeMoveIt:
    """A minimal `MoveItClients`-shaped double: `.clients[key]` is `key` itself, so `.call()` can
    dispatch purely on the key `_classify_shortfall` indexes with; each key's canned responses are
    returned in order. No `rclpy`/running ROS graph involved -- every response is a plain
    `moveit_msgs` message built directly, exactly as test_cartesian_moveit.py does."""

    def __init__(self, responses: Dict[str, List[Any]]):
        self.clients = {key: key for key in responses}
        self._queues = {key: list(resp) for key, resp in responses.items()}

    def call(self, client, request, timeout_s=None):
        return self._queues[client].pop(0)


def _ik_response(values=None, error_code=MoveItErrorCodes.SUCCESS):
    resp = GetPositionIK.Response()
    resp.error_code.val = error_code
    if values is not None:
        resp.solution.joint_state.name = list(JOINT_NAMES)
        resp.solution.joint_state.position = list(values)
    return resp


def _validity_response(valid: bool):
    resp = GetStateValidity.Response()
    resp.valid = valid
    return resp


def test_classify_shortfall_ik_failure_stays_cartesian_incomplete(cfg):
    moveit = _FakeMoveIt({"ik": [_ik_response(error_code=MoveItErrorCodes.NO_IK_SOLUTION)]})
    reason = _classify_shortfall(moveit, cfg, _joint_dict(_zeros6()), _pose(0.1))
    assert reason == "cartesian_incomplete"


def test_classify_shortfall_invalid_ik_solution_is_collision(cfg):
    moveit = _FakeMoveIt({
        "ik": [_ik_response(_zeros6())],
        "validity": [_validity_response(False)],
    })
    reason = _classify_shortfall(moveit, cfg, _joint_dict(_zeros6()), _pose(0.1))
    assert reason == "collision"


def test_classify_shortfall_big_joint_step_is_joint_jump(cfg):
    big_step = [1.0] + [0.0] * 5  # exceeds recheck.max_joint_step_rad (0.2 rad)
    moveit = _FakeMoveIt({
        "ik": [_ik_response(big_step)],
        "validity": [_validity_response(True)],
    })
    reason = _classify_shortfall(moveit, cfg, _joint_dict(_zeros6()), _pose(0.1))
    assert reason == "joint_jump"


def test_classify_shortfall_small_joint_step_stays_cartesian_incomplete(cfg):
    moveit = _FakeMoveIt({
        "ik": [_ik_response(_zeros6())],
        "validity": [_validity_response(True)],
    })
    reason = _classify_shortfall(moveit, cfg, _joint_dict(_zeros6()), _pose(0.1))
    assert reason == "cartesian_incomplete"


# --------------------------------------------------------------------------------------
# _attempt_candidate end-to-end with fake services (§13.5 steps 1-5), proving the fraction==1.0
# full-trace fix (docs/INTERFACES.md §15, ADR-017 addendum, MOT-06.11) at the level the card
# requires: fake services drive the real step 1-5 control flow, no rclpy/running ROS graph.
# --------------------------------------------------------------------------------------

def _traj(names, points_positions):
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names = list(names)
    for positions in points_positions:
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        traj.joint_trajectory.points.append(pt)
    return traj


def _motion_plan_response(configs):
    resp = GetMotionPlan.Response()
    resp.motion_plan_response.error_code.val = MoveItErrorCodes.SUCCESS
    resp.motion_plan_response.trajectory = _traj(JOINT_NAMES, configs)
    return resp


def _cartesian_response(configs, fraction=1.0):
    resp = GetCartesianPath.Response()
    resp.error_code.val = MoveItErrorCodes.SUCCESS
    resp.fraction = fraction
    resp.solution = _traj(JOINT_NAMES, configs)
    return resp


def _fk_response(pose: Pose):
    resp = GetPositionFK.Response()
    resp.error_code.val = MoveItErrorCodes.SUCCESS
    resp.fk_link_names = ["tool_tip"]
    ps = PoseStamped()
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pose.position_m
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = pose.quat_xyzw
    resp.pose_stamped = [ps]
    return resp


_TRACE_XS = [0.0, 0.002, 0.004, 0.006, 0.008]  # 5 waypoints, matches paths3d's own default spacing


def _full_trace_fake_moveit(cfg, *, trace_fk_poses, trace_fraction=1.0, extra_ik=()):
    """Fake services for one complete §13.5 steps 1-5 pass over `_task(_TRACE_XS)`: IK+transit
    succeed trivially, the Cartesian approach/retract legs are single points at the first/last
    trace pose, and the Cartesian trace leg returns `trace_fraction` with exactly `trace_fk_poses`'
    own configurations (so its own FK responses are `trace_fk_poses`, however sparse)."""
    task = _task(_TRACE_XS)
    n = len(trace_fk_poses)
    # small per-config steps (0.01 rad): distinct configs, but well under recheck.max_joint_step_rad
    # (0.2) regardless of how sparse `n` is.
    trace_configs = [[float(i) * 0.01] * 6 for i in range(n)]
    return _FakeMoveIt({
        "ik": [_ik_response(_zeros6()), *extra_ik],
        "motion_plan": [_motion_plan_response([_zeros6(), _zeros6()])],
        "cartesian": [
            _cartesian_response([_zeros6()]),  # step 3: approach to trace[0]
            _cartesian_response(trace_configs, fraction=trace_fraction),  # step 4: the trace leg under test
            _cartesian_response([_zeros6()]),  # step 5: retract
        ],
        "fk": [
            _fk_response(task.approach),  # step 2: transit final pose check
            _fk_response(task.trace[0]),  # step 3: approach config FK
            *[_fk_response(p) for p in trace_fk_poses],  # step 4: trace config(s) FK
            _fk_response(task.retract),  # step 5: retract config FK
        ],
        "validity": [_validity_response(True)] * (2 + 1 + n + 1),  # transit(2)+approach(1)+trace(n)+retract(1)
    })


def test_attempt_candidate_full_trace_with_widely_resampled_configs_is_full(cfg):
    """The regression this card exists to fix: a `fraction == 1.0` trace leg whose returned
    configurations are resampled far more sparsely than the requested waypoints (2 configs for 5
    waypoints here, matching every intermediate FK-matched position exactly) must still be
    reported `full` -- the old §13.7 pointer walk would stall partway through this (ADR-017
    addendum measured it directly on a live mock-stack trace)."""
    task = _task(_TRACE_XS)
    trace_fk_poses = [task.trace[0], task.trace[-1]]
    moveit = _full_trace_fake_moveit(cfg, trace_fk_poses=trace_fk_poses)
    cand = _candidate("forward", 0.0)
    limits = _LIMITS

    attempt = _attempt_candidate(moveit, cfg, limits, _joint_dict(_zeros6()), task, cand)

    assert attempt.full is True
    assert attempt.last_reached_index == len(task.trace) - 1
    trace_leg = next(leg for leg in attempt.legs if leg.kind == "trace")
    assert trace_leg.waypoint_index_map == [0, len(task.trace) - 1]
    assert trace_leg.recheck["max_pos_err_m"] == pytest.approx(0.0, abs=1e-9)
    assert attempt.reasons == set()


def test_attempt_candidate_fraction_below_one_still_uses_pointer_walk_cutoff(cfg):
    """§13.7's own fraction<1 behaviour is unchanged by this card: a trace leg that only reaches
    waypoint index 1 of 4 is reported partial, at exactly that cutoff."""
    task = _task(_TRACE_XS)
    # two configs, FK-matching trace[0] then trace[1] exactly -- a fraction<1 partial reach, plus
    # the extra /compute_ik call _classify_shortfall makes for the shortfall classification.
    trace_fk_poses = [task.trace[0], task.trace[1]]
    moveit = _full_trace_fake_moveit(
        cfg, trace_fk_poses=trace_fk_poses, trace_fraction=0.4,
        extra_ik=[_ik_response(error_code=MoveItErrorCodes.NO_IK_SOLUTION)],
    )
    cand = _candidate("forward", 0.0)
    limits = _LIMITS

    attempt = _attempt_candidate(moveit, cfg, limits, _joint_dict(_zeros6()), task, cand)

    assert attempt.full is False
    assert attempt.last_reached_index == 1
    trace_leg = next(leg for leg in attempt.legs if leg.kind == "trace")
    assert trace_leg.waypoint_index_map == [0, 1]
    assert "cartesian_incomplete" in attempt.reasons


def test_attempt_candidate_full_trace_fk_mismatch_fails_whole_candidate(cfg):
    """One deviating configuration among an otherwise `fraction == 1.0` trace fails the whole
    candidate with `fk_mismatch` -- never a partial salvage (that is §13.7's job, fraction<1 only)."""
    task = _task(_TRACE_XS)
    trace_fk_poses = [task.trace[0], _pose(x=0.004, y=0.05), task.trace[-1]]
    moveit = _full_trace_fake_moveit(cfg, trace_fk_poses=trace_fk_poses)
    cand = _candidate("forward", 0.0)
    limits = _LIMITS

    attempt = _attempt_candidate(moveit, cfg, limits, _joint_dict(_zeros6()), task, cand)

    assert attempt.full is False
    assert attempt.last_reached_index == -1
    assert attempt.legs == []
    assert attempt.reasons == {"fk_mismatch"}


# --------------------------------------------------------------------------------------
# CLI: --dry-run only (no rclpy.init, no ROS, no file write) -- §13.15
# --------------------------------------------------------------------------------------

def test_dry_run_exits_ok_and_never_touches_rclpy(tmp_path, monkeypatch):
    import crackvision_motion.plan_crack_path as module

    def _boom(*a, **kw):
        raise AssertionError("--dry-run must never call rclpy.init")

    monkeypatch.setattr(module.rclpy, "init", _boom)
    monkeypatch.chdir(REPO_ROOT)

    exit_code = main(["--dry-run", "--paths3d", str(MULTI_COMPONENT_FIXTURE), "--root", str(REPO_ROOT)])
    assert exit_code == EXIT_OK


def test_dry_run_writes_no_plan_file(tmp_path, monkeypatch):
    monkeypatch.chdir(REPO_ROOT)
    out_path = tmp_path / "would_not_exist.json"
    exit_code = main([
        "--dry-run", "--paths3d", str(MULTI_COMPONENT_FIXTURE), "--root", str(REPO_ROOT), "--out", str(out_path),
    ])
    assert exit_code == EXIT_OK
    assert not out_path.exists()


def test_dry_run_bad_planning_config_is_usage_error():
    exit_code = main([
        "--dry-run", "--paths3d", str(MULTI_COMPONENT_FIXTURE), "--root", str(REPO_ROOT),
        "--planning-config", "/nonexistent-cartesian-planning.yaml",
    ])
    assert exit_code == EXIT_USAGE


def test_dry_run_bad_paths3d_is_precondition_refusal():
    exit_code = main(["--dry-run", "--paths3d", "/nonexistent-paths3d.json", "--root", str(REPO_ROOT)])
    assert exit_code == EXIT_PRECONDITION


def test_dry_run_bad_scene_config_is_usage_error():
    exit_code = main([
        "--dry-run", "--paths3d", str(MULTI_COMPONENT_FIXTURE), "--root", str(REPO_ROOT),
        "--scene-config", "/nonexistent-scene.yaml",
    ])
    assert exit_code == EXIT_USAGE


def test_no_action_client_import_anywhere_in_module():
    """§13's own preamble + this card's acceptance criteria: nothing in this file can command or
    execute a trajectory."""
    source = Path(
        Path(__file__).resolve().parents[1] / "crackvision_motion" / "plan_crack_path.py"
    ).read_text()
    assert "ActionClient" not in source
    assert "FollowJointTrajectory" not in source
    assert "control_msgs" not in source

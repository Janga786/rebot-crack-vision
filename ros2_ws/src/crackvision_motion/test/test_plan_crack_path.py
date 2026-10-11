"""Unit tests for plan_crack_path (MOT-06.6). Runs without a colcon build or a running ROS graph
-- the selection/splitting logic is tested with fake planner results (`attempt_fn`/`synth_retract_fn`/
`split_fn` injected), never a real MoveIt call; the CLI-level tests only ever exercise `--dry-run`
(no `rclpy.init`, no file write).

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py -q -p no:cacheprovider
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from crackvision_motion.cartesian_core import Candidate, Pose, SegmentTask
from crackvision_motion.cli_common import EXIT_OK, EXIT_PRECONDITION, EXIT_USAGE
from crackvision_motion.joint_trajectory import JOINT_NAMES
from crackvision_motion.plan_crack_path import (
    CandidateAttempt,
    LegResult,
    _index_map_and_errors,
    _joint_dict,
    _max_joint_step,
    _min_joint_margin,
    _pose_errors,
    _prefix_length_m,
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

    leg_specs, report_specs, final = run_segment_splits(task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn)
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

    leg_specs, report_specs, final = run_segment_splits(task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn)
    assert leg_specs == []
    assert len(report_specs) == 1
    r = report_specs[0]
    assert r["trace_waypoints_planned"] == 0
    assert r["chosen"] is None
    assert r["reasons"] == ["ik_failed", "transit_failed"]
    assert final == _joint_dict(_zeros6())


def test_run_segment_splits_partial_then_split_then_full():
    """A partial first round (reaches waypoint 0 of 3) synthesises a retract, splits off the
    2-waypoint remainder (>= min_waypoints_per_remainder would normally be 3, but the injected
    split_fn controls this directly), and the second round fully completes it."""
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

    leg_specs, report_specs, final = run_segment_splits(task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, split_fn)

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
    task = _task([0.0, 0.01, 0.02])
    cand = _candidate()

    def attempt_fn(t, positions):
        return [_partial_attempt(t, cand, last_idx=0, margin=0.3)]

    def synth_retract_fn(rolled, idx, positions):
        return _leg("retract", 1)

    def split_fn(rolled, idx):
        return None  # too_short

    leg_specs, report_specs, final = run_segment_splits(task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, split_fn)

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

    leg_specs, report_specs, final = run_segment_splits(task, _joint_dict(_zeros6()), 0, attempt_fn, synth_retract_fn, split_fn)

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

    leg_specs, report_specs, final = run_segment_splits(task, _joint_dict(_zeros6()), 3, attempt_fn, synth_retract_fn, split_fn)

    assert "retract" not in [spec["kind"] for spec in leg_specs]
    assert "cartesian_incomplete" in report_specs[0]["reasons"]


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
    assert max_pos_err > 1.0  # the far-off config dominates the diagnostic max


def test_prefix_length_m():
    trace = [_pose(0.0), _pose(0.01), _pose(0.03)]
    assert _prefix_length_m(trace, 0) == 0.0
    assert _prefix_length_m(trace, 1) == 0.0
    assert _prefix_length_m(trace, 3) == pytest.approx(0.03)


def test_synthesized_retract_pose_offset_matches_approach_standoff():
    task = _task([0.0, 0.01, 0.02])
    pose = _synthesized_retract_pose(task, last_idx=1)
    standoff = math.dist(task.approach.position_m, task.surface_points[0])
    assert pose.position_m[2] == pytest.approx(standoff)
    assert pose.quat_xyzw == task.trace[1].quat_xyzw


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

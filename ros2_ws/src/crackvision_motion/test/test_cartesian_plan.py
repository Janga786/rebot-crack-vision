"""Unit tests for cartesian_plan (MOT-06.4). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_cartesian_plan.py -q -p no:cacheprovider
"""

import copy
import hashlib
import json
from pathlib import Path

import pytest

from crackvision_motion.cartesian_plan import (
    CONTINUITY_TOL_RAD,
    CartesianPlanError,
    PLAN_SCHEMA,
    VIEW_SCHEMA,
    add_leg,
    build_leg,
    add_segment_report,
    add_view_candidate,
    build_caveats,
    finalize,
    finalize_view,
    load_plan,
    load_view_plan,
    new_plan,
    new_view_plan,
    validate_plan,
    validate_view_plan,
    write_plan,
    write_view_plan,
)
from crackvision_motion.joint_trajectory import JOINT_NAMES, load_limits

REPO_ROOT = Path(__file__).resolve().parents[4]
LIMITS_PATH = REPO_ROOT / "config" / "robot" / "b601_dm_limits.yaml"
LIMITS = load_limits(LIMITS_PATH)

_ZERO6 = [0.0] * 6


class _Task:
    """Minimal stand-in for cartesian_core.SegmentTask -- finalize() only reads .segment_ref."""

    def __init__(self, segment_ref):
        self.segment_ref = segment_ref


def _sha(data: bytes = b"x") -> str:
    return hashlib.sha256(data).hexdigest()


def _seg_ref(segment_index=0, sub_segment=None, component_id="crack0", polyline_kind="main", branch_index=None):
    return {
        "component_id": component_id,
        "polyline_kind": polyline_kind,
        "branch_index": branch_index,
        "segment_index": segment_index,
        "sub_segment": sub_segment,
    }


def _recheck(max_pos_err_m=0.0001, max_ang_err_rad=0.001, max_joint_step_rad=0.01):
    return {
        "max_pos_err_m": max_pos_err_m,
        "max_ang_err_rad": max_ang_err_rad,
        "all_valid": True,
        "within_limits": True,
        "max_joint_step_rad": max_joint_step_rad,
    }


def _sources():
    return {
        "paths3d": {"path": "data/paths3d/x.json", "sha256": _sha(b"p3d"), "execution_eligible": True},
        "scene": {"path": "config/scene/scene.yaml", "sha256": _sha(b"scene")},
        "end_effector": {"path": "config/robot/end_effector.yaml", "sha256": _sha(b"ee")},
        "limits": {"path": str(LIMITS_PATH), "sha256": _sha(LIMITS_PATH.read_bytes())},
        "config": {"path": "config/motion/cartesian_planning.yaml", "sha256": _sha(b"cfg")},
    }


def _parameters():
    return {"schema": "crackvision.cartesian_planning_config/1", "end_state": "none"}


def _start_state():
    return {"joint_names": list(JOINT_NAMES), "positions_rad": list(_ZERO6), "source": "capture_record"}


def _end_state():
    return {"mode": "none", "positions_rad": None}


def _paths3d_doc(points=100, valid=90, invalid_by_reason=None, execution_eligible=True):
    return {
        "counts": {
            "points": points,
            "valid": valid,
            "invalid_by_reason": invalid_by_reason or {"depth_invalid": 10},
        },
        "execution_eligible": execution_eligible,
    }


def _bare_plan():
    return new_plan(
        case_id="case1", sources=_sources(), parameters=_parameters(), start_state=_start_state(),
        end_state=_end_state(),
    )


def _add_simple_full_leg(plan, positions=None, segment_ref=None, kind="approach", direction="forward", roll=0.0):
    positions = positions or [_ZERO6, _ZERO6]
    return add_leg(
        plan, kind, segment_ref or _seg_ref(), direction, roll, positions,
        [0] * len(positions), _recheck(),
    )


# --------------------------------------------------------------------------------------
# new_plan / add_leg
# --------------------------------------------------------------------------------------

def test_new_plan_shape():
    plan = _bare_plan()
    assert plan["schema"] == PLAN_SCHEMA
    assert plan["joint_names"] == list(JOINT_NAMES)
    assert plan["legs"] == []
    assert plan["segments"] == []


def test_add_leg_assigns_leg_index_in_order():
    plan = _bare_plan()
    leg0 = _add_simple_full_leg(plan)
    leg1 = _add_simple_full_leg(plan, positions=[_ZERO6, _ZERO6])
    assert leg0["leg_index"] == 0
    assert leg1["leg_index"] == 1


def test_add_leg_rejects_bad_kind():
    plan = _bare_plan()
    with pytest.raises(CartesianPlanError):
        add_leg(plan, "bogus", _seg_ref(), "forward", 0.0, [_ZERO6], [0], _recheck())


def test_add_leg_rejects_partial_null_triplet():
    plan = _bare_plan()
    with pytest.raises(CartesianPlanError):
        add_leg(plan, "transit", _seg_ref(), None, 0.0, [_ZERO6], [0], _recheck())


def test_add_leg_allows_trailing_transit_all_null():
    plan = _bare_plan()
    leg = add_leg(plan, "transit", None, None, None, [_ZERO6, _ZERO6], [None, None], _recheck())
    assert leg["segment_ref"] is None
    assert leg["direction"] is None


def test_add_leg_rejects_waypoint_index_map_length_mismatch():
    plan = _bare_plan()
    with pytest.raises(CartesianPlanError):
        add_leg(plan, "approach", _seg_ref(), "forward", 0.0, [_ZERO6, _ZERO6], [0], _recheck())


def test_add_leg_rejects_non_finite_position():
    plan = _bare_plan()
    bad = [0.0, 0.0, float("nan"), 0.0, 0.0, 0.0]
    with pytest.raises(CartesianPlanError):
        add_leg(plan, "approach", _seg_ref(), "forward", 0.0, [bad], [0], _recheck())


# --------------------------------------------------------------------------------------
# add_segment_report: status derivation
# --------------------------------------------------------------------------------------

def test_add_segment_report_full():
    plan = _bare_plan()
    entry = add_segment_report(plan, _seg_ref(), [], 1, {"direction": "forward", "roll_offset_rad": 0.0}, 10, 10, 1.0, 1.0)
    assert entry["status"] == "full"


def test_add_segment_report_partial():
    plan = _bare_plan()
    entry = add_segment_report(plan, _seg_ref(), ["cartesian_incomplete"], 24, {"direction": "forward", "roll_offset_rad": 0.0}, 10, 6, 1.0, 0.6)
    assert entry["status"] == "partial"


def test_add_segment_report_failed():
    plan = _bare_plan()
    entry = add_segment_report(plan, _seg_ref(), ["ik_failed"], 24, None, 10, 0, 1.0, 0.0)
    assert entry["status"] == "failed"


def test_add_segment_report_mismatch():
    plan = _bare_plan()
    entry = add_segment_report(plan, _seg_ref(), ["surface_scene_mismatch"], 0, None, 10, 0, 1.0, 0.0, mismatch=True)
    assert entry["status"] == "surface_scene_mismatch"


def test_add_segment_report_mismatch_requires_zero_candidates():
    plan = _bare_plan()
    with pytest.raises(CartesianPlanError):
        add_segment_report(plan, _seg_ref(), [], 1, None, 10, 0, 1.0, 0.0, mismatch=True)


def test_add_segment_report_failed_requires_candidates_tried():
    plan = _bare_plan()
    with pytest.raises(CartesianPlanError):
        add_segment_report(plan, _seg_ref(), [], 0, None, 10, 0, 1.0, 0.0)


def test_add_segment_report_rejects_planned_over_total():
    plan = _bare_plan()
    with pytest.raises(CartesianPlanError):
        add_segment_report(plan, _seg_ref(), [], 1, {"direction": "forward", "roll_offset_rad": 0.0}, 10, 11, 1.0, 1.0)


def test_add_segment_report_failed_rejects_chosen_set():
    plan = _bare_plan()
    with pytest.raises(CartesianPlanError):
        add_segment_report(plan, _seg_ref(), ["ik_failed"], 1, {"direction": "forward", "roll_offset_rad": 0.0}, 10, 0, 1.0, 0.0)


def test_add_segment_report_failed_rejects_empty_reasons():
    plan = _bare_plan()
    with pytest.raises(CartesianPlanError):
        add_segment_report(plan, _seg_ref(), [], 1, None, 10, 0, 1.0, 0.0)


def test_add_segment_report_full_requires_chosen():
    plan = _bare_plan()
    with pytest.raises(CartesianPlanError):
        add_segment_report(plan, _seg_ref(), [], 1, None, 10, 10, 1.0, 1.0)


def test_add_segment_report_partial_requires_chosen():
    plan = _bare_plan()
    with pytest.raises(CartesianPlanError):
        add_segment_report(plan, _seg_ref(), ["cartesian_incomplete"], 1, None, 10, 6, 1.0, 0.6)


# --------------------------------------------------------------------------------------
# finalize: coverage arithmetic on hand-built examples
# --------------------------------------------------------------------------------------

def _plan_with_one_full_leg_pair_and_segment(total_wp=10, planned_wp=10, total_len=1.0, planned_len=1.0, status_kind="full", mismatch=False, candidates_tried=1, chosen=None):
    plan = _bare_plan()
    ref = _seg_ref()
    if chosen is None and not mismatch and planned_wp > 0:
        chosen = {"direction": "forward", "roll_offset_rad": 0.0}
    reasons = [] if planned_wp == total_wp else ["cartesian_incomplete"]
    add_segment_report(plan, ref, reasons, candidates_tried, chosen, total_wp, planned_wp, total_len, planned_len, mismatch=mismatch)
    return plan, [_Task(ref)]


def test_finalize_coverage_full():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment()
    finalize(plan, _paths3d_doc(), tasks, LIMITS)
    cov = plan["coverage"]
    assert cov["trace_waypoints_total"] == 10
    assert cov["trace_waypoints_planned"] == 10
    assert cov["fraction_by_length"] == 1.0
    assert cov["complete"] is True


def test_finalize_coverage_partial_known_numbers():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment(total_wp=10, planned_wp=6, total_len=2.0, planned_len=1.2)
    finalize(plan, _paths3d_doc(), tasks, LIMITS)
    cov = plan["coverage"]
    assert cov["trace_waypoints_total"] == 10
    assert cov["trace_waypoints_planned"] == 6
    assert cov["length_m_total"] == 2.0
    assert cov["length_m_planned"] == 1.2
    assert cov["fraction_by_length"] == pytest.approx(0.6)
    assert cov["complete"] is False


def test_finalize_coverage_failed_is_zero():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment(total_wp=10, planned_wp=0, total_len=2.0, planned_len=0.0, chosen=None)
    finalize(plan, _paths3d_doc(), tasks, LIMITS)
    cov = plan["coverage"]
    assert cov["fraction_by_length"] == 0.0
    assert cov["complete"] is False


def test_finalize_coverage_mismatch_counts_as_failed_to_cover():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment(total_wp=10, planned_wp=0, total_len=2.0, planned_len=0.0, mismatch=True, candidates_tried=0)
    finalize(plan, _paths3d_doc(), tasks, LIMITS)
    cov = plan["coverage"]
    assert cov["fraction_by_length"] == 0.0
    assert cov["complete"] is False
    assert plan["segments"][0]["status"] == "surface_scene_mismatch"


def test_finalize_coverage_split_chain_complete_even_with_partial_original():
    """Original attempt partially covers (7/10); its own retry sub-segment finishes the
    remaining 3/3. Per §13.12 the aggregate is complete even though the *original* entry's own
    status is 'partial' (its own scope never reaches 'full')."""
    plan = _bare_plan()
    original_ref = _seg_ref(segment_index=0, sub_segment=None)
    retry_ref = _seg_ref(segment_index=0, sub_segment=0)
    add_segment_report(
        plan, original_ref, ["cartesian_incomplete"], 24,
        {"direction": "forward", "roll_offset_rad": 0.0}, 10, 7, 1.0, 0.7,
    )
    add_segment_report(
        plan, retry_ref, [], 1, {"direction": "forward", "roll_offset_rad": 0.0}, 3, 3, 0.3, 0.3,
    )
    finalize(plan, _paths3d_doc(), [_Task(original_ref)], LIMITS)
    cov = plan["coverage"]
    assert cov["trace_waypoints_total"] == 10  # only the original term counts toward total
    assert cov["trace_waypoints_planned"] == 10  # 7 + 3, disjoint
    assert cov["complete"] is True
    assert plan["segments"][0]["status"] == "partial"


def test_finalize_upstream_copied_verbatim():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment()
    finalize(plan, _paths3d_doc(points=123, valid=100, invalid_by_reason={"depth_invalid": 23}), tasks, LIMITS)
    assert plan["coverage"]["upstream"] == {"points": 123, "valid": 100, "invalid_by_reason": {"depth_invalid": 23}}


def test_finalize_missing_segment_raises():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment()
    extra_task = _Task(_seg_ref(segment_index=1))
    with pytest.raises(CartesianPlanError):
        finalize(plan, _paths3d_doc(), tasks + [extra_task], LIMITS)


def test_finalize_sets_caveats_disclaimer_first():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment()
    finalize(plan, _paths3d_doc(), tasks, LIMITS)
    assert plan["caveats"][0] == build_caveats("x", True)[0]


def test_finalize_ineligible_note_when_execution_ineligible():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment()
    plan["sources"]["paths3d"]["execution_eligible"] = None
    finalize(plan, _paths3d_doc(execution_eligible=False), tasks, LIMITS)
    assert plan["sources"]["paths3d"]["execution_eligible"] is False
    assert any("ineligible" in c.lower() for c in plan["caveats"])


def test_finalize_copies_execution_eligible_from_paths3d_doc():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment()
    plan["sources"]["paths3d"]["execution_eligible"] = None
    finalize(plan, _paths3d_doc(execution_eligible=True), tasks, LIMITS)
    assert plan["sources"]["paths3d"]["execution_eligible"] is True
    assert not any("ineligible" in c.lower() for c in plan["caveats"])


def test_finalize_raises_on_execution_eligible_mismatch():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment()
    # _sources() sets sources.paths3d.execution_eligible to True; a paths3d_doc disagreeing
    # with it must be rejected rather than silently trusted.
    with pytest.raises(CartesianPlanError):
        finalize(plan, _paths3d_doc(execution_eligible=False), tasks, LIMITS)


# --------------------------------------------------------------------------------------
# sources.paths3d shape (§11.2)
# --------------------------------------------------------------------------------------

def test_validate_plan_rejects_paths3d_source_missing_field():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment()
    finalize(plan, _paths3d_doc(), tasks, LIMITS)
    del plan["sources"]["paths3d"]["execution_eligible"]
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_paths3d_source_extra_field():
    plan, tasks = _plan_with_one_full_leg_pair_and_segment()
    finalize(plan, _paths3d_doc(), tasks, LIMITS)
    plan["sources"]["paths3d"]["extra"] = 1
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


# --------------------------------------------------------------------------------------
# validate_plan: strict rejections
# --------------------------------------------------------------------------------------

def _finalized_plan_with_two_legs():
    plan = _bare_plan()
    ref = _seg_ref()
    add_leg(plan, "approach", ref, "forward", 0.0, [_ZERO6, _ZERO6], [0, 0], _recheck())
    add_leg(plan, "trace", ref, "forward", 0.0, [_ZERO6, _ZERO6], [0, 1], _recheck())
    add_segment_report(plan, ref, [], 1, {"direction": "forward", "roll_offset_rad": 0.0}, 10, 10, 1.0, 1.0)
    finalize(plan, _paths3d_doc(), [_Task(ref)], LIMITS)
    return plan


def test_validate_plan_accepts_well_formed_plan():
    plan = _finalized_plan_with_two_legs()
    validated = validate_plan(plan, LIMITS)
    assert validated["schema"] == PLAN_SCHEMA


def test_validate_plan_rejects_missing_top_level_field():
    plan = _finalized_plan_with_two_legs()
    del plan["caveats"]
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_extra_top_level_field():
    plan = _finalized_plan_with_two_legs()
    plan["bogus"] = 1
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_wrong_joint_names():
    plan = _finalized_plan_with_two_legs()
    plan["joint_names"] = ["joint1", "joint2"]
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_non_finite_leg_position():
    plan = _finalized_plan_with_two_legs()
    plan["legs"][0]["positions"][0][0] = float("inf")
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_position_outside_limits():
    plan = _finalized_plan_with_two_legs()
    plan["legs"][0]["positions"][0][0] = 3.0  # joint1 upper bound is 2.8
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_accepts_position_at_inclusive_bound():
    plan = _finalized_plan_with_two_legs()
    plan["legs"][0]["positions"][0] = [2.8, 0.0, 0.0, 0.0, 0.0, 0.0]
    plan["legs"][0]["positions"][1] = [2.8, 0.0, 0.0, 0.0, 0.0, 0.0]
    plan["legs"][1]["positions"][0] = [2.8, 0.0, 0.0, 0.0, 0.0, 0.0]
    validate_plan(plan, LIMITS)


def test_validate_plan_rejects_discontinuous_legs():
    plan = _finalized_plan_with_two_legs()
    plan["legs"][1]["positions"][0][0] = 0.5  # was 0.0, previous leg's last config is still 0.0
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_accepts_within_continuity_tolerance():
    plan = _finalized_plan_with_two_legs()
    plan["legs"][1]["positions"][0][0] = CONTINUITY_TOL_RAD / 2.0
    validate_plan(plan, LIMITS)


def test_validate_plan_rejects_leg_index_gap():
    plan = _finalized_plan_with_two_legs()
    plan["legs"][1]["leg_index"] = 5
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_coverage_inconsistent_with_segments():
    plan = _finalized_plan_with_two_legs()
    plan["coverage"]["trace_waypoints_planned"] = 3
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_coverage_complete_mismatch():
    plan = _finalized_plan_with_two_legs()
    plan["coverage"]["complete"] = False
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_caveat_not_verbatim_disclaimer():
    plan = _finalized_plan_with_two_legs()
    plan["caveats"][0] = "something else"
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_segment_ref_bad_polyline_kind():
    plan = _finalized_plan_with_two_legs()
    plan["segments"][0]["segment_ref"]["polyline_kind"] = "bogus"
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_status_inconsistent_with_counts():
    plan = _finalized_plan_with_two_legs()
    plan["segments"][0]["status"] = "partial"
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_failed_with_chosen_set():
    plan = _finalized_plan_with_two_legs()
    plan["segments"][0]["status"] = "failed"
    plan["segments"][0]["reasons"] = ["ik_failed"]
    plan["segments"][0]["trace_waypoints"]["planned"] = 0
    plan["segments"][0]["trace_length_m"]["planned"] = 0.0
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_failed_with_empty_reasons():
    plan = _finalized_plan_with_two_legs()
    plan["segments"][0]["status"] = "failed"
    plan["segments"][0]["reasons"] = []
    plan["segments"][0]["chosen"] = None
    plan["segments"][0]["trace_waypoints"]["planned"] = 0
    plan["segments"][0]["trace_length_m"]["planned"] = 0.0
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_full_with_chosen_none():
    plan = _finalized_plan_with_two_legs()
    plan["segments"][0]["chosen"] = None
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


def test_validate_plan_rejects_partial_with_chosen_none():
    plan = _finalized_plan_with_two_legs()
    plan["segments"][0]["status"] = "partial"
    plan["segments"][0]["chosen"] = None
    plan["segments"][0]["trace_waypoints"]["planned"] = 6
    plan["segments"][0]["trace_length_m"]["planned"] = 0.6
    with pytest.raises(CartesianPlanError):
        validate_plan(plan, LIMITS)


# --------------------------------------------------------------------------------------
# write_plan / load_plan round trip
# --------------------------------------------------------------------------------------

def test_write_and_load_plan_round_trip(tmp_path):
    plan = _finalized_plan_with_two_legs()
    out_path = tmp_path / "case1_cartesian_plan.json"
    write_plan(plan, out_path)
    loaded = load_plan(out_path, LIMITS_PATH)
    assert loaded == plan


def test_write_plan_is_atomic_no_leftover_tmp(tmp_path):
    plan = _finalized_plan_with_two_legs()
    out_path = tmp_path / "case1_cartesian_plan.json"
    write_plan(plan, out_path)
    leftovers = list(tmp_path.glob(".*.tmp"))
    assert leftovers == []
    assert out_path.exists()


def test_load_plan_rejects_malformed_json(tmp_path):
    out_path = tmp_path / "bad.json"
    out_path.write_text("{not json")
    with pytest.raises(CartesianPlanError):
        load_plan(out_path, LIMITS_PATH)


def test_load_plan_rejects_invalid_document(tmp_path):
    plan = _finalized_plan_with_two_legs()
    plan["joint_names"] = ["joint1"]
    out_path = tmp_path / "bad.json"
    out_path.write_text(json.dumps(plan))
    with pytest.raises(CartesianPlanError):
        load_plan(out_path, LIMITS_PATH)


# --------------------------------------------------------------------------------------
# no segment of the input is ever absent from segments[]
# --------------------------------------------------------------------------------------

def test_finalize_requires_every_original_segment_present():
    plan = _bare_plan()
    ref0 = _seg_ref(segment_index=0)
    add_segment_report(plan, ref0, [], 1, {"direction": "forward", "roll_offset_rad": 0.0}, 5, 5, 0.5, 0.5)
    tasks = [_Task(ref0), _Task(_seg_ref(segment_index=1))]
    with pytest.raises(CartesianPlanError):
        finalize(plan, _paths3d_doc(), tasks, LIMITS)


def test_finalize_passes_when_every_original_segment_present():
    plan = _bare_plan()
    ref0 = _seg_ref(segment_index=0)
    ref1 = _seg_ref(segment_index=1)
    add_segment_report(plan, ref0, [], 1, {"direction": "forward", "roll_offset_rad": 0.0}, 5, 5, 0.5, 0.5)
    add_segment_report(plan, ref1, [], 1, {"direction": "forward", "roll_offset_rad": 0.0}, 5, 5, 0.5, 0.5)
    tasks = [_Task(ref0), _Task(ref1)]
    finalize(plan, _paths3d_doc(), tasks, LIMITS)
    assert plan["coverage"]["complete"] is True


# --------------------------------------------------------------------------------------
# view_plan
# --------------------------------------------------------------------------------------

def _view_sources():
    return {
        "scene": {"path": "config/scene/scene.yaml", "sha256": _sha(b"scene")},
        "end_effector": {"path": "config/robot/end_effector.yaml", "sha256": _sha(b"ee")},
        "limits": {"path": str(LIMITS_PATH), "sha256": _sha(LIMITS_PATH.read_bytes())},
        "config": {"path": "config/motion/cartesian_planning.yaml", "sha256": _sha(b"cfg")},
    }


def _view_pose():
    return {"position_m": [0.0, 0.0, 0.5], "quat_xyzw": [0.0, 0.0, 0.0, 1.0]}


def _view_fov_check(passed=True):
    return {"passed": passed, "horizontal_margins_deg": [1.0, 1.0, 1.0, 1.0], "vertical_margins_deg": [1.0, 1.0, 1.0, 1.0]}


def test_new_view_plan_shape():
    plan = new_view_plan(_view_sources(), {"view_distance_m": 0.25}, "--center", [0.1, 0.2], 0.0, [0.2, 0.2])
    assert plan["schema"] == VIEW_SCHEMA
    assert plan["candidates"] == []
    assert plan["chosen"] is None


def test_add_view_candidate_and_finalize_selects_passing_candidate():
    plan = new_view_plan(_view_sources(), {"view_distance_m": 0.25}, "--center", [0.1, 0.2], 0.0, [0.2, 0.2])
    add_view_candidate(plan, 0.0, 0.0, _view_pose(), _view_fov_check(passed=False), False, False)
    add_view_candidate(plan, 0.0, 90.0, _view_pose(), _view_fov_check(passed=True), True, True)
    transit_leg = build_leg("transit", None, None, None, [_ZERO6, _ZERO6], [None, None], _recheck())
    plan["candidates"][1]["camera_pose"]  # sanity
    finalize_view(plan, {"tilt_deg": 0.0, "yaw_deg": 90.0}, transit_leg, LIMITS)
    assert plan["chosen"] == {"tilt_deg": 0.0, "yaw_deg": 90.0}
    assert plan["transit_leg"]["kind"] == "transit"


def test_finalize_view_rejects_chosen_not_matching_any_candidate():
    plan = new_view_plan(_view_sources(), {"view_distance_m": 0.25}, "--center", [0.1, 0.2], 0.0, [0.2, 0.2])
    add_view_candidate(plan, 0.0, 0.0, _view_pose(), _view_fov_check(passed=True), True, True)
    transit_leg = build_leg("transit", None, None, None, [_ZERO6, _ZERO6], [None, None], _recheck())
    with pytest.raises(CartesianPlanError):
        finalize_view(plan, {"tilt_deg": 15.0, "yaw_deg": 180.0}, transit_leg, LIMITS)


def test_finalize_view_rejects_chosen_candidate_that_did_not_pass():
    plan = new_view_plan(_view_sources(), {"view_distance_m": 0.25}, "--center", [0.1, 0.2], 0.0, [0.2, 0.2])
    add_view_candidate(plan, 0.0, 0.0, _view_pose(), _view_fov_check(passed=False), True, True)
    transit_leg = build_leg("transit", None, None, None, [_ZERO6, _ZERO6], [None, None], _recheck())
    with pytest.raises(CartesianPlanError):
        finalize_view(plan, {"tilt_deg": 0.0, "yaw_deg": 0.0}, transit_leg, LIMITS)


def test_finalize_view_rejects_chosen_without_transit_leg():
    plan = new_view_plan(_view_sources(), {"view_distance_m": 0.25}, "--center", [0.1, 0.2], 0.0, [0.2, 0.2])
    add_view_candidate(plan, 0.0, 0.0, _view_pose(), _view_fov_check(passed=True), True, True)
    with pytest.raises(CartesianPlanError):
        finalize_view(plan, {"tilt_deg": 0.0, "yaw_deg": 0.0}, None, LIMITS)


def test_view_plan_refusal_has_null_chosen_and_transit_leg():
    plan = new_view_plan(_view_sources(), {"view_distance_m": 0.25}, "--center", [0.1, 0.2], 0.0, [0.2, 0.2])
    add_view_candidate(plan, 0.0, 0.0, _view_pose(), _view_fov_check(passed=False), False, False)
    finalize_view(plan, None, None, LIMITS)
    assert plan["chosen"] is None
    assert plan["transit_leg"] is None
    validate_view_plan(plan, LIMITS)


def test_write_and_load_view_plan_round_trip(tmp_path):
    plan = new_view_plan(_view_sources(), {"view_distance_m": 0.25}, "--center", [0.1, 0.2], 0.0, [0.2, 0.2])
    add_view_candidate(plan, 0.0, 0.0, _view_pose(), _view_fov_check(passed=True), True, True)
    transit_leg = build_leg("transit", None, None, None, [_ZERO6, _ZERO6], [None, None], _recheck())
    finalize_view(plan, {"tilt_deg": 0.0, "yaw_deg": 0.0}, transit_leg, LIMITS)
    out_path = tmp_path / "view_plan.json"
    write_view_plan(plan, out_path)
    loaded = load_view_plan(out_path, LIMITS_PATH)
    assert loaded == plan


def test_validate_view_plan_rejects_bad_center_source():
    plan = new_view_plan(_view_sources(), {"view_distance_m": 0.25}, "--center", [0.1, 0.2], 0.0, [0.2, 0.2])
    add_view_candidate(plan, 0.0, 0.0, _view_pose(), _view_fov_check(passed=False), False, False)
    finalize_view(plan, None, None, LIMITS)
    plan["center_source"] = "bogus"
    with pytest.raises(CartesianPlanError):
        validate_view_plan(plan, LIMITS)


# --------------------------------------------------------------------------------------
# caveats
# --------------------------------------------------------------------------------------

def test_build_caveats_disclaimer_first():
    caveats = build_caveats("note", True)
    assert caveats[0] == (
        "Nominal recommendation only; the operator places the specimen and MOT-10 measures and "
        "supersedes this file with the real, as-placed pose."
    )


def test_build_caveats_ineligible_only_when_false():
    assert not any("ineligible" in c.lower() for c in build_caveats("note", True))
    assert not any("ineligible" in c.lower() for c in build_caveats("note", None))
    assert any("ineligible" in c.lower() for c in build_caveats("note", False))

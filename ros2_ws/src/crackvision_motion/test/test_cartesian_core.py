"""Unit tests for cartesian_core (MOT-06.3). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_cartesian_core.py -q -p no:cacheprovider
"""

import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from crackvision_motion.cartesian_core import (
    CartesianConfigError,
    Paths3dError,
    Pose,
    SegmentTask,
    apply_candidate,
    candidates,
    load_config,
    load_paths3d,
    segment_tasks,
    split_remainder,
    surface_scene_mismatch,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
PRODUCTION_CONFIG = REPO_ROOT / "config" / "motion" / "cartesian_planning.yaml"
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures" / "cartesian"
MULTI_COMPONENT_FIXTURE = FIXTURES_DIR / "paths3d_multi_component.json"

# X = [0, 0, -1] (boresight down into the surface), Y = [0, 1, 0], Z = [1, 0, 0]; a 90 deg
# rotation about Y, same convention §10.5 uses (+X = -n_out, +Z completed right-handed).
_BASE_QUAT_XYZW = (0.0, 0.7071067811865476, 0.0, 0.7071067811865476)


def _quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _valid_config_raw() -> dict:
    return yaml.safe_load(PRODUCTION_CONFIG.read_text())


# --------------------------------------------------------------------------------------
# load_config
# --------------------------------------------------------------------------------------

def test_load_config_production_file():
    cfg = load_config(PRODUCTION_CONFIG)
    assert cfg["schema"] == "crackvision.cartesian_planning_config/1"
    assert cfg["end_state"] in ("start", "home", "none")
    assert cfg["value_status"] == "nominal"
    assert 0.20 <= cfg["view"]["view_distance_m"] <= 0.30
    assert len(cfg["candidates"]["roll_offsets_deg"]) == 12


def test_load_config_rejects_unknown_key(tmp_path):
    raw = _valid_config_raw()
    raw["bogus_key"] = 1
    p = tmp_path / "bad.yaml"
    p.write_text(yaml.safe_dump(raw))
    with pytest.raises(CartesianConfigError):
        load_config(p)


def test_load_config_rejects_missing_key(tmp_path):
    raw = _valid_config_raw()
    del raw["end_state"]
    p = tmp_path / "bad.yaml"
    p.write_text(yaml.safe_dump(raw))
    with pytest.raises(CartesianConfigError):
        load_config(p)


@pytest.mark.parametrize(
    "mutate",
    [
        lambda raw: raw["view"].__setitem__("view_distance_m", 0.5),
        lambda raw: raw["view"].__setitem__("tilt_candidates_deg", [20.0]),
        lambda raw: raw["candidates"].__setitem__("roll_offsets_deg", [200.0]),
        lambda raw: raw["candidates"].__setitem__("roll_offsets_deg", [30.0, 30.0]),
        lambda raw: raw["cartesian"].__setitem__("max_step_m", -0.001),
        lambda raw: raw.__setitem__("end_state", "bogus"),
        lambda raw: raw.__setitem__("value_status", "measured"),
    ],
)
def test_load_config_rejects_out_of_range(tmp_path, mutate):
    raw = _valid_config_raw()
    mutate(raw)
    p = tmp_path / "bad.yaml"
    p.write_text(yaml.safe_dump(raw))
    with pytest.raises(CartesianConfigError):
        load_config(p)


# --------------------------------------------------------------------------------------
# load_paths3d
# --------------------------------------------------------------------------------------

def test_load_paths3d_valid_fixture():
    doc, sha256 = load_paths3d(MULTI_COMPONENT_FIXTURE)
    assert doc["schema"] == "crackvision.paths3d/1"
    assert len(sha256) == 64
    import hashlib
    assert sha256 == hashlib.sha256(MULTI_COMPONENT_FIXTURE.read_bytes()).hexdigest()


def _minimal_doc() -> dict:
    with MULTI_COMPONENT_FIXTURE.open() as fh:
        return json.load(fh)


def test_load_paths3d_rejects_bad_schema(tmp_path):
    doc = _minimal_doc()
    doc["schema"] = "wrong/1"
    p = tmp_path / "bad.json"
    p.write_text(json.dumps(doc))
    with pytest.raises(Paths3dError):
        load_paths3d(p)


def test_load_paths3d_rejects_bad_frame(tmp_path):
    doc = _minimal_doc()
    doc["frame"] = "world"
    p = tmp_path / "bad.json"
    p.write_text(json.dumps(doc))
    with pytest.raises(Paths3dError):
        load_paths3d(p)


def test_load_paths3d_rejects_bad_phase_order(tmp_path):
    doc = _minimal_doc()
    waypoints = doc["components"][1]["polylines"][0]["segments"][0]["waypoints"]
    waypoints[0], waypoints[1] = waypoints[1], waypoints[0]
    p = tmp_path / "bad.json"
    p.write_text(json.dumps(doc))
    with pytest.raises(Paths3dError):
        load_paths3d(p)


def test_load_paths3d_rejects_non_unit_quaternion(tmp_path):
    doc = _minimal_doc()
    doc["components"][1]["polylines"][0]["segments"][0]["waypoints"][0]["quat_xyzw"] = [0.0, 0.0, 0.0, 1.5]
    p = tmp_path / "bad.json"
    p.write_text(json.dumps(doc))
    with pytest.raises(Paths3dError):
        load_paths3d(p)


def test_load_paths3d_rejects_non_finite_vector(tmp_path):
    doc = _minimal_doc()
    doc["components"][1]["polylines"][0]["segments"][0]["waypoints"][0]["position_m"] = [0.0, 0.0, float("nan")]
    p = tmp_path / "bad.json"
    p.write_text(json.dumps(doc))
    with pytest.raises(Paths3dError):
        load_paths3d(p)


def test_load_paths3d_never_raises_bare_keyerror(tmp_path):
    doc = _minimal_doc()
    del doc["components"][0]["component_id"]
    p = tmp_path / "bad.json"
    p.write_text(json.dumps(doc))
    with pytest.raises(Paths3dError):
        load_paths3d(p)


# --------------------------------------------------------------------------------------
# segment_tasks ordering (§13.3)
# --------------------------------------------------------------------------------------

def test_segment_tasks_ordering():
    doc, _ = load_paths3d(MULTI_COMPONENT_FIXTURE)
    tasks = segment_tasks(doc)
    refs = [(t.segment_ref["component_id"], t.segment_ref["polyline_kind"], t.segment_ref["branch_index"]) for t in tasks]
    # order_index ascending (crack_B=0 before crack_A=1); main before branch within a component.
    assert refs == [
        ("crack_B", "main", None),
        ("crack_A", "main", None),
        ("crack_A", "branch", 0),
    ]
    main_task = tasks[1]
    assert len(main_task.trace) == 5
    assert main_task.trace_length_m == pytest.approx(0.04)
    assert main_task.segment_ref["sub_segment"] is None


# --------------------------------------------------------------------------------------
# candidates (§13.4)
# --------------------------------------------------------------------------------------

def test_candidates_order_matches_spec():
    cfg = {"candidates": {"roll_offsets_deg": [0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180]}}
    cands = candidates(cfg)
    expected_degs = [0, 30, 30, 60, 60, 90, 90, 120, 120, 150, 150, 180]
    expected_degs_signed = [0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180]
    expected_directions = []
    expected_rolls_deg = []
    for deg in expected_degs_signed:
        expected_rolls_deg.append(deg)
        expected_rolls_deg.append(deg)
        expected_directions.append("forward")
        expected_directions.append("reverse")
    assert [c.direction for c in cands] == expected_directions
    got_degs = [round(math.degrees(c.roll_offset_rad)) for c in cands]
    assert got_degs == expected_rolls_deg


def test_candidates_deterministic_and_covers_all():
    cfg = {"candidates": {"roll_offsets_deg": [0, 30, -30, 180]}}
    cands = candidates(cfg)
    assert len(cands) == 8
    # roll magnitude ascending, positive before negative, forward before reverse.
    assert (cands[0].direction, round(math.degrees(cands[0].roll_offset_rad))) == ("forward", 0)
    assert (cands[1].direction, round(math.degrees(cands[1].roll_offset_rad))) == ("reverse", 0)
    assert round(math.degrees(cands[2].roll_offset_rad)) == 30
    assert round(math.degrees(cands[4].roll_offset_rad)) == -30
    assert round(math.degrees(cands[6].roll_offset_rad)) == 180


# --------------------------------------------------------------------------------------
# apply_candidate (§13.4)
# --------------------------------------------------------------------------------------

def _single_pose_task() -> SegmentTask:
    pose = Pose((0.1, 0.2, 0.3), _BASE_QUAT_XYZW)
    approach = Pose((0.1, 0.2, 0.33), _BASE_QUAT_XYZW)
    retract = Pose((0.14, 0.2, 0.33), _BASE_QUAT_XYZW)
    return SegmentTask(
        segment_ref={"component_id": "c", "polyline_kind": "main", "branch_index": None, "segment_index": 0, "sub_segment": None},
        approach=approach,
        trace=(pose,),
        retract=retract,
        normals=((0.0, 0.0, 1.0),),
        surface_points=((0.1, 0.2, 0.0),),
        trace_length_m=0.0,
    )


def test_apply_candidate_roll_rotates_z_about_local_x():
    task = _single_pose_task()
    roll_rad = math.pi / 2.0
    result = apply_candidate(task, "forward", roll_rad)
    rotated = result.trace[0]

    # position never changes under a roll.
    assert rotated.position_m == pytest.approx(task.trace[0].position_m)

    old_matrix = _quat_to_matrix(task.trace[0].quat_xyzw)
    new_matrix = _quat_to_matrix(rotated.quat_xyzw)
    old_x, old_z = old_matrix[:, 0], old_matrix[:, 2]
    new_x, new_z = new_matrix[:, 0], new_matrix[:, 2]

    # the boresight (+X) is unchanged by a rotation about itself.
    assert new_x == pytest.approx(old_x, abs=1e-9)
    # +Z rotates by the roll offset about +X (Rodrigues, since Z is already perpendicular to X).
    expected_new_z = (
        old_z * math.cos(roll_rad)
        + np.cross(old_x, old_z) * math.sin(roll_rad)
    )
    assert new_z == pytest.approx(expected_new_z, abs=1e-9)


def test_apply_candidate_reverse_swaps_and_reverses():
    pose_a = Pose((0.0, 0.0, 0.01), _BASE_QUAT_XYZW)
    pose_b = Pose((0.01, 0.0, 0.01), _BASE_QUAT_XYZW)
    approach = Pose((0.0, 0.0, 0.04), _BASE_QUAT_XYZW)
    retract = Pose((0.01, 0.0, 0.04), _BASE_QUAT_XYZW)
    task = SegmentTask(
        segment_ref={"component_id": "c", "polyline_kind": "main", "branch_index": None, "segment_index": 0, "sub_segment": None},
        approach=approach,
        trace=(pose_a, pose_b),
        retract=retract,
        normals=((0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),
        surface_points=((0.0, 0.0, 0.0), (0.01, 0.0, 0.0)),
        trace_length_m=0.01,
    )

    result = apply_candidate(task, "reverse", 0.0)

    assert result.approach.position_m == pytest.approx(retract.position_m)
    assert result.approach.quat_xyzw == pytest.approx(retract.quat_xyzw)
    assert result.retract.position_m == pytest.approx(approach.position_m)
    assert result.retract.quat_xyzw == pytest.approx(approach.quat_xyzw)
    assert [p.position_m for p in result.trace] == [pose_b.position_m, pose_a.position_m]


# --------------------------------------------------------------------------------------
# split_remainder (§13.7)
# --------------------------------------------------------------------------------------

def _five_point_task() -> SegmentTask:
    xs = [0.0, 0.01, 0.02, 0.03, 0.04]
    trace = tuple(Pose((x, 0.0, 0.01), _BASE_QUAT_XYZW) for x in xs)
    normals = tuple((0.0, 0.0, 1.0) for _ in xs)
    surface_points = tuple((x, 0.0, 0.0) for x in xs)
    approach = Pose((0.0, 0.0, 0.04), _BASE_QUAT_XYZW)
    retract = Pose((0.04, 0.0, 0.04), _BASE_QUAT_XYZW)
    return SegmentTask(
        segment_ref={"component_id": "c", "polyline_kind": "main", "branch_index": None, "segment_index": 0, "sub_segment": None},
        approach=approach,
        trace=trace,
        retract=retract,
        normals=normals,
        surface_points=surface_points,
        trace_length_m=0.04,
    )


def test_split_remainder_builds_subtask():
    task = _five_point_task()
    remainder = split_remainder(task, last_reached_index=1)
    assert remainder is not None
    assert len(remainder.trace) == 3
    assert remainder.segment_ref["sub_segment"] == 0
    # offset_m recovered from the original approach (0.04 m standoff).
    assert remainder.approach.position_m == pytest.approx((0.02, 0.0, 0.04))
    assert remainder.retract.position_m == pytest.approx((0.04, 0.0, 0.04))
    assert remainder.approach.quat_xyzw == pytest.approx(remainder.trace[0].quat_xyzw)


def test_split_remainder_none_when_too_short():
    task = _five_point_task()
    remainder = split_remainder(task, last_reached_index=2)
    assert remainder is None


def test_split_remainder_subsegment_increments():
    task = _five_point_task()
    first = split_remainder(task, last_reached_index=1)
    second = split_remainder(first, last_reached_index=-1)
    assert second is not None
    assert second.segment_ref["sub_segment"] == 1


# --------------------------------------------------------------------------------------
# surface_scene_mismatch (§13.9)
# --------------------------------------------------------------------------------------

def _scene_with_box(position_m, dimensions_m, rpy_rad=(0.0, 0.0, 0.0), obj_id="table"):
    return {
        "objects": [
            {
                "id": obj_id,
                "dimensions_m": dimensions_m,
                "pose": {"position_m": position_m, "rpy_rad": rpy_rad},
            }
        ]
    }


def test_surface_scene_mismatch_clear_trace_passes():
    task = _five_point_task()  # trace z = 0.01
    scene_cfg = _scene_with_box([0.02, 0.0, -0.05], [1.0, 1.0, 0.1])  # top face at z=0, far below trace
    reasons = surface_scene_mismatch(task, scene_cfg, clearance_m=0.005)
    assert reasons == []


def test_surface_scene_mismatch_flags_close_trace():
    task = _five_point_task()  # trace z = 0.01
    # top face at z = 0.005: trace sits only 0.005 m above it, less than the 0.01 clearance.
    scene_cfg = _scene_with_box([0.02, 0.0, -0.045], [1.0, 1.0, 0.1])
    reasons = surface_scene_mismatch(task, scene_cfg, clearance_m=0.01)
    assert len(reasons) == 5


def test_surface_scene_mismatch_handles_rotated_box():
    # A 45 deg-yawed box centred at the origin; the point (0.06, 0, 0) lies outside the
    # unrotated axis-aligned footprint but inside the actual rotated box.
    pose = Pose((0.06, 0.0, 0.0), _BASE_QUAT_XYZW)
    task = SegmentTask(
        segment_ref={"component_id": "c", "polyline_kind": "main", "branch_index": None, "segment_index": 0, "sub_segment": None},
        approach=pose,
        trace=(pose,),
        retract=pose,
        normals=((0.0, 0.0, 1.0),),
        surface_points=((0.06, 0.0, 0.0),),
        trace_length_m=0.0,
    )
    scene_cfg = _scene_with_box([0.0, 0.0, 0.0], [0.1, 0.1, 1.0], rpy_rad=(0.0, 0.0, math.pi / 4.0))
    reasons = surface_scene_mismatch(task, scene_cfg, clearance_m=0.001)
    assert len(reasons) == 1

    # Sanity check: the unrotated axis-aligned box would NOT flag this point (outside by 0.01 m).
    axis_aligned_scene = _scene_with_box([0.0, 0.0, 0.0], [0.1, 0.1, 1.0])
    assert surface_scene_mismatch(task, axis_aligned_scene, clearance_m=0.001) == []

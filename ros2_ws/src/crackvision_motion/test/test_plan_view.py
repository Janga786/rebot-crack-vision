"""Unit tests for plan_view (MOT-06.9). Runs without a colcon build or a running ROS graph --
`_attempt_view_candidate` is tested with canned `moveit_msgs` response messages via a
`MoveItClients`-shaped fake (mirrors test_plan_crack_path.py's own `_FakeMoveIt`, still no
`rclpy.init`, no running graph); the CLI-level tests only ever exercise `--dry-run` (no
`rclpy.init`, no file write).

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_plan_view.py -q -p no:cacheprovider
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List

import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
from trajectory_msgs.msg import JointTrajectoryPoint

from crackvision_motion.cartesian_core import load_config
from crackvision_motion.cli_common import EXIT_OK, EXIT_PRECONDITION, EXIT_USAGE
from crackvision_motion.joint_trajectory import JOINT_NAMES
from crackvision_motion.plan_view import (
    _attempt_view_candidate,
    _joint_dict,
    _max_joint_step,
    _pose_errors,
    _resolve_specimen,
    _scene_specimen_footprint_m,
    _within_limits,
    main,
)
from crackvision_motion.view_core import Pose, specimen_frame, view_candidates

REPO_ROOT = Path(__file__).resolve().parents[4]
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures" / "cartesian" / "smoke"
PRODUCTION_PLANNING_CONFIG = REPO_ROOT / "config" / "motion" / "cartesian_planning.yaml"

_LIMITS = {n: {"lower": -3.0, "upper": 3.0} for n in JOINT_NAMES}
_ZEROS6 = [0.0] * 6


@pytest.fixture()
def cfg():
    return load_config(PRODUCTION_PLANNING_CONFIG)


class _FakeMoveIt:
    """A minimal `MoveItClients`-shaped double: `.clients[key]` is `key` itself, so `.call()` can
    dispatch purely on the key `_attempt_view_candidate` indexes with; each key's canned responses
    are returned in order. No `rclpy`/running ROS graph involved."""

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


def _traj(names, points_positions):
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names = list(names)
    for positions in points_positions:
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        traj.joint_trajectory.points.append(pt)
    return traj


def _motion_plan_response(configs, error_code=MoveItErrorCodes.SUCCESS):
    resp = GetMotionPlan.Response()
    resp.motion_plan_response.error_code.val = error_code
    if configs is not None:
        resp.motion_plan_response.trajectory = _traj(JOINT_NAMES, configs)
    return resp


def _fk_response(pose: Pose):
    resp = GetPositionFK.Response()
    resp.error_code.val = MoveItErrorCodes.SUCCESS
    resp.fk_link_names = ["camera_link"]
    ps = PoseStamped()
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pose.position_m
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = pose.quat_xyzw
    resp.pose_stamped = [ps]
    return resp


def _validity_response(valid: bool):
    resp = GetStateValidity.Response()
    resp.valid = valid
    return resp


def _specimen_frame_and_candidates(cfg):
    frame = specimen_frame(center=(0.26, 0.0), footprint_m=[0.2, 0.2], surface_z_m=0.01)
    cands = view_candidates(cfg, frame.centre_top_m, frame.normal_out)
    return frame, cands


# --------------------------------------------------------------------------------------
# _attempt_view_candidate, fake services
# --------------------------------------------------------------------------------------

def test_attempt_view_candidate_fov_failure_short_circuits_before_any_ros_call(cfg):
    frame, cands = _specimen_frame_and_candidates(cfg)
    # A huge footprint guarantees the FOV check fails for every candidate -- no ik/fk/motion_plan
    # queue is ever supplied, so any ROS call would raise KeyError.
    far_frame = specimen_frame(center=(0.26, 0.0), footprint_m=[5.0, 5.0], surface_z_m=0.01)
    moveit = _FakeMoveIt({})
    attempt = _attempt_view_candidate(moveit, cfg, _LIMITS, _joint_dict(_ZEROS6), cands[0], far_frame.corners_m)
    assert attempt.reachable is False
    assert attempt.recheck_passed is False
    assert attempt.transit_leg_spec is None
    assert attempt.reason == "fov_failed"
    assert attempt.fov_entry["passed"] is False


def test_attempt_view_candidate_ik_failure(cfg):
    frame, cands = _specimen_frame_and_candidates(cfg)
    moveit = _FakeMoveIt({"ik": [_ik_response(error_code=MoveItErrorCodes.NO_IK_SOLUTION)]})
    attempt = _attempt_view_candidate(moveit, cfg, _LIMITS, _joint_dict(_ZEROS6), cands[0], frame.corners_m)
    assert attempt.reachable is False
    assert attempt.recheck_passed is False
    assert attempt.transit_leg_spec is None
    assert attempt.reason == "ik_failed"


def test_attempt_view_candidate_transit_failure(cfg):
    frame, cands = _specimen_frame_and_candidates(cfg)
    moveit = _FakeMoveIt({
        "ik": [_ik_response(_ZEROS6)],
        "fk": [_fk_response(cands[0].pose)],
        "motion_plan": [_motion_plan_response(None, error_code=MoveItErrorCodes.PLANNING_FAILED)],
    })
    attempt = _attempt_view_candidate(moveit, cfg, _LIMITS, _joint_dict(_ZEROS6), cands[0], frame.corners_m)
    assert attempt.reachable is False
    assert attempt.recheck_passed is False
    assert attempt.transit_leg_spec is None
    assert attempt.reason == "transit_failed"


def test_attempt_view_candidate_fk_mismatch_fails_recheck_but_is_reachable(cfg):
    frame, cands = _specimen_frame_and_candidates(cfg)
    wrong_pose = Pose(
        (cands[0].pose.position_m[0] + 1.0, cands[0].pose.position_m[1], cands[0].pose.position_m[2]),
        cands[0].pose.quat_xyzw,
    )
    moveit = _FakeMoveIt({
        "ik": [_ik_response(_ZEROS6)],
        "fk": [_fk_response(wrong_pose)],
        "motion_plan": [_motion_plan_response([_ZEROS6, _ZEROS6])],
        "validity": [_validity_response(True)] * 2,
    })
    attempt = _attempt_view_candidate(moveit, cfg, _LIMITS, _joint_dict(_ZEROS6), cands[0], frame.corners_m)
    assert attempt.reachable is True
    assert attempt.recheck_passed is False
    assert attempt.transit_leg_spec is None
    assert "fk_mismatch" in attempt.reason


def test_attempt_view_candidate_collision_fails_recheck(cfg):
    frame, cands = _specimen_frame_and_candidates(cfg)
    moveit = _FakeMoveIt({
        "ik": [_ik_response(_ZEROS6)],
        "fk": [_fk_response(cands[0].pose)],
        "motion_plan": [_motion_plan_response([_ZEROS6, _ZEROS6])],
        "validity": [_validity_response(False)],
    })
    attempt = _attempt_view_candidate(moveit, cfg, _LIMITS, _joint_dict(_ZEROS6), cands[0], frame.corners_m)
    assert attempt.reachable is True
    assert attempt.recheck_passed is False
    assert attempt.transit_leg_spec is None
    assert "collision" in attempt.reason


def test_attempt_view_candidate_full_success_builds_transit_leg_spec(cfg):
    frame, cands = _specimen_frame_and_candidates(cfg)
    transit_configs = [_ZEROS6, [0.01] * 6]
    moveit = _FakeMoveIt({
        "ik": [_ik_response(_ZEROS6)],
        "fk": [_fk_response(cands[0].pose)],
        "motion_plan": [_motion_plan_response(transit_configs)],
        "validity": [_validity_response(True)] * len(transit_configs),
    })
    attempt = _attempt_view_candidate(moveit, cfg, _LIMITS, _joint_dict(_ZEROS6), cands[0], frame.corners_m)
    assert attempt.reachable is True
    assert attempt.recheck_passed is True
    assert attempt.reason is None
    spec = attempt.transit_leg_spec
    assert spec is not None
    assert spec["kind"] == "transit"
    assert spec["segment_ref"] is None and spec["direction"] is None and spec["roll_offset_rad"] is None
    assert spec["positions"] == transit_configs
    assert spec["waypoint_index_map"] == [None, None]
    assert spec["recheck"]["all_valid"] is True
    assert spec["recheck"]["within_limits"] is True
    assert spec["recheck"]["max_joint_step_rad"] == pytest.approx(_max_joint_step(transit_configs))


# --------------------------------------------------------------------------------------
# small pure helpers
# --------------------------------------------------------------------------------------

def test_within_limits():
    assert _within_limits([[0.0] * 6], _LIMITS) is True
    assert _within_limits([[10.0] + [0.0] * 5], _LIMITS) is False


def test_pose_errors_zero_for_identical_pose():
    p = Pose((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
    pos_err, ang_err = _pose_errors(p, p)
    assert pos_err == pytest.approx(0.0)
    assert ang_err == pytest.approx(0.0)


def test_scene_specimen_footprint_m_reads_dimensions_xy():
    scene_cfg = {"objects": [{"id": "specimen", "dimensions_m": [0.2, 0.3, 0.01]}]}
    assert _scene_specimen_footprint_m(scene_cfg) == [0.2, 0.3]
    assert _scene_specimen_footprint_m({"objects": []}) is None


def test_resolve_specimen_center_takes_precedence_over_scene(cfg):
    import argparse

    scene_cfg = {"objects": [{
        "id": "specimen", "dimensions_m": [0.2, 0.2, 0.01],
        "pose": {"position_m": [0.3, 0.0, 0.005], "rpy_rad": [0.0, 0.0, 0.0]},
    }]}
    args = argparse.Namespace(center=[0.26, 0.0])
    center_source, center_xy_m, surface_z_m, footprint_m, frame = _resolve_specimen(args, cfg, scene_cfg)
    assert center_source == "--center"
    assert center_xy_m == [0.26, 0.0]
    assert surface_z_m == cfg["view"]["surface_z_m"]
    assert footprint_m == list(cfg["view"]["footprint_m"])
    assert frame.centre_top_m == (0.26, 0.0, cfg["view"]["surface_z_m"])


def test_resolve_specimen_falls_back_to_scene_specimen(cfg):
    import argparse

    scene_cfg = {"objects": [{
        "id": "specimen", "dimensions_m": [0.2, 0.2, 0.01],
        "pose": {"position_m": [0.3, 0.0, 0.005], "rpy_rad": [0.0, 0.0, 0.0]},
    }]}
    args = argparse.Namespace(center=None)
    center_source, center_xy_m, surface_z_m, footprint_m, frame = _resolve_specimen(args, cfg, scene_cfg)
    assert center_source == "scene.specimen"
    assert center_xy_m == pytest.approx([0.3, 0.0])
    assert surface_z_m == pytest.approx(0.01)
    assert footprint_m == [0.2, 0.2]


# --------------------------------------------------------------------------------------
# CLI: --dry-run only (no rclpy.init, no ROS, no file write) -- §13.15
# --------------------------------------------------------------------------------------

def test_dry_run_exits_ok_and_never_touches_rclpy(monkeypatch):
    import crackvision_motion.plan_view as plan_view_mod

    def _forbidden_init(*args, **kwargs):
        raise AssertionError("--dry-run must never call rclpy.init")

    monkeypatch.setattr(plan_view_mod.rclpy, "init", _forbidden_init)
    exit_code = main([
        "--dry-run", "--root", str(REPO_ROOT),
        "--scene-config", str(FIXTURES_DIR / "scene.yaml"),
    ])
    assert exit_code == EXIT_OK


def test_dry_run_writes_no_plan_file(tmp_path):
    out_path = tmp_path / "view_plan.json"
    exit_code = main([
        "--dry-run", "--root", str(REPO_ROOT),
        "--scene-config", str(FIXTURES_DIR / "scene.yaml"), "--out", str(out_path),
    ])
    assert exit_code == EXIT_OK
    assert not out_path.exists()


def test_dry_run_bad_planning_config_is_usage_error(tmp_path):
    bad_cfg = tmp_path / "bad.yaml"
    bad_cfg.write_text("schema: crackvision.cartesian_planning_config/1\nunknown_key: 1\n", encoding="utf-8")
    exit_code = main([
        "--dry-run", "--root", str(REPO_ROOT), "--planning-config", str(bad_cfg),
        "--scene-config", str(FIXTURES_DIR / "scene.yaml"),
    ])
    assert exit_code == EXIT_USAGE


def test_dry_run_no_center_and_no_scene_specimen_is_precondition_refusal(tmp_path):
    no_specimen_scene = tmp_path / "scene.yaml"
    no_specimen_scene.write_text(
        "schema: crackvision.scene_config/1\n"
        "frame: base_link\n"
        "objects:\n"
        "  - id: table\n"
        "    shape: box\n"
        "    dimensions_m: [1.0, 1.0, 0.05]\n"
        "    pose:\n"
        "      frame: base_link\n"
        "      position_m: [0.35, 0.0, -0.025]\n"
        "      rpy_rad: [0.0, 0.0, 0.0]\n"
        "    value_status: nominal\n"
        "    source: test fixture\n"
        "allowed_collisions: []\n",
        encoding="utf-8",
    )
    exit_code = main([
        "--dry-run", "--root", str(REPO_ROOT), "--scene-config", str(no_specimen_scene),
    ])
    assert exit_code == EXIT_PRECONDITION


def test_dry_run_with_center_ignores_missing_scene_specimen(tmp_path):
    no_specimen_scene = tmp_path / "scene.yaml"
    no_specimen_scene.write_text(
        "schema: crackvision.scene_config/1\n"
        "frame: base_link\n"
        "objects:\n"
        "  - id: table\n"
        "    shape: box\n"
        "    dimensions_m: [1.0, 1.0, 0.05]\n"
        "    pose:\n"
        "      frame: base_link\n"
        "      position_m: [0.35, 0.0, -0.025]\n"
        "      rpy_rad: [0.0, 0.0, 0.0]\n"
        "    value_status: nominal\n"
        "    source: test fixture\n"
        "allowed_collisions: []\n",
        encoding="utf-8",
    )
    exit_code = main([
        "--dry-run", "--root", str(REPO_ROOT), "--scene-config", str(no_specimen_scene),
        "--center", "0.3", "0.0",
    ])
    assert exit_code == EXIT_OK

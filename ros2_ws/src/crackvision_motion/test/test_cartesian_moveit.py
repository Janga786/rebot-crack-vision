"""Unit tests for cartesian_moveit (MOT-06.5). Runs without a colcon build or a running ROS graph
-- every test here builds/parses `moveit_msgs` messages directly; none calls `rclpy.init` or spins
a node.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py -q -p no:cacheprovider
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import (
    GetCartesianPath,
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)
from trajectory_msgs.msg import JointTrajectoryPoint

from crackvision_motion.cartesian_core import Pose, load_config
from crackvision_motion.cartesian_moveit import (
    FRAME,
    GROUP,
    MoveItClients,
    ServiceCallTimeoutError,
    build_cartesian_request,
    build_fk_request,
    build_ik_request,
    build_motion_plan_request,
    build_validity_request,
    match_reached_waypoints,
    parse_cartesian,
    parse_fk,
    parse_ik,
    parse_motion_plan,
)
from crackvision_motion.joint_trajectory import JOINT_NAMES

REPO_ROOT = Path(__file__).resolve().parents[4]
PRODUCTION_CONFIG = REPO_ROOT / "config" / "motion" / "cartesian_planning.yaml"


@pytest.fixture()
def cfg():
    return load_config(PRODUCTION_CONFIG)


def _joints(values):
    return dict(zip(JOINT_NAMES, values))


def _pose(x=0.1, y=0.2, z=0.3, quat=(0.0, 0.0, 0.0, 1.0)) -> Pose:
    return Pose((x, y, z), quat)


def _joint_trajectory(names, points_positions):
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names = list(names)
    for positions in points_positions:
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        traj.joint_trajectory.points.append(pt)
    return traj


# --------------------------------------------------------------------------------------
# request builders
# --------------------------------------------------------------------------------------

def test_build_cartesian_request_fields(cfg):
    start = _joints([0.0] * 6)
    poses = [_pose(0.1, 0.0, 0.3), _pose(0.1, 0.01, 0.3)]
    req = build_cartesian_request(start, poses, "tool_tip", cfg)

    assert isinstance(req, GetCartesianPath.Request)
    assert req.avoid_collisions is True
    assert req.group_name == GROUP
    assert req.header.frame_id == FRAME
    assert req.link_name == "tool_tip"
    assert len(req.waypoints) == 2
    assert req.waypoints[0].position.x == pytest.approx(0.1)
    assert req.waypoints[1].position.y == pytest.approx(0.01)
    assert req.max_step == pytest.approx(cfg["cartesian"]["max_step_m"])
    assert req.jump_threshold == pytest.approx(cfg["cartesian"]["jump_threshold"])
    assert req.prismatic_jump_threshold == pytest.approx(cfg["cartesian"]["prismatic_jump_threshold"])
    assert req.revolute_jump_threshold == pytest.approx(cfg["cartesian"]["revolute_jump_threshold"])
    assert list(req.start_state.joint_state.name) == list(JOINT_NAMES)
    assert list(req.start_state.joint_state.position) == [0.0] * 6


def test_build_cartesian_request_has_no_avoid_collisions_parameter(cfg):
    """There is no way to make this builder produce avoid_collisions=False (ADR-017 §7):
    confirm by signature inspection, not just by checking the one call above."""
    import inspect

    sig = inspect.signature(build_cartesian_request)
    assert "avoid_collisions" not in sig.parameters


def test_build_motion_plan_request_fields(cfg):
    start = _joints([0.0] * 6)
    goal = _joints([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    req = build_motion_plan_request(start, goal, cfg)

    assert isinstance(req, GetMotionPlan.Request)
    mpr = req.motion_plan_request
    assert mpr.group_name == GROUP
    assert list(mpr.start_state.joint_state.name) == list(JOINT_NAMES)
    assert len(mpr.goal_constraints) == 1
    jcs = {jc.joint_name: jc.position for jc in mpr.goal_constraints[0].joint_constraints}
    assert jcs == goal
    for jc in mpr.goal_constraints[0].joint_constraints:
        assert jc.tolerance_above > 0
        assert jc.tolerance_below > 0


def test_build_ik_request_fields(cfg):
    seed = _joints([0.0] * 6)
    pose = _pose(0.2, 0.0, 0.3)
    req = build_ik_request(pose, seed, "tool_tip", cfg)

    assert isinstance(req, GetPositionIK.Request)
    assert req.ik_request.group_name == GROUP
    assert req.ik_request.ik_link_name == "tool_tip"
    assert req.ik_request.avoid_collisions is True
    assert req.ik_request.pose_stamped.header.frame_id == FRAME
    assert req.ik_request.pose_stamped.pose.position.x == pytest.approx(0.2)
    assert list(req.ik_request.robot_state.joint_state.name) == list(JOINT_NAMES)
    total_timeout = req.ik_request.timeout.sec + req.ik_request.timeout.nanosec * 1e-9
    assert total_timeout == pytest.approx(cfg["service_timeout_s"])


def test_build_fk_request_fields():
    positions = _joints([0.1] * 6)
    req = build_fk_request(positions, ["tool_tip", "camera_link"])

    assert isinstance(req, GetPositionFK.Request)
    assert req.header.frame_id == FRAME
    assert list(req.fk_link_names) == ["tool_tip", "camera_link"]
    assert list(req.robot_state.joint_state.position) == [0.1] * 6


def test_build_validity_request_fields():
    positions = _joints([0.2] * 6)
    req = build_validity_request(positions)

    assert isinstance(req, GetStateValidity.Request)
    assert req.group_name == GROUP
    assert list(req.robot_state.joint_state.name) == list(JOINT_NAMES)
    assert list(req.robot_state.joint_state.position) == [0.2] * 6


# --------------------------------------------------------------------------------------
# response parsers
# --------------------------------------------------------------------------------------

def test_parse_cartesian_full_success():
    resp = GetCartesianPath.Response()
    resp.error_code.val = MoveItErrorCodes.SUCCESS
    resp.fraction = 1.0
    resp.solution = _joint_trajectory(JOINT_NAMES, [[0.0] * 6, [0.1] * 6])

    configs, fraction, error = parse_cartesian(resp)
    assert fraction == 1.0
    assert error is None
    assert configs == [[0.0] * 6, [0.1] * 6]


def test_parse_cartesian_partial_fraction_is_incomplete():
    resp = GetCartesianPath.Response()
    resp.error_code.val = MoveItErrorCodes.SUCCESS
    resp.fraction = 0.5
    resp.solution = _joint_trajectory(JOINT_NAMES, [[0.0] * 6])

    configs, fraction, error = parse_cartesian(resp)
    assert fraction == 0.5
    assert error == "cartesian_incomplete"
    assert configs == [[0.0] * 6]


def test_parse_cartesian_error_code_discards_solution():
    resp = GetCartesianPath.Response()
    resp.error_code.val = MoveItErrorCodes.FAILURE
    resp.fraction = 0.0

    configs, fraction, error = parse_cartesian(resp)
    assert configs == []
    assert error == "cartesian_incomplete"


def test_parse_motion_plan_success():
    resp = GetMotionPlan.Response()
    resp.motion_plan_response.error_code.val = MoveItErrorCodes.SUCCESS
    resp.motion_plan_response.trajectory = _joint_trajectory(JOINT_NAMES, [[0.0] * 6, [0.2] * 6])

    configs, error = parse_motion_plan(resp)
    assert error is None
    assert configs == [[0.0] * 6, [0.2] * 6]


def test_parse_motion_plan_failure_maps_to_transit_failed():
    resp = GetMotionPlan.Response()
    resp.motion_plan_response.error_code.val = MoveItErrorCodes.PLANNING_FAILED

    configs, error = parse_motion_plan(resp)
    assert configs == []
    assert error == "transit_failed"


def test_parse_ik_success():
    resp = GetPositionIK.Response()
    resp.error_code.val = MoveItErrorCodes.SUCCESS
    resp.solution.joint_state.name = list(JOINT_NAMES) + ["gripper_joint1"]
    resp.solution.joint_state.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.0]

    joints, error = parse_ik(resp)
    assert error is None
    assert joints == _joints([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])


def test_parse_ik_no_solution_maps_to_ik_failed():
    resp = GetPositionIK.Response()
    resp.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION

    joints, error = parse_ik(resp)
    assert joints is None
    assert error == "ik_failed"


def test_parse_ik_timed_out_maps_to_ik_failed():
    resp = GetPositionIK.Response()
    resp.error_code.val = MoveItErrorCodes.TIMED_OUT

    _, error = parse_ik(resp)
    assert error == "ik_failed"


def test_parse_ik_other_error_is_distinct_from_ik_failed():
    resp = GetPositionIK.Response()
    resp.error_code.val = MoveItErrorCodes.INVALID_GROUP_NAME

    joints, error = parse_ik(resp)
    assert joints is None
    assert error != "ik_failed"
    assert error == "ik_error:INVALID_GROUP_NAME"


def test_parse_fk_success():
    resp = GetPositionFK.Response()
    resp.error_code.val = MoveItErrorCodes.SUCCESS
    resp.fk_link_names = ["tool_tip"]
    from geometry_msgs.msg import PoseStamped

    ps = PoseStamped()
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = 0.1, 0.2, 0.3
    ps.pose.orientation.w = 1.0
    resp.pose_stamped = [ps]

    pose, error = parse_fk(resp, "tool_tip")
    assert error is None
    assert pose.position_m == (0.1, 0.2, 0.3)
    assert pose.quat_xyzw == (0.0, 0.0, 0.0, 1.0)


def test_parse_fk_failure():
    resp = GetPositionFK.Response()
    resp.error_code.val = MoveItErrorCodes.FAILURE

    pose, error = parse_fk(resp, "tool_tip")
    assert pose is None
    assert error == "fk_error:FAILURE"


# --------------------------------------------------------------------------------------
# match_reached_waypoints (§13.7 FK matching)
# --------------------------------------------------------------------------------------

def test_match_reached_waypoints_full_match():
    targets = [_pose(0.0, 0.0, 0.0), _pose(0.0, 0.01, 0.0), _pose(0.0, 0.02, 0.0)]
    fk_poses = list(targets)

    last_index, errors = match_reached_waypoints(fk_poses, targets, fk_tolerance_m=0.001, fk_tolerance_deg=1.0)
    assert last_index == 2
    assert len(errors) == 3


def test_match_reached_waypoints_partial_match():
    targets = [_pose(0.0, 0.0, 0.0), _pose(0.0, 0.01, 0.0), _pose(0.0, 0.02, 0.0)]
    # fk trajectory only ever reaches target 0, then a far-off point that matches nothing.
    fk_poses = [targets[0], _pose(5.0, 5.0, 5.0)]

    last_index, errors = match_reached_waypoints(fk_poses, targets, fk_tolerance_m=0.001, fk_tolerance_deg=1.0)
    assert last_index == 0


def test_match_reached_waypoints_no_match_returns_minus_one():
    targets = [_pose(0.0, 0.0, 0.0), _pose(0.0, 0.01, 0.0)]
    fk_poses = [_pose(5.0, 5.0, 5.0)]

    last_index, _ = match_reached_waypoints(fk_poses, targets, fk_tolerance_m=0.001, fk_tolerance_deg=1.0)
    assert last_index == -1


def test_match_reached_waypoints_never_advances_past_tolerance_breach():
    """A configuration that matches target 1 but not target 0 (out of order) must not advance the
    pointer: the pointer only ever walks forward against the one target it is currently at."""
    targets = [_pose(0.0, 0.0, 0.0), _pose(0.0, 0.01, 0.0)]
    fk_poses = [_pose(0.0, 0.01, 0.0)]  # matches target 1's pose, but pointer is still at 0

    last_index, _ = match_reached_waypoints(fk_poses, targets, fk_tolerance_m=0.001, fk_tolerance_deg=1.0)
    assert last_index == -1


def test_match_reached_waypoints_respects_angle_tolerance():
    quat_0deg = (0.0, 0.0, 0.0, 1.0)
    quat_180deg = (0.0, 0.0, 1.0, 0.0)
    targets = [Pose((0.0, 0.0, 0.0), quat_0deg)]
    fk_poses = [Pose((0.0, 0.0, 0.0), quat_180deg)]

    last_index, errors = match_reached_waypoints(fk_poses, targets, fk_tolerance_m=0.001, fk_tolerance_deg=1.0)
    assert last_index == -1
    assert errors[0]["ang_error_deg"] == pytest.approx(180.0, abs=1e-6)


# --------------------------------------------------------------------------------------
# MoveItClients
# --------------------------------------------------------------------------------------

class _FakeFuture:
    def __init__(self, done: bool, result=None, exc: Exception | None = None):
        self._done = done
        self._result = result
        self._exc = exc
        self.cancelled = False

    def done(self):
        return self._done

    def cancel(self):
        self.cancelled = True

    def result(self):
        if self._exc is not None:
            raise self._exc
        return self._result


class _FakeClient:
    srv_name = "/fake_service"

    def __init__(self, future: _FakeFuture):
        self._future = future

    def call_async(self, request):
        return self._future


class _FakeNode:
    pass


def test_moveit_clients_call_returns_result(monkeypatch):
    sentinel = object()
    future = _FakeFuture(done=True, result=sentinel)
    client = _FakeClient(future)
    monkeypatch.setattr(
        "crackvision_motion.cartesian_moveit.rclpy.spin_until_future_complete",
        lambda node, fut, timeout_sec=None: None,
    )

    clients = MoveItClients(_FakeNode(), timeout_s=1.0)
    result = clients.call(client, object(), timeout_s=1.0)
    assert result is sentinel


def test_moveit_clients_call_timeout_is_distinct_error(monkeypatch):
    future = _FakeFuture(done=False)
    client = _FakeClient(future)
    monkeypatch.setattr(
        "crackvision_motion.cartesian_moveit.rclpy.spin_until_future_complete",
        lambda node, fut, timeout_sec=None: None,
    )

    clients = MoveItClients(_FakeNode(), timeout_s=1.0)
    with pytest.raises(ServiceCallTimeoutError):
        clients.call(client, object(), timeout_s=1.0)
    assert future.cancelled is True


def test_moveit_clients_call_exception_is_distinct_error(monkeypatch):
    future = _FakeFuture(done=True, exc=RuntimeError("boom"))
    client = _FakeClient(future)
    monkeypatch.setattr(
        "crackvision_motion.cartesian_moveit.rclpy.spin_until_future_complete",
        lambda node, fut, timeout_sec=None: None,
    )

    clients = MoveItClients(_FakeNode(), timeout_s=1.0)
    with pytest.raises(ServiceCallTimeoutError):
        clients.call(client, object(), timeout_s=1.0)


def test_moveit_clients_wait_for_services_raises_precondition_error(monkeypatch):
    from crackvision_motion.cli_common import PreconditionError

    class _UnavailableClient:
        def wait_for_service(self, timeout_sec):
            return False

    class _NodeThatNeverHasServices:
        def create_client(self, srv_type, name):
            return _UnavailableClient()

    clients = MoveItClients(_NodeThatNeverHasServices(), timeout_s=0.01)
    with pytest.raises(PreconditionError):
        clients.wait_for_services()

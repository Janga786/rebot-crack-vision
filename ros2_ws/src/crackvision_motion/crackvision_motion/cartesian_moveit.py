"""cartesian_moveit -- MoveIt client layer for Cartesian/transit/IK/FK/validity requests (MOT-06.5).

A thin, testable layer between `cartesian_core` (pure-Python geometry/config, MOT-06.3) and the
node MOT-06.6 writes: request builders that turn plain Python/`cartesian_core.Pose` values into
`moveit_msgs` request messages, response parsers that turn those messages back into plain Python
results, and a small `MoveItClients` helper that creates/waits-for the five services this card's
plan ever calls (`/compute_ik`, `/compute_fk`, `/compute_cartesian_path`, `/plan_kinematic_path`,
`/check_state_validity`) and runs one timed call. Runs under the system python3 (ROS Humble,
py3.10) -- imports rclpy/moveit_msgs, so (unlike cartesian_core.py) it needs the colcon-built
overlay, but every function here is a pure message builder/parser: none of them call a service or
spin a node themselves, so the unit tests exercise them with synthetic request/response messages
and no running ROS graph (no `rclpy.init`).

`avoid_collisions` is hardcoded `True` in every IK/Cartesian request this module builds -- there is
no parameter anywhere in this file that can set it `False` (ADR-017 §7, docs/INTERFACES.md §13.2).
`group`/`frame` are likewise the fixed `GROUP`/`FRAME` constants below, not config keys, matching
every other MoveIt-calling module in this package (reachability_sweep.py, execute_trajectory.py,
plan_joint_goal.py all hardcode `"arm"`/`"base_link"` the same way).

Reused rather than reimplemented: `GetStateValidity.Request` shape from `execute_trajectory.py`'s
`OnlineChecks.collision`, the `_call_service`/`_wait_for_services` pattern from
`reachability_sweep.py` (generalised here into `MoveItClients`), and `JOINT_NAMES` from
`joint_trajectory.py`. `cartesian_core.Pose`/`Candidate` are the plain-Python pose/config types
every builder/parser here accepts or returns -- no second pose representation is introduced.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, RobotState
from moveit_msgs.srv import (
    GetCartesianPath,
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)

from .cartesian_core import Pose
from .cli_common import PreconditionError
from .joint_trajectory import JOINT_NAMES

GROUP = "arm"
FRAME = "base_link"

# §13.2 does not configure planner_id/num_planning_attempts/allowed_planning_time for
# /plan_kinematic_path -- CARTESIAN_PROBE.md's own transit query left planner_id unset (the
# pipeline's configured default planner is used) and this module does the same; these two
# values mirror plan_joint_goal.py's existing MOT-02 smoke-test defaults, the only other caller
# of /plan_kinematic_path-shaped requests in this package.
DEFAULT_NUM_PLANNING_ATTEMPTS = 5
DEFAULT_ALLOWED_PLANNING_TIME_S = 5.0

# Joint-constraint tolerance for a motion-plan goal: tight enough that "reached the goal" means
# the commanded configuration, matching plan_joint_goal.py's own DEFAULT_JOINT_GOAL tolerance.
_GOAL_JOINT_TOLERANCE_RAD = 0.001

# Only these /compute_ik codes mean "no IK solution for this pose", the §13.10 `ik_failed`
# reason; every other non-SUCCESS code is a service/request-level problem (bad group/link name,
# frame transform failure, planner internal error, ...) and must not be silently folded into
# `ik_failed` (mirrors reachability_sweep.py's own _IK_INFEASIBLE_CODES distinction).
_IK_INFEASIBLE_CODES = frozenset({MoveItErrorCodes.NO_IK_SOLUTION, MoveItErrorCodes.TIMED_OUT})

_MOVEIT_ERROR_CODE_NAMES = {
    getattr(MoveItErrorCodes, name): name
    for name in dir(MoveItErrorCodes)
    if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)
}

_SERVICE_SPECS: Tuple[Tuple[str, Any, str], ...] = (
    ("ik", GetPositionIK, "/compute_ik"),
    ("fk", GetPositionFK, "/compute_fk"),
    ("cartesian", GetCartesianPath, "/compute_cartesian_path"),
    ("motion_plan", GetMotionPlan, "/plan_kinematic_path"),
    ("validity", GetStateValidity, "/check_state_validity"),
)


def _error_name(val: int) -> str:
    return _MOVEIT_ERROR_CODE_NAMES.get(val, str(val))


class ServiceCallTimeoutError(Exception):
    """A service call did not complete within its timeout -- a distinct runtime failure (exit 1),
    never conflated with an infeasible-but-completed result like `ik_failed`/`unreachable`."""


# --------------------------------------------------------------------------------------
# small message helpers
# --------------------------------------------------------------------------------------

def _pose_msg(pose: Pose) -> PoseMsg:
    msg = PoseMsg()
    msg.position.x, msg.position.y, msg.position.z = (float(v) for v in pose.position_m)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (
        float(v) for v in pose.quat_xyzw
    )
    return msg


def _robot_state(positions: Dict[str, float]) -> RobotState:
    rs = RobotState()
    rs.joint_state.name = list(positions.keys())
    rs.joint_state.position = [float(v) for v in positions.values()]
    return rs


def _joint_trajectory_positions(joint_trajectory) -> List[List[float]]:
    """Each trajectory point's positions, reordered to `JOINT_NAMES` order."""
    names = list(joint_trajectory.joint_names)
    indices = [names.index(n) for n in JOINT_NAMES]
    return [[float(point.positions[i]) for i in indices] for point in joint_trajectory.points]


# --------------------------------------------------------------------------------------
# request builders
# --------------------------------------------------------------------------------------

def build_cartesian_request(
    start_positions: Dict[str, float], poses: Sequence[Pose], link_name: str, cfg: Dict[str, Any],
) -> GetCartesianPath.Request:
    """`/compute_cartesian_path` request for `poses` (`cartesian_core.Pose`s, already
    candidate-transformed), from `start_positions`. `group_name`/`header.frame_id` are the fixed
    `GROUP`/`FRAME`; `avoid_collisions` is unconditionally `True`; `max_step`/`jump_threshold`/
    `prismatic_jump_threshold`/`revolute_jump_threshold` come from `cfg['cartesian']`
    (docs/INTERFACES.md §13.2, the field names MOT-06.1's probe confirmed exist on this install)."""
    req = GetCartesianPath.Request()
    req.header.frame_id = FRAME
    req.start_state = _robot_state(start_positions)
    req.group_name = GROUP
    req.link_name = link_name
    req.waypoints = [_pose_msg(p) for p in poses]
    cartesian_cfg = cfg["cartesian"]
    req.max_step = float(cartesian_cfg["max_step_m"])
    req.jump_threshold = float(cartesian_cfg["jump_threshold"])
    req.prismatic_jump_threshold = float(cartesian_cfg["prismatic_jump_threshold"])
    req.revolute_jump_threshold = float(cartesian_cfg["revolute_jump_threshold"])
    req.avoid_collisions = True
    return req


def build_motion_plan_request(
    start_positions: Dict[str, float], goal_positions: Dict[str, float], cfg: Dict[str, Any],
) -> GetMotionPlan.Request:
    """`/plan_kinematic_path` request: `start_state` = `start_positions`, a joint-constraint goal
    = `goal_positions` (CARTESIAN_PROBE.md consequence 7's confirmed shape). `group_name` is the
    fixed `GROUP`; `cfg` is accepted for interface symmetry with the other builders even though
    §13.2 does not configure planner_id/attempts/time (see DEFAULT_* above)."""
    del cfg  # no §13.2 key configures this request's planner_id/attempts/time (see module docstring)
    req = GetMotionPlan.Request()
    mpr = MotionPlanRequest()
    mpr.group_name = GROUP
    mpr.start_state = _robot_state(start_positions)
    constraints = Constraints()
    for joint_name, position in goal_positions.items():
        jc = JointConstraint()
        jc.joint_name = joint_name
        jc.position = float(position)
        jc.tolerance_above = _GOAL_JOINT_TOLERANCE_RAD
        jc.tolerance_below = _GOAL_JOINT_TOLERANCE_RAD
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    mpr.goal_constraints = [constraints]
    mpr.num_planning_attempts = DEFAULT_NUM_PLANNING_ATTEMPTS
    mpr.allowed_planning_time = DEFAULT_ALLOWED_PLANNING_TIME_S
    req.motion_plan_request = mpr
    return req


def build_ik_request(
    pose: Pose, seed: Dict[str, float], link_name: str, cfg: Dict[str, Any],
) -> GetPositionIK.Request:
    """`/compute_ik` request for `pose`, seeded from `seed` (reuses
    `reachability_sweep._build_ik_request`'s field layout). `avoid_collisions` is unconditionally
    `True`; the solver timeout is `cfg['service_timeout_s']` -- §13.2 has no separate IK-solver
    timeout key, so the one `cartesian_planning.yaml` timeout value (the per-service-call budget)
    is reused for both the request's own internal solver timeout and the call's wait timeout."""
    req = GetPositionIK.Request()
    req.ik_request.group_name = GROUP
    req.ik_request.ik_link_name = link_name
    req.ik_request.avoid_collisions = True
    ps = PoseStamped()
    ps.header.frame_id = FRAME
    ps.pose = _pose_msg(pose)
    req.ik_request.pose_stamped = ps
    timeout_s = float(cfg["service_timeout_s"])
    sec = int(timeout_s)
    req.ik_request.timeout.sec = sec
    req.ik_request.timeout.nanosec = int(round((timeout_s - sec) * 1e9))
    req.ik_request.robot_state = _robot_state(seed)
    return req


def build_fk_request(positions: Dict[str, float], links: Sequence[str]) -> GetPositionFK.Request:
    """`/compute_fk` request for `positions`, evaluated at every link in `links`."""
    req = GetPositionFK.Request()
    req.header.frame_id = FRAME
    req.fk_link_names = list(links)
    req.robot_state = _robot_state(positions)
    return req


def build_validity_request(positions: Dict[str, float]) -> GetStateValidity.Request:
    """`/check_state_validity` request for `positions` against the group `GROUP` -- the same
    request shape `execute_trajectory.OnlineChecks.collision` already builds."""
    req = GetStateValidity.Request()
    req.group_name = GROUP
    req.robot_state.joint_state.name = list(JOINT_NAMES)
    req.robot_state.joint_state.position = [float(positions[n]) for n in JOINT_NAMES]
    return req


# --------------------------------------------------------------------------------------
# response parsers
# --------------------------------------------------------------------------------------

def parse_cartesian(resp) -> Tuple[List[List[float]], float, Optional[str]]:
    """`(configs, fraction, error)`. MoveIt's own per-point timestamps are discarded -- only
    `positions` survive into `configs`. A non-`SUCCESS` error code (a request-level problem, not
    the ordinary collision-truncation path CARTESIAN_PROBE.md fact 5 documents, which returns
    `SUCCESS` with `fraction < 1`) maps to the §13.10 `cartesian_incomplete` reason, the same
    reason a `SUCCESS`-but-partial (`fraction < 1`) result gets: this module does not attempt the
    §13.8/§13.7 `collision`/`joint_jump` re-classification, which needs extra IK/validity calls
    this layer deliberately leaves to its caller."""
    fraction = float(resp.fraction)
    if resp.error_code.val != MoveItErrorCodes.SUCCESS:
        return [], fraction, "cartesian_incomplete"
    configs = _joint_trajectory_positions(resp.solution.joint_trajectory)
    error = None if fraction >= 1.0 else "cartesian_incomplete"
    return configs, fraction, error


def parse_motion_plan(resp) -> Tuple[List[List[float]], Optional[str]]:
    """`(configs, error)`. A non-`SUCCESS` `motion_plan_response.error_code` maps to the §13.10
    `transit_failed` reason (§13.5 step 2)."""
    mpr = resp.motion_plan_response
    if mpr.error_code.val != MoveItErrorCodes.SUCCESS:
        return [], "transit_failed"
    return _joint_trajectory_positions(mpr.trajectory.joint_trajectory), None


def parse_ik(resp) -> Tuple[Optional[Dict[str, float]], Optional[str]]:
    """`(joints, error)`. `NO_IK_SOLUTION`/`TIMED_OUT` map to the §13.10 `ik_failed` reason (§13.5
    step 1); any other non-`SUCCESS` code is a service/request-level problem and is surfaced as a
    distinct `ik_error:<CODE_NAME>` string, never silently folded into `ik_failed`."""
    if resp.error_code.val == MoveItErrorCodes.SUCCESS:
        joints = {
            name: float(value)
            for name, value in zip(resp.solution.joint_state.name, resp.solution.joint_state.position)
            if name in JOINT_NAMES
        }
        return joints, None
    if resp.error_code.val in _IK_INFEASIBLE_CODES:
        return None, "ik_failed"
    return None, f"ik_error:{_error_name(resp.error_code.val)}"


def _pose_from_msg(pose_msg: PoseMsg) -> Pose:
    return Pose(
        (float(pose_msg.position.x), float(pose_msg.position.y), float(pose_msg.position.z)),
        (
            float(pose_msg.orientation.x), float(pose_msg.orientation.y),
            float(pose_msg.orientation.z), float(pose_msg.orientation.w),
        ),
    )


def parse_fk(resp, link_name: str) -> Tuple[Optional[Pose], Optional[str]]:
    """`(pose, error)` for `link_name` within `resp.fk_link_names`/`resp.pose_stamped`. A
    non-`SUCCESS` error code (or `link_name` missing from the response) surfaces as a distinct
    `fk_error:<CODE_NAME>` string -- `/compute_fk` failing is a service/request-level problem, not
    a §13.10 planning-outcome reason."""
    if resp.error_code.val != MoveItErrorCodes.SUCCESS:
        return None, f"fk_error:{_error_name(resp.error_code.val)}"
    try:
        index = list(resp.fk_link_names).index(link_name)
    except ValueError:
        return None, f"fk_error:link '{link_name}' not in response"
    return _pose_from_msg(resp.pose_stamped[index].pose), None


def _quat_angle_deg(q1: Sequence[float], q2: Sequence[float]) -> float:
    """Shortest-path angle between two xyzw unit quaternions, in degrees."""
    dot = max(-1.0, min(1.0, abs(sum(a * b for a, b in zip(q1, q2)))))
    return math.degrees(2.0 * math.acos(dot))


def match_reached_waypoints(
    fk_poses: Sequence[Pose], target_poses: Sequence[Pose], fk_tolerance_m: float, fk_tolerance_deg: float,
) -> Tuple[int, List[Dict[str, float]]]:
    """§13.7 FK matching: walk `fk_poses` (one per returned trajectory configuration, in order)
    and `target_poses` (the requested trace waypoints) with a pointer starting at 0; a
    configuration advances the pointer when it agrees with the pointed-at target within
    `fk_tolerance_m`/`fk_tolerance_deg`. Returns `(last_index, errors)`: `last_index` is the
    highest target-pose index ever matched, minus one position (i.e. the pointer's final value
    minus one), `-1` if no configuration ever matched target 0; `errors` is every attempted
    `(pos_error_m, ang_error_deg)` comparison against the currently pointed-at target, for
    diagnostics. Never uses `fraction` or the fk_poses/target_poses index counts directly (ADR-017
    §3) -- this is the only decision rule."""
    pointer = 0
    errors: List[Dict[str, float]] = []
    n_targets = len(target_poses)
    for fk_pose in fk_poses:
        if pointer >= n_targets:
            break
        target = target_poses[pointer]
        pos_error_m = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(fk_pose.position_m, target.position_m))
        )
        ang_error_deg = _quat_angle_deg(fk_pose.quat_xyzw, target.quat_xyzw)
        errors.append({"pos_error_m": pos_error_m, "ang_error_deg": ang_error_deg})
        if pos_error_m <= fk_tolerance_m and ang_error_deg <= fk_tolerance_deg:
            pointer += 1
    return pointer - 1, errors


# --------------------------------------------------------------------------------------
# MoveItClients
# --------------------------------------------------------------------------------------

class MoveItClients:
    """Creates the five services this card's plan ever calls on `node`, waits for them (each
    within `timeout_s`), and runs one timed call via `call()`. No action client anywhere in this
    class -- nothing here can command a trajectory."""

    def __init__(self, node, timeout_s: float):
        self.node = node
        self.timeout_s = timeout_s
        self.clients: Dict[str, Any] = {}

    def wait_for_services(self) -> Dict[str, Any]:
        deadline = time.monotonic() + self.timeout_s
        for key, srv_type, name in _SERVICE_SPECS:
            client = self.node.create_client(srv_type, name)
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not client.wait_for_service(timeout_sec=max(remaining, 0.0)):
                raise PreconditionError(
                    f"MoveIt service '{name}' did not become available within {self.timeout_s}s"
                )
            self.clients[key] = client
        return self.clients

    def call(self, client, request, timeout_s: Optional[float] = None):
        """One timed `call_async`/`spin_until_future_complete` round-trip. Raises
        `ServiceCallTimeoutError` (a distinct, non-'unreachable' failure) if the call does not
        complete within `timeout_s` (default `self.timeout_s`) or raises while completing."""
        effective_timeout_s = self.timeout_s if timeout_s is None else timeout_s
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=effective_timeout_s)
        if not future.done():
            future.cancel()
            raise ServiceCallTimeoutError(
                f"service call to {client.srv_name!r} did not complete within {effective_timeout_s}s"
            )
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001 - surfaced as a distinct runtime failure, not swallowed
            raise ServiceCallTimeoutError(
                f"service call to {client.srv_name!r} raised: {exc}"
            ) from exc

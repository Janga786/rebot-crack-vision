#!/usr/bin/env python3
"""scripts/ros/probe_cartesian_path.py — live probe of /compute_cartesian_path and
/plan_kinematic_path for the ADR-014 task frames (MOT-06.1).

Read-only research probe against the headless mock MoveIt stack (scripts/ros/mock_planning.launch.py
via scripts/ros/probe_cartesian_path.sh). It only ever calls /apply_planning_scene,
/get_planning_scene, /compute_ik, /compute_fk, /compute_cartesian_path and /plan_kinematic_path --
never an action client, so nothing here can move the mock (let alone real) arm. It never sets
avoid_collisions to false on a /compute_cartesian_path call; the one exception is the
singularity_seed_sweep() diagnostic, which also tries avoid_collisions=False /compute_ik calls
(still just IK -- no path, no motion) to confirm the roll=0 IK failure is not a collision artifact.

If the IK fallback used to seed steps 3-7 also fails (see SINGULARITY_FALLBACK_ROLL_DEG), the probe
writes its partial JSON and exits 1 (EXIT_RUNTIME) rather than 0, since steps 3-7 never ran.

Applies a probe-local scene (table top at z=0 + a 0.20x0.20 m specimen box, top face z=0.010, at
the recommended placement centre from config/motion/specimen_placement.yaml), IKs a tool_tip pose
0.04 m above the specimen with +X down, then runs a series of /compute_cartesian_path and
/compute_fk queries plus one /plan_kinematic_path call, recording every raw observation (including
the GetCartesianPath request's actual field names on this Humble install, via
get_fields_and_field_types()) to --out as JSON.

Usage:
    scripts/ros/probe_cartesian_path.py --out logs/cartesian_probe.json

Exit codes (card MOT-06.1 §0): 0 = probe completed and the JSON was written, whatever the facts
turned out to be; 1 = a runtime failure (a service call failed/timed out); 3 = a required service
did not become available.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
)
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

EXIT_OK = 0
EXIT_RUNTIME = 1
EXIT_PRECONDITION = 3

SERVICE_WAIT_S = 90.0
CALL_TIMEOUT_S = 15.0

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
GRIPPER_JOINTS = ("gripper_joint1", "gripper_joint2")
HOME_SEED = {name: 0.0 for name in (*ARM_JOINTS, *GRIPPER_JOINTS)}

GROUP = "arm"
# Discovered empirically (see docs/motion/CARTESIAN_PROBE.md): "+X straight down, roll=0" sits
# exactly on a wrist singularity for this arm -- /compute_ik returns NO_IK_SOLUTION there for
# every seed tried, while any nonzero roll a few degrees away succeeds. This epsilon is the
# fallback roll used to obtain a usable IK seed for the rest of the probe once that literal
# failure is recorded.
SINGULARITY_FALLBACK_ROLL_DEG = 2.0
TABLE_TOP_Z = 0.0
SPECIMEN_CENTER_XY = (0.26, 0.0)
SPECIMEN_FOOTPRINT = (0.20, 0.20)
SPECIMEN_TOP_Z = 0.010
SPECIMEN_HEIGHT = 0.05  # top face at SPECIMEN_TOP_Z, extends down to the table top
TABLE_DIMS = (1.0, 1.0, 0.02)  # top face at TABLE_TOP_Z


def _quat_x_down() -> Tuple[float, float, float, float]:
    """(x, y, z, w) rotating local +X onto world -Z: 90 deg about the world Y axis."""
    half = math.pi / 4.0
    return (0.0, math.sin(half), 0.0, math.cos(half))


def _quat_mul(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_roll_x(angle_rad: float) -> Tuple[float, float, float, float]:
    half = angle_rad / 2.0
    return (math.sin(half), 0.0, 0.0, math.cos(half))


def _quat_to_angle_deg(a, b) -> float:
    """Angle between two unit quaternions (shortest-path), in degrees."""
    dot = sum(ai * bi for ai, bi in zip(a, b))
    dot = max(-1.0, min(1.0, abs(dot)))
    return math.degrees(2.0 * math.acos(dot))


def _call(node: Node, client, request, timeout_s: float = CALL_TIMEOUT_S):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        future.cancel()
        return None
    try:
        return future.result()
    except Exception:  # noqa: BLE001 - mapped to a runtime failure by the caller
        return None


# --------------------------------------------------------------------------------------
# probe-local scene (built in-script; never config/scene/scene.yaml)
# --------------------------------------------------------------------------------------

def _box_collision_object(obj_id: str, dims, center_xyz) -> CollisionObject:
    co = CollisionObject()
    co.header.frame_id = "base_link"
    co.id = obj_id
    co.operation = CollisionObject.ADD
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = list(dims)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = center_xyz
    pose.orientation.w = 1.0
    co.primitives = [prim]
    co.primitive_poses = [pose]
    return co


def apply_probe_scene(node: Node, get_scene_client, apply_scene_client) -> None:
    table = _box_collision_object(
        "table", TABLE_DIMS, (0.0, 0.0, TABLE_TOP_Z - TABLE_DIMS[2] / 2.0)
    )
    specimen = _box_collision_object(
        "specimen",
        (SPECIMEN_FOOTPRINT[0], SPECIMEN_FOOTPRINT[1], SPECIMEN_HEIGHT),
        (SPECIMEN_CENTER_XY[0], SPECIMEN_CENTER_XY[1], SPECIMEN_TOP_Z - SPECIMEN_HEIGHT / 2.0),
    )

    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    resp = _call(node, get_scene_client, req)
    if resp is None:
        raise RuntimeError("get_planning_scene failed while reading the ACM")

    # The robot base is bolted to the table (table top z=0 == base_link's bottom face), so
    # table/base_link must be an allowed pair -- mirrors config/scene/scene.yaml's rationale.
    # Without it every joint state collides with the table and IK/Cartesian queries would
    # fail with NO_IK_SOLUTION regardless of reachability.
    allowed_pairs = {("table", "base_link"), ("base_link", "table")}

    def is_allowed(a: str, b: str) -> bool:
        return a != b and (a, b) in allowed_pairs

    old_names = list(resp.scene.allowed_collision_matrix.entry_names)
    old_rows = [list(v.enabled) for v in resp.scene.allowed_collision_matrix.entry_values]
    new_ids = [oid for oid in ("table", "specimen") if oid not in old_names]
    all_names = old_names + new_ids
    new_rows: List[List[bool]] = []
    for i, name_i in enumerate(old_names):
        new_rows.append(old_rows[i] + [is_allowed(name_i, nid) for nid in new_ids])
    for name_i in new_ids:
        new_rows.append([is_allowed(name_i, name_j) for name_j in all_names])
    new_acm = AllowedCollisionMatrix()
    new_acm.entry_names = all_names
    new_acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in new_rows]
    new_acm.default_entry_names = list(resp.scene.allowed_collision_matrix.default_entry_names)
    new_acm.default_entry_values = list(resp.scene.allowed_collision_matrix.default_entry_values)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [table, specimen]
    scene.allowed_collision_matrix = new_acm
    apply_req = ApplyPlanningScene.Request()
    apply_req.scene = scene
    apply_resp = _call(node, apply_scene_client, apply_req)
    if apply_resp is None or not apply_resp.success:
        raise RuntimeError("apply_planning_scene failed for the probe-local scene")


# --------------------------------------------------------------------------------------
# IK / FK / cartesian / motion-plan requests
# --------------------------------------------------------------------------------------

def _pose(x: float, y: float, z: float, quat) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
    return p


def compute_ik(node: Node, client, link_name: str, x: float, y: float, z: float, quat,
                avoid_collisions: bool = True, seed: Dict[str, float] = None):
    req = GetPositionIK.Request()
    req.ik_request.group_name = GROUP
    req.ik_request.ik_link_name = link_name
    req.ik_request.avoid_collisions = avoid_collisions
    ps = PoseStamped()
    ps.header.frame_id = "base_link"
    ps.pose = _pose(x, y, z, quat)
    req.ik_request.pose_stamped = ps
    req.ik_request.timeout.sec = 5
    seed = seed if seed is not None else HOME_SEED
    rs = RobotState()
    rs.joint_state.name = list(seed.keys())
    rs.joint_state.position = [float(v) for v in seed.values()]
    req.ik_request.robot_state = rs
    return _call(node, client, req)


def singularity_seed_sweep(node: Node, ik_client, link_name: str, x: float, y: float, z: float,
                            quat) -> dict:
    """Systematic recheck of the roll=0 IK failure: home seed plus 10 fixed (reproducible, not
    interactively-picked) random seeds, each tried with avoid_collisions both True and False.
    Logged in full so the doc's seed/collision-independence claim is reproducible from the JSON,
    not just asserted."""
    rng = __import__("random").Random(20261010)
    seeds = [dict(HOME_SEED)]
    for _ in range(10):
        seed = dict(HOME_SEED)
        for j in ARM_JOINTS:
            seed[j] = rng.uniform(-math.pi, math.pi)
        seeds.append(seed)
    trials = []
    for avoid_collisions in (True, False):
        for seed_index, seed in enumerate(seeds):
            resp = compute_ik(node, ik_client, link_name, x, y, z, quat,
                               avoid_collisions=avoid_collisions, seed=seed)
            trials.append({
                "seed_index": seed_index,
                "avoid_collisions": avoid_collisions,
                "error_code": None if resp is None else resp.error_code.val,
                "success": resp is not None and resp.error_code.val == MoveItErrorCodes.SUCCESS,
            })
    return {
        "num_trials": len(trials),
        "num_seeds": len(seeds),
        "all_no_ik_solution": all(not t["success"] for t in trials),
        "trials": trials,
    }


def compute_fk(node: Node, client, link_name: str, robot_state: RobotState):
    req = GetPositionFK.Request()
    req.header.frame_id = "base_link"
    req.fk_link_names = [link_name]
    req.robot_state = robot_state
    return _call(node, client, req)


def compute_cartesian_path(node: Node, client, link_name: str, start_state: RobotState,
                            waypoints: List[Pose], avoid_collisions: bool = True):
    req = GetCartesianPath.Request()
    req.header.frame_id = "base_link"
    req.start_state = start_state
    req.group_name = GROUP
    req.link_name = link_name
    req.waypoints = waypoints
    req.max_step = 0.005
    req.jump_threshold = 0.0
    req.avoid_collisions = avoid_collisions
    return _call(node, client, req)


def _line_waypoints(x: float, y0: float, z: float, length: float, spacing: float,
                     roll_ramp: bool, base_quat) -> List[Pose]:
    n = int(round(length / spacing))
    out = []
    for i in range(n + 1):
        y = y0 + i * spacing
        if roll_ramp:
            roll = (math.pi / 2.0) * (i / n)
            quat = _quat_mul(base_quat, _quat_roll_x(roll))
        else:
            quat = base_quat
        out.append(_pose(x, y, z, quat))
    return out


def _traj_summary(trajectory) -> dict:
    pts = trajectory.joint_trajectory.points
    return {
        "num_points": len(pts),
        "last_time_from_start_s": (
            pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9 if pts else None
        ),
        "first_time_from_start_s": (
            pts[0].time_from_start.sec + pts[0].time_from_start.nanosec * 1e-9 if pts else None
        ),
    }


def _pose_pos_error(fk_pose: Pose, target_pose: Pose) -> float:
    dx = fk_pose.position.x - target_pose.position.x
    dy = fk_pose.position.y - target_pose.position.y
    dz = fk_pose.position.z - target_pose.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _fk_error_nearest(node: Node, fk_client, link_name: str, robot_state: RobotState,
                       candidate_poses: List[Pose]) -> dict:
    """FK the given robot_state and compare to the NEAREST (by position) of `candidate_poses`,
    not the same-index waypoint: the returned RobotTrajectory is resampled during time
    parameterization and its point count does not match len(waypoints), so same-index comparison
    overstates tracking error (see docs/motion/CARTESIAN_PROBE.md)."""
    fk_resp = compute_fk(node, fk_client, link_name, robot_state)
    if fk_resp is None or fk_resp.error_code.val != MoveItErrorCodes.SUCCESS or not fk_resp.pose_stamped:
        return {"ok": False, "error_code": None if fk_resp is None else fk_resp.error_code.val}
    fk_pose = fk_resp.pose_stamped[0].pose
    best_idx, best_err = min(
        ((i, _pose_pos_error(fk_pose, p)) for i, p in enumerate(candidate_poses)),
        key=lambda t: t[1],
    )
    target_pose = candidate_poses[best_idx]
    ang_err = _quat_to_angle_deg(
        (fk_pose.orientation.x, fk_pose.orientation.y, fk_pose.orientation.z, fk_pose.orientation.w),
        (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w),
    )
    return {"ok": True, "pos_error_m": best_err, "angle_error_deg": ang_err, "nearest_waypoint_index": best_idx}


def _robot_state_from_traj_point(seed_names: List[str], joint_names: List[str], positions: List[float],
                                  fixed: Dict[str, float]) -> RobotState:
    rs = RobotState()
    name_to_pos = dict(zip(joint_names, positions))
    names = list(seed_names)
    rs.joint_state.name = names
    rs.joint_state.position = [name_to_pos.get(n, fixed.get(n, 0.0)) for n in names]
    return rs


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    repo_root = Path(__file__).resolve().parents[2]
    p.add_argument("--out", type=Path, default=repo_root / "logs" / "cartesian_probe.json")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    result: Dict = {"probe": "cartesian_path", "scene": {}, "queries": {}}

    rclpy.init(args=None)
    node = Node("probe_cartesian_path")
    try:
        clients = {
            "get_scene": node.create_client(GetPlanningScene, "/get_planning_scene"),
            "apply_scene": node.create_client(ApplyPlanningScene, "/apply_planning_scene"),
            "ik": node.create_client(GetPositionIK, "/compute_ik"),
            "fk": node.create_client(GetPositionFK, "/compute_fk"),
            "cartesian": node.create_client(GetCartesianPath, "/compute_cartesian_path"),
            "motion_plan": node.create_client(GetMotionPlan, "/plan_kinematic_path"),
        }
        for key, client in clients.items():
            if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
                print(f"probe_cartesian_path: service for '{key}' not available within "
                      f"{SERVICE_WAIT_S}s", file=sys.stderr)
                return EXIT_PRECONDITION

        result["request_fields"] = {
            "GetCartesianPath.Request": GetCartesianPath.Request.get_fields_and_field_types(),
            "GetMotionPlan.Request (via MotionPlanRequest)": MotionPlanRequest.get_fields_and_field_types(),
        }

        # 1. probe-local scene
        try:
            apply_probe_scene(node, clients["get_scene"], clients["apply_scene"])
        except RuntimeError as exc:
            print(f"probe_cartesian_path: {exc}", file=sys.stderr)
            return EXIT_RUNTIME
        result["scene"] = {
            "table_top_z_m": TABLE_TOP_Z,
            "specimen_center_xy_m": list(SPECIMEN_CENTER_XY),
            "specimen_footprint_m": list(SPECIMEN_FOOTPRINT),
            "specimen_top_z_m": SPECIMEN_TOP_Z,
        }

        approach_x, approach_y = SPECIMEN_CENTER_XY
        quat_down_literal = _quat_x_down()

        # 2. /compute_ik for tool_tip, 0.04 m clearance above the specimen, +X down, literally
        # as specified (roll=0 about the boresight).
        ik_z = SPECIMEN_TOP_Z + 0.04
        ik_resp = compute_ik(node, clients["ik"], "tool_tip", approach_x, approach_y, ik_z, quat_down_literal)
        if ik_resp is None:
            print("probe_cartesian_path: /compute_ik call failed/timed out", file=sys.stderr)
            return EXIT_RUNTIME
        result["queries"]["compute_ik_tool_tip"] = {
            "error_code": ik_resp.error_code.val,
            "success": ik_resp.error_code.val == MoveItErrorCodes.SUCCESS,
            "requested_pose": {"x": approach_x, "y": approach_y, "z": ik_z, "quat_xyzw": list(quat_down_literal)},
            "joint_state": dict(zip(ik_resp.solution.joint_state.name, ik_resp.solution.joint_state.position)),
            "note": None,
        }
        ik_state = ik_resp.solution
        quat_down = quat_down_literal
        if ik_resp.error_code.val != MoveItErrorCodes.SUCCESS:
            # roll=0 ("+X straight down" exactly) is a wrist singularity for this arm (see
            # docs/motion/CARTESIAN_PROBE.md): no seed finds a solution there. Fall back to a
            # small roll offset purely so the rest of this probe (steps 3-7) has a usable seed;
            # this substitution is recorded, not hidden.
            quat_fallback = _quat_mul(quat_down_literal, _quat_roll_x(math.radians(SINGULARITY_FALLBACK_ROLL_DEG)))
            fallback_resp = compute_ik(node, clients["ik"], "tool_tip", approach_x, approach_y, ik_z, quat_fallback)
            result["queries"]["compute_ik_tool_tip"]["note"] = (
                f"roll=0 returned NO_IK_SOLUTION-class error {ik_resp.error_code.val}; retried with a "
                f"{SINGULARITY_FALLBACK_ROLL_DEG} deg roll fallback for steps 3-7 (see "
                "compute_ik_tool_tip_singularity_fallback)"
            )
            result["queries"]["compute_ik_tool_tip_singularity_fallback"] = {
                "roll_deg": SINGULARITY_FALLBACK_ROLL_DEG,
                "error_code": None if fallback_resp is None else fallback_resp.error_code.val,
                "success": fallback_resp is not None and fallback_resp.error_code.val == MoveItErrorCodes.SUCCESS,
            }
            # Systematic recheck (not just the single home seed above): home + 10 fixed seeds,
            # each with avoid_collisions both True and False, logged in full.
            result["queries"]["compute_ik_tool_tip_singularity_seed_sweep"] = singularity_seed_sweep(
                node, clients["ik"], "tool_tip", approach_x, approach_y, ik_z, quat_down_literal
            )
            if fallback_resp is None or fallback_resp.error_code.val != MoveItErrorCodes.SUCCESS:
                print("probe_cartesian_path: fallback IK also failed; writing partial results and stopping",
                      file=sys.stderr)
                _write(args.out, result)
                return EXIT_RUNTIME
            ik_state = fallback_resp.solution
            quat_down = quat_fallback

        # 3a. descend to 0.01 m clearance, then a straight 0.06 m line along +Y at 0.01 m,
        #     0.002 m spacing, no roll.
        line_z = SPECIMEN_TOP_Z + 0.01
        descend = _pose(approach_x, approach_y, line_z, quat_down)
        waypoints_a = [descend] + _line_waypoints(approach_x, approach_y, line_z, 0.06, 0.002, roll_ramp=False, base_quat=quat_down)
        resp_a = compute_cartesian_path(node, clients["cartesian"], "tool_tip", ik_state, waypoints_a)
        if resp_a is None:
            print("probe_cartesian_path: /compute_cartesian_path (set a) failed/timed out", file=sys.stderr)
            return EXIT_RUNTIME
        result["queries"]["cartesian_tool_tip_line_no_roll"] = {
            "error_code": resp_a.error_code.val,
            "fraction": resp_a.fraction,
            "num_requested_waypoints": len(waypoints_a),
            "trajectory": _traj_summary(resp_a.solution),
        }

        # 4. FK-check every returned configuration of run (a) against the nearest (by position)
        # requested waypoint -- NOT the same-index waypoint, since the returned RobotTrajectory
        # is resampled by time parameterization and its point count differs from len(waypoints).
        pts = resp_a.solution.joint_trajectory.points
        jt_names = list(resp_a.solution.joint_trajectory.joint_names)
        fk_errors = [
            _fk_error_nearest(
                node, clients["fk"], "tool_tip",
                _robot_state_from_traj_point(list(HOME_SEED.keys()), jt_names, list(pt.positions), HOME_SEED),
                waypoints_a,
            )
            for pt in pts
        ]
        ok_errors = [e for e in fk_errors if e.get("ok")]
        # The trajectory's leading points are still en route along the initial ramp segment from
        # start_state's pose to waypoints_a[0] (the "descend" pose): their nearest requested
        # waypoint is necessarily waypoints_a[0] (index 0) because nothing closer exists yet, but
        # the point itself is not there yet, so each reports a large "error" that is actually ramp
        # transit position, not tracking error. Exclude every leading point that still maps to
        # that same nearest index before measuring real per-point tracking error.
        ramp_nearest_index = ok_errors[0]["nearest_waypoint_index"] if ok_errors else None
        first_non_ramp = next(
            (i for i, e in enumerate(ok_errors) if e["nearest_waypoint_index"] != ramp_nearest_index),
            len(ok_errors),
        )
        ok_errors_excl_ramp = ok_errors[first_non_ramp:]
        result["queries"]["fk_check_line_no_roll"] = {
            "num_points_checked": len(fk_errors),
            "max_pos_error_m": max((e["pos_error_m"] for e in ok_errors), default=None),
            "max_angle_error_deg": max((e["angle_error_deg"] for e in ok_errors), default=None),
            "max_pos_error_m_excluding_ramp_segment": max(
                (e["pos_error_m"] for e in ok_errors_excl_ramp), default=None
            ),
            "max_angle_error_deg_excluding_ramp_segment": max(
                (e["angle_error_deg"] for e in ok_errors_excl_ramp), default=None
            ),
            "last_fk_point_nearest_waypoint_index": ok_errors[-1]["nearest_waypoint_index"] if ok_errors else None,
            "last_reached_waypoint_index_from_fraction": round(resp_a.fraction * (len(waypoints_a) - 1)),
            "note": (
                "trajectory.num_points != num_requested_waypoints (resampled by time parameterization); "
                "map fraction to the last reached requested waypoint via "
                "round(fraction * (len(waypoints) - 1)), not via trajectory point count or index. "
                "max_pos_error_m includes the first trajectory point, which is still on the ramp from "
                "start_state's pose to waypoints_a[0] and so is not a tracking-error sample; "
                "max_pos_error_m_excluding_ramp_segment drops it."
            ),
        }

        # 5. same waypoint set but one waypoint placed 0.005 m inside the specimen (avoid_collisions true).
        inside_z = SPECIMEN_TOP_Z - 0.005
        waypoints_collide = [descend] + [
            _pose(approach_x, approach_y, inside_z, quat_down),
        ] + _line_waypoints(approach_x, approach_y, line_z, 0.06, 0.002, roll_ramp=False, base_quat=quat_down)
        resp_collide = compute_cartesian_path(node, clients["cartesian"], "tool_tip", ik_state,
                                               waypoints_collide, avoid_collisions=True)
        if resp_collide is None:
            print("probe_cartesian_path: /compute_cartesian_path (collision set) failed/timed out",
                  file=sys.stderr)
            return EXIT_RUNTIME
        result["queries"]["cartesian_tool_tip_collision_waypoint"] = {
            "error_code": resp_collide.error_code.val,
            "fraction": resp_collide.fraction,
            "num_requested_waypoints": len(waypoints_collide),
            "last_reached_point_index": len(resp_collide.solution.joint_trajectory.points) - 1,
            "trajectory": _traj_summary(resp_collide.solution),
        }

        # 3b. the same line with roll about tool_tip +X ramping 0 -> 90 deg.
        waypoints_b = [descend] + _line_waypoints(approach_x, approach_y, line_z, 0.06, 0.002, roll_ramp=True, base_quat=quat_down)
        resp_b = compute_cartesian_path(node, clients["cartesian"], "tool_tip", ik_state, waypoints_b)
        if resp_b is None:
            print("probe_cartesian_path: /compute_cartesian_path (roll ramp) failed/timed out", file=sys.stderr)
            return EXIT_RUNTIME
        result["queries"]["cartesian_tool_tip_line_roll_ramp"] = {
            "error_code": resp_b.error_code.val,
            "fraction": resp_b.fraction,
            "num_requested_waypoints": len(waypoints_b),
            "trajectory": _traj_summary(resp_b.solution),
        }

        # 6. camera_link view pose 0.25 m above the specimen centre, same line pattern.
        view_z = SPECIMEN_TOP_Z + 0.25
        ik_view = compute_ik(node, clients["ik"], "camera_link", approach_x, approach_y, view_z, quat_down)
        if ik_view is None:
            print("probe_cartesian_path: /compute_ik (camera_link) call failed/timed out", file=sys.stderr)
            return EXIT_RUNTIME
        result["queries"]["compute_ik_camera_link"] = {
            "error_code": ik_view.error_code.val,
            "success": ik_view.error_code.val == MoveItErrorCodes.SUCCESS,
            "requested_pose": {"x": approach_x, "y": approach_y, "z": view_z, "quat_xyzw": list(quat_down)},
        }
        if ik_view.error_code.val == MoveItErrorCodes.SUCCESS:
            cam_descend = _pose(approach_x, approach_y, view_z, quat_down)
            cam_line = [cam_descend] + _line_waypoints(approach_x, approach_y, view_z, 0.06, 0.002, roll_ramp=False, base_quat=quat_down)
            resp_cam = compute_cartesian_path(node, clients["cartesian"], "camera_link", ik_view.solution, cam_line)
            if resp_cam is None:
                print("probe_cartesian_path: /compute_cartesian_path (camera_link) failed/timed out",
                      file=sys.stderr)
                return EXIT_RUNTIME
            result["queries"]["cartesian_camera_link_line"] = {
                "error_code": resp_cam.error_code.val,
                "fraction": resp_cam.fraction,
                "num_requested_waypoints": len(cam_line),
                "trajectory": _traj_summary(resp_cam.solution),
            }

        # 7. /plan_kinematic_path with an explicit start_state (home) and a joint-constraint goal
        #    equal to the step-2 IK solution.
        start_state = RobotState()
        start_state.joint_state.name = list(HOME_SEED.keys())
        start_state.joint_state.position = [float(v) for v in HOME_SEED.values()]

        goal_constraints = Constraints()
        for name, pos in zip(ik_state.joint_state.name, ik_state.joint_state.position):
            if name not in ARM_JOINTS:
                continue
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        mp_req = GetMotionPlan.Request()
        mp_req.motion_plan_request = MotionPlanRequest()
        mp_req.motion_plan_request.group_name = GROUP
        mp_req.motion_plan_request.start_state = start_state
        mp_req.motion_plan_request.goal_constraints = [goal_constraints]
        mp_req.motion_plan_request.num_planning_attempts = 1
        mp_req.motion_plan_request.allowed_planning_time = 5.0
        mp_resp = _call(node, clients["motion_plan"], mp_req)
        if mp_resp is None:
            print("probe_cartesian_path: /plan_kinematic_path call failed/timed out", file=sys.stderr)
            return EXIT_RUNTIME
        mpr = mp_resp.motion_plan_response
        result["queries"]["plan_kinematic_path"] = {
            "error_code": mpr.error_code.val,
            "success": mpr.error_code.val == MoveItErrorCodes.SUCCESS,
            "planner_id_requested": mp_req.motion_plan_request.planner_id,
            "group_name": mpr.group_name,
            "num_points": len(mpr.trajectory.joint_trajectory.points),
            "planning_time_s": mpr.planning_time,
        }

    finally:
        node.destroy_node()
        rclpy.shutdown()

    _write(args.out, result)
    print(f"probe_cartesian_path: wrote {args.out}")
    return EXIT_OK


def _write(out_path: Path, result: dict) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__":
    sys.exit(main())

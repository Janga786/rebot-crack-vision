"""reachability_sweep -- MoveIt reachability sweep node with an independent FK re-check (MOT-04.4).

console_script: `ros2 run crackvision_motion reachability_sweep`. Runs under the system python3
(ROS Humble, py3.10), never `./env.sh` -- invoked via `scripts/ros/env_ros.sh` or
`scripts/ros/run_reachability.sh`. See `docs/INTERFACES.md` §7.4 for the CLI contract and exit
codes and §7.2 for the output schema.

This node only ever calls `/compute_ik`, `/compute_fk`, `/get_planning_scene` and
`/apply_planning_scene`, and only ever subscribes to `/robot_description` and
`/robot_description_semantic`. There is no action client anywhere in this file and nothing here
can command a trajectory -- IK/FK are pure kinematics queries, and the planning-scene calls only
add/remove a static collision slab used for IK's own collision checking.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import itertools
import math
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK, GetPositionIK
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from .cli_common import PreconditionError, RunSummary, add_common_args, find_root, run_cli, setup_logging
from .reachability_core import ConfigError, config_sha256, enumerate_targets, load_config, orientations, prefiltered, reach_bound_from_urdf, target_position
from .reachability_map import add_target, finalize, joint_limit_margin, limits_from_yaml, new_map, write_map

TOOL = "reachability_sweep"

DEFAULT_CONFIG = "config/motion/reachability.yaml"
DEFAULT_LIMITS = "config/robot/b601_dm_limits.yaml"
DEFAULT_OUT = "data/motion/reachability_map.json"
DEFAULT_SERVICE_TIMEOUT_S = 90.0
DEFAULT_MAX_DURATION_S = 0.0

_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
_SURFACE_OBJECT_ID = "mot04_surface"

_FK_POSITION_TOL_M = 1e-3
_FK_AXIS_TOL_DEG = 0.5

_IK_CALL_TIMEOUT_MARGIN_S = 2.0
_FK_CALL_TIMEOUT_S = 5.0
_SCENE_CALL_TIMEOUT_S = 10.0
_SRDF_TIMEOUT_S = 5.0
_TOPIC_SPIN_STEP_S = 0.2

# Only these /compute_ik codes mean "this orientation sample is kinematically infeasible" --
# every other non-SUCCESS code is a service/request-level failure (bad group/link name, frame
# transform failure, planner internal error, ...) and must surface as an 'error' target, not
# silently count toward 'unreachable' (docs/INTERFACES.md §7.2).
_IK_INFEASIBLE_CODES = frozenset({MoveItErrorCodes.NO_IK_SOLUTION, MoveItErrorCodes.TIMED_OUT})

_MOVEIT_ERROR_CODE_NAMES = {
    getattr(MoveItErrorCodes, name): name
    for name in dir(MoveItErrorCodes)
    if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)
}


def _ik_error_name(val: int) -> str:
    return _MOVEIT_ERROR_CODE_NAMES.get(val, str(val))

_SERVICE_SPECS = (
    ("ik", GetPositionIK, "/compute_ik"),
    ("fk", GetPositionFK, "/compute_fk"),
    ("get_scene", GetPlanningScene, "/get_planning_scene"),
    ("apply_scene", ApplyPlanningScene, "/apply_planning_scene"),
)


# --------------------------------------------------------------------------------------
# small helpers
# --------------------------------------------------------------------------------------

def _resolve(root: Path, value: Optional[Path], default: str) -> Path:
    if value is not None:
        return Path(value).expanduser().resolve()
    return (root / default).resolve()


def _relpath(path: Path, root: Path) -> str:
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path)


def _git_commit(root: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=str(root), capture_output=True, text=True, timeout=5.0
        )
        if out.returncode == 0:
            return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return "unknown"


def _matrix_from_quat(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# --------------------------------------------------------------------------------------
# rclpy plumbing: services + /robot_description(_semantic) topics
# --------------------------------------------------------------------------------------

def _wait_for_services(node: Node, timeout_s: float, log) -> Dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    clients: Dict[str, Any] = {}
    for key, srv_type, name in _SERVICE_SPECS:
        client = node.create_client(srv_type, name)
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=max(remaining, 0.0)):
            raise PreconditionError(
                f"MoveIt service '{name}' did not become available within --service-timeout-s={timeout_s}"
            )
        clients[key] = client
        log.info("service '%s' available", name)
    return clients


def _wait_for_topic_message(node: Node, topic: str, timeout_s: float, required: bool, log) -> Optional[str]:
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    holder: Dict[str, str] = {}

    def _cb(msg: String) -> None:
        holder["data"] = msg.data

    sub = node.create_subscription(String, topic, _cb, qos)
    deadline = time.monotonic() + timeout_s
    try:
        while "data" not in holder:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if required:
                    raise PreconditionError(f"no message received on '{topic}' within {timeout_s}s")
                return None
            rclpy.spin_once(node, timeout_sec=min(_TOPIC_SPIN_STEP_S, remaining))
        log.info("received message on '%s' (%d bytes)", topic, len(holder["data"]))
        return holder["data"]
    finally:
        node.destroy_subscription(sub)


def _assert_link_in_urdf(urdf_xml: str, link_name: str) -> None:
    root = ET.fromstring(urdf_xml)
    names = {link.get("name") for link in root.findall("link")}
    if link_name not in names:
        raise PreconditionError(f"ik_link '{link_name}' not found in the URDF served on /robot_description")


def _load_home_state(node: Node, timeout_s: float, group: str, log) -> Dict[str, float]:
    srdf_xml = _wait_for_topic_message(node, "/robot_description_semantic", min(_SRDF_TIMEOUT_S, timeout_s), False, log)
    if srdf_xml:
        try:
            root_el = ET.fromstring(srdf_xml)
            for gs in root_el.findall("group_state"):
                if gs.get("name") == "home" and gs.get("group") == group:
                    values = {j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
                    if all(name in values for name in _JOINT_NAMES):
                        log.info("using SRDF 'home' group_state for group '%s' as the IK seed fallback", group)
                        return {name: values[name] for name in _JOINT_NAMES}
        except ET.ParseError as exc:
            log.warning("could not parse /robot_description_semantic: %s", exc)
    log.warning("no SRDF 'home' group_state for group '%s'; falling back to an all-zero seed", group)
    return {name: 0.0 for name in _JOINT_NAMES}


def _call_service(node: Node, client, request, timeout_s: float):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        future.cancel()
        return None
    try:
        return future.result()
    except Exception:  # noqa: BLE001 - a service exception here maps to this target's "error" status
        return None


# --------------------------------------------------------------------------------------
# IK / FK requests
# --------------------------------------------------------------------------------------

def _build_ik_request(cfg: dict, position, quat_xyzw, seed: Dict[str, float]) -> GetPositionIK.Request:
    req = GetPositionIK.Request()
    req.ik_request.group_name = cfg["group"]
    req.ik_request.ik_link_name = cfg["ik_link"]
    req.ik_request.avoid_collisions = cfg["ik"]["avoid_collisions"]
    ps = PoseStamped()
    ps.header.frame_id = cfg["frame"]
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in position)
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = (
        float(v) for v in quat_xyzw
    )
    req.ik_request.pose_stamped = ps
    timeout_s = cfg["ik"]["timeout_s"]
    sec = int(timeout_s)
    req.ik_request.timeout.sec = sec
    req.ik_request.timeout.nanosec = int(round((timeout_s - sec) * 1e9))
    rs = RobotState()
    rs.joint_state.name = list(seed.keys())
    rs.joint_state.position = [float(v) for v in seed.values()]
    req.ik_request.robot_state = rs
    return req


def _build_fk_request(cfg: dict, solution: RobotState) -> GetPositionFK.Request:
    req = GetPositionFK.Request()
    req.header.frame_id = cfg["frame"]
    req.fk_link_names = [cfg["ik_link"]]
    req.robot_state = solution
    return req


# --------------------------------------------------------------------------------------
# surface collision slab (per surface_z layer)
# --------------------------------------------------------------------------------------

def _surface_object(cfg: dict, surface_z: float, operation: int) -> CollisionObject:
    obj = CollisionObject()
    obj.header.frame_id = cfg["frame"]
    obj.id = _SURFACE_OBJECT_ID
    obj.operation = operation
    if operation == CollisionObject.ADD:
        sc = cfg["surface_collision"]
        x_axis, y_axis = cfg["grid"]["x_m"], cfg["grid"]["y_m"]
        x_extent = (x_axis["max"] - x_axis["min"]) + 2.0 * sc["margin_m"]
        y_extent = (y_axis["max"] - y_axis["min"]) + 2.0 * sc["margin_m"]
        thickness = sc["thickness_m"]
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [x_extent, y_extent, thickness]
        pose = Pose()
        pose.position.x = (x_axis["min"] + x_axis["max"]) / 2.0
        pose.position.y = (y_axis["min"] + y_axis["max"]) / 2.0
        pose.position.z = surface_z - 0.001 - thickness / 2.0
        pose.orientation.w = 1.0
        obj.primitives = [prim]
        obj.primitive_poses = [pose]
    return obj


def _get_acm(node: Node, clients: Dict[str, Any]) -> AllowedCollisionMatrix:
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    resp = _call_service(node, clients["get_scene"], req, _SCENE_CALL_TIMEOUT_S)
    if resp is None:
        raise PreconditionError("get_planning_scene call failed while reading the ACM for the surface-collision layer")
    return resp.scene.allowed_collision_matrix


def _extend_acm(acm: AllowedCollisionMatrix, allowed_links) -> AllowedCollisionMatrix:
    """Symmetric NxN -> (N+1)x(N+1) matrix, adding `mot04_surface` allowed only vs `allowed_links`."""
    allowed_set = set(allowed_links)
    old_names = list(acm.entry_names)
    old_rows = [list(v.enabled) for v in acm.entry_values]

    new_rows = [row + [name in allowed_set] for name, row in zip(old_names, old_rows)]
    new_rows.append([name in allowed_set for name in old_names] + [False])

    new_acm = AllowedCollisionMatrix()
    new_acm.entry_names = old_names + [_SURFACE_OBJECT_ID]
    new_acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in new_rows]
    new_acm.default_entry_names = list(acm.default_entry_names)
    new_acm.default_entry_values = list(acm.default_entry_values)
    return new_acm


def _apply_scene_diff(node: Node, clients: Dict[str, Any], collision_objects, acm: AllowedCollisionMatrix) -> None:
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = collision_objects
    scene.allowed_collision_matrix = acm
    req = ApplyPlanningScene.Request()
    req.scene = scene
    resp = _call_service(node, clients["apply_scene"], req, _SCENE_CALL_TIMEOUT_S)
    if resp is None or not resp.success:
        raise PreconditionError("apply_planning_scene call failed while updating the surface-collision layer")


@contextlib.contextmanager
def _surface_collision_layer(node: Node, clients: Dict[str, Any], cfg: dict, surface_z: float, log):
    sc = cfg["surface_collision"]
    if not sc["enabled"]:
        yield
        return
    orig_acm = _get_acm(node, clients)
    new_acm = _extend_acm(orig_acm, sc["allowed_links"])
    _apply_scene_diff(node, clients, [_surface_object(cfg, surface_z, CollisionObject.ADD)], new_acm)
    log.info("added surface-collision slab '%s' for surface_z=%.4f", _SURFACE_OBJECT_ID, surface_z)
    try:
        yield
    finally:
        _apply_scene_diff(node, clients, [_surface_object(cfg, surface_z, CollisionObject.REMOVE)], orig_acm)
        log.info("removed surface-collision slab '%s'", _SURFACE_OBJECT_ID)


# --------------------------------------------------------------------------------------
# per-target IK + FK re-check
# --------------------------------------------------------------------------------------

def _process_target(
    node: Node,
    clients: Dict[str, Any],
    cfg: dict,
    target,
    limits: Dict[str, tuple],
    reach_bound_m: float,
    seed: Dict[str, float],
) -> Dict[str, Any]:
    down_world = np.array(cfg["boresight"]["down_world"], dtype=float)
    nominal_position = target_position((target.x, target.y, target.surface_z), target.standoff_m, down_world)

    if cfg["prefilter"]["enabled"] and prefiltered(nominal_position, reach_bound_m):
        return {"status": "prefiltered", "position": nominal_position}

    roll_search = cfg["ik"]["roll_search"]
    ik_calls = 0
    ik_time_s = 0.0
    best: Optional[Dict[str, Any]] = None

    for o in orientations(cfg):
        pos = target_position((target.x, target.y, target.surface_z), target.standoff_m, o.d_world)
        req = _build_ik_request(cfg, pos, o.quat_xyzw, seed)
        t0 = time.monotonic()
        resp = _call_service(node, clients["ik"], req, cfg["ik"]["timeout_s"] + _IK_CALL_TIMEOUT_MARGIN_S)
        ik_calls += 1
        ik_time_s += time.monotonic() - t0

        if resp is None:
            return {
                "status": "error",
                "position": nominal_position,
                "ik_calls": ik_calls,
                "ik_time_s": ik_time_s,
                "error": f"compute_ik service call failed/timed out for target {target.target_id}",
            }
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            if resp.error_code.val in _IK_INFEASIBLE_CODES:
                continue
            return {
                "status": "error",
                "position": nominal_position,
                "ik_calls": ik_calls,
                "ik_time_s": ik_time_s,
                "error": f"compute_ik returned {_ik_error_name(resp.error_code.val)} for target {target.target_id}",
            }

        joints = {
            name: value
            for name, value in zip(resp.solution.joint_state.name, resp.solution.joint_state.position)
            if name in _JOINT_NAMES
        }
        if len(joints) != len(_JOINT_NAMES):
            continue
        margin = joint_limit_margin(joints, limits)
        if margin < 0:
            continue

        candidate = {"orientation": o, "joints": joints, "margin": margin, "position": pos, "solution": resp.solution}
        if roll_search == "first":
            best = candidate
            break
        if best is None or margin > best["margin"]:
            best = candidate

    if best is None:
        return {"status": "unreachable", "position": nominal_position, "ik_calls": ik_calls, "ik_time_s": ik_time_s}

    fk_resp = _call_service(node, clients["fk"], _build_fk_request(cfg, best["solution"]), _FK_CALL_TIMEOUT_S)
    if fk_resp is None or fk_resp.error_code.val != MoveItErrorCodes.SUCCESS:
        return {
            "status": "error",
            "position": best["position"],
            "ik_calls": ik_calls,
            "ik_time_s": ik_time_s,
            "error": f"compute_fk service call failed for target {target.target_id}",
        }

    fk_pose = fk_resp.pose_stamped[0].pose
    fk_position = np.array([fk_pose.position.x, fk_pose.position.y, fk_pose.position.z])
    fk_position_error_m = float(np.linalg.norm(fk_position - best["position"]))
    r_fk = _matrix_from_quat(fk_pose.orientation.x, fk_pose.orientation.y, fk_pose.orientation.z, fk_pose.orientation.w)
    axis_local = np.array(cfg["boresight"]["axis_local"], dtype=float)
    axis_world_fk = r_fk @ axis_local
    cos_angle = float(np.clip(np.dot(axis_world_fk, best["orientation"].d_world), -1.0, 1.0))
    fk_axis_error_deg = math.degrees(math.acos(cos_angle))

    if fk_position_error_m > _FK_POSITION_TOL_M or fk_axis_error_deg > _FK_AXIS_TOL_DEG:
        return {
            "status": "fk_mismatch",
            "position": best["position"],
            "ik_calls": ik_calls,
            "ik_time_s": ik_time_s,
            "error": (
                f"fk mismatch: position_error_m={fk_position_error_m:.6f} (tol {_FK_POSITION_TOL_M}), "
                f"axis_error_deg={fk_axis_error_deg:.3f} (tol {_FK_AXIS_TOL_DEG})"
            ),
        }

    return {
        "status": "reachable",
        "position": best["position"],
        "ik_calls": ik_calls,
        "ik_time_s": ik_time_s,
        "tilt_deg": best["orientation"].tilt_deg,
        "azimuth_rad": best["orientation"].azimuth_rad,
        "roll_rad": best["orientation"].roll_rad,
        "quat_xyzw": best["orientation"].quat_xyzw,
        "joints": best["joints"],
        "min_joint_limit_margin_rad": best["margin"],
        "fk_position_error_m": fk_position_error_m,
        "fk_axis_error_deg": fk_axis_error_deg,
    }


# --------------------------------------------------------------------------------------
# the sweep itself
# --------------------------------------------------------------------------------------

def _sweep(
    node: Node,
    clients: Dict[str, Any],
    cfg: dict,
    limits: Dict[str, tuple],
    reach_bound_m: float,
    home_positions: Dict[str, float],
    map_dict: Dict[str, Any],
    max_duration_s: float,
    require_all_reachable: bool,
    log,
    summary: RunSummary,
) -> Dict[str, str]:
    start = time.monotonic()
    complete = True
    seed_chain: Optional[Dict[str, float]] = None
    seed_mode = cfg["ik"]["seed"]

    targets = enumerate_targets(cfg)
    total = len(targets)
    done = 0
    next_pct = 5

    for _iz, group_iter in itertools.groupby(targets, key=lambda t: t.iz):
        group = list(group_iter)
        surface_z = group[0].surface_z
        with _surface_collision_layer(node, clients, cfg, surface_z, log):
            for target in group:
                if max_duration_s > 0 and (time.monotonic() - start) >= max_duration_s:
                    complete = False
                    log.warning(
                        "--max-duration-s=%.1f reached; stopping before target %s", max_duration_s, target.target_id
                    )
                    break

                seed = seed_chain if (seed_mode == "neighbour" and seed_chain is not None) else home_positions
                entry = _process_target(node, clients, cfg, target, limits, reach_bound_m, seed)
                add_target(map_dict, target, **entry)

                if entry["status"] == "reachable" and seed_mode == "neighbour":
                    seed_chain = entry["joints"]

                done += 1
                pct = done * 100 // total if total else 100
                if pct >= next_pct:
                    elapsed = time.monotonic() - start
                    eta = elapsed / done * (total - done) if done else 0.0
                    log.info(
                        "progress: %d/%d (%d%%) elapsed=%.1fs eta=%.1fs counts=%s",
                        done, total, pct, elapsed, eta, _running_counts(map_dict),
                    )
                    next_pct += 5
            else:
                continue
            break

    duration_s = time.monotonic() - start
    finalize(map_dict, complete, duration_s)
    counts = map_dict["summary"]["counts_by_status"]
    summary.counts.update(counts)
    summary.counts["targets_total"] = total
    summary.counts["targets_processed"] = done

    if not complete:
        return {"status": "partial"}
    if counts.get("fk_mismatch", 0) or counts.get("error", 0):
        return {"status": "failed"}
    if require_all_reachable and any(counts.get(s, 0) for s in ("unreachable", "prefiltered", "fk_mismatch", "error")):
        return {"status": "failed"}
    return {"status": "ok"}


def _running_counts(map_dict: Dict[str, Any]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for t in map_dict["targets"]:
        counts[t["status"]] = counts.get(t["status"], 0) + 1
    return counts


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def _extra_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--reachability-config", type=Path, default=None, help="reachability config YAML (default: config/motion/reachability.yaml)")
    parser.add_argument("--limits", type=Path, default=None, help="joint limits YAML (default: config/robot/b601_dm_limits.yaml)")
    parser.add_argument("--out", type=Path, default=None, help="reachability map JSON to write (default: data/motion/reachability_map.json)")
    parser.add_argument("--service-timeout-s", type=float, default=DEFAULT_SERVICE_TIMEOUT_S, help="seconds to wait for MoveIt services/topics before exit 3")
    parser.add_argument("--max-duration-s", type=float, default=DEFAULT_MAX_DURATION_S, help="wall-clock sweep budget; 0 = unlimited")
    parser.add_argument("--require-all-reachable", action="store_true", help="exit 1 if any target is not 'reachable'")


def _dry_run(args: argparse.Namespace) -> int:
    from .cli_common import EXIT_OK, EXIT_USAGE

    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    log_dir = root / "logs"
    logger = setup_logging(TOOL, log_dir, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()
    status, exit_code = "ok", EXIT_OK
    try:
        config_path = _resolve(root, args.reachability_config, DEFAULT_CONFIG)
        cfg = load_config(config_path)
        targets = enumerate_targets(cfg)
        orients = orientations(cfg)
        n_targets, n_orientations = len(targets), len(orients)
        ik_call_upper_bound = n_targets * n_orientations
        logger.info(
            "--dry-run: config=%s targets=%d orientation_samples=%d ik_call_upper_bound=%d",
            config_path, n_targets, n_orientations, ik_call_upper_bound,
        )
        summary.counts.update({
            "targets": n_targets,
            "orientation_samples": n_orientations,
            "ik_call_upper_bound": ik_call_upper_bound,
        })
    except ConfigError as exc:
        logger.error("%s", exc)
        summary.add_error(str(exc))
        status, exit_code = "failed", EXIT_USAGE
    summary.write(log_dir, TOOL, status, exit_code)
    return exit_code


def _run(args: argparse.Namespace, log, summary: RunSummary) -> Dict[str, str]:
    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    config_path = _resolve(root, args.reachability_config, DEFAULT_CONFIG)
    limits_path = _resolve(root, args.limits, DEFAULT_LIMITS)
    out_path = _resolve(root, args.out, DEFAULT_OUT)

    cfg = load_config(config_path)
    limits = limits_from_yaml(limits_path)

    if args.skip_existing and out_path.exists():
        log.info("--skip-existing: %s already exists, skipping", out_path)
        summary.increment("skipped")
        return {"status": "ok"}

    cfg_sha = config_sha256(config_path)
    limits_sha = hashlib.sha256(limits_path.read_bytes()).hexdigest()
    git_commit = _git_commit(root)

    rclpy.init(args=None)
    node = Node(TOOL)
    map_dict: Optional[Dict[str, Any]] = None
    try:
        clients = _wait_for_services(node, args.service_timeout_s, log)
        urdf_xml = _wait_for_topic_message(node, "/robot_description", args.service_timeout_s, True, log)
        _assert_link_in_urdf(urdf_xml, cfg["ik_link"])
        reach_bound_m = reach_bound_from_urdf(urdf_xml, cfg["frame"], cfg["ik_link"])
        log.info("reach_bound_m (prefilter bound from /robot_description) = %.6f", reach_bound_m)
        home_positions = _load_home_state(node, args.service_timeout_s, cfg["group"], log)

        robot_info = {
            "group": cfg["group"],
            "ik_link": cfg["ik_link"],
            "ik_solver": "trac_ik",
            "reach_bound_m": reach_bound_m,
            "limits_file": _relpath(limits_path, root),
            "limits_sha256": limits_sha,
        }
        map_dict = new_map(cfg, _relpath(config_path, root), cfg_sha, robot_info, git_commit)

        return _sweep(
            node, clients, cfg, limits, reach_bound_m, home_positions, map_dict,
            args.max_duration_s, args.require_all_reachable, log, summary,
        )
    finally:
        if map_dict is not None:
            try:
                write_map(map_dict, out_path)
                log.info("wrote reachability map to %s (complete=%s)", out_path, map_dict.get("complete"))
            except OSError:
                log.exception("failed to write reachability map to %s", out_path)
        node.destroy_node()
        rclpy.shutdown()


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

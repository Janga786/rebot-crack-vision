"""plan_view -- camera view-pose planning node/CLI (MOT-06.9).

console_script: `ros2 run crackvision_motion plan_view`. Runs under the system python3 (ROS
Humble, py3.10). See docs/INTERFACES.md §13.14 (schema `crackvision.view_plan/1`) for the full
contract: specimen-frame resolution, the `(tilt_deg, yaw_deg)` candidate set, the FOV check, the
reachability step (`/compute_ik` + `/plan_kinematic_path` from `home`) and the independent
re-check (§13.8), selection (first candidate that passes all three) and the plan file
(§13.14.1), §13.15 CLI/exit codes.

Only ever calls `/compute_ik`, `/compute_fk`, `/plan_kinematic_path`, `/check_state_validity`,
`/get_planning_scene` and `/apply_planning_scene` (the last two via `scene_apply.apply_scene`).
There is no action client anywhere in this file, and `/compute_cartesian_path` is never called --
the view phase positions a camera, it never traces anything.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node

from .cartesian_core import CartesianConfigError, config_sha256, load_config
from .cartesian_moveit import (
    MoveItClients,
    build_fk_request,
    build_ik_request,
    build_motion_plan_request,
    build_validity_request,
    ik_call_timeout_s,
    parse_fk,
    parse_ik,
    parse_motion_plan,
)
from .cartesian_plan import add_view_candidate, build_leg, finalize_view, new_view_plan, write_view_plan
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
from .view_core import Pose, ViewCoreError, fov_check, specimen_frame, view_candidates

TOOL = "plan_view"
LINK = "camera_link"

DEFAULT_PLANNING_CONFIG = "config/motion/cartesian_planning.yaml"
DEFAULT_SCENE_CONFIG = "config/scene/scene.yaml"
DEFAULT_END_EFFECTOR_CONFIG = "config/robot/end_effector.yaml"
DEFAULT_LIMITS = "config/robot/b601_dm_limits.yaml"
DEFAULT_OUT = "data/plans/view_plan.json"

_SPECIMEN_OBJECT_ID = "specimen"
_N_JOINTS = len(JOINT_NAMES)

_SCENE_SERVICE_SPECS = (
    ("get_scene", GetPlanningScene, "/get_planning_scene"),
    ("apply_scene", ApplyPlanningScene, "/apply_planning_scene"),
)


# --------------------------------------------------------------------------------------
# small pure helpers (joints, FK errors, limits, joint step) -- mirror plan_crack_path.py's own
# --------------------------------------------------------------------------------------

def _joint_dict(values: Sequence[float]) -> Dict[str, float]:
    return dict(zip(JOINT_NAMES, (float(v) for v in values)))


def _quat_angle_deg(q1: Sequence[float], q2: Sequence[float]) -> float:
    dot = max(-1.0, min(1.0, abs(sum(a * b for a, b in zip(q1, q2)))))
    return math.degrees(2.0 * math.acos(dot))


def _pose_errors(pose: Pose, target: Pose) -> Tuple[float, float]:
    pos_err = math.sqrt(sum((a - b) ** 2 for a, b in zip(pose.position_m, target.position_m)))
    return pos_err, _quat_angle_deg(pose.quat_xyzw, target.quat_xyzw)


def _within_limits(configs: Sequence[Sequence[float]], limits: Dict[str, Dict[str, float]]) -> bool:
    for vals in configs:
        for name, v in zip(JOINT_NAMES, vals):
            spec = limits[name]
            if v < spec["lower"] or v > spec["upper"]:
                return False
    return True


def _max_joint_step(configs: Sequence[Sequence[float]]) -> float:
    if len(configs) < 2:
        return 0.0
    return max(
        max(abs(a - b) for a, b in zip(configs[i], configs[i + 1]))
        for i in range(len(configs) - 1)
    )


def _pose_dict(pose: Pose) -> Dict[str, List[float]]:
    return {"position_m": list(pose.position_m), "quat_xyzw": list(pose.quat_xyzw)}


def _fov_dict(fov_ok: bool, details: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "passed": fov_ok,
        "horizontal_margins_deg": list(details["horizontal_margins_deg"]),
        "vertical_margins_deg": list(details["vertical_margins_deg"]),
    }


def _scene_specimen_footprint_m(scene_cfg: Dict[str, Any]) -> Optional[List[float]]:
    for obj in scene_cfg.get("objects", []):
        if obj.get("id") == _SPECIMEN_OBJECT_ID:
            dims = obj["dimensions_m"]
            return [float(dims[0]), float(dims[1])]
    return None


def _resolve_specimen(
    args: argparse.Namespace, cfg: Dict[str, Any], scene_cfg: Dict[str, Any],
) -> Tuple[str, List[float], float, List[float], Any]:
    """§13.14 "Geometry": `--center`, when given, always takes precedence over the scene's
    `specimen` object. Returns `(center_source, center_xy_m, surface_z_m, footprint_m, frame)`."""
    if args.center is not None:
        cx, cy = float(args.center[0]), float(args.center[1])
        footprint_m = [float(v) for v in cfg["view"]["footprint_m"]]
        surface_z_m = float(cfg["view"]["surface_z_m"])
        frame = specimen_frame(center=(cx, cy), footprint_m=footprint_m, surface_z_m=surface_z_m)
        return "--center", [cx, cy], surface_z_m, footprint_m, frame

    frame = specimen_frame(scene_cfg=scene_cfg)
    footprint_m = _scene_specimen_footprint_m(scene_cfg)
    center_xy_m = [frame.centre_top_m[0], frame.centre_top_m[1]]
    surface_z_m = frame.centre_top_m[2]
    return "scene.specimen", center_xy_m, surface_z_m, footprint_m, frame


def _check_all_valid(moveit: MoveItClients, configs: Sequence[Sequence[float]]) -> bool:
    for vals in configs:
        resp = moveit.call(moveit.clients["validity"], build_validity_request(_joint_dict(vals)))
        if resp is None or not resp.valid:
            return False
    return True


@dataclass
class ViewAttempt:
    """The outcome of attempting one `(tilt_deg, yaw_deg)` candidate (§13.14): `transit_leg_spec`
    (kwargs ready for `cartesian_plan.build_leg`) is non-`None` only when this candidate was
    selected (fov passed, IK+transit succeeded, independent re-check passed); `reason` is the
    attempt's own failure note (never set when selected)."""

    tilt_deg: float
    yaw_deg: float
    pose: Pose
    fov_entry: Dict[str, Any]
    reachable: bool
    recheck_passed: bool
    transit_leg_spec: Optional[Dict[str, Any]]
    reason: Optional[str]


def _attempt_view_candidate(
    moveit: MoveItClients, cfg: Dict[str, Any], limits: Dict[str, Dict[str, float]],
    home_positions: Dict[str, float], cand: Any, corners_m: Sequence[Tuple[float, float, float]],
) -> ViewAttempt:
    """§13.14's per-candidate procedure, independent of any ROS graph being up (`moveit` is any
    `MoveItClients`-shaped object, see `test_plan_view.py`'s `_FakeMoveIt`): the FOV check, then
    (only if it passes) `/compute_ik` for `camera_link`, an independent `/compute_fk` agreement
    check on that solution, `/plan_kinematic_path` from `home_positions`, and the §13.8-style
    collision/limit/joint-step re-check over the whole transit. Never accepts the IK solution's
    own claim of success without the explicit FK re-check."""
    fov_ok, details = fov_check(cand.pose, corners_m, cfg)
    fov_entry = _fov_dict(fov_ok, details)
    if not fov_ok:
        return ViewAttempt(cand.tilt_deg, cand.yaw_deg, cand.pose, fov_entry, False, False, None, "fov_failed")

    ik_resp = moveit.call(
        moveit.clients["ik"], build_ik_request(cand.pose, home_positions, LINK, cfg),
        timeout_s=ik_call_timeout_s(cfg),
    )
    ik_joints, ik_err = parse_ik(ik_resp)
    if ik_err is not None:
        if ik_err.startswith("ik_error:"):
            raise RuntimeError(f"/compute_ik {ik_err}")
        return ViewAttempt(cand.tilt_deg, cand.yaw_deg, cand.pose, fov_entry, False, False, None, ik_err)

    fk_resp = moveit.call(moveit.clients["fk"], build_fk_request(ik_joints, [LINK]))
    fk_pose, fk_err = parse_fk(fk_resp, LINK)
    if fk_err is not None:
        raise RuntimeError(f"/compute_fk failed: {fk_err}")
    pos_err, ang_err = _pose_errors(fk_pose, cand.pose)
    fk_ok = pos_err <= cfg["recheck"]["fk_tolerance_m"] and ang_err <= cfg["recheck"]["fk_tolerance_deg"]

    mp_resp = moveit.call(moveit.clients["motion_plan"], build_motion_plan_request(home_positions, ik_joints, cfg))
    transit_configs, mp_err = parse_motion_plan(mp_resp)
    if mp_err is not None:
        return ViewAttempt(cand.tilt_deg, cand.yaw_deg, cand.pose, fov_entry, False, False, None, mp_err)

    all_valid = _check_all_valid(moveit, transit_configs)
    within_limits_ok = _within_limits(transit_configs, limits)
    step = _max_joint_step(transit_configs)
    step_ok = step <= cfg["recheck"]["max_joint_step_rad"]
    recheck_passed = fk_ok and all_valid and within_limits_ok and step_ok

    if recheck_passed:
        leg_spec = {
            "kind": "transit", "segment_ref": None, "direction": None, "roll_offset_rad": None,
            "positions": transit_configs, "waypoint_index_map": [None] * len(transit_configs),
            "recheck": {
                "max_pos_err_m": 0.0, "max_ang_err_rad": 0.0, "all_valid": all_valid,
                "within_limits": within_limits_ok, "max_joint_step_rad": step,
            },
        }
        return ViewAttempt(cand.tilt_deg, cand.yaw_deg, cand.pose, fov_entry, True, True, leg_spec, None)

    bad = []
    if not fk_ok:
        bad.append("fk_mismatch")
    if not all_valid:
        bad.append("collision")
    if not within_limits_ok:
        bad.append("joint_limit")
    if not step_ok:
        bad.append("joint_jump")
    return ViewAttempt(cand.tilt_deg, cand.yaw_deg, cand.pose, fov_entry, True, False, None, ",".join(bad))


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
    try:
        return load_config(path)
    except OSError as exc:
        raise CartesianConfigError(f"cannot read planning config '{path}': {exc}") from exc


def _load_scene_cfg(path: Path) -> Dict[str, Any]:
    try:
        return load_scene_config(path)
    except OSError as exc:
        raise SceneError(f"cannot read scene config '{path}': {exc}") from exc


def _extra_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--out", default=None, help=f"view plan file to write (default: {DEFAULT_OUT})")
    parser.add_argument(
        "--center", type=float, nargs=2, default=None, metavar=("X", "Y"),
        help="explicit specimen centre (m); overrides the scene's 'specimen' object even if present (§13.14)",
    )
    parser.add_argument("--scene-config", default=None, help=f"default: {DEFAULT_SCENE_CONFIG} under --root")
    parser.add_argument("--planning-config", default=None, help=f"default: {DEFAULT_PLANNING_CONFIG} under --root")


def _dry_run(args: argparse.Namespace) -> int:
    """§13.15: validate every input and print the candidate/FOV plan -- no ROS service call, no
    `rclpy.init`, no file write, exit 0 on success."""
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

        center_source, center_xy_m, surface_z_m, footprint_m, frame = _resolve_specimen(args, cfg, scene_cfg)
        cand_list = view_candidates(cfg, frame.centre_top_m, frame.normal_out)
        logger.info(
            "--dry-run: center_source=%s center_xy_m=%s surface_z_m=%.4f footprint_m=%s candidates=%d",
            center_source, center_xy_m, surface_z_m, footprint_m, len(cand_list),
        )
        for cand in cand_list:
            fov_ok, details = fov_check(cand.pose, frame.corners_m, cfg)
            logger.info(
                "  tilt=%.1f yaw=%.1f fov_passed=%s h_margins_deg=%s v_margins_deg=%s",
                cand.tilt_deg, cand.yaw_deg, fov_ok,
                [round(v, 2) for v in details["horizontal_margins_deg"]],
                [round(v, 2) for v in details["vertical_margins_deg"]],
            )
        summary.counts.update({"candidates": len(cand_list)})
    except (CartesianConfigError, SceneError) as exc:
        logger.error("%s", exc)
        summary.add_error(str(exc))
        status, exit_code = "failed", EXIT_USAGE
    except (ViewCoreError, PreconditionError) as exc:
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
        center_source, center_xy_m, surface_z_m, footprint_m, frame = _resolve_specimen(args, cfg, scene_cfg)
    except ViewCoreError as exc:
        raise PreconditionError(str(exc)) from exc

    out_path = Path(args.out).expanduser().resolve() if args.out else (root / DEFAULT_OUT)
    if args.skip_existing and out_path.exists():
        log.info("--skip-existing: %s already exists, skipping", out_path)
        summary.increment("skipped")
        return {"status": "ok"}

    cand_list = view_candidates(cfg, frame.centre_top_m, frame.normal_out)

    plan = new_view_plan(
        sources={
            "scene": {"path": _relpath(paths["scene_config"], root), "sha256": scene_sha},
            "end_effector": {"path": _relpath(paths["end_effector"], root), "sha256": ee_sha},
            "limits": {"path": _relpath(paths["limits"], root), "sha256": limits_sha},
            "config": {"path": _relpath(paths["planning_config"], root), "sha256": cfg_sha},
        },
        parameters=cfg["view"],
        center_source=center_source,
        center_xy_m=center_xy_m,
        surface_z_m=surface_z_m,
        footprint_m=footprint_m,
    )

    chosen: Optional[Dict[str, float]] = None
    transit_leg: Optional[Dict[str, Any]] = None
    reasons: List[str] = []

    rclpy.init(args=None)
    node = Node(TOOL)
    try:
        moveit = MoveItClients(node, cfg["service_timeout_s"])
        moveit.wait_for_services()
        scene_clients = _wait_for_scene_services(node, cfg["service_timeout_s"])
        apply_scene(node, scene_clients["get_scene"], scene_clients["apply_scene"], scene_cfg)
        log.info("applied scene from %s", paths["scene_config"])

        home_positions = _joint_dict([0.0] * _N_JOINTS)

        for cand in cand_list:
            attempt = _attempt_view_candidate(moveit, cfg, limits, home_positions, cand, frame.corners_m)
            add_view_candidate(
                plan, attempt.tilt_deg, attempt.yaw_deg, _pose_dict(attempt.pose), attempt.fov_entry,
                reachable=attempt.reachable, recheck_passed=attempt.recheck_passed,
            )
            if attempt.transit_leg_spec is not None:
                # Selection (§13.14): the first candidate that passes all three stages wins.
                transit_leg = build_leg(**attempt.transit_leg_spec)
                chosen = {"tilt_deg": attempt.tilt_deg, "yaw_deg": attempt.yaw_deg}
                break
            reasons.append(f"tilt={attempt.tilt_deg} yaw={attempt.yaw_deg}: {attempt.reason}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    finalize_view(plan, chosen, transit_leg, limits)

    if chosen is None:
        detail = "; ".join(reasons) if reasons else "no candidates configured"
        raise PreconditionError(f"no view candidate selected (§13.14 Selection): {detail}")

    write_view_plan(plan, out_path)
    log.info(
        "wrote %s: chosen tilt=%.1f yaw=%.1f, %d candidates tried",
        out_path, chosen["tilt_deg"], chosen["yaw_deg"], len(plan["candidates"]),
    )
    summary.counts.update({"candidates_tried": len(plan["candidates"]), "chosen": 1})
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

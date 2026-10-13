"""preview_trajectory -- motion-free RViz preview of a trajectory file (§18.4, MOT-08.6).

console_script: `ros2 run crackvision_motion preview_trajectory --trajectory PATH`. Reads a
`crackvision.joint_trajectory/1` file directly and publishes a `moveit_msgs/DisplayTrajectory` on
`/display_planned_path` plus a tool-path/crack `visualization_msgs/MarkerArray` on
`/crackvision/preview_markers`, both latched (TRANSIENT_LOCAL + RELIABLE, depth 1) so a RViz
instance started after this node still sees the last message. This is purely a convenience for
looking at a planned trajectory in 3D (§18.4) -- it is never read by `trajectory_preview`,
`approve_trajectory` or any execution gate.

Motion-free by construction, not by configuration: this module never builds an action or service
client of any kind, never subscribes to or publishes any vendor/controller command or feedback
topic, and the two message builders below (`build_display_trajectory`, `build_markers`) are pure
functions over plain dicts/arrays with no ROS dependency, independently unit-testable without a
ROS graph.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from moveit_msgs.msg import DisplayTrajectory, RobotState as RobotStateMsg, RobotTrajectory
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from .cartesian_core import Paths3dError, load_paths3d
from .cli_common import EXIT_OK, EXIT_PRECONDITION, EXIT_USAGE, add_common_args, find_root
from .execution_gate import GateConfigError, load_execution_config
from .joint_trajectory import TrajectoryError, load_trajectory
from .tool_fk import ToolFKError, load_tool_chain

TOOL = "preview_trajectory"

DEFAULT_EXECUTION_CONFIG = "config/motion/execution.yaml"
DEFAULT_END_EFFECTOR_CONFIG = "config/robot/end_effector.yaml"
DEFAULT_URDF = "presentation/sim/reBot_B601_DM_with_gripper.urdf"

MODEL_ID = "rebotarm"
FRAME = "base_link"
DEFAULT_WAIT_S = 10.0

TOOL_PATH_NS = "tool_path"
CRACK_PATH_NS = "crack_path"


# --------------------------------------------------------------------------------------
# pure message builders (§18.4): no rclpy.node.Node, no ROS graph, unit-tested directly
# --------------------------------------------------------------------------------------

def build_display_trajectory(traj: Dict[str, Any], speed_scale: float) -> DisplayTrajectory:
    """§18.4: a `DisplayTrajectory` with `model_id='rebotarm'`, §11.2's joint order, and every
    point's `time_from_start` scaled by `t_s / speed_scale`. `trajectory_start` is the trajectory's
    own first point."""
    if isinstance(speed_scale, bool) or not isinstance(speed_scale, (int, float)) or float(speed_scale) <= 0.0:
        raise ValueError(f"speed_scale must be a positive number, got {speed_scale!r}")
    speed_scale = float(speed_scale)

    joint_names = list(traj["joint_names"])
    points: List[JointTrajectoryPoint] = []
    for p in traj["points"]:
        jtp = JointTrajectoryPoint()
        jtp.positions = [float(x) for x in p["positions"]]
        if p.get("velocities") is not None:
            jtp.velocities = [float(x) for x in p["velocities"]]
        if p.get("accelerations") is not None:
            jtp.accelerations = [float(x) for x in p["accelerations"]]
        scaled_t = float(p["t_s"]) / speed_scale
        total_ns = round(scaled_t * 1e9)
        sec, nanosec = divmod(total_ns, 1_000_000_000)
        jtp.time_from_start = Duration(sec=int(sec), nanosec=int(nanosec))
        points.append(jtp)

    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = joint_names
    joint_trajectory.points = points

    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory = joint_trajectory

    start_joint_state = JointState()
    start_joint_state.name = joint_names
    start_joint_state.position = [float(x) for x in traj["points"][0]["positions"]]
    start_state = RobotStateMsg()
    start_state.joint_state = start_joint_state

    display = DisplayTrajectory()
    display.model_id = MODEL_ID
    display.trajectory = [robot_trajectory]
    display.trajectory_start = start_state
    return display


def _line_strip_marker(ns: str, marker_id: int, frame: str, rgba: Sequence[float], width_m: float, pts) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = float(width_m)
    marker.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
    marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
    return marker


def build_markers(
    traj: Dict[str, Any],
    tool_xyz,
    paths3d_doc: Optional[Dict[str, Any]] = None,
    frame: str = FRAME,
) -> MarkerArray:
    """§18.4: a `LINE_STRIP` of the tool-tip path (namespace `tool_path`, id 0) plus one
    `LINE_STRIP` per crack polyline (namespace `crack_path`, ids 1..N) when `paths3d_doc` is
    given. `tool_xyz` is an (N, 3) array-like of tool-tip positions, one per `traj['points']`
    entry, in `frame`."""
    n_points = len(traj["points"])
    if len(tool_xyz) != n_points:
        raise ValueError(
            f"tool_xyz must have one row per trajectory point ({n_points}), got {len(tool_xyz)}"
        )

    markers = MarkerArray()
    markers.markers.append(
        _line_strip_marker(TOOL_PATH_NS, 0, frame, (0.0, 0.6, 1.0, 1.0), 0.002, tool_xyz)
    )

    if paths3d_doc is not None:
        marker_id = 1
        for component in paths3d_doc["components"]:
            for polyline in component["polylines"]:
                pts = [
                    pt["p_base_m"]
                    for pt in polyline["points"]
                    if pt.get("valid") and pt.get("p_base_m") is not None
                ]
                if len(pts) < 2:
                    continue
                markers.markers.append(
                    _line_strip_marker(CRACK_PATH_NS, marker_id, frame, (1.0, 0.0, 0.0, 1.0), 0.001, pts)
                )
                marker_id += 1

    return markers


# --------------------------------------------------------------------------------------
# CLI wiring
# --------------------------------------------------------------------------------------

def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog=TOOL)
    add_common_args(parser)
    parser.add_argument("--trajectory", required=True, help="crackvision.joint_trajectory/1 file to preview")
    parser.add_argument(
        "--paths3d", default=None,
        help="crackvision.paths3d/1 file for the crack overlay; sha-verified against the "
             "trajectory's source.paths3d and ignored with a warning on mismatch",
    )
    parser.add_argument("--speed-scale", type=float, default=None, help="default: execution.yaml speed_scale.real_default")
    parser.add_argument("--execution-config", default=None, help=f"default: {DEFAULT_EXECUTION_CONFIG} under --root")
    parser.add_argument("--end-effector-config", default=None, help=f"default: {DEFAULT_END_EFFECTOR_CONFIG} under --root")
    parser.add_argument("--urdf", default=None, help=f"default: {DEFAULT_URDF} under --root")
    parser.add_argument(
        "--once", action="store_true",
        help="publish once, wait up to --wait-s for a listener, then exit 0",
    )
    parser.add_argument("--wait-s", type=float, default=DEFAULT_WAIT_S, help=f"default: {DEFAULT_WAIT_S}")
    return parser


def _resolve_under_root(root: Path, value: Optional[str], default: str) -> Path:
    if value is not None:
        return Path(value).expanduser().resolve()
    return (root / default).resolve()


def _resolve_paths3d(traj: Dict[str, Any], paths3d_arg: Optional[str]) -> Optional[Dict[str, Any]]:
    """Load `--paths3d`, sha-verified against the trajectory's own `source.paths3d.sha256`
    (§18.4). A load failure or a sha mismatch is a warning, never a refusal: the preview still
    renders, just without the crack overlay."""
    if paths3d_arg is None:
        return None
    try:
        doc, sha256 = load_paths3d(paths3d_arg)
    except Paths3dError as exc:
        print(f"{TOOL}: warning: --paths3d could not be loaded, ignoring crack overlay: {exc}", file=sys.stderr)
        return None

    source_paths3d = (traj.get("source") or {}).get("paths3d")
    expected_sha = source_paths3d["sha256"] if source_paths3d else None
    if expected_sha is None or expected_sha != sha256:
        print(
            f"{TOOL}: warning: --paths3d sha256 {sha256} does not match the trajectory's "
            f"source.paths3d.sha256 ({expected_sha!r}); ignoring crack overlay",
            file=sys.stderr,
        )
        return None
    return doc


def _latched_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    return qos


def main(argv=None) -> int:
    argv = list(argv) if argv is not None else sys.argv[1:]
    args = _build_arg_parser().parse_args(argv)

    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    execution_config_path = _resolve_under_root(root, args.execution_config, DEFAULT_EXECUTION_CONFIG)
    end_effector_path = _resolve_under_root(root, args.end_effector_config, DEFAULT_END_EFFECTOR_CONFIG)
    urdf_path = _resolve_under_root(root, args.urdf, DEFAULT_URDF)

    try:
        traj = load_trajectory(args.trajectory)
    except (OSError, TrajectoryError) as exc:
        print(f"{TOOL}: invalid trajectory '{args.trajectory}': {exc}", file=sys.stderr)
        return EXIT_PRECONDITION

    speed_scale = args.speed_scale
    if speed_scale is None:
        try:
            exec_cfg = load_execution_config(execution_config_path)
        except (OSError, GateConfigError) as exc:
            print(f"{TOOL}: could not read execution config '{execution_config_path}': {exc}", file=sys.stderr)
            return EXIT_USAGE
        speed_scale = exec_cfg["speed_scale"]["real_default"]

    try:
        chain = load_tool_chain(urdf_path, end_effector_path)
        tool_xyz = chain.tool_positions([p["positions"] for p in traj["points"]])
    except ToolFKError as exc:
        print(f"{TOOL}: tool forward-kinematics failed: {exc}", file=sys.stderr)
        return EXIT_USAGE

    paths3d_doc = _resolve_paths3d(traj, args.paths3d)

    try:
        display = build_display_trajectory(traj, speed_scale)
    except ValueError as exc:
        print(f"{TOOL}: {exc}", file=sys.stderr)
        return EXIT_USAGE
    markers = build_markers(traj, tool_xyz, paths3d_doc)

    rclpy.init(args=None)
    node = Node(TOOL)
    try:
        qos = _latched_qos()
        display_pub = node.create_publisher(DisplayTrajectory, "/display_planned_path", qos)
        markers_pub = node.create_publisher(MarkerArray, "/crackvision/preview_markers", qos)

        display_pub.publish(display)
        markers_pub.publish(markers)

        if args.once:
            deadline = time.monotonic() + max(args.wait_s, 0.0)
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                if display_pub.get_subscription_count() > 0 or markers_pub.get_subscription_count() > 0:
                    break
            return EXIT_OK

        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=1.0)
                display_pub.publish(display)
                markers_pub.publish(markers)
        except KeyboardInterrupt:
            pass
        return EXIT_OK
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

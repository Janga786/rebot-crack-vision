"""check_end_effector -- verify the live MoveIt model against config/robot/end_effector.yaml (ADR-014).

console_script: `ros2 run crackvision_motion check_end_effector` (run via scripts/ros/env_ros.sh against a
running mock_planning.launch.py; scripts/ros/test_end_effector.sh does launch -> check -> teardown).

Read-only: it only calls /compute_fk, /compute_ik and /check_state_validity and reads
/robot_description. Nothing is planned or executed. Checks, each counted in the §0.4 summary:
  1. the overlay links (tool frame, camera_link, collision proxies) are in /robot_description and their
     fixed-joint origins equal the config values;
  2. FK at the all-zero joint state reproduces T_gripper_link_<frame> for tool_tip and camera_link;
  3. the all-zero (SRDF `home`) state is collision-free with the camera/mount proxies present, and the
     camera sits above the gripper there (the documented mount-side assumption, ADR-014 §1);
  4. /compute_ik accepts the tool frame and camera_link as ik_link_name (MoveIt resolves frames rigidly
     attached to the chain tip) for a boresight-down tool pose and a look-down camera view pose, and FK of
     the returned solution agrees within 1 mm / 0.5 deg.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rclpy
from crackvision_description.end_effector import EndEffectorError, load_config, rpy_to_matrix, transform
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from .cli_common import PreconditionError, RunSummary, find_root, run_cli
from .reachability_core import ConfigError

TOOL = "check_end_effector"
DEFAULT_CONFIG = "config/robot/end_effector.yaml"
ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
ZERO_STATE = {**{j: 0.0 for j in ARM_JOINTS}, "gripper_joint1": 0.0, "gripper_joint2": 0.0}
POS_TOL_M = 1e-3
AXIS_TOL_DEG = 0.5
ORIGIN_TOL = 1e-9
FK_TOL_M = 1e-6
FK_TOL_RAD = 1e-5
ROLL_SAMPLES = 8
# Probe targets, base_link. Tool: tip 1 cm above a surface at z=0.04 in the comfortable region found by
# TECHNICAL_APPROACH §2.5. View: camera 0.25 m above the same surface patch (D405 working band).
TOOL_PROBE = (0.25, 0.0, 0.05)
VIEW_PROBE = (0.30, 0.0, 0.29)


def _quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _matrix_to_quat(m: np.ndarray):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return tuple(q)


def _axis_down(axis_local: np.ndarray, roll: float) -> np.ndarray:
    """Rotation taking `axis_local` onto world -Z, then rolled by `roll` about world Z."""
    a = axis_local / np.linalg.norm(axis_local)
    d = np.array([0.0, 0.0, -1.0])
    v = np.cross(a, d)
    c = float(np.dot(a, d))
    if np.linalg.norm(v) < 1e-12:
        r0 = np.eye(3) if c > 0 else rpy_to_matrix([math.pi, 0.0, 0.0])
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        r0 = np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))
    return rpy_to_matrix([0.0, 0.0, roll]) @ r0


class _Live:
    def __init__(self, node: Node, timeout_s: float):
        self.node = node
        self.clients = {}
        deadline = time.monotonic() + timeout_s
        for key, srv, name in (("fk", GetPositionFK, "/compute_fk"), ("ik", GetPositionIK, "/compute_ik"),
                               ("valid", GetStateValidity, "/check_state_validity")):
            client = node.create_client(srv, name)
            if not client.wait_for_service(timeout_sec=max(deadline - time.monotonic(), 0.0)):
                raise PreconditionError(f"MoveIt service '{name}' not available within {timeout_s}s")
            self.clients[key] = client

    def call(self, key: str, req, timeout_s: float = 10.0):
        fut = self.clients[key].call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout_s)
        if not fut.done() or fut.result() is None:
            raise RuntimeError(f"{key} service call failed or timed out")
        return fut.result()

    def robot_description(self, timeout_s: float) -> str:
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        box: Dict[str, str] = {}
        sub = self.node.create_subscription(String, "/robot_description", lambda m: box.setdefault("x", m.data), qos)
        deadline = time.monotonic() + timeout_s
        while "x" not in box and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        if "x" not in box:
            raise PreconditionError("no /robot_description received")
        return box["x"]

    def fk(self, state: Dict[str, float], links: List[str]) -> Dict[str, np.ndarray]:
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = list(links)
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = list(state)
        req.robot_state.joint_state.position = [float(v) for v in state.values()]
        resp = self.call("fk", req)
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"compute_fk failed with code {resp.error_code.val}")
        out = {}
        for name, ps in zip(resp.fk_link_names, resp.pose_stamped):
            t = np.eye(4)
            t[:3, :3] = _quat_to_matrix(ps.pose.orientation)
            t[:3, 3] = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
            out[name] = t
        return out

    def valid(self, state: Dict[str, float]) -> bool:
        req = GetStateValidity.Request()
        req.group_name = "arm"
        req.robot_state.joint_state.name = list(state)
        req.robot_state.joint_state.position = [float(v) for v in state.values()]
        return bool(self.call("valid", req).valid)

    def ik(self, link: str, position, rotation: np.ndarray) -> Optional[Dict[str, float]]:
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = link
        req.ik_request.avoid_collisions = True
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in position)
        (ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
         ps.pose.orientation.w) = _matrix_to_quat(rotation)
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout.nanosec = 50_000_000
        req.ik_request.robot_state.joint_state.name = list(ZERO_STATE)
        req.ik_request.robot_state.joint_state.position = list(ZERO_STATE.values())
        resp = self.call("ik", req)
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        return dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))


def _joint_origin(urdf: ET.Element, child: str):
    for j in urdf.findall("joint"):
        c = j.find("child")
        if c is not None and c.get("link") == child:
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()] if o is not None else [0.0] * 3
            rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None else [0.0] * 3
            return j.find("parent").get("link"), xyz, rpy
    return None


def _check(summary: RunSummary, log, ok: bool, what: str) -> bool:
    summary.increment("checks_passed" if ok else "checks_failed")
    (log.info if ok else log.error)("%s: %s", "PASS" if ok else "FAIL", what)
    if not ok:
        summary.add_error(what)
    return ok


def _run(args: argparse.Namespace, log, summary: RunSummary) -> Dict[str, str]:
    root = Path(args.root).expanduser().resolve() if args.root is not None else find_root()
    cfg_path = Path(args.end_effector_config).resolve() if args.end_effector_config else root / DEFAULT_CONFIG
    try:
        cfg = load_config(cfg_path)
    except EndEffectorError as exc:  # §0.2: invalid config -> exit 2
        raise ConfigError(str(exc)) from exc
    tool, cam = cfg["tool"], cfg["wrist_camera"]
    parent = cfg["parent_link"]

    rclpy.init(args=None)
    node = Node(TOOL)
    try:
        live = _Live(node, args.service_timeout_s)
        urdf = ET.fromstring(live.robot_description(args.service_timeout_s))

        # 1. overlay links + origins
        expected = [(tool["frame"], parent, tool), (cam["frame"], parent, cam)]
        expected += [(c["id"], c["frame"], c) for c in cfg["collision"]]
        for link, want_parent, block in expected:
            got = _joint_origin(urdf, link)
            ok = (got is not None and got[0] == want_parent
                  and np.allclose(got[1], block["xyz_m"], atol=ORIGIN_TOL)
                  and np.allclose(got[2], block["rpy_rad"], atol=ORIGIN_TOL))
            _check(summary, log, ok, f"URDF joint origin of '{link}' under '{want_parent}' equals the config "
                                     f"(got {got})")

        # 2. FK reproduces the fixed transforms
        frames = [parent, tool["frame"], cam["frame"]]
        poses = live.fk(ZERO_STATE, frames)
        for frame, block in ((tool["frame"], tool), (cam["frame"], cam)):
            rel = np.linalg.inv(poses[parent]) @ poses[frame]
            want = transform(block)
            dpos = float(np.linalg.norm(rel[:3, 3] - want[:3, 3]))
            drot = float(np.linalg.norm(rel[:3, :3] - want[:3, :3]))
            _check(summary, log, dpos <= FK_TOL_M and drot <= FK_TOL_RAD,
                   f"FK T_{parent}_{frame} matches config (|dp|={dpos:.2e} m, |dR|={drot:.2e})")

        # 3. zero pose: collision-free, camera above the gripper (mount-side assumption)
        _check(summary, log, live.valid(ZERO_STATE), "all-zero (SRDF home) state is collision-free with the camera proxies")
        dz = float(poses[cam["frame"]][2, 3] - poses[parent][2, 3])
        _check(summary, log, dz > 0.03, f"camera_link is above gripper_link at the zero pose (dz={dz:+.4f} m)")

        # 4. IK on rigidly attached task frames + FK agreement
        for link, axis, target in ((tool["frame"], np.array(tool["pointing_axis"]), TOOL_PROBE),
                                   (cam["frame"], np.array([1.0, 0.0, 0.0]), VIEW_PROBE)):
            solved = None
            for k in range(ROLL_SAMPLES):
                rot = _axis_down(axis, 2 * math.pi * k / ROLL_SAMPLES)
                sol = live.ik(link, target, rot)
                if sol is not None:
                    solved = (sol, rot)
                    break
            if not _check(summary, log, solved is not None,
                          f"compute_ik with ik_link_name='{link}' reaches {target} pointing down"):
                continue
            sol, rot = solved
            state = {**ZERO_STATE, **{j: sol[j] for j in ARM_JOINTS}}
            t = live.fk(state, [link])[link]
            perr = float(np.linalg.norm(t[:3, 3] - np.array(target)))
            aerr = math.degrees(math.acos(float(np.clip(np.dot(t[:3, :3] @ axis, [0.0, 0.0, -1.0]), -1, 1))))
            _check(summary, log, perr <= POS_TOL_M and aerr <= AXIS_TOL_DEG,
                   f"FK of the '{link}' IK solution agrees (pos {perr * 1000:.3f} mm, axis {aerr:.3f} deg)")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return {"status": "failed" if summary.counts.get("checks_failed") else "ok"}


def _extra_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--end-effector-config", default=None,
                        help=f"end-effector YAML (default: {DEFAULT_CONFIG} under the repo root)")
    parser.add_argument("--service-timeout-s", type=float, default=90.0)


def main(argv=None) -> int:
    argv = list(argv) if argv is not None else sys.argv[1:]
    return run_cli(TOOL, _run, argv, extra_args=_extra_args)


if __name__ == "__main__":
    sys.exit(main())

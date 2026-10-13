"""crackvision_motion.tool_fk — pure-numpy base_link->tool_tip FK for the retimer (MOT-07.2).

docs/INTERFACES.md §16.2 Step 4: a ROS-free `tool_fk(q)` composing the URDF chain
base_link->gripper_link with end_effector.yaml's `tool` block (via
crackvision_description.end_effector, which already validates/parses that file -- this module never
re-parses it), in exactly the composition `crackvision.kinematics.load_chain(...).fk(q)` followed by
`load_end_effector(...).T_gripper_link_tool_tip` performs, so the retimer and the §13/§8 path
generation code can never silently diverge on where `tool_tip` is. numpy + PyYAML (via
crackvision_description.end_effector) + stdlib only -- no rclpy/*_msgs, no `crackvision` import, so
this runs under plain /usr/bin/python3 with no colcon build and no conda env.
"""

from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from crackvision_description.end_effector import load_config, transform as ee_transform

BASE_LINK = "base_link"
GRIPPER_LINK = "gripper_link"
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


class ToolFKError(ValueError):
    """Malformed URDF/end_effector.yaml, or a tool_fk call with the wrong joint count."""


def _file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _parse_vec3(text, default=(0.0, 0.0, 0.0)):
    if text is None:
        return tuple(default)
    parts = [float(x) for x in text.split()]
    if len(parts) != 3:
        raise ToolFKError(f"expected 3 floats, got {text!r}")
    return tuple(parts)


def _rpy_to_matrix(rpy) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _transform_from_xyz_rpy(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def _axis_angle_matrix(axis, angle: float) -> np.ndarray:
    """Rodrigues' rotation formula: revolute joint rotation about its normalised axis."""
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        raise ToolFKError(f"joint axis is zero: {axis.tolist()}")
    x, y, z = axis / norm
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


@dataclass
class _JointSpec:
    name: str
    type: str
    xyz: tuple
    rpy: tuple
    axis: tuple


@dataclass
class ToolChain:
    """base_link->tool_tip chain: the URDF revolute/fixed joints plus the tool offset."""

    joints: list
    urdf_sha256: str
    end_effector_sha256: str
    T_gripper_link_tool_tip: np.ndarray

    @property
    def joint_names(self) -> list:
        return [j.name for j in self.joints if j.type != "fixed"]

    def _T_base_gripper(self, q) -> np.ndarray:
        names = self.joint_names
        q = list(q)
        if len(q) != len(names):
            raise ToolFKError(f"expected {len(names)} joint positions ({names}), got {len(q)}")
        values = dict(zip(names, (float(v) for v in q)))
        T = np.eye(4)
        for j in self.joints:
            origin = _transform_from_xyz_rpy(j.xyz, j.rpy)
            if j.type == "fixed":
                joint_T = np.eye(4)
            else:
                qv = values[j.name]
                if not math.isfinite(qv):
                    raise ToolFKError(f"joint {j.name!r} position is not finite: {qv!r}")
                joint_T = np.eye(4)
                joint_T[:3, :3] = _axis_angle_matrix(j.axis, qv)
            T = T @ origin @ joint_T
        return T

    def tool_pose(self, q) -> np.ndarray:
        """4x4 T_base_link_tool_tip for joint positions `q` (joint1..6, radians)."""
        return self._T_base_gripper(q) @ self.T_gripper_link_tool_tip

    def tool_positions(self, Q) -> np.ndarray:
        """(N, 3) tool_tip origins in base_link for an (N, 6) array of joint configurations."""
        Q = np.asarray(Q, dtype=float)
        n = len(self.joint_names)
        if Q.ndim != 2 or Q.shape[1] != n:
            raise ToolFKError(f"expected an (N, {n}) array, got shape {Q.shape}")
        out = np.empty((Q.shape[0], 3), dtype=float)
        for i in range(Q.shape[0]):
            out[i] = self.tool_pose(Q[i])[:3, 3]
        return out


def load_tool_chain(
    urdf_path,
    end_effector_path,
    base: str = BASE_LINK,
    tip: str = GRIPPER_LINK,
) -> ToolChain:
    """Build a `ToolChain` from a URDF (`base`->`tip` chain) and an end_effector.yaml tool block."""
    urdf_path = Path(urdf_path)
    end_effector_path = Path(end_effector_path)

    if not urdf_path.is_file():
        raise ToolFKError(f"URDF not found: {urdf_path}")
    urdf_sha256 = _file_sha256(urdf_path)
    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError as exc:
        raise ToolFKError(f"malformed URDF {urdf_path}: {exc}") from exc

    joints_by_child = {}
    for joint_el in root.findall("joint"):
        child = joint_el.find("child")
        if child is None:
            raise ToolFKError(f"joint {joint_el.get('name')!r} has no <child>")
        joints_by_child[child.get("link")] = joint_el

    ordered = []
    link = tip
    while link != base:
        joint_el = joints_by_child.get(link)
        if joint_el is None:
            raise ToolFKError(
                f"no joint chain from {base!r} to {tip!r}: nothing produces link {link!r}"
            )
        parent_el = joint_el.find("parent")
        if parent_el is None:
            raise ToolFKError(f"joint {joint_el.get('name')!r} has no <parent>")

        origin_el = joint_el.find("origin")
        xyz = _parse_vec3(origin_el.get("xyz") if origin_el is not None else None)
        rpy = _parse_vec3(origin_el.get("rpy") if origin_el is not None else None)

        axis_el = joint_el.find("axis")
        axis = _parse_vec3(axis_el.get("xyz") if axis_el is not None else None, default=(1.0, 0.0, 0.0))

        ordered.append(_JointSpec(
            name=joint_el.get("name"), type=joint_el.get("type"), xyz=xyz, rpy=rpy, axis=axis,
        ))
        link = parent_el.get("link")

    ordered.reverse()
    names = [j.name for j in ordered if j.type != "fixed"]
    if names != list(JOINT_NAMES):
        raise ToolFKError(
            f"expected joint chain {base!r} -> {tip!r} to carry revolute joints "
            f"{list(JOINT_NAMES)}, got {names}"
        )

    try:
        cfg = load_config(end_effector_path)
    except Exception as exc:
        raise ToolFKError(f"failed to load end-effector config {end_effector_path}: {exc}") from exc
    if cfg["parent_link"] != tip:
        raise ToolFKError(
            f"end_effector.yaml parent_link must be {tip!r}, got {cfg['parent_link']!r}"
        )
    end_effector_sha256 = _file_sha256(end_effector_path)
    T_tool = ee_transform(cfg["tool"])

    return ToolChain(
        joints=ordered,
        urdf_sha256=urdf_sha256,
        end_effector_sha256=end_effector_sha256,
        T_gripper_link_tool_tip=T_tool,
    )

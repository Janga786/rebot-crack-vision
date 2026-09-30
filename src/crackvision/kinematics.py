"""crackvision.kinematics — pure-numpy B601-DM forward kinematics + end-effector transforms (GEOM-08.2).

Implements docs/INTERFACES.md §8.4/§9.2's `FK(q)` and the `end_effector.yaml` loader it composes
with. Uses only numpy, scipy, PyYAML and the stdlib `xml.etree` — never ROS (rclpy/moveit/tf2/
geometry_msgs), so this module works outside a ROS environment (e.g. in path-lifting CLIs).

Convention (ADR-012, docs/INTERFACES.md §6.2): `T_a_b` maps a point from frame `b` into frame `a`,
quaternions are ROS `(x, y, z, w)`. URDF `<origin rpy="r p y"/>` is the fixed-axis (extrinsic)
convention `R = Rz(yaw) * Ry(pitch) * Rx(roll)`.
"""

from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from crackvision.config import find_root

DEFAULT_URDF = find_root() / "presentation" / "sim" / "reBot_B601_DM_with_gripper.urdf"
DEFAULT_END_EFFECTOR = find_root() / "config" / "robot" / "end_effector.yaml"

# §9.1 nominal_d405: zero translation, rpy (-pi/2, 0, -pi/2). optical +z -> camera_link +x,
# optical +x -> -y, optical +y -> -z.
NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME_XYZ = (0.0, 0.0, 0.0)
NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME_RPY = (-math.pi / 2, 0.0, -math.pi / 2)

# §10.4 nominal calibration priors, used while a block's value_status is "nominal".
_NOMINAL_CAMERA_SIGMA_POS_M = 0.010
_NOMINAL_CAMERA_SIGMA_ROT_RAD = math.radians(5.0)
_NOMINAL_TOOL_SIGMA_POS_M = 0.005


class KinematicsError(Exception):
    """Malformed URDF/end_effector.yaml, or an FK call outside the chain's contract."""


def file_sha256(path: Path | str) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def transform_from_xyz_rpy(xyz, rpy) -> np.ndarray:
    """URDF fixed-axis convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    roll, pitch, yaw = rpy
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = xyz
    return T


def transform_from_xyz_quat(xyz, quat_xyzw) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = xyz
    return T


def transform_to_xyz_quat(T: np.ndarray):
    xyz = tuple(np.asarray(T)[:3, 3].tolist())
    quat = tuple(Rotation.from_matrix(np.asarray(T)[:3, :3]).as_quat().tolist())
    return xyz, quat


def _parse_vec3(text: str | None, default=(0.0, 0.0, 0.0)):
    if text is None:
        return tuple(default)
    parts = [float(x) for x in text.split()]
    if len(parts) != 3:
        raise KinematicsError(f"expected 3 floats, got {text!r}")
    return tuple(parts)


@dataclass
class _JointSpec:
    name: str
    type: str
    xyz: tuple
    rpy: tuple
    axis: tuple
    lower: float | None
    upper: float | None


@dataclass
class KinematicChain:
    joints: list  # ordered _JointSpec, base -> tip, all joint types (including fixed)
    base: str
    tip: str
    urdf_sha256: str

    @property
    def joint_names(self) -> list[str]:
        return [j.name for j in self.joints if j.type != "fixed"]

    def fk(self, positions) -> np.ndarray:
        names = self.joint_names
        if isinstance(positions, dict):
            given = set(positions.keys())
            expected = set(names)
            if given != expected:
                raise KinematicsError(
                    f"joint name mismatch: expected {sorted(expected)}, got {sorted(given)}"
                )
            values = {k: float(v) for k, v in positions.items()}
        else:
            positions = list(positions)
            if len(positions) != len(names):
                raise KinematicsError(
                    f"expected {len(names)} joint positions ({names}), got {len(positions)}"
                )
            values = dict(zip(names, (float(v) for v in positions)))

        T = np.eye(4)
        for j in self.joints:
            origin = transform_from_xyz_rpy(j.xyz, j.rpy)
            if j.type == "fixed":
                joint_T = np.eye(4)
            else:
                q = values[j.name]
                if not math.isfinite(q):
                    raise KinematicsError(f"joint {j.name!r} position is not finite: {q!r}")
                if j.lower is not None and j.upper is not None:
                    if q < j.lower - 1e-3 or q > j.upper + 1e-3:
                        raise KinematicsError(
                            f"joint {j.name!r} position {q} outside limits "
                            f"[{j.lower}, {j.upper}] (tolerance 1e-3 rad)"
                        )
                axis = np.asarray(j.axis, dtype=float)
                axis = axis / np.linalg.norm(axis)
                joint_T = np.eye(4)
                if j.type in ("revolute", "continuous"):
                    joint_T[:3, :3] = Rotation.from_rotvec(axis * q).as_matrix()
                elif j.type == "prismatic":
                    joint_T[:3, 3] = axis * q
                else:
                    raise KinematicsError(f"unsupported joint type {j.type!r}")
            T = T @ origin @ joint_T
        return T


def load_chain(
    urdf_path: Path | str = None,
    base: str = "base_link",
    tip: str = "gripper_link",
) -> KinematicChain:
    urdf_path = Path(urdf_path) if urdf_path is not None else DEFAULT_URDF
    urdf_sha256 = file_sha256(urdf_path)
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints_by_child: dict[str, ET.Element] = {}
    for joint_el in root.findall("joint"):
        child = joint_el.find("child")
        if child is None:
            raise KinematicsError(f"joint {joint_el.get('name')!r} has no <child>")
        joints_by_child[child.get("link")] = joint_el

    ordered: list[_JointSpec] = []
    link = tip
    while link != base:
        joint_el = joints_by_child.get(link)
        if joint_el is None:
            raise KinematicsError(
                f"no joint chain from {base!r} to {tip!r}: nothing produces link {link!r}"
            )
        parent_el = joint_el.find("parent")
        if parent_el is None:
            raise KinematicsError(f"joint {joint_el.get('name')!r} has no <parent>")

        origin_el = joint_el.find("origin")
        xyz = _parse_vec3(origin_el.get("xyz") if origin_el is not None else None)
        rpy = _parse_vec3(origin_el.get("rpy") if origin_el is not None else None)

        axis_el = joint_el.find("axis")
        axis = _parse_vec3(axis_el.get("xyz") if axis_el is not None else None, default=(1.0, 0.0, 0.0))

        lower = upper = None
        limit_el = joint_el.find("limit")
        if limit_el is not None:
            if limit_el.get("lower") is not None:
                lower = float(limit_el.get("lower"))
            if limit_el.get("upper") is not None:
                upper = float(limit_el.get("upper"))

        ordered.append(
            _JointSpec(
                name=joint_el.get("name"),
                type=joint_el.get("type"),
                xyz=xyz,
                rpy=rpy,
                axis=axis,
                lower=lower,
                upper=upper,
            )
        )
        link = parent_el.get("link")

    ordered.reverse()
    chain = KinematicChain(joints=ordered, base=base, tip=tip, urdf_sha256=urdf_sha256)
    assert chain.joint_names == [f"joint{i}" for i in range(1, 7)], (
        f"expected joint1..joint6, got {chain.joint_names}"
    )
    return chain


@dataclass
class EndEffector:
    T_gripper_link_camera_link: np.ndarray
    T_gripper_link_tool_tip: np.ndarray
    wrist_camera_value_status: str
    tool_value_status: str
    sha256: str
    camera_sigma_pos_m: float
    camera_sigma_rot_rad: float
    tool_sigma_pos_m: float
    sigma_source: dict


def _load_block_sigma(block: dict, name: str, needs_rotation: bool, nominal_pos: float, nominal_rot: float | None):
    status = block.get("value_status")
    if status == "measured":
        uncertainty = block.get("uncertainty")
        if not uncertainty or "position_sigma_m" not in uncertainty:
            raise KinematicsError(
                f"end_effector.yaml block {name!r} is value_status: measured but lacks the "
                "§10 uncertainty.position_sigma_m field"
            )
        pos = float(uncertainty["position_sigma_m"])
        source = uncertainty.get("source", "measured")
        if needs_rotation:
            if "rotation_sigma_rad" not in uncertainty:
                raise KinematicsError(
                    f"end_effector.yaml block {name!r} is value_status: measured but lacks the "
                    "§10 uncertainty.rotation_sigma_rad field"
                )
            rot = float(uncertainty["rotation_sigma_rad"])
        else:
            rot = None
        return pos, rot, source
    if status == "nominal":
        return nominal_pos, nominal_rot, "§10 nominal prior"
    raise KinematicsError(f"end_effector.yaml block {name!r} has unknown value_status {status!r}")


def load_end_effector(path: Path | str = None) -> EndEffector:
    path = Path(path) if path is not None else DEFAULT_END_EFFECTOR
    sha256 = file_sha256(path)
    with open(path, "r") as f:
        cfg = yaml.safe_load(f)

    if cfg.get("parent_link") != "gripper_link":
        raise KinematicsError(
            f"end_effector.yaml parent_link must be 'gripper_link', got {cfg.get('parent_link')!r}"
        )

    tool = cfg["tool"]
    wrist_camera = cfg["wrist_camera"]

    T_tool = transform_from_xyz_rpy(tool["xyz_m"], tool["rpy_rad"])
    T_camera = transform_from_xyz_rpy(wrist_camera["xyz_m"], wrist_camera["rpy_rad"])

    tool_pos_sigma, _tool_rot_sigma, tool_source = _load_block_sigma(
        tool, "tool", needs_rotation=False,
        nominal_pos=_NOMINAL_TOOL_SIGMA_POS_M, nominal_rot=None,
    )
    camera_pos_sigma, camera_rot_sigma, camera_source = _load_block_sigma(
        wrist_camera, "wrist_camera", needs_rotation=True,
        nominal_pos=_NOMINAL_CAMERA_SIGMA_POS_M, nominal_rot=_NOMINAL_CAMERA_SIGMA_ROT_RAD,
    )

    return EndEffector(
        T_gripper_link_camera_link=T_camera,
        T_gripper_link_tool_tip=T_tool,
        wrist_camera_value_status=wrist_camera["value_status"],
        tool_value_status=tool["value_status"],
        sha256=sha256,
        camera_sigma_pos_m=camera_pos_sigma,
        camera_sigma_rot_rad=camera_rot_sigma,
        tool_sigma_pos_m=tool_pos_sigma,
        sigma_source={"wrist_camera": camera_source, "tool": tool_source},
    )


NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME = transform_from_xyz_rpy(
    NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME_XYZ,
    NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME_RPY,
)

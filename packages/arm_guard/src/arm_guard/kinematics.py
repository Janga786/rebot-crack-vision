"""arm_guard.kinematics — stdlib-only URDF forward kinematics for the keep-out checks.

No numpy/scipy/yaml on purpose: the guard has to import inside every interpreter that can reach the motor
bus (conda envs `rebot`, `lerobot`, `crackvision`), and those do not share a dependency set.

Convention: URDF `<origin rpy="r p y"/>` is R = Rz(y) Ry(p) Rx(r); a revolute joint rotates its child about
`<axis>` (Rodrigues) after the origin transform; a prismatic joint translates along it.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

Mat = list  # 4x4 row-major nested lists


class KinematicsError(Exception):
    pass


def _mm(a: Mat, b: Mat) -> Mat:
    """Product of two homogeneous transforms (bottom rows are [0, 0, 0, 1])."""
    (a00, a01, a02, a03), (a10, a11, a12, a13), (a20, a21, a22, a23) = a[0], a[1], a[2]
    (b00, b01, b02, b03), (b10, b11, b12, b13), (b20, b21, b22, b23) = b[0], b[1], b[2]
    return [[a00 * b00 + a01 * b10 + a02 * b20, a00 * b01 + a01 * b11 + a02 * b21,
             a00 * b02 + a01 * b12 + a02 * b22, a00 * b03 + a01 * b13 + a02 * b23 + a03],
            [a10 * b00 + a11 * b10 + a12 * b20, a10 * b01 + a11 * b11 + a12 * b21,
             a10 * b02 + a11 * b12 + a12 * b22, a10 * b03 + a11 * b13 + a12 * b23 + a13],
            [a20 * b00 + a21 * b10 + a22 * b20, a20 * b01 + a21 * b11 + a22 * b21,
             a20 * b02 + a21 * b12 + a22 * b22, a20 * b03 + a21 * b13 + a22 * b23 + a23],
            [0.0, 0.0, 0.0, 1.0]]


def _rpy(r: float, p: float, y: float) -> list:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _homog(R: list, t) -> Mat:
    return [[R[0][0], R[0][1], R[0][2], t[0]],
            [R[1][0], R[1][1], R[1][2], t[1]],
            [R[2][0], R[2][1], R[2][2], t[2]],
            [0.0, 0.0, 0.0, 1.0]]


def _axis_rot(a, q: float) -> Mat:
    x, y, z = a
    c, s, v = math.cos(q), math.sin(q), 1.0 - math.cos(q)
    return _homog([[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
                   [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
                   [z * x * v - y * s, z * y * v + x * s, c + z * z * v]], (0.0, 0.0, 0.0))


def _vec3(text, default=(0.0, 0.0, 0.0)):
    if text is None:
        return tuple(default)
    parts = [float(x) for x in text.split()]
    if len(parts) != 3:
        raise KinematicsError(f"expected 3 floats, got {text!r}")
    return tuple(parts)


@dataclass(frozen=True)
class _Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: tuple  # 4x4 as tuple of tuples
    axis: tuple


class Chain:
    """Every joint of a URDF in topological order from the root link."""

    def __init__(self, urdf_path: str) -> None:
        try:
            root = ET.parse(urdf_path).getroot()
        except (OSError, ET.ParseError) as e:
            raise KinematicsError(f"cannot read URDF {urdf_path}: {e}") from e
        joints = []
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = _vec3(o.get("xyz") if o is not None else None)
            rpy = _vec3(o.get("rpy") if o is not None else None)
            ax = j.find("axis")
            axis = _vec3(ax.get("xyz") if ax is not None else None, (1.0, 0.0, 0.0))
            n = math.sqrt(sum(c * c for c in axis))
            if n == 0.0:
                if j.get("type") != "fixed":
                    raise KinematicsError(f"joint {j.get('name')}: zero axis")
                n = 1.0
            joints.append(_Joint(j.get("name"), j.get("type"), j.find("parent").get("link"),
                                 j.find("child").get("link"),
                                 tuple(tuple(r) for r in _homog(_rpy(*rpy), xyz)),
                                 tuple(c / n for c in axis)))
        children = {j.child for j in joints}
        roots = {j.parent for j in joints} - children
        if len(roots) != 1:
            raise KinematicsError(f"URDF must have exactly one root link, found {sorted(roots)}")
        self.base = roots.pop()
        self.order: list[_Joint] = []
        frontier = [self.base]
        while frontier:
            link = frontier.pop(0)
            for j in joints:
                if j.parent == link:
                    self.order.append(j)
                    frontier.append(j.child)
        self.links = {self.base} | {j.child for j in self.order}
        self.joint_names = [j.name for j in self.order if j.type != "fixed"]

    def link_positions(self, q: dict, links=None) -> dict:
        """World (base-frame) origin of each requested link for joint values `q` (missing joints = 0)."""
        T = {self.base: [[1.0 if i == k else 0.0 for k in range(4)] for i in range(4)]}
        for j in self.order:
            Tj = _mm(T[j.parent], j.origin)
            if j.type in ("revolute", "continuous"):
                Tj = _mm(Tj, _axis_rot(j.axis, float(q.get(j.name, 0.0))))
            elif j.type == "prismatic":
                d = float(q.get(j.name, 0.0))
                Tj = _mm(Tj, _homog([[1, 0, 0], [0, 1, 0], [0, 0, 1]], tuple(a * d for a in j.axis)))
            T[j.child] = Tj
        want = self.links if links is None else links
        return {k: (T[k][0][3], T[k][1][3], T[k][2][3]) for k in want}

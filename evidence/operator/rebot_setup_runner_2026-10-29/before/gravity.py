"""Static gravity holding torque per arm joint from the vendor URDF (pure Python, no numpy).

hold_torques(q) returns, for joints j1..j6, the motor torque (Nm) that holds the arm still against gravity at joint
angles q (rad, motor angle = URDF angle; base upright, gravity -Z). It is what an MIT feed-forward tau_ff should be.
Sign check (2026-10-17): at the folded pose j3 needs about -2.8 Nm, and a real j3 hold measured -5.1 Nm (with friction
and kp error). The model ignores the D405 camera and its mount; scale per joint with GRAVITY_SCALE if needed.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

URDF = Path(__file__).with_name("model") / "reBot_B601_DM_with_gripper.urdf"
ARM_JOINTS = {"j1": "joint1", "j2": "joint2", "j3": "joint3", "j4": "joint4", "j5": "joint5", "j6": "joint6"}
G = (0.0, 0.0, -9.81)


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _origin(el):
    o = el.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    R = _rpy(*rpy)
    return [R[0] + [xyz[0]], R[1] + [xyz[1]], R[2] + [xyz[2]], [0.0, 0.0, 0.0, 1.0]]


def _axis_rot(a, q):
    n = math.sqrt(sum(x * x for x in a))
    x, y, z = (v / n for v in a)
    c, s, C = math.cos(q), math.sin(q), 1 - math.cos(q)
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s, 0.0],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s, 0.0],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


class GravityModel:
    def __init__(self, urdf: Path = URDF, extra: tuple = ()) -> None:
        """extra: point masses not in the URDF, as (link, (x, y, z) in that link's frame, kg)."""
        root = ET.parse(urdf).getroot()
        self.inertial = {}
        for l in root.findall("link"):
            i = l.find("inertial")
            if i is not None:
                self.inertial[l.get("name")] = (float(i.find("mass").get("value")), _origin(i))
        for link, xyz, m in extra:   # merged into the link's inertial as a separate point mass
            self.inertial[f"{link}+{xyz}"] = (m, [[1.0, 0.0, 0.0, xyz[0]], [0.0, 1.0, 0.0, xyz[1]],
                                                    [0.0, 0.0, 1.0, xyz[2]], [0.0, 0.0, 0.0, 1.0]])
        self._extra_parent = {f"{link}+{xyz}": link for link, xyz, m in extra}
        joints = root.findall("joint")
        children = {j.find("child").get("link") for j in joints}
        base = [l.get("name") for l in root.findall("link") if l.get("name") not in children][0]
        # topological order from the base
        self.order = []
        known, pending = {base}, list(joints)
        while pending:
            progressed = False
            for j in list(pending):
                if j.find("parent").get("link") in known:
                    ax = j.find("axis")
                    self.order.append((j.get("name"), j.get("type"), j.find("parent").get("link"),
                                       j.find("child").get("link"), _origin(j),
                                       [float(v) for v in (ax.get("xyz") if ax is not None else "1 0 0").split()]))
                    known.add(j.find("child").get("link"))
                    pending.remove(j)
                    progressed = True
            if not progressed:
                raise ValueError("URDF joints do not form a tree")
        self.base = base
        self.sub = {}   # link -> all links at or below it
        kids = {}
        for name, typ, p, c, *_ in self.order:
            kids.setdefault(p, []).append(c)

        def subtree(link):
            out = [link]
            for c in kids.get(link, []):
                out += subtree(c)
            return out
        for name, typ, p, c, *_ in self.order:
            self.sub[name] = subtree(c)

    def hold_torques(self, q: dict[str, float]) -> dict[str, float]:
        """q: {"j1": rad, ...} (missing = 0). Returns {"j1".."j6": holding torque, Nm}."""
        urdf_q = {ARM_JOINTS[k]: v for k, v in q.items() if k in ARM_JOINTS}
        T = {self.base: [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]}
        jinfo = {}
        for name, typ, p, c, org, ax in self.order:
            Tj = _mm(T[p], org)
            if typ in ("revolute", "continuous"):
                n = math.sqrt(sum(x * x for x in ax))
                a = [sum(Tj[i][k] * ax[k] for k in range(3)) / n for i in range(3)]
                jinfo[name] = ([Tj[0][3], Tj[1][3], Tj[2][3]], a)
                Tj = _mm(Tj, _axis_rot(ax, urdf_q.get(name, 0.0)))
            T[c] = Tj
        com = {}
        for ln, (m, org) in self.inertial.items():
            Tc = _mm(T[self._extra_parent.get(ln, ln)], org)
            com[ln] = (m, [Tc[0][3], Tc[1][3], Tc[2][3]])
        out = {}
        for short, name in ARM_JOINTS.items():
            p, a = jinfo[name]
            tau = 0.0
            for ln in self.sub[name] + [k for k, p in self._extra_parent.items() if p in self.sub[name]]:
                if ln not in com:
                    continue
                m, c = com[ln]
                r = [c[i] - p[i] for i in range(3)]
                f = [m * g for g in G]
                cross = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]]
                tau += sum(a[i] * cross[i] for i in range(3))   # torque gravity applies about the axis
            out[short] = -tau                                    # the motor must apply the opposite to hold
        return out


if __name__ == "__main__":
    import sys
    gm = GravityModel()
    q = {}
    for arg in sys.argv[1:]:
        k, v = arg.split("=")
        q[k] = float(v)
    print(", ".join(f"{k} {v:+.3f}" for k, v in gm.hold_torques(q).items()))

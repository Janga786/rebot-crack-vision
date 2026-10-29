"""crackvision.gravity_model — vectorised URDF gravity for the B601-DM and the `crackvision.gravity_model/1` file.

The same physics as the runner's `~/rebot_setup/gravity.py` (its copy: evidence/operator/controller-tuning-2026-10/
code/gravity.py), in numpy over many poses at once, plus the linear regressor of the gravity holding torque in the
unknown extra bodies. For a point mass m at local position r on link L (world rotation R_L, origin t_L) and a joint
i proximal to L (world axis a_i through p_i, gravity g):

    G_i = -a_i . ((R_L r + t_L - p_i) x m g) = -m (t_L - p_i).(g x a_i) - (m r).(R_L^T (g x a_i))

so G is linear in m (one column) and in the first moment m r (three columns) of every extra body: rigid-body gravity
extrapolates by geometry, not by curve shape. Used by tools/fit_gravity_model.py (fit), tools/analyze_run_gains.py
(--gravity-model re-scoring) and the tests. Nothing here talks to hardware.

A `crackvision.gravity_model/1` file (config/robot/gravity_model.yaml) holds the fitted extra point masses
(link, xyz in that link's frame, kg), the sha256 of the URDF they belong to, the per-joint scale the runner applies
with it (1.0), the runs it was fitted on, the residuals and where it is valid. `~/rebot_setup/demo_move.py
--gravity-model FILE` loads the same file into `GravityModel(extra=...)`.
"""

from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from crackvision.config import find_root

SCHEMA = "crackvision.gravity_model/1"
ROOT = find_root()
# byte-identical to ~/rebot_setup/model/reBot_B601_DM_with_gripper.urdf (the file the runner's gravity.py reads);
# presentation/sim/ has the same model with absolute mesh paths (a different sha256)
DEFAULT_URDF = ROOT / "evidence" / "operator" / "controller-tuning-2026-10" / "code" / "model" / \
    "reBot_B601_DM_with_gripper.urdf"
DEFAULT_MODEL = ROOT / "config" / "robot" / "gravity_model.yaml"
G = np.array([0.0, 0.0, -9.81])
ARM = ("j1", "j2", "j3", "j4", "j5", "j6")
URDF_JOINT = {f"j{i}": f"joint{i}" for i in range(1, 7)}

# ~/rebot_setup (first_move_j1.TUNED, demo_move.py), 2026-10-25: what the runner does today
KP = {"j1": 100.0, "j2": 100.0, "j3": 80.0, "j4": 40.0, "j5": 40.0, "j6": 40.0}
RUNNER_SCALE = {"j1": 1.0, "j2": 0.6, "j3": 2.5, "j4": 2.5, "j5": 1.0, "j6": 1.0}
RUNNER_FRICTION = {"j1": 1.0, "j2": 1.0, "j3": 2.0, "j4": 0.5, "j5": 0.3, "j6": 0.1}
FRICTION_VEL_EPS = 0.02
BIAS_2026_10_25 = {"j2": 1.4, "j3": -0.6, "j4": -0.2}
FF_CAP = {"j1": 10.0, "j2": 10.0, "j3": 10.0, "j4": 2.5, "j5": 2.5, "j6": 2.5}   # runner and arm_guard tau_ff_max


class GravityModelError(ValueError):
    """A malformed gravity-model file, or one that belongs to a different URDF."""


def file_sha256(path: Path | str) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(el) -> tuple[np.ndarray, np.ndarray]:
    o = el.find("origin")
    xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return _rpy(*rpy), xyz


def _rodrigues(axis: np.ndarray, q: np.ndarray) -> np.ndarray:
    """(N, 3, 3) rotations about the unit `axis` by angles q (N,)."""
    x, y, z = axis
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    s, c = np.sin(q)[:, None, None], np.cos(q)[:, None, None]
    return np.eye(3)[None] + s * K[None] + (1 - c) * (K @ K)[None]


@dataclass
class Frames:
    """Forward kinematics of N poses: per link, world rotation (N,3,3) and origin (N,3); per arm joint, world axis
    and a point on it (N,3), taken before the joint's own rotation (as gravity.py does)."""
    R: dict
    t: dict
    axis: dict
    point: dict


class UrdfGravity:
    """The URDF's links, masses and joint tree, with vectorised FK and gravity holding torques."""

    def __init__(self, urdf: Path | str = DEFAULT_URDF) -> None:
        self.path = Path(urdf)
        self.sha256 = file_sha256(self.path)
        root = ET.parse(self.path).getroot()
        self.mass, self.com = {}, {}
        for link in root.findall("link"):
            i = link.find("inertial")
            if i is not None:
                self.mass[link.get("name")] = float(i.find("mass").get("value"))
                self.com[link.get("name")] = _origin(i)[1]
        joints = root.findall("joint")
        children = {j.find("child").get("link") for j in joints}
        self.base = [l.get("name") for l in root.findall("link") if l.get("name") not in children][0]
        self.order, known, pending = [], {self.base}, list(joints)
        while pending:
            progressed = False
            for j in list(pending):
                if j.find("parent").get("link") in known:
                    ax = j.find("axis")
                    a = np.array([float(v) for v in (ax.get("xyz") if ax is not None else "1 0 0").split()])
                    self.order.append((j.get("name"), j.get("type"), j.find("parent").get("link"),
                                       j.find("child").get("link"), *_origin(j),
                                       a / np.linalg.norm(a) if np.linalg.norm(a) > 0 else a))
                    known.add(j.find("child").get("link"))
                    pending.remove(j)
                    progressed = True
            if not progressed:
                raise GravityModelError("URDF joints do not form a tree")
        kids: dict = {}
        for name, typ, p, c, *_ in self.order:
            kids.setdefault(p, []).append(c)

        def subtree(link: str) -> list[str]:
            out = [link]
            for c in kids.get(link, []):
                out += subtree(c)
            return out
        self.distal = {name: set(subtree(c)) for name, typ, p, c, *_ in self.order}   # joint -> links it carries
        self.links = [self.base] + [c for _, _, _, c, *_ in self.order]

    def fk(self, Q: np.ndarray) -> Frames:
        """Q: (N, 6) joint angles j1..j6 (rad, motor angle = URDF angle). Prismatic/fixed joints sit at 0."""
        Q = np.atleast_2d(np.asarray(Q, dtype=float))
        n = len(Q)
        R = {self.base: np.broadcast_to(np.eye(3), (n, 3, 3)).copy()}
        t = {self.base: np.zeros((n, 3))}
        axis, point = {}, {}
        col = {URDF_JOINT[k]: i for i, k in enumerate(ARM)}
        for name, typ, p, c, Ro, xo, a in self.order:
            Rj = R[p] @ Ro
            tj = t[p] + R[p] @ xo
            if typ in ("revolute", "continuous"):
                axis[name] = Rj @ a
                point[name] = tj.copy()
                if name in col:
                    Rj = Rj @ _rodrigues(a, Q[:, col[name]])
            R[c], t[c] = Rj, tj
        return Frames(R, t, axis, point)

    def point_mass_columns(self, frames: Frames, link: str) -> np.ndarray:
        """(N, 6, 4): d G_j / d [m, m*rx, m*ry, m*rz] for a point mass on `link` (r in the link's frame)."""
        n = len(frames.t[self.base])
        out = np.zeros((n, 6, 4))
        for k, short in enumerate(ARM):
            jn = URDF_JOINT[short]
            if link not in self.distal[jn]:
                continue
            ga = np.cross(G, frames.axis[jn])                                     # g x a, (N,3)
            out[:, k, 0] = -np.einsum("ni,ni->n", frames.t[link] - frames.point[jn], ga)
            out[:, k, 1:] = -np.einsum("nji,nj->ni", frames.R[link], ga)          # R^T (g x a)
        return out

    def point_mass_torques(self, frames: Frames, link: str, xyz: Sequence[float], kg: float) -> np.ndarray:
        cols = self.point_mass_columns(frames, link)
        return kg * (cols[:, :, 0] + cols[:, :, 1:] @ np.asarray(xyz, dtype=float))

    def hold_torques(self, Q: np.ndarray, extra: Iterable = (), frames: Frames | None = None) -> np.ndarray:
        """(N, 6) torque each arm motor must apply to hold the arm still: the URDF links plus `extra` point masses
        [(link, (x, y, z), kg), ...], exactly gravity.GravityModel(extra=...).hold_torques."""
        frames = frames or self.fk(Q)
        out = 0.0
        for link, m in self.mass.items():
            out = out + self.point_mass_torques(frames, link, self.com[link], m)
        for link, xyz, kg in extra:
            out = out + self.point_mass_torques(frames, link, xyz, kg)
        return out


# ---- the gravity-model file ---------------------------------------------------------------------------------------

def read_model_text(text: str) -> dict:
    """The file is YAML written in JSON style (a '#' comment header, then a JSON object) so that the runner's
    environment, which has no PyYAML, reads it with json; any YAML reader reads it too."""
    import json
    body = "\n".join(line for line in text.splitlines() if not line.lstrip().startswith("#"))
    try:
        return json.loads(body)
    except ValueError:
        import yaml
        return yaml.safe_load(text)


def dump_model_text(doc: dict, header: str) -> str:
    """The inverse of read_model_text: header comment lines + indented JSON with no exponent floats (PyYAML 1.1
    would read 1e-05 as a string)."""
    import json

    def clean(o):
        if isinstance(o, float):
            return 0.0 if abs(o) < 1e-4 else o
        if isinstance(o, dict):
            return {k: clean(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [clean(v) for v in o]
        return o
    doc = clean(doc)

    def check(o):
        if isinstance(o, float) and ("e" in repr(o) or not math.isfinite(o)):
            raise ValueError(f"float {o!r} would not survive a YAML 1.1 reader")
        for v in (o.values() if isinstance(o, dict) else o if isinstance(o, list) else ()):
            check(v)
    check(doc)
    import re
    text = json.dumps(doc, indent=1)
    # innermost lists on one line (vectors, ranges), so the file reads like YAML flow sequences
    text = re.sub(r"\[\s+([^\[\]{}]*?)\s+\]", lambda m: "[" + " ".join(m.group(1).split()) + "]", text)
    return "".join(f"# {line}\n" if line else "#\n" for line in header.splitlines()) + text + "\n"


def load_model(path: Path | str = DEFAULT_MODEL, urdf: Path | str | None = None) -> dict:
    """Read and validate a crackvision.gravity_model/1 file; with `urdf`, refuse one fitted on a different URDF."""
    doc = read_model_text(Path(path).read_text())
    if not isinstance(doc, dict) or doc.get("schema") != SCHEMA:
        raise GravityModelError(f"{path}: schema {doc.get('schema') if isinstance(doc, dict) else None!r} != {SCHEMA!r}")
    want = (doc.get("urdf") or {}).get("sha256")
    if not isinstance(want, str) or len(want) != 64:
        raise GravityModelError(f"{path}: urdf.sha256 missing")
    if urdf is not None and file_sha256(urdf) != want:
        raise GravityModelError(f"{path}: fitted on URDF sha256 {want[:12]}..., but {urdf} is "
                                f"{file_sha256(urdf)[:12]}...")
    bodies = doc.get("bodies")
    if not isinstance(bodies, list):
        raise GravityModelError(f"{path}: bodies must be a list")
    for b in bodies:
        xyz, kg = b.get("xyz"), b.get("kg")
        if not (isinstance(b.get("link"), str) and isinstance(xyz, list) and len(xyz) == 3
                and all(isinstance(v, (int, float)) and math.isfinite(v) for v in xyz + [kg])):
            raise GravityModelError(f"{path}: bad body {b!r}")
        if not -0.5 <= kg <= 2.0 or max(abs(v) for v in xyz) > 1.0:
            raise GravityModelError(f"{path}: implausible body {b!r} (kg in [-0.5, 2], |xyz| <= 1 m)")
    if sum(float(b["kg"]) for b in bodies) > 5.0:
        raise GravityModelError(f"{path}: more than 5 kg of extra mass")
    scale = doc.get("scale") or {}
    for k, v in scale.items():
        if k not in ARM[1:] or not isinstance(v, (int, float)) or not 0.0 <= v <= 1.5:
            raise GravityModelError(f"{path}: bad scale {k}: {v!r}")
    return doc


def runner_problems(doc: dict, urdf: Path | str | None = None) -> list[str]:
    """Every reason ~/rebot_setup/demo_move.py's load_gravity_model would refuse this model document (a replica of its
    checks, 2026-10-29.3): schema; urdf.sha256 = the runner's URDF (DEFAULT_URDF, the same file); 1..20 bodies, each
    {link str, xyz 3 finite numbers with |xyz| <= 1 m, kg finite in [-0.5, 2]}, no duplicate (link, xyz), on a moving
    link of the URDF; total extra mass <= 5 kg; scale a non-empty map of some of j2..j6 to finite numbers in 0..1.5;
    the model evaluates to finite torques. [] = the runner would accept it."""
    out: list[str] = []
    num = lambda v: isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
    if not isinstance(doc, dict) or doc.get("schema") != SCHEMA:
        return [f"schema {doc.get('schema') if isinstance(doc, dict) else None!r} != {SCHEMA!r}"]
    urdf = Path(urdf) if urdf is not None else DEFAULT_URDF
    want = (doc.get("urdf") or {}).get("sha256")
    if want != file_sha256(urdf):
        out.append(f"urdf.sha256 {str(want)[:16]}... is not the runner's URDF {file_sha256(urdf)[:16]}...")
    bodies = doc.get("bodies")
    if not isinstance(bodies, list) or not 1 <= len(bodies) <= 20:
        return out + ["bodies must be a list of 1..20 point masses"]
    ug = UrdfGravity(urdf)
    moving = set().union(*ug.distal.values())
    seen, total, extra = set(), 0.0, []
    for b in bodies:
        link, xyz, kg = (b.get("link"), b.get("xyz"), b.get("kg")) if isinstance(b, dict) else (None, None, None)
        if not (isinstance(link, str) and isinstance(xyz, list) and len(xyz) == 3 and all(num(v) for v in xyz + [kg])):
            out.append(f"bad body {b!r}")
            continue
        if max(abs(v) for v in xyz) > 1.0 or not -0.5 <= kg <= 2.0:
            out.append(f"body {b!r}: |xyz| must be <= 1 m and kg in [-0.5, 2] (the runner refuses it)")
        key = (link, tuple(float(v) for v in xyz))
        if key in seen:
            out.append(f"duplicate body {b!r}")
        seen.add(key)
        if link not in moving:
            out.append(f"body on {link!r}, which is not a moving link of the URDF")
        total += float(kg)
        extra.append((link, key[1], float(kg)))
    if total > 5.0:
        out.append(f"{total:.2f} kg of extra mass > 5 kg")
    scale = doc.get("scale")
    if not isinstance(scale, dict) or not scale or any(k not in ARM[1:] or not num(v) or not 0.0 <= v <= 1.5
                                                       for k, v in scale.items()):
        out.append(f"scale must map some of j2..j6 to 0..1.5, got {scale!r}")
    if not out:
        g = ug.hold_torques(np.zeros((1, 6)), extra)
        if not np.all(np.isfinite(g)):
            out.append("the model does not evaluate")
    return out


def extra_bodies(doc: dict) -> list[tuple[str, tuple[float, float, float], float]]:
    return [(b["link"], tuple(float(v) for v in b["xyz"]), float(b["kg"])) for b in doc["bodies"]]


# ---- runs (demo_move.py CSV logs) ---------------------------------------------------------------------------------

def load_run(path: Path | str) -> dict:
    """A demo_move.py run log (t_s, mode, seg, then jN_ref/pos/vel/torq/ff; .csv or .csv.gz) as numpy columns;
    rows without feedback are dropped. Keys: t, mode, seg, ref/pos/vel/torq/ff (N, 7) arrays (j1..j7)."""
    import csv
    import gzip
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt", newline="") as fh:
        rows = [r for r in csv.DictReader(fh) if r.get("j2_pos")]
    if not rows or "j1_ref" not in rows[0]:
        raise ValueError(f"{path}: not a demo_move.py run log")
    out = {"t": np.array([float(r["t_s"]) for r in rows]), "mode": np.array([r["mode"] for r in rows]),
           "seg": np.array([int(r["seg"]) for r in rows]), "name": Path(path).name}
    for c in ("ref", "pos", "vel", "torq", "ff"):
        out[c] = np.array([[float(r[f"j{j}_{c}"]) for j in range(1, 8)] for r in rows])
    return out


def runner_friction(v_ref: np.ndarray) -> np.ndarray:
    """(N, 6) the runner's Coulomb feed-forward on the commanded velocity (demo_move.feed_forward)."""
    fc = np.array([RUNNER_FRICTION[k] for k in ARM])
    return fc * np.tanh(np.asarray(v_ref)[:, :6] / FRICTION_VEL_EPS)


FF_LIMIT_MARGIN = 0.05   # demo_move.py --gravity-model keeps the total feed-forward this far inside the cap


def model_feed_forward(grav: np.ndarray, fric: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """demo_move.py --gravity-model's composition: gravity + the runner's Coulomb term, the Coulomb term reduced
    where the sum would pass (cap - FF_LIMIT_MARGIN); gravity alone beyond that is refused (the runner's cap check
    and arm_guard's tau_ff_max stop it). Returns (ff (N,6), friction_limited (N,6) bool, refused (N,6) bool)."""
    cap = np.array([FF_CAP[k] for k in ARM]) - FF_LIMIT_MARGIN
    total = grav + fric
    over = np.abs(total) > cap
    refused = np.abs(grav) > cap
    limited = over & ~refused
    ff = np.where(limited, np.sign(total) * cap, total)
    return ff, limited, refused


def path_bias_profile(run: dict) -> np.ndarray:
    """(N,) demo_move.bias_scale over the run's 'path' rows (0 elsewhere): ramps over 1 s at each end."""
    t, path = run["t"], run["mode"] == "path"
    out = np.zeros(len(t))
    if path.any():
        t0, t1 = t[path][0], t[path][-1]
        tp = t - t0
        out = np.clip(np.minimum(tp / 1.0, (t1 - t0 - tp) / 1.0), 0.0, 1.0) * path
    return out

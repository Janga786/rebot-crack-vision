"""arm_guard.config — load, validate and hash-check the guard configuration (TOML, stdlib `tomllib`).

The config is the single source of the limits the guard enforces. It is only accepted if its SHA-256 (and the
URDF's) match `arm_guard.approved.json` next to it, the way industrial safety controllers refuse to run an
unapproved/changed safety configuration. `python -m arm_guard approve` (a human) re-approves after an edit.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10 (ROS Humble's system interpreter): same API
    import tomli as tomllib
from dataclasses import dataclass, field
from pathlib import Path

from .errors import GuardConfigError
from .kinematics import Chain, KinematicsError

REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = REPO_ROOT / "config" / "robot" / "arm_guard.toml"
CONFIG_ENV = "ARM_GUARD_CONFIG"


@dataclass(frozen=True)
class JointCfg:
    name: str
    motor_id: int
    feedback_id: int
    model: str
    urdf_joint: str          # "" for a joint outside the URDF (the gripper motor angle)
    arm: bool                # counts for the full-arm requirement and the keep-out FK
    lower: float
    upper: float
    vel_cap: float           # |commanded velocity| / vlim ceiling, rad/s
    vel_abort: float         # |measured velocity| above this = fault, rad/s
    accel_stop: float        # deceleration assumed for the stopping-distance speed bound, rad/s^2
    max_step: float          # |commanded position - measured position| ceiling, rad
    kp_max: float
    kd_max: float            # MIT kd above this is lowered to it (the MIT encoder clamps kd to 5 anyway)
    tau_ff_max: float        # |MIT feed-forward torque| ceiling, Nm
    torque_abort: float      # |measured torque| above this = fault, Nm
    hold_kp: float
    hold_kd: float
    beyond_urdf_reason: str = ""


@dataclass(frozen=True)
class FloorPoint:
    link: str
    z_min: float


@dataclass(frozen=True)
class Keepout:
    floor: tuple = ()                 # FloorPoint...
    cyl_radius: float = 0.0           # vertical cylinder around the base z axis
    cyl_z_max: float = 0.0
    cyl_links: tuple = ()
    seg_from: str = ""                # a link segment (e.g. the upper arm) the listed links must keep away from
    seg_to: str = ""
    seg_min_distance: float = 0.0
    seg_links: tuple = ()


@dataclass(frozen=True)
class GuardConfig:
    path: str
    sha256: str
    urdf_path: str
    urdf_sha256: str
    joints: dict                      # name -> JointCfg
    models: dict                      # model -> (pmax, vmax, tmax)
    keepout: Keepout
    chain: Chain = field(repr=False, compare=False, default=None)
    stale_feedback_s: float = 0.25
    rest_tolerance_rad: float = 0.05
    rest_slack_rad: float = 0.01
    require_full_arm: bool = True
    recent_command_s: float = 0.10
    hold_vlim: float = 0.2
    min_vlim: float = 0.05
    allow_velocity_mode: bool = False
    force_pos_ratio_max: float = 0.5
    torque_margin_nm: float = 0.5
    register_writes: dict = field(default_factory=dict)   # rid -> (lo, hi) allowed without config_writes()

    def joint_by_motor_id(self, motor_id: int):
        for j in self.joints.values():
            if j.motor_id == motor_id:
                return j
        return None

    @property
    def arm_joints(self) -> list:
        return [j for j in self.joints.values() if j.arm]


def sha256_file(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def config_path() -> Path:
    return Path(os.environ.get(CONFIG_ENV) or DEFAULT_CONFIG)


def approval_path(cfg_path: Path) -> Path:
    return cfg_path.with_name(cfg_path.stem + ".approved.json")


def _num(d: dict, key: str, where: str, positive: bool = False, nonneg: bool = False) -> float:
    if key not in d:
        raise GuardConfigError(f"{where}: missing '{key}'")
    v = d[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise GuardConfigError(f"{where}.{key}: not a finite number: {v!r}")
    if positive and not v > 0:
        raise GuardConfigError(f"{where}.{key}: must be > 0, got {v}")
    if nonneg and not v >= 0:
        raise GuardConfigError(f"{where}.{key}: must be >= 0, got {v}")
    return float(v)


def load(path=None, check_approval: bool = True) -> GuardConfig:
    """Parse + validate; raises GuardConfigError on anything wrong (never returns a partial config)."""
    p = Path(path) if path else config_path()
    try:
        raw_bytes = p.read_bytes()
        raw = tomllib.loads(raw_bytes.decode())
    except (OSError, UnicodeDecodeError, tomllib.TOMLDecodeError) as e:
        raise GuardConfigError(f"cannot read guard config {p}: {e}") from e
    g = raw.get("guard", {})
    urdf = (p.parent / g.get("urdf", "")).resolve()
    if not urdf.is_file():
        raise GuardConfigError(f"guard.urdf not found: {urdf}")
    urdf_sha = sha256_file(urdf)
    if g.get("urdf_sha256") != urdf_sha:
        raise GuardConfigError(f"guard.urdf_sha256 {g.get('urdf_sha256')} != actual {urdf_sha} ({urdf})")
    try:
        chain = Chain(str(urdf))
    except KinematicsError as e:
        raise GuardConfigError(str(e)) from e
    urdf_limits = _urdf_limits(str(urdf))

    models = {}
    for name, m in raw.get("models", {}).items():
        models[name] = tuple(_num(m, k, f"models.{name}", positive=True) for k in ("pmax", "vmax", "tmax"))

    joints = {}
    for name, d in raw.get("joints", {}).items():
        w = f"joints.{name}"
        model = d.get("model")
        if model not in models:
            raise GuardConfigError(f"{w}.model {model!r} not in [models]")
        jc = JointCfg(
            name=name, motor_id=int(_num(d, "motor_id", w, positive=True)),
            feedback_id=int(_num(d, "feedback_id", w, nonneg=True)), model=model,
            urdf_joint=str(d.get("urdf_joint", "")), arm=bool(d.get("arm", False)),
            lower=_num(d, "lower", w), upper=_num(d, "upper", w),
            vel_cap=_num(d, "vel_cap", w, positive=True), vel_abort=_num(d, "vel_abort", w, positive=True),
            accel_stop=_num(d, "accel_stop", w, positive=True), max_step=_num(d, "max_step", w, positive=True),
            kp_max=_num(d, "kp_max", w, nonneg=True), kd_max=_num(d, "kd_max", w, nonneg=True),
            tau_ff_max=_num(d, "tau_ff_max", w, nonneg=True), torque_abort=_num(d, "torque_abort", w, positive=True),
            hold_kp=_num(d, "hold_kp", w, nonneg=True), hold_kd=_num(d, "hold_kd", w, nonneg=True),
            beyond_urdf_reason=str(d.get("beyond_urdf_reason", "")),
        )
        if not jc.lower < jc.upper:
            raise GuardConfigError(f"{w}: lower {jc.lower} must be < upper {jc.upper}")
        if jc.vel_abort < jc.vel_cap:
            raise GuardConfigError(f"{w}: vel_abort {jc.vel_abort} must be >= vel_cap {jc.vel_cap}")
        if jc.hold_kp > jc.kp_max or jc.hold_kd > jc.kd_max:
            raise GuardConfigError(f"{w}: hold gains exceed kp_max/kd_max")
        pmax, vmax, tmax = models[model]
        if max(abs(jc.lower), abs(jc.upper)) >= pmax or jc.vel_cap >= vmax or jc.tau_ff_max >= tmax:
            raise GuardConfigError(f"{w}: limits must sit inside the {model} MIT mapping (PMAX/VMAX/TMAX)")
        if jc.urdf_joint:
            if jc.urdf_joint not in urdf_limits:
                raise GuardConfigError(f"{w}.urdf_joint {jc.urdf_joint!r} not a limited joint of the URDF")
            ulo, uhi = urdf_limits[jc.urdf_joint]
            if (jc.lower < ulo - 1e-9 or jc.upper > uhi + 1e-9) and not jc.beyond_urdf_reason:
                raise GuardConfigError(f"{w}: [{jc.lower}, {jc.upper}] wider than URDF [{ulo}, {uhi}] "
                                       f"without beyond_urdf_reason")
        if jc.arm and not jc.urdf_joint:
            raise GuardConfigError(f"{w}: an arm joint needs urdf_joint")
        joints[name] = jc
    if not joints:
        raise GuardConfigError("no [joints]")
    ids = [j.motor_id for j in joints.values()]
    if len(set(ids)) != len(ids):
        raise GuardConfigError("duplicate motor_id")
    arm_urdf = {j.urdf_joint for j in joints.values() if j.arm}
    movable = {n for n in chain.joint_names if n in urdf_limits and not n.startswith("gripper_joint")}
    if arm_urdf != movable:
        raise GuardConfigError(f"arm joints {sorted(arm_urdf)} != URDF arm joints {sorted(movable)}")

    k = raw.get("keepout", {})
    floor = tuple(FloorPoint(str(f["link"]), _num(f, "z_min", "keepout.floor")) for f in k.get("floor", []))
    cyl = k.get("base_cylinder", {})
    seg = k.get("segment", {})
    keepout = Keepout(
        floor=floor,
        cyl_radius=float(cyl.get("radius", 0.0)), cyl_z_max=float(cyl.get("z_max", 0.0)),
        cyl_links=tuple(cyl.get("links", ())),
        seg_from=str(seg.get("from_link", "")), seg_to=str(seg.get("to_link", "")),
        seg_min_distance=float(seg.get("min_distance", 0.0)), seg_links=tuple(seg.get("links", ())),
    )
    named = [f.link for f in floor] + list(keepout.cyl_links) + list(keepout.seg_links)
    named += [x for x in (keepout.seg_from, keepout.seg_to) if x]
    for link in named:
        if link not in chain.links:
            raise GuardConfigError(f"keepout link {link!r} not in URDF")

    regs = {}
    for rid, rng in raw.get("register_writes", {}).get("allowed", {}).items():
        if not (isinstance(rng, list) and len(rng) == 2):
            raise GuardConfigError(f"register_writes.allowed.{rid}: expected [lo, hi]")
        regs[int(rid)] = (float(rng[0]), float(rng[1]))

    cfg = GuardConfig(
        path=str(p), sha256=hashlib.sha256(raw_bytes).hexdigest(), urdf_path=str(urdf), urdf_sha256=urdf_sha,
        joints=joints, models=models, keepout=keepout, chain=chain,
        stale_feedback_s=_num(g, "stale_feedback_s", "guard", positive=True),
        rest_tolerance_rad=_num(g, "rest_tolerance_rad", "guard", nonneg=True),
        rest_slack_rad=_num(g, "rest_slack_rad", "guard", nonneg=True),
        require_full_arm=bool(g.get("require_full_arm", True)),
        recent_command_s=_num(g, "recent_command_s", "guard", positive=True),
        hold_vlim=_num(g, "hold_vlim_rad_s", "guard", positive=True),
        min_vlim=_num(g, "min_vlim_rad_s", "guard", positive=True),
        allow_velocity_mode=bool(g.get("allow_velocity_mode", False)),
        force_pos_ratio_max=_num(g, "force_pos_ratio_max", "guard", nonneg=True),
        torque_margin_nm=_num(g, "torque_margin_nm", "guard", nonneg=True),
        register_writes=regs,
    )
    if check_approval:
        check_approved(cfg)
    return cfg


def _urdf_limits(urdf: str) -> dict:
    import xml.etree.ElementTree as ET
    out = {}
    for j in ET.parse(urdf).getroot().findall("joint"):
        lim = j.find("limit")
        if lim is not None and lim.get("lower") is not None and lim.get("upper") is not None:
            out[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    return out


def check_approved(cfg: GuardConfig) -> dict:
    ap = approval_path(Path(cfg.path))
    try:
        rec = json.loads(ap.read_text())
    except (OSError, ValueError) as e:
        raise GuardConfigError(f"guard config is not approved ({ap}: {e}); run `python -m arm_guard approve`") from e
    if rec.get("config_sha256") != cfg.sha256 or rec.get("urdf_sha256") != cfg.urdf_sha256:
        raise GuardConfigError(
            f"guard config changed since approval ({ap}): config {cfg.sha256[:12]} vs approved "
            f"{str(rec.get('config_sha256'))[:12]}, urdf {cfg.urdf_sha256[:12]} vs {str(rec.get('urdf_sha256'))[:12]}; "
            f"review the diff, then `python -m arm_guard approve`")
    return rec

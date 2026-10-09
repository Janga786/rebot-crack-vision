"""execution_gate — the §11 offline commissioning-gated-execution gate (ADR-016, MOT-05.3).

Stdlib + PyYAML only -- no rclpy, control_msgs, moveit_msgs or sensor_msgs import anywhere in
this file, so it runs under the system python3 without a colcon build and no ROS graph, exactly
like `scene_core.py`/`end_effector.py`/`joint_trajectory.py`. It evaluates every *offline* §11.7
gate (`G-ARM`, `G-TRAJ`, `G-LIMITS-HASH`, `G-LIMITS`, `G-SPEED`, `G-SCENE`, `G-EE`,
`G-COMMISSIONING`, `G-ESTOP`, `G-ELIGIBLE`, `G-APPROVAL`, `G-STALE-CONFIG`) for a mode and returns
a `GateReport`. The online read-only gates (`G-GRAPH`, `G-START-STATE`, `G-COLLISION`) and the
interactive `G-CONFIRM` are not evaluated here -- they need a ROS graph / a controlling tty and
belong to MOT-05.4/.5/.6.

`GateCheck.outcome` carries all five §11.7 outcomes verbatim: `pass`, `fail`, `warn` (evaluated in
`mock`/`dry`, this mode's rule holds but `real`'s rule would fail; never refuses; never produced in
`real`), `skip` (the three real-only authorization/commissioning gates `G-ARM`, `G-COMMISSIONING`,
`G-ESTOP` outside `real`) and `error` (could not be evaluated: a precondition gate failed, or an
input file was missing/unreadable/malformed; refuses like `fail`). `GateReport.passed`/`.refusals`
treat `fail` and `error` as refusing and `warn`/`skip`/`pass` as non-refusing, matching §11.7's
refusal rule exactly. `mock`/`dry`'s "preview what `real` would refuse" (`real_preview`/
`would_refuse_in_real`, §11.9) is not reconstructable from this report; MOT-05.4 gets it for free by
calling `evaluate_offline(mode="real", ...)` a second time with the same inputs.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Union

import yaml

from crackvision_description.end_effector import (
    EndEffectorError,
    EndEffectorNotCalibratedError,
)
from crackvision_description.end_effector import assert_commissioning_ready as ee_assert_commissioning_ready
from crackvision_description.end_effector import load_config as ee_load_config

from crackvision_motion.joint_trajectory import (
    TrajectoryError,
    check_limits,
    file_sha256,
    load_limits,
    load_trajectory,
    scale_time,
)
from crackvision_motion.scene_core import SceneError, SceneNotCommissionedError
from crackvision_motion.scene_core import assert_commissioning_ready as scene_assert_commissioning_ready
from crackvision_motion.scene_core import load_config as scene_load_config

EXECUTION_CONFIG_SCHEMA = "crackvision.execution_config/1"
COMMISSIONING_SCHEMA = "crackvision.commissioning/1"
APPROVAL_SCHEMA = "crackvision.execution_approval/1"
PATHS3D_SCHEMA = "crackvision.paths3d/1"

MODES = ("mock", "dry", "real")

G_ARM = "G-ARM"
G_TRAJ = "G-TRAJ"
G_LIMITS_HASH = "G-LIMITS-HASH"
G_LIMITS = "G-LIMITS"
G_SPEED = "G-SPEED"
G_SCENE = "G-SCENE"
G_EE = "G-EE"
G_COMMISSIONING = "G-COMMISSIONING"
G_ESTOP = "G-ESTOP"
G_ELIGIBLE = "G-ELIGIBLE"
G_APPROVAL = "G-APPROVAL"
G_STALE_CONFIG = "G-STALE-CONFIG"

GATE_IDS = (
    G_ARM, G_TRAJ, G_LIMITS_HASH, G_LIMITS, G_SPEED, G_SCENE, G_EE,
    G_COMMISSIONING, G_ESTOP, G_ELIGIBLE, G_APPROVAL, G_STALE_CONFIG,
)

# Real-only authorization/commissioning gates: skip (never evaluated) outside real (§11.7).
_REAL_ONLY_GATES = frozenset({G_ARM, G_COMMISSIONING, G_ESTOP})

# §11.7's own category column: G-ARM and G-CONFIRM are "confirmation"; every other gate evaluated
# here (G-CONFIRM is not -- it needs a controlling tty, MOT-05.5/.6) is "offline".
_CONFIRMATION_GATES = frozenset({G_ARM})

OUTCOMES = ("pass", "fail", "warn", "skip", "error")
# §11.7 refusal rule: the run refuses iff at least one gate reports `fail` or `error`.
_REFUSING_OUTCOMES = frozenset({"fail", "error"})


class GateConfigError(ValueError):
    """Raised for a structurally or numerically invalid gate-input config file."""


@dataclass(frozen=True)
class GateCheck:
    id: str
    outcome: str  # "pass" | "fail" | "warn" | "skip" | "error"
    message: str

    @property
    def category(self) -> str:
        return "confirmation" if self.id in _CONFIRMATION_GATES else "offline"

    def to_dict(self) -> Dict[str, Any]:
        return {"gate": self.id, "category": self.category, "outcome": self.outcome, "detail": self.message}


@dataclass(frozen=True)
class GateReport:
    mode: str
    checks: List[GateCheck]
    offline_only: bool = True

    @property
    def passed(self) -> bool:
        return not any(c.outcome in _REFUSING_OUTCOMES for c in self.checks)

    @property
    def refusals(self) -> List[str]:
        return [c.id for c in self.checks if c.outcome in _REFUSING_OUTCOMES]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mode": self.mode,
            "offline_only": self.offline_only,
            "gate_report": [c.to_dict() for c in self.checks],
            "passed": self.passed,
            "refusals": self.refusals,
        }


# --------------------------------------------------------------------------------------
# strict config loaders
# --------------------------------------------------------------------------------------

def _load_yaml_mapping(path: Union[str, Path], ctx: str) -> Dict[str, Any]:
    path = Path(path)
    if not path.is_file():
        raise GateConfigError(f"{ctx} not found: {path}")
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise GateConfigError(f"{ctx} unreadable: {exc}") from exc
    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise GateConfigError(f"{ctx} is not valid YAML: {exc}") from exc
    if not isinstance(raw, dict):
        raise GateConfigError(f"{ctx} must be a mapping at the top level: {path}")
    return raw


def _require_keys(raw: Any, required: frozenset, optional: frozenset, ctx: str) -> None:
    if not isinstance(raw, dict):
        raise GateConfigError(f"{ctx} must be a mapping, got {type(raw).__name__}")
    allowed = required | optional
    unknown = set(raw) - allowed
    if unknown:
        raise GateConfigError(f"unknown key(s) in {ctx}: {sorted(unknown)}")
    missing = required - set(raw)
    if missing:
        raise GateConfigError(f"{ctx} is missing key(s): {sorted(missing)}")


_EXECUTION_CONFIG_REQUIRED = frozenset({
    "schema", "default_mode", "driver_profiles", "default_driver_profile", "speed_scale",
    "tolerances", "timeouts", "densify_step_m", "position_margin_rad", "approval_max_age_s",
    "estop_topics",
})
_DRIVER_PROFILE_KEYS = frozenset({"action", "joint_states_topic", "expected_node", "modes"})
_SPEED_SCALE_KEYS = frozenset({"default", "cap", "real_default"})
_TOLERANCES_KEYS = frozenset({"start_state_rad", "tracking_rad"})
_TIMEOUTS_KEYS = frozenset({"joint_state_s", "goal_s", "cancel_settle_s"})
_DRIVER_PROFILES_REQUIRED = frozenset({"moveit_mock", "vendor_mock", "vendor"})


def load_execution_config(path: Union[str, Path]) -> Dict[str, Any]:
    """Load and strictly validate `config/motion/execution.yaml` (`crackvision.execution_config/1`)."""
    raw = _load_yaml_mapping(path, "execution config")
    _require_keys(raw, _EXECUTION_CONFIG_REQUIRED, frozenset(), "<execution config root>")
    if raw["schema"] != EXECUTION_CONFIG_SCHEMA:
        raise GateConfigError(f"'schema' must be {EXECUTION_CONFIG_SCHEMA!r}, got {raw['schema']!r}")
    if raw["default_mode"] not in MODES:
        raise GateConfigError(f"'default_mode' must be one of {MODES}, got {raw['default_mode']!r}")

    profiles_raw = raw["driver_profiles"]
    if not isinstance(profiles_raw, dict) or set(profiles_raw) != _DRIVER_PROFILES_REQUIRED:
        raise GateConfigError(f"'driver_profiles' must have exactly keys {sorted(_DRIVER_PROFILES_REQUIRED)}")
    profiles: Dict[str, Any] = {}
    for name, p in profiles_raw.items():
        optional = {"arm_status_topic"} if name == "vendor" else set()
        _require_keys(p, _DRIVER_PROFILE_KEYS, frozenset(optional), f"driver_profiles.{name}")
        modes = p["modes"]
        if not isinstance(modes, list) or not modes or any(m not in MODES for m in modes):
            raise GateConfigError(f"'driver_profiles.{name}.modes' must be a non-empty list of {MODES}")
        profiles[name] = dict(p)
    if raw["default_driver_profile"] not in profiles:
        raise GateConfigError(
            f"'default_driver_profile' must be one of {sorted(profiles)}, got {raw['default_driver_profile']!r}"
        )

    speed_scale = raw["speed_scale"]
    _require_keys(speed_scale, _SPEED_SCALE_KEYS, frozenset(), "speed_scale")
    for key in _SPEED_SCALE_KEYS:
        v = speed_scale[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not (0.0 < float(v) <= 1.0):
            raise GateConfigError(f"'speed_scale.{key}' must be in (0, 1], got {v!r}")

    tolerances = raw["tolerances"]
    _require_keys(tolerances, _TOLERANCES_KEYS, frozenset(), "tolerances")
    timeouts = raw["timeouts"]
    _require_keys(timeouts, _TIMEOUTS_KEYS, frozenset(), "timeouts")

    for key in ("densify_step_m", "position_margin_rad"):
        v = raw[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or float(v) <= 0.0:
            raise GateConfigError(f"'{key}' must be a positive number, got {v!r}")
    age = raw["approval_max_age_s"]
    if isinstance(age, bool) or not isinstance(age, (int, float)) or float(age) <= 0.0:
        raise GateConfigError(f"'approval_max_age_s' must be a positive number, got {age!r}")

    estop_topics = raw["estop_topics"]
    if not isinstance(estop_topics, list) or not estop_topics or not all(isinstance(t, str) and t for t in estop_topics):
        raise GateConfigError("'estop_topics' must be a non-empty list of non-empty strings")

    return {
        "schema": raw["schema"],
        "default_mode": raw["default_mode"],
        "driver_profiles": profiles,
        "default_driver_profile": raw["default_driver_profile"],
        "speed_scale": {k: float(speed_scale[k]) for k in _SPEED_SCALE_KEYS},
        "tolerances": {k: float(tolerances[k]) for k in _TOLERANCES_KEYS},
        "timeouts": {k: float(timeouts[k]) for k in _TIMEOUTS_KEYS},
        "densify_step_m": float(raw["densify_step_m"]),
        "position_margin_rad": float(raw["position_margin_rad"]),
        "approval_max_age_s": float(age),
        "estop_topics": list(estop_topics),
    }


_COMMISSIONING_REQUIRED = frozenset({
    "schema", "commissioned", "limits_file_sha256", "speed_scale_cap", "estop",
    "joint_ranges_verified", "evidence", "source",
})
_ESTOP_KEYS = frozenset({"kind", "verified", "verified_utc", "operator", "evidence"})


def load_commissioning(path: Union[str, Path]) -> Dict[str, Any]:
    """Load and strictly validate `config/robot/commissioning.yaml` (`crackvision.commissioning/1`)."""
    raw = _load_yaml_mapping(path, "commissioning record")
    _require_keys(raw, _COMMISSIONING_REQUIRED, frozenset(), "<commissioning root>")
    if raw["schema"] != COMMISSIONING_SCHEMA:
        raise GateConfigError(f"'schema' must be {COMMISSIONING_SCHEMA!r}, got {raw['schema']!r}")
    if not isinstance(raw["commissioned"], bool):
        raise GateConfigError("'commissioned' must be a bool")
    sha = raw["limits_file_sha256"]
    if sha is not None and not isinstance(sha, str):
        raise GateConfigError("'limits_file_sha256' must be a string or null")
    cap = raw["speed_scale_cap"]
    if isinstance(cap, bool) or not isinstance(cap, (int, float)) or not (0.0 < float(cap) <= 1.0):
        raise GateConfigError(f"'speed_scale_cap' must be in (0, 1], got {cap!r}")

    estop = raw["estop"]
    _require_keys(estop, _ESTOP_KEYS, frozenset(), "estop")
    if estop["kind"] != "hardware":
        raise GateConfigError(f"'estop.kind' must be 'hardware', got {estop['kind']!r}")
    if not isinstance(estop["verified"], bool):
        raise GateConfigError("'estop.verified' must be a bool")
    if estop["verified_utc"] is not None and not isinstance(estop["verified_utc"], str):
        raise GateConfigError("'estop.verified_utc' must be a string or null")
    if estop["operator"] is not None and not isinstance(estop["operator"], str):
        raise GateConfigError("'estop.operator' must be a string or null")
    if not isinstance(estop["evidence"], list) or not all(isinstance(e, str) for e in estop["evidence"]):
        raise GateConfigError("'estop.evidence' must be a list of strings")

    if not isinstance(raw["joint_ranges_verified"], bool):
        raise GateConfigError("'joint_ranges_verified' must be a bool")
    if not isinstance(raw["evidence"], list) or not all(isinstance(e, str) for e in raw["evidence"]):
        raise GateConfigError("'evidence' must be a list of strings")
    if not isinstance(raw["source"], str):
        raise GateConfigError("'source' must be a string")

    return {
        "schema": raw["schema"],
        "commissioned": raw["commissioned"],
        "limits_file_sha256": sha,
        "speed_scale_cap": float(cap),
        "estop": dict(estop),
        "joint_ranges_verified": raw["joint_ranges_verified"],
        "evidence": list(raw["evidence"]),
        "source": raw["source"],
    }


_APPROVAL_REQUIRED = frozenset({"schema", "trajectory_sha256", "approved_by", "approved_utc", "preview_artifacts"})


def load_approval(path: Union[str, Path]) -> Dict[str, Any]:
    """Load and strictly validate a `crackvision.execution_approval/1` file (JSON)."""
    path = Path(path)
    if not path.is_file():
        raise GateConfigError(f"approval file not found: {path}")
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise GateConfigError(f"approval file unreadable: {exc}") from exc
    try:
        raw = json.loads(text)
    except json.JSONDecodeError as exc:
        raise GateConfigError(f"invalid JSON in approval file {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise GateConfigError("approval root must be a JSON object")
    _require_keys(raw, _APPROVAL_REQUIRED, frozenset(), "<approval root>")
    if raw["schema"] != APPROVAL_SCHEMA:
        raise GateConfigError(f"'schema' must be {APPROVAL_SCHEMA!r}, got {raw['schema']!r}")
    if not isinstance(raw["trajectory_sha256"], str) or not raw["trajectory_sha256"]:
        raise GateConfigError("'trajectory_sha256' must be a non-empty string")
    if not isinstance(raw["approved_by"], str) or not raw["approved_by"]:
        raise GateConfigError("'approved_by' must be a non-empty string")
    if not isinstance(raw["approved_utc"], str) or not raw["approved_utc"]:
        raise GateConfigError("'approved_utc' must be a non-empty string")
    artifacts = raw["preview_artifacts"]
    if not isinstance(artifacts, list):
        raise GateConfigError("'preview_artifacts' must be a list")
    for i, a in enumerate(artifacts):
        if not isinstance(a, dict) or set(a) != {"path", "sha256"}:
            raise GateConfigError(f"'preview_artifacts[{i}]' must be an object with exactly 'path','sha256'")

    return {
        "schema": raw["schema"],
        "trajectory_sha256": raw["trajectory_sha256"].lower(),
        "approved_by": raw["approved_by"],
        "approved_utc": raw["approved_utc"],
        "preview_artifacts": [dict(a) for a in artifacts],
    }


def _sha256_of(path: Union[str, Path]) -> str:
    return file_sha256(path)


def _resolve(root: Path, maybe_relative: str) -> Path:
    p = Path(maybe_relative)
    return p if p.is_absolute() else root / p


def _parse_utc(s: str) -> Optional[datetime]:
    try:
        return datetime.fromisoformat(s.replace("Z", "+00:00"))
    except ValueError:
        return None


# --------------------------------------------------------------------------------------
# confirmation helpers (§11.8)
# --------------------------------------------------------------------------------------

def confirmation_phrase(trajectory_sha256: str) -> str:
    return f"EXECUTE {trajectory_sha256[:8]}"


def confirmation_ok(typed: str, trajectory_sha256: str) -> bool:
    return typed.strip() == confirmation_phrase(trajectory_sha256)


# --------------------------------------------------------------------------------------
# evaluate_offline
# --------------------------------------------------------------------------------------

def evaluate_offline(
    mode: str,
    trajectory_path: Union[str, Path],
    *,
    root: Union[str, Path],
    execution_config_path: Union[str, Path],
    commissioning_path: Union[str, Path],
    limits_path: Union[str, Path],
    scene_config_path: Union[str, Path],
    end_effector_config_path: Union[str, Path],
    approval_path: Optional[Union[str, Path]] = None,
    speed_scale: Optional[float] = None,
    env: Mapping[str, str],
) -> GateReport:
    """Evaluate every offline §11.7 gate for `mode` and return a `GateReport`.

    `env` is an explicit mapping (never read from `os.environ` here) -- the caller decides what
    the process environment looks like, so `G-ARM` stays pure and testable.
    """
    if mode not in MODES:
        raise GateConfigError(f"mode must be one of {MODES}, got {mode!r}")

    root = Path(root)
    checks: List[GateCheck] = []

    # G-ARM: real-only, env CRACKVISION_ARM_REAL=1.
    if mode == "real":
        if env.get("CRACKVISION_ARM_REAL") == "1":
            checks.append(GateCheck(G_ARM, "pass", "CRACKVISION_ARM_REAL=1 set"))
        else:
            checks.append(GateCheck(G_ARM, "fail", "env CRACKVISION_ARM_REAL=1 not set"))
    else:
        checks.append(GateCheck(G_ARM, "skip", f"G-ARM is real-only, mode={mode}"))

    # G-TRAJ. `load_trajectory` can raise before its own validation even starts (the file is
    # missing/unreadable) -- that is "unreadable input" (§11.7 `error`), distinct from a file that
    # reads fine but fails schema validation (`TrajectoryError`, an evaluated `fail`).
    traj: Optional[dict] = None
    try:
        traj = load_trajectory(trajectory_path)
    except (OSError, UnicodeDecodeError) as exc:
        checks.append(GateCheck(G_TRAJ, "error", f"trajectory file unreadable: {exc}"))
    except TrajectoryError as exc:
        checks.append(GateCheck(G_TRAJ, "fail", str(exc)))
    else:
        if traj["purpose"] == "test":
            if mode == "real":
                checks.append(GateCheck(G_TRAJ, "fail", "purpose 'test' is refused in real mode"))
            else:
                checks.append(GateCheck(G_TRAJ, "warn", "schema valid; purpose 'test' only refuses in real"))
        else:
            checks.append(GateCheck(G_TRAJ, "pass", "schema valid"))

    # `traj_ok` means the file parsed successfully (downstream gates can read its data), not that
    # G-TRAJ reported `pass` -- a real-mode `purpose: "test"` refusal is a policy failure on a
    # structurally valid file, so every other gate still evaluates against its real data.
    traj_ok = traj is not None
    traj_sha: Optional[str] = None
    if traj is not None:
        try:
            traj_sha = file_sha256(trajectory_path)
        except OSError:
            traj_sha = None

    # G-LIMITS-HASH
    limits: Optional[Dict[str, Dict[str, float]]] = None
    current_limits_sha: Optional[str] = None
    try:
        current_limits_sha = _sha256_of(limits_path)
        limits = load_limits(limits_path)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        checks.append(GateCheck(G_LIMITS_HASH, "error", f"cannot read current limits file: {exc}"))
        checks.append(GateCheck(G_LIMITS, "error", "cannot evaluate: current limits file unreadable"))
    else:
        if not traj_ok:
            checks.append(GateCheck(G_LIMITS_HASH, "error", "cannot evaluate: G-TRAJ failed to load trajectory"))
        else:
            declared = traj["limits_file"]["sha256"]
            if declared is not None and declared != current_limits_sha:
                checks.append(GateCheck(
                    G_LIMITS_HASH, "fail",
                    f"trajectory's limits_file.sha256 {declared} does not match current {current_limits_sha}",
                ))
            elif declared is None:
                if mode == "real":
                    checks.append(GateCheck(G_LIMITS_HASH, "fail", "limits_file.sha256 is null; required in real"))
                else:
                    checks.append(GateCheck(G_LIMITS_HASH, "warn", "limits_file.sha256 is null (warn outside real)"))
            else:
                checks.append(GateCheck(G_LIMITS_HASH, "pass", "limits_file.sha256 matches current limits file"))

        # G-LIMITS
        if not traj_ok:
            checks.append(GateCheck(G_LIMITS, "error", "cannot evaluate: G-TRAJ failed to load trajectory"))
        else:
            try:
                exec_cfg_for_scale = load_execution_config(execution_config_path)
            except GateConfigError as exc:
                checks.append(GateCheck(G_LIMITS, "error", f"cannot evaluate: execution config unreadable: {exc}"))
            else:
                position_margin_rad = exec_cfg_for_scale["position_margin_rad"]
                violations = check_limits(traj, limits, position_margin_rad=position_margin_rad)
                if violations:
                    checks.append(GateCheck(
                        G_LIMITS, "fail",
                        f"{len(violations)} unscaled limit violation(s), e.g. {violations[0]}",
                    ))
                else:
                    effective_scale = _resolve_speed_scale(mode, speed_scale, exec_cfg_for_scale)
                    try:
                        scaled_traj = scale_time(traj, effective_scale)
                    except TrajectoryError as exc:
                        checks.append(GateCheck(
                            G_LIMITS, "error",
                            f"cannot evaluate: effective speed_scale {effective_scale!r} invalid: {exc}",
                        ))
                    else:
                        scaled_limits = {
                            j: {
                                "lower": spec["lower"],
                                "upper": spec["upper"],
                                "velocity": spec["velocity"] * effective_scale,
                                "acceleration": spec["acceleration"] * effective_scale * effective_scale,
                            }
                            for j, spec in limits.items()
                        }
                        scaled_violations = check_limits(
                            scaled_traj, scaled_limits, position_margin_rad=position_margin_rad
                        )
                        if scaled_violations:
                            checks.append(GateCheck(
                                G_LIMITS, "fail",
                                f"{len(scaled_violations)} scaled (s={effective_scale}) limit violation(s), "
                                f"e.g. {scaled_violations[0]}",
                            ))
                        else:
                            checks.append(GateCheck(
                                G_LIMITS, "pass", f"within limits unscaled and at s={effective_scale}"
                            ))

    # G-SPEED
    try:
        exec_cfg = load_execution_config(execution_config_path)
    except GateConfigError as exc:
        checks.append(GateCheck(G_SPEED, "error", f"cannot evaluate: execution config unreadable: {exc}"))
        exec_cfg = None
    else:
        effective_scale = _resolve_speed_scale(mode, speed_scale, exec_cfg)
        if mode == "real":
            try:
                commissioning_for_cap = load_commissioning(commissioning_path)
                cap = commissioning_for_cap["speed_scale_cap"]
            except GateConfigError as exc:
                checks.append(GateCheck(G_SPEED, "error", f"cannot read commissioning speed_scale_cap: {exc}"))
                cap = None

            if cap is not None:
                if 0.0 < effective_scale <= cap:
                    checks.append(GateCheck(
                        G_SPEED, "pass", f"effective speed_scale {effective_scale} within (0, {cap}]"
                    ))
                else:
                    checks.append(GateCheck(
                        G_SPEED, "fail",
                        f"effective speed_scale {effective_scale} outside (0, {cap}] for mode {mode}",
                    ))
        else:
            # mock/dry: the primary refusing cap is always execution.yaml.speed_scale.cap. `dry`
            # additionally previews `real`'s commissioning-cap rule as `warn` (§11.7 G-SPEED row);
            # `mock` never consults the commissioning record at all.
            cap = exec_cfg["speed_scale"]["cap"]
            if not (0.0 < effective_scale <= cap):
                checks.append(GateCheck(
                    G_SPEED, "fail",
                    f"effective speed_scale {effective_scale} outside (0, {cap}] for mode {mode}",
                ))
            elif mode == "dry":
                try:
                    commissioning_for_cap = load_commissioning(commissioning_path)
                    real_cap = commissioning_for_cap["speed_scale_cap"]
                except GateConfigError as exc:
                    checks.append(GateCheck(
                        G_SPEED, "warn",
                        f"within (0, {cap}]; commissioning record unreadable, real cap unknown "
                        f"(warn outside real): {exc}",
                    ))
                else:
                    if effective_scale > real_cap:
                        checks.append(GateCheck(
                            G_SPEED, "warn",
                            f"within (0, {cap}] but exceeds commissioning speed_scale_cap {real_cap} "
                            f"(warn outside real; real would refuse)",
                        ))
                    else:
                        checks.append(GateCheck(
                            G_SPEED, "pass", f"effective speed_scale {effective_scale} within (0, {cap}]"
                        ))
            else:
                checks.append(GateCheck(G_SPEED, "pass", f"effective speed_scale {effective_scale} within (0, {cap}]"))

    # G-SCENE. `scene_core.load_config` only raises `SceneError` for a file it could read and
    # parse but that fails schema validation (an evaluated `fail`); a missing file or unparseable
    # YAML is "unreadable input" (§11.7 `error`).
    try:
        scene_cfg = scene_load_config(scene_config_path)
    except SceneError as exc:
        checks.append(GateCheck(G_SCENE, "fail", str(exc)))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        checks.append(GateCheck(G_SCENE, "error", f"scene config unreadable: {exc}"))
    else:
        try:
            scene_assert_commissioning_ready(scene_cfg)
        except SceneNotCommissionedError as exc:
            if mode == "real":
                checks.append(GateCheck(G_SCENE, "fail", str(exc)))
            else:
                checks.append(GateCheck(G_SCENE, "warn", f"loads; not commissioning-ready (warn in {mode}): {exc}"))
        else:
            checks.append(GateCheck(G_SCENE, "pass", "scene config loads and is commissioning-ready"))

    # G-EE. Same unreadable-input/evaluated-fail split as G-SCENE above.
    try:
        ee_cfg = ee_load_config(end_effector_config_path)
    except EndEffectorError as exc:
        checks.append(GateCheck(G_EE, "fail", str(exc)))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        checks.append(GateCheck(G_EE, "error", f"end-effector config unreadable: {exc}"))
    else:
        try:
            ee_assert_commissioning_ready(ee_cfg)
        except EndEffectorNotCalibratedError as exc:
            if mode == "real":
                checks.append(GateCheck(G_EE, "fail", str(exc)))
            else:
                checks.append(GateCheck(G_EE, "warn", f"loads; not commissioning-ready (warn in {mode}): {exc}"))
        else:
            checks.append(GateCheck(G_EE, "pass", "end-effector config loads and is commissioning-ready"))

    # G-COMMISSIONING / G-ESTOP: real-only.
    if mode != "real":
        checks.append(GateCheck(G_COMMISSIONING, "skip", f"G-COMMISSIONING is real-only, mode={mode}"))
        checks.append(GateCheck(G_ESTOP, "skip", f"G-ESTOP is real-only, mode={mode}"))
    else:
        try:
            commissioning = load_commissioning(commissioning_path)
        except GateConfigError as exc:
            checks.append(GateCheck(G_COMMISSIONING, "error", str(exc)))
            checks.append(GateCheck(G_ESTOP, "error", f"cannot evaluate: commissioning record unreadable: {exc}"))
        else:
            if commissioning["commissioned"] and commissioning["limits_file_sha256"] == current_limits_sha:
                checks.append(GateCheck(G_COMMISSIONING, "pass", "commissioned and limits_file_sha256 matches"))
            else:
                checks.append(GateCheck(
                    G_COMMISSIONING, "fail",
                    f"commissioned={commissioning['commissioned']}, "
                    f"limits_file_sha256 match={commissioning['limits_file_sha256'] == current_limits_sha}",
                ))
            estop = commissioning["estop"]
            if estop["kind"] == "hardware" and estop["verified"]:
                checks.append(GateCheck(G_ESTOP, "pass", "hardware e-stop verified"))
            else:
                checks.append(GateCheck(
                    G_ESTOP, "fail", f"estop.kind={estop['kind']!r}, estop.verified={estop['verified']!r}",
                ))

    # G-ELIGIBLE
    if not traj_ok:
        checks.append(GateCheck(G_ELIGIBLE, "error", "cannot evaluate: G-TRAJ failed to load trajectory"))
    else:
        checks.append(_evaluate_eligible(traj, root, mode))

    # G-APPROVAL
    if not traj_ok or traj_sha is None:
        checks.append(GateCheck(G_APPROVAL, "error", "cannot evaluate: trajectory unavailable"))
    else:
        checks.append(_evaluate_approval(approval_path, traj_sha, execution_config_path, mode))

    # G-STALE-CONFIG
    if not traj_ok:
        checks.append(GateCheck(G_STALE_CONFIG, "error", "cannot evaluate: G-TRAJ failed to load trajectory"))
    else:
        checks.append(_evaluate_stale_config(traj, scene_config_path, end_effector_config_path, mode))

    ordered = sorted(checks, key=lambda c: GATE_IDS.index(c.id))
    return GateReport(mode=mode, checks=ordered)


def _resolve_speed_scale(mode: str, speed_scale: Optional[float], exec_cfg: Dict[str, Any]) -> float:
    if speed_scale is not None:
        return float(speed_scale)
    if mode == "mock":
        return exec_cfg["speed_scale"]["default"]
    return exec_cfg["speed_scale"]["real_default"]


def _eligible_violation(mode: str, msg: str) -> GateCheck:
    if mode == "real":
        return GateCheck(G_ELIGIBLE, "fail", msg)
    return GateCheck(G_ELIGIBLE, "warn", f"{msg} (warn outside real)")


def _evaluate_eligible(traj: dict, root: Path, mode: str) -> GateCheck:
    source = traj["source"]["paths3d"]
    if source is None:
        if traj["purpose"] == "crack_task":
            return _eligible_violation(mode, "purpose crack_task with source.paths3d: null")
        return GateCheck(G_ELIGIBLE, "pass", "purpose 'test' with no paths3d binding: nothing to check")

    p3d_path = _resolve(root, source["path"])
    if not p3d_path.is_file():
        return GateCheck(G_ELIGIBLE, "fail", f"source.paths3d.path does not exist: {p3d_path}")
    try:
        actual_sha = file_sha256(p3d_path)
    except OSError as exc:
        return GateCheck(G_ELIGIBLE, "error", f"source.paths3d.path unreadable: {exc}")
    if actual_sha != source["sha256"]:
        return GateCheck(
            G_ELIGIBLE, "fail",
            f"source.paths3d.sha256 {source['sha256']} does not match file's actual sha256 {actual_sha}",
        )
    try:
        p3d_text = p3d_path.read_text(encoding="utf-8")
    except OSError as exc:
        return GateCheck(G_ELIGIBLE, "error", f"source.paths3d.path unreadable: {exc}")
    except UnicodeDecodeError as exc:
        # sha matched, so this is the bound file -- it is just not a paths3d document (integrity fail).
        return GateCheck(G_ELIGIBLE, "fail", f"source.paths3d.path is not UTF-8 text, not a {PATHS3D_SCHEMA!r} file: {exc}")
    try:
        p3d = json.loads(p3d_text)
    except json.JSONDecodeError as exc:
        return GateCheck(G_ELIGIBLE, "fail", f"source.paths3d.path does not parse as JSON: {exc}")
    if not isinstance(p3d, dict) or p3d.get("schema") != PATHS3D_SCHEMA:
        return GateCheck(G_ELIGIBLE, "fail", f"source.paths3d.path is not a {PATHS3D_SCHEMA!r} file")

    file_eligible = p3d.get("execution_eligible")
    reasons = p3d.get("ineligible_reasons", [])
    declared_eligible = source["execution_eligible"]

    if declared_eligible != file_eligible:
        return _eligible_violation(
            mode, f"trajectory's execution_eligible copy ({declared_eligible}) disagrees with the file ({file_eligible})"
        )
    if not file_eligible:
        return _eligible_violation(mode, f"paths3d file reports execution_eligible=false, reasons={reasons}")
    return GateCheck(G_ELIGIBLE, "pass", "paths3d sha matches and is execution_eligible")


def _evaluate_approval(
    approval_path: Optional[Union[str, Path]], traj_sha: str, execution_config_path: Union[str, Path], mode: str
) -> GateCheck:
    violation_outcome = "fail" if mode == "real" else "warn"
    suffix = "" if mode == "real" else " (warn outside real)"

    if approval_path is None:
        return GateCheck(G_APPROVAL, violation_outcome, "no approval file given" + suffix)
    try:
        approval = load_approval(approval_path)
        exec_cfg = load_execution_config(execution_config_path)
    except GateConfigError as exc:
        return GateCheck(G_APPROVAL, violation_outcome, f"approval unusable: {exc}" + suffix)

    if approval["trajectory_sha256"] != traj_sha:
        return GateCheck(
            G_APPROVAL, violation_outcome,
            f"approval.trajectory_sha256 {approval['trajectory_sha256']} does not match trajectory {traj_sha}" + suffix,
        )

    approved_at = _parse_utc(approval["approved_utc"])
    if approved_at is None:
        return GateCheck(
            G_APPROVAL, violation_outcome, "approval.approved_utc is not a parseable ISO-8601 timestamp" + suffix
        )
    if approved_at.tzinfo is None:
        approved_at = approved_at.replace(tzinfo=timezone.utc)
    age_s = (datetime.now(timezone.utc) - approved_at).total_seconds()
    if age_s > exec_cfg["approval_max_age_s"]:
        return GateCheck(
            G_APPROVAL, violation_outcome,
            f"approval is {age_s:.0f}s old, exceeds approval_max_age_s={exec_cfg['approval_max_age_s']}" + suffix,
        )
    return GateCheck(G_APPROVAL, "pass", "approval binds the current trajectory and is fresh")


def _evaluate_stale_config(
    traj: dict, scene_config_path: Union[str, Path], end_effector_config_path: Union[str, Path], mode: str
) -> GateCheck:
    try:
        current_scene_sha = _sha256_of(scene_config_path)
        current_ee_sha = _sha256_of(end_effector_config_path)
    except OSError as exc:
        return GateCheck(G_STALE_CONFIG, "error", f"cannot read current scene/end-effector config: {exc}")

    declared_scene = traj["scene_config_sha256"]
    declared_ee = traj["end_effector_config_sha256"]

    mismatches = []
    if declared_scene is not None and declared_scene != current_scene_sha:
        mismatches.append(f"scene_config_sha256 {declared_scene} != current {current_scene_sha}")
    if declared_ee is not None and declared_ee != current_ee_sha:
        mismatches.append(f"end_effector_config_sha256 {declared_ee} != current {current_ee_sha}")
    if mismatches:
        return GateCheck(G_STALE_CONFIG, "fail", "; ".join(mismatches))

    nulls = []
    if declared_scene is None:
        nulls.append("scene_config_sha256")
    if declared_ee is None:
        nulls.append("end_effector_config_sha256")
    if nulls:
        if mode == "real":
            return GateCheck(G_STALE_CONFIG, "fail", f"{', '.join(nulls)} is null; required in real")
        return GateCheck(G_STALE_CONFIG, "warn", f"{', '.join(nulls)} is null (warn outside real)")
    return GateCheck(G_STALE_CONFIG, "pass", "scene/end-effector config shas match current files")

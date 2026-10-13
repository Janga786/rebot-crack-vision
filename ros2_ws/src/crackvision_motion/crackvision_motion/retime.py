"""crackvision_motion.retime — §16 constant-tool-speed retimer + §11.2/§16.4 document assembly
(MOT-07.4).

Stdlib + numpy + PyYAML only -- no rclpy or `*_msgs` import anywhere in this file, mirroring
`retime_core.py`/`tool_fk.py`/`joint_trajectory.py`/`cartesian_plan.py`, so it runs standalone
under the system python3 without a colcon build. Normative source: `docs/INTERFACES.md` §16 and
`docs/adr/019-time-parameterization.md`.

`retime_plan_doc` is the whole producer: it loads `retime.yaml`, checks the limits/end-effector
sha256s the plan itself recorded (§16.3) against the files actually read here, runs
`retime_core.plan_polyline`/`refine` (§16.2 Steps 1-2), bisects each trace leg's constant tool
speed (§16.2 Step 4, using `tool_fk` for the chord/tool-speed geometry and `retime_core.time_profile`
for the per-trial joint-space profile), runs the full-limit G-SELFCHECK (§16.2 Step 5) and
assembles the §11.2 trajectory document and §16.4 report (§16.2 Step 6, §16.3, §16.4). It never
clamps and never applies a hidden uniform slowdown: any guarantee it cannot verify directly is a
typed `RetimeError` refusal, and no file is written until the whole document has already passed
self-check and schema validation (`write_outputs` is a separate step precisely so a refusal never
leaves a partial trajectory/report pair on disk).
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
import yaml

from .joint_trajectory import (
    JOINT_NAMES,
    SCHEMA as TRAJECTORY_SCHEMA,
    TrajectoryError,
    check_limits,
    file_sha256,
    load_limits,
    load_trajectory,
    max_abs_velocity,
)
from .retime_core import (
    RetimeConfigError,
    RetimeError as _RetimeCoreError,
    load_retime_config,
    plan_polyline,
    refine,
    time_profile,
)
from .tool_fk import ToolFKError, load_tool_chain

REPORT_SCHEMA = "crackvision.retime_report/1"
PRODUCER = "retime_plan"

_N_JOINTS = len(JOINT_NAMES)
_SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")

# Bisection for the per-trace-leg constant tool speed v_c (§16.2 Step 4). Bounded iterations,
# matching ADR-019's "never a partially-converged, silently-non-compliant result" convention: a
# bounded search that never finds a feasible v_c is a refusal, not a best-effort guess.
_BISECT_TOL_M_S = 1e-4
_BISECT_MAX_ITERS = 40
_CHORD_EPS_M = 1e-9
_SPEED_EPS = 1e-9


class RetimeError(ValueError):
    """Raised for any refusal in `retime_plan_doc`/`validate_retime_report`: a limits/end-effector
    sha mismatch against `plan.sources`, a trace leg whose bisected `v_c` falls below
    `trace.min_tool_speed_m_s`, a failed G-SELFCHECK, or an assembled document that fails its own
    schema validation."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _canonical_json_bytes(doc: Dict[str, Any]) -> bytes:
    """The exact byte sequence written by `write_outputs` -- used both to write a file and to hash
    the content that will be written, so `trajectory_sha256` in the report is never computed from a
    different serialization than the one actually on disk."""
    return json.dumps(doc, indent=2, sort_keys=False, ensure_ascii=False).encode("utf-8")


def _atomic_write_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(dir=str(path.parent), prefix=f".{path.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "wb") as fh:
            fh.write(data)
        os.replace(tmp_name, path)
    except BaseException:
        if os.path.exists(tmp_name):
            os.remove(tmp_name)
        raise


def _load_position_margin_rad(execution_config_path: Union[str, Path]) -> float:
    """The one `execution.yaml` field this module needs (§16.1): read directly, without pulling in
    the full `execution_gate.load_execution_config` (and its `crackvision_description`/
    `crackvision_motion.scene_core` import chain) for a single float."""
    path = Path(execution_config_path)
    try:
        raw = yaml.safe_load(path.read_text())
    except OSError as exc:
        raise RetimeError(f"could not read execution config '{path}': {exc}") from exc
    if not isinstance(raw, dict) or raw.get("schema") != "crackvision.execution_config/1":
        raise RetimeError(f"'{path}' is not a valid crackvision.execution_config/1 file")
    margin = raw.get("position_margin_rad")
    if isinstance(margin, bool) or not isinstance(margin, (int, float)) or float(margin) <= 0:
        raise RetimeError(f"'position_margin_rad' in '{path}' must be a positive number, got {margin!r}")
    return float(margin)


def _max_abs_acceleration(traj: Dict[str, Any]) -> List[float]:
    """Per-joint max |finite-difference acceleration| over the whole trajectory, the same centred
    formula `joint_trajectory.check_limits` uses -- not exported there, so reimplemented here for
    the report's `totals.peak_acceleration_fraction` (§16.4)."""
    points = traj["points"]
    n = len(points)
    t_s = [p["t_s"] for p in points]
    result: List[float] = []
    for jidx in range(_N_JOINTS):
        positions = [p["positions"][jidx] for p in points]
        seg_v = [(positions[i + 1] - positions[i]) / (t_s[i + 1] - t_s[i]) for i in range(n - 1)]
        amax = 0.0
        for i in range(1, n - 1):
            dt = (t_s[i + 1] - t_s[i - 1]) / 2.0
            a = (seg_v[i] - seg_v[i - 1]) / dt
            amax = max(amax, abs(a))
        result.append(amax)
    return result


# --------------------------------------------------------------------------------------
# G-TOOLSPEED (§16.2 Step 4)
# --------------------------------------------------------------------------------------

def _leg_range(spans, leg_index: int) -> Tuple[int, int]:
    for li, _kind, first, last in spans:
        if li == leg_index:
            return first, last
    raise RetimeError(f"leg_index {leg_index} missing from the profiled polyline's own spans")


def _leg_tool_speeds(chain, configs, t_s, first: int, last: int) -> Tuple[np.ndarray, np.ndarray]:
    """Per-segment tool speed `s_i` and tool-tip chord length for leg range `[first, last]`
    (inclusive, §16.4's shared-boundary convention): `len(configs[first:last+1]) - 1` segments."""
    if last <= first:
        return np.array([]), np.array([])
    pts = chain.tool_positions(np.asarray(configs[first:last + 1]))
    chords = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    tau = np.diff(np.asarray(t_s[first:last + 1], dtype=float))
    return chords / tau, chords


def _check_toolspeed(
    speeds: np.ndarray, chords: np.ndarray, v_c: float, tol: float, max_ramp_m: float,
) -> Tuple[bool, Optional[float], Optional[float], Optional[float], Optional[float]]:
    """G-TOOLSPEED rules 1-3 (§16.2 Step 4) for one trace leg's segments at a candidate `v_c`.
    Returns `(ok, ramp_in_m, ramp_out_m, min_speed, max_speed)`; the last four are `None` when
    `ok` is `False`."""
    n = len(speeds)
    if n == 0:
        return True, 0.0, 0.0, 0.0, 0.0

    upper = v_c * (1.0 + tol)
    if np.any(speeds > upper + _SPEED_EPS):
        return False, None, None, None, None

    lower = v_c * (1.0 - tol)
    below = speeds < lower - _SPEED_EPS

    lead = 0
    while lead < n and below[lead]:
        lead += 1
    if lead == n:
        # Rule 2's "whole leg is one ramp": counts as both leading and trailing.
        total = float(np.sum(chords))
        if total > max_ramp_m + 1e-12:
            return False, None, None, None, None
        return True, total, total, float(np.min(speeds)), float(np.max(speeds))

    trail = 0
    while trail < n - lead and below[n - 1 - trail]:
        trail += 1
    middle = below[lead:n - trail]
    if np.any(middle):
        return False, None, None, None, None

    ramp_in = float(np.sum(chords[:lead])) if lead else 0.0
    ramp_out = float(np.sum(chords[n - trail:])) if trail else 0.0
    if ramp_in > max_ramp_m + 1e-12 or ramp_out > max_ramp_m + 1e-12:
        return False, None, None, None, None

    return True, ramp_in, ramp_out, float(np.min(speeds)), float(np.max(speeds))


def _self_validate_trajectory(doc: Dict[str, Any]) -> None:
    """Round-trip `doc` through the real `joint_trajectory.load_trajectory` schema validator
    (§11.2), exactly as any downstream consumer would read it back -- never a hand-rolled
    approximation of that schema."""
    payload = _canonical_json_bytes(doc)
    fd, tmp_name = tempfile.mkstemp(suffix=".json")
    try:
        with os.fdopen(fd, "wb") as fh:
            fh.write(payload)
        load_trajectory(tmp_name)
    except TrajectoryError as exc:
        raise RetimeError(f"assembled trajectory failed §11.2 schema validation: {exc}") from exc
    finally:
        try:
            os.remove(tmp_name)
        except OSError:
            pass


# --------------------------------------------------------------------------------------
# retime_plan_doc (§16.2-16.4)
# --------------------------------------------------------------------------------------

def retime_plan_doc(
    plan: Dict[str, Any],
    *,
    limits_path: Union[str, Path],
    end_effector_path: Union[str, Path],
    urdf_path: Union[str, Path],
    retime_config_path: Union[str, Path],
    execution_config_path: Union[str, Path],
    purpose: str,
    plan_path: Union[str, Path],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """Retime an already-validated `crackvision.cartesian_plan/1` document (`plan`, as returned by
    `cartesian_plan.load_plan`) into a `(trajectory_doc, report_doc)` pair (§11.2/§16.4). Raises
    `RetimeError` for any refusal condition in §16.2/§16.3 -- never a partially-converged or
    silently-non-compliant result. Writes nothing; see `write_outputs`."""
    # Step 1 (§16.3): the plan and the retimer must agree on which limits/end-effector files were
    # in force.
    limits_sha = file_sha256(limits_path)
    plan_limits_sha = plan["sources"]["limits"]["sha256"]
    if limits_sha != plan_limits_sha:
        raise RetimeError(
            f"limits file sha256 mismatch: '{limits_path}' hashes to {limits_sha}, but "
            f"plan.sources.limits.sha256 is {plan_limits_sha}"
        )

    end_effector_sha = file_sha256(end_effector_path)
    plan_ee_sha = plan["sources"]["end_effector"]["sha256"]
    if end_effector_sha != plan_ee_sha:
        raise RetimeError(
            f"end_effector config sha256 mismatch: '{end_effector_path}' hashes to "
            f"{end_effector_sha}, but plan.sources.end_effector.sha256 is {plan_ee_sha}"
        )

    try:
        retime_cfg = load_retime_config(retime_config_path)
    except RetimeConfigError as exc:
        raise RetimeError(f"invalid retime config '{retime_config_path}': {exc}") from exc

    limits = load_limits(limits_path)

    try:
        chain = load_tool_chain(urdf_path, end_effector_path)
    except ToolFKError as exc:
        raise RetimeError(f"failed to build the tool_fk chain: {exc}") from exc

    margin_rad = _load_position_margin_rad(execution_config_path)

    f_v = retime_cfg["limit_fraction"]["velocity"]
    f_a = retime_cfg["limit_fraction"]["acceleration"]
    max_joint_step_rad = retime_cfg["sampling"]["max_joint_step_rad"]
    max_period_s = retime_cfg["sampling"]["max_period_s"]
    target_v_c = retime_cfg["trace"]["tool_speed_m_s"]
    min_v_c = retime_cfg["trace"]["min_tool_speed_m_s"]
    tol = retime_cfg["trace"]["speed_tolerance"]
    max_ramp_m = retime_cfg["trace"]["max_ramp_m"]

    # Step 2 (§16.2 Steps 1-2): plan polyline + refinement.
    try:
        configs, spans = plan_polyline(plan)
        refined, spans = refine(configs, spans, max_joint_step_rad)
    except _RetimeCoreError as exc:
        raise RetimeError(str(exc)) from exc

    # Step 3-4 setup: the base refined polyline's own joint-space step and tool-tip chord per
    # segment, used by every bisection trial's external cap (never recomputed from a trial's own,
    # possibly-subdivided, output mesh).
    base_h = np.linalg.norm(np.diff(refined, axis=0), axis=1)
    tool_pts = chain.tool_positions(refined)
    base_chords = np.linalg.norm(np.diff(tool_pts, axis=0), axis=1)

    trace_v_c: Dict[int, float] = {
        li: target_v_c for (li, kind, _first, _last) in spans if kind == "trace"
    }

    def build_caps() -> np.ndarray:
        caps = np.full(len(base_h), math.inf)
        for li, kind, first, last in spans:
            if kind != "trace":
                continue
            v_c = trace_v_c[li]
            for i in range(first, last):
                chord = base_chords[i]
                if chord > _CHORD_EPS_M:
                    caps[i] = v_c * base_h[i] / chord
        return caps

    def run_profile(caps: np.ndarray):
        try:
            return time_profile(
                refined, spans, limits, f_v=f_v, f_a=f_a, margin_rad=margin_rad,
                max_period_s=max_period_s, caps=caps.tolist(),
            )
        except _RetimeCoreError:
            return None

    def trial(leg_index: int, v_c: float):
        saved = trace_v_c[leg_index]
        trace_v_c[leg_index] = v_c
        caps = build_caps()
        trace_v_c[leg_index] = saved

        result = run_profile(caps)
        if result is None:
            return None
        out_configs, out_t_s, out_spans = result
        out_first, out_last = _leg_range(out_spans, leg_index)
        speeds, chords = _leg_tool_speeds(chain, out_configs, out_t_s, out_first, out_last)
        ok, ramp_in, ramp_out, smin, smax = _check_toolspeed(speeds, chords, v_c, tol, max_ramp_m)
        if not ok:
            return None
        return ramp_in, ramp_out, smin, smax

    for leg_index, kind, first, last in spans:
        if kind != "trace" or first == last:
            continue

        if trial(leg_index, target_v_c) is not None:
            trace_v_c[leg_index] = target_v_c
            continue

        if trial(leg_index, min_v_c) is None:
            raise RetimeError(
                f"trace leg {leg_index}: no constant tool speed in "
                f"[{min_v_c}, {target_v_c}] m/s satisfies G-TOOLSPEED"
            )

        lo, hi = min_v_c, target_v_c
        for _ in range(_BISECT_MAX_ITERS):
            if hi - lo <= _BISECT_TOL_M_S:
                break
            mid = (lo + hi) / 2.0
            if trial(leg_index, mid) is not None:
                lo = mid
            else:
                hi = mid
        trace_v_c[leg_index] = lo

    # One final profile with every trace leg's chosen v_c simultaneously.
    final_result = run_profile(build_caps())
    if final_result is None:
        raise RetimeError("the final combined profile (every trace leg at its chosen v_c) did not converge")
    final_configs, final_t_s, final_spans = final_result

    leg_stats: Dict[int, Tuple[float, float, float, float]] = {}
    for leg_index, kind, first, last in final_spans:
        if kind != "trace":
            continue
        speeds, chords = _leg_tool_speeds(chain, final_configs, final_t_s, first, last)
        if len(speeds) == 0:
            leg_stats[leg_index] = (0.0, 0.0, 0.0, 0.0)
            continue
        ok, ramp_in, ramp_out, smin, smax = _check_toolspeed(
            speeds, chords, trace_v_c[leg_index], tol, max_ramp_m
        )
        if not ok:
            raise RetimeError(
                f"trace leg {leg_index}: G-TOOLSPEED no longer holds once every trace leg's own "
                "v_c is applied simultaneously"
            )
        leg_stats[leg_index] = (ramp_in, ramp_out, smin, smax)

    final_points = [
        {"t_s": float(t), "positions": row.tolist()} for t, row in zip(final_t_s, final_configs)
    ]

    trajectory_doc: Dict[str, Any] = {
        "schema": TRAJECTORY_SCHEMA,
        "created_utc": _utc_now(),
        "producer": PRODUCER,
        "purpose": purpose,
        "planning_frame": "base_link",
        "joint_names": list(JOINT_NAMES),
        "points": final_points,
        "limits_file": {"path": str(limits_path), "sha256": limits_sha},
        "end_effector_config_sha256": plan_ee_sha,
        "scene_config_sha256": plan["sources"]["scene"]["sha256"],
        "source": {"paths3d": dict(plan["sources"]["paths3d"])},
    }

    # Step 5 (G-SELFCHECK): full, unscaled limits, no explicit velocities/accelerations (G8), so
    # only the finite-difference pass of the §11.6 margin rule is live -- a precondition for
    # writing anything.
    self_check_violations = check_limits(trajectory_doc, limits, position_margin_rad=margin_rad)
    if self_check_violations:
        raise RetimeError(
            f"G-SELFCHECK failed: {len(self_check_violations)} violation(s) at full limits "
            f"(first: {self_check_violations[0]})"
        )

    _self_validate_trajectory(trajectory_doc)

    # Step 6/7 (§16.3/§16.4): assemble the report.
    trajectory_sha256 = hashlib.sha256(_canonical_json_bytes(trajectory_doc)).hexdigest()

    parameters = dict(retime_cfg)
    parameters["position_margin_rad"] = margin_rad

    legs_report: List[Dict[str, Any]] = []
    for leg_index, kind, first, last in final_spans:
        entry: Dict[str, Any] = {
            "leg_index": leg_index,
            "kind": kind,
            "point_range": [first, last],
            "duration_s": float(final_t_s[last] - final_t_s[first]),
        }
        if kind == "trace":
            ramp_in, ramp_out, smin, smax = leg_stats[leg_index]
            entry["tool_speed"] = {
                "target": target_v_c,
                "constant": trace_v_c[leg_index],
                "ramp_in_m": ramp_in,
                "ramp_out_m": ramp_out,
                "min": smin,
                "max": smax,
            }
        legs_report.append(entry)

    v_fraction = [v / limits[jn]["velocity"] for v, jn in zip(max_abs_velocity(trajectory_doc), JOINT_NAMES)]
    a_fraction = [
        a / limits[jn]["acceleration"] for a, jn in zip(_max_abs_acceleration(trajectory_doc), JOINT_NAMES)
    ]

    report_doc: Dict[str, Any] = {
        "schema": REPORT_SCHEMA,
        "created_utc": _utc_now(),
        "trajectory_sha256": trajectory_sha256,
        "input_shas": {
            "plan": file_sha256(plan_path),
            "limits": limits_sha,
            "end_effector": end_effector_sha,
            "urdf": chain.urdf_sha256,
            "retime_config": file_sha256(retime_config_path),
            "execution_config": file_sha256(execution_config_path),
        },
        "parameters": parameters,
        "legs": legs_report,
        "totals": {
            "n_points": len(final_configs),
            "duration_s": float(final_t_s[-1]),
            "peak_velocity_fraction": v_fraction,
            "peak_acceleration_fraction": a_fraction,
        },
        "self_check": {"violations": 0},
    }

    validate_retime_report(report_doc)
    return trajectory_doc, report_doc


# --------------------------------------------------------------------------------------
# validate_retime_report (§16.4, strict loader)
# --------------------------------------------------------------------------------------

_REPORT_TOP_KEYS = frozenset({
    "schema", "created_utc", "trajectory_sha256", "input_shas", "parameters", "legs", "totals", "self_check",
})
_INPUT_SHA_KEYS = frozenset({"plan", "limits", "end_effector", "urdf", "retime_config", "execution_config"})
_PARAMETER_KEYS = frozenset({"schema", "limit_fraction", "sampling", "trace", "value_status", "sources", "position_margin_rad"})
_LEG_REPORT_BASE_KEYS = frozenset({"leg_index", "kind", "point_range", "duration_s"})
_TOOL_SPEED_KEYS = frozenset({"target", "constant", "ramp_in_m", "ramp_out_m", "min", "max"})
_TOTALS_KEYS = frozenset({"n_points", "duration_s", "peak_velocity_fraction", "peak_acceleration_fraction"})
_LEG_KINDS = frozenset({"transit", "approach", "trace", "retract"})


def _check_sha(v: object, ctx: str) -> None:
    if not isinstance(v, str) or not _SHA256_RE.match(v):
        raise RetimeError(f"'{ctx}' must be a 64-hex-char sha256 digest, got {v!r}")


def _check_number(v: object, ctx: str, *, allow_negative: bool = False) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise RetimeError(f"'{ctx}' must be numeric, got {v!r}")
    fv = float(v)
    if not math.isfinite(fv):
        raise RetimeError(f"'{ctx}' must be finite, got {v!r}")
    if not allow_negative and fv < 0:
        raise RetimeError(f"'{ctx}' must be >= 0, got {v!r}")
    return fv


def validate_retime_report(doc: object) -> Dict[str, Any]:
    """Strictly validate a `crackvision.retime_report/1` document (§16.4). Raises `RetimeError` on
    any unknown/missing key, wrong type, or value outside its stated shape."""
    if not isinstance(doc, dict):
        raise RetimeError("retime report root must be an object")
    unknown = set(doc) - _REPORT_TOP_KEYS
    missing = _REPORT_TOP_KEYS - set(doc)
    if unknown or missing:
        raise RetimeError(f"retime report: unknown key(s) {sorted(unknown)}, missing key(s) {sorted(missing)}")

    if doc["schema"] != REPORT_SCHEMA:
        raise RetimeError(f"'schema' must be {REPORT_SCHEMA!r}, got {doc['schema']!r}")
    if not isinstance(doc["created_utc"], str) or not doc["created_utc"]:
        raise RetimeError("'created_utc' must be a non-empty string")
    _check_sha(doc["trajectory_sha256"], "trajectory_sha256")

    input_shas = doc["input_shas"]
    if not isinstance(input_shas, dict) or set(input_shas) != _INPUT_SHA_KEYS:
        raise RetimeError(f"'input_shas' must have exactly keys {sorted(_INPUT_SHA_KEYS)}")
    for k, v in input_shas.items():
        _check_sha(v, f"input_shas.{k}")

    parameters = doc["parameters"]
    if not isinstance(parameters, dict) or set(parameters) != _PARAMETER_KEYS:
        raise RetimeError(f"'parameters' must have exactly keys {sorted(_PARAMETER_KEYS)}")
    _check_number(parameters["position_margin_rad"], "parameters.position_margin_rad")
    if parameters["position_margin_rad"] <= 0:
        raise RetimeError("'parameters.position_margin_rad' must be > 0")

    legs_raw = doc["legs"]
    if not isinstance(legs_raw, list) or not legs_raw:
        raise RetimeError("'legs' must be a non-empty list")
    for i, leg in enumerate(legs_raw):
        ctx = f"legs[{i}]"
        if not isinstance(leg, dict):
            raise RetimeError(f"'{ctx}' must be an object")
        kind = leg.get("kind")
        allowed_keys = _LEG_REPORT_BASE_KEYS | ({"tool_speed"} if kind == "trace" else frozenset())
        if set(leg) != allowed_keys:
            raise RetimeError(f"'{ctx}' must have exactly keys {sorted(allowed_keys)}, got {sorted(leg)}")

        leg_index = leg["leg_index"]
        if isinstance(leg_index, bool) or not isinstance(leg_index, int) or leg_index < 0:
            raise RetimeError(f"'{ctx}.leg_index' must be a non-negative int")
        if kind not in _LEG_KINDS:
            raise RetimeError(f"'{ctx}.kind' must be one of {sorted(_LEG_KINDS)}, got {kind!r}")

        point_range = leg["point_range"]
        if (
            not isinstance(point_range, list) or len(point_range) != 2
            or any(isinstance(x, bool) or not isinstance(x, int) for x in point_range)
            or point_range[0] < 0 or point_range[1] < point_range[0]
        ):
            raise RetimeError(f"'{ctx}.point_range' must be [first, last] non-negative ints with first <= last")

        _check_number(leg["duration_s"], f"{ctx}.duration_s")

        if kind == "trace":
            ts = leg["tool_speed"]
            if not isinstance(ts, dict) or set(ts) != _TOOL_SPEED_KEYS:
                raise RetimeError(f"'{ctx}.tool_speed' must have exactly keys {sorted(_TOOL_SPEED_KEYS)}")
            for key in _TOOL_SPEED_KEYS:
                _check_number(ts[key], f"{ctx}.tool_speed.{key}")
            if ts["constant"] <= 0:
                raise RetimeError(f"'{ctx}.tool_speed.constant' must be > 0")

    totals = doc["totals"]
    if not isinstance(totals, dict) or set(totals) != _TOTALS_KEYS:
        raise RetimeError(f"'totals' must have exactly keys {sorted(_TOTALS_KEYS)}")
    n_points = totals["n_points"]
    if isinstance(n_points, bool) or not isinstance(n_points, int) or n_points < 2:
        raise RetimeError("'totals.n_points' must be an int >= 2")
    _check_number(totals["duration_s"], "totals.duration_s")
    for key in ("peak_velocity_fraction", "peak_acceleration_fraction"):
        vec = totals[key]
        if not isinstance(vec, list) or len(vec) != _N_JOINTS:
            raise RetimeError(f"'totals.{key}' must be a list of {_N_JOINTS} numbers")
        for j, v in enumerate(vec):
            _check_number(v, f"totals.{key}[{j}]")

    self_check = doc["self_check"]
    if not isinstance(self_check, dict) or set(self_check) != {"violations"} or self_check["violations"] != 0:
        raise RetimeError("'self_check' must be exactly {'violations': 0}")

    return doc


# --------------------------------------------------------------------------------------
# write_outputs
# --------------------------------------------------------------------------------------

def write_outputs(traj: Dict[str, Any], report: Dict[str, Any], out_path: Union[str, Path]) -> None:
    """Write `traj` to `out_path` and `report` to `<out_path>.retime.json` (§16.4), each atomically
    (temp file + `os.replace`). Never called on a refusal -- `retime_plan_doc` raises before
    returning anything to write."""
    out_path = Path(out_path)
    _atomic_write_bytes(out_path, _canonical_json_bytes(traj))
    _atomic_write_bytes(Path(str(out_path) + ".retime.json"), _canonical_json_bytes(report))

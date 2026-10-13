"""Unit tests for retime (MOT-07.4): constant tool speed on trace legs + §11.2 trajectory /
§16.4 retime_report assembly with self-check.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_retime.py -q -p no:cacheprovider

Synthetic plans are built with `cartesian_plan`'s own incremental builders (`new_plan`/`add_leg`/
`add_segment_report`), never a bare hand-rolled dict, and validated with `cartesian_plan.
validate_plan` before being handed to `retime_plan_doc` -- exactly how a real `plan_crack_path`
output would reach the retimer. `limits`/`end_effector`/`urdf` are the real production files (their
sha256 is always computed at test time, never hardcoded, mirroring `test_retime_core.py`/
`test_tool_fk.py`); `retime.yaml`/`execution.yaml` are synthetic, test-local fixtures so trace-leg
tool-speed parameters can be tuned for a fast, deterministic test run.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
import tempfile
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import pytest
import yaml

from crackvision_motion.cartesian_plan import (
    CONTINUITY_TOL_RAD,
    add_leg,
    add_segment_report,
    build_caveats,
    new_plan,
    validate_plan,
)
from crackvision_motion.joint_trajectory import JOINT_NAMES, check_limits, file_sha256, load_limits, load_trajectory
from crackvision_motion.retime import (
    REPORT_SCHEMA,
    RetimeError,
    retime_plan_doc,
    validate_retime_report,
    write_outputs,
)
from crackvision_motion.tool_fk import load_tool_chain

REPO_ROOT = Path(__file__).resolve().parents[4]
LIMITS_PATH = REPO_ROOT / "config" / "robot" / "b601_dm_limits.yaml"
END_EFFECTOR_PATH = REPO_ROOT / "config" / "robot" / "end_effector.yaml"
URDF_PATH = REPO_ROOT / "presentation" / "sim" / "reBot_B601_DM_with_gripper.urdf"

LIMITS = load_limits(LIMITS_PATH)
SEG_REF = {
    "component_id": "c0", "polyline_kind": "main", "branch_index": None, "segment_index": 0, "sub_segment": None,
}

# Joint configs chosen (and their resulting tool_tip geometry read back via `tool_fk`, not
# asserted independently) to give a trace leg with one straight joint-space segment and one with
# a sharp joint-space corner -- i.e. the Cartesian trace shape is whatever `tool_fk` says it is for
# these configs, never a Cartesian path invented first and assumed to match.
Q_HOME = [0.0] * 6
Q_APPROACH_START = [0.0, -0.9, -0.6, 0.3, 0.3, 0.0]
Q_TRACE_FIRST = [0.0, -1.0, -0.7, 0.35, 0.3, 0.0]
Q_MID_CURVED = [0.03, -1.0, -0.7, 0.35, 0.3, 0.0]
Q_END_CURVED = [0.03, -1.0, -0.65, 0.4, 0.3, 0.0]
Q_RETRACT_END = Q_APPROACH_START

STRAIGHT_TRACE = [Q_TRACE_FIRST, Q_END_CURVED]
CURVED_TRACE = [Q_TRACE_FIRST, Q_MID_CURVED, Q_END_CURVED]


def _recheck(positions: List[List[float]]) -> Dict[str, Any]:
    arr = np.array(positions)
    step = float(np.max(np.abs(np.diff(arr, axis=0)))) if len(positions) > 1 else 0.0
    return {
        "max_pos_err_m": 0.0, "max_ang_err_rad": 0.0, "all_valid": True, "within_limits": True,
        "max_joint_step_rad": step,
    }


def _make_sources(tmp_path: Path) -> Dict[str, Any]:
    paths3d = tmp_path / "paths3d.json"
    paths3d.write_text(json.dumps({"dummy": "paths3d"}))
    scene = tmp_path / "scene.yaml"
    scene.write_text("dummy: scene\n")
    config = tmp_path / "config.yaml"
    config.write_text("dummy: config\n")
    return {
        "paths3d": {"path": str(paths3d), "sha256": file_sha256(paths3d), "execution_eligible": True},
        "scene": {"path": str(scene), "sha256": file_sha256(scene)},
        "end_effector": {"path": str(END_EFFECTOR_PATH), "sha256": file_sha256(END_EFFECTOR_PATH)},
        "limits": {"path": str(LIMITS_PATH), "sha256": file_sha256(LIMITS_PATH)},
        "config": {"path": str(config), "sha256": file_sha256(config)},
    }


def _build_plan(tmp_path: Path, trace_positions: List[List[float]], label: str):
    sources = _make_sources(tmp_path)
    plan = new_plan(
        f"synthetic_{label}", sources, {"note": "synthetic MOT-07.4 fixture"},
        {"joint_names": list(JOINT_NAMES), "positions_rad": Q_HOME, "source": "start_joints_override"},
        {"mode": "none", "positions_rad": None},
    )
    add_leg(plan, "transit", SEG_REF, "forward", 0.0, [Q_HOME, Q_APPROACH_START], [None, None],
            _recheck([Q_HOME, Q_APPROACH_START]))
    add_leg(plan, "approach", SEG_REF, "forward", 0.0, [Q_APPROACH_START, Q_TRACE_FIRST], [None, 0],
            _recheck([Q_APPROACH_START, Q_TRACE_FIRST]))
    n = len(trace_positions)
    wmap = [0] + [None] * (n - 2) + [n - 1]
    add_leg(plan, "trace", SEG_REF, "forward", 0.0, trace_positions, wmap, _recheck(trace_positions))
    trace_last = trace_positions[-1]
    add_leg(plan, "retract", SEG_REF, "forward", 0.0, [trace_last, Q_RETRACT_END], [0, None],
            _recheck([trace_last, Q_RETRACT_END]))

    add_segment_report(plan, SEG_REF, [], 1, {"direction": "forward", "roll_offset_rad": 0.0}, n, n, 0.03, 0.03)
    plan["coverage"] = {
        "trace_waypoints_total": n, "trace_waypoints_planned": n,
        "length_m_total": 0.03, "length_m_planned": 0.03,
        "fraction_by_length": 1.0, "complete": True,
        "upstream": {"points": n, "valid": n, "invalid_by_reason": {}},
    }
    plan["caveats"] = build_caveats("tool_tip; synthetic test fixture (MOT-07.4)", execution_eligible=True)
    plan = validate_plan(plan, LIMITS)

    plan_path = tmp_path / f"plan_{label}.json"
    plan_path.write_text(json.dumps(plan))
    return plan, plan_path


def _write_retime_config(
    tmp_path: Path, *, target=0.05, min_speed=0.005, tol=0.15, max_ramp=0.01, max_step=0.05, max_period=1.0,
    name="retime.yaml",
) -> Path:
    raw = {
        "schema": "crackvision.retime_config/1",
        "limit_fraction": {"velocity": 0.95, "acceleration": 0.95},
        "sampling": {"max_joint_step_rad": max_step, "max_period_s": max_period},
        "trace": {
            "tool_speed_m_s": target, "min_tool_speed_m_s": min_speed, "speed_tolerance": tol,
            "max_ramp_m": max_ramp,
        },
        "value_status": "nominal",
        "sources": {"limit_fraction": "x", "sampling": "x", "trace": "x"},
    }
    path = tmp_path / name
    path.write_text(yaml.safe_dump(raw))
    return path


def _write_execution_config(tmp_path: Path, *, margin=0.03, name="execution.yaml") -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump({"schema": "crackvision.execution_config/1", "position_margin_rad": margin}))
    return path


def _flip_sha(sha: str) -> str:
    first = "0" if sha[0] != "0" else "1"
    return first + sha[1:]


def _recompute_peak_fractions(traj: Dict[str, Any], limits: Dict[str, Any]) -> tuple:
    """Independent re-derivation of `totals.peak_velocity_fraction`/`peak_acceleration_fraction`
    (§16.4) straight from the trajectory's own `points[]` and `b601_dm_limits.yaml`, via the same
    finite-difference formulas §16.4 specifies -- never by reading the report's own numbers."""
    points = traj["points"]
    t_s = [p["t_s"] for p in points]
    n = len(points)
    v_fracs: List[float] = []
    a_fracs: List[float] = []
    for jidx, jn in enumerate(JOINT_NAMES):
        positions = [p["positions"][jidx] for p in points]
        seg_v = [(positions[i + 1] - positions[i]) / (t_s[i + 1] - t_s[i]) for i in range(n - 1)]
        vmax = max(abs(v) for v in seg_v)
        amax = 0.0
        for i in range(1, n - 1):
            dt = (t_s[i + 1] - t_s[i - 1]) / 2.0
            a = (seg_v[i] - seg_v[i - 1]) / dt
            amax = max(amax, abs(a))
        v_fracs.append(vmax / limits[jn]["velocity"])
        a_fracs.append(amax / limits[jn]["acceleration"])
    return v_fracs, a_fracs


def _recompute_ramp_m(chain, configs: np.ndarray, t_s: np.ndarray, v_c: float, tol: float) -> tuple:
    """Independent re-derivation of one trace leg's `ramp_in_m`/`ramp_out_m` (§16.2 Step 4: the
    leading/trailing arc length of segments whose tool speed falls below `v_c*(1-tol)`), via
    `tool_fk` directly on the trajectory slice -- never by reading the report's own numbers."""
    pts = chain.tool_positions(configs)
    chords = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    speeds = chords / np.diff(t_s)
    n = len(speeds)
    if n == 0:
        return 0.0, 0.0
    below = speeds < v_c * (1.0 - tol) - 1e-9
    lead = 0
    while lead < n and below[lead]:
        lead += 1
    if lead == n:
        total = float(np.sum(chords))
        return total, total
    trail = 0
    while trail < n - lead and below[n - 1 - trail]:
        trail += 1
    ramp_in = float(np.sum(chords[:lead])) if lead else 0.0
    ramp_out = float(np.sum(chords[n - trail:])) if trail else 0.0
    return ramp_in, ramp_out


def _assert_report_matches_trajectory(
    plan: Dict[str, Any], report: Dict[str, Any], traj: Dict[str, Any], retime_cfg_path: Path,
) -> None:
    # Every leg's reported point_range boundary points are bit-exact with that leg's own first/
    # last plan configuration (§16.4's shared-boundary convention).
    for entry in report["legs"]:
        leg = next(l for l in plan["legs"] if l["leg_index"] == entry["leg_index"])
        first, last = entry["point_range"]
        assert traj["points"][first]["positions"] == leg["positions"][0]
        assert traj["points"][last]["positions"] == leg["positions"][-1]

    v_fracs, a_fracs = _recompute_peak_fractions(traj, LIMITS)
    assert report["totals"]["peak_velocity_fraction"] == pytest.approx(v_fracs, abs=1e-9)
    assert report["totals"]["peak_acceleration_fraction"] == pytest.approx(a_fracs, abs=1e-9)

    retime_raw = yaml.safe_load(Path(retime_cfg_path).read_text())
    tol = retime_raw["trace"]["speed_tolerance"]
    chain = load_tool_chain(URDF_PATH, END_EFFECTOR_PATH)
    for entry in report["legs"]:
        if entry["kind"] != "trace":
            continue
        first, last = entry["point_range"]
        configs = np.array([p["positions"] for p in traj["points"][first:last + 1]])
        t_s = np.array([p["t_s"] for p in traj["points"][first:last + 1]])
        ramp_in, ramp_out = _recompute_ramp_m(chain, configs, t_s, entry["tool_speed"]["constant"], tol)
        assert entry["tool_speed"]["ramp_in_m"] == pytest.approx(ramp_in, abs=1e-9)
        assert entry["tool_speed"]["ramp_out_m"] == pytest.approx(ramp_out, abs=1e-9)


def _assert_plan_positions_bit_exact_in_order(plan: Dict[str, Any], trajectory_doc: Dict[str, Any]) -> None:
    """Mirrors `retime_core.plan_polyline`'s own §16.2 Step 1 dedup: a leg's first position is not
    checked again when it is within `CONTINUITY_TOL_RAD` of the previous leg's last (the plan's
    own shared-boundary convention), since only one physical trajectory point exists there."""
    traj_positions = [p["positions"] for p in trajectory_doc["points"]]
    cursor = 0
    prev_last = None
    for leg in plan["legs"]:
        for i, pos in enumerate(leg["positions"]):
            if i == 0 and prev_last is not None and max(abs(a - b) for a, b in zip(prev_last, pos)) <= CONTINUITY_TOL_RAD:
                continue
            while cursor < len(traj_positions) and traj_positions[cursor] != pos:
                cursor += 1
            assert cursor < len(traj_positions), f"leg {leg['leg_index']} position {pos} missing from trajectory"
            cursor += 1
        prev_last = leg["positions"][-1]


# --------------------------------------------------------------------------------------
# retime_plan_doc -- happy path
# --------------------------------------------------------------------------------------

def test_retime_plan_doc_straight_trace_happy_path(tmp_path):
    plan, plan_path = _build_plan(tmp_path, STRAIGHT_TRACE, "straight")
    retime_cfg_path = _write_retime_config(tmp_path)
    exec_cfg_path = _write_execution_config(tmp_path)

    traj, report = retime_plan_doc(
        plan, limits_path=LIMITS_PATH, end_effector_path=END_EFFECTOR_PATH, urdf_path=URDF_PATH,
        retime_config_path=retime_cfg_path, execution_config_path=exec_cfg_path,
        purpose="test", plan_path=plan_path,
    )

    # §11.2 schema + G8 (no explicit kinematics).
    fd, tmp_name = tempfile.mkstemp(suffix=".json")
    with os.fdopen(fd, "w") as fh:
        json.dump(traj, fh)
    load_trajectory(tmp_name)
    os.remove(tmp_name)
    for p in traj["points"]:
        assert set(p) == {"t_s", "positions"}

    # §16.3 field mapping, copied verbatim from plan.sources.
    assert traj["producer"] == "retime_plan"
    assert traj["purpose"] == "test"
    assert traj["limits_file"]["sha256"] == file_sha256(LIMITS_PATH)
    assert traj["end_effector_config_sha256"] == plan["sources"]["end_effector"]["sha256"]
    assert traj["scene_config_sha256"] == plan["sources"]["scene"]["sha256"]
    assert traj["source"]["paths3d"] == plan["sources"]["paths3d"]

    # G-PATH (this card's own restatement): every plan configuration appears bit-exact, in order.
    _assert_plan_positions_bit_exact_in_order(plan, traj)

    # G-SELFCHECK, independently re-run.
    assert check_limits(traj, LIMITS, position_margin_rad=0.03) == []

    # §16.4 report validates and matches the trajectory.
    validate_retime_report(report)
    assert report["schema"] == REPORT_SCHEMA
    expected_traj_sha = hashlib.sha256(
        json.dumps(traj, indent=2, sort_keys=False, ensure_ascii=False).encode("utf-8")
    ).hexdigest()
    assert report["trajectory_sha256"] == expected_traj_sha
    assert report["input_shas"] == {
        "plan": file_sha256(plan_path),
        "limits": file_sha256(LIMITS_PATH),
        "end_effector": file_sha256(END_EFFECTOR_PATH),
        "urdf": file_sha256(URDF_PATH),
        "retime_config": file_sha256(retime_cfg_path),
        "execution_config": file_sha256(exec_cfg_path),
    }
    assert report["totals"]["n_points"] == len(traj["points"])
    assert report["totals"]["duration_s"] == pytest.approx(traj["points"][-1]["t_s"])
    assert report["self_check"] == {"violations": 0}

    for entry in report["legs"]:
        first, last = entry["point_range"]
        expected_duration = traj["points"][last]["t_s"] - traj["points"][first]["t_s"]
        assert entry["duration_s"] == pytest.approx(expected_duration, abs=1e-9)
    trace_entry = next(e for e in report["legs"] if e["kind"] == "trace")
    assert trace_entry["tool_speed"]["constant"] > 0
    assert trace_entry["tool_speed"]["constant"] <= trace_entry["tool_speed"]["target"] + 1e-9

    _assert_report_matches_trajectory(plan, report, traj, retime_cfg_path)


def test_retime_plan_doc_curved_trace_has_ramp_and_toolspeed_holds(tmp_path):
    plan, plan_path = _build_plan(tmp_path, CURVED_TRACE, "curved")
    retime_cfg_path = _write_retime_config(tmp_path)
    exec_cfg_path = _write_execution_config(tmp_path)

    traj, report = retime_plan_doc(
        plan, limits_path=LIMITS_PATH, end_effector_path=END_EFFECTOR_PATH, urdf_path=URDF_PATH,
        retime_config_path=retime_cfg_path, execution_config_path=exec_cfg_path,
        purpose="test", plan_path=plan_path,
    )
    validate_retime_report(report)
    assert check_limits(traj, LIMITS, position_margin_rad=0.03) == []
    _assert_plan_positions_bit_exact_in_order(plan, traj)

    trace_entry = next(e for e in report["legs"] if e["kind"] == "trace")
    ts = trace_entry["tool_speed"]
    tol = 0.15
    v_c = ts["constant"]
    assert 0.0 < v_c <= ts["target"] + 1e-9
    # The joint-space corner induces a real leading ramp that this card's bisection must detect
    # and bound, not just a trivially-constant leg.
    assert ts["ramp_in_m"] > 0.0
    assert ts["ramp_in_m"] <= 0.01 + 1e-12
    assert ts["ramp_out_m"] <= 0.01 + 1e-12

    # Independent cross-check of the reported min/max/band using tool_fk directly on the output
    # trajectory (never the report's own numbers).
    chain = load_tool_chain(URDF_PATH, END_EFFECTOR_PATH)
    first, last = trace_entry["point_range"]
    configs = np.array([p["positions"] for p in traj["points"][first:last + 1]])
    t_s = np.array([p["t_s"] for p in traj["points"][first:last + 1]])
    pts = chain.tool_positions(configs)
    chords = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    speeds = chords / np.diff(t_s)
    assert speeds.min() == pytest.approx(ts["min"], abs=1e-6)
    assert speeds.max() == pytest.approx(ts["max"], abs=1e-6)
    assert np.all(speeds <= v_c * (1.0 + tol) + 1e-9)

    _assert_report_matches_trajectory(plan, report, traj, retime_cfg_path)


# --------------------------------------------------------------------------------------
# retime_plan_doc -- refusals
# --------------------------------------------------------------------------------------

def test_retime_plan_doc_refuses_on_limits_sha_mismatch(tmp_path):
    plan, plan_path = _build_plan(tmp_path, STRAIGHT_TRACE, "straight")
    plan["sources"]["limits"]["sha256"] = _flip_sha(plan["sources"]["limits"]["sha256"])
    retime_cfg_path = _write_retime_config(tmp_path)
    exec_cfg_path = _write_execution_config(tmp_path)

    with pytest.raises(RetimeError, match="limits"):
        retime_plan_doc(
            plan, limits_path=LIMITS_PATH, end_effector_path=END_EFFECTOR_PATH, urdf_path=URDF_PATH,
            retime_config_path=retime_cfg_path, execution_config_path=exec_cfg_path,
            purpose="test", plan_path=plan_path,
        )


def test_retime_plan_doc_refuses_on_end_effector_sha_mismatch(tmp_path):
    plan, plan_path = _build_plan(tmp_path, STRAIGHT_TRACE, "straight")
    plan["sources"]["end_effector"]["sha256"] = _flip_sha(plan["sources"]["end_effector"]["sha256"])
    retime_cfg_path = _write_retime_config(tmp_path)
    exec_cfg_path = _write_execution_config(tmp_path)

    with pytest.raises(RetimeError, match="end_effector"):
        retime_plan_doc(
            plan, limits_path=LIMITS_PATH, end_effector_path=END_EFFECTOR_PATH, urdf_path=URDF_PATH,
            retime_config_path=retime_cfg_path, execution_config_path=exec_cfg_path,
            purpose="test", plan_path=plan_path,
        )


def test_retime_plan_doc_refuses_when_trace_leg_too_slow(tmp_path):
    plan, plan_path = _build_plan(tmp_path, CURVED_TRACE, "curved_refuse")
    # The corner's own leading ramp is ~0.0099 m at v_c close to the 0.05 m/s target (see the
    # happy-path curved test); raising the floor above where that ramp shrinks enough, while
    # capping the allowed ramp far below it, leaves no feasible v_c anywhere in range.
    retime_cfg_path = _write_retime_config(tmp_path, min_speed=0.045, max_ramp=0.001)
    exec_cfg_path = _write_execution_config(tmp_path)

    with pytest.raises(RetimeError, match=r"trace leg \d+"):
        retime_plan_doc(
            plan, limits_path=LIMITS_PATH, end_effector_path=END_EFFECTOR_PATH, urdf_path=URDF_PATH,
            retime_config_path=retime_cfg_path, execution_config_path=exec_cfg_path,
            purpose="test", plan_path=plan_path,
        )


# --------------------------------------------------------------------------------------
# write_outputs
# --------------------------------------------------------------------------------------

def test_write_outputs_roundtrip(tmp_path):
    plan, plan_path = _build_plan(tmp_path, STRAIGHT_TRACE, "straight")
    retime_cfg_path = _write_retime_config(tmp_path)
    exec_cfg_path = _write_execution_config(tmp_path)
    traj, report = retime_plan_doc(
        plan, limits_path=LIMITS_PATH, end_effector_path=END_EFFECTOR_PATH, urdf_path=URDF_PATH,
        retime_config_path=retime_cfg_path, execution_config_path=exec_cfg_path,
        purpose="test", plan_path=plan_path,
    )

    out_dir = tmp_path / "out"
    out_path = out_dir / "synthetic_straight_trajectory.json"
    write_outputs(traj, report, out_path)

    loaded_traj = load_trajectory(out_path)
    assert loaded_traj["points"][0]["positions"] == traj["points"][0]["positions"]
    assert len(loaded_traj["points"]) == len(traj["points"])

    report_path = Path(str(out_path) + ".retime.json")
    loaded_report = json.loads(report_path.read_text())
    assert validate_retime_report(loaded_report) == loaded_report
    assert loaded_report["trajectory_sha256"] == report["trajectory_sha256"]

    leftovers = list(out_dir.glob(".*"))
    assert leftovers == []


# --------------------------------------------------------------------------------------
# validate_retime_report -- strict loader
# --------------------------------------------------------------------------------------

@pytest.fixture(scope="module")
def _valid_report(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("report_fixture")
    plan, plan_path = _build_plan(tmp_path, STRAIGHT_TRACE, "straight")
    retime_cfg_path = _write_retime_config(tmp_path)
    exec_cfg_path = _write_execution_config(tmp_path)
    _traj, report = retime_plan_doc(
        plan, limits_path=LIMITS_PATH, end_effector_path=END_EFFECTOR_PATH, urdf_path=URDF_PATH,
        retime_config_path=retime_cfg_path, execution_config_path=exec_cfg_path,
        purpose="test", plan_path=plan_path,
    )
    return report


def test_validate_retime_report_accepts_valid_doc(_valid_report):
    assert validate_retime_report(_valid_report) == _valid_report


def test_validate_retime_report_rejects_missing_key(_valid_report):
    bad = copy.deepcopy(_valid_report)
    del bad["self_check"]
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_unknown_key(_valid_report):
    bad = copy.deepcopy(_valid_report)
    bad["bogus"] = 1
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_bad_schema(_valid_report):
    bad = copy.deepcopy(_valid_report)
    bad["schema"] = "wrong/1"
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_nonzero_self_check(_valid_report):
    bad = copy.deepcopy(_valid_report)
    bad["self_check"] = {"violations": 1}
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_nonpositive_tool_speed_constant(_valid_report):
    bad = copy.deepcopy(_valid_report)
    trace_leg = next(leg for leg in bad["legs"] if leg["kind"] == "trace")
    trace_leg["tool_speed"]["constant"] = 0.0
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_bad_point_range(_valid_report):
    bad = copy.deepcopy(_valid_report)
    bad["legs"][0]["point_range"] = [5, 2]
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_bad_input_shas(_valid_report):
    bad = copy.deepcopy(_valid_report)
    bad["input_shas"]["plan"] = "not-a-sha"
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_parameters_wrong_schema(_valid_report):
    bad = copy.deepcopy(_valid_report)
    bad["parameters"]["schema"] = "wrong/9"
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_parameters_non_mapping_nested_block(_valid_report):
    bad = copy.deepcopy(_valid_report)
    bad["parameters"]["limit_fraction"] = None
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_parameters_wrong_type_nested_block(_valid_report):
    bad = copy.deepcopy(_valid_report)
    bad["parameters"]["trace"] = "garbage"
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_parameters_missing_nested_key(_valid_report):
    bad = copy.deepcopy(_valid_report)
    del bad["parameters"]["trace"]["max_ramp_m"]
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_parameters_extra_nested_key(_valid_report):
    bad = copy.deepcopy(_valid_report)
    bad["parameters"]["sampling"]["bogus"] = 1.0
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_parameters_out_of_range_value(_valid_report):
    bad = copy.deepcopy(_valid_report)
    bad["parameters"]["limit_fraction"]["velocity"] = 1.5
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_non_iso8601_created_utc(_valid_report):
    bad = copy.deepcopy(_valid_report)
    bad["created_utc"] = "hello"
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


def test_validate_retime_report_rejects_point_range_last_at_n_points(_valid_report):
    bad = copy.deepcopy(_valid_report)
    n_points = bad["totals"]["n_points"]
    bad["legs"][-1]["point_range"] = [0, n_points]
    with pytest.raises(RetimeError):
        validate_retime_report(bad)


# --------------------------------------------------------------------------------------
# module hygiene
# --------------------------------------------------------------------------------------

def test_no_ros_imports():
    src = (Path(__file__).resolve().parents[1] / "crackvision_motion" / "retime.py").read_text()
    for line in src.splitlines():
        stripped = line.strip()
        if stripped.startswith("import rclpy") or stripped.startswith("from rclpy"):
            pytest.fail("retime.py must not import rclpy")
        if "_msgs" in stripped and (stripped.startswith("import ") or stripped.startswith("from ")):
            pytest.fail("retime.py must not import a *_msgs package")

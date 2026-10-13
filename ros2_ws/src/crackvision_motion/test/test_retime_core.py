"""Unit tests for retime_core (MOT-07.3). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_retime_core.py -q -p no:cacheprovider
"""

from __future__ import annotations

import copy
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from crackvision_motion.cartesian_plan import CONTINUITY_TOL_RAD
from crackvision_motion.joint_trajectory import JOINT_NAMES, check_limits, load_limits
from crackvision_motion.retime_core import (
    RetimeConfigError,
    RetimeError,
    load_retime_config,
    plan_polyline,
    refine,
    time_profile,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
PRODUCTION_LIMITS = REPO_ROOT / "config" / "robot" / "b601_dm_limits.yaml"
PRODUCTION_RETIME_CONFIG = REPO_ROOT / "config" / "motion" / "retime.yaml"
PRODUCTION_MARGIN_RAD = 0.03  # config/motion/execution.yaml's position_margin_rad, carried as a
# literal (not loaded from that file) per the card's "do not depend on production config shas".

LIMITS = load_limits(PRODUCTION_LIMITS)

_VALID_RAW = {
    "schema": "crackvision.retime_config/1",
    "limit_fraction": {"velocity": 0.95, "acceleration": 0.95},
    "sampling": {"max_joint_step_rad": 0.01, "max_period_s": 0.02},
    "trace": {
        "tool_speed_m_s": 0.05,
        "min_tool_speed_m_s": 0.005,
        "speed_tolerance": 0.05,
        "max_ramp_m": 0.005,
    },
    "value_status": "nominal",
    "sources": {"limit_fraction": "x", "sampling": "x", "trace": "x"},
}


def _write(tmp_path, raw) -> Path:
    p = tmp_path / "retime.yaml"
    p.write_text(yaml.safe_dump(raw))
    return p


def _leg(leg_index, kind, positions):
    return {"leg_index": leg_index, "kind": kind, "positions": positions}


def _q(*vals):
    assert len(vals) == 6
    return list(vals)


# --------------------------------------------------------------------------------------
# load_retime_config
# --------------------------------------------------------------------------------------

def test_load_production_retime_config():
    cfg = load_retime_config(PRODUCTION_RETIME_CONFIG)
    assert cfg["schema"] == "crackvision.retime_config/1"
    assert 0.0 < cfg["limit_fraction"]["velocity"] <= 1.0
    assert 0.0 < cfg["limit_fraction"]["acceleration"] <= 1.0
    assert cfg["sampling"]["max_joint_step_rad"] > 0
    assert cfg["sampling"]["max_period_s"] > 0
    assert cfg["trace"]["tool_speed_m_s"] > cfg["trace"]["min_tool_speed_m_s"]
    assert cfg["value_status"] == "nominal"
    for key in ("limit_fraction", "sampling", "trace"):
        assert cfg["sources"][key]


def test_load_retime_config_roundtrip(tmp_path):
    cfg = load_retime_config(_write(tmp_path, _VALID_RAW))
    assert cfg["sampling"]["max_period_s"] == 0.02


def test_load_retime_config_unknown_top_key(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["bogus"] = 1
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))


def test_load_retime_config_missing_key(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    del raw["sampling"]["max_period_s"]
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))


@pytest.mark.parametrize("value", [0.0, -0.1, 1.5])
def test_load_retime_config_velocity_fraction_out_of_range(tmp_path, value):
    raw = copy.deepcopy(_VALID_RAW)
    raw["limit_fraction"]["velocity"] = value
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))


@pytest.mark.parametrize("value", [0.0, -0.1, 1.5])
def test_load_retime_config_acceleration_fraction_out_of_range(tmp_path, value):
    raw = copy.deepcopy(_VALID_RAW)
    raw["limit_fraction"]["acceleration"] = value
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))


@pytest.mark.parametrize("field", ["max_joint_step_rad", "max_period_s"])
@pytest.mark.parametrize("value", [0.0, -1.0])
def test_load_retime_config_sampling_must_be_positive(tmp_path, field, value):
    raw = copy.deepcopy(_VALID_RAW)
    raw["sampling"][field] = value
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))


def test_load_retime_config_min_speed_must_be_below_target(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["trace"]["min_tool_speed_m_s"] = 0.05
    raw["trace"]["tool_speed_m_s"] = 0.05
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))

    raw["trace"]["min_tool_speed_m_s"] = 0.06
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))


@pytest.mark.parametrize("value", [0.0, 1.0, -0.1, 1.5])
def test_load_retime_config_speed_tolerance_out_of_range(tmp_path, value):
    raw = copy.deepcopy(_VALID_RAW)
    raw["trace"]["speed_tolerance"] = value
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))


def test_load_retime_config_max_ramp_must_be_positive(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["trace"]["max_ramp_m"] = 0.0
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))


def test_load_retime_config_bad_schema(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["schema"] = "wrong/1"
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))


def test_load_retime_config_bad_value_status(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["value_status"] = "measured"
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))


def test_load_retime_config_empty_sources(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    raw["sources"] = {}
    with pytest.raises(RetimeConfigError):
        load_retime_config(_write(tmp_path, raw))


# --------------------------------------------------------------------------------------
# plan_polyline
# --------------------------------------------------------------------------------------

def test_plan_polyline_single_leg():
    p0, p1 = _q(0, 0, 0, 0, 0, 0), _q(0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
    plan = {"legs": [_leg(0, "transit", [p0, p1])]}
    configs, spans = plan_polyline(plan)
    assert configs.shape == (2, 6)
    assert configs.tolist() == [p0, p1]
    assert spans == [(0, "transit", 0, 1)]


def test_plan_polyline_drops_continuous_leg_boundary_and_shares_index():
    p0 = _q(0, 0, 0, 0, 0, 0)
    p1 = _q(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
    p1_dup = [v + 1e-9 for v in p1]  # within CONTINUITY_TOL_RAD of p1
    p2 = _q(0.2, 0.0, 0.0, 0.0, 0.0, 0.0)
    plan = {"legs": [_leg(0, "transit", [p0, p1]), _leg(1, "trace", [p1_dup, p2])]}
    configs, spans = plan_polyline(plan)
    assert configs.shape == (3, 6)
    # Every surviving row is bit-exact from whichever leg contributed it (leg 0's own p1, not
    # leg 1's near-duplicate copy).
    assert configs[1].tolist() == p1
    assert spans == [(0, "transit", 0, 1), (1, "trace", 1, 2)]


def test_plan_polyline_keeps_non_continuous_leg_boundary_distinct():
    p0 = _q(0, 0, 0, 0, 0, 0)
    p1 = _q(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
    p1_far = _q(0.1 + 10 * CONTINUITY_TOL_RAD, 0.0, 0.0, 0.0, 0.0, 0.0)
    p2 = _q(0.2, 0.0, 0.0, 0.0, 0.0, 0.0)
    plan = {"legs": [_leg(0, "transit", [p0, p1]), _leg(1, "trace", [p1_far, p2])]}
    configs, spans = plan_polyline(plan)
    assert configs.shape == (4, 6)
    assert spans == [(0, "transit", 0, 1), (1, "trace", 2, 3)]


def test_plan_polyline_three_legs_leg_index_order():
    p0, p1, p2, p3 = (_q(float(i), 0, 0, 0, 0, 0) for i in range(4))
    plan = {"legs": [_leg(0, "transit", [p0, p1]), _leg(1, "trace", [p1, p2]), _leg(2, "transit", [p2, p3])]}
    configs, spans = plan_polyline(plan)
    assert configs.shape == (4, 6)
    assert spans == [(0, "transit", 0, 1), (1, "trace", 1, 2), (2, "transit", 2, 3)]


def test_plan_polyline_rejects_empty_legs():
    with pytest.raises(RetimeError):
        plan_polyline({"legs": []})
    with pytest.raises(RetimeError):
        plan_polyline({})


def test_plan_polyline_rejects_leg_with_no_positions():
    plan = {"legs": [_leg(0, "transit", [])]}
    with pytest.raises(RetimeError):
        plan_polyline(plan)


def test_plan_polyline_rejects_wrong_length_vector():
    plan = {"legs": [_leg(0, "transit", [[0, 0, 0, 0, 0], _q(0.1, 0, 0, 0, 0, 0)])]}
    with pytest.raises(RetimeError):
        plan_polyline(plan)


def test_plan_polyline_rejects_nan():
    plan = {"legs": [_leg(0, "transit", [_q(0, 0, 0, 0, 0, 0), _q(float("nan"), 0, 0, 0, 0, 0)])]}
    with pytest.raises(RetimeError):
        plan_polyline(plan)


# --------------------------------------------------------------------------------------
# refine (G-PATH)
# --------------------------------------------------------------------------------------

def test_refine_respects_max_step_and_keeps_endpoints_bit_exact():
    p0 = np.array(_q(0, 0, 0, 0, 0, 0))
    p1 = np.array(_q(0.1, -0.1, 0.0, 0.0, 0.0, 0.0))
    configs = np.array([p0, p1])
    spans = [(0, "transit", 0, 1)]
    refined, new_spans = refine(configs, spans, 0.01)
    assert refined[0].tolist() == p0.tolist()
    assert refined[-1].tolist() == p1.tolist()
    steps = np.max(np.abs(refined[1:] - refined[:-1]), axis=1)
    assert np.all(steps <= 0.01 + 1e-12)
    assert new_spans == [(0, "transit", 0, refined.shape[0] - 1)]


def test_refine_no_corner_blending_stays_on_segment():
    p0 = np.array(_q(0, 0, 0, 0, 0, 0))
    p1 = np.array(_q(0.05, 0, 0, 0, 0, 0))
    p2 = np.array(_q(0.05, 0.05, 0, 0, 0, 0))
    configs = np.array([p0, p1, p2])
    refined, spans = refine(configs, [(0, "transit", 0, 2)], 0.01)
    # Every interior point lies on one of the two straight segments (no interpolation crossing
    # the corner at p1).
    for row in refined:
        on_first = abs(row[1]) < 1e-12 and 0.0 <= row[0] <= 0.05 + 1e-12
        on_second = abs(row[0] - 0.05) < 1e-12 and 0.0 <= row[1] <= 0.05 + 1e-12
        assert on_first or on_second
    assert any(np.array_equal(row, p1) for row in refined)


def test_refine_preserves_every_plan_vertex_bit_exact():
    rng = np.random.default_rng(1)
    pts = [rng.uniform(-0.2, 0.2, size=6) for _ in range(4)]
    configs = np.array(pts)
    refined, _ = refine(configs, [(0, "transit", 0, 3)], 0.01)
    for p in pts:
        assert any(np.array_equal(row, p) for row in refined)


def test_refine_rejects_too_few_points():
    with pytest.raises(RetimeError):
        refine(np.zeros((1, 6)), [(0, "transit", 0, 0)], 0.01)


def test_refine_rejects_non_positive_step():
    configs = np.array([_q(0, 0, 0, 0, 0, 0), _q(0.1, 0, 0, 0, 0, 0)])
    with pytest.raises(RetimeError):
        refine(configs, [(0, "transit", 0, 1)], 0.0)


def test_refine_rejects_nan():
    configs = np.array([_q(0, 0, 0, 0, 0, 0), [float("nan")] * 6])
    with pytest.raises(RetimeError):
        refine(configs, [(0, "transit", 0, 1)], 0.01)


# --------------------------------------------------------------------------------------
# time_profile -- shared helpers
# --------------------------------------------------------------------------------------

F_V = 0.95
F_A = 0.95
MAX_PERIOD_S = 0.02
MAX_STEP_RAD = 0.01


def _retime(positions, kind="transit", caps=None):
    plan = {"legs": [_leg(0, kind, positions)]}
    configs, spans = plan_polyline(plan)
    refined, spans = refine(configs, spans, MAX_STEP_RAD)
    return time_profile(
        refined, spans, LIMITS, f_v=F_V, f_a=F_A, margin_rad=PRODUCTION_MARGIN_RAD,
        max_period_s=MAX_PERIOD_S, caps=caps,
    )


def _as_traj(configs, t_s):
    return {"points": [{"t_s": float(t), "positions": row.tolist()} for t, row in zip(t_s, configs)]}


def _assert_fully_compliant(configs, t_s):
    traj = _as_traj(configs, t_s)
    violations = check_limits(traj, LIMITS, position_margin_rad=PRODUCTION_MARGIN_RAD)
    assert violations == []

    tau = np.diff(t_s)
    assert np.all(tau > 0)
    assert np.all(tau <= MAX_PERIOD_S + 1e-9)
    steps = np.max(np.abs(np.diff(configs, axis=0)), axis=1)
    assert np.all(steps <= MAX_STEP_RAD + 1e-9)

    # (R): rest-to-rest, directly from the FD formulas this module itself guarantees.
    alim = np.array([LIMITS[j]["acceleration"] for j in JOINT_NAMES]) * F_A
    v0 = (configs[1] - configs[0]) / tau[0]
    assert np.all(np.abs(v0) <= alim * tau[0] + 1e-6)
    v_last = (configs[-1] - configs[-2]) / tau[-1]
    assert np.all(np.abs(v_last) <= alim * tau[-1] + 1e-6)


def _analytic_duration(delta, f_v=F_V, f_a=F_A):
    v = np.array([LIMITS[j]["velocity"] for j in JOINT_NAMES]) * f_v
    a = np.array([LIMITS[j]["acceleration"] for j in JOINT_NAMES]) * f_a
    times = []
    for j in range(6):
        h = abs(delta[j])
        if h == 0:
            times.append(0.0)
            continue
        t_acc = v[j] / a[j]
        d_acc = 0.5 * a[j] * t_acc * t_acc
        if 2 * d_acc <= h:
            times.append(2 * t_acc + (h - 2 * d_acc) / v[j])
        else:
            times.append(2 * math.sqrt(h / a[j]))
    return max(times)


# --------------------------------------------------------------------------------------
# time_profile -- required scenarios
# --------------------------------------------------------------------------------------

def test_time_profile_single_straight_move_near_optimal():
    delta = [0.5, -0.3, -0.2, 0.1, 0.3, 0.4]
    start = _q(0, 0, 0, 0, 0, 0)
    end = [a + b for a, b in zip(start, delta)]
    configs, t_s, spans = _retime([start, end])
    _assert_fully_compliant(configs, t_s)

    analytic = _analytic_duration(np.array(delta))
    ratio = t_s[-1] / analytic
    assert 0.9 <= ratio <= 1.1, f"duration {t_s[-1]} vs analytic {analytic} (ratio {ratio})"


def test_time_profile_sharp_corners():
    positions = [
        _q(0, 0, 0, 0, 0, 0),
        _q(0.3, -0.2, -0.1, 0.05, 0.1, 0.1),
        _q(0.1, -0.4, -0.3, 0.2, -0.2, 0.3),
        _q(0.4, -0.1, -0.35, 0.0, 0.15, -0.1),
    ]
    configs, t_s, spans = _retime(positions)
    _assert_fully_compliant(configs, t_s)
    assert spans[0][2] == 0
    assert spans[0][3] == configs.shape[0] - 1


def test_time_profile_joint2_home_approach():
    # Mirrors §11.6's own home-approach shape: joint2 (upper bound 0.0) monotonically approaches
    # and reaches its bound; the margin rule (M) must not let the arm speed up into it.
    positions = [
        _q(0.2, -0.6, -0.4, 0.1, 0.1, 0.1),
        _q(0.1, -0.3, -0.2, 0.05, 0.05, 0.05),
        _q(0.0, -0.02, -0.01, 0.0, 0.0, 0.0),
        _q(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ]
    configs, t_s, spans = _retime(positions)
    _assert_fully_compliant(configs, t_s)
    assert configs[-1].tolist() == positions[-1]


def test_time_profile_coarse_transit_path():
    rng = np.random.default_rng(0)
    cur = np.zeros(6)
    positions = [cur.tolist()]
    for _ in range(6):
        cur = cur + rng.uniform(-0.1, 0.1, size=6)
        positions.append(cur.tolist())
    configs, t_s, spans = _retime(positions)
    _assert_fully_compliant(configs, t_s)


def test_time_profile_external_caps_bind():
    start = _q(0, 0, 0, 0, 0, 0)
    end = _q(0.3, -0.2, -0.1, 0.1, 0.1, 0.1)
    plan = {"legs": [_leg(0, "trace", [start, end])]}
    configs, spans = plan_polyline(plan)
    refined, spans = refine(configs, spans, MAX_STEP_RAD)
    n_seg = refined.shape[0] - 1
    cap_value = 0.05
    configs_out, t_s, spans_out = time_profile(
        refined, spans, LIMITS, f_v=F_V, f_a=F_A, margin_rad=PRODUCTION_MARGIN_RAD,
        max_period_s=MAX_PERIOD_S, caps=[cap_value] * n_seg,
    )
    _assert_fully_compliant(configs_out, t_s)
    tau = np.diff(t_s)
    h = np.linalg.norm(np.diff(configs_out, axis=0), axis=1)
    u = h / tau
    assert np.all(u <= cap_value + 1e-6)
    # The cap is actually binding (not just a no-op upper bound).
    assert np.max(u) > cap_value * 0.5


# --------------------------------------------------------------------------------------
# time_profile -- infeasible inputs, determinism, and signature details
# --------------------------------------------------------------------------------------

def test_time_profile_rejects_nan_configs():
    configs = np.array([_q(0, 0, 0, 0, 0, 0), [float("nan")] * 6])
    with pytest.raises(RetimeError):
        time_profile(configs, [(0, "transit", 0, 1)], LIMITS, f_v=F_V, f_a=F_A, margin_rad=0.03, max_period_s=0.02)


def test_time_profile_rejects_too_few_points():
    configs = np.zeros((1, 6))
    with pytest.raises(RetimeError):
        time_profile(configs, [(0, "transit", 0, 0)], LIMITS, f_v=F_V, f_a=F_A, margin_rad=0.03, max_period_s=0.02)


@pytest.mark.parametrize("field,value", [("f_v", 0.0), ("f_v", 1.5), ("f_a", 0.0), ("f_a", 1.5)])
def test_time_profile_rejects_bad_fraction(field, value):
    configs = np.array([_q(0, 0, 0, 0, 0, 0), _q(0.1, 0, 0, 0, 0, 0)])
    kwargs = dict(f_v=F_V, f_a=F_A, margin_rad=0.03, max_period_s=0.02)
    kwargs[field] = value
    with pytest.raises(RetimeError):
        time_profile(configs, [(0, "transit", 0, 1)], LIMITS, **kwargs)


def test_time_profile_rejects_non_positive_max_period_s():
    configs = np.array([_q(0, 0, 0, 0, 0, 0), _q(0.1, 0, 0, 0, 0, 0)])
    with pytest.raises(RetimeError):
        time_profile(configs, [(0, "transit", 0, 1)], LIMITS, f_v=F_V, f_a=F_A, margin_rad=0.03, max_period_s=0.0)


def test_time_profile_rejects_mismatched_caps_length():
    configs = np.array([_q(0, 0, 0, 0, 0, 0), _q(0.1, 0, 0, 0, 0, 0), _q(0.2, 0, 0, 0, 0, 0)])
    with pytest.raises(RetimeError):
        time_profile(
            configs, [(0, "transit", 0, 2)], LIMITS, f_v=F_V, f_a=F_A, margin_rad=0.03, max_period_s=0.02,
            caps=[1.0],
        )


def test_time_profile_rejects_non_positive_cap():
    configs = np.array([_q(0, 0, 0, 0, 0, 0), _q(0.1, 0, 0, 0, 0, 0)])
    with pytest.raises(RetimeError):
        time_profile(
            configs, [(0, "transit", 0, 1)], LIMITS, f_v=F_V, f_a=F_A, margin_rad=0.03, max_period_s=0.02,
            caps=[0.0],
        )


def test_time_profile_deterministic():
    positions = [
        _q(0, 0, 0, 0, 0, 0),
        _q(0.3, -0.2, -0.1, 0.05, 0.1, 0.1),
        _q(0.1, -0.4, -0.3, 0.2, -0.2, 0.3),
    ]
    configs1, t_s1, _ = _retime(positions)
    configs2, t_s2, _ = _retime(positions)
    assert np.array_equal(configs1, configs2)
    assert np.array_equal(t_s1, t_s2)


def test_time_profile_degenerate_zero_length_segment_raises():
    configs = np.array([_q(0, 0, 0, 0, 0, 0), _q(0, 0, 0, 0, 0, 0)])
    with pytest.raises(RetimeError):
        time_profile(configs, [(0, "transit", 0, 1)], LIMITS, f_v=F_V, f_a=F_A, margin_rad=0.03, max_period_s=0.02)


# --------------------------------------------------------------------------------------
# module hygiene
# --------------------------------------------------------------------------------------

def test_no_ros_imports():
    src = (Path(__file__).resolve().parents[1] / "crackvision_motion" / "retime_core.py").read_text()
    for line in src.splitlines():
        stripped = line.strip()
        if stripped.startswith("import rclpy") or stripped.startswith("from rclpy"):
            pytest.fail("retime_core.py must not import rclpy")
        if "_msgs" in stripped and (stripped.startswith("import ") or stripped.startswith("from ")):
            pytest.fail("retime_core.py must not import a *_msgs package")

"""Unit tests for joint_trajectory (MOT-05.2). Runs without a colcon build.

Usage: bash scripts/ros/env_ros.sh python3 -m pytest \
    ros2_ws/src/crackvision_motion/test/test_joint_trajectory.py -q -p no:cacheprovider
"""

import copy
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from crackvision_motion.joint_trajectory import (  # noqa: E402
    JOINT_NAMES,
    TrajectoryError,
    Violation,
    check_limits,
    densify,
    file_sha256,
    load_limits,
    load_trajectory,
    max_abs_velocity,
    scale_time,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
PRODUCTION_LIMITS = REPO_ROOT / "config" / "robot" / "b601_dm_limits.yaml"
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"

SMOKE = FIXTURES_DIR / "trajectory_smoke.json"
ESTOP = FIXTURES_DIR / "trajectory_estop.json"
OVERSPEED = FIXTURES_DIR / "trajectory_overspeed.json"
OUT_OF_LIMITS = FIXTURES_DIR / "trajectory_out_of_limits.json"

_VALID_RAW = {
    "schema": "crackvision.joint_trajectory/1",
    "created_utc": "2026-01-01T00:00:00Z",
    "producer": "test",
    "purpose": "test",
    "planning_frame": "base_link",
    "joint_names": list(JOINT_NAMES),
    "points": [
        {"t_s": 0.0, "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
        {"t_s": 1.0, "positions": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]},
    ],
    "limits_file": {"path": "config/robot/b601_dm_limits.yaml", "sha256": None},
    "end_effector_config_sha256": None,
    "scene_config_sha256": None,
    "source": {"paths3d": None},
}

_SHA_A = "a" * 64
_SHA_B = "b" * 64


def _write(tmp_path, raw):
    p = tmp_path / "t.json"
    p.write_text(json.dumps(raw))
    return p


def _mutate(raw, **kw):
    out = copy.deepcopy(raw)
    out.update(kw)
    return out


# --------------------------------------------------------------------------------------
# load_trajectory: valid fixtures
# --------------------------------------------------------------------------------------

def test_load_smoke_fixture():
    traj = load_trajectory(SMOKE)
    assert traj["schema"] == "crackvision.joint_trajectory/1"
    assert traj["joint_names"] == list(JOINT_NAMES)
    assert len(traj["points"]) == 3
    assert traj["points"][0]["t_s"] == 0.0
    assert traj["_path"] == str(Path(SMOKE))
    assert "notes" in traj


def test_load_trajectory_valid_minimal(tmp_path):
    path = _write(tmp_path, _VALID_RAW)
    traj = load_trajectory(path)
    assert traj["points"][0]["positions"] == [0.0] * 6
    assert traj["limits_file"]["sha256"] is None
    assert traj["source"]["paths3d"] is None


def test_load_trajectory_accepts_sha_fields(tmp_path):
    raw = _mutate(
        _VALID_RAW,
        limits_file={"path": "config/robot/b601_dm_limits.yaml", "sha256": _SHA_A},
        end_effector_config_sha256=_SHA_B,
        scene_config_sha256=_SHA_A,
        source={"paths3d": {"path": "a/b.json", "sha256": _SHA_B, "execution_eligible": True}},
    )
    path = _write(tmp_path, raw)
    traj = load_trajectory(path)
    assert traj["limits_file"]["sha256"] == _SHA_A
    assert traj["end_effector_config_sha256"] == _SHA_B
    assert traj["source"]["paths3d"]["execution_eligible"] is True


# --------------------------------------------------------------------------------------
# load_trajectory: each malformed case raises TrajectoryError with a specific message
# --------------------------------------------------------------------------------------

def test_wrong_joint_names(tmp_path):
    raw = _mutate(_VALID_RAW, joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint7"])
    with pytest.raises(TrajectoryError, match="joint_names"):
        load_trajectory(_write(tmp_path, raw))


def test_reordered_joint_names(tmp_path):
    raw = _mutate(_VALID_RAW, joint_names=["joint2", "joint1", "joint3", "joint4", "joint5", "joint6"])
    with pytest.raises(TrajectoryError, match="joint_names"):
        load_trajectory(_write(tmp_path, raw))


def test_fewer_than_two_points(tmp_path):
    raw = _mutate(_VALID_RAW, points=[_VALID_RAW["points"][0]])
    with pytest.raises(TrajectoryError, match="at least 2 entries"):
        load_trajectory(_write(tmp_path, raw))


def test_first_t_s_nonzero(tmp_path):
    points = copy.deepcopy(_VALID_RAW["points"])
    points[0]["t_s"] = 0.5
    raw = _mutate(_VALID_RAW, points=points)
    with pytest.raises(TrajectoryError, match="first point's t_s must be 0"):
        load_trajectory(_write(tmp_path, raw))


def test_non_increasing_t_s(tmp_path):
    points = copy.deepcopy(_VALID_RAW["points"]) + [{"t_s": 1.0, "positions": [0.2] * 6}]
    raw = _mutate(_VALID_RAW, points=points)
    with pytest.raises(TrajectoryError, match="strictly increasing"):
        load_trajectory(_write(tmp_path, raw))


def test_wrong_vector_length(tmp_path):
    points = copy.deepcopy(_VALID_RAW["points"])
    points[1]["positions"] = [0.1] * 5
    raw = _mutate(_VALID_RAW, points=points)
    with pytest.raises(TrajectoryError, match="length 6"):
        load_trajectory(_write(tmp_path, raw))


def test_nan_position(tmp_path):
    points = copy.deepcopy(_VALID_RAW["points"])
    points[1]["positions"][0] = math.nan
    path = tmp_path / "t.json"
    # json.dumps(nan) emits the literal NaN, which json.loads accepts (not strict JSON, but
    # matches Python's json module default behaviour either way).
    raw = _mutate(_VALID_RAW, points=points)
    path.write_text(json.dumps(raw))
    with pytest.raises(TrajectoryError, match="finite"):
        load_trajectory(path)


def test_inf_velocity(tmp_path):
    points = copy.deepcopy(_VALID_RAW["points"])
    points[1]["velocities"] = [math.inf, 0.0, 0.0, 0.0, 0.0, 0.0]
    raw = _mutate(_VALID_RAW, points=points)
    path = tmp_path / "t.json"
    path.write_text(json.dumps(raw))
    with pytest.raises(TrajectoryError, match="finite"):
        load_trajectory(path)


def test_bad_sha_hex_limits_file(tmp_path):
    raw = _mutate(_VALID_RAW, limits_file={"path": "x.yaml", "sha256": "not-a-sha"})
    with pytest.raises(TrajectoryError, match="sha256 digest"):
        load_trajectory(_write(tmp_path, raw))


def test_bad_sha_hex_end_effector(tmp_path):
    raw = _mutate(_VALID_RAW, end_effector_config_sha256="deadbeef")
    with pytest.raises(TrajectoryError, match="sha256 digest"):
        load_trajectory(_write(tmp_path, raw))


def test_unknown_top_level_key(tmp_path):
    raw = _mutate(_VALID_RAW, extra_field="nope")
    with pytest.raises(TrajectoryError, match="unknown top-level key"):
        load_trajectory(_write(tmp_path, raw))


def test_unknown_point_key(tmp_path):
    points = copy.deepcopy(_VALID_RAW["points"])
    points[1]["bogus"] = 1
    raw = _mutate(_VALID_RAW, points=points)
    with pytest.raises(TrajectoryError, match="unknown key"):
        load_trajectory(_write(tmp_path, raw))


def test_bad_purpose(tmp_path):
    raw = _mutate(_VALID_RAW, purpose="production")
    with pytest.raises(TrajectoryError, match="purpose"):
        load_trajectory(_write(tmp_path, raw))


def test_bad_planning_frame(tmp_path):
    raw = _mutate(_VALID_RAW, planning_frame="world")
    with pytest.raises(TrajectoryError, match="planning_frame"):
        load_trajectory(_write(tmp_path, raw))


def test_missing_required_key(tmp_path):
    raw = copy.deepcopy(_VALID_RAW)
    del raw["producer"]
    with pytest.raises(TrajectoryError, match="missing required top-level key"):
        load_trajectory(_write(tmp_path, raw))


def test_bad_source_shape(tmp_path):
    raw = _mutate(_VALID_RAW, source={"paths3d": {"path": "a", "sha256": _SHA_A}})
    with pytest.raises(TrajectoryError, match="source.paths3d"):
        load_trajectory(_write(tmp_path, raw))


# --------------------------------------------------------------------------------------
# file_sha256
# --------------------------------------------------------------------------------------

def test_file_sha256_matches_hashlib(tmp_path):
    import hashlib
    p = tmp_path / "f.bin"
    p.write_bytes(b"hello world")
    assert file_sha256(p) == hashlib.sha256(b"hello world").hexdigest()


# --------------------------------------------------------------------------------------
# load_limits
# --------------------------------------------------------------------------------------

def test_load_limits_production_config():
    limits = load_limits(PRODUCTION_LIMITS)
    assert set(limits) == set(JOINT_NAMES)
    for name in JOINT_NAMES:
        spec = limits[name]
        assert set(spec) == {"lower", "upper", "velocity", "acceleration"}
        assert spec["lower"] < spec["upper"]
        assert spec["velocity"] > 0
        assert spec["acceleration"] > 0
    assert limits["joint2"]["upper"] == 0.0
    assert limits["joint3"]["upper"] == 0.0


# --------------------------------------------------------------------------------------
# check_limits
# --------------------------------------------------------------------------------------

@pytest.fixture
def limits():
    return load_limits(PRODUCTION_LIMITS)


def test_check_limits_smoke_fixture_passes(limits):
    traj = load_trajectory(SMOKE)
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    assert violations == []


def test_check_limits_out_of_limits_reports_position_violation(limits):
    traj = load_trajectory(OUT_OF_LIMITS)
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    position_violations = [v for v in violations if v.kind == "position"]
    assert any(v.joint == "joint2" and v.index == 1 for v in position_violations)
    v = next(v for v in position_violations if v.joint == "joint2")
    assert v.value == pytest.approx(0.5)
    assert v.bound == pytest.approx(0.0)


def test_check_limits_overspeed_unscaled_fails(limits):
    traj = load_trajectory(OVERSPEED)
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    vel_violations = [v for v in violations if v.kind == "velocity"]
    assert len(vel_violations) >= 2  # joint2 and joint3 both overspeed
    assert all(v.joint in ("joint2", "joint3") for v in vel_violations)


def test_check_limits_reports_every_violation_not_just_first(limits):
    traj = load_trajectory(OUT_OF_LIMITS)
    # push joint3 out of bounds too, and make joint1 overspeed, to get 3+ distinct violations
    traj["points"][1]["positions"][2] = -5.0
    traj["points"][1]["t_s"] = 0.1
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    joints_with_violation = {v.joint for v in violations}
    assert {"joint2", "joint3"}.issubset(joints_with_violation)
    assert len(violations) >= 3


def test_check_limits_explicit_velocity_violation(limits):
    traj = load_trajectory(SMOKE)
    traj["points"][1]["velocities"] = [0.0, 5.0, 0.0, 0.0, 0.0, 0.0]
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    explicit_vel = [v for v in violations if v.kind == "velocity" and v.joint == "joint2" and v.value == 5.0]
    assert len(explicit_vel) == 1
    assert explicit_vel[0].bound == pytest.approx(limits["joint2"]["velocity"])


def test_check_limits_explicit_acceleration_violation(limits):
    traj = load_trajectory(SMOKE)
    traj["points"][1]["accelerations"] = [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    assert any(v.kind == "acceleration" and v.joint == "joint2" and v.value == 10.0 for v in violations)


def test_check_limits_finite_difference_acceleration_violation(limits):
    # three points, equal dt, with a sudden speed change at the interior point -> nonzero accel
    traj = load_trajectory(SMOKE)
    traj["points"] = [
        {"t_s": 0.0, "positions": [0.0] * 6},
        {"t_s": 1.0, "positions": [0.0, -0.05, 0.0, 0.0, 0.0, 0.0]},
        {"t_s": 2.0, "positions": [0.0, -5.0, 0.0, 0.0, 0.0, 0.0]},
    ]
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    # also triggers position + velocity, but acceleration must be among them
    assert any(v.kind == "acceleration" and v.joint == "joint2" for v in violations)


def test_check_limits_violation_dataclass_fields():
    v = Violation(index=1, joint="joint2", kind="position", value=0.5, bound=0.0)
    assert (v.index, v.joint, v.kind, v.value, v.bound) == (1, "joint2", "position", 0.5, 0.0)


# --------------------------------------------------------------------------------------
# §11.6 margin policy: home-approach example and counter-example from docs/INTERFACES.md #11.6
# --------------------------------------------------------------------------------------

def _home_approach_traj(speed=0.10):
    # joint2 points: -0.04, -0.02, -0.01, 0.00 at spacing giving `speed` rad/s on every segment.
    qs = [-0.04, -0.02, -0.01, 0.00]
    t = 0.0
    points = [{"t_s": t, "positions": [0.0, qs[0], 0.0, 0.0, 0.0, 0.0]}]
    for i in range(1, len(qs)):
        dt = abs(qs[i] - qs[i - 1]) / speed
        t += dt
        points.append({"t_s": t, "positions": [0.0, qs[i], 0.0, 0.0, 0.0, 0.0]})
    return points


def test_margin_home_approach_passes(limits):
    traj = load_trajectory(SMOKE)
    traj["points"] = _home_approach_traj()
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    margin_violations = [v for v in violations if v.kind == "position_margin"]
    assert margin_violations == []


def test_margin_pass_through_bound_passes(limits):
    # -0.20, -0.01, 0.00, -0.01, -0.20 at equal dt: passes through the bound and back down.
    qs = [-0.20, -0.01, 0.00, -0.01, -0.20]
    points = [{"t_s": float(i), "positions": [0.0, q, 0.0, 0.0, 0.0, 0.0]} for i, q in enumerate(qs)]
    traj = load_trajectory(SMOKE)
    traj["points"] = points
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    margin_violations = [v for v in violations if v.kind == "position_margin"]
    assert margin_violations == []


def test_margin_counter_example_violation(limits):
    # -0.03 -> -0.02 at 0.10 rad/s, then -0.02 -> 0.00 at 0.20 rad/s: violation at -0.02.
    points = [
        {"t_s": 0.0, "positions": [0.0, -0.03, 0.0, 0.0, 0.0, 0.0]},
        {"t_s": 0.1, "positions": [0.0, -0.02, 0.0, 0.0, 0.0, 0.0]},
        {"t_s": 0.15, "positions": [0.0, 0.00, 0.0, 0.0, 0.0, 0.0]},
    ]
    traj = load_trajectory(SMOKE)
    traj["points"] = points
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    margin_violations = [v for v in violations if v.kind == "position_margin" and v.joint == "joint2"]
    assert any(v.index == 1 for v in margin_violations)


def test_margin_last_point_explicit_velocity_into_bound_violates(limits):
    traj = load_trajectory(SMOKE)
    traj["points"] = [
        {"t_s": 0.0, "positions": [0.0, -0.02, 0.0, 0.0, 0.0, 0.0]},
        {"t_s": 1.0, "positions": [0.0, 0.00, 0.0, 0.0, 0.0, 0.0], "velocities": [0.0, 0.05, 0.0, 0.0, 0.0, 0.0]},
    ]
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    margin_violations = [v for v in violations if v.kind == "position_margin" and v.joint == "joint2"]
    assert any(v.index == 1 for v in margin_violations)


def test_margin_last_point_at_rest_passes(limits):
    traj = load_trajectory(SMOKE)
    traj["points"] = [
        {"t_s": 0.0, "positions": [0.0, -0.02, 0.0, 0.0, 0.0, 0.0]},
        {"t_s": 1.0, "positions": [0.0, 0.00, 0.0, 0.0, 0.0, 0.0]},
    ]
    violations = check_limits(traj, limits, position_margin_rad=0.03)
    margin_violations = [v for v in violations if v.kind == "position_margin" and v.joint == "joint2"]
    assert margin_violations == []


# --------------------------------------------------------------------------------------
# scale_time
# --------------------------------------------------------------------------------------

def test_scale_time_never_mutates(limits):
    traj = load_trajectory(OVERSPEED)
    original = copy.deepcopy(traj)
    scale_time(traj, 0.5)
    assert traj == original


def test_scale_time_rejects_out_of_range():
    traj = load_trajectory(OVERSPEED)
    for bad in (0.0, -0.1, 1.5):
        with pytest.raises(TrajectoryError, match="speed_scale"):
            scale_time(traj, bad)


def test_scale_time_applies_t_v_a_scaling():
    traj = load_trajectory(SMOKE)
    traj["points"][1]["velocities"] = [1.0] * 6
    traj["points"][1]["accelerations"] = [2.0] * 6
    scaled = scale_time(traj, 0.5)
    assert scaled["points"][1]["t_s"] == pytest.approx(traj["points"][1]["t_s"] / 0.5)
    assert scaled["points"][1]["velocities"][0] == pytest.approx(0.5)
    assert scaled["points"][1]["accelerations"][0] == pytest.approx(0.5)
    assert scaled["points"][1]["positions"] == traj["points"][1]["positions"]


def test_overspeed_fails_unscaled_passes_scaled(limits):
    traj = load_trajectory(OVERSPEED)
    unscaled_violations = check_limits(traj, limits, position_margin_rad=0.03)
    assert any(v.kind == "velocity" for v in unscaled_violations)

    scaled = scale_time(traj, 0.4)
    scaled_violations = check_limits(scaled, limits, position_margin_rad=0.03)
    assert scaled_violations == []


# --------------------------------------------------------------------------------------
# densify
# --------------------------------------------------------------------------------------

def test_densify_keeps_original_points_and_bounds_step():
    traj = load_trajectory(OVERSPEED)  # joint2/3 move 1.0 rad in one segment
    configs = densify(traj, max_step_rad=0.1)
    originals = [p["positions"] for p in traj["points"]]
    for q in originals:
        assert q in configs
    for a, b in zip(configs, configs[1:]):
        step = max(abs(x - y) for x, y in zip(a, b))
        assert step <= 0.1 + 1e-9


def test_densify_no_op_when_step_already_small():
    traj = load_trajectory(SMOKE)
    configs = densify(traj, max_step_rad=10.0)
    assert configs == [p["positions"] for p in traj["points"]]


def test_densify_rejects_non_positive_step():
    traj = load_trajectory(SMOKE)
    with pytest.raises(TrajectoryError, match="max_step_rad"):
        densify(traj, 0.0)


# --------------------------------------------------------------------------------------
# max_abs_velocity
# --------------------------------------------------------------------------------------

def test_max_abs_velocity_matches_manual_computation():
    traj = load_trajectory(OVERSPEED)
    vmax = max_abs_velocity(traj)
    assert len(vmax) == 6
    assert vmax[1] == pytest.approx(2.0)  # joint2: 1.0 rad / 0.5 s
    assert vmax[2] == pytest.approx(2.0)  # joint3
    assert vmax[0] == pytest.approx(0.0)


def test_estop_fixture_is_slow_and_within_limits(limits):
    traj = load_trajectory(ESTOP)
    assert traj["points"][-1]["t_s"] - traj["points"][0]["t_s"] >= 10.0
    assert check_limits(traj, limits, position_margin_rad=0.03) == []

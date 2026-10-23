"""crackvision.arm_ik and tools/make_fancy_trajectory.py: IK correctness, path following, timing, the shapes."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from crackvision.arm_ik import (GRIPPER_FLOOR_Z_M, RUNNER_SPEED_CAP_RAD_S, TRAJECTORY_SCHEMA, ArmIK, IKError, Piece,
                                build_trajectory, peak_speed_fraction, ramp_profile, slow_to_fit, time_pieces,
                                trajectory_doc)

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def ik() -> ArmIK:
    return ArmIK()


def test_fk_matches_kinematics_module(ik):
    from crackvision import kinematics
    q = [0.2, -0.9, -1.1, 0.4, 0.3, -0.5]
    T = kinematics.load_chain().fk(q) @ kinematics.load_end_effector().T_gripper_link_tool_tip
    np.testing.assert_allclose(ik.fk(q), T, atol=1e-12)


@pytest.mark.parametrize("q_true", [[0.1, -0.9, -1.2, 0.5, 0.2, 0.0], [-0.4, -1.3, -0.8, 0.9, -0.3, 0.1]])
def test_ik_recovers_a_reachable_pose_with_pointing(ik, q_true):
    T = ik.fk(q_true)
    r = ik.solve_global(T[:3, 3], T[:3, 0])
    assert r.ok
    T2 = ik.fk(r.q)
    assert np.linalg.norm(T2[:3, 3] - T[:3, 3]) < 2e-4
    assert math.degrees(math.acos(min(1.0, T2[:3, 0] @ T[:3, 0]))) < 0.2
    assert np.all(r.q >= ik.lo - 1e-9) and np.all(r.q <= ik.hi + 1e-9)


def test_unreachable_target_is_refused(ik):
    with pytest.raises(IKError):
        ik.solve_global([1.5, 0.0, 0.3])


def test_follow_refuses_a_branch_jump(ik):
    pts = np.array([[0.30, 0.0, 0.45], [0.30, 0.0, 0.46]])
    q0 = ik.solve_global(pts[0], [1, 0, 0]).q
    with pytest.raises(IKError, match="jumps"):
        ik.follow(pts, np.tile([1.0, 0, 0], (2, 1)), q0, max_joint_step_rad=1e-4)


def test_ramp_profile_is_smooth_and_reaches_the_end():
    T, s = ramp_profile(0.2, 0.05, 0.8)
    assert T == pytest.approx(0.2 / 0.05 + 0.8)
    ts = np.linspace(0, T, 2001)
    ss = np.array([s(t) for t in ts])
    v = np.diff(ss) / np.diff(ts)
    assert ss[0] == 0 and ss[-1] == pytest.approx(0.2)
    assert np.all(v >= -1e-9) and v.max() <= 0.05 + 1e-6
    assert v[0] < 1e-3 and v[-1] < 1e-3                       # at rest at both ends


def test_ramp_profile_short_path_lowers_the_speed():
    T, s = ramp_profile(0.01, 0.05, 0.8)
    assert s(T) == pytest.approx(0.01)
    assert T == pytest.approx(1.6)


def test_time_pieces_stops_only_where_asked():
    line = lambda a, b: np.linspace(a, b, 30)
    p1 = Piece(line([0.3, -0.05, 0.45], [0.3, 0.0, 0.45]), None, 0.05, stop=False)
    p2 = Piece(line([0.3, 0.0, 0.45], [0.3, 0.05, 0.45]), None, 0.05, stop=True)
    t, pts, nrm, spans = time_pieces([p1, p2])
    assert nrm is None and len(spans) == 1                    # merged into one profile: no stop in the middle
    speed = np.linalg.norm(np.diff(pts, axis=0), axis=1) / np.diff(t)
    mid = len(t) // 2
    assert speed[mid] == pytest.approx(0.05, rel=0.05)


def test_mixed_pointing_is_refused():
    a = Piece(np.zeros((2, 3)) + [0.3, 0, 0.4], np.tile([1.0, 0, 0], (2, 1)))
    b = Piece(np.zeros((2, 3)) + [0.3, 0, 0.4], None)
    with pytest.raises(ValueError):
        time_pieces([a, b])


def test_slow_to_fit_caps_every_joint():
    t = np.arange(0, 2.0, 0.05)
    q = np.zeros((len(t), 6))
    q[:, 1] = np.sin(t * 3)                                    # 3 rad/s peak on J2: far over 0.6
    t2, f = slow_to_fit(t, q, 0.7)
    assert f > 1
    assert peak_speed_fraction(t2, q).max() == pytest.approx(0.7, rel=1e-6)


def test_build_trajectory_refuses_the_floor(ik):
    pts = np.linspace([0.30, -0.03, 0.125], [0.30, 0.03, 0.125], 40)
    pc = Piece(pts, np.tile([0, 0, -1.0], (40, 1)), 0.03)
    q0 = ik.solve_global(pts[0], [0, 0, -1.0]).q
    with pytest.raises(IKError, match="below"):
        build_trajectory(ik, [pc], q0)


def test_trajectory_doc_shape():
    t = np.array([0.0, 0.5])
    doc = trajectory_doc(t, np.zeros((2, 6)), producer="x", purpose="test", notes="n")
    assert doc["schema"] == TRAJECTORY_SCHEMA and doc["joint_names"] == [f"joint{i}" for i in range(1, 7)]
    assert [p["t_s"] for p in doc["points"]] == [0.0, 0.5]
    with pytest.raises(ValueError):
        trajectory_doc(t + 1, np.zeros((2, 6)), producer="x", purpose="test", notes="n")


def _fancy():
    spec = importlib.util.spec_from_file_location("make_fancy_trajectory", REPO / "tools" / "make_fancy_trajectory.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize("shape", ["figure8", "star", "heart", "spiral", "rose", "lissajous", "wave", "helix",
                                   "raster", "crack", "showreel"])
def test_every_fancy_shape_is_within_the_runner_limits(shape):
    mod = _fancy()
    doc, rep = mod.build(shape, *mod.DEFAULTS[shape], 0.7)
    assert rep.max_tip_error_mm < 1.0
    assert max(rep.peak_speed_fraction_of_cap) <= 0.7 + 1e-6
    assert rep.min_gripper_z_m >= GRIPPER_FLOOR_Z_M + 0.01
    q = np.array([p["positions"] for p in doc["points"]])
    t = np.array([p["t_s"] for p in doc["points"]])
    assert t[0] == 0 and np.all(np.diff(t) > 0)
    assert (np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None] <= 0.7 * RUNNER_SPEED_CAP_RAD_S + 1e-6).all()


def test_star_stops_at_every_point():
    mod = _fancy()
    ik = ArmIK()
    pieces = mod.wall_pieces("star", *mod.DEFAULTS["star"][:4], 0.05)
    _, _, _, spans = time_pieces(pieces)
    assert len(spans) == 5                                     # five edges, each its own profile

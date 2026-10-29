"""tools/make_j4_tuning_trajectory.py: wrist-only sweeps and steps at two poses, inside the guard windows (0.05 rad margin
at the poses), clear of the table and the guard's keep-outs, under 60 % of the runner caps, <= 4 min, fold to fold."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from crackvision.arm_ik import RUNNER_SPEED_CAP_RAD_S, check_motion_clearance, peak_speed_fraction

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("make_j4_tuning_trajectory", REPO / "tools" / "make_j4_tuning_trajectory.py")
mj = importlib.util.module_from_spec(spec)
sys.modules["make_j4_tuning_trajectory"] = mj
spec.loader.exec_module(mj)
ms = mj.ms

pytestmark = pytest.mark.skipif(not mj.HEART.exists(), reason="no ~/rebot_setup/trajectories/fancy/heart.json")


@pytest.fixture(scope="module")
def built():
    return mj.make()


@pytest.fixture(scope="module")
def checker():
    return ms.Checker()


def tq(doc):
    return np.array([p["t_s"] for p in doc["points"]]), np.array([p["positions"] for p in doc["points"]])


def windows(doc):
    return {b["name"]: b for b in doc["j4_tune"]["poses"]}


def test_schema_fold_to_fold_and_duration(built):
    doc = built["doc"]
    t, q = tq(doc)
    assert doc["schema"] == "crackvision.joint_trajectory/1" and doc["purpose"] == "test"
    assert doc["joint_names"] == [f"joint{i}" for i in range(1, 7)]
    assert np.abs(q[0]).max() == 0 and np.abs(q[-1]).max() == 0
    assert t[0] == 0 and np.all(np.diff(t) > 0) and t[-1] <= mj.MAX_MINUTES * 60
    assert set(windows(doc)) == {"heart", "crack"}


def test_every_sample_inside_the_windows_and_clear(built, checker):
    t, q = tq(built["doc"])
    w = windows(built["doc"])
    for i, qq in enumerate(q):
        in_pose = any(b["window_s"][0] - 1e-9 <= t[i] <= b["window_s"][1] + 1e-9 for b in w.values())
        assert not checker.problems(qq, margin=ms.LIMIT_MARGIN if in_pose else 0.0), (t[i], qq)
    clear = check_motion_clearance(checker.ik, q, floor_z_m=ms.BODY_Z_MIN)
    assert clear["min_body_z_m"] >= ms.BODY_Z_MIN
    assert built["report"]["min_gripper_z_m"] >= ms.GRIPPER_Z_MIN
    lo, hi, _ = ms.guard_limits()
    assert (q.min(axis=0) >= lo).all() and (q.max(axis=0) <= hi).all()


def test_speeds_under_60_percent_of_the_caps(built):
    t, q = tq(built["doc"])
    frac = peak_speed_fraction(t, q)
    assert frac.max() <= ms.SPEED_FRACTION + 1e-6 and frac[3] <= mj.J4_SPEED_FRACTION + 1e-6
    assert built["report"]["peak_j4_speed_in_the_wrist_moves_rad_s"] <= max(mj.SPEEDS) * 1.02
    assert RUNNER_SPEED_CAP_RAD_S[3] * mj.J4_SPEED_FRACTION > max(mj.SPEEDS)


def test_the_pose_blocks_move_the_wrist_only(built):
    t, q = tq(built["doc"])
    for b in windows(built["doc"]).values():
        m = (t >= b["window_s"][0]) & (t <= b["window_s"][1])
        c = np.array(b["q"])
        assert np.abs(q[m][:, [0, 4, 5]]).max() == 0
        assert np.abs(q[m][:, 1:3] - c[1:3]).max() < 1e-12                      # J2/J3 held
        assert q[m][:, 3].min() >= c[3] - mj.AMP - 1e-9 and q[m][:, 3].max() <= c[3] + mj.AMP + 1e-9
        assert np.isclose(q[m][:, 3].min(), c[3] - mj.AMP) and np.isclose(q[m][:, 3].max(), c[3] + mj.AMP)


def test_sweeps_cruise_at_their_speeds_in_both_directions(built):
    t, q = tq(built["doc"])
    for b in windows(built["doc"]).values():
        sw = [s for s in b["sections"] if s["kind"] == "sweep"]
        assert [round(s["speed_rad_s"], 3) for s in sw] == [round(v, 3) for v in mj.SPEEDS]
        for s in sw:
            m = (t >= s["t0_s"] - 1e-9) & (t <= s["t1_s"] + 1e-9)
            v = np.diff(q[m][:, 3]) / np.diff(t[m])
            assert abs(v.max() - s["speed_rad_s"]) < 1e-3 and abs(v.min() + s["speed_rad_s"]) < 1e-3
            # at the cruise speed except for the 0.5 s ramps at each corner (half of a 2 s leg at 0.2 rad/s)
            assert (np.abs(np.abs(v) - s["speed_rad_s"]) < 1e-3).mean() > 0.4
            assert abs(q[m][0, 3] - b["q"][3]) < 1e-9 and abs(q[m][-1, 3] - b["q"][3]) < 1e-9
            # the corners are blended: no velocity jump bigger than a ramp step
            assert np.abs(np.diff(v)).max() < 1.875 * s["speed_rad_s"] / mj.RAMP_S * mj.DT * 1.2


def test_steps_and_holds(built):
    t, q = tq(built["doc"])
    for b in windows(built["doc"]).values():
        st = next(s for s in b["sections"] if s["kind"] == "steps")
        m = (t >= st["t0_s"] - 1e-9) & (t <= st["t1_s"] + 1e-9)
        tt, j4 = t[m], q[m][:, 3]
        holds = [(tt[i], j4[i]) for i in range(len(tt) - 1)
                 if abs(tt[i + 1] - tt[i] - mj.STEP_HOLD_S) < 1e-6 and j4[i] == j4[i + 1]]
        assert len(holds) == len(mj.STEP_PATTERN)
        levels = np.round(np.diff([b["q"][3]] + [h[1] for h in holds]) / mj.STEP).astype(int).tolist()
        assert levels == list(mj.STEP_PATTERN)                                  # 0.05 rad up and down
        assert sum(1 for d in levels if d > 0) == 6 and sum(1 for d in levels if d < 0) == 6


def test_crack_pose_comes_from_the_survey_and_passes_its_filters(built, checker):
    crack = windows(built["doc"])["crack"]
    c = np.array(crack["q"])
    assert c[3] == mj.CRACK_J4
    poses, _ = ms.select_poses(checker, 20, ms.crack_anchors(), ms.Risk(mj.gmod.DEFAULT_MODEL))
    assert any(np.allclose(p[1:3], c[1:3], atol=1e-3) for p in poses)
    assert mj.pose_problems(checker, ms.Risk(mj.gmod.DEFAULT_MODEL), c) == []
    heart = windows(built["doc"])["heart"]
    assert heart["q"][1:3] == mj.heart_pose()[1:3].tolist() and heart["q"][3] == mj.HEART_J4


def test_leg_is_at_rest_at_both_ends_and_exact():
    x, V = mj.leg(0.2, -0.1, 0.1)
    assert x[0] == 0.2 and abs(x[-1] + 0.1) < 1e-12 and abs(V - 0.1) < 1e-9
    v = np.diff(x) / mj.DT
    assert abs(v[0]) < 0.01 and abs(v[-1]) < 0.01 and np.all(v <= 1e-12)
    j4, Vs = mj.sweep(0.2, 0.05)
    assert j4[0] == j4[-1] == 0.2 and np.isclose(j4.max(), 0.35) and np.isclose(j4.min(), 0.05)


def test_the_written_file_in_the_runner_workspace_is_valid_if_present(checker):
    if not mj.OUT.exists():
        pytest.skip("no ~/rebot_setup/trajectories/j4_tune.json on this machine")
    doc = json.loads(mj.OUT.read_text())
    t, q = tq(doc)
    assert peak_speed_fraction(t, q).max() <= ms.SPEED_FRACTION + 1e-6 and t[-1] <= mj.MAX_MINUTES * 60
    assert all(not checker.problems(qq, margin=0.0) for qq in q[::5])
    assert {b["name"] for b in doc["j4_tune"]["poses"]} == {"heart", "crack"}

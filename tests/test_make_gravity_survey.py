"""tools/make_gravity_survey.py: the survey poses and moves stay inside the guard windows, clear of the table and the
guard's keep-outs, under 60 % of the runner's speed caps, and every pose is held twice, once from each side."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from crackvision import gravity_model as gmod
from crackvision.arm_ik import check_motion_clearance, peak_speed_fraction

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("make_gravity_survey", REPO / "tools" / "make_gravity_survey.py")
ms = importlib.util.module_from_spec(spec)
sys.modules["make_gravity_survey"] = ms
spec.loader.exec_module(ms)


@pytest.fixture(scope="module")
def survey():
    return ms.make(n=8, max_minutes=10.0, model_path=None)


@pytest.fixture(scope="module")
def checker():
    return ms.Checker()


def tq(doc):
    return np.array([p["t_s"] for p in doc["points"]]), np.array([p["positions"] for p in doc["points"]])


def test_every_sample_is_inside_the_windows_and_clear(survey, checker):
    for name, doc in survey["files"].items():
        t, q = tq(doc)
        assert doc["schema"] == "crackvision.joint_trajectory/1" and doc["purpose"] == "test"
        assert np.abs(q[0]).max() == 0 and np.abs(q[-1]).max() == 0          # starts and ends at the fold
        for qq in q:
            assert not checker.problems(qq, margin=0.0), (name, qq, checker.problems(qq, margin=0.0))
        clear = check_motion_clearance(checker.ik, q, floor_z_m=ms.BODY_Z_MIN)
        assert clear["min_body_z_m"] >= ms.BODY_Z_MIN
        assert survey["report"]["files"][name]["min_gripper_z_m"] >= ms.GRIPPER_Z_MIN
        lo, hi, _ = ms.guard_limits()
        assert (q.min(axis=0) >= lo).all() and (q.max(axis=0) <= hi).all()


def test_speeds_stay_under_60_percent_of_the_caps(survey):
    for doc in survey["files"].values():
        t, q = tq(doc)
        assert peak_speed_fraction(t, q).max() <= ms.SPEED_FRACTION + 1e-6
        assert np.all(np.diff(t) > 0) and t[0] == 0.0


def test_each_pose_is_held_twice_from_opposite_sides_inside_the_margins(survey, checker):
    for doc in survey["files"].values():
        t, q = tq(doc)
        holds = doc["gravity_survey"]["holds"]
        by_pose: dict = {}
        for h in holds:
            by_pose.setdefault(h["pose"], []).append(h)
            assert h["t1_s"] - h["t0_s"] == pytest.approx(ms.HOLD_S)
            i0, i1 = np.searchsorted(t, h["t0_s"] - 1e-9), np.searchsorted(t, h["t1_s"] - 1e-9)
            assert np.allclose(q[i0], q[i1]) and np.allclose(q[i0], h["q"], atol=1e-4)
            assert not checker.problems(np.array(h["q"]))                    # 0.05 rad inside the guard windows
            # the move into the hold came from below (from below) or above (from above) on J2-J4
            before = q[i0 - 2]
            d = np.sign(q[i0][1:4] - before[1:4])
            assert (d == (1 if h["approach"] == "from below" else -1)).all(), (h, before)
        assert all(sorted(x["approach"] for x in v) == ["from above", "from below"] for v in by_pose.values())
        poses = np.array([v[0]["q"] for v in by_pose.values()])
        assert poses[:, 1].min() >= -1.9 - 1e-9 and poses[:, 1].max() <= -0.6 + 1e-9
        assert poses[:, 3].min() >= -1e-9 and poses[:, 3].max() <= 1.4 + 1e-9
        assert np.abs(poses[:, [0, 4, 5]]).max() == 0


def test_checker_rejects_what_the_guard_or_the_table_would(checker):
    assert checker.problems(np.array([0, -0.9, -1.2, 0.35, 0, 0])) == []           # the heart's start pose
    assert any("gripper_link" in p for p in checker.problems(np.array([0, -0.6, 0, 0, 0, 0])))   # J2 alone dips
    assert any("joint window" in p for p in checker.problems(np.array([0, -0.9, -1.2, 1.55, 0, 0])))
    assert any("joint window" in p for p in checker.problems(np.array([0, -0.02, -1.2, 0.3, 0, 0])))
    assert checker.problems(np.zeros(6)) == []                                       # the fold itself


def test_risk_filter_uses_the_model(tmp_path, checker):
    heavy = {"schema": gmod.SCHEMA, "urdf": {"sha256": gmod.file_sha256(gmod.DEFAULT_URDF)},
             "bodies": [{"link": "gripper_link", "xyz": [-0.05, 0.0, 0.0], "kg": 1.5}], "scale": {"j2": 1.0}}
    p = tmp_path / "heavy.yaml"
    p.write_text(gmod.dump_model_text(heavy, "test model"))
    risk = ms.Risk(p)
    far = np.array([0, -1.9, -1.9, 0.0, 0, 0])
    assert any("predicted hold error" in w or "caps" in w for w in risk.problems(checker, far))
    assert ms.Risk(None).problems(checker, far) == []
    poses, rejected = ms.select_poses(checker, 6, [], risk)
    for q in poses:
        assert np.abs(risk.sag(q)[1:4]).max() * 1000 <= ms.SAG_MAX_MRAD


def test_a_long_survey_is_split_nearer_poses_first():
    res = ms.make(n=8, max_minutes=1.0, model_path=None)
    assert set(res["files"]) == {"gravity_survey_a", "gravity_survey_b"}
    a, b = (res["report"]["files"][k]["poses"] for k in ("gravity_survey_a", "gravity_survey_b"))
    assert sorted(a + b) == list(range(8))
    poses = np.array([p["q"] for p in res["report"]["poses"]])
    dist = np.linalg.norm(poses[:, 1:4] - ms.DATA_CENTRE[1:4], axis=1)
    assert dist[a].max() <= dist[b].min() + 1e-9


def test_the_written_survey_in_the_runner_workspace_is_valid_if_present(checker):
    f = ms.OUT_DIR / "gravity_survey.json"
    if not f.exists():
        pytest.skip("no ~/rebot_setup/trajectories/gravity_survey.json on this machine")
    doc = json.loads(f.read_text())
    t, q = tq(doc)
    assert peak_speed_fraction(t, q).max() <= ms.SPEED_FRACTION + 1e-6
    assert all(not checker.problems(qq, margin=0.0) for qq in q[::5])

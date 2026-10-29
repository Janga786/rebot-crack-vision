"""crackvision.crack_trace: scan pose, the settle detector on the runner's state file, trace planning, projection."""

from __future__ import annotations

import json
import logging
import os
import threading
import time

import numpy as np
import pytest

from crackvision import crack_trace as ct
from crackvision import kinematics
from crackvision.arm_ik import ArmIK

LOG = logging.getLogger("test_crack_trace")
Q_SCAN = np.array([-0.023, -1.283, -1.136, 1.162, 0.006, 0.023])


def state(q, kind="trajectory", vel=0.0, mode="path", wall=None):
    return {"schema": ct.STATE_SCHEMA, "wall_time_ns": wall if wall is not None else time.time_ns(), "mode": mode,
            "seg": 1, "segment_kind": kind, "joint_names": [f"joint{i}" for i in range(1, 7)],
            "positions_rad": list(map(float, q)), "ref_rad": list(map(float, q)), "velocity_rad_s": [vel] * 6}


def test_scan_pose_looks_at_the_target_from_the_view_distance():
    target = np.array([0.30, 0.0, 0.0])
    q, info = ct.scan_pose(target, np.array([0, 0, 1.0]), 0.25)
    ik = ArmIK()
    _, T_opt = ct.camera_frames(kinematics.load_end_effector())
    T = ik.fk_frame(q, T_opt)
    np.testing.assert_allclose(T[:3, 3], target + [0, 0, 0.25], atol=1e-3)
    assert T[:3, 2] @ np.array([0, 0, -1.0]) > np.cos(np.radians(0.5))   # optical +z looks straight down
    assert info["gripper_link_z_m"] > ct.TIP_Z_MIN_M


def test_settled_needs_rest_at_the_scan_pose_inside_the_dwell():
    assert ct.settled([state(Q_SCAN)] * 3, Q_SCAN, 0.04, 0.05)
    assert not ct.settled([], Q_SCAN, 0.04, 0.05)
    assert not ct.settled([state(Q_SCAN, kind="approach")], Q_SCAN, 0.04, 0.05)
    assert not ct.settled([state(Q_SCAN, vel=0.2)], Q_SCAN, 0.04, 0.05)
    assert not ct.settled([state(Q_SCAN + 0.1)], Q_SCAN, 0.04, 0.05)
    assert not ct.settled([state(Q_SCAN), state(Q_SCAN + 0.01)], Q_SCAN, 0.04, 0.05)   # still creeping


def _publisher(path, script):
    def run():
        for s, dt in script:
            tmp = str(path) + ".tmp"
            with open(tmp, "w") as fh:
                json.dump(dict(s, wall_time_ns=time.time_ns()), fh)
            os.replace(tmp, path)
            time.sleep(dt)
    th = threading.Thread(target=run, daemon=True)
    th.start()
    return th


def test_wait_settled_returns_once_the_arm_rests(tmp_path):
    f = tmp_path / "arm_state.json"
    script = [(state(Q_SCAN * 0.5, kind="approach", vel=0.3), 0.02)] * 10 + [(state(Q_SCAN), 0.02)] * 60
    _publisher(f, script)
    s = ct.wait_settled(f, Q_SCAN, timeout_s=5, hold_s=0.3, poll_s=0.01)
    assert s["segment_kind"] == "trajectory"


def test_wait_settled_gives_up_when_the_arm_returns(tmp_path):
    f = tmp_path / "arm_state.json"
    _publisher(f, [(state(Q_SCAN * 0.5, kind="approach", vel=0.3), 0.02)] * 5
               + [(state(Q_SCAN * 0.5, kind="return", vel=0.3), 0.02)] * 20)
    with pytest.raises(TimeoutError, match="left the scan pose"):
        ct.wait_settled(f, Q_SCAN, timeout_s=5, hold_s=0.3, poll_s=0.01)


def test_wait_settled_times_out_without_a_state_file(tmp_path):
    with pytest.raises(TimeoutError):
        ct.wait_settled(tmp_path / "none.json", Q_SCAN, timeout_s=0.2, poll_s=0.01)


def _paths3d_line(z=0.0, n=60, sigma=0.005):
    ys = np.linspace(-0.05, 0.05, n)
    pts = np.stack([0.30 + 0.01 * np.sin(ys * 40), ys, np.full(n, z)], 1)
    wps = [{"phase": "trace", "surface_point_m": p.tolist(), "normal_base": [0, 0, 1.0],
            "sigma_along_normal_m": sigma} for p in pts]
    return {"components": [{"component_id": 0, "polylines": [{"kind": "main", "segments": [{"waypoints": wps}]}]}]}, pts


def _plan(doc, **kw):
    args = dict(hover_m=0.05, approach_m=0.04, trace_speed=0.02, transit_speed=0.04, max_tilt_deg=30.0,
                min_length_m=0.01, all_paths=False, max_segments=3, smooth=1, flat=True, speed_fraction=0.6, log=LOG)
    return ct.plan_trace(doc, **(args | kw))


def test_plan_trace_follows_the_crack_above_the_floor():
    doc, pts = _paths3d_line()
    t, q, plan = _plan(doc)
    seg = plan["segments"][0]
    np.testing.assert_allclose(np.array(seg["surface_points_m"]), pts, atol=1e-5)   # stored rounded to 0.01 mm
    assert seg["hover_m"] >= ct.TIP_Z_MIN_M                    # on the table, the floor sets the hover
    ik = ArmIK()
    tips = np.array([ik.fk(qq)[:3, 3] for qq in q])
    d = np.linalg.norm(tips[:, None, :] - np.array(seg["tool_points_m"])[None], axis=2).min(axis=0)
    assert d.max() < 0.002                                     # every planned tool point is visited
    assert plan["report"]["min_gripper_z_m"] >= ct.TIP_Z_MIN_M - 0.005
    assert max(plan["report"]["peak_speed_fraction_of_cap"]) <= 0.6 + 1e-6


def test_hover_covers_three_sigma_of_the_calibration_uncertainty():
    doc, _ = _paths3d_line(z=0.10, sigma=0.03)                  # a raised surface: the floor does not bind
    _, _, plan = _plan(doc, hover_m=0.02)
    assert plan["segments"][0]["hover_m"] >= 0.09 - 1e-9


def test_short_cracks_are_skipped():
    doc, _ = _paths3d_line()
    with pytest.raises(ct.IKError, match="no crack segment"):
        _plan(doc, min_length_m=0.5)


def test_tilted_leans_away_from_the_base():
    d = ct.tilted(np.array([0, 0, 1.0]), np.array([0.3, 0.0, 0.0]), 20)
    assert d[2] == pytest.approx(-np.cos(np.radians(20)))
    assert d[0] == pytest.approx(np.sin(np.radians(20)))
    np.testing.assert_allclose(ct.tilted(np.array([0, 0, 1.0]), np.array([0.3, 0, 0]), 0), [0, 0, -1])


def test_project_inverts_a_pinhole():
    T = np.eye(4)
    intr = {"fx": 400.0, "fy": 400.0, "ppx": 320.0, "ppy": 240.0}
    uv = ct.project(T, intr, np.array([[0.1, -0.05, 0.5]]))
    np.testing.assert_allclose(uv[0], [320 + 400 * 0.2, 240 - 400 * 0.1])


def test_locate_crack_lifts_the_centroid_with_the_surface_depth_and_the_pose():
    h, w = 480, 848
    mask = np.zeros((h, w), np.uint8)
    mask[200:204, 300:500] = 1                                   # a 200 x 4 px crack, centred at (399.5, 201.5)
    depth = np.full((h, w), 2500, np.uint16)                     # 0.25 m at 1e-4 m / unit
    T = np.eye(4)
    T[:3, 3] = [0.3, 0.0, 0.25]
    T[:3, :3] = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1.0]])   # optical z looks down
    intr = {"fx": 436.0, "fy": 436.0, "ppx": 424.0, "ppy": 240.0}
    r = ct.locate_crack(mask, depth, 1e-4, intr, T)
    assert r["depth_m"] == pytest.approx(0.25)
    u, v = 399.5, 201.5
    expect = T @ np.array([(u - 424) / 436 * 0.25, (v - 240) / 436 * 0.25, 0.25, 1.0])
    np.testing.assert_allclose(r["target_base_m"], expect[:3].round(4), atol=1e-4)
    assert abs(r["target_base_m"][2]) < 1e-3 and not r["touches_border"]
    assert r["extent_base_m"] == pytest.approx(200 / 436 * 0.25, rel=0.02)


def test_locate_crack_flags_the_border_and_refuses_tiny_or_depthless_cracks():
    h, w = 480, 848
    depth = np.full((h, w), 2500, np.uint16)
    intr = {"fx": 436.0, "fy": 436.0, "ppx": 424.0, "ppy": 240.0}
    edge = np.zeros((h, w), np.uint8)
    edge[200:204, 0:200] = 1
    assert ct.locate_crack(edge, depth, 1e-4, intr, np.eye(4))["touches_border"]
    tiny = np.zeros((h, w), np.uint8)
    tiny[200:202, 300:310] = 1
    assert ct.locate_crack(tiny, depth, 1e-4, intr, np.eye(4)) is None
    mid = np.zeros((h, w), np.uint8)
    mid[200:204, 300:500] = 1
    assert ct.locate_crack(mid, np.zeros((h, w), np.uint16), 1e-4, intr, np.eye(4)) is None


def test_plan_search_stops_are_continuous_clear_and_rest_where_they_say():
    t, q, stops = ct.plan_search([0.26, 0.36], [-0.15, 0.0, 0.15], 0.25, 3.0)
    assert len(stops) == 6
    assert np.all(np.diff(t) > 0)
    v = np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]
    assert (v <= 0.6 * np.array([0.6, 0.6, 0.6, 1.2, 1.2, 1.2]) + 1e-6).all()
    assert np.abs(np.diff(q, axis=0)).max() < 0.1               # no jump between samples (the stop pose = leg arrival)
    for s in stops:
        i = np.searchsorted(t, 0.5 * (s["t0"] + s["t1"]))
        np.testing.assert_allclose(q[i], s["q"], atol=1e-9)     # at rest at the stop's pose for the whole dwell
        assert s["t1"] - s["t0"] == pytest.approx(3.0, abs=0.06)


def test_plan_search_refuses_an_unreachable_area():
    with pytest.raises(ct.IKError, match="reachable"):
        ct.plan_search([1.2, 1.4], [0.0, 0.1], 0.25, 2.0)


def test_render_view_puts_the_print_on_the_plane_with_exact_depth():
    ik = ArmIK()
    q, _ = ct.scan_pose(np.array([0.30, 0.0, 0.0]), np.array([0, 0, 1.0]), 0.25, ik)
    ci = {"width": 848, "height": 480, "fx": 436.0, "fy": 436.0, "ppx": 424.0, "ppy": 240.0}
    photo = np.full((100, 200, 3), 50, np.uint8)
    rgb, depth, cov = ct.render_view(photo, q, ci, 1e-4, np.array([0.30, 0.0, 0.0]), np.array([0, 0, 1.0]), 0.20, ik)
    assert 0.1 < cov < 0.3 and rgb.shape == (480, 848, 3)
    assert abs(int(depth[240, 424]) * 1e-4 - 0.25) < 0.01


def test_read_state_ignores_a_state_file_older_than_the_capture_start(tmp_path):
    """audit 2026-10-29 item 3: a stale arm_state.json from an earlier run is never taken for the pose now."""
    f = tmp_path / "arm_state.json"
    old = time.time_ns() - 60 * 10**9
    f.write_text(json.dumps(state(Q_SCAN, wall=old)))
    os.utime(f, ns=(old, old))
    t_start = time.time_ns()
    assert ct.read_state(f) is not None                       # without a start time: as before
    assert ct.read_state(f, t_start) is None                  # older mtime -> ignored
    f.write_text(json.dumps(state(Q_SCAN, wall=old)))         # fresh mtime but an old timestamp inside -> ignored
    assert ct.read_state(f, t_start) is None
    t0 = time.time_ns()                                       # capture starts, then the runner writes a state whose
    f.write_text(json.dumps(state(Q_SCAN)))                   # mtime sits a few ms BEHIND the wall clock (the fs clock
    os.utime(f, ns=(t0 - 5 * 10**6, t0 - 5 * 10**6))          # lags ~1 ms, measured 2026-10-29) -> still used
    assert ct.read_state(f, t0) is not None
    f.write_text(json.dumps(state(Q_SCAN)))                   # fresh file, fresh timestamp -> used
    assert ct.read_state(f, t_start) is not None


def test_wait_settled_does_not_settle_on_a_stale_state_file(tmp_path):
    f = tmp_path / "arm_state.json"
    old = time.time_ns() - 60 * 10**9
    f.write_text(json.dumps(state(Q_SCAN, mode="final_hold", wall=old)))   # the end of an earlier run
    os.utime(f, ns=(old, old))
    with pytest.raises(TimeoutError, match="left the scan pose"):            # read as now: wrong
        ct.wait_settled(f, Q_SCAN, timeout_s=0.4, hold_s=0.3, poll_s=0.01)
    with pytest.raises(TimeoutError, match="did not settle"):                # ignored: keeps waiting
        ct.wait_settled(f, Q_SCAN, timeout_s=0.4, hold_s=0.3, poll_s=0.01, not_before_ns=time.time_ns())

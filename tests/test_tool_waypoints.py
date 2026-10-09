"""tests/test_tool_waypoints.py — GEOM-08.6 tool_tip trace/approach/retract waypoints."""

from __future__ import annotations

import numpy as np
import pytest

from crackvision.lift3d import LiftedPolyline
from crackvision.tool_waypoints import (
    WaypointParams,
    capture_tool_z,
    segment_waypoints,
)


def _make_lifted(p_base, n_base, cov_base, interpolated=None) -> LiftedPolyline:
    p_base = np.asarray(p_base, dtype=np.float64)
    n_base = np.asarray(n_base, dtype=np.float64)
    cov_base = np.asarray(cov_base, dtype=np.float64)
    n = p_base.shape[0]
    if interpolated is None:
        interpolated = np.zeros(n, dtype=bool)
    else:
        interpolated = np.asarray(interpolated, dtype=bool)
    filler1 = np.full(n, np.nan)
    filler3 = np.full((n, 3), np.nan)
    return LiftedPolyline(
        rows=np.zeros(n, dtype=np.int64),
        cols=np.zeros(n, dtype=np.int64),
        valid=np.ones(n, dtype=bool),
        reason=np.full(n, "", dtype=object),
        interpolated=interpolated,
        depth_m=filler1,
        fraction_valid=filler1,
        p_optical=filler3,
        p_base=p_base,
        n_base=n_base,
        normal_rms_m=filler1,
        sigma_normal_rad=filler1,
        cov_base=cov_base,
        segments=[(0, n - 1)],
        dropped_segments=0,
    )


def _check_rotation_basics(wp, n_out, tol=1e-9):
    R = __import__("scipy.spatial.transform", fromlist=["Rotation"]).Rotation.from_quat(
        wp.quat_xyzw
    ).as_matrix()
    x_axis = R[:, 0]
    assert np.allclose(x_axis, -np.asarray(n_out), atol=1e-9)
    assert abs(np.linalg.norm(wp.quat_xyzw) - 1.0) < 1e-9
    assert np.allclose(
        wp.position - wp.surface_point, wp.clearance_m * np.asarray(n_out), atol=1e-9
    )


SMALL_COV = np.diag([1e-8, 1e-8, 1e-8])


# --- 1. Planar straight crack ------------------------------------------------------------


def test_planar_straight_crack_basic_geometry():
    n = 5
    xs = np.linspace(0.0, 0.01, n)
    p_base = np.column_stack([xs, np.zeros(n), np.zeros(n)])
    n_base = np.tile([0.0, 0.0, 1.0], (n, 1))
    cov_base = np.tile(SMALL_COV, (n, 1, 1))
    lifted = _make_lifted(p_base, n_base, cov_base)

    params = WaypointParams(trace_clearance_m=0.01, approach_clearance_m=0.04, waypoint_spacing_m=0.002)
    wps = segment_waypoints(lifted, (0, n - 1), np.array([1.0, 0.0, 0.0]), 0.0, params)

    trace = [w for w in wps if w.phase == "trace"]
    for w in trace:
        _check_rotation_basics(w, w.normal)

    # endpoints included
    assert np.allclose(trace[0].surface_point, p_base[0])
    assert np.allclose(trace[-1].surface_point, p_base[-1])

    # consecutive spacing bound
    for a, b in zip(trace[:-1], trace[1:]):
        d = np.linalg.norm(np.asarray(b.surface_point) - np.asarray(a.surface_point))
        assert d <= params.waypoint_spacing_m + 1e-9


# --- 2. Arc on a cylinder (roll-continuity bound) ---------------------------------------


def test_cylinder_arc_roll_continuity():
    radius = 0.1
    n = 60
    thetas = np.linspace(0.0, np.pi / 2, n)
    p_base = np.column_stack([radius * np.cos(thetas), radius * np.sin(thetas), np.zeros(n)])
    n_base = np.column_stack([np.cos(thetas), np.sin(thetas), np.zeros(n)])
    cov_base = np.tile(SMALL_COV, (n, 1, 1))
    lifted = _make_lifted(p_base, n_base, cov_base)

    params = WaypointParams(waypoint_spacing_m=0.002)
    wps = segment_waypoints(lifted, (0, n - 1), np.array([0.0, 0.0, 1.0]), 0.0, params)
    trace = [w for w in wps if w.phase == "trace"]

    for w in trace:
        _check_rotation_basics(w, w.normal)

    def angle_between(u, v):
        c = np.clip(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v)), -1.0, 1.0)
        return np.arccos(c)

    from scipy.spatial.transform import Rotation

    zs = [Rotation.from_quat(w.quat_xyzw).as_matrix()[:, 2] for w in trace]
    for i in range(len(trace) - 1):
        normal_angle = angle_between(trace[i].normal, trace[i + 1].normal)
        z_angle = angle_between(zs[i], zs[i + 1])
        assert z_angle <= normal_angle + 1e-9


# --- 3. Degenerate Z0 (capture tool Z parallel to the normal) ---------------------------


def test_degenerate_capture_z_falls_back():
    n = 3
    p_base = np.column_stack([np.linspace(0.0, 0.004, n), np.zeros(n), np.zeros(n)])
    n_base = np.tile([0.0, 0.0, 1.0], (n, 1))
    cov_base = np.tile(SMALL_COV, (n, 1, 1))
    lifted = _make_lifted(p_base, n_base, cov_base)

    # capture tool Z parallel to the normal -> first fallback (base +Z) is also parallel to
    # X = (0, 0, -1), so this exercises the second fallback (base +X) too.
    capture_z = np.array([0.0, 0.0, 1.0])
    wps = segment_waypoints(lifted, (0, n - 1), capture_z, 0.0, WaypointParams())
    trace = [w for w in wps if w.phase == "trace"]

    from scipy.spatial.transform import Rotation

    R0 = Rotation.from_quat(trace[0].quat_xyzw).as_matrix()
    assert np.allclose(R0[:, 0], np.array([0.0, 0.0, -1.0]), atol=1e-9)
    assert np.allclose(R0[:, 2], np.array([1.0, 0.0, 0.0]), atol=1e-9)
    for w in trace:
        _check_rotation_basics(w, w.normal)


# --- 4. Spacing and endpoint inclusion ---------------------------------------------------


def test_spacing_and_endpoint_inclusion_irregular():
    p_base = np.array(
        [[0.0, 0.0, 0.0], [0.0015, 0.0, 0.0], [0.0057, 0.0, 0.0], [0.0057 + 0.0033, 0.0, 0.0]]
    )
    n_base = np.tile([0.0, 0.0, 1.0], (4, 1))
    cov_base = np.tile(SMALL_COV, (4, 1, 1))
    lifted = _make_lifted(p_base, n_base, cov_base)

    params = WaypointParams(waypoint_spacing_m=0.002)
    wps = segment_waypoints(lifted, (0, 3), np.array([1.0, 0.0, 0.0]), 0.0, params)
    trace = [w for w in wps if w.phase == "trace"]

    assert np.allclose(trace[0].surface_point, p_base[0])
    assert np.allclose(trace[-1].surface_point, p_base[-1], atol=1e-9)
    for a, b in zip(trace[:-1], trace[1:]):
        d = np.linalg.norm(np.asarray(b.surface_point) - np.asarray(a.surface_point))
        assert d <= params.waypoint_spacing_m + 1e-9


def test_single_point_segment_yields_one_trace_waypoint():
    p_base = np.array([[0.0, 0.0, 0.0]])
    n_base = np.array([[0.0, 0.0, 1.0]])
    cov_base = np.array([SMALL_COV])
    lifted = _make_lifted(p_base, n_base, cov_base)

    wps = segment_waypoints(lifted, (0, 0), np.array([1.0, 0.0, 0.0]), 0.0, WaypointParams())
    trace = [w for w in wps if w.phase == "trace"]
    assert len(trace) == 1


# --- 5. Approach / retract geometry ------------------------------------------------------


def test_approach_and_retract_bracket_each_segment():
    n = 4
    p_base = np.column_stack([np.linspace(0.0, 0.006, n), np.zeros(n), np.zeros(n)])
    n_base = np.tile([0.0, 0.0, 1.0], (n, 1))
    cov_base = np.tile(SMALL_COV, (n, 1, 1))
    lifted = _make_lifted(p_base, n_base, cov_base)

    params = WaypointParams(trace_clearance_m=0.01, approach_clearance_m=0.04)
    wps = segment_waypoints(lifted, (0, n - 1), np.array([1.0, 0.0, 0.0]), 0.0, params)

    assert wps[0].phase == "approach"
    assert wps[-1].phase == "retract"
    assert sum(1 for w in wps if w.phase == "approach") == 1
    assert sum(1 for w in wps if w.phase == "retract") == 1

    trace = [w for w in wps if w.phase == "trace"]
    approach, retract = wps[0], wps[-1]

    assert approach.clearance_m == pytest.approx(0.04)
    assert retract.clearance_m == pytest.approx(0.04)
    assert np.allclose(approach.quat_xyzw, trace[0].quat_xyzw)
    assert np.allclose(retract.quat_xyzw, trace[-1].quat_xyzw)
    assert np.allclose(approach.surface_point, trace[0].surface_point)
    assert np.allclose(retract.surface_point, trace[-1].surface_point)
    assert np.allclose(
        approach.position - approach.surface_point, 0.04 * approach.normal, atol=1e-9
    )
    assert np.allclose(
        retract.position - retract.surface_point, 0.04 * retract.normal, atol=1e-9
    )


# --- 6. Sigma / within_budget boundary ---------------------------------------------------


def test_sigma_along_normal_formula_and_within_budget_boundary():
    n_base_vec = np.array([0.0, 0.0, 1.0])
    clearance = 0.01
    tool_sigma = 0.001

    # choose sigma_normal_sq so that 3*sqrt(sigma_normal_sq + tool_sigma^2) == clearance exactly
    target_sigma = clearance / 3.0
    sigma_normal_sq = target_sigma**2 - tool_sigma**2
    assert sigma_normal_sq > 0
    cov_boundary = np.diag([0.0, 0.0, sigma_normal_sq])

    lifted = _make_lifted(
        [[0.0, 0.0, 0.0]], [n_base_vec], [cov_boundary],
    )
    params = WaypointParams(trace_clearance_m=clearance)
    wps = segment_waypoints(lifted, (0, 0), np.array([1.0, 0.0, 0.0]), tool_sigma, params)
    trace_wp = wps[1]

    expected_sigma = np.sqrt(n_base_vec @ cov_boundary @ n_base_vec + tool_sigma**2)
    assert trace_wp.sigma_along_normal_m == pytest.approx(expected_sigma)
    assert trace_wp.sigma_along_normal_m == pytest.approx(clearance / 3.0)
    assert trace_wp.within_budget is True

    # nudge the variance up slightly -> 3 sigma exceeds clearance -> not within budget
    cov_over = np.diag([0.0, 0.0, sigma_normal_sq * 1.1 + 1e-12])
    lifted_over = _make_lifted([[0.0, 0.0, 0.0]], [n_base_vec], [cov_over])
    wps_over = segment_waypoints(lifted_over, (0, 0), np.array([1.0, 0.0, 0.0]), tool_sigma, params)
    assert wps_over[1].within_budget is False


# --- capture_tool_z -----------------------------------------------------------------------


class _StubChain:
    def fk(self, q):
        return np.eye(4)


class _StubEndEffector:
    T_gripper_link_tool_tip = np.array(
        [
            [0.0, -1.0, 0.0, 0.01],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.02],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def test_capture_tool_z_reads_fk_tool_tip_z_column():
    z = capture_tool_z(_StubChain(), _StubEndEffector(), [0.0] * 6)
    assert np.allclose(z, np.array([0.0, 0.0, 1.0]))

"""tests/test_uncertainty_mc.py — GEOM-09.3.1 Monte-Carlo cross-check of crackvision.uncertainty.

N_SAMPLES = 1500 for every MC call below: each sample costs one `geometry.deproject_pixels`
call (vectorised across all samples) plus one `KinematicChain.fk` call (an unavoidable
per-sample python loop, ~6 small matrix multiplies). 1500 samples keeps the empirical
covariance's relative error small (~sqrt(2/1500) ~= 3.6%) while keeping this file's handful of
MC calls well under a minute of wall-clock time.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from crackvision import kinematics
from crackvision.geometry import Intrinsics
from crackvision.uncertainty import PointBreakdown, decompose_point, decompose_waypoint, fk_covariance
from crackvision.uncertainty_mc import mc_point_covariance, mc_waypoint_sigma

N_SAMPLES = 1500

CAM_POS_M = 0.010  # kinematics._NOMINAL_CAMERA_SIGMA_POS_M
CAM_ROT_RAD_SMALL = math.radians(5.0)  # kinematics._NOMINAL_CAMERA_SIGMA_ROT_RAD
TOOL_SIGMA_POS_M = 0.005  # kinematics._NOMINAL_TOOL_SIGMA_POS_M
SIGMA_JOINT_RAD = 0.001
SIGMA_PX = 1.0


def make_intrinsics(**overrides) -> Intrinsics:
    intr = Intrinsics(
        width=640, height=480, fx=500.0, fy=500.0, ppx=320.0, ppy=240.0,
        model="none", coeffs=(0.0, 0.0, 0.0, 0.0, 0.0),
    )
    if overrides:
        from dataclasses import replace

        intr = replace(intr, **overrides)
    return intr


def _chain_and_effector():
    chain = kinematics.load_chain()
    end_effector = kinematics.load_end_effector()
    T_camera_optical = kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME
    q = np.array([-0.3, -1.0, -1.0, 0.0, 0.0, 0.0])
    return chain, end_effector, T_camera_optical, q


def _analytic_point_cov(intr, row, col, depth_m, chain, end_effector, T_camera_optical, q, cam_rot_rad):
    from crackvision.lift3d import CalibSigmas

    p_opt = (float((col - intr.ppx) * depth_m / intr.fx), float((row - intr.ppy) * depth_m / intr.fy), float(depth_m))
    T_base_optical = chain.fk(q) @ end_effector.T_gripper_link_camera_link @ T_camera_optical
    calib = CalibSigmas(cam_pos_m=CAM_POS_M, cam_rot_rad=cam_rot_rad)
    bd = decompose_point(
        p_opt, intr.fx, intr.fy, T_base_optical, SIGMA_PX, calib,
        valid=True, interpolated=False, cov_base=np.eye(3),
    )

    def T_base_optical_of_q(qq):
        return chain.fk(qq) @ end_effector.T_gripper_link_camera_link @ T_camera_optical

    fk_cov = fk_covariance(T_base_optical_of_q, q, p_opt, SIGMA_JOINT_RAD)
    bd = PointBreakdown(pixel=bd.pixel, depth=bd.depth, handeye=bd.handeye, fk=fk_cov)
    return bd, p_opt, T_base_optical


# --- 1. agreement case: small, realistic sigmas, near-linear regime --------------------


def test_mc_point_agrees_with_analytic_in_near_linear_regime():
    intr = make_intrinsics()
    chain, end_effector, T_camera_optical, q = _chain_and_effector()
    row, col, depth_m = 260, 360, 0.25

    bd, p_opt, T_base_optical = _analytic_point_cov(
        intr, row, col, depth_m, chain, end_effector, T_camera_optical, q, CAM_ROT_RAD_SMALL,
    )

    result = mc_point_covariance(
        row=row, col=col, depth_m=depth_m, intrinsics=intr, sigma_px=SIGMA_PX,
        chain=chain, q=q, T_gripper_camera=end_effector.T_gripper_link_camera_link,
        T_camera_optical=T_camera_optical, sigma_joint_rad=SIGMA_JOINT_RAD,
        cam_pos_m=CAM_POS_M, cam_rot_rad=CAM_ROT_RAD_SMALL,
        n_samples=N_SAMPLES, seed=0, analytic_cov_base=bd.total,
    )

    assert result.flagged is False
    assert 0.5 <= result.ratio_trace <= 2.0
    assert 0.5 <= result.ratio_eig_max <= 2.0


def test_mc_waypoint_agrees_with_analytic_in_near_linear_regime():
    intr = make_intrinsics()
    chain, end_effector, T_camera_optical, q = _chain_and_effector()
    # Closer to the camera than the point-only case above: at this normal (near the optical
    # axis), the analytic handeye term's isotropic surrogate is weakest relative to the
    # rotation-induced term (which is tangential, not radial, to first order) -- shrinking
    # depth keeps the translation (isotropic) contribution dominant so the two stay in
    # agreement even along a near-radial normal.
    row, col, depth_m = 240, 320, 0.10

    bd, p_opt, T_base_optical = _analytic_point_cov(
        intr, row, col, depth_m, chain, end_effector, T_camera_optical, q, CAM_ROT_RAD_SMALL,
    )
    normal = np.array([0.0, 0.0, -1.0])  # outward-facing in base_link given T_base_optical = eye-ish
    normal = T_base_optical[:3, :3] @ np.array([0.0, 0.0, -1.0])
    normal = normal / np.linalg.norm(normal)
    clearance_m = 0.01

    wp_bd = decompose_waypoint(bd, normal, tool_sigma_pos_m=TOOL_SIGMA_POS_M)
    analytic_sigma = math.sqrt(wp_bd.total)

    result = mc_waypoint_sigma(
        row=row, col=col, depth_m=depth_m, intrinsics=intr, sigma_px=SIGMA_PX,
        chain=chain, q=q, T_gripper_camera=end_effector.T_gripper_link_camera_link,
        T_camera_optical=T_camera_optical, sigma_joint_rad=SIGMA_JOINT_RAD,
        cam_pos_m=CAM_POS_M, cam_rot_rad=CAM_ROT_RAD_SMALL,
        normal=normal, clearance_m=clearance_m, tool_sigma_pos_m=TOOL_SIGMA_POS_M,
        n_samples=N_SAMPLES, seed=0, analytic_sigma_along_normal_m=analytic_sigma,
    )

    assert result.flagged is False
    assert 0.5 <= result.ratio_along_normal <= 2.0


# --- 2. disagreement case: large hand-eye rotation sigma breaks linearity --------------


def test_mc_point_disagrees_with_analytic_when_handeye_rotation_is_large():
    intr = make_intrinsics()
    chain, end_effector, T_camera_optical, q = _chain_and_effector()
    row, col, depth_m = 260, 360, 0.45  # far from the camera -> strongly nonlinear rotation effect
    cam_rot_rad_large = math.radians(45.0)

    bd, p_opt, T_base_optical = _analytic_point_cov(
        intr, row, col, depth_m, chain, end_effector, T_camera_optical, q, cam_rot_rad_large,
    )

    result = mc_point_covariance(
        row=row, col=col, depth_m=depth_m, intrinsics=intr, sigma_px=SIGMA_PX,
        chain=chain, q=q, T_gripper_camera=end_effector.T_gripper_link_camera_link,
        T_camera_optical=T_camera_optical, sigma_joint_rad=SIGMA_JOINT_RAD,
        cam_pos_m=CAM_POS_M, cam_rot_rad=cam_rot_rad_large,
        n_samples=N_SAMPLES, seed=0, analytic_cov_base=bd.total,
    )

    assert result.flagged is True
    assert not (0.5 <= result.ratio_trace <= 2.0 and 0.5 <= result.ratio_eig_max <= 2.0)


# --- 3. determinism: same seed -> bit-identical, different seed -> not identical ------


def test_mc_point_covariance_is_seed_deterministic():
    intr = make_intrinsics()
    chain, end_effector, T_camera_optical, q = _chain_and_effector()
    row, col, depth_m = 260, 360, 0.25
    bd, p_opt, T_base_optical = _analytic_point_cov(
        intr, row, col, depth_m, chain, end_effector, T_camera_optical, q, CAM_ROT_RAD_SMALL,
    )

    kwargs = dict(
        row=row, col=col, depth_m=depth_m, intrinsics=intr, sigma_px=SIGMA_PX,
        chain=chain, q=q, T_gripper_camera=end_effector.T_gripper_link_camera_link,
        T_camera_optical=T_camera_optical, sigma_joint_rad=SIGMA_JOINT_RAD,
        cam_pos_m=CAM_POS_M, cam_rot_rad=CAM_ROT_RAD_SMALL,
        n_samples=N_SAMPLES, analytic_cov_base=bd.total,
    )

    r1 = mc_point_covariance(seed=0, **kwargs)
    r2 = mc_point_covariance(seed=0, **kwargs)
    assert np.array_equal(r1.cov_base_mc, r2.cov_base_mc)
    assert r1.ratio_trace == r2.ratio_trace
    assert r1.ratio_eig_max == r2.ratio_eig_max

    r3 = mc_point_covariance(seed=1, **kwargs)
    assert not np.array_equal(r1.cov_base_mc, r3.cov_base_mc)


# --- 4. tcp term only affects mc_waypoint_sigma -----------------------------------------


def test_tcp_term_only_affects_waypoint_sigma_not_point_covariance():
    intr = make_intrinsics()
    chain, end_effector, T_camera_optical, q = _chain_and_effector()
    row, col, depth_m = 260, 360, 0.25
    bd, p_opt, T_base_optical = _analytic_point_cov(
        intr, row, col, depth_m, chain, end_effector, T_camera_optical, q, CAM_ROT_RAD_SMALL,
    )
    normal = T_base_optical[:3, :3] @ np.array([0.0, 0.0, -1.0])
    normal = normal / np.linalg.norm(normal)
    clearance_m = 0.01

    # mc_point_covariance has no tcp parameter at all -- a bare point can't be affected by it.
    point_kwargs = dict(
        row=row, col=col, depth_m=depth_m, intrinsics=intr, sigma_px=SIGMA_PX,
        chain=chain, q=q, T_gripper_camera=end_effector.T_gripper_link_camera_link,
        T_camera_optical=T_camera_optical, sigma_joint_rad=SIGMA_JOINT_RAD,
        cam_pos_m=CAM_POS_M, cam_rot_rad=CAM_ROT_RAD_SMALL,
        n_samples=N_SAMPLES, seed=0, analytic_cov_base=bd.total,
    )
    r_point_1 = mc_point_covariance(**point_kwargs)
    r_point_2 = mc_point_covariance(**point_kwargs)
    assert np.array_equal(r_point_1.cov_base_mc, r_point_2.cov_base_mc)

    waypoint_kwargs = dict(
        row=row, col=col, depth_m=depth_m, intrinsics=intr, sigma_px=SIGMA_PX,
        chain=chain, q=q, T_gripper_camera=end_effector.T_gripper_link_camera_link,
        T_camera_optical=T_camera_optical, sigma_joint_rad=SIGMA_JOINT_RAD,
        cam_pos_m=CAM_POS_M, cam_rot_rad=CAM_ROT_RAD_SMALL,
        normal=normal, clearance_m=clearance_m,
        n_samples=N_SAMPLES, seed=0, analytic_sigma_along_normal_m=1.0,  # unused for this assertion
    )
    r_tool_zero = mc_waypoint_sigma(tool_sigma_pos_m=0.0, **waypoint_kwargs)
    r_tool_nonzero = mc_waypoint_sigma(tool_sigma_pos_m=0.02, **waypoint_kwargs)

    assert r_tool_nonzero.sigma_along_normal_mc_m > r_tool_zero.sigma_along_normal_mc_m

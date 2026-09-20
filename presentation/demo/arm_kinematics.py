#!/usr/bin/env python
"""Self-contained forward + inverse kinematics for the reBot B601-DM (6-DOF arm).

Chain source: `~/rebot_ws/src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf`
(read-only reference — not imported, values transcribed and verified below).
`base_link -> joint1..joint6 -> gripper_joint (fixed) -> gripper_link` is the tool frame.

Dependency-light by design: numpy only (no scipy, no URDF parser). Every joint
origin/rpy/axis/limit below was read directly off the URDF's <origin>, <axis> and
<limit> tags for joint1..joint6 and the fixed gripper_joint; see the docstring of
`presentation/demo/path_to_joint_trajectory.py` for the cross-check against the
task's rounded chain description (they agree to the printed precision).

URDF convention used throughout: a joint's <origin xyz rpy> is a fixed transform
from the parent frame (translate by xyz, then rotate by rpy, both expressed in the
parent frame), followed by a variable rotation of `theta` about the joint's local
`axis` vector. In matrix form, per joint i:

    T_i = Trans(xyz_i) @ R(rpy_i) @ Rot(axis_i, theta_i)

and R(rpy) is the ROS/URDF fixed-axis (extrinsic) convention R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

TOOL BORESIGHT AXIS — determined empirically, not assumed:
    Evaluating fk([0,0,0,0,0,0]) gives a tool rotation matrix whose 3rd column
    (the tool frame's local +Z axis, expressed in world/base coordinates) is
    (0, 0, -1) to numerical precision. That is: at the all-zero joint configuration
    the tool's local **+Z axis points straight down** (world -Z), i.e. away from the
    flange and into the coupon below. This is NOT true in general for other joint
    configurations (a random sweep over q2..q6 shows the tool orientation is a
    genuine function of all 6 joints), so IK below actively drives this same local
    +Z axis to stay aligned with world -Z at every solved waypoint — see `ik()`
    for why only this one axis (not the full 3x3 frame) is constrained.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

# --- URDF chain (base_link -> ... -> gripper_link), transcribed + verified ---
# Each entry: xyz origin (m), rpy origin (rad, fixed-axis XYZ), local joint axis,
# and (lower, upper) limits (rad). Order = base_link -> link1 -> ... -> link6.
JOINT_ORIGINS_XYZ = [
    (-8.416e-05, 0.0, 0.08465),          # joint1
    (0.020084, 0.031625, 0.05555),       # joint2
    (-0.264, 0.0, 0.0),                  # joint3
    (0.2426, -0.054, -0.001625),         # joint4
    (0.078308, -0.0375, -0.03),          # joint5
    (0.023692, 0.0, 0.04),               # joint6
]
JOINT_ORIGINS_RPY = [
    (0.0, 0.0, 0.0),        # joint1
    (-1.5708, 0.0, 0.0),    # joint2
    (0.0, 0.0, 0.0),        # joint3
    (0.0, 0.0, 0.0),        # joint4
    (-1.5708, 0.0, 0.0),    # joint5
    (0.0, 1.5708, 0.0),     # joint6
]
JOINT_AXES = [
    (0.0, 0.0, 1.0),    # joint1
    (0.0, 0.0, -1.0),   # joint2
    (0.0, 0.0, 1.0),    # joint3
    (0.0, 0.0, 1.0),    # joint4
    (0.0, 0.0, 1.0),    # joint5
    (0.0, 0.0, 1.0),    # joint6
]
JOINT_LIMITS = np.array([
    (-2.8, 2.8),     # joint1
    (-3.14, 0.0),    # joint2
    (-3.14, 0.0),    # joint3
    (-1.87, 1.57),   # joint4
    (-1.57, 1.57),   # joint5
    (-3.14, 3.14),   # joint6
])
NUM_JOINTS = 6

# fixed gripper_joint: link6 -> gripper_link (the tool frame)
GRIPPER_XYZ = np.array([0.0, 0.0, 0.15971])
GRIPPER_RPY = (0.0, -1.5708, 0.0)


# --- rotation primitives -----------------------------------------------------
def rotx(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def roty(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rotz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis (extrinsic) XYZ convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    return rotz(yaw) @ roty(pitch) @ rotx(roll)


def axis_angle_matrix(axis: Sequence[float], theta: float) -> np.ndarray:
    """Rodrigues' formula: rotation by `theta` about unit vector `axis`."""
    a = np.asarray(axis, dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-12:
        return np.eye(3)
    a = a / n
    K = np.array([
        [0.0, -a[2], a[1]],
        [a[2], 0.0, -a[0]],
        [-a[1], a[0], 0.0],
    ])
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def homogeneous(R: np.ndarray, t: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# Precomputed per-joint fixed origin transforms (translate then rotate by rpy).
_ORIGIN_T = [homogeneous(rpy_matrix(*rpy), xyz)
             for xyz, rpy in zip(JOINT_ORIGINS_XYZ, JOINT_ORIGINS_RPY)]
_GRIPPER_T = homogeneous(rpy_matrix(*GRIPPER_RPY), GRIPPER_XYZ)


def fk_frames(q: Sequence[float]) -> Tuple[List[np.ndarray], np.ndarray]:
    """Return ([T_1..T_6] cumulative link frames, T_tool) for joint angles q.

    T_i is the *full* transform after joint i's own rotation is applied. As shown
    in the module docstring's derivation, a joint's own axis rotation leaves its
    own axis vector and its own origin position unchanged, so T_i doubles as the
    "before this joint's rotation" frame needed for the analytic Jacobian below.
    """
    assert len(q) == NUM_JOINTS
    T = np.eye(4)
    frames: List[np.ndarray] = []
    for i in range(NUM_JOINTS):
        T_axis = homogeneous(axis_angle_matrix(JOINT_AXES[i], q[i]), (0.0, 0.0, 0.0))
        T = T @ _ORIGIN_T[i] @ T_axis
        frames.append(T)
    T_tool = T @ _GRIPPER_T
    return frames, T_tool


def fk(q: Sequence[float]) -> np.ndarray:
    """Forward kinematics: 6 joint angles -> 4x4 tool-frame pose (world/base frame)."""
    _, T_tool = fk_frames(q)
    return T_tool


def geometric_jacobian(q: Sequence[float]) -> np.ndarray:
    """Analytic 6x6 geometric Jacobian of the tool frame (rows: [vx,vy,vz,wx,wy,wz]).

    For revolute joint i with world-frame axis z_i and origin position p_i:
        J_v_i = z_i x (p_tool - p_i)
        J_w_i = z_i
    z_i and p_i are read off the *full* cumulative frame T_i (see fk_frames):
    p_i = T_i[:3,3] (a joint's own rotation is about its own origin, so this does
    not depend on q_i), and z_i = T_i[:3,:3] @ axis_i_local (a joint's own rotation
    leaves a vector along its own axis unchanged, so this equals the axis direction
    both before and after applying q_i).
    """
    frames, T_tool = fk_frames(q)
    p_tool = T_tool[:3, 3]
    J = np.zeros((6, NUM_JOINTS))
    for i in range(NUM_JOINTS):
        T_i = frames[i]
        p_i = T_i[:3, 3]
        z_i = T_i[:3, :3] @ np.asarray(JOINT_AXES[i], dtype=float)
        z_i = z_i / np.linalg.norm(z_i)
        J[:3, i] = np.cross(z_i, p_tool - p_i)
        J[3:, i] = z_i
    return J


def clamp_to_limits(q: np.ndarray) -> np.ndarray:
    return np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])


# World-frame direction the tool boresight (tool frame's local +Z axis — see the
# module docstring's empirical derivation) must point along: straight down at the
# coupon below.
TOOL_DOWN_AXIS = np.array([0.0, 0.0, -1.0])


@dataclass
class IKResult:
    q: np.ndarray
    converged: bool
    pos_err_m: float
    axis_err_deg: float
    iterations: int


def _boresight_and_jacobian(q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """One FK+Jacobian pass. Returns (p_tool, z_tool, Jv (3x6), Jz (3x6)).

    Reuses `geometric_jacobian` (whose bottom 3 rows are exactly the per-joint
    world-frame axes z_i, i.e. J_w) rather than re-deriving the frame loop: the
    axis-Jacobian column is Jz_i = z_i x z_tool = J_w[:, i] x z_tool.
    """
    T_tool = fk(q)
    p_tool = T_tool[:3, 3]
    z_tool = T_tool[:3, 2]  # boresight: tool frame's local +Z axis, in world coords
    J = geometric_jacobian(q)
    Jv, Jw = J[:3, :], J[3:, :]
    Jz = np.cross(Jw.T, z_tool).T
    return p_tool, z_tool, Jv, Jz


def ik(target_pos: Sequence[float], target_axis: Sequence[float], q_seed: Sequence[float],
       max_iter: int = 300, tol_pos: float = 1e-4, tol_axis_deg: float = 0.05,
       damping: float = 0.02, max_step: float = 0.2) -> IKResult:
    """Damped least-squares IK: 3-DOF position + 2-DOF tool-boresight-axis pointing.

    We deliberately constrain only the tool's local +Z axis direction (5 real
    constraints: 3 position + 2 pointing), NOT a full 3x3 orientation. Task
    wording ("position + tool-axis orientation") and an earlier full-orientation
    attempt both point the same way: pinning the whole frame (including rotation
    about the boresight, i.e. tool "roll") burns the arm's one redundant DOF on a
    direction nothing downstream cares about, and empirically made several
    waypoints of the demo path unreachable — a multi-start (150 random
    restarts/waypoint) search under a full-orientation constraint could not close
    position error at several waypoints even though position alone is trivially
    reachable there. Leaving roll-about-boresight free resolves that: every one
    of the 20 waypoints in the demo path converges to sub-millimeter position
    error with just the axis pinned down (see path_to_joint_trajectory.py's
    printed summary). Rotation about the boresight is resolved implicitly by the
    minimum-norm damped least-squares step, not chosen explicitly.

    Error/Jacobian derivation (standard resolved-rate IK):
      pos:  e_pos = target_pos - p_tool ; d(e_pos)/dq_i = J_v_i (the usual
            geometric-Jacobian column), matching the dq = J^T(JJ^T+lam^2 I)^-1 e
            update used below.
      axis: let z(q) be the tool's world-frame boresight axis. Its rate of
            change under joint velocities is dz/dq_i = axis_i_world x z(q) (a
            body-fixed unit vector under an angular-velocity field). Define
            e_axis = target_axis - z(q); then d(e_axis)/dq_i = -axis_i_world x
            z(q) = z(q) x axis_i_world, i.e. the Jacobian column used below
            (`Jz[:, i] = cross(z_i, z_tool)`).

    Joint limits are enforced by clamping q after every step, so the returned
    solution is always inside JOINT_LIMITS even when it has not converged.

    `max_step` caps the per-iteration joint step (rad, per joint). With 5
    constraints and 6 joints the null space is 1-D (tool roll), and a large
    unclamped DLS step from a big initial residual can wander along that null
    space to a *different* valid solution branch than the seed's — technically
    still correct (position + axis both land on target) but not the "nearest"
    solution to the seed, which breaks path continuity between adjacent
    waypoints. Capping the step size keeps each iteration a small local move, so
    the solver settles near the seed's own branch instead of drifting to a
    distant one. This does not change what counts as converged, only which
    (equally valid) solution is found when several exist.
    """
    q = clamp_to_limits(np.array(q_seed, dtype=float))
    target_pos = np.asarray(target_pos, dtype=float)
    target_axis = np.asarray(target_axis, dtype=float)
    target_axis = target_axis / np.linalg.norm(target_axis)
    best_q, best_score = q.copy(), np.inf
    n_iter = 0
    for n_iter in range(1, max_iter + 1):
        p_tool, z_tool, Jv, Jz = _boresight_and_jacobian(q)
        e_pos = target_pos - p_tool
        e_axis = target_axis - z_tool
        pos_err = float(np.linalg.norm(e_pos))
        axis_err_deg = float(np.degrees(np.arccos(np.clip(np.dot(z_tool, target_axis), -1.0, 1.0))))
        score = pos_err + axis_err_deg / 1000.0  # position dominates; axis breaks ties
        if score < best_score:
            best_score, best_q = score, q.copy()
            best_pos_err, best_axis_err = pos_err, axis_err_deg
        if pos_err < tol_pos and axis_err_deg < tol_axis_deg:
            return IKResult(q=q, converged=True, pos_err_m=pos_err,
                             axis_err_deg=axis_err_deg, iterations=n_iter)
        J = np.vstack([Jv, Jz])
        e = np.concatenate([e_pos, e_axis])
        lam2 = damping * damping
        dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(6), e)
        dq = np.clip(dq, -max_step, max_step)
        q = clamp_to_limits(q + dq)
    return IKResult(q=best_q, converged=False, pos_err_m=best_pos_err,
                     axis_err_deg=best_axis_err, iterations=n_iter)


def solve_path(targets: Sequence[Sequence[float]],
                q_seed: Optional[Sequence[float]] = None,
                target_axis: Sequence[float] = TOOL_DOWN_AXIS) -> List[IKResult]:
    """Solve IK for an ordered list of xyz position targets, tool pointing `target_axis`.

    Each waypoint is seeded from the previous waypoint's solution (continuity —
    no elbow flips between adjacent waypoints); the very first waypoint is seeded
    from `q_seed` (or a default reach-forward-and-down posture if omitted).
    """
    if q_seed is None:
        q_seed = np.array([0.0, -1.4, -1.3, 0.0, -1.2, 0.0])
    q_prev = np.array(q_seed, dtype=float)
    out: List[IKResult] = []
    for pos in targets:
        res = ik(pos, target_axis, q_prev)
        out.append(res)
        q_prev = res.q
    return out


if __name__ == "__main__":
    # tiny self-check: FK at zero should show the tool +Z axis pointing down.
    T0 = fk(np.zeros(NUM_JOINTS))
    print("fk(0) tool position:", np.round(T0[:3, 3], 5))
    print("fk(0) tool +Z axis (world):", np.round(T0[:3, 2], 5), "<- should be ~(0,0,-1)")

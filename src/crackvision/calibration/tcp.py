"""crackvision.calibration.tcp — pivot calibration solver for the tool-centre point (GEOM-06).

Implements the classic "pivot calibration" algorithm: the operator holds the physical tool tip
(the point `docs/adr/012-frames-and-conventions.md` calls `TCP`, i.e. `gripper_tcp`) fixed against
a stationary reference point (e.g. a divot/cone fixture) while sweeping the arm through a set of
orientations. At every pose the physical tip touches the *same* fixed point in the world, but its
offset from `tool0` (`gripper_link`, per ADR-012) is *also* fixed and unknown — the two constant
unknowns (`p_tcp` in `tool0`'s frame, `p_pivot` in `base_link`) are recovered by least squares from
the recorded `T_base_link_tool0` pose at each orientation:

    p_pivot = R_i @ p_tcp + t_i   for every pose i

Only `p_tcp`'s *translation* is observable this way — pivoting a point about itself carries no
information about the TCP frame's orientation, so this solver (and GEOM-06/07 generally) treats
`T_tool0_TCP` as translation-only, consistent with ADR-012's `TCP`/`gripper_tcp` definition (a
point, not an independently-oriented frame).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

# rebotarm.urdf.xacro:8-12's fixed `gripper_tcp` offset relative to `gripper_link` (== `tool0`,
# ADR-012): xyz="-0.0443 0 0", i.e. -0.0443 m along `gripper_link`'s local +X. This is CAD/URDF
# *prior evidence*, not a calibrated fact (ADR-012, "Boresight / TCP — status of prior evidence")
# — GEOM-07 is the only card allowed to report it as measured.
PRIOR_TCP_OFFSET_M = (-0.0443, 0.0, 0.0)

# Below 4 poses the 6-unknown system (p_tcp, p_pivot) has no more equations than unknowns, so a
# solve "succeeds" numerically but reports zero residual regardless of how wrong the poses'
# orientation diversity is. Require at least one pose of headroom so residuals are ever meaningful.
MIN_POSES = 4


@dataclass(frozen=True)
class PoseSample:
    """One recorded `T_base_link_tool0` pose (ADR-012 `T_a_b` convention: `p_base = T · p_tool0`).

    `quat_xyzw` is ROS/`tf2`-order `(x, y, z, w)` per ADR-012; `translation_m` is metres.
    """

    quat_xyzw: tuple[float, float, float, float]
    translation_m: tuple[float, float, float]

    def rotation_matrix(self) -> np.ndarray:
        return Rotation.from_quat(np.asarray(self.quat_xyzw, dtype=np.float64)).as_matrix()


@dataclass(frozen=True)
class PivotCalibrationResult:
    """Solved pivot calibration: TCP offset, fixed pivot point, and per-pose residuals."""

    tcp_offset_m: np.ndarray  # (3,) p_tcp, i.e. the translation part of T_tool0_TCP
    pivot_point_m: np.ndarray  # (3,) the fixed physical pivot point, expressed in base_link
    residuals_m: np.ndarray  # (n, 3) per-pose residual vector: (R_i @ p_tcp + t_i) - pivot_point_m
    residual_norms_m: np.ndarray  # (n,) Euclidean norm of each row of residuals_m
    rms_residual_m: float
    max_residual_m: float
    num_poses: int


def pivot_calibrate(poses: Sequence[PoseSample]) -> PivotCalibrationResult:
    """Least-squares pivot calibration: recover `p_tcp` and the fixed pivot point from `poses`.

    Stacks `R_i @ p_tcp - p_pivot = -t_i` over every pose into one `3n x 6` linear system and
    solves it with `numpy.linalg.lstsq`. Requires `len(poses) >= MIN_POSES`; the procedure
    (`docs/calibration/TCP_BORESIGHT_PROCEDURE.md`) additionally requires orientation diversity
    across the collected poses, which this function cannot itself verify from pose count alone —
    a degenerate (all-near-identical-orientation) pose set still solves numerically but does not
    actually constrain `p_tcp` and `p_pivot` independently.
    """
    n = len(poses)
    if n < MIN_POSES:
        raise ValueError(
            f"pivot_calibrate needs at least {MIN_POSES} poses with diverse orientations, got {n}"
        )

    a_matrix = np.zeros((3 * n, 6), dtype=np.float64)
    b_vector = np.zeros(3 * n, dtype=np.float64)
    for i, pose in enumerate(poses):
        rotation = pose.rotation_matrix()
        translation = np.asarray(pose.translation_m, dtype=np.float64)
        a_matrix[3 * i : 3 * i + 3, 0:3] = rotation
        a_matrix[3 * i : 3 * i + 3, 3:6] = -np.eye(3)
        b_vector[3 * i : 3 * i + 3] = -translation

    solution, _residual_sum, _rank, _singular_values = np.linalg.lstsq(
        a_matrix, b_vector, rcond=None
    )
    tcp_offset_m = solution[0:3]
    pivot_point_m = solution[3:6]

    residuals_m = (a_matrix @ solution - b_vector).reshape(n, 3)
    residual_norms_m = np.linalg.norm(residuals_m, axis=1)

    return PivotCalibrationResult(
        tcp_offset_m=tcp_offset_m,
        pivot_point_m=pivot_point_m,
        residuals_m=residuals_m,
        residual_norms_m=residual_norms_m,
        rms_residual_m=float(np.sqrt(np.mean(residual_norms_m**2))),
        max_residual_m=float(np.max(residual_norms_m)),
        num_poses=n,
    )


@dataclass(frozen=True)
class BoresightCheck:
    """Comparison of a measured TCP offset against the CAD/URDF prior (ADR-012)."""

    measured_offset_m: np.ndarray  # (3,)
    measured_magnitude_m: float
    prior_offset_m: np.ndarray  # (3,)
    prior_magnitude_m: float
    angle_from_prior_deg: float  # angle between the two offset directions
    magnitude_delta_m: float  # measured_magnitude_m - prior_magnitude_m (signed)


def check_boresight(
    tcp_offset_m: np.ndarray, prior_offset_m: tuple[float, float, float] = PRIOR_TCP_OFFSET_M
) -> BoresightCheck:
    """Compare a solved `tcp_offset_m` against `prior_offset_m` (default: the URDF CAD prior).

    Reports the angle between the two offset directions and the magnitude difference so GEOM-07
    can record how far the measured boresight axis and TCP distance are from the simulation-derived
    prior, per ADR-012's "prior evidence, not calibrated fact" caveat — this function never decides
    pass/fail on its own; the thresholds belong in the procedure document.
    """
    measured = np.asarray(tcp_offset_m, dtype=np.float64)
    prior = np.asarray(prior_offset_m, dtype=np.float64)

    measured_magnitude = float(np.linalg.norm(measured))
    prior_magnitude = float(np.linalg.norm(prior))

    if measured_magnitude == 0.0 or prior_magnitude == 0.0:
        angle_deg = float("nan")
    else:
        cos_angle = np.clip(
            np.dot(measured, prior) / (measured_magnitude * prior_magnitude), -1.0, 1.0
        )
        angle_deg = float(np.degrees(np.arccos(cos_angle)))

    return BoresightCheck(
        measured_offset_m=measured,
        measured_magnitude_m=measured_magnitude,
        prior_offset_m=prior,
        prior_magnitude_m=prior_magnitude,
        angle_from_prior_deg=angle_deg,
        magnitude_delta_m=measured_magnitude - prior_magnitude,
    )

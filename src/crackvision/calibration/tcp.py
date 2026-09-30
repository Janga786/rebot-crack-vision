"""crackvision.calibration.tcp — pivot calibration solver for the tool-centre point (GEOM-06).

Implements the classic "pivot calibration" algorithm: the operator holds the physical tool tip
(`config/robot/end_effector.yaml`'s `tool` block, frame `tool_tip`, per ADR-014 §8.1) fixed against
a stationary reference point (e.g. a divot/cone fixture) while sweeping the arm through a set of
orientations. At every pose the physical tip touches the *same* fixed point in the world, but its
offset from `tool0` (`gripper_link`, per ADR-012) is *also* fixed and unknown — the two constant
unknowns (`p_tcp` in `tool0`'s frame, `p_pivot` in `base_link`) are recovered by least squares from
the recorded `T_base_link_tool0` pose at each orientation:

    p_pivot = R_i @ p_tcp + t_i   for every pose i

Only `p_tcp`'s *translation* is observable this way — pivoting a point about itself carries no
information about orientation, so this solver (and GEOM-06/07 generally) treats `T_tool0_tool_tip`
as translation-only. `p_tcp` here names the calibrated physical tool tip (`tool_tip`), not the
vendor MoveIt grasp centre (`gripper_tcp`, `GRASP_CENTRE_OFFSET_M` below) — see ADR-014 §5.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

# `rebotarm.urdf.xacro:8-12`'s fixed `gripper_tcp` offset relative to `gripper_link` (== `tool0`,
# ADR-012): xyz="-0.0443 0 0". This is the vendor MoveIt *grasp centre*, 44.3 mm proximal to the
# fingertips — it is documented here only for reference/regression purposes (ADR-014 §5) and is
# NEVER the default prior for `check_boresight`. The pivot calibration measures the physical tool
# tip (`config/robot/end_effector.yaml`'s `tool` block, `tool_tip`), which for a closed gripper is
# the canonical URDF finger meshes' distal reach, i.e. `gripper_link`'s own origin (0, 0, 0).
GRASP_CENTRE_OFFSET_M = (-0.0443, 0.0, 0.0)

_DEFAULT_END_EFFECTOR_CONFIG = (
    Path(__file__).resolve().parents[3] / "config" / "robot" / "end_effector.yaml"
)


@dataclass(frozen=True)
class ToolTipPrior:
    """The `tool` block's `xyz_m` prior read from `config/robot/end_effector.yaml`, plus provenance."""

    xyz_m: tuple[float, float, float]
    value_status: str
    provenance: str


def load_tool_tip_prior(config_path: Optional[Path] = None) -> ToolTipPrior:
    """Read the tool-tip prior (`tool.xyz_m`) from `config/robot/end_effector.yaml`.

    This is a light read of the single `tool` block used by the boresight comparison — it does not
    perform the full schema validation `crackvision_description.end_effector.load_config` does
    (that module lives in `ros2_ws/`, outside this package's dependency footprint); it only pulls
    the three fields (`xyz_m`, `value_status`, `provenance`) needed to know what the pivot solve is
    being compared against, per ADR-014 §8.1.
    """
    path = config_path if config_path is not None else _DEFAULT_END_EFFECTOR_CONFIG
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    tool = raw["tool"]
    return ToolTipPrior(
        xyz_m=tuple(float(v) for v in tool["xyz_m"]),
        value_status=str(tool["value_status"]),
        provenance=str(tool["provenance"]),
    )

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


# Below this vector length (m), a direction is not meaningfully defined (noise dominates), so
# `angle_from_prior_deg` is withheld rather than computed from an ~arbitrary direction. This also
# keeps the nominal tool-tip prior (0, 0, 0) from ever driving a spurious "opposite direction" flag
# — the thresholds and pass/fail interpretation live in the procedure document, not here.
MIN_VECTOR_LENGTH_FOR_ANGLE_M = 0.005


@dataclass(frozen=True)
class BoresightCheck:
    """Point comparison of a measured tool-tip position against the `end_effector.yaml` prior."""

    measured_point_m: np.ndarray  # (3,) solved tool-tip position, tool0 frame
    prior_point_m: np.ndarray  # (3,) prior tool-tip position (tool.xyz_m), tool0 frame
    position_delta_m: np.ndarray  # (3,) measured_point_m - prior_point_m
    position_delta_norm_m: float  # |measured - prior|, the primary comparison number
    angle_from_prior_deg: Optional[float]  # angle between the two point vectors, or None if either
    # vector is shorter than MIN_VECTOR_LENGTH_FOR_ANGLE_M (direction undefined near the origin)


def check_boresight(
    tcp_offset_m: np.ndarray, prior_offset_m: Optional[tuple[float, float, float]] = None
) -> BoresightCheck:
    """Compare a solved tool-tip point `tcp_offset_m` against `prior_offset_m`.

    `prior_offset_m` defaults to `load_tool_tip_prior().xyz_m`, i.e. `end_effector.yaml`'s `tool`
    block — the physical tool tip, not `GRASP_CENTRE_OFFSET_M`/`gripper_tcp`'s grasp centre.

    This is a point comparison, not a vector/angle one: `position_delta_norm_m` (the Euclidean
    distance between the measured and prior tool-tip points) is the primary, always-defined number.
    `angle_from_prior_deg` is reported only when both points are farther than
    `MIN_VECTOR_LENGTH_FOR_ANGLE_M` from `tool0`'s origin — for a prior near (0, 0, 0) (the nominal
    closed-gripper tip) a direction is not meaningful, and this avoids ever reporting a NaN or an
    "opposite direction" artifact for a small, valid offset. Pass/fail thresholds on either number
    belong in `docs/calibration/TCP_BORESIGHT_PROCEDURE.md`, not in this function.
    """
    if prior_offset_m is None:
        prior_offset_m = load_tool_tip_prior().xyz_m

    measured = np.asarray(tcp_offset_m, dtype=np.float64)
    prior = np.asarray(prior_offset_m, dtype=np.float64)

    position_delta = measured - prior
    position_delta_norm = float(np.linalg.norm(position_delta))

    measured_length = float(np.linalg.norm(measured))
    prior_length = float(np.linalg.norm(prior))

    angle_deg: Optional[float]
    if measured_length < MIN_VECTOR_LENGTH_FOR_ANGLE_M or prior_length < MIN_VECTOR_LENGTH_FOR_ANGLE_M:
        angle_deg = None
    else:
        cos_angle = np.clip(np.dot(measured, prior) / (measured_length * prior_length), -1.0, 1.0)
        angle_deg = float(np.degrees(np.arccos(cos_angle)))

    return BoresightCheck(
        measured_point_m=measured,
        prior_point_m=prior,
        position_delta_m=position_delta,
        position_delta_norm_m=position_delta_norm,
        angle_from_prior_deg=angle_deg,
    )

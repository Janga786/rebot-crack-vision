"""tools/synth_cracks.py — synthetic crack rasters with known centerlines. Built by PERC-09.

Generates binary crack masks (parametric curves, optional branches, optional salt noise) alongside
the exact `(row, col)` ground-truth centerline(s) they were drawn from, so `crackvision.paths`
output can be scored against a known answer (tests/test_path_accuracy.py). Nothing here reads or
writes the real dataset — every raster is synthesized in-memory from a caller-supplied seed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy.spatial import cKDTree

Pixel = tuple[int, int]


@dataclass(frozen=True)
class SynthCrack:
    """A synthetic crack raster plus the ground-truth centerline(s) it was drawn from."""

    mask: np.ndarray  # bool, shape (height, width)
    centerline: tuple[Pixel, ...]  # dense, ordered, one polyline (the main crack)
    branch_centerlines: tuple[tuple[Pixel, ...], ...] = ()


# ---------------------------------------------------------------------------
# Rasterization
# ---------------------------------------------------------------------------


def _dense_centerline(points: Sequence[tuple[float, float]]) -> tuple[Pixel, ...]:
    """Walk consecutive float `(row, col)` control points, emitting one integer pixel per unit step."""
    dense: list[Pixel] = []
    for (r0, c0), (r1, c1) in zip(points[:-1], points[1:]):
        dist = math.hypot(r1 - r0, c1 - c0)
        steps = max(1, int(math.ceil(dist)))
        for i in range(steps):
            t = i / steps
            pixel = (round(r0 + (r1 - r0) * t), round(c0 + (c1 - c0) * t))
            if not dense or dense[-1] != pixel:
                dense.append(pixel)
    last = (round(points[-1][0]), round(points[-1][1]))
    if not dense or dense[-1] != last:
        dense.append(last)
    return tuple(dense)


def _stamp_disk(mask: np.ndarray, center: tuple[float, float], radius: float) -> None:
    height, width = mask.shape
    r0, c0 = center
    r_lo, r_hi = max(0, int(r0 - radius) - 1), min(height, int(r0 + radius) + 2)
    c_lo, c_hi = max(0, int(c0 - radius) - 1), min(width, int(c0 + radius) + 2)
    if r_lo >= r_hi or c_lo >= c_hi:
        return
    rr, cc = np.mgrid[r_lo:r_hi, c_lo:c_hi]
    hit = (rr - r0) ** 2 + (cc - c0) ** 2 <= radius**2
    mask[r_lo:r_hi, c_lo:c_hi] |= hit


def rasterize_centerline(mask: np.ndarray, centerline: Sequence[tuple[float, float]], thickness_px: float) -> None:
    """Draw a thick line along `centerline` into `mask` (in place) by stamping disks of `radius = thickness_px / 2`."""
    radius = max(0.5, thickness_px / 2.0)
    for point in centerline:
        _stamp_disk(mask, point, radius)


# ---------------------------------------------------------------------------
# Generators
# ---------------------------------------------------------------------------


def wavy_centerline(
    height: int,
    width: int,
    *,
    amplitude_px: float,
    frequency: float,
    phase: float,
    row0: float | None = None,
    margin: int = 8,
) -> tuple[tuple[float, float], ...]:
    """A sinusoidal centerline spanning the width, `margin` px clear of each edge."""
    row0 = height / 2.0 if row0 is None else row0
    cols = np.linspace(margin, width - 1 - margin, num=max(2, width - 2 * margin))
    return tuple((row0 + amplitude_px * math.sin(frequency * c + phase), float(c)) for c in cols)


def generate_wavy_crack(
    height: int = 200,
    width: int = 300,
    *,
    amplitude_px: float = 20.0,
    frequency: float = 0.03,
    phase: float = 0.0,
    thickness_px: float = 3.0,
    seed: int | None = None,
    salt_noise_prob: float = 0.0,
) -> SynthCrack:
    """A single wavy crack of known centerline, optionally with scattered salt-noise pixels."""
    control = wavy_centerline(height, width, amplitude_px=amplitude_px, frequency=frequency, phase=phase)
    centerline = _dense_centerline(control)
    mask = np.zeros((height, width), dtype=bool)
    rasterize_centerline(mask, control, thickness_px)

    if salt_noise_prob > 0:
        rng = np.random.default_rng(seed)
        salt = rng.random((height, width)) < salt_noise_prob
        mask |= salt

    return SynthCrack(mask=mask, centerline=centerline)


def generate_branched_crack(
    height: int = 200,
    width: int = 300,
    *,
    amplitude_px: float = 15.0,
    frequency: float = 0.025,
    thickness_px: float = 3.0,
    branch_frac: float = 0.5,
    branch_length_px: float = 60.0,
    branch_angle_rad: float = math.radians(40),
    seed: int | None = None,
) -> SynthCrack:
    """A wavy main crack plus one straight branch splitting off partway along it."""
    control = wavy_centerline(height, width, amplitude_px=amplitude_px, frequency=frequency, phase=0.0)
    centerline = _dense_centerline(control)
    mask = np.zeros((height, width), dtype=bool)
    rasterize_centerline(mask, control, thickness_px)

    idx = int(round(branch_frac * (len(control) - 1)))
    idx = max(1, min(len(control) - 2, idx))
    r0, c0 = control[idx]
    r1, c1 = control[idx + 1]
    tangent = math.atan2(r1 - r0, c1 - c0)
    branch_dir = tangent + branch_angle_rad
    branch_end = (r0 + branch_length_px * math.sin(branch_dir), c0 + branch_length_px * math.cos(branch_dir))
    branch_end = (
        float(np.clip(branch_end[0], 1, height - 2)),
        float(np.clip(branch_end[1], 1, width - 2)),
    )
    branch_control = [(r0, c0), branch_end]
    branch_centerline = _dense_centerline(branch_control)
    rasterize_centerline(mask, branch_centerline, thickness_px)

    return SynthCrack(mask=mask, centerline=centerline, branch_centerlines=(branch_centerline,))


# ---------------------------------------------------------------------------
# Accuracy metrics
# ---------------------------------------------------------------------------


def _as_array(points: Sequence[Pixel]) -> np.ndarray:
    return np.asarray(points, dtype=np.float64).reshape(-1, 2)


def symmetric_hausdorff_px(gt: Sequence[Pixel], extracted: Sequence[Pixel]) -> float:
    """`max(directed_hausdorff(gt -> extracted), directed_hausdorff(extracted -> gt))`, in px."""
    gt_arr, ext_arr = _as_array(gt), _as_array(extracted)
    tree_ext = cKDTree(ext_arr)
    tree_gt = cKDTree(gt_arr)
    d_gt_to_ext = float(tree_ext.query(gt_arr)[0].max())
    d_ext_to_gt = float(tree_gt.query(ext_arr)[0].max())
    return max(d_gt_to_ext, d_ext_to_gt)


def mean_distance_px(gt: Sequence[Pixel], extracted: Sequence[Pixel]) -> float:
    """Mean of the two directed nearest-neighbor mean distances (symmetric mean error), in px."""
    gt_arr, ext_arr = _as_array(gt), _as_array(extracted)
    tree_ext = cKDTree(ext_arr)
    tree_gt = cKDTree(gt_arr)
    d_gt_to_ext = tree_ext.query(gt_arr)[0].mean()
    d_ext_to_gt = tree_gt.query(ext_arr)[0].mean()
    return float((d_gt_to_ext + d_ext_to_gt) / 2.0)


def coverage_fraction(gt: Sequence[Pixel], extracted: Sequence[Pixel], threshold_px: float) -> float:
    """Fraction of `gt` points that have some `extracted` point within `threshold_px`."""
    gt_arr, ext_arr = _as_array(gt), _as_array(extracted)
    tree_ext = cKDTree(ext_arr)
    dist, _ = tree_ext.query(gt_arr)
    return float(np.mean(dist <= threshold_px))


def ordering_error_rate(gt: Sequence[Pixel], extracted: Sequence[Pixel]) -> float:
    """Fraction of adjacent `extracted` steps whose matched ground-truth index moves backwards.

    Each `extracted` point is matched to the index of its nearest `gt` point (`gt` is itself
    ordered along the true centerline). If `extracted` faithfully traces the centerline in one
    direction, those matched indices should be monotonic; a "step back" is counted whichever
    overall direction (increasing or decreasing) has fewer such steps, so the metric does not
    penalize a path recovered in reverse.
    """
    gt_arr, ext_arr = _as_array(gt), _as_array(extracted)
    tree_gt = cKDTree(gt_arr)
    _, matched_idx = tree_gt.query(ext_arr)
    if len(matched_idx) < 2:
        return 0.0
    diffs = np.diff(matched_idx)
    forward_breaks = int(np.sum(diffs < 0))
    backward_breaks = int(np.sum(diffs > 0))
    breaks = min(forward_breaks, backward_breaks)
    return float(breaks / (len(matched_idx) - 1))

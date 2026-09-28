"""crackvision.geometry — pixel + aligned depth + intrinsics -> 3D points (GEOM-02).

Deprojection is a vectorised NumPy port of librealsense's `rs2_deproject_pixel_to_point`
(`rsutil.h`) for the distortion models the D405 colour stream actually reports
("Inverse Brown Conrady", with "Brown Conrady" and the distortion-free "none"/"synthetic"
cases also supported). The exact iterative undistortion formula below was verified against
`pyrealsense2.rs2_deproject_pixel_to_point` directly (only the header declaration, not the
C++ implementation, is vendored on this box) — see `tests/test_geometry.py`, which skips the
comparison when `pyrealsense2` is absent (docs/RISKS.md R-12) rather than hard-coding it.

Pixel convention, the pixel-centre (no +0.5) rule, the depth-scale rule and the invalid-depth
band are all fixed by `docs/adr/012-frames-and-conventions.md`; this module implements that
ADR and must not silently diverge from it. Callers pass `(row, col)` array-convention pixel
coordinates (`docs/INTERFACES.md` §0.5); this module does the `(row, col)` -> `(u, v) =
(col, row)` conversion internally so nothing above it has to.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ADR-012 "Depth" section, RISKS.md R-08: the D405's documented usable working-distance band
# at the planned 10-40 cm evaluation distances. A property of this device/use-case, not a
# universal constant.
DEFAULT_VALID_DEPTH_RANGE_M = (0.07, 0.50)

REASON_ZERO = "zero"
REASON_NEGATIVE = "negative"
REASON_OUT_OF_BAND = "out_of_band"
REASON_NAN = "nan"

# librealsense's rsutil.h uses 10 Newton iterations for Brown-Conrady undistortion
# ("10 iterations determined empirically"); matched here for bit-for-bit-equivalent behaviour.
_UNDISTORT_ITERATIONS = 10

# Intel D405 product datasheet nominal values, used only to size a per-point depth-uncertainty
# estimate. Prior evidence, not a calibrated measurement (ADR-012's evidence-vs-fact distinction
# applies here too) — a later card may replace these with measured numbers.
D405_NOMINAL_BASELINE_M = 0.018
D405_NOMINAL_DISPARITY_SIGMA_PX = 0.08

_IDENTITY_MODELS = frozenset({"none", "synthetic"})
_UNDISTORT_MODELS = frozenset({"inverse_brown_conrady", "brown_conrady"})


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole intrinsics + distortion, shaped like `docs/INTERFACES.md` §3.10's per-stream block."""

    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float
    model: str
    coeffs: tuple[float, float, float, float, float]

    @classmethod
    def from_stream_dict(cls, stream: dict) -> "Intrinsics":
        """Build from a `{"width", "height", "intrinsics": {...}}` block (§3.10 `color`/`depth`)."""
        intr = stream["intrinsics"]
        coeffs = tuple(float(c) for c in intr["coeffs"])
        if len(coeffs) != 5:
            raise ValueError(f"expected 5 distortion coefficients, got {len(coeffs)}")
        return cls(
            width=int(stream["width"]),
            height=int(stream["height"]),
            fx=float(intr["fx"]),
            fy=float(intr["fy"]),
            ppx=float(intr["ppx"]),
            ppy=float(intr["ppy"]),
            model=str(intr["model"]),
            coeffs=coeffs,
        )


def _normalized_model(model: str) -> str:
    """`"distortion.inverse_brown_conrady"`, `"Inverse Brown Conrady"`, `"synthetic"` -> one token."""
    return model.rsplit(".", 1)[-1].strip().lower().replace(" ", "_")


def deproject_pixels(intrinsics: Intrinsics, rows, cols, depth_m) -> np.ndarray:
    """Vectorised `rs2_deproject_pixel_to_point`. `rows`/`cols`/`depth_m` broadcast together.

    `rows`/`cols` are array-convention `(row, col)` pixel coordinates addressing a pixel's
    *centre* directly (no `+0.5` offset — ADR-012, confirmed against `rsutil.h`). Returns an
    array of shape `broadcast_shape + (3,)` holding `(x, y, z)` metres in the frame the
    intrinsics belong to (`camera_color_optical_frame` for colour-aligned depth, ADR-012).
    """
    rows_arr = np.asarray(rows, dtype=np.float64)
    cols_arr = np.asarray(cols, dtype=np.float64)
    depth_arr = np.asarray(depth_m, dtype=np.float64)
    rows_arr, cols_arr, depth_arr = np.broadcast_arrays(rows_arr, cols_arr, depth_arr)

    model = _normalized_model(intrinsics.model)
    x0 = (cols_arr - intrinsics.ppx) / intrinsics.fx
    y0 = (rows_arr - intrinsics.ppy) / intrinsics.fy

    if model in _IDENTITY_MODELS:
        x, y = x0, y0
    elif model in _UNDISTORT_MODELS:
        c = intrinsics.coeffs
        x, y = x0.copy(), y0.copy()
        for _ in range(_UNDISTORT_ITERATIONS):
            r2 = x * x + y * y
            icdist = 1.0 / (1 + ((c[4] * r2 + c[1]) * r2 + c[0]) * r2)
            dx = 2 * c[2] * x * y + c[3] * (r2 + 2 * x * x)
            dy = 2 * c[3] * x * y + c[2] * (r2 + 2 * y * y)
            x = (x0 - dx) * icdist
            y = (y0 - dy) * icdist
    else:
        raise NotImplementedError(
            f"deproject_pixels: distortion model {intrinsics.model!r} is not supported here "
            "(only 'none'/'synthetic' and the D405 colour stream's Brown-Conrady family are)"
        )

    points = np.empty(rows_arr.shape + (3,), dtype=np.float64)
    points[..., 0] = depth_arr * x
    points[..., 1] = depth_arr * y
    points[..., 2] = depth_arr
    return points


@dataclass
class DepthClassification:
    """Per-pixel invalid-depth classification (ADR-012 "Depth" section)."""

    depth_m: np.ndarray  # float64, NaN wherever `valid` is False
    valid: np.ndarray  # bool
    reason: np.ndarray  # object array of str; "" wherever `valid` is True


def classify_depth(
    depth_raw,
    depth_scale_m_per_unit: float,
    valid_range_m: tuple[float, float] = DEFAULT_VALID_DEPTH_RANGE_M,
) -> DepthClassification:
    """Classify raw `uint16` depth counts as valid/invalid, with an explicit reason.

    Invalid per ADR-012: the device's `0` "no return" sentinel, a value outside
    `valid_range_m`, or a non-finite value. Never silently returns a point at depth 0.
    """
    depth_raw_arr = np.asarray(depth_raw)
    depth_m = depth_raw_arr.astype(np.float64) * depth_scale_m_per_unit

    reason = np.full(depth_m.shape, "", dtype=object)
    nan_mask = ~np.isfinite(depth_m)
    zero_mask = (~nan_mask) & (depth_raw_arr == 0)
    negative_mask = (~nan_mask) & (~zero_mask) & (depth_m < 0)
    lo, hi = valid_range_m
    out_of_band_mask = (
        (~nan_mask) & (~zero_mask) & (~negative_mask) & ((depth_m < lo) | (depth_m > hi))
    )

    reason[nan_mask] = REASON_NAN
    reason[zero_mask] = REASON_ZERO
    reason[negative_mask] = REASON_NEGATIVE
    reason[out_of_band_mask] = REASON_OUT_OF_BAND

    valid = reason == ""
    depth_m_out = np.where(valid, depth_m, np.nan)
    return DepthClassification(depth_m=depth_m_out, valid=valid, reason=reason)


def deproject_pixels_from_image(
    intrinsics: Intrinsics,
    rows,
    cols,
    depth_image_raw: np.ndarray,
    depth_scale_m_per_unit: float,
    valid_range_m: tuple[float, float] = DEFAULT_VALID_DEPTH_RANGE_M,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Deproject `(row, col)` pixels by reading their own depth out of `depth_image_raw`.

    Returns `(points, valid, reason)`: `points` has shape `rows.shape + (3,)` and is `NaN` at
    every invalid pixel (never a silent point at the camera origin); `valid`/`reason` mirror
    `classify_depth`.
    """
    rows_arr = np.asarray(rows)
    cols_arr = np.asarray(cols)
    depth_raw = np.asarray(depth_image_raw)[rows_arr, cols_arr]
    classification = classify_depth(depth_raw, depth_scale_m_per_unit, valid_range_m)

    safe_depth_m = np.where(classification.valid, classification.depth_m, 0.0)
    points = deproject_pixels(intrinsics, rows_arr, cols_arr, safe_depth_m)
    points[~classification.valid] = np.nan
    return points, classification.valid, classification.reason


def depth_uncertainty_m(
    depth_m,
    fx: float,
    baseline_m: float = D405_NOMINAL_BASELINE_M,
    disparity_sigma_px: float = D405_NOMINAL_DISPARITY_SIGMA_PX,
) -> np.ndarray:
    """Per-point 1-sigma depth uncertainty from the standard stereo error model.

    `sigma_z = z^2 * sigma_disparity / (f * baseline)` — quadratic in depth, as expected for
    passive/active stereo. `baseline_m`/`disparity_sigma_px` default to nominal D405 datasheet
    values (prior evidence, not calibrated — see module docstring); pass measured values once
    a later card has them.
    """
    depth_m_arr = np.asarray(depth_m, dtype=np.float64)
    return depth_m_arr**2 * disparity_sigma_px / (fx * baseline_m)


@dataclass
class AnnulusSample:
    """Result of sampling surface depth around one crack-path pixel (RISKS.md R-08)."""

    row: int
    col: int
    depth_m: float  # NaN if invalid
    valid: bool
    reason: str  # "" if valid
    fraction_valid: float
    n_candidates: int
    n_valid: int


REASON_NO_ANNULUS_CANDIDATES = "no_annulus_candidates"
REASON_NO_VALID_DEPTH_IN_ANNULUS = "no_valid_depth_in_annulus"
REASON_INSUFFICIENT_VALID_FRACTION = "insufficient_valid_fraction"


def sample_surface_depth_annulus(
    depth_image_raw: np.ndarray,
    mask: np.ndarray,
    row: int,
    col: int,
    depth_scale_m_per_unit: float,
    inner_radius_px: int,
    outer_radius_px: int,
    valid_range_m: tuple[float, float] = DEFAULT_VALID_DEPTH_RANGE_M,
    min_fraction_valid: float = 0.3,
) -> AnnulusSample:
    """Sample surface depth for a crack-path pixel from an annulus, excluding mask pixels.

    RISKS.md R-08: a crack is a narrow cavity, often narrower than the stereo correlation
    window, so depth sampled *at* the crack pixel is the surface depth spanning the gap, or a
    hole. Instead this samples the annulus `inner_radius_px <= dist <= outer_radius_px` around
    `(row, col)`, drops any pixel where `mask` is truthy (the crack/cavity itself) and any pixel
    outside the image bounds, classifies each remaining candidate's depth (`classify_depth`),
    and returns the robust median of the valid depths plus the fraction of candidates that were
    valid — so a caller can see when the estimate rests on too little evidence.
    """
    if inner_radius_px < 0 or outer_radius_px <= inner_radius_px:
        raise ValueError("require 0 <= inner_radius_px < outer_radius_px")

    depth_image_raw = np.asarray(depth_image_raw)
    mask_arr = np.asarray(mask, dtype=bool)
    height, width = depth_image_raw.shape

    r0 = max(0, row - outer_radius_px)
    r1 = min(height, row + outer_radius_px + 1)
    c0 = max(0, col - outer_radius_px)
    c1 = min(width, col + outer_radius_px + 1)

    rr, cc = np.meshgrid(np.arange(r0, r1), np.arange(c0, c1), indexing="ij")
    dist2 = (rr - row) ** 2 + (cc - col) ** 2
    annulus = (dist2 >= inner_radius_px**2) & (dist2 <= outer_radius_px**2)
    annulus &= ~mask_arr[r0:r1, c0:c1]

    n_candidates = int(annulus.sum())
    if n_candidates == 0:
        return AnnulusSample(
            row, col, float("nan"), False, REASON_NO_ANNULUS_CANDIDATES, 0.0, 0, 0
        )

    candidate_depth_raw = depth_image_raw[r0:r1, c0:c1][annulus]
    classification = classify_depth(candidate_depth_raw, depth_scale_m_per_unit, valid_range_m)
    n_valid = int(classification.valid.sum())
    fraction_valid = n_valid / n_candidates

    if n_valid == 0:
        return AnnulusSample(
            row, col, float("nan"), False, REASON_NO_VALID_DEPTH_IN_ANNULUS,
            fraction_valid, n_candidates, n_valid,
        )
    if fraction_valid < min_fraction_valid:
        return AnnulusSample(
            row, col, float("nan"), False, REASON_INSUFFICIENT_VALID_FRACTION,
            fraction_valid, n_candidates, n_valid,
        )

    depth_m = float(np.median(classification.depth_m[classification.valid]))
    return AnnulusSample(row, col, depth_m, True, "", fraction_valid, n_candidates, n_valid)


@dataclass
class PlaneFit:
    """Best-fit plane through a point cloud: `normal . (p - centroid) = 0`."""

    normal: np.ndarray  # unit vector, shape (3,)
    centroid: np.ndarray  # shape (3,)
    rms_residual_m: float


def fit_plane(points_xyz: np.ndarray) -> PlaneFit:
    """Least-squares plane fit via SVD of the centred points.

    Used to validate deprojected point clouds against a known synthetic surface
    (`tests/test_geometry.py`'s tilted/noisy plane tests) and, more generally, to summarise a
    local surface patch's normal for later cards.
    """
    pts = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] < 3:
        raise ValueError("need at least 3 points to fit a plane")

    centroid = pts.mean(axis=0)
    centered = pts - centroid
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    normal = vt[-1]
    if normal[2] < 0:  # canonicalize: normal's z-component faces back toward the camera origin
        normal = -normal

    residuals = centered @ normal
    rms = float(np.sqrt(np.mean(residuals**2)))
    return PlaneFit(normal=normal, centroid=centroid, rms_residual_m=rms)

"""tools/synth_scene3d.py — deterministic synthetic eye-in-hand scene generator (GEOM-08.8).

Renders a known crack-bearing surface under the nominal `FK(q) . T_gripper_link_camera_link .
T_camera_link_camera_color_optical_frame` chain (docs/INTERFACES.md §9.2) and writes a complete
synthetic `crackvision` case (skeleton PNG, dilated mask, colour placeholder, aligned depth PNG,
§9 capture record, `case_map.json` entry) plus a ground-truth JSON, so the real
`crackvision.paths` -> `crackvision.path3d` pipeline can be run on it and scored against a known
answer (`tests/test_path3d_synthetic.py`).

Ground truth is produced by its own ray/plane intersection + forward projection here — it never
calls `crackvision.geometry.deproject_pixels`, `crackvision.lift3d` or `crackvision.tool_waypoints`,
so a bug shared between the generator and the pipeline under test cannot hide itself.

CLI:
    ./env.sh python tools/synth_scene3d.py --out ROOT --seed N [--tilt-deg 0|15]
                                            [--view-tilt-deg 0|15] [--depth-noise] [--holes]
                                            [--case-id ID]

Verifies *software consistency* of the lifting pipeline under a nominal, noise-free-unless-asked
chain — not physical accuracy of the real robot/camera (that is GEOM-05/07 and INT-04/05's job;
see docs/geometry/PATH3D_VERIFICATION.md).
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from PIL import Image
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from crackvision import capture_record as cr
from crackvision import kinematics as kin
from crackvision import naming
from crackvision.config import load_config
from crackvision.geometry import depth_uncertainty_m

# ---------------------------------------------------------------------------
# Scene constants (card body, GEOM-08.8)
# ---------------------------------------------------------------------------

IMAGE_WIDTH = 848
IMAGE_HEIGHT = 480
FX = 430.0
FY = 430.0
PPX = IMAGE_WIDTH / 2.0
PPY = IMAGE_HEIGHT / 2.0
INTRINSICS_MODEL = "none"
DEPTH_SCALE_M_PER_UNIT = 1e-4

VIEW_DISTANCE_M = 0.25  # ADR-014 §2 view phase
IK_RESIDUAL_TOL = 1e-6
IK_ROLL_GRID_DEG = tuple(range(0, 360, 30))
IK_SEEDS = (
    (0.0, -0.5, -1.0, 0.0, 0.5, 0.0),
    (0.0, -1.0, -0.5, 0.0, 0.8, 0.0),
    (0.1, -0.8, -0.8, 0.2, 0.6, 0.1),
    (0.0, -1.5, -0.3, 0.0, 1.0, 0.0),
    (0.0, -0.3, -1.5, 0.0, 0.3, 0.0),
    (0.3, -1.7, -1.5, 1.4, 0.25, 1.9),
    (0.0, -2.0, -1.0, 0.5, 0.5, 0.0),
    (0.0, -1.0, -2.0, 1.0, 1.0, 1.0),
    (0.0, -2.5, -0.5, -1.0, 1.0, 0.0),
)

CAVITY_DEPTH_M = 0.003  # R-08: crack cavity, >= 3 mm deeper (farther from camera) than the surface
MASK_RADIUS_PX = 2.5  # ~5-px-wide dilation of the 1-px skeleton
CRACK_HALF_LENGTH_M = 0.03  # ~60 mm total arc length
CRACK_AMPLITUDE_M = 0.005
CRACK_SAMPLES = 2000

# docs/INTERFACES.md §10.3 annulus defaults, used only to size the injected holes below so that a
# hole's own annulus is fully invalid regardless of --annulus-* overrides a caller might later use.
ANNULUS_OUTER_PX_DEFAULT = 8

# Hole sizing (card body: "one 3-px and one 10-px along the crack"): the short hole's injected
# run must land at or below path3d's default `max_gap_px` (5, docs/INTERFACES.md §10.3) so it gets
# interpolated, and the long one strictly above so it splits the polyline. Zeroing a full
# `ANNULUS_OUTER_PX_DEFAULT`-radius disk around every one of a multi-pixel target run reliably
# produces a long, clearly-split gap (empirically ~15 px here), but the same disk around even a
# single-pixel target already bleeds into neighbours past `max_gap_px` (empirically 6-7 px) --
# so the short hole instead uses a single target pixel with a smaller zero radius, tuned
# (`tests/test_path3d_synthetic.py`-adjacent exploration, see docs/geometry/PATH3D_VERIFICATION.md)
# to leave a 3-4 px invalid run at both tilts tested here, comfortably inside `max_gap_px`.
HOLE_SHORT_LEN = 1
HOLE_SHORT_ZERO_RADIUS_PX = 7.1
HOLE_LONG_LEN = 10
HOLE_LONG_ZERO_RADIUS_PX = float(ANNULUS_OUTER_PX_DEFAULT)
HOLE_EDGE_MARGIN = 20
HOLE_GAP = 25

# ADR-014 §3 nominal placement fallback, used by `specimen_center()` whenever
# config/motion/specimen_placement.yaml is missing, unreadable or carries no feasible placement
# under the given `root`. As of this card the *live* repo file (config/motion/specimen_placement.yaml)
# is feasible with centre (0.26, 0.0, 0.0) -- but every invocation this card exercises (the test
# suite's scaffolded tmp_path root, and the doc's own `--out` examples) resolves `root` to a
# scaffolded scratch directory that never contains a copy of that file (`_ensure_project_scaffold`
# only copies `config/project.yaml`), so this fallback is still what actually gets used here. See
# `specimen_center()` and docs/geometry/PATH3D_VERIFICATION.md's "Specimen centre" section.
FALLBACK_SPECIMEN_CENTER_XY_M = (0.29, 0.0)
# config/scene/scene.yaml's specimen box: position_m [0.30, 0.0, -0.015], dimensions_m
# [..,..,0.01] -> top face z = -0.015 + 0.01/2 = -0.01 (base_link). Used as "table z" since
# specimen_placement.yaml has no feasible surface_z of its own.
FALLBACK_SPECIMEN_TOP_Z_M = -0.01


Pixel = tuple[int, int]


# ---------------------------------------------------------------------------
# Specimen centre (reads config/motion/specimen_placement.yaml; falls back to ADR-014)
# ---------------------------------------------------------------------------


def specimen_center(root: Path) -> tuple[np.ndarray, str]:
    """`(center_xyz_base, source)`.

    Reads `{root}/config/motion/specimen_placement.yaml`'s `placement.center_xy_m` (MOT-04.3
    schema) when that file is present under `root` and has a `placement` with `center_xy_m`.
    Whenever the file is missing/unreadable under `root`, or the expected keys are missing or
    differently shaped, this falls back to the ADR-014 §3 nominal placement `(0.29, 0, table_z)`
    with `table_z` taken from `config/scene/scene.yaml`'s specimen box top face
    (`FALLBACK_SPECIMEN_TOP_Z_M`), and says so in the returned source string.

    Note this reads `root`'s copy of the file, not necessarily the live repo's: every call this
    card's generator (`generate_case`/`main`) makes passes the generator's `--out`/scaffolded
    root, and `_ensure_project_scaffold` never copies `config/motion/specimen_placement.yaml`
    into that root -- so those calls always take the fallback branch below, regardless of
    whether the live repo's `config/motion/specimen_placement.yaml` is feasible.
    """
    path = Path(root) / "config" / "motion" / "specimen_placement.yaml"
    try:
        doc = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except OSError:
        doc = {}

    placement = doc.get("placement")
    if isinstance(placement, dict) and "center_xy_m" in placement:
        xy = placement["center_xy_m"]
        z = placement.get("surface_z_m", FALLBACK_SPECIMEN_TOP_Z_M)
        try:
            return (
                np.array([float(xy[0]), float(xy[1]), float(z)], dtype=np.float64),
                f"{path}: placement.center_xy_m/surface_z_m",
            )
        except (TypeError, ValueError, IndexError):
            pass

    return (
        np.array(
            [FALLBACK_SPECIMEN_CENTER_XY_M[0], FALLBACK_SPECIMEN_CENTER_XY_M[1], FALLBACK_SPECIMEN_TOP_Z_M],
            dtype=np.float64,
        ),
        f"fallback: {path} has no usable placement.center_xy_m under this root (file missing, "
        "unreadable, or placement/center_xy_m absent or malformed) -- using the ADR-014 §3 "
        f"nominal placement (0.29, 0) on the table, with table_z={FALLBACK_SPECIMEN_TOP_Z_M} "
        "from config/scene/scene.yaml's specimen box top face",
    )


# ---------------------------------------------------------------------------
# Plane + camera-pose geometry
# ---------------------------------------------------------------------------


def plane_normal(tilt_deg: float) -> np.ndarray:
    """Outward surface normal (points toward the camera side), tilted about base_link +Y."""
    theta = math.radians(tilt_deg)
    return np.array([math.sin(theta), 0.0, math.cos(theta)], dtype=np.float64)


def _orthonormal_basis(z_axis: np.ndarray, roll_rad: float) -> tuple[np.ndarray, np.ndarray]:
    """An arbitrary right-handed `(x, y)` pair orthogonal to `z_axis`, rolled by `roll_rad`."""
    ref = np.array([0.0, 1.0, 0.0])
    x0 = np.cross(ref, z_axis)
    if np.linalg.norm(x0) < 1e-6:
        ref = np.array([1.0, 0.0, 0.0])
        x0 = np.cross(ref, z_axis)
    x0 = x0 / np.linalg.norm(x0)
    y0 = np.cross(z_axis, x0)
    x_axis = math.cos(roll_rad) * x0 + math.sin(roll_rad) * y0
    y_axis = np.cross(z_axis, x_axis)
    return x_axis, y_axis


def camera_target_pose(
    center: np.ndarray, normal: np.ndarray, roll_rad: float, view_tilt_rad: float = 0.0
) -> np.ndarray:
    """`T_base_link_camera_color_optical_frame` for a view `VIEW_DISTANCE_M` from `center`,
    boresight (+Z) tilted `view_tilt_rad` away from the surface anti-normal (ADR-014 §2: the view
    phase bounds the angle between the optical axis and the anti-normal to <= 15 deg).

    `view_tilt_rad` is independent of `normal`'s own tilt (`plane_normal`'s `tilt_deg`): the
    camera is placed on the cone of half-angle `view_tilt_rad` around `normal`, at `VIEW_DISTANCE_M`
    from `center`, and pointed back at `center` -- so the boresight-to-anti-normal angle is exactly
    `view_tilt_rad` by construction (`cam_dir = cos(view_tilt_rad)*normal + sin(view_tilt_rad)*x0`,
    `x0 ⟂ normal`; boresight `z_axis = -cam_dir` points from the camera straight at `center`, and
    `dot(z_axis, -normal) = dot(cam_dir, normal) = cos(view_tilt_rad)`). At `view_tilt_rad=0` this
    is exactly the previous fronto-parallel pose (`z_axis = -normal`).
    """
    x0, _y0 = _orthonormal_basis(-normal, 0.0)
    cam_dir = math.cos(view_tilt_rad) * normal + math.sin(view_tilt_rad) * x0
    cam_dir = cam_dir / np.linalg.norm(cam_dir)
    z_axis = -cam_dir
    x_axis, y_axis = _orthonormal_basis(z_axis, roll_rad)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
    T[:3, 3] = center + VIEW_DISTANCE_M * cam_dir
    return T


def solve_view_ik(
    chain: "kin.KinematicChain", end_effector: "kin.EndEffector", T_base_optical_target: np.ndarray
) -> tuple[np.ndarray, float]:
    """Numeric IK (`scipy.optimize.least_squares`) for `q` placing the camera optical frame at
    `T_base_optical_target`. Returns `(q, residual_norm)`; does not raise on non-convergence."""
    T_gripper_cam = end_effector.T_gripper_link_camera_link
    T_base_camlink_target = T_base_optical_target @ np.linalg.inv(
        kin.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME
    )
    T_base_gripper_target = T_base_camlink_target @ np.linalg.inv(T_gripper_cam)

    lowers = np.array([j.lower for j in chain.joints if j.type != "fixed"])
    uppers = np.array([j.upper for j in chain.joints if j.type != "fixed"])
    eps = 1e-6
    lo, hi = lowers + eps, uppers - eps

    def residual(q: np.ndarray) -> np.ndarray:
        T = chain.fk(np.clip(q, lo, hi))
        pos_err = T[:3, 3] - T_base_gripper_target[:3, 3]
        R_err = T[:3, :3].T @ T_base_gripper_target[:3, :3]
        rot_err = Rotation.from_matrix(R_err).as_rotvec()
        return np.concatenate([pos_err, rot_err])

    best_resid = None
    best_q = None
    for seed in IK_SEEDS:
        result = least_squares(
            residual, np.clip(np.array(seed), lo, hi), method="trf", bounds=(lo, hi),
            xtol=1e-15, ftol=1e-15, gtol=1e-15, max_nfev=30000,
        )
        n = float(np.linalg.norm(result.fun))
        if best_resid is None or n < best_resid:
            best_resid, best_q = n, result.x.copy()
    return best_q, best_resid


def solve_camera_pose(
    chain: "kin.KinematicChain",
    end_effector: "kin.EndEffector",
    center: np.ndarray,
    normal: np.ndarray,
    view_tilt_rad: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Search `IK_ROLL_GRID_DEG` camera rolls for the best-converging IK solution.

    The view phase's roll is unconstrained (ADR-014 §2 fixes only distance + anti-normal angle),
    so trying several rolls lets the solver avoid wrist singularities that make one particular
    roll unreachable. Returns `(q, T_base_optical_target, residual_norm, roll_deg)`.
    """
    best = None
    for roll_deg in IK_ROLL_GRID_DEG:
        T_target = camera_target_pose(center, normal, math.radians(roll_deg), view_tilt_rad)
        q, resid = solve_view_ik(chain, end_effector, T_target)
        if best is None or resid < best[1]:
            best = (q, resid, T_target, roll_deg)
    q, resid, T_target, roll_deg = best
    return q, T_target, resid, float(roll_deg)


# ---------------------------------------------------------------------------
# Crack curve + forward projection (independent of crackvision.geometry.deproject_pixels)
# ---------------------------------------------------------------------------


def crack_curve_points_base(center: np.ndarray, normal: np.ndarray, n_samples: int = CRACK_SAMPLES) -> np.ndarray:
    """A smooth sine arc, ~`2 * CRACK_HALF_LENGTH_M` long, lying exactly in the surface plane."""
    z_axis = -normal
    e1, e2 = _orthonormal_basis(z_axis, 0.0)
    t = np.linspace(-CRACK_HALF_LENGTH_M, CRACK_HALF_LENGTH_M, n_samples)
    wavelength = 2.0 * CRACK_HALF_LENGTH_M
    v = CRACK_AMPLITUDE_M * np.sin(2.0 * math.pi * t / wavelength)
    return center[None, :] + t[:, None] * e1[None, :] + v[:, None] * e2[None, :]


def project_to_pixels(T_base_optical: np.ndarray, points_base: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Forward-project `base_link` points through the pinhole model (`model='none'`).

    Returns `(row, col, depth_m)`, all `(N,)`; this is the exact inverse of
    `crackvision.geometry.deproject_pixels` for the identity distortion model, re-derived here
    rather than imported, per this card's "lift3d code is not reused" rule.
    """
    T_optical_base = np.linalg.inv(T_base_optical)
    pts_h = np.concatenate([points_base, np.ones((points_base.shape[0], 1))], axis=1)
    p_opt = (T_optical_base @ pts_h.T).T[:, :3]
    x, y, z = p_opt[:, 0], p_opt[:, 1], p_opt[:, 2]
    col = FX * x / z + PPX
    row = FY * y / z + PPY
    return row, col, z


def ray_plane_points_base(
    T_base_optical: np.ndarray, center: np.ndarray, normal: np.ndarray, rows: np.ndarray, cols: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Ray/plane intersection in `base_link` for a batch of pixels (vectorised).

    `rows`/`cols` broadcast together; returns `(points_base (..., 3), depth_m (...))`. `depth_m`
    is negative/non-finite wherever the ray does not hit the plane in front of the camera.
    """
    rows = np.asarray(rows, dtype=np.float64)
    cols = np.asarray(cols, dtype=np.float64)
    x0 = (cols - PPX) / FX
    y0 = (rows - PPY) / FY
    d_optical = np.stack([x0, y0, np.ones_like(x0)], axis=-1)
    R = T_base_optical[:3, :3]
    d_base = d_optical @ R.T
    cam = T_base_optical[:3, 3]
    denom = d_base @ normal
    t = -np.dot(cam - center, normal) / denom
    points = cam + t[..., None] * d_base
    return points, t


# ---------------------------------------------------------------------------
# Rasterisation (1-px 8-connected skeleton + dilated mask)
# ---------------------------------------------------------------------------


CHAIN_COARSE_SAMPLES = 40
CHAIN_ANCHOR_PX_FACTOR = 0.5


def _bresenham_line(r0: int, c0: int, r1: int, c1: int) -> list[Pixel]:
    """Standard integer Bresenham: the unique thin 8-connected path from `(r0,c0)` to `(r1,c1)`."""
    points: list[Pixel] = []
    dr, dc = r1 - r0, c1 - c0
    r, c = r0, c0
    sr = 1 if dr >= 0 else -1
    sc = 1 if dc >= 0 else -1
    adr, adc = abs(dr), abs(dc)
    if adr >= adc:
        err = 0
        for _ in range(adr + 1):
            points.append((r, c))
            err += adc
            if 2 * err >= adr:
                c += sc
                err -= adr
            r += sr
    else:
        err = 0
        for _ in range(adc + 1):
            points.append((r, c))
            err += adr
            if 2 * err >= adc:
                r += sr
                err -= adc
            c += sc
    return points


def build_chain_pixels(T_base_optical: np.ndarray, center: np.ndarray, normal: np.ndarray) -> list[Pixel]:
    """A thin (no 2x2 blocks), 8-connected pixel chain tracing the forward-projected crack curve.

    Two-pass arc-length sampling: a coarse pass estimates the projected curve's pixel length,
    then a second pass resamples at roughly `CHAIN_ANCHOR_PX_FACTOR` anchors per pixel of that
    length. Sampling a smooth curve much denser than ~1 px and rounding each axis independently
    produces spurious 3-neighbour "thick corner" pixels (verified empirically for this
    generator's geometry); anchor spacing at or below 1 px/anchor avoids it. Consecutive
    *rounded* anchors are connected by a true integer Bresenham segment, never by independent
    per-axis rounding of intermediate points.
    """
    coarse = crack_curve_points_base(center, normal, n_samples=CHAIN_COARSE_SAMPLES)
    r0, c0, _ = project_to_pixels(T_base_optical, coarse)
    length_px = float(np.sum(np.hypot(np.diff(r0), np.diff(c0))))
    n_samples = max(CHAIN_COARSE_SAMPLES, int(math.ceil(length_px * CHAIN_ANCHOR_PX_FACTOR)) + 5)

    curve = crack_curve_points_base(center, normal, n_samples=n_samples)
    rows, cols, _ = project_to_pixels(T_base_optical, curve)

    chain: list[Pixel] = [(round(rows[0]), round(cols[0]))]
    for i in range(len(rows) - 1):
        p0 = (round(rows[i]), round(cols[i]))
        p1 = (round(rows[i + 1]), round(cols[i + 1]))
        if p0 == p1:
            continue
        for p in _bresenham_line(*p0, *p1)[1:]:
            if chain[-1] != p:
                chain.append(p)
    return chain


def _stamp_disk(canvas: np.ndarray, center_rc: Pixel, radius: float) -> None:
    height, width = canvas.shape
    r0, c0 = center_rc
    r_lo, r_hi = max(0, int(r0 - radius) - 1), min(height, int(r0 + radius) + 2)
    c_lo, c_hi = max(0, int(c0 - radius) - 1), min(width, int(c0 + radius) + 2)
    if r_lo >= r_hi or c_lo >= c_hi:
        return
    rr, cc = np.mgrid[r_lo:r_hi, c_lo:c_hi]
    hit = (rr - r0) ** 2 + (cc - c0) ** 2 <= radius**2
    canvas[r_lo:r_hi, c_lo:c_hi] |= hit


def _pick_hole_runs(chain_pixels: list[Pixel]) -> tuple[list[Pixel], list[Pixel]]:
    """Two index ranges along `chain_pixels`: a `HOLE_SHORT_LEN`-run and a `HOLE_LONG_LEN`-run,
    both clear of the chain's ends and of each other by at least `HOLE_GAP`."""
    n = len(chain_pixels)
    short_start = HOLE_EDGE_MARGIN
    long_start = short_start + HOLE_SHORT_LEN + HOLE_GAP
    needed = long_start + HOLE_LONG_LEN + HOLE_EDGE_MARGIN
    if n < needed:
        raise ValueError(f"crack chain too short for the requested holes: have {n} px, need >= {needed}")
    short_run = chain_pixels[short_start : short_start + HOLE_SHORT_LEN]
    long_run = chain_pixels[long_start : long_start + HOLE_LONG_LEN]
    return short_run, long_run


# ---------------------------------------------------------------------------
# Depth rendering
# ---------------------------------------------------------------------------


def render_depth_m(
    T_base_optical: np.ndarray, center: np.ndarray, normal: np.ndarray, height: int, width: int, mask: np.ndarray
) -> np.ndarray:
    """Noise-free per-pixel surface depth (metres, optical frame), `NaN` off-plane/behind-camera.

    Pixels under `mask` (the crack cavity) get `+ CAVITY_DEPTH_M`, pushed farther from the camera
    along the surface normal (R-08: a crack is a cavity, deeper than the surrounding surface).
    """
    rows, cols = np.meshgrid(np.arange(height), np.arange(width), indexing="ij")
    points, depth = ray_plane_points_base(T_base_optical, center, normal, rows.astype(np.float64), cols.astype(np.float64))
    del points
    invalid = ~np.isfinite(depth) | (depth <= 0.0)
    depth = np.where(invalid, np.nan, depth)
    depth = depth + np.where(mask, CAVITY_DEPTH_M, 0.0)
    return depth


def depth_m_to_u16(depth_m: np.ndarray, zero_mask: np.ndarray | None = None) -> np.ndarray:
    """Quantise metres to the `uint16` z16 raw-depth encoding (ADR-012); invalid/zeroed -> 0."""
    invalid = ~np.isfinite(depth_m) | (depth_m <= 0.0)
    raw = np.where(invalid, 0.0, np.round(depth_m / DEPTH_SCALE_M_PER_UNIT))
    raw = np.clip(raw, 0, 65535)
    out = raw.astype(np.uint16)
    if zero_mask is not None:
        out[zero_mask] = 0
    return out


# ---------------------------------------------------------------------------
# Ground truth container
# ---------------------------------------------------------------------------


@dataclass
class SceneGroundTruth:
    case_id: str
    seed: int
    tilt_deg: float
    view_tilt_deg: float
    depth_noise: bool
    holes: bool
    image_height: int
    image_width: int
    center_base: np.ndarray
    normal_base: np.ndarray
    center_source: str
    q: np.ndarray
    roll_deg: float
    ik_residual_norm: float
    T_base_optical: np.ndarray
    curve_points_base: np.ndarray
    chain_pixels: list[Pixel]
    hole_runs: dict[str, list[Pixel]] = field(default_factory=dict)

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "case_id": self.case_id,
            "seed": self.seed,
            "tilt_deg": self.tilt_deg,
            "view_tilt_deg": self.view_tilt_deg,
            "depth_noise": self.depth_noise,
            "holes": self.holes,
            "image_height": self.image_height,
            "image_width": self.image_width,
            "center_base_m": self.center_base.tolist(),
            "normal_base": self.normal_base.tolist(),
            "center_source": self.center_source,
            "q_rad": self.q.tolist(),
            "roll_deg": self.roll_deg,
            "ik_residual_norm": self.ik_residual_norm,
            "T_base_link_camera_color_optical_frame": self.T_base_optical.tolist(),
            "curve_points_base_m": self.curve_points_base.tolist(),
            "chain_pixels": [[r, c] for r, c in self.chain_pixels],
            "hole_runs": {k: [[r, c] for r, c in v] for k, v in self.hole_runs.items()},
        }


# ---------------------------------------------------------------------------
# case_map.json helper
# ---------------------------------------------------------------------------


def _add_case_map_entry(root: Path, case_id: str) -> None:
    path = Path(root) / "data" / "case_map.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {"cases": []}
    if path.is_file():
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            doc = {"cases": []}
    cases = [c for c in doc.get("cases", []) if c.get("case_id") != case_id]
    cases.append({"case_id": case_id, "downscaled": False})
    doc["cases"] = cases
    path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# Scene assembly
# ---------------------------------------------------------------------------


def _color_intrinsics_dict() -> dict[str, Any]:
    return {
        "width": IMAGE_WIDTH,
        "height": IMAGE_HEIGHT,
        "fx": FX,
        "fy": FY,
        "ppx": PPX,
        "ppy": PPY,
        "model": INTRINSICS_MODEL,
        "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
    }


def generate_case(
    root: Path,
    case_id: str,
    *,
    seed: int,
    tilt_deg: float = 0.0,
    view_tilt_deg: float = 0.0,
    depth_noise: bool = False,
    holes: bool = False,
    end_effector: "kin.EndEffector | None" = None,
) -> SceneGroundTruth:
    """Generate one full synthetic case under `root`'s `data/` tree and return its ground truth.

    Writes: `data/skeletons/{case}_skeleton.png`, the mask (`naming.mask_path`), a colour
    placeholder and the aligned depth PNG under `data/synthetic/`, a §9 capture record
    (`data/captures/{case}_capture.json`), a `case_map.json` entry, and the ground-truth JSON
    (`data/synthetic/{case}_gt.json`).
    """
    root = Path(root)
    rng = np.random.default_rng(seed)

    chain = kin.load_chain()
    ee = end_effector if end_effector is not None else kin.load_end_effector()

    center, center_source = specimen_center(root)
    normal = plane_normal(tilt_deg)
    view_tilt_rad = math.radians(view_tilt_deg)

    q, T_target, resid, roll_deg = solve_camera_pose(chain, ee, center, normal, view_tilt_rad)
    if resid >= IK_RESIDUAL_TOL:
        raise RuntimeError(
            f"synth_scene3d: view-pose IK did not converge for case {case_id!r} "
            f"(tilt_deg={tilt_deg}, view_tilt_deg={view_tilt_deg}): "
            f"best residual norm {resid} >= {IK_RESIDUAL_TOL}"
        )
    chain.fk(q)  # re-assert q is within the URDF's joint limits (raises KinematicsError otherwise)

    T_fk = chain.fk(q)
    T_base_camera_link = T_fk @ ee.T_gripper_link_camera_link
    T_base_optical = T_base_camera_link @ kin.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME
    assert np.allclose(T_base_optical, T_target, atol=1e-6), "FK(q) disagrees with the IK target pose"

    curve_base = crack_curve_points_base(center, normal)
    rows_f, cols_f, depths_f = project_to_pixels(T_base_optical, curve_base)
    if not np.all(np.isfinite(depths_f)) or np.any(depths_f <= 0.0):
        raise RuntimeError("synth_scene3d: crack curve projects behind the camera")
    if np.any(rows_f < 0) or np.any(rows_f >= IMAGE_HEIGHT) or np.any(cols_f < 0) or np.any(cols_f >= IMAGE_WIDTH):
        raise RuntimeError("synth_scene3d: crack curve projects outside the image")
    chain_pixels = build_chain_pixels(T_base_optical, center, normal)

    mask = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH), dtype=bool)
    for p in chain_pixels:
        _stamp_disk(mask, p, MASK_RADIUS_PX)

    skeleton = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH), dtype=bool)
    for r, c in chain_pixels:
        skeleton[r, c] = True

    hole_runs: dict[str, list[Pixel]] = {}
    extra_zero = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH), dtype=bool)
    if holes:
        short_run, long_run = _pick_hole_runs(chain_pixels)
        hole_runs = {"short": short_run, "long": long_run}
        for p in short_run:
            _stamp_disk(extra_zero, p, HOLE_SHORT_ZERO_RADIUS_PX)
        for p in long_run:
            _stamp_disk(extra_zero, p, HOLE_LONG_ZERO_RADIUS_PX)

    depth_m = render_depth_m(T_base_optical, center, normal, IMAGE_HEIGHT, IMAGE_WIDTH, mask)
    if depth_noise:
        sigma = depth_uncertainty_m(np.nan_to_num(depth_m, nan=VIEW_DISTANCE_M), FX)
        depth_m = depth_m + rng.normal(size=depth_m.shape) * sigma

    depth_u16 = depth_m_to_u16(depth_m, zero_mask=extra_zero)

    # ---- write artefacts -------------------------------------------------
    skeleton_path = root / naming.skeleton_path(case_id)
    skeleton_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray((skeleton.astype(np.uint8) * 255)).save(skeleton_path)

    mask_path = root / naming.mask_path(case_id)
    mask_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray((mask.astype(np.uint8) * 255)).save(mask_path)

    synthetic_dir = root / "data" / "synthetic"
    synthetic_dir.mkdir(parents=True, exist_ok=True)

    color_path = synthetic_dir / f"{case_id}_color.png"
    Image.fromarray(np.full((IMAGE_HEIGHT, IMAGE_WIDTH, 3), 128, dtype=np.uint8)).save(color_path)

    depth_rel = f"data/synthetic/{case_id}_depth.png"
    Image.fromarray(depth_u16, mode="I;16").save(root / depth_rel)

    gt_path = synthetic_dir / f"{case_id}_gt.json"

    kinematic_model_path = str(kin.DEFAULT_URDF)
    doc = cr.build_record(
        case_id=case_id,
        synthetic=True,
        image_height=IMAGE_HEIGHT,
        image_width=IMAGE_WIDTH,
        color_intrinsics=_color_intrinsics_dict(),
        depth_file=depth_rel,
        depth_scale_m_per_unit=DEPTH_SCALE_M_PER_UNIT,
        depth_aligned_to="color",
        source_kind="synthetic",
        source_ref=str(gt_path),
        source_frame_index=None,
        capture_stamp_ns=1_000_000_000_000,
        joint_names=list(chain.joint_names),
        positions_rad=[float(v) for v in q],
        robot_stamp_ns=1_000_000_000_000,
        robot_stamp_source="synth_scene3d",
        kinematic_model_path=kinematic_model_path,
        kinematic_model_sha256=kin.file_sha256(kin.DEFAULT_URDF),
        end_effector_path="config/robot/end_effector.yaml",
        end_effector_sha256=ee.sha256,
        wrist_camera_value_status=ee.wrist_camera_value_status,
        tool_value_status=ee.tool_value_status,
        optical_xyz_m=[0.0, 0.0, 0.0],
        optical_quat_xyzw=list(
            kin.transform_to_xyz_quat(kin.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME)[1]
        ),
        optical_source="nominal_d405",
    )
    cr.write_record(doc, cr.capture_record_path(root, case_id))

    _add_case_map_entry(root, case_id)

    gt = SceneGroundTruth(
        case_id=case_id,
        seed=seed,
        tilt_deg=tilt_deg,
        view_tilt_deg=view_tilt_deg,
        depth_noise=depth_noise,
        holes=holes,
        image_height=IMAGE_HEIGHT,
        image_width=IMAGE_WIDTH,
        center_base=center,
        normal_base=normal,
        center_source=center_source,
        q=np.asarray(q, dtype=np.float64),
        roll_deg=roll_deg,
        ik_residual_norm=resid,
        T_base_optical=T_base_optical,
        curve_points_base=curve_base,
        chain_pixels=chain_pixels,
        hole_runs=hole_runs,
    )
    gt_path.write_text(json.dumps(gt.to_json_dict(), indent=2) + "\n", encoding="utf-8")
    return gt


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _ensure_project_scaffold(out: Path) -> None:
    """Minimal `config/project.yaml` + data dirs under `out`, mirroring `tests/conftest.py`."""
    real_project_yaml = Path(__file__).resolve().parents[1] / "config" / "project.yaml"
    (out / "config").mkdir(parents=True, exist_ok=True)
    project_yaml = out / "config" / "project.yaml"
    if not project_yaml.is_file():
        shutil.copy(real_project_yaml, project_yaml)
    cfg = load_config(root=out)
    for directory in cfg.paths.values():
        Path(directory).mkdir(parents=True, exist_ok=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="synth_scene3d.py",
        description="Deterministic synthetic eye-in-hand scene generator (GEOM-08.8).",
    )
    parser.add_argument("--out", type=Path, required=True, metavar="ROOT")
    parser.add_argument("--case-id", default="synthetic_case", metavar="CASE_ID")
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--tilt-deg", type=float, choices=[0.0, 15.0], default=0.0)
    parser.add_argument("--view-tilt-deg", type=float, choices=[0.0, 15.0], default=0.0)
    parser.add_argument("--depth-noise", action="store_true")
    parser.add_argument("--holes", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    out = args.out.resolve()
    _ensure_project_scaffold(out)
    gt = generate_case(
        out, args.case_id, seed=args.seed, tilt_deg=args.tilt_deg, view_tilt_deg=args.view_tilt_deg,
        depth_noise=args.depth_noise, holes=args.holes,
    )
    print(
        f"synth_scene3d: wrote case {args.case_id!r} under {out} "
        f"(tilt_deg={args.tilt_deg}, view_tilt_deg={args.view_tilt_deg}, "
        f"ik_residual_norm={gt.ik_residual_norm:.3e}, "
        f"roll_deg={gt.roll_deg}, chain_px={len(gt.chain_pixels)})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

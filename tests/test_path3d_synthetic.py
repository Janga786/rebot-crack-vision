"""tests/test_path3d_synthetic.py — GEOM-08.8 synthetic ground-truth verification of the
pixel -> base_link 3D path lifting pipeline (`crackvision.paths` -> `crackvision.path3d`).

Generates a deterministic synthetic scene (`tools.synth_scene3d`) under the nominal eye-in-hand
chain, runs the real pipeline over it, and scores the result against the generator's own
independently-derived ground truth (ray/plane intersection + forward projection -- never
`crackvision.geometry.deproject_pixels`/`lift3d`/`tool_waypoints`). See
docs/geometry/PATH3D_VERIFICATION.md for the measured numbers this test asserts and their
limitations (software consistency under a nominal chain, not physical accuracy).
"""

from __future__ import annotations

import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from synth_scene3d import (  # noqa: E402
    _ensure_project_scaffold,
    generate_case,
    ray_plane_points_base,
)

from crackvision import capture_record as cr  # noqa: E402
from crackvision import kinematics as kin  # noqa: E402
from crackvision.config import load_config  # noqa: E402
from crackvision.logging_setup import EXIT_PRECONDITION  # noqa: E402
from crackvision.path3d import build_case_paths3d  # noqa: E402
from crackvision.path3d import main as path3d_main  # noqa: E402
from crackvision.paths import main as paths_main  # noqa: E402

SEED = 1
NOISE_SEED = 42

# docs/INTERFACES.md §10.3 defaults, matched explicitly here (rather than relying on CLI
# argparse defaults) since the tests call `build_case_paths3d` directly.
ANNULUS_INNER_PX = 3
ANNULUS_OUTER_PX = 8
MIN_FRACTION_VALID = 0.3
MAX_GAP_PX = 5
PIXEL_SIGMA_PX = 1.0
WAYPOINT_SPACING_M = 0.002
TRACE_CLEARANCE_M = 0.01
APPROACH_CLEARANCE_M = 0.04

MEDIAN_ERROR_MAX_M = 0.0005
MAX_ERROR_MAX_M = 0.0015
NORMAL_ANGLE_MAX_DEG = 1.0
CLEARANCE_ERROR_MAX_M = 0.0005
SIGMA_COVERAGE_MIN_FRACTION = 0.90


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


def _zero_sigma_end_effector(tmp_path: Path) -> "kin.EndEffector":
    """A `measured`, zero-sigma copy of the real `end_effector.yaml` (§10.4 calibration term
    disabled), so the noise-consistency test exercises only the sensor (pixel + depth) terms."""
    doc = yaml.safe_load(kin.DEFAULT_END_EFFECTOR.read_text(encoding="utf-8"))
    doc["tool"]["value_status"] = "measured"
    doc["tool"]["uncertainty"] = {
        "position_sigma_m": 0.0,
        "source": "tests/test_path3d_synthetic.py fixture: zero calibration sigma",
    }
    doc["wrist_camera"]["value_status"] = "measured"
    doc["wrist_camera"]["uncertainty"] = {
        "position_sigma_m": 0.0,
        "rotation_sigma_rad": 0.0,
        "source": "tests/test_path3d_synthetic.py fixture: zero calibration sigma",
    }
    path = tmp_path / "zero_sigma_end_effector.yaml"
    path.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return kin.load_end_effector(path=path)


def _run_pipeline(root: Path, case_id: str, *, end_effector: "kin.EndEffector") -> dict:
    """Real `crackvision.paths` CLI, then `crackvision.path3d.build_case_paths3d` directly (so a
    custom, e.g. zero-sigma, `end_effector` can be injected without touching the real file)."""
    assert paths_main(["--root", str(root), "--cases", case_id, "-q"]) == 0

    cfg = load_config(root=root)
    case_map = json.loads((root / "data" / "case_map.json").read_text(encoding="utf-8"))
    entry = next(c for c in case_map["cases"] if c["case_id"] == case_id)

    result = build_case_paths3d(
        cfg, case_id, entry,
        end_effector=end_effector,
        annulus_inner_px=ANNULUS_INNER_PX,
        annulus_outer_px=ANNULUS_OUTER_PX,
        min_fraction_valid=MIN_FRACTION_VALID,
        max_gap_px=MAX_GAP_PX,
        pixel_sigma_px=PIXEL_SIGMA_PX,
        waypoint_spacing_m=WAYPOINT_SPACING_M,
        trace_clearance_m=TRACE_CLEARANCE_M,
        approach_clearance_m=APPROACH_CLEARANCE_M,
        max_skew_s=cr.DEFAULT_MAX_SKEW_S,
        accept_end_effector_change=False,
    )
    assert result["status"] == "ok", result
    return result["doc"]


def _main_polyline(doc: dict) -> dict:
    assert len(doc["components"]) == 1
    return doc["components"][0]["polylines"][0]


def _generate(tmp_path_factory, name: str, *, tilt_deg: float = 0.0, view_tilt_deg: float = 0.0,
              holes: bool = False, depth_noise: bool = False, seed: int = SEED, end_effector=None):
    root = tmp_path_factory.mktemp(name)
    _ensure_project_scaffold(root)
    gt = generate_case(
        root, "synthetic_case", seed=seed, tilt_deg=tilt_deg, view_tilt_deg=view_tilt_deg,
        depth_noise=depth_noise, holes=holes, end_effector=end_effector,
    )
    return root, gt


# ---------------------------------------------------------------------------
# Fixtures: one scene generation per scenario, reused by every assertion about it
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module", params=[0.0, 15.0], ids=["tilt0", "tilt15"])
def flat_scene(tmp_path_factory, request):
    """Noise-free, hole-free scene at the ADR-014 view pose, at 0 deg and 15 deg *view* tilt --
    i.e. the camera optical axis is genuinely offset from the surface anti-normal by
    `request.param` degrees (the surface itself stays flat, `tilt_deg=0`); see
    `tools/synth_scene3d.py::camera_target_pose`."""
    view_tilt_deg = request.param
    root, gt = _generate(tmp_path_factory, f"viewtilt_{int(view_tilt_deg)}", view_tilt_deg=view_tilt_deg)
    ee = kin.load_end_effector()
    doc = _run_pipeline(root, "synthetic_case", end_effector=ee)
    return root, gt, doc, ee


@pytest.fixture(scope="module")
def holes_scene(tmp_path_factory):
    root, gt = _generate(tmp_path_factory, "holes", tilt_deg=0.0, holes=True)
    ee = kin.load_end_effector()
    doc = _run_pipeline(root, "synthetic_case", end_effector=ee)
    return root, gt, doc, ee


@pytest.fixture(scope="module")
def noise_scene(tmp_path_factory):
    """Gaussian depth noise (GEOM-02 stereo model) + a zero-sigma measured end_effector, so
    `cov_base`/`sigma_base_m` reflect only the sensor terms (§10.4 `uncertainty_model.terms`
    `pixel_noise_px`/`depth_uncertainty_stereo`), not `wrist_camera_calibration`/`tool_calibration`."""
    tmp_dir = tmp_path_factory.mktemp("noise")
    ee0 = _zero_sigma_end_effector(tmp_dir)
    root, gt = _generate(
        tmp_path_factory, "noise_scene", tilt_deg=0.0, depth_noise=True, seed=NOISE_SEED, end_effector=ee0,
    )
    doc = _run_pipeline(root, "synthetic_case", end_effector=ee0)
    return root, gt, doc, ee0


# ---------------------------------------------------------------------------
# Generator self-checks (IK convergence, image bounds)
# ---------------------------------------------------------------------------


def test_ik_converges_within_tolerance_and_joint_limits(flat_scene):
    _root, gt, _doc, _ee = flat_scene
    assert gt.ik_residual_norm < 1e-6
    chain = kin.load_chain()
    chain.fk(gt.q)  # raises KinematicsError if any joint is outside its URDF limits


def test_capture_distance_and_anti_normal_angle(flat_scene):
    """ADR-014 §2 view phase: camera_link 0.25 m from the surface, boresight within
    `gt.view_tilt_deg` of the anti-normal (0 deg and the 15 deg band edge are both exercised by
    `flat_scene`'s params -- see `tools/synth_scene3d.py::camera_target_pose`)."""
    _root, gt, _doc, _ee = flat_scene
    cam_pos = gt.T_base_optical[:3, 3]
    dist = float(np.linalg.norm(cam_pos - gt.center_base))
    assert math.isclose(dist, 0.25, abs_tol=1e-9)

    boresight = gt.T_base_optical[:3, 2]
    # n_optical_z = dot(boresight, normal_base); the view angle is arccos(-n_optical_z) (the
    # acceptance condition's "arccos of the optical-frame normal's z" phrasing, re-derived in
    # tools/synth_scene3d.py::camera_target_pose's docstring).
    n_optical_z = float(np.dot(boresight, gt.normal_base))
    view_angle_deg = math.degrees(math.acos(np.clip(-n_optical_z, -1.0, 1.0)))
    assert math.isclose(view_angle_deg, gt.view_tilt_deg, abs_tol=0.1)


# ---------------------------------------------------------------------------
# Noise-free position + normal + waypoint accuracy (acceptance criteria #2, #3)
# ---------------------------------------------------------------------------


def test_position_accuracy_vs_ground_truth_curve(flat_scene):
    _root, gt, doc, _ee = flat_scene
    main_pl = _main_polyline(doc)
    valid_points = [p for p in main_pl["points"] if p["valid"]]
    assert len(valid_points) > 50

    p_base = np.array([p["p_base_m"] for p in valid_points])
    tree = cKDTree(gt.curve_points_base)
    dist_m, _ = tree.query(p_base)

    median_m, max_m = float(np.median(dist_m)), float(np.max(dist_m))
    assert median_m <= MEDIAN_ERROR_MAX_M, f"median error {median_m * 1e3:.3f} mm"
    assert max_m <= MAX_ERROR_MAX_M, f"max error {max_m * 1e3:.3f} mm"


def test_normal_accuracy_vs_ground_truth(flat_scene):
    _root, gt, doc, _ee = flat_scene
    main_pl = _main_polyline(doc)
    n_base = np.array([p["n_base"] for p in main_pl["points"] if p["valid"]])
    cos_angle = np.clip(n_base @ gt.normal_base, -1.0, 1.0)
    angle_deg = np.degrees(np.arccos(cos_angle))
    assert float(angle_deg.max()) <= NORMAL_ANGLE_MAX_DEG


def test_trace_waypoint_boresight_and_clearance(flat_scene):
    """+X . n_true <= -cos(1 deg) (§10.5 +X = -n_out, validated against the *true* normal) and
    the waypoint's standoff from the true plane matches its declared `clearance_m` to <= 0.5 mm."""
    _root, gt, doc, _ee = flat_scene
    main_pl = _main_polyline(doc)

    dot_max = -math.cos(math.radians(1.0))
    clearance_errs = []
    for seg in main_pl["segments"]:
        for wp in seg["waypoints"]:
            R = Rotation.from_quat(wp["quat_xyzw"]).as_matrix()
            x_axis = R[:, 0]
            dot = float(x_axis @ gt.normal_base)
            assert dot <= dot_max, f"waypoint {wp['phase']} +X.n_true={dot}"

            pos = np.array(wp["position_m"])
            signed_dist = float((pos - gt.center_base) @ gt.normal_base)
            clearance_errs.append(abs(signed_dist - wp["clearance_m"]))

    assert clearance_errs
    assert max(clearance_errs) <= CLEARANCE_ERROR_MAX_M


def test_cavity_does_not_bias_surface_depth(flat_scene):
    """R-08: the annulus sampler must never pick up the +3 mm crack-cavity depth; every valid
    point's `depth_m` stays at the noise-free 0.25 m view distance along its own ray."""
    _root, gt, doc, _ee = flat_scene
    main_pl = _main_polyline(doc)
    for p in main_pl["points"]:
        if not p["valid"]:
            continue
        _point_true, depth_true = ray_plane_points_base(
            gt.T_base_optical, gt.center_base, gt.normal_base,
            np.array([float(p["row"])]), np.array([float(p["col"])]),
        )
        assert abs(p["depth_m"] - float(depth_true[0])) <= MAX_ERROR_MAX_M


# ---------------------------------------------------------------------------
# Noise-consistency (acceptance criterion #4)
# ---------------------------------------------------------------------------


def test_depth_noise_within_3sigma_zero_calibration(noise_scene):
    _root, gt, doc, ee0 = noise_scene
    assert ee0.camera_sigma_pos_m == 0.0
    assert ee0.camera_sigma_rot_rad == 0.0
    assert ee0.tool_sigma_pos_m == 0.0

    main_pl = _main_polyline(doc)
    valid_points = [p for p in main_pl["points"] if p["valid"] and not p["interpolated"]]
    assert len(valid_points) > 50

    rows = np.array([float(p["row"]) for p in valid_points])
    cols = np.array([float(p["col"]) for p in valid_points])
    p_true, _ = ray_plane_points_base(gt.T_base_optical, gt.center_base, gt.normal_base, rows, cols)
    p_actual = np.array([p["p_base_m"] for p in valid_points])
    sigma_base = np.array([p["sigma_base_m"] for p in valid_points])

    err = p_actual - p_true
    within = np.abs(err) <= 3.0 * sigma_base
    for axis in range(3):
        fraction = float(np.mean(within[:, axis]))
        assert fraction >= SIGMA_COVERAGE_MIN_FRACTION, f"axis {axis}: only {fraction:.3f} within 3 sigma"


# ---------------------------------------------------------------------------
# Injected invalid-depth holes: §10.3 interpolation vs. split (acceptance criterion #5)
# ---------------------------------------------------------------------------


def test_short_hole_interpolated_long_hole_splits(holes_scene):
    _root, gt, doc, _ee = holes_scene
    main_pl = _main_polyline(doc)
    points = main_pl["points"]

    interpolated_idx = [i for i, p in enumerate(points) if p["interpolated"]]
    assert interpolated_idx, "expected the short hole to produce at least one interpolated point"
    for i in interpolated_idx:
        # §10.2: a valid point (interpolated or measured) always carries reason: null.
        assert points[i]["valid"] is True
        assert points[i]["reason"] is None
        assert points[i]["p_base_m"] is not None

    assert len(main_pl["segments"]) >= 2, "expected the long hole to split the polyline"

    invalid_reasons = {points[i]["reason"] for i in range(len(points)) if not points[i]["valid"]}
    assert invalid_reasons, "expected at least one genuinely-invalid (split, not bridged) point"
    assert doc["counts"]["invalid_by_reason"]
    assert doc["counts"]["interpolated"] == len(interpolated_idx)


# ---------------------------------------------------------------------------
# Refusal rules + eligibility (acceptance criterion #6, docs/INTERFACES.md §9.4/§10.6)
# ---------------------------------------------------------------------------


def test_nominal_synthetic_capture_is_execution_ineligible(flat_scene):
    _root, _gt, doc, _ee = flat_scene
    assert doc["execution_eligible"] is False
    for reason in ("wrist_camera_nominal", "tool_nominal", "optical_frames_nominal", "synthetic_capture"):
        assert reason in doc["ineligible_reasons"]


def test_capture_record_without_robot_block_exits_3(tmp_path_factory):
    root, _gt = _generate(tmp_path_factory, "norobot", tilt_deg=0.0)
    assert paths_main(["--root", str(root), "--cases", "synthetic_case", "-q"]) == 0

    cap_path = cr.capture_record_path(root, "synthetic_case")
    doc = json.loads(cap_path.read_text(encoding="utf-8"))
    del doc["robot"]
    cap_path.write_text(json.dumps(doc), encoding="utf-8")

    exit_code = path3d_main(["--root", str(root), "--cases", "synthetic_case", "-q"])
    assert exit_code == EXIT_PRECONDITION == 3

#!/usr/bin/env python3
"""Deterministic generator for the MOT-06.7 Cartesian-plan smoke fixtures (TEST-ONLY).

Run with the system python3 (numpy only); it rewrites, next to this file:

  scene.yaml         crackvision.scene_config/1: table (top at z=0) + a 0.20x0.20 m `specimen`
                     box centred at the recommended placement (0.26, 0.0) whose top face is
                     exactly at SURFACE_Z, so a 0.01 m tool clearance is collision-free by
                     construction. Test-only: config/scene/scene.yaml is never touched.
  capture_stub.json  the only part of a §9 capture record plan_crack_path consumes (the `robot`
                     joint state, §13.1 start state); marked synthetic, not a full §9 record.
  paths3d.json       crackvision.paths3d/1 (§10) on the flat SURFACE_Z plane, waypoints per §10.5
                     (2 mm spacing, +X = -n, parallel-transported +Z, trace at 0.01 m,
                     approach/retract at 0.04 m), with four segments:
                       crack_a main      straight 0.06 m, reachable          -> must be `full`
                       crack_a branch 0  120 deg arc, r = 0.03 m, reachable  -> must be `full`
                       crack_far main    straight at x ~ 0.62..0.66 m, outside reach -> must fail
                       crack_sunk main   surface 5 mm below the specimen top -> surface_scene_mismatch
  expected.json      the numbers test_cartesian_plan.sh asserts against (derived here, not from
                     any plan output).

`points[]` holds one entry per trace waypoint surface point (the fixture has no image, so there is
no denser pixel chain); pixel coordinates come from a synthetic downward pinhole camera that sees
every segment. Uncertainty follows §10.4 with the nominal calibration priors, which (like real
nominal data) puts every waypoint outside its budget, so `execution_eligible` is false.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO_REL = "ros2_ws/src/crackvision_motion/test/fixtures/cartesian/smoke"

SURFACE_Z = 0.010                 # specimen top face, base_link z (m)
CENTER_XY = (0.26, 0.0)           # config/motion/specimen_placement.yaml placement.center_xy_m
SPECIMEN_XY = (0.20, 0.20)        # placement.footprint_m
SPACING = 0.002                   # §10.5 waypoint_spacing_m
TRACE_CLEARANCE = 0.01            # §10.5 trace_clearance_m
OFFSET = 0.04                     # §10.5 approach_retract_offset_m
SINK = 0.005                      # crack_sunk surface depth below the specimen top
START_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# synthetic downward colour camera (D405-like intrinsics); optical +Z = base -Z
IMG_W, IMG_H, FX, FY, PPX, PPY = 640, 480, 430.0, 430.0, 320.0, 240.0
CAM_POS = np.array([0.44, 0.0, 0.50])
R_BASE_OPT = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # columns: optical x,y,z
SIGMA_PX = 1.0
BASELINE_M, DISPARITY_SIGMA_PX = 0.018, 0.08        # geometry.D405_NOMINAL_* priors
CAM_POS_SIGMA, CAM_ROT_SIGMA = 0.010, math.radians(5.0)   # §10.4 nominal wrist_camera prior
TOOL_POS_SIGMA = 0.005                                    # §10.4 nominal tool prior
UNMODELLED = ["joint_encoder_and_fk", "depth_bias", "distortion_jacobian",
              "capture_time_skew_motion", "thermal_drift"]


def _r(x, nd=9):
    return [round(float(v), nd) + 0.0 for v in x]   # + 0.0 folds -0.0 into 0.0


def _unit(v):
    return v / np.linalg.norm(v)


def resample(dense: np.ndarray) -> np.ndarray:
    """§10.5 point 1: resample a dense polyline by 3D arc length at SPACING, both ends kept."""
    seg = np.linalg.norm(np.diff(dense, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    stations = list(np.arange(0.0, total, SPACING))
    if total - stations[-1] < 1e-9:
        stations[-1] = total
    else:
        stations.append(total)
    return np.stack([np.interp(stations, s, dense[:, k]) for k in range(3)], axis=1)


def quat_from_axes(x, y, z):
    m = np.stack([x, y, z], axis=1)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    q = q / np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def point_record(p_base: np.ndarray, n_base: np.ndarray):
    p_opt = R_BASE_OPT.T @ (p_base - CAM_POS)
    x, y, z = p_opt
    u, v = PPX + FX * x / z, PPY + FY * y / z
    assert 0 <= u < IMG_W and 0 <= v < IMG_H, (u, v)
    sigma_z = z * z * DISPARITY_SIGMA_PX / (FX * BASELINE_M)
    jac = np.array([[z / FX, 0.0, x / z], [0.0, z / FY, y / z], [0.0, 0.0, 1.0]])
    cov_opt = jac @ np.diag([SIGMA_PX ** 2, SIGMA_PX ** 2, sigma_z ** 2]) @ jac.T
    cov_base = R_BASE_OPT @ cov_opt @ R_BASE_OPT.T + (
        CAM_POS_SIGMA ** 2 + (np.linalg.norm(p_opt) * CAM_ROT_SIGMA) ** 2) * np.eye(3)
    return {
        "row": int(round(v)), "col": int(round(u)), "u": int(round(u)), "v": int(round(v)),
        "valid": True, "reason": None, "interpolated": False,
        "depth_m": round(float(z), 9), "fraction_valid": 1.0,
        "p_optical_m": _r(p_opt), "p_base_m": _r(p_base), "n_base": _r(n_base),
        "normal_rms_m": 0.0, "sigma_normal_rad": 0.0,
        "cov_base_m2": _r(cov_base.reshape(-1), 12), "sigma_base_m": _r(np.sqrt(np.diag(cov_base)), 9),
    }, cov_base


def build_segment(dense_surface: np.ndarray, z_ref: np.ndarray):
    """Points + one §10.5 segment for a flat (normal +Z) surface polyline."""
    surf = resample(dense_surface)
    n = np.array([0.0, 0.0, 1.0])
    points, waypoints, covs = [], [], []
    for p in surf:
        rec, cov = point_record(p, n)
        points.append(rec)
        covs.append(cov)
    x_axis = -n
    zt = z_ref - np.dot(z_ref, x_axis) * x_axis          # §10.5 point 3 projection
    quats = []
    for _ in surf:
        zt = _unit(zt - np.dot(zt, x_axis) * x_axis)     # parallel transport (constant normal)
        quats.append(quat_from_axes(x_axis, np.cross(zt, x_axis), zt))

    def wp(phase, i, clearance):
        sigma_n = math.sqrt(float(n @ covs[i] @ n) + TOOL_POS_SIGMA ** 2)
        return {
            "phase": phase, "position_m": _r(surf[i] + clearance * n), "quat_xyzw": _r(quats[i]),
            "surface_point_m": _r(surf[i]), "normal_base": _r(n), "clearance_m": clearance,
            "sigma_along_normal_m": round(sigma_n, 9),
            "sigma_max_m": round(float(np.sqrt(np.diag(covs[i])).max()), 9),
            "interpolated": False, "within_budget": bool(3.0 * sigma_n <= clearance),
        }

    waypoints.append(wp("approach", 0, OFFSET))
    waypoints += [wp("trace", i, TRACE_CLEARANCE) for i in range(len(surf))]
    waypoints.append(wp("retract", len(surf) - 1, OFFSET))
    segment = {"start_index": 0, "end_index": len(points) - 1, "waypoints": waypoints}
    return points, segment


def line(p0, p1, z):
    t = np.linspace(0.0, 1.0, 2001)[:, None]
    a, b = np.array([*p0, z]), np.array([*p1, z])
    return a + t * (b - a)


def arc(center, radius, th0_deg, th1_deg, z):
    th = np.radians(np.linspace(th0_deg, th1_deg, 4001))
    return np.stack([center[0] + radius * np.cos(th), center[1] + radius * np.sin(th),
                     np.full_like(th, z)], axis=1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out-dir", type=Path, default=HERE,
                        help="where to write the fixtures (default: next to this script); "
                             "test_cartesian_plan.sh regenerates into a temp dir and diffs")
    out_dir = parser.parse_args().out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    z_ref = np.array([1.0, 0.0, 0.0])   # stand-in for the capture-pose tool_tip Z (no FK here)
    cx, cy = CENTER_XY
    specs = [
        # (component_id, order_index, kind, branch_index, dense polyline, expected status)
        ("crack_a", 0, "main", None, line((cx - 0.03, cy + 0.02), (cx + 0.03, cy + 0.02), SURFACE_Z), "full"),
        ("crack_a", 0, "branch", 0, arc((cx, cy - 0.01), 0.03, 90.0, -30.0, SURFACE_Z), "full"),
        ("crack_far", 1, "main", None, line((0.62, 0.0), (0.66, 0.0), SURFACE_Z), "failed"),
        ("crack_sunk", 2, "main", None,
         line((cx - 0.06, cy - 0.07), (cx - 0.05, cy - 0.07), SURFACE_Z - SINK), "surface_scene_mismatch"),
    ]
    components = {}
    expected_segments = []
    all_valid = 0
    eligible_budget = True
    for comp_id, order_index, kind, branch_index, dense, status in specs:
        points, segment = build_segment(dense, z_ref)
        all_valid += len(points)
        eligible_budget &= all(w["within_budget"] for w in segment["waypoints"])
        comp = components.setdefault(comp_id, {"order_index": order_index, "component_id": comp_id, "polylines": []})
        comp["polylines"].append({"kind": kind, "branch_index": branch_index, "points": points, "segments": [segment]})
        trace = np.array([w["position_m"] for w in segment["waypoints"] if w["phase"] == "trace"])
        expected_segments.append({
            "segment_ref": {"component_id": comp_id, "polyline_kind": kind, "branch_index": branch_index,
                            "segment_index": 0, "sub_segment": None},
            "expected_status": status,
            "trace_waypoints": len(trace),
            "trace_length_m": float(np.sum(np.linalg.norm(np.diff(trace, axis=0), axis=1))),
        })

    capture = {
        "schema": "crackvision.capture_3d/1",
        "case_id": "mot067_smoke",
        "synthetic": True,
        "note": "TEST-ONLY stub for MOT-06.7: only the robot block (the §13.1 start state) is meaningful.",
        "robot": {"joint_names": [f"joint{i}" for i in range(1, 7)], "positions_rad": START_JOINTS,
                  "stamp_ns": 0, "stamp_source": "synthetic"},
    }
    capture_bytes = (json.dumps(capture, indent=1) + "\n").encode()
    (out_dir / "capture_stub.json").write_bytes(capture_bytes)

    reasons = ["wrist_camera_nominal", "tool_nominal", "optical_frames_nominal", "synthetic_capture"]
    if not eligible_budget:
        reasons.append("uncertainty_exceeds_clearance")
    paths3d = {
        "schema": "crackvision.paths3d/1",
        "case_id": "mot067_smoke",
        "frame": "base_link",
        "image_height": IMG_H,
        "image_width": IMG_W,
        "sources": {
            "paths_json": f"{REPO_REL}/NONE_synthetic_fixture",
            "paths_json_sha256": hashlib.sha256(b"").hexdigest(),
            "capture_record": f"{REPO_REL}/capture_stub.json",
            "capture_record_sha256": hashlib.sha256(capture_bytes).hexdigest(),
            "mask": f"{REPO_REL}/NONE_synthetic_fixture",
            "mask_sha256": hashlib.sha256(b"").hexdigest(),
            "end_effector": {"sha256_at_capture": "synthetic", "sha256_now": "synthetic"},
            "urdf_sha256": "synthetic",
        },
        "parameters": {
            "annulus_inner_px": 3, "annulus_outer_px": 8, "annulus_min_fraction_valid": 0.3,
            "valid_depth_range_m": [0.07, 0.50], "max_gap_px": 5, "sigma_px": SIGMA_PX,
            "waypoint_spacing_m": SPACING, "approach_retract_offset_m": OFFSET,
            "trace_clearance_m": TRACE_CLEARANCE, "roll_free": True, "max_clock_skew_s": 0.05,
        },
        "calibration": {
            "wrist_camera": {"value_status": "nominal", "position_sigma_m": CAM_POS_SIGMA,
                             "rotation_sigma_rad": round(CAM_ROT_SIGMA, 9), "source": "§10.4 nominal prior"},
            "tool": {"value_status": "nominal", "position_sigma_m": TOOL_POS_SIGMA, "source": "§10.4 nominal prior"},
        },
        "uncertainty_model": {"terms": ["pixel_sigma", "depth_sigma", "wrist_camera_pose", "tool_position"],
                              "unmodelled": UNMODELLED},
        "execution_eligible": False,
        "ineligible_reasons": reasons,
        "counts": {"points": all_valid, "valid": all_valid, "interpolated": 0, "invalid_by_reason": {}},
        "components": sorted(components.values(), key=lambda c: c["order_index"]),
    }
    (out_dir / "paths3d.json").write_text(json.dumps(paths3d, indent=1) + "\n")

    sx, sy = SPECIMEN_XY
    scene = f"""# TEST-ONLY fixture scene for MOT-06.7 (scripts/ros/test_cartesian_plan.sh), generated by
# make_fixture.py -- do not hand-edit. Not the production scene (config/scene/scene.yaml).
# The specimen top face is exactly at z = {SURFACE_Z} m, the paths3d.json surface height.
schema: crackvision.scene_config/1
frame: base_link
objects:
  - id: table
    shape: box
    dimensions_m: [1.20, 1.00, 0.05]
    pose:
      frame: base_link
      position_m: [0.35, 0.0, -0.025]
      rpy_rad: [0.0, 0.0, 0.0]
    value_status: nominal
    source: "test fixture: table top = base_link z=0 plane (same box as config/scene/scene.yaml)"
  - id: specimen
    shape: box
    dimensions_m: [{sx}, {sy}, {SURFACE_Z}]
    pose:
      frame: base_link
      position_m: [{cx}, {cy}, {SURFACE_Z / 2}]
      rpy_rad: [0.0, 0.0, 0.0]
    value_status: nominal
    source: "test fixture: recommended placement centre and footprint, top face at the fixture surface height"
allowed_collisions:
  - link_a: table
    link_b: base_link
    reason: "the robot base stands on the table"
    value_status: nominal
    source: "test fixture"
  - link_a: specimen
    link_b: table
    reason: "the specimen rests on the table top"
    value_status: nominal
    source: "test fixture"
"""
    (out_dir / "scene.yaml").write_text(scene)

    expected = {
        "surface_z_m": SURFACE_Z,
        "trace_clearance_m": TRACE_CLEARANCE,
        "approach_retract_offset_m": OFFSET,
        "specimen": {"position_m": [cx, cy, SURFACE_Z / 2], "dimensions_m": [sx, sy, SURFACE_Z]},
        "start_joints": START_JOINTS,
        "segments": expected_segments,
        "trace_waypoints_total": sum(s["trace_waypoints"] for s in expected_segments),
        "length_m_total": sum(s["trace_length_m"] for s in expected_segments),
        "fraction_by_length": (
            sum(s["trace_length_m"] for s in expected_segments if s["expected_status"] == "full")
            / sum(s["trace_length_m"] for s in expected_segments)
        ),
    }
    (out_dir / "expected.json").write_text(json.dumps(expected, indent=1) + "\n")


if __name__ == "__main__":
    main()

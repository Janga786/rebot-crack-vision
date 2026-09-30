# GEOM-08.5 — Lift ordered pixel polylines to base_link surface points with normals, gap policy and per-point covariance (crackvision.lift3d)

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-08.1",
    "GEOM-08.2",
    "GEOM-08.4"
  ],
  "requirements": [
    "REQ-GEOM-1",
    "REQ-GEOM-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/lift3d.py",
      "tests/test_lift3d.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0.5",
    "docs/INTERFACES.md#3.13",
    "docs/INTERFACES.md#6",
    "docs/INTERFACES.md#9",
    "docs/INTERFACES.md#10",
    "src/crackvision/geometry.py",
    "src/crackvision/kinematics.py",
    "src/crackvision/capture_record.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_lift3d.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Surface depth comes from geometry.sample_surface_depth_annulus. The crack-cavity depth under the mask is never used as the surface.",
      "The outward normal is oriented explicitly toward the camera, and fit_plane's sign is not relied on (a test uses a tilted plane where fit_plane's raw normal points away).",
      "Gap interpolation, splitting, trimming and minimum-segment behaviour match §10 exactly, with tests for each.",
      "The covariance matches the §10 formula: a test checks J·Σ·Jᵀ numerically against a finite-difference Jacobian at a centre and an off-axis pixel, and the calibration term's growth with range.",
      "Invalid points carry NaN/None geometry and a reason code (GEOM-02 codes plus normal_fit_failed). No NaN leaks into valid points."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4,
    "task_class": "impl-library",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-08.5",
  "parent": "GEOM-08",
  "title": "Lift ordered pixel polylines to base_link surface points with normals, gap policy and per-point covariance (crackvision.lift3d)",
  "outcome": "`crackvision.lift3d.lift_polyline` turns one paths.json dense polyline, plus depth, mask and a capture record, into per-point base_link surface points and outward normals. Each point gets an invalid reason, the §10 gap interpolation and segmentation, and the §10 first-order covariance. It never produces a point at the camera origin."
}
```

Create src/crackvision/lift3d.py. It is a pure library with no CLI and no file writes.

API:
- `@dataclass LiftParams`: annulus_inner_px=3, annulus_outer_px=8, min_fraction_valid=0.3, valid_range_m=geometry.DEFAULT_VALID_DEPTH_RANGE_M, pixel_sigma_px=1.0, max_gap_px=5, min_segment_points=3, min_normal_points=6.
- `@dataclass CalibSigmas`: cam_pos_m, cam_rot_rad (from kinematics.EndEffector).
- `@dataclass LiftedPolyline`, arrays of length N in dense order:
  - rows, cols (int), valid (bool), reason (object str), interpolated (bool);
  - depth_m, fraction_valid;
  - p_optical (N,3), p_base (N,3), n_base (N,3), normal_rms_m, sigma_normal_rad;
  - cov_base (N,3,3);
  - segments: list[(start, end)] inclusive; dropped_segments: int.
- `lift_polyline(points_rc: Sequence[(row, col)], depth_u16, mask_bool, intrinsics, depth_scale, T_base_link_optical, calib: CalibSigmas, params) -> LiftedPolyline`.

Algorithm per point:
(a) `sample_surface_depth_annulus(...)`: if invalid, keep its reason.
(b) p_opt = `deproject_pixels(intr, row, col, z)`.
(c) Normal: gather the annulus pixels (same inner/outer/in-bounds/non-mask geometry as the sampler; reimplement locally, do NOT edit geometry.py), `classify_depth` them, and deproject the valid ones.
    - If there are < min_normal_points → invalid, reason `normal_fit_failed`.
    - Else use `fit_plane`, then orient n so that n·(−p_opt) > 0.
    - normal_rms_m = its rms_residual; sigma_normal_rad = rms / (outer_px·z/fx).
(d) Σ_opt = J·diag(σpx², σpx², σz²)·Jᵀ with J = [[z/fx, 0, x/z], [0, z/fy, y/z], [0, 0, 1]] and σz = `depth_uncertainty_m(z, fx)`.
(e) p_base = T·p_opt, n_base = R·n. Σ_base = R·Σ_opt·Rᵀ + (cam_pos² + (‖p_opt‖·cam_rot)²)·I.

Then gaps (§10):
- Interpolate interior invalid runs of ≤ max_gap_px linearly in base_link (p and nlerp n), with the larger-trace neighbour's Σ; set `interpolated` = True, valid = True, and keep reason as `interpolated:<original reason>`.
- Longer runs split segments. Trim the ends. Drop segments with < min_segment_points non-interpolated valid points and count them.

Tests (synthetic, analytic; build fixtures in the test):
1. Fronto-parallel plane at z = 0.25 with a crack cavity 5 mm deeper under the mask. Lifted z = 0.25 (not 0.255) and n_opt = (0, 0, −1).
2. A plane tilted 30°: the normal is within 0.5° of truth and outward, and the test asserts fit_plane's raw normal would point away.
3. With identity T and a known T: p_base matches T·p_opt.
4. Holes: a 3-px zero-depth run is interpolated; an 8-px run splits into 2 segments; invalid ends are trimmed; a 2-point segment is dropped.
5. Covariance: compare against the finite-difference Jacobian of deproject (model 'none'); check the calibration term.
6. Every invalid point has NaN p_base and a non-empty reason. No valid point is within 1 cm of the camera origin.

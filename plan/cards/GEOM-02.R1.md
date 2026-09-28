# GEOM-02.R1 — Repair GEOM-02: Inverse Brown Conrady undistortion omits librealsense's xq=x/icdist ta

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-GEOM-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/geometry.py",
      "tests/test_geometry.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0.5",
    "docs/INTERFACES.md#3.10",
    "docs/adr/012-frames-and-conventions.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "pytest",
        "cmd": "./env.sh pytest tests/test_geometry.py -q"
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q",
        "timeout_s": 1200
      }
    ],
    "criteria": [
      "deproject_pixels, for model inverse_brown_conrady, computes the tangential terms with xq=x/icdist, yq=y/icdist as librealsense does. A pyrealsense2 parity test using coefficients of magnitude ≥0.01 (e.g. [-0.0547,0.0576,0.00036,0.00066,-0.0187] and one set with |p1|,|p2|≥0.004) passes with max error <1e-6 m for both inverse_brown_conrady and brown_conrady, and the misleading float32 comment in the test fixture is removed.",
      "The tilted/noisy and flat plane tests assert the recovered offset (e.g. |normal·centroid − z0·n_true[2]| < a stated tolerance such as 1 mm) as well as the normal angle, with the tolerances stated. The tilted test fails if depths are scaled by 2.",
      "All acceptance criteria of GEOM-02 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5
  },
  "track": "calibration",
  "priority": 99,
  "repairs": "GEOM-02",
  "findings": [
    "d580c48c5beaab56",
    "558c26cd5cd2c0df"
  ],
  "id": "GEOM-02.R1",
  "title": "Repair GEOM-02: Inverse Brown Conrady undistortion omits librealsense's xq=x/icdist ta",
  "parent": "L-GEOM",
  "outcome": "Resolve the review findings on GEOM-02 while every acceptance criterion of GEOM-02 still holds."
}
```

Repair work generated deterministically from review attempt `00024-GEOM-02-review` of `GEOM-02`.
Original card: `plan/cards/GEOM-02.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] Inverse Brown Conrady undistortion omits librealsense's xq=x/icdist tangential scaling; parity fails at realistic coefficient magnitudes
- Evidence: src/crackvision/geometry.py: `_UNDISTORT_MODELS = frozenset({"inverse_brown_conrady", "brown_conrady"})`. Both models use the same loop: `dx = 2 * c[2] * x * y + c[3] * (r2 + 2 * x * x)`. librealsense's inverse variant computes the tangential terms on xq = x/icdist, yq = y/icdist. Probe /tmp/probe_geom.py against pyrealsense2 2.58.4, max error over 300 pixels: inverse_brown_conrady with coeffs [-0.0547,0.0576,0.00036,0.00066,-0.0187] gives impl=2.15e-05 m, f32 port with the xq scaling 1.83e-08 m; with coeffs [0.05,0.03,0.005,-0.004,0.01] impl=3.68e-04 m, correct port 2.16e-08 m. The implementer blamed 'reference SDK float32 accumulation', but a float32 port of the correct formula matches the SDK to about 2e-8 m. The test fixture's coefficients ([0.001,-0.0005,0.0001,-0.00005,0.00002]) were chosen small enough to hide the difference.
- Consequence: For the model the module docstring says the D405 colour stream reports, points come out wrong by tens to hundreds of micrometres whenever the device's distortion coefficients are around 1e-2, which is typical of Brown–Conrady calibrations. Every downstream 3D crack-path point (GEOM-03+) inherits this, and it breaks the card's <1e-6 m acceptance criterion.
- Affected: src/crackvision/geometry.py, tests/test_geometry.py
- Acceptance condition: deproject_pixels, for model inverse_brown_conrady, computes the tangential terms with xq=x/icdist, yq=y/icdist as librealsense does. A pyrealsense2 parity test using coefficients of magnitude ≥0.01 (e.g. [-0.0547,0.0576,0.00036,0.00066,-0.0187] and one set with |p1|,|p2|≥0.004) passes with max error <1e-6 m for both inverse_brown_conrady and brown_conrady, and the misleading float32 comment in the test fixture is removed.

## Finding 2 [major] Plane-recovery tests never check the plane offset, so depth-scale/distance errors pass
- Evidence: tests/test_geometry.py test_fit_plane_recovers_tilted_noisy_plane asserts only `angle_deg < 2.0` and `fit.rms_residual_m < 0.003`. The comment 'normal . centroid ~ z0 * normal[2]' has no matching assert. test_fit_plane_flat_surface_low_residual also checks only the normal and the residual. Probe: scaling the tilted-plane points ×2 gives angle 0.047°, rms 0.0016 m and offset 0.492 m (true 0.246 m), which still satisfies both assertions.
- Consequence: The criterion 'recover plane parameters within stated tolerances' is only half tested. A depth-scale or z bug (RISKS R-09 class) would not be caught by the synthetic plane tests.
- Affected: tests/test_geometry.py
- Acceptance condition: The tilted/noisy and flat plane tests assert the recovered offset (e.g. |normal·centroid − z0·n_true[2]| < a stated tolerance such as 1 mm) as well as the normal angle, with the tolerances stated. The tilted test fails if depths are scaled by 2.

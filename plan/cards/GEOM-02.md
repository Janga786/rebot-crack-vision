# GEOM-02 — Depth projection library with invalid-depth policy

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-01",
    "TC-013"
  ],
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
      "Vectorised deprojection supporting the D405 colour distortion model; matches pyrealsense2.rs2_deproject_pixel_to_point to <1e-6 m on sampled pixels",
      "Crack-cavity policy (RISKS R-08): surface depth sampled from an annulus around each path pixel excluding mask pixels; robust median; reports the fraction of valid samples",
      "Invalid depth (0, out of band, NaN) never produces a point silently: flagged with a reason",
      "Synthetic plane tests (tilted, noisy) recover plane parameters within stated tolerances"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5
  },
  "track": "calibration",
  "priority": 55,
  "id": "GEOM-02",
  "title": "Depth projection library with invalid-depth policy",
  "parent": "L-GEOM",
  "outcome": "Pixels + aligned depth + intrinsics → 3D points in the colour optical frame, with an explicit invalid-depth policy."
}
```



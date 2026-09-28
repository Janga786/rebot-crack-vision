# GEOM-03 — Calibration method + camera mounting decision

```json card
{
  "kind": "decision",
  "depends_on": [
    "GEOM-01",
    "MOT-01"
  ],
  "requirements": [
    "REQ-GEOM-2",
    "REQ-GEOM-3"
  ],
  "scope": {
    "write": [
      "docs/adr/013-calibration-method.md",
      "docs/calibration/PLAN.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "docs",
        "cmd": "test -f docs/adr/013-calibration-method.md && test -f docs/calibration/PLAN.md"
      }
    ],
    "criteria": [
      "Eye-in-hand vs eye-to-hand justified; ChArUco spec (dictionary, squares, size, print scale check); ≥15 poses with rotation diversity",
      "Algorithms: OpenCV calibrateHandEye (≥2 methods cross-checked) + robot-world variant if eye-to-hand; residual thresholds (mm/deg) and how uncertainty propagates",
      "Camera mounting confirmed by the operator (return blocked/operator_decision if not documented anywhere)"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 4,
    "context": 3,
    "consequence": 4
  },
  "track": "calibration",
  "priority": 45,
  "id": "GEOM-03",
  "title": "Calibration method + camera mounting decision",
  "parent": "L-GEOM",
  "outcome": "A written, operator-confirmed calibration plan: mounting, target, algorithm, pose count, residual thresholds."
}
```



# GEOM-08 — Pixel paths → robot-frame 3D paths with uncertainty

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-02",
    "GEOM-03",
    "PERC-03"
  ],
  "requirements": [
    "REQ-GEOM-1",
    "REQ-GEOM-2"
  ],
  "spec_state": "draft",
  "refine_after": [
    "GEOM-03",
    "PERC-03"
  ],
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 5
  },
  "track": "calibration",
  "id": "GEOM-08",
  "title": "Pixel paths → robot-frame 3D paths with uncertainty",
  "parent": "L-GEOM",
  "outcome": "Ordered 2D paths + depth + calibration → 3D tool waypoints (position + orientation) with per-point uncertainty."
}
```

Draft: needs the path contract and calibration plan.

# MOT-06 — Cartesian path planning along crack paths

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-03",
    "MOT-04",
    "GEOM-08"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "spec_state": "draft",
  "refine_after": [
    "MOT-02",
    "GEOM-08"
  ],
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4
  },
  "track": "simulation",
  "id": "MOT-06",
  "title": "Cartesian path planning along crack paths",
  "parent": "L-MOTION",
  "outcome": "Cartesian path planning along crack paths"
}
```

Draft: approach → inspection → retract, computeCartesianPath, fallbacks, coverage report.

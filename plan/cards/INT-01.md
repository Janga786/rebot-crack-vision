# INT-01 — End-to-end simulation pipeline test

```json card
{
  "kind": "integration",
  "depends_on": [
    "TC-014",
    "PERC-04",
    "GEOM-08",
    "MOT-07"
  ],
  "requirements": [
    "REQ-INT-1"
  ],
  "spec_state": "draft",
  "refine_after": [
    "GEOM-08",
    "MOT-07"
  ],
  "track": "simulation",
  "id": "INT-01",
  "title": "End-to-end simulation pipeline test",
  "parent": "L-INT",
  "outcome": "Synthetic RGB-D scene → paths → 3D → MoveIt plan (mock) → limits, one command."
}
```

Draft.

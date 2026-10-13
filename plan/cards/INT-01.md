# INT-01 — End-to-end simulation pipeline test

```json card
{
  "kind": "branch",
  "depends_on": [
    "TC-014",
    "PERC-04",
    "GEOM-08",
    "MOT-07"
  ],
  "requirements": [
    "REQ-INT-1"
  ],
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

## Decomposed 2026-10-13T20:23:12Z by L-INT (claude-sonnet-5, high)
Children: INT-01.1, INT-01.2, INT-01.3

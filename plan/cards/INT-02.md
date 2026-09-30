# INT-02 — Recorded-data integration test

```json card
{
  "kind": "integration",
  "depends_on": [
    "INT-01",
    "CAM-04"
  ],
  "requirements": [
    "REQ-INT-1",
    "REQ-CAM-3"
  ],
  "spec_state": "draft",
  "refine_after": [
    "INT-01"
  ],
  "track": "simulation",
  "id": "INT-02",
  "title": "Recorded-data integration test",
  "parent": "L-INT",
  "outcome": "Replay a real D405 recording through the full pipeline to a mock execution."
}
```

Draft.

## ADR-014 guidance (technical-lead recovery, 2026-09-30)
- Recorded-data replay needs the joint state per capture (INTERFACES §8.4); recordings without it can only be used
  for 2D perception checks.

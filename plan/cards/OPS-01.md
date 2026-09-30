# OPS-01 — Operator CLI workflow (doctor → capture → segment → paths → plan → preview → execute)

```json card
{
  "kind": "impl",
  "depends_on": [
    "PERC-04",
    "MOT-08"
  ],
  "requirements": [
    "REQ-OPS-1"
  ],
  "spec_state": "draft",
  "refine_after": [
    "PERC-04",
    "MOT-06"
  ],
  "id": "OPS-01",
  "title": "Operator CLI workflow (doctor → capture → segment → paths → plan → preview → execute)",
  "parent": "L-OPS",
  "outcome": "One operator-facing CLI with resumable stages; execute delegates to the commissioning gate."
}
```

Draft.

## ADR-014 guidance (technical-lead recovery, 2026-09-30)
- capture = move to a view pose (camera_link target), record RGB-D + joint state; plan = GEOM-08 3D path -> MOT-06
  view/approach/trace/retract; execute only through the MOT-05 gate (refuses nominal end-effector geometry).

# OPS-01 — Operator CLI workflow (doctor → capture → segment → paths → plan → preview → execute)

```json card
{
  "kind": "branch",
  "depends_on": [
    "PERC-04",
    "MOT-08"
  ],
  "requirements": [
    "REQ-OPS-1"
  ],
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

## Decomposed 2026-10-14T08:29:28Z by L-OPS (claude-sonnet-5, high)
Children: OPS-01.1, OPS-01.2, OPS-01.3, OPS-01.4, OPS-01.5, OPS-01.6, OPS-01.7, OPS-01.8

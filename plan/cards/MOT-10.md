# MOT-10 — Measure the workcell geometry (operator)

```json card
{
  "kind": "branch",
  "depends_on": [
    "MOT-03"
  ],
  "requirements": [
    "REQ-MOT-1"
  ],
  "refine_after": [
    "MOT-03"
  ],
  "hardware": [
    "workcell_measured"
  ],
  "track": "physical",
  "id": "MOT-10",
  "title": "Measure the workcell geometry (operator)",
  "parent": "L-MOTION",
  "outcome": "Measured table, specimen and camera-mount poses replace nominal scene values.",
  "operator_procedure": true
}
```

Draft.

## ADR-014 guidance (technical-lead recovery, 2026-09-30)
- Measure the table (top face should be the base_link z=0 plane), the specimen pose/height and the workcell
  clearances; the camera pose is measured by hand-eye calibration (GEOM-05), not by tape. Replace the nominal
  specimen/table entries in config/scene/scene.yaml with measured values and re-run MOT-04.5's checks.

## Decomposed 2026-10-09T06:09:26Z by L-MOTION (claude-opus-5-5, medium)
Children: MOT-10.1, MOT-10.2, MOT-10.3, MOT-10.4, MOT-10.5

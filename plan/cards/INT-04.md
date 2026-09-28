# INT-04 — Physical no-contact traversal at safe standoff (operator)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "MOT-09",
    "GEOM-05",
    "GEOM-07",
    "INT-02",
    "INT-03",
    "MOT-10"
  ],
  "requirements": [
    "REQ-INT-2"
  ],
  "spec_state": "draft",
  "refine_after": [
    "INT-03"
  ],
  "hardware": [
    "arm_powered_estop_verified",
    "workspace_clear",
    "d405_mounted"
  ],
  "motion": true,
  "track": "physical",
  "id": "INT-04",
  "title": "Physical no-contact traversal at safe standoff (operator)",
  "parent": "L-INT",
  "outcome": "Measured tracking vs plan at ≤10% speed."
}
```

Draft.

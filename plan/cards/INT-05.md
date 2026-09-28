# INT-05 — Controlled end-to-end physical test on a real crack (operator)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "INT-04",
    "PERC-06",
    "OPS-01"
  ],
  "requirements": [
    "REQ-INT-2",
    "REQ-OPS-1"
  ],
  "spec_state": "draft",
  "refine_after": [
    "INT-04"
  ],
  "hardware": [
    "arm_powered_estop_verified",
    "workspace_clear",
    "crack_specimens_available"
  ],
  "motion": true,
  "track": "physical",
  "id": "INT-05",
  "title": "Controlled end-to-end physical test on a real crack (operator)",
  "parent": "L-INT",
  "outcome": "Capture → plan → preview → operator approval → execution with measured path/boresight accuracy."
}
```

Draft.

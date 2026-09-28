# PERC-05 — Collect the real D405 crack dataset (operator)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "CAM-04",
    "TC-015"
  ],
  "requirements": [
    "REQ-PERC-3"
  ],
  "spec_state": "draft",
  "refine_after": [
    "CAM-04"
  ],
  "hardware": [
    "d405_connected",
    "crack_specimens_available"
  ],
  "track": "camera",
  "id": "PERC-05",
  "title": "Collect the real D405 crack dataset (operator)",
  "parent": "L-PERC",
  "outcome": "50–100 real images per docs/D405_TEST_PLAN.md with manifest and checksums.",
  "operator_procedure": true
}
```

Draft: operator capture protocol per D405_TEST_PLAN §6; tooling from TC-013/TC-015/CAM-02.

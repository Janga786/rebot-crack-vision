# GEOM-07 — Execute TCP calibration + physical boresight verification (operator)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "GEOM-06",
    "MOT-09"
  ],
  "requirements": [
    "REQ-GEOM-3"
  ],
  "spec_state": "draft",
  "refine_after": [
    "GEOM-06"
  ],
  "hardware": [
    "tcp_tip_fixture_ready",
    "arm_commissioned_low_speed"
  ],
  "motion": true,
  "track": "calibration",
  "id": "GEOM-07",
  "title": "Execute TCP calibration + physical boresight verification (operator)",
  "parent": "L-GEOM",
  "outcome": "Measured TCP and boresight error."
}
```

Draft.

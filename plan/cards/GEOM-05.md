# GEOM-05 — Execute hand-eye calibration on hardware (operator)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "GEOM-04",
    "CAM-01",
    "MOT-09"
  ],
  "requirements": [
    "REQ-GEOM-2"
  ],
  "spec_state": "draft",
  "refine_after": [
    "GEOM-04"
  ],
  "hardware": [
    "d405_mounted",
    "charuco_target_ready",
    "arm_commissioned_low_speed"
  ],
  "motion": true,
  "track": "calibration",
  "id": "GEOM-05",
  "title": "Execute hand-eye calibration on hardware (operator)",
  "parent": "L-GEOM",
  "outcome": "Measured camera↔robot transform with residuals."
}
```

Draft: operator procedure via the commissioning tool; evidence = captures + solver report.

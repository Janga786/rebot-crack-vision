# GEOM-05 — Execute hand-eye calibration on hardware (operator)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "GEOM-04",
    "CAM-01",
    "MOT-09",
    "GEOM-10"
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

## ADR-014 guidance (technical-lead recovery, 2026-09-30)
- Before capturing: the operator confirms the mount side (camera on top of the gripper at the all-zero joint pose,
  gripper_link -Z) and records it as evidence; a mismatch means end_effector.yaml's prior must be flipped first.
- Result: end_effector.yaml wrist_camera (and its collision proxies if the mount geometry differs) become
  value_status measured; re-run MOT-04.5's placement/view checks afterwards.

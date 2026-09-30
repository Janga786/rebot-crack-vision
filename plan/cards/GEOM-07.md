# GEOM-07 — Execute TCP calibration + physical boresight verification (operator)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "GEOM-06",
    "MOT-09",
    "GEOM-11"
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

## ADR-014 guidance (technical-lead recovery, 2026-09-30)
- Calibrate the tool that will face the crack (closed gripper tip, or a held probe/pen) with the GEOM-06 solver as
  corrected by GEOM-11; the prior is end_effector.yaml tool (nominal (0,0,0) = closed-finger tip), not gripper_tcp.
- Write the measured tool block (value_status measured, residuals, evidence) into config/robot/end_effector.yaml.

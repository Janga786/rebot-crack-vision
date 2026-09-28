# MOT-09 — Arm bring-up commissioning (operator)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "MOT-05",
    "HOST-04"
  ],
  "requirements": [
    "REQ-INT-2"
  ],
  "spec_state": "draft",
  "refine_after": [
    "MOT-05"
  ],
  "hardware": [
    "arm_usb_connected",
    "arm_powered_estop_verified",
    "workspace_clear"
  ],
  "motion": true,
  "track": "physical",
  "id": "MOT-09",
  "title": "Arm bring-up commissioning (operator)",
  "parent": "L-MOTION",
  "outcome": "Arm powered, e-stop tested, joint readback and limits verified at low speed via mock → dry → real progression."
}
```

Draft: operator procedure; evidence = logs + measured limits.

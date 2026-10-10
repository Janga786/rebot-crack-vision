# GEOM-07 — Execute TCP calibration + physical boresight verification (operator)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "GEOM-06",
    "MOT-09",
    "GEOM-11",
    "MOT-11"
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

## 2026-10-10 operator note (overnight session): vendor gravity compensation is not ready yet
ADR-016 §3 has this card move the arm by hand under the vendor driver's
`/rebotarm/gravity_compensation/start` (`hardware_manager.py:456-478`: MIT kp 7, kd 0.8, pinocchio gravity
plus a clipped integral). That needs a running `reBotArmController`, so this card now depends on MOT-11.
- The driver cannot start against today's vendored SDK. There is an API mismatch, the runtime dependencies are
  missing for `/usr/bin/python3`, and the default `safe_park.yaml` is unsafe (see `docs/motion/VENDOR_DRIVER.md`).
- The SDK's gravity model is the gripper-less `reBot-DevArm_fixend.urdf` (SDK `kinematics/robot_model.py:22-30`).
  The gripper, the D405 and its mount are not in it. Expect the wrist and forearm to sag while hand-guiding. Try
  gravity compensation first at a low, safe pose with the arm supported by hand, before any calibration pose.
- Bring-up facts (IDs, signs, zero, gripper open = negative) are in `evidence/operator/MOT-09/`.

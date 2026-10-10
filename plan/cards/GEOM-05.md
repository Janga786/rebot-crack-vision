# GEOM-05 — Execute hand-eye calibration on hardware (operator)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "GEOM-04",
    "CAM-01",
    "MOT-09",
    "GEOM-10",
    "MOT-11"
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

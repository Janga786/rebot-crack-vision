# GEOM-01 — Frames, units and pixel conventions (ADR-012 + geometry contract)

**Accepted:** 2026-09-28 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`29298d1`](https://github.com/Janga786/rebot-crack-vision/commit/29298d149a3726f346b25fc98664f4a574105539)

## What was done

Added ADR-012, the single reference document defining every coordinate frame, transform naming convention, and pixel convention the crack-inspection pipeline will use going forward (robot frames, camera frames, row/col vs u/v pixel ordering, and how raw depth converts to metres). This closes a real risk: a swapped pixel convention or transform order would silently aim the robot arm at the wrong 3D location with no runtime error. The pixel-centre convention was verified directly against Intel's librealsense source code rather than assumed.

After the first audit, a repair round (GEOM-01.R1) fixed the reported issues: Corrected the camera frame documentation (ADR-012 and INTERFACES.md) so it no longer claims the camera_link-to-color-frame transform is translation-free or interchangeable with the raw device depth-to-colour extrinsic metadata — it now correctly states that transform must come from the camera driver's own published TF. Combined with the already-fixed tool0/TCP frame naming from the prior attempt, all reviewer-flagged inconsistencies in the robot/camera frame conventions document are now resolved.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/29298d149a3726f346b25fc98664f4a574105539/docs/INTERFACES.md) (+64 / −6)
- [docs/adr/012-frames-and-conventions.md](https://github.com/Janga786/rebot-crack-vision/blob/29298d149a3726f346b25fc98664f4a574105539/docs/adr/012-frames-and-conventions.md) (+189 / −21)

Commits: [`47b4315`](https://github.com/Janga786/rebot-crack-vision/commit/47b4315bd4c3dd124b32a0ffedc6217e6bafa1de), [`d0f1f1a`](https://github.com/Janga786/rebot-crack-vision/commit/d0f1f1a6c78fcba725c4ebf28258e2314acd684c), [`29298d1`](https://github.com/Janga786/rebot-crack-vision/commit/29298d149a3726f346b25fc98664f4a574105539)

## Why it was done

One authoritative definition of every frame, transform name, unit and pixel convention used downstream.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty
- Requirement **REQ-GEOM-2**: Camera-to-robot calibration with explicit transform conventions and residuals

## How it moves the project forward

- Geometry & calibration: **1/9** tasks accepted; whole project: **11/69**.
- REQ-GEOM-1: 1/4 contributing tasks done
- REQ-GEOM-2: 1/5 contributing tasks done
- Verification: automated checks run by the pipeline itself (adr ✔); independent audit accepted it (criteria: 9 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Both repair findings are fixed at 29298d1, and every GEOM-01 criterion still holds. ADR-012 and INTERFACES §6.1 now say that `tool0` is a new alias of `gripper_link`, published as an identity transform with `gripper_link` as parent. `TCP` is the existing MoveIt `gripper_tcp` frame, with the -0.0443 m offset along +X kept as prior evidence. The false 'consistent with' claim is withdrawn. I checked these frame facts against rebot_ws. The camera chain is now written as T_camera_link_camera_color_frame (per-device, real translation) · T_camera_color_frame_camera_color_optical_f
…[375 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task GEOM-01.R1): camera_link → camera_color_optical_frame wrongly described as rotation-only and not per-device; tool0/TCP are not mapped to the actual robot frames; the claim of consistency with the MoveIt config is false.

## What it unlocks next

- **Ready to start:** GEOM-06 — TCP (pivot) calibration solver + boresight procedure
- Closer: GEOM-02 — Depth projection library with invalid-depth policy (still needs TC-013)
- Closer: GEOM-03 — Calibration method + camera mounting decision (still needs MOT-01)
- Closer: MOT-04 — Reachability map + recommended specimen placement (still needs MOT-02, MOT-02)

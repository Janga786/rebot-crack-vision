# CAM-05.1 — Decision: scope of ROS 2 camera integration (ADR-015)

**Accepted:** 2026-10-09 · **Work package:** D405 capture (L-CAM) · **Track:** camera · **Verified commit:** [`68ec5e0`](https://github.com/Janga786/rebot-crack-vision/commit/68ec5e0c7054d3fcbc75f636f7fa1626c9ebe486)

## What was done

Wrote a short design decision (ADR-015) clarifying that the camera integration work does not need a full ROS camera driver running at all times; instead it needs only two small one-off ROS tools to grab a calibration transform and the arm's joint angles at the moment of each photo. This keeps the camera capture code simple and ROS-free while still meeting the robot's 3D-mapping requirements. All three required checks on the new document and the project log passed.

Files changed:
- [docs/COMPLETION_LOG.md](https://github.com/Janga786/rebot-crack-vision/blob/68ec5e0c7054d3fcbc75f636f7fa1626c9ebe486/docs/COMPLETION_LOG.md) (+40 / −0)
- [docs/adr/015-camera-ros-integration-scope.md](https://github.com/Janga786/rebot-crack-vision/blob/68ec5e0c7054d3fcbc75f636f7fa1626c9ebe486/docs/adr/015-camera-ros-integration-scope.md) (+91 / −0)

Commits: [`68ec5e0`](https://github.com/Janga786/rebot-crack-vision/commit/68ec5e0c7054d3fcbc75f636f7fa1626c9ebe486)

## Why it was done

ADR-015 settles, in writing, that CAM-05 does not stand up a continuously-running realsense2_camera driver; it defines the exact two narrow ROS touchpoints (static optical-frame TF capture, joint-state-at-capture bridge) that CAM-05.2 implements, and reconciles wording in ADR-012 §6.2 and INTERFACES §8.4 that could otherwise be read as contradictory.

- Requirement **REQ-CAM-2**: Aligned colour/depth capture with timestamps, intrinsics, depth scale, extrinsics and metadata

## How it moves the project forward

- D405 capture: **5/9** tasks accepted; whole project: **44/92**.
- REQ-CAM-2: 4/8 contributing tasks done
- Verification: automated checks run by the pipeline itself (exists ✔, content ✔, completion-log ✔); independent audit accepted it (criteria: 8 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “ADR-015 correctly settles CAM-05's scope: no continuously-running realsense2_camera driver, exactly two one-shot ROS touchpoints (capture_optical_tf, capture_joint_state) delivered by CAM-05.2, and a clear reconciliation of ADR-012 §6.2 vs INTERFACES §8.4's "device record" wording. Only the two in-scope files were touched, matching the template structure (Status: accepted, Supersedes: —), and all claimed output formats match GEOM-08.4's CLI exactly.”

## What it unlocks next

- **Ready to start:** CAM-05.2 — Static optical-frame TF capture + joint-state capture bridge (crackvision_camera ROS 2 package)

# GEOM-03 — Calibration method + camera mounting decision

**Accepted:** 2026-09-30 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`f712424`](https://github.com/Janga786/rebot-crack-vision/commit/f712424aacce2aaf7610ec3db252974a98a2defa)

## What was done

Finalized the written calibration plan for the wrist camera: ADR-013 now formally records the operator's decision that the D405 is wrist-mounted (eye-in-hand) on the gripper, with a fully specified ChArUco target, ≥15-pose capture procedure, and three-layer residual thresholds (target detection, cross-method agreement, held-out verification). The plan also flags the one remaining physical check still needed before running the real calibration: confirming which face of the gripper the camera actually sits on. Both required files exist and pass the scheduler's file-presence check.

Files changed:
- [docs/adr/013-calibration-method.md](https://github.com/Janga786/rebot-crack-vision/blob/f712424aacce2aaf7610ec3db252974a98a2defa/docs/adr/013-calibration-method.md) (+281 / −49)
- [docs/calibration/PLAN.md](https://github.com/Janga786/rebot-crack-vision/blob/f712424aacce2aaf7610ec3db252974a98a2defa/docs/calibration/PLAN.md) (+179 / −45)

Commits: [`d25f822`](https://github.com/Janga786/rebot-crack-vision/commit/d25f822d9719244b82919a1377d9c1f931488677), [`f712424`](https://github.com/Janga786/rebot-crack-vision/commit/f712424aacce2aaf7610ec3db252974a98a2defa)

## Why it was done

A written, operator-confirmed calibration plan: mounting, target, algorithm, pose count, residual thresholds.

- Requirement **REQ-GEOM-2**: Camera-to-robot calibration with explicit transform conventions and residuals
- Requirement **REQ-GEOM-3**: TCP calibration and physical boresight verification

## How it moves the project forward

- Geometry & calibration: **5/11** tasks accepted; whole project: **38/77**.
- REQ-GEOM-2: 2/6 contributing tasks done
- REQ-GEOM-3: 3/5 contributing tasks done
- Verification: automated checks run by the pipeline itself (docs ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “ADR-013 and PLAN.md now record the eye-in-hand mounting as an operator decision. The claim checks out: ADR-014, which is accepted and dated 2026-09-30, states "Operator facts (authoritative, 2026-09-30): The Intel RealSense D405 is eye-in-hand, on Seeed's stock D405_305_Mount.step". config/robot/end_effector.yaml has a matching wrist_camera block. Both documents give the ChArUco spec (DICT_5X5_250, 7×5 squares with 6×4 interior corners, 30/22 mm, and a mandatory caliper print-scale check with a 0.5% limit). They require at least 15 poses (20 recommended), with explicit tilt
…[711 chars clipped]”

## What it unlocks next

- **Can now be planned in detail:** CAM-05 — ROS 2 camera integration (only if calibration/workflow needs it)
- Closer: GEOM-04 — Hand-eye calibration software (synthetic-verified) (still needs GEOM-10)
- Closer: GEOM-08 — Pixel paths → robot-frame 3D paths with uncertainty (still needs GEOM-10)

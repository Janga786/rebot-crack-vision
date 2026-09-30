# GEOM-11 — TCP/pivot calibration uses the tool_tip prior (ADR-014), not the grasp-centre gripper_tcp

**Accepted:** 2026-09-30 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`e9f0ed0`](https://github.com/Janga786/rebot-crack-vision/commit/e9f0ed0d4403e43641c0535ce045f3658f181040)

## What was done

Fixed a calibration bug where the TCP/boresight check was comparing measured tool-tip calibrations against the wrong reference point (the gripper's internal grasp-centre, 44mm off from the actual tool tip), which would have hidden a real 44mm calibration error as a false match. The check now compares against the correct physical tool-tip reference from the robot's config file and reports a clean distance-based number instead of a direction angle that could break down near zero. All 11 calibration tests pass, including a new regression test proving the old bug would have shown a 44mm disagreement on an otherwise-correct calibration.

Files changed:
- [docs/calibration/TCP_BORESIGHT_PROCEDURE.md](https://github.com/Janga786/rebot-crack-vision/blob/e9f0ed0d4403e43641c0535ce045f3658f181040/docs/calibration/TCP_BORESIGHT_PROCEDURE.md) (+56 / −41)
- [src/crackvision/calibration/tcp.py](https://github.com/Janga786/rebot-crack-vision/blob/e9f0ed0d4403e43641c0535ce045f3658f181040/src/crackvision/calibration/tcp.py) (+90 / −36)
- [tests/test_tcp.py](https://github.com/Janga786/rebot-crack-vision/blob/e9f0ed0d4403e43641c0535ce045f3658f181040/tests/test_tcp.py) (+51 / −16)

Commits: [`e9f0ed0`](https://github.com/Janga786/rebot-crack-vision/commit/e9f0ed0d4403e43641c0535ce045f3658f181040)

## Why it was done

The pivot-calibration comparison and the TCP/boresight procedure refer to the physical tool tip (config/robot/end_effector.yaml tool block, nominal (0,0,0) = closed-finger tip on gripper_link) instead of gripper_tcp's -0.0443 m grasp-centre offset, so a correct GEOM-07 calibration is not flagged as a 44 mm disagreement.

- Requirement **REQ-GEOM-3**: TCP calibration and physical boresight verification

## How it moves the project forward

- Geometry & calibration: **4/11** tasks accepted; whole project: **37/77**.
- REQ-GEOM-3: 2/5 contributing tasks done
- Verification: automated checks run by the pipeline itself (pytest ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “GEOM-11 correctly replaces the hard-coded gripper_tcp grasp-centre prior (-0.0443m) with a tool_tip prior read from config/robot/end_effector.yaml (nominal (0,0,0)), converts check_boresight into a point comparison with position_delta_norm_m as the primary always-defined metric, and suppresses the direction angle when either vector is under 5mm to avoid NaN/false-opposite-direction flags. GRASP_CENTRE_OFFSET_M is retained only for the regression test/doc reference, never as a default. The procedure doc correctly names tool_tip as the calibrated frame, the T_gripper_link_too
…[131 chars clipped]”

## What it unlocks next

- Closer: GEOM-07 — Execute TCP calibration + physical boresight verification (operator) (still needs GEOM-06, MOT-09, GEOM-06)

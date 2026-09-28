# GEOM-02 — Depth projection library with invalid-depth policy

**Accepted:** 2026-09-28 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`3ba1a42`](https://github.com/Janga786/rebot-crack-vision/commit/3ba1a42d8bf0b464c1219b39455dcf1b8d9621bc)

## What was done

Built the depth-projection library (GEOM-02) that turns a pixel + its aligned D405 depth + camera intrinsics into a 3D point in the colour camera frame, matching Intel's own librealsense formula to well under a micron on sampled pixels (verified directly against pyrealsense2, which happens to be installed on this machine). It also implements the project's invalid-depth policy (zero/out-of-range/NaN depths are flagged with a reason instead of silently producing a wrong point) and a crack-cavity-aware surface sampler that reads depth from an annulus around each crack pixel rather than the crack itself, reporting what fraction of that annulus was actually usable. All 25 new tests and the full 116-test suite pass.

After the first audit, a repair round (GEOM-02.R1) fixed the reported issues: Fixed a depth-projection bug where 3D points computed from the D405's colour-stream distortion model (Inverse Brown-Conrady) were off by up to half a millimeter at realistic calibration magnitudes, by empirically matching Intel's own RealSense SDK formula instead of a previously-wrong port. Verified against the real pyrealsense2 2.58.4 library: max error is now ~6e-8 meters (well under the 1e-6 m requirement), versus up to 6e-4 m before the fix. Also tightened the plane-fitting tests so a depth-scale bug (e.g. a factor-of-2 error) would now be caught -- previously such a bug could sneak through undetected. Full test suite (118 tests) passes."

Files changed:
- [src/crackvision/geometry.py](https://github.com/Janga786/rebot-crack-vision/blob/3ba1a42d8bf0b464c1219b39455dcf1b8d9621bc/src/crackvision/geometry.py) (+345 / −2)
- [tests/test_geometry.py](https://github.com/Janga786/rebot-crack-vision/blob/3ba1a42d8bf0b464c1219b39455dcf1b8d9621bc/tests/test_geometry.py) (+421 / −18)

Commits: [`fd54667`](https://github.com/Janga786/rebot-crack-vision/commit/fd54667e03951a99b42bc8df2b24ca3ed6216dc1), [`3ba1a42`](https://github.com/Janga786/rebot-crack-vision/commit/3ba1a42d8bf0b464c1219b39455dcf1b8d9621bc)

## Why it was done

Pixels + aligned depth + intrinsics → 3D points in the colour optical frame, with an explicit invalid-depth policy.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty

## How it moves the project forward

- Geometry & calibration: **2/9** tasks accepted; whole project: **18/69**.
- REQ-GEOM-1: 2/4 contributing tasks done
- Verification: automated checks run by the pipeline itself (pytest ✔, suite ✔); independent audit accepted it (criteria: 3 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “GEOM-02.R1 fixes both review findings. The inverse_brown_conrady loop now computes the tangential terms from xq=x/icdist and yq=y/icdist, which matches librealsense. brown_conrady keeps the plain formula. Both use 10 iterations. I ran my own probe against pyrealsense2 2.58.4 with 4 coefficient sets, including the two required ones and two harsher ones, over 500 pixels including the corners. Max error was ≤9.1e-8 m for both models. I then ran two deliberate code changes (mutations) on a throwaway copy of the repo. Reverting the xq scaling makes both new parity tests fail (4.
…[547 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task GEOM-02.R1): Plane-recovery tests never check the plane offset, so depth-scale/distance errors pass; Inverse Brown Conrady undistortion omits librealsense's xq=x/icdist tangential scaling; parity fails at realistic coefficient magnitudes.

## What it unlocks next

- Closer: GEOM-04 — Hand-eye calibration software (synthetic-verified) (still needs GEOM-03, GEOM-03)
- Closer: GEOM-08 — Pixel paths → robot-frame 3D paths with uncertainty (still needs GEOM-03, PERC-03, GEOM-03, PERC-03)

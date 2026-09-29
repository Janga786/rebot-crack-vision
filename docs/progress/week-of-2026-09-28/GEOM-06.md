# GEOM-06 — TCP (pivot) calibration solver + boresight procedure

**Accepted:** 2026-09-29 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`1caa854`](https://github.com/Janga786/rebot-crack-vision/commit/1caa854b2753a485b70442c7b550fd23192496c0)

## What was done

Added the software and procedure for calibrating the robot arm's tool-centre point (the exact 3D offset of the physical tip relative to the wrist). The solver recovers a known test offset to within 0.1 mm even with simulated measurement noise, verified by 8 passing automated tests, and the accompanying procedure document specifies the fixture, arm poses, and pass/fail thresholds an operator will use to run this calibration on the real arm next.

Files changed:
- [docs/calibration/TCP_BORESIGHT_PROCEDURE.md](https://github.com/Janga786/rebot-crack-vision/blob/1caa854b2753a485b70442c7b550fd23192496c0/docs/calibration/TCP_BORESIGHT_PROCEDURE.md) (+119 / −0)
- [src/crackvision/calibration/__init__.py](https://github.com/Janga786/rebot-crack-vision/blob/1caa854b2753a485b70442c7b550fd23192496c0/src/crackvision/calibration/__init__.py) (+1 / −0)
- [src/crackvision/calibration/tcp.py](https://github.com/Janga786/rebot-crack-vision/blob/1caa854b2753a485b70442c7b550fd23192496c0/src/crackvision/calibration/tcp.py) (+154 / −0)
- [tests/test_tcp.py](https://github.com/Janga786/rebot-crack-vision/blob/1caa854b2753a485b70442c7b550fd23192496c0/tests/test_tcp.py) (+151 / −0)

Commits: [`1caa854`](https://github.com/Janga786/rebot-crack-vision/commit/1caa854b2753a485b70442c7b550fd23192496c0)

## Why it was done

Software to calibrate the tool-centre point and a written procedure for physical boresight verification.

- Requirement **REQ-GEOM-3**: TCP calibration and physical boresight verification

## How it moves the project forward

- Geometry & calibration: **3/9** tasks accepted; whole project: **33/74**.
- REQ-GEOM-3: 1/3 contributing tasks done
- Verification: automated checks run by the pipeline itself (pytest ✔); independent audit accepted it (criteria: 3 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “GEOM-06 adds a least-squares pivot-calibration solver that reports residuals, 8 passing tests and a procedure document that follows ADR-012. The diff touches only the four files the card allows, and the working tree is clean. The noise-recovery test is not a lucky seed: over 200 seeds the worst TCP error was 0.034 mm, well inside the 0.1 mm limit. The procedure covers the fixture, the poses, the three-layer pass/fail thresholds, a near/far parallax check for boresight direction, and a table of what the operator measures versus what the solver computes. The prior offset (-0.
…[92 chars clipped]”

## What it unlocks next

- Closer: GEOM-07 — Execute TCP calibration + physical boresight verification (operator) (still needs MOT-09)

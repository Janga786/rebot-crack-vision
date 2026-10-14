# INT-03.2 — Invalid-depth failure-injection test (perception+geometry chain)

**Accepted:** 2026-10-14 · **Work package:** Integration & commissioning (L-INT) · **Track:** simulation · **Verified commit:** [`aa2fc91`](https://github.com/Janga786/rebot-crack-vision/commit/aa2fc911f9c06864b9b21cacc4e97d5a6c115c7f)

## What was done

Added an automated test that corrupts a synthetic crack scan's depth image (the real D405's documented all-zero "no signal" value) both completely and in a partial band, then runs it through the 3D path-lifting pipeline. Confirmed the pipeline never invents a fake 3D point for bad depth data -- it correctly flags every affected point as invalid with a reason, and correctly marks the whole scan as unsafe to execute. No code fixes were needed since the existing safety logic already handled this correctly; the test and a results writeup are now in place for future reference.

Files changed:
- [docs/integration/safety/INVALID_DEPTH.md](https://github.com/Janga786/rebot-crack-vision/blob/aa2fc911f9c06864b9b21cacc4e97d5a6c115c7f/docs/integration/safety/INVALID_DEPTH.md) (+124 / −0)
- [scripts/perc/test_safety_invalid_depth.sh](https://github.com/Janga786/rebot-crack-vision/blob/aa2fc911f9c06864b9b21cacc4e97d5a6c115c7f/scripts/perc/test_safety_invalid_depth.sh) (+288 / −0)

Commits: [`aa2fc91`](https://github.com/Janga786/rebot-crack-vision/commit/aa2fc911f9c06864b9b21cacc4e97d5a6c115c7f)

## Why it was done

A new headless script corrupts a synthetic capture record's depth image (device-zero sentinel, full and partial) and proves crackvision.path3d never fabricates a 3D point from invalid depth and correctly marks the result ineligible, documented in docs/integration/safety/INVALID_DEPTH.md.

- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Integration & commissioning: **7/14** tasks accepted; whole project: **133/170**.
- REQ-INT-2: 18/26 contributing tasks done
- REQ-OPS-1: 20/33 contributing tasks done
- Verification: automated checks run by the pipeline itself (invalid-depth ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The script builds a real synthetic crack case through skeleton/paths, then corrupts the §9 capture's depth PNG with the device's zero no-return sentinel (100% and a 24-row partial band), runs crackvision.path3d on each, and verifies no fabricated geometry. Reproduced the test independently with identical results: 100%-invalid run has all 105 points valid=false with populated reasons and null geometry, no_valid_points in ineligible_reasons, execution_eligible=false, exit 0; partial run splits into 2 segments per the already-documented §10.3 policy with 0 interpolated and no 
…[585 chars clipped]”

## What it unlocks next

- Closer: INT-03.5 — Assemble the safety case + unified failure-injection runner (still needs INT-03.3, INT-03.4)

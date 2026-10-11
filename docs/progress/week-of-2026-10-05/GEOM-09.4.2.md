# GEOM-09.4.2 — Write docs/geometry/UNCERTAINTY_BUDGET.md

**Accepted:** 2026-10-11 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`baeff69`](https://github.com/Janga786/rebot-crack-vision/commit/baeff6989a4ae0eedfc5a08c82fee5faf145b291)

## What was done

Added documentation for the new uncertainty-budget report tool, explaining in plain language what the per-point/per-waypoint uncertainty breakdown means, how to run it, and its honest limits (it only covers modelled error sources like pixel, depth, calibration and joint-encoder noise, not real hardware accuracy). Verified the doc's required keywords and claims against the actual implemented CLI code rather than the original design spec.

Files changed:
- [docs/geometry/UNCERTAINTY_BUDGET.md](https://github.com/Janga786/rebot-crack-vision/blob/baeff6989a4ae0eedfc5a08c82fee5faf145b291/docs/geometry/UNCERTAINTY_BUDGET.md) (+98 / −0)

Commits: [`baeff69`](https://github.com/Janga786/rebot-crack-vision/commit/baeff6989a4ae0eedfc5a08c82fee5faf145b291)

## Why it was done

docs/geometry/UNCERTAINTY_BUDGET.md documents the `crackvision.uncertainty_budget` report and CLI: what it means, how to run it, and its honest scope/limits, at the same honesty standard as docs/geometry/PATH3D_VERIFICATION.md.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty

## How it moves the project forward

- Geometry & calibration: **22/32** tasks accepted; whole project: **91/128**.
- REQ-GEOM-1: 15/17 contributing tasks done
- Verification: automated checks run by the pipeline itself (doc-content ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “docs/geometry/UNCERTAINTY_BUDGET.md accurately documents the uncertainty_budget CLI/report, matching flag names, config schema/defaults, output path, exit codes and caveats verified against uncertainty_budget.py.”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

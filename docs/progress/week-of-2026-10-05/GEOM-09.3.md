# GEOM-09.3 — Monte-Carlo cross-check of the pixel-to-tool-pose uncertainty chain (crackvision.uncertainty_mc)

**Accepted:** 2026-10-11 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`ae2898b`](https://github.com/Janga786/rebot-crack-vision/commit/ae2898b286575f7e23af02a698b0e58608619fcc)

## What was done

All sub-tasks of this work package were completed and audited together: GEOM-09.3.1.

Files changed:
- [src/crackvision/uncertainty_mc.py](https://github.com/Janga786/rebot-crack-vision/blob/ae2898b286575f7e23af02a698b0e58608619fcc/src/crackvision/uncertainty_mc.py) (+210 / −91)

Commits: [`77f895f`](https://github.com/Janga786/rebot-crack-vision/commit/77f895f94fe6c58ce711692bcc75566c3b5151c5)

## Why it was done

crackvision/uncertainty_mc.py resamples the pixel->base_link->tool_tip chain per §14's perturbation distributions and reports empirical covariance plus an agreement statistic against crackvision.uncertainty's analytic result.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty

## How it moves the project forward

- Geometry & calibration: **20/30** tasks accepted; whole project: **89/126**.
- REQ-GEOM-1: 13/15 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The GEOM-09.3.R1 repair (commit 77f895f, retained unchanged at HEAD ae2898b) fixes the previously-raised blocker: uncertainty_mc.py now genuinely reruns crackvision.lift3d.lift_polyline and crackvision.tool_waypoints.segment_waypoints per MC sample (building a synthetic perturbed one-pixel depth patch and feeding it through the real annulus/plane-fit/waypoint pipeline) instead of re-deriving a simplified geometric formula. All acceptance criteria are met and tests pass.”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance: uncertainty_mc.py reimplements deprojection/waypoint geometry instead of calling the real lift3d/tool_waypoints functions.

## What it unlocks next

- **Can now be planned in detail:** GEOM-09.4 — Uncertainty budget CLI + report (crackvision.uncertainty_budget)

# GEOM-09.4.1 — Implement crackvision.uncertainty_budget CLI + report assembly

**Accepted:** 2026-10-11 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`4adff15`](https://github.com/Janga786/rebot-crack-vision/commit/4adff15fa4e41de64ca840aef525d02062891d65)

## What was done

Built the uncertainty-budget CLI (`./env.sh python -m crackvision.uncertainty_budget`) that takes an already-generated 3D crack path file and produces a detailed error-budget report, breaking down the pixel, depth-sensor, hand-eye-calibration and robot-joint-encoder contributions to position uncertainty at every point and tool waypoint, and cross-checking that analytic math against an independent Monte-Carlo simulation. All 12 of the card's own tests pass, including one that deliberately forces a disagreement between the two methods and confirms the tool still writes its report file but correctly signals failure via its exit code.

Files changed:
- [src/crackvision/uncertainty_budget.py](https://github.com/Janga786/rebot-crack-vision/blob/4adff15fa4e41de64ca840aef525d02062891d65/src/crackvision/uncertainty_budget.py) (+632 / −0)
- [tests/test_uncertainty_budget_cli.py](https://github.com/Janga786/rebot-crack-vision/blob/4adff15fa4e41de64ca840aef525d02062891d65/tests/test_uncertainty_budget_cli.py) (+420 / −0)

Commits: [`4adff15`](https://github.com/Janga786/rebot-crack-vision/commit/4adff15fa4e41de64ca840aef525d02062891d65)

## Why it was done

`./env.sh python -m crackvision.uncertainty_budget` reads a §10 paths3d file (+ its capture record, end_effector.yaml, kinematic chain, optional limits/config files), computes the §14 analytic per-term decomposition and Monte-Carlo cross-check for every valid point and waypoint, writes `data/uncertainty/{case}_uncertainty_budget.json` validating against `crackvision.uncertainty_budget/1` exactly, and exits non-zero (while still writing the file) whenever any point/waypoint is MC/analytic-disagreement-flagged.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty

## How it moves the project forward

- Geometry & calibration: **21/32** tasks accepted; whole project: **90/128**.
- REQ-GEOM-1: 14/17 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔); independent audit accepted it (criteria: 9 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “uncertainty_budget.py correctly implements the §14 CLI/report per docs/INTERFACES.md, building on crackvision.uncertainty / uncertainty_mc exactly as specified (decompose_point+fk_covariance for measured points, measured_source_index reuse for interpolated/waypoints, decompose_waypoint with copied sigma_along_normal_base_m/within_budget, MC cross-check with new analytic total, strict §14.6 config loader, EXIT_RUNTIME on overall_pass=false while still writing the file). Full test suite (403 passed) including the card's own 12 tests pass with no regressions, and scope is limi
…[30 chars clipped]”

## What it unlocks next

- **Ready to start:** GEOM-09.4.2 — Write docs/geometry/UNCERTAINTY_BUDGET.md

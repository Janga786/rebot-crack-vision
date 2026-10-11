# GEOM-09.4 — Uncertainty budget CLI + report (crackvision.uncertainty_budget)

**Accepted:** 2026-10-11 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`cec524c`](https://github.com/Janga786/rebot-crack-vision/commit/cec524c3feb84fe19afd74a15a1b7820a53e28e6)

## What was done

All sub-tasks of this work package were completed and audited together: GEOM-09.4.1, GEOM-09.4.2.

## Why it was done

A CLI reads a paths3d file (+ capture record) and writes a `crackvision.uncertainty_budget/1` report combining the analytic breakdown and Monte-Carlo cross-check for every point/waypoint.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty

## How it moves the project forward

- Geometry & calibration: **23/32** tasks accepted; whole project: **92/128**.
- REQ-GEOM-1: 16/17 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “GEOM-09.4 (uncertainty_budget CLI + docs) matches §14/§0 contracts: schema fields, exit codes (0/1/2/3), always-write-on-disagreement behavior, and documented scope/limits all verified against docs/INTERFACES.md §14 and against the implementation. Full test suite passes (12/12).”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

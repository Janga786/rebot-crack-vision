# GEOM-09.3.1 — Implement crackvision.uncertainty_mc Monte-Carlo cross-check (pixel/depth/handeye/fk/tcp)

**Accepted:** 2026-10-11 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`7a18c89`](https://github.com/Janga786/rebot-crack-vision/commit/7a18c8969bcea72cbddb0a88fafcc4a9254e5087)

## What was done

Implemented crackvision.uncertainty_mc (mc_point_covariance, mc_waypoint_sigma) per docs/INTERFACES.md §14/ADR-018 §4: seeded Monte-Carlo cross-check that reruns geometry.deproject_pixels and kinematics.KinematicChain.fk per sample for pixel/depth/handeye/fk, with an isotropic tcp perturbation added only at the waypoint tool-tip level. tests/test_uncertainty_mc.py (N_SAMPLES=1500, documented) proves agreement in a near-linear regime and a flagged disagreement when hand-eye rotation sigma is driven large on a far point.

Files changed:
- [src/crackvision/uncertainty_mc.py](https://github.com/Janga786/rebot-crack-vision/blob/7a18c8969bcea72cbddb0a88fafcc4a9254e5087/src/crackvision/uncertainty_mc.py) (+220 / −0)
- [tests/test_uncertainty_mc.py](https://github.com/Janga786/rebot-crack-vision/blob/7a18c8969bcea72cbddb0a88fafcc4a9254e5087/tests/test_uncertainty_mc.py) (+220 / −0)

Commits: [`7a18c89`](https://github.com/Janga786/rebot-crack-vision/commit/7a18c8969bcea72cbddb0a88fafcc4a9254e5087)

## Why it was done

crackvision/uncertainty_mc.py draws seeded samples that perturb pixel, depth, hand-eye and FK inputs and rerun the real lift3d/kinematics code paths (plus a directly-added isotropic TCP perturbation at the waypoint level), accumulates empirical covariance, and reports a §14 agreement statistic against crackvision.uncertainty's analytic breakdown; tests/test_uncertainty_mc.py proves the check agrees in a near-linear regime and flags a deliberately non-linear one.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty

## How it moves the project forward

- Geometry & calibration: **19/30** tasks accepted; whole project: **88/126**.
- REQ-GEOM-1: 12/15 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔); independent audit accepted it (criteria: 9 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “uncertainty_mc.py correctly implements a real Monte-Carlo rerun (deproject_pixels + KinematicChain.fk) for pixel/depth/handeye/fk, with isotropic tcp perturbation applied only at the waypoint tool-tip level per §14.2. Tests demonstrate agreement in a near-linear regime, discriminating disagreement in a nonlinear regime, seed determinism, and tcp-only-affects-waypoint behavior. Scope, imports, purity, and reproducibility constraints are all satisfied; full test suite (391 passed) and the module's own tests (5 passed, repeated 3x) are stable.”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

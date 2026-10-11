# GEOM-09.2 — Analytic per-term uncertainty decomposition library (crackvision.uncertainty)

**Accepted:** 2026-10-11 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`c2c748c`](https://github.com/Janga786/rebot-crack-vision/commit/c2c748cdbe9038d88cb3c5dfdfc0b68ce3f92e96)

## What was done

All sub-tasks of this work package were completed and audited together: GEOM-09.2.1.

## Why it was done

crackvision/uncertainty.py computes, for a lifted 3D point or tool waypoint, the named per-term covariance contributions (pixel noise, depth noise, hand-eye calibration, TCP calibration) defined by §14, consistent with the existing §10.4 total.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty

## How it moves the project forward

- Geometry & calibration: **18/29** tasks accepted; whole project: **87/125**.
- REQ-GEOM-1: 11/14 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “crackvision/uncertainty.py correctly decomposes the §10.4 cov_base into pixel/depth/handeye terms by exactly replicating lift3d's formula (verified line-by-line against lift3d.py:272-288), adds a central finite-difference fk term per ADR-018 §3, and provides waypoint-level decomposition whose index-mapping logic (waypoint_source_indices) exactly mirrors tool_waypoints._resample_segment and segment_waypoints. All 9 tests pass, the module is pure (no I/O/CLI), and it does not modify lift3d/tool_waypoints/kinematics.”

## What it unlocks next

- **Can now be planned in detail:** GEOM-09.3 — Monte-Carlo cross-check of the pixel-to-tool-pose uncertainty chain (crackvision.uncertainty_mc)
- Closer: GEOM-09.4 — Uncertainty budget CLI + report (crackvision.uncertainty_budget) (still needs GEOM-09.3)

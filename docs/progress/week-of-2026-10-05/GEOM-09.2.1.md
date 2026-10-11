# GEOM-09.2.1 — Implement crackvision.uncertainty analytic per-term decomposition (pixel, depth, handeye, tcp, fk)

**Accepted:** 2026-10-11 · **Work package:** Geometry & calibration (L-GEOM) · **Track:** calibration · **Verified commit:** [`fb9d56b`](https://github.com/Janga786/rebot-crack-vision/commit/fb9d56b818d4c4b6eb2dfcbb3f89dfb52d957d01)

## What was done

Built and tested a new uncertainty-decomposition module that breaks a lifted 3D crack point's existing position uncertainty down into its pixel-noise, depth-noise, hand-eye-calibration, and new joint/FK-error components, and does the same for each robot tool waypoint. All 9 new unit tests pass, including one that runs the real path-lifting and waypoint-generation code on a synthetic crack with a bridged data gap and confirms the per-term breakdown adds up exactly to the already-accepted total uncertainty number for every waypoint.

Files changed:
- [src/crackvision/uncertainty.py](https://github.com/Janga786/rebot-crack-vision/blob/fb9d56b818d4c4b6eb2dfcbb3f89dfb52d957d01/src/crackvision/uncertainty.py) (+224 / −0)
- [tests/test_uncertainty.py](https://github.com/Janga786/rebot-crack-vision/blob/fb9d56b818d4c4b6eb2dfcbb3f89dfb52d957d01/tests/test_uncertainty.py) (+285 / −0)

Commits: [`fb9d56b`](https://github.com/Janga786/rebot-crack-vision/commit/fb9d56b818d4c4b6eb2dfcbb3f89dfb52d957d01)

## Why it was done

src/crackvision/uncertainty.py is a pure library that decomposes a lift3d point's §10.4 cov_base into the §14.2 pixel/depth/handeye terms, computes the ADR-018 §3 fk term via a central finite-difference FK Jacobian, maps every waypoint to its §14.5/§10.3 source point, and decomposes a tool waypoint's along-normal variance into pixel/depth/handeye/fk/tcp scalars per §14.4/§14.5, verified by tests/test_uncertainty.py.

- Requirement **REQ-GEOM-1**: Correct depth projection with invalid-depth handling and per-point uncertainty

## How it moves the project forward

- Geometry & calibration: **17/29** tasks accepted; whole project: **86/125**.
- REQ-GEOM-1: 10/14 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔); independent audit accepted it (criteria: 9 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “uncertainty.py correctly decomposes cov_base into pixel/depth/handeye (exact split before the J/R sandwich), adds a PSD fk term via central finite-difference Jacobian that is exactly zero at σ=0 and scales with σ², and the source-point resolution helpers (waypoint_source_indices, measured_source_index) faithfully mirror tool_waypoints._resample_segment and lift3d's gap-fill neighbour-selection logic line-for-line. Tests run real lift3d/tool_waypoints code including a gap-fill case with differing bounding-neighbour traces and verify the without-fk sum matches tool_waypoints'
…[156 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

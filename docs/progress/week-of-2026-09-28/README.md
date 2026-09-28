# Progress — week of Mon Sep 28 – Sun Oct 04, 2026

**3 task(s) completed and independently audited this week** · project total **12/69** accepted

## Notes

- [00-baseline](00-baseline.md)

## Completed this week

- **ARCH-01 — ADR-011: Phase-2 scope lifts the phase-1 deferrals in a controlled order** ([details](ARCH-01.md)): Documented the project's move from phase-1 (perception-only) to phase-2 (full pipeline) scope in a new architecture decision record, ADR-011, which spells out the order deferred work resumes in: order
…[288 chars clipped]
- **GEOM-01 — Frames, units and pixel conventions (ADR-012 + geometry contract)** ([details](GEOM-01.md)): Added ADR-012, the single reference document defining every coordinate frame, transform naming convention, and pixel convention the crack-inspection pipeline will use going forward (robot frames, came
…[344 chars clipped]
- **TC-012 — RealSense software validation** ([details](TC-012.md)): Added a software-only health check for the RealSense D405 camera stack (scripts/check_realsense.py) that verifies the pyrealsense2 library, versions, and any attached device, and correctly reports "no
…[300 chars clipped]

## Quality loop

- Implementation/review sessions run: 5 / 5
- Audits that rejected work with evidence-backed findings: 2 (repairs created: 1)

## Progress by work package

| Work package | Accepted | Total |
|---|---|---|
| Host platform & reproducibility (L-HOST) | 3 | 9 |
| Local coding model (L-LLM) | 0 | 6 |
| D405 capture (L-CAM) | 1 | 7 |
| Perception (L-PERC) | 6 | 16 |
| Geometry & calibration (L-GEOM) | 1 | 9 |
| Motion planning (L-MOTION) | 0 | 10 |
| Integration & commissioning (L-INT) | 0 | 6 |
| Operator workflow & handoff (L-OPS) | 0 | 4 |
| Imitation learning (later phase) (L-IL) | 0 | 1 |

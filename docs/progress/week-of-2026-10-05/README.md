# Progress — week of Mon Oct 05 – Sun Oct 11, 2026

**2 task(s) completed and independently audited this week** · project total **43/92** accepted

## Completed this week

- **GEOM-08.4 — Capture record library + assemble/validate CLI (crackvision.capture_record, INTERFACES §9)** ([details](GEOM-08.4.md)): Implemented crackvision.capture_record: §9 CaptureRecord dataclass, load_capture_record with all listed §9.4 refusal rules (missing robot block, bad joint names, out-of-limit q via FK, missing/mismatc
…[338 chars clipped]
- **GEOM-08.5 — Lift ordered pixel polylines to base_link surface points with normals, gap policy and per-point covariance (crackvision.lift3d)** ([details](GEOM-08.5.md)): Added the crackvision.lift3d module, which converts a 2D crack-path pixel trace plus a depth image into 3D points on the robot's base frame, each with an outward-facing surface normal and an uncertain
…[296 chars clipped]

## Quality loop

- Implementation/review sessions run: 3 / 3
- Audits that rejected work with evidence-backed findings: 1 (repairs created: 1)

## Progress by work package

| Work package | Accepted | Total |
|---|---|---|
| Host platform & reproducibility (L-HOST) | 5 | 9 |
| Local coding model (L-LLM) | 1 | 6 |
| D405 capture (L-CAM) | 4 | 9 |
| Perception (L-PERC) | 13 | 16 |
| Geometry & calibration (L-GEOM) | 10 | 24 |
| Motion planning (L-MOTION) | 7 | 16 |
| Integration & commissioning (L-INT) | 1 | 6 |
| Operator workflow & handoff (L-OPS) | 1 | 4 |
| Imitation learning (later phase) (L-IL) | 0 | 1 |

## Waiting on the operator / hardware (snapshot)

- GEOM-04.1: privilege_denied — Denied. This request fails criteria 1, 2, 3 and 5.

(1) Root isn't needed. The target is /home/boosterk1/miniconda3/envs
…[2288 chars clipped]
- HOST-04: hardware — operator must declare: arm_usb_connected

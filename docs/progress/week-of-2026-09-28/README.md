# Progress — week of Mon Sep 28 – Sun Oct 04, 2026

**11 task(s) completed and independently audited this week** · project total **19/74** accepted

## Notes

- [00-baseline](00-baseline.md)

## Completed this week

- **ARCH-01 — ADR-011: Phase-2 scope lifts the phase-1 deferrals in a controlled order** ([details](ARCH-01.md)): Documented the project's move from phase-1 (perception-only) to phase-2 (full pipeline) scope in a new architecture decision record, ADR-011, which spells out the order deferred work resumes in: order
…[288 chars clipped]
- **GEOM-01 — Frames, units and pixel conventions (ADR-012 + geometry contract)** ([details](GEOM-01.md)): Added ADR-012, the single reference document defining every coordinate frame, transform naming convention, and pixel convention the crack-inspection pipeline will use going forward (robot frames, came
…[344 chars clipped]
- **TC-012 — RealSense software validation** ([details](TC-012.md)): Added a software-only health check for the RealSense D405 camera stack (scripts/check_realsense.py) that verifies the pyrealsense2 library, versions, and any attached device, and correctly reports "no
…[300 chars clipped]
- **TC-010 — Skeletonization utility** ([details](TC-010.md)): Implemented src/crackvision/skeleton.py per docs/INTERFACES.md §3.4: remove_small_objects + skeletonize (nothing more), writing {case}_skeleton.png, {case}_skeleton_overlay.png, {case}_skeleton_stats.
…[262 chars clipped]
- **MOT-01 — Robot model reconciliation (vendor vs presentation URDF) + limits file** ([details](MOT-01.md)): Settled which robot model file is 'the' B601-DM model for planning, confirmed our simulation copy matches the vendor file exactly, and caught a real mismatch where the physical robot's driver currentl
…[395 chars clipped]
- **HOST-01 — Host version manifest + drift/isolation checker** ([details](HOST-01.md)): Added an automated host-health check that snapshots the machine's OS, driver, CUDA, ROS package, Gazebo, and crackvision-environment versions into a pinned manifest and re-verifies them on demand, cat
…[289 chars clipped]
- **LLM-01 — Local coding model shortlist from primary sources (no downloads)** ([details](LLM-01.md)): For the local coding-model shortlist, I sourced exact GGUF files, sizes, cryptographic hashes and licenses directly from Hugging Face for three candidates that fit the 24 GB GPU with 32K+ token contex
…[386 chars clipped]
- **TC-013 — D405 capture utility (hardware-optional)** ([details](TC-013.md)): The D405 camera capture tool (crackvision.realsense_capture) now works end-to-end in synthetic mode with no camera attached: it writes aligned color/depth PNGs plus full intrinsics/extrinsics metadata
…[493 chars clipped]
- **GEOM-02 — Depth projection library with invalid-depth policy** ([details](GEOM-02.md)): Built the depth-projection library (GEOM-02) that turns a pixel + its aligned D405 depth + camera intrinsics into a 3D point in the colour camera frame, matching Intel's own librealsense formula to we
…[519 chars clipped]
- **MOT-02 — ROS 2 overlay workspace + headless MoveIt mock planning** ([details](MOT-02.md)): This repo now has its own ROS 2 overlay workspace that builds on top of the robot's existing ROS install and can plan robot arm motions entirely in software, with no hardware attached. Running the two
…[451 chars clipped]
- **PERC-02 — Skeleton graph: junctions, endpoints, spur pruning** ([details](PERC-02.md)): Built and tested a new perception module that turns a 1-pixel-wide crack skeleton into a graph of junctions, endpoints and loops with the full pixel path along each edge, all in original image coordin
…[330 chars clipped]

## Quality loop

- Implementation/review sessions run: 16 / 19
- Audits that rejected work with evidence-backed findings: 4 (repairs created: 3)
- Re-verified after an upstream change: GEOM-01, GEOM-01.R1, TC-001

## Planning changes

- decomposed: MOT-04 → MOT-04.1, MOT-04.2, MOT-04.3, MOT-04.4, MOT-04.5

## Progress by work package

| Work package | Accepted | Total |
|---|---|---|
| Host platform & reproducibility (L-HOST) | 3 | 9 |
| Local coding model (L-LLM) | 1 | 6 |
| D405 capture (L-CAM) | 2 | 7 |
| Perception (L-PERC) | 8 | 16 |
| Geometry & calibration (L-GEOM) | 2 | 9 |
| Motion planning (L-MOTION) | 2 | 15 |
| Integration & commissioning (L-INT) | 0 | 6 |
| Operator workflow & handoff (L-OPS) | 0 | 4 |
| Imitation learning (later phase) (L-IL) | 0 | 1 |

# Progress — week of Mon Sep 28 – Sun Oct 04, 2026

**24 task(s) completed and independently audited this week** · project total **31/74** accepted

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
- **PERC-03 — Ordered crack paths + path contract** ([details](PERC-03.md)): Added the crack-path extraction stage: given a 1-pixel skeleton, the pipeline now produces an ordered, simplified centerline path for the main crack plus any side branches, with multiple cracks in one
…[196 chars clipped]
- **TC-011 — One-command runner run_test.sh** ([details](TC-011.md)): Built the project's single "run everything" command: ./run_test.sh takes images dropped in data/input_originals/ through environment checks, input prep, GPU/CPU inference, visualization and skeletoniz
…[495 chars clipped]
- **CAM-02 — Lossless RGB-D recording + deterministic replay** ([details](CAM-02.md)): Added a lossless recording/replay layer for the D405 RGB-D pipeline: sessions record colour (lossless PNG) and raw depth (lossless 16-bit PNG) frames plus full device/intrinsics/extrinsics metadata, a
…[233 chars clipped]
- **TC-014 — Integration + contract + hygiene tests** ([details](TC-014.md)): Added the integration and contract test suite for the crack-vision pipeline (TC-014): it runs the real prepare-inputs to visualize to skeleton chain end-to-end on synthetic images (including a non-squ
…[510 chars clipped]
- **TC-015 — Evaluation-manifest tooling** ([details](TC-015.md)): Built the evaluation-manifest CLI (tools/dataset_manifest.py) that will let us track distance, angle, lighting, surface and crack labels for every D405 evaluation image before any real images are capt
…[467 chars clipped]
- **MOT-04.1 — Reachability config + boresight-down target/pose generator (pure Python)** ([details](MOT-04.1.md)): Built the pure-Python geometry and config layer for the arm's reachability sweep: a validated YAML config describing the sweep grid, boresight-down orientation sampling, and IK/placement parameters, p
…[383 chars clipped]
- **MOT-04.2 — Reachability map + placement contract (INTERFACES §7), map I/O, ROS-side CLI common layer** ([details](MOT-04.2.md)): Defined and implemented the reachability-map and specimen-placement data contracts for the robot motion-planning reachability sweep: a new INTERFACES.md section pins down the exact JSON/YAML schemas (
…[266 chars clipped]
- **TC-016 — Phase-1 documentation and reproducibility** ([details](TC-016.md)): Finished the project's documentation deliverable: a rewritten README with a clear quickstart, pipeline diagram and command reference, a dated environment snapshot, a step-by-step setup runbook with an
…[303 chars clipped]
- **MOT-04.3 — Specimen placement scorer + recommend_placement CLI (+ verification-grid emitter)** ([details](MOT-04.3.md)): Finished the specimen-placement scorer and its recommend_placement CLI for the robot motion-planning reachability pipeline: given a completed reachability sweep, it now picks a deterministic best spec
…[391 chars clipped]
- **MOT-04.4 — MoveIt reachability sweep node (compute_ik + independent FK re-check) + headless run/test scripts** ([details](MOT-04.4.md)): Built the MoveIt reachability-sweep tool for the robot arm: it queries the real IK/FK solver (via move_group's compute_ik/compute_fk services, never an action client, so it can't move the arm) across 
…[758 chars clipped]
- **CAM-03 — Recording validator with actionable diagnostics** ([details](CAM-03.md)): Built a one-command recording validator (scripts/validate_recording.py) that checks a D405 recording for metadata completeness, plausible depth scale, timestamp sync/monotonicity, dropped frames, USB-
…[182 chars clipped]

## Quality loop

- Implementation/review sessions run: 35 / 70
- Audits that rejected work with evidence-backed findings: 6 (repairs created: 6)
- Re-verified after an upstream change: CAM-02, GEOM-01, GEOM-01.R1, MOT-02, MOT-04.1, MOT-04.2, MOT-04.3, MOT-04.4, MOT-04.4.R1, PERC-03, PERC-03.R1, TC-001, TC-011

## Planning changes

- decomposed: MOT-04 → MOT-04.1, MOT-04.2, MOT-04.3, MOT-04.4, MOT-04.5

## Progress by work package

| Work package | Accepted | Total |
|---|---|---|
| Host platform & reproducibility (L-HOST) | 4 | 9 |
| Local coding model (L-LLM) | 1 | 6 |
| D405 capture (L-CAM) | 4 | 7 |
| Perception (L-PERC) | 11 | 16 |
| Geometry & calibration (L-GEOM) | 2 | 9 |
| Motion planning (L-MOTION) | 6 | 15 |
| Integration & commissioning (L-INT) | 1 | 6 |
| Operator workflow & handoff (L-OPS) | 1 | 4 |
| Imitation learning (later phase) (L-IL) | 0 | 1 |

## Waiting on the operator / hardware (snapshot)

- GEOM-03: operator_decision — Camera mounting (eye-in-hand vs eye-to-hand) for the D405 is not documented anywhere in this repo or ~/rebot_ws — ARCHIT
…[181 chars clipped]
- MOT-04.5: operator_decision — The committed standoffs [0.01, 0.04] m give no collision-free boresight-down IK solution anywhere: the gripper body exte
…[181 chars clipped]

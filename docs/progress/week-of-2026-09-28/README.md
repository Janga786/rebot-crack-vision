# Progress — week of Mon Sep 28 – Sun Oct 04, 2026

**33 task(s) completed and independently audited this week** · project total **39/92** accepted

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
- **PERC-04 — Path visualization + pipeline stage** ([details](PERC-04.md)): Added a new pipeline stage that draws crack path overlays on top of the original photos — numbered labels show which crack is visited first/second/etc., arrows show the direction each crack is traced,
…[194 chars clipped]
- **GEOM-06 — TCP (pivot) calibration solver + boresight procedure** ([details](GEOM-06.md)): Added the software and procedure for calibrating the robot arm's tool-centre point (the exact 3D offset of the physical tip relative to the wrist). The solver recovers a known test offset to within 0.
…[248 chars clipped]
- **HOST-02 — Lock the crackvision environment + verify-lock script** ([details](HOST-02.md)): The crackvision perception environment can now be locked and its drift automatically checked: `verify_lock.py` correctly reports a clean match against the committed lock files, and `rebuild_env.sh --d
…[416 chars clipped]
- **PERC-09 — Path accuracy metrics on synthetic ground truth** ([details](PERC-09.md)): Added a synthetic crack generator with known ground-truth centerlines (straight, curved, branched, and noisy variants) and a test suite that measures the crack-path-extraction pipeline's accuracy agai
…[232 chars clipped]
- **GEOM-10 — End-of-arm model: tool_tip + wrist D405 (nominal CAD prior) in the planning model** ([details](GEOM-10.md)): Implemented interactively by the technical-lead recovery session (Claude Opus 5.5, operator-authorised, 2026-09-30); commit baed324. Operator facts: D405 eye-in-hand on Seeed stock D405_305_Mount (~15
…[135 chars clipped]
- **MOT-04.6 — Reachability on task frames (tool_tip / camera_link) with a workcell-faithful specimen proxy + camera view check** ([details](MOT-04.6.md)): Implemented interactively by the technical-lead recovery session (Claude Opus 5.5, operator-authorised, 2026-09-30); commit 54e522e. Fixes MOT-04.5's frame error (standoffs meant as tool-tip clearance
…[217 chars clipped]
- **GEOM-11 — TCP/pivot calibration uses the tool_tip prior (ADR-014), not the grasp-centre gripper_tcp** ([details](GEOM-11.md)): Fixed a calibration bug where the TCP/boresight check was comparing measured tool-tip calibrations against the wrong reference point (the gripper's internal grasp-centre, 44mm off from the actual tool
…[439 chars clipped]
- **GEOM-03 — Calibration method + camera mounting decision** ([details](GEOM-03.md)): Finalized the written calibration plan for the wrist camera: ADR-013 now formally records the operator's decision that the D405 is wrist-mounted (eye-in-hand) on the gripper, with a fully specified Ch
…[385 chars clipped]
- **GEOM-08.1 — Contracts: eye-in-hand capture record (§9) + robot-frame 3D path / tool-waypoint file with uncertainty and execution-eligibility policy (§10)** ([details](GEOM-08.1.md)): Wrote the full normative spec for two new pipeline contracts into docs/INTERFACES.md: the eye-in-hand capture record format (§9) that pairs a colour/depth image with the robot's joint state and calibr
…[379 chars clipped]

## Quality loop

- Implementation/review sessions run: 50 / 118
- Audits that rejected work with evidence-backed findings: 6 (repairs created: 6)
- Re-verified after an upstream change: CAM-02, GEOM-01, GEOM-01.R1, GEOM-02, GEOM-02.R1, GEOM-06, GEOM-10, GEOM-11, MOT-01, MOT-02, MOT-04.1, MOT-04.1.R1, MOT-04.2, MOT-04.3, MOT-04.3.R1, MOT-04.4, MOT-04.4.R1, MOT-04.6, PERC-03, PERC-03.R1, TC-001, TC-011, TC-016

## Planning changes

- decomposed: MOT-04 → MOT-04.1, MOT-04.2, MOT-04.3, MOT-04.4, MOT-04.5
- decomposed: GEOM-08 → GEOM-08.1, GEOM-08.2, GEOM-08.3, GEOM-08.4, GEOM-08.5, GEOM-08.6, GEOM-08.7, GEOM-08.8
- decomposed: GEOM-04 → GEOM-04.1, GEOM-04.2, GEOM-04.3, GEOM-04.4, GEOM-04.5
- decomposed: CAM-05 → CAM-05.1, CAM-05.2

## Progress by work package

| Work package | Accepted | Total |
|---|---|---|
| Host platform & reproducibility (L-HOST) | 5 | 9 |
| Local coding model (L-LLM) | 1 | 6 |
| D405 capture (L-CAM) | 4 | 9 |
| Perception (L-PERC) | 13 | 16 |
| Geometry & calibration (L-GEOM) | 7 | 24 |
| Motion planning (L-MOTION) | 6 | 16 |
| Integration & commissioning (L-INT) | 1 | 6 |
| Operator workflow & handoff (L-OPS) | 1 | 4 |
| Imitation learning (later phase) (L-IL) | 0 | 1 |

## Waiting on the operator / hardware (snapshot)

- HOST-04: hardware — operator must declare: arm_usb_connected

# Progress — week of Mon Oct 05 – Sun Oct 11, 2026

**24 task(s) completed and independently audited this week** · project total **65/108** accepted

## Notes

- [01-arm-bringup-2026-10-10](01-arm-bringup-2026-10-10.md)

## Completed this week

- **GEOM-08.4 — Capture record library + assemble/validate CLI (crackvision.capture_record, INTERFACES §9)** ([details](GEOM-08.4.md)): Implemented crackvision.capture_record: §9 CaptureRecord dataclass, load_capture_record with all listed §9.4 refusal rules (missing robot block, bad joint names, out-of-limit q via FK, missing/mismatc
…[338 chars clipped]
- **GEOM-08.5 — Lift ordered pixel polylines to base_link surface points with normals, gap policy and per-point covariance (crackvision.lift3d)** ([details](GEOM-08.5.md)): Added the crackvision.lift3d module, which converts a 2D crack-path pixel trace plus a depth image into 3D points on the robot's base frame, each with an outward-facing surface normal and an uncertain
…[296 chars clipped]
- **GEOM-08.6 — tool_tip trace/approach/retract waypoints from lifted segments (crackvision.tool_waypoints)** ([details](GEOM-08.6.md)): Added the module that turns a lifted 3D crack-surface segment into a sequence of robot tool-tip waypoints: evenly spaced trace points 1cm off the surface along the surface normal, bracketed by approac
…[191 chars clipped]
- **CAM-05.1 — Decision: scope of ROS 2 camera integration (ADR-015)** ([details](CAM-05.1.md)): Wrote a short design decision (ADR-015) clarifying that the camera integration work does not need a full ROS camera driver running at all times; instead it needs only two small one-off ROS tools to gr
…[268 chars clipped]
- **GEOM-08.7 — crackvision.path3d CLI: paths.json + mask + capture record → data/paths3d/{case}_paths3d.json with eligibility** ([details](GEOM-08.7.md)): Built the crackvision.path3d command-line tool, which takes a crack's 2D pixel path plus a robot camera-capture snapshot and computes the actual 3D positions, surface normals, and robot tool-tip waypo
…[470 chars clipped]
- **CAM-05.2 — Static optical-frame TF capture + joint-state capture bridge (crackvision_camera ROS 2 package)** ([details](CAM-05.2.md)): Built the crackvision_camera ROS 2 package with two small one-shot command-line tools: one reads the camera's optical-frame calibration transform off ROS's TF tree and saves it to JSON, the other read
…[370 chars clipped]
- **GEOM-08.3 — FK parity: crackvision.kinematics vs live MoveIt /compute_fk (gripper_link, tool_tip, camera_link) on the mock stack** ([details](GEOM-08.3.md)): Built an automated check that compares the project's own fast forward-kinematics math against MoveIt's official kinematics for the robot's gripper, tool tip, and wrist camera. Across 21 different arm 
…[385 chars clipped]
- **GEOM-08.8 — Synthetic ground-truth verification of pixel→base_link 3D paths and tool waypoints (+ evidence note)** ([details](GEOM-08.8.md)): Added a synthetic ground-truth generator and test suite that validates the full pixel-to-robot-base 3D crack-path pipeline against a known, independently-computed answer. Across two camera tilt angles
…[633 chars clipped]
- **LLM-02 — Build pinned llama.cpp (CUDA, sm_86) in user space** ([details](LLM-02.md)): The pinned llama.cpp build (CUDA, RTX 3090 target) is now fully set up at ~/opt/llama.cpp with no sudo involved, and the project's runtime docs/config were regenerated from that real build rather than
…[156 chars clipped]
- **HOST-06.1 — Fresh-clone + from-lock reproducibility audit, with RUNBOOK conformance fixes** ([details](HOST-06.1.md)): Added an automated audit script that proves this project's environment setup is truly portable: it spins up a full copy of the repo in a scratch location and a brand-new throwaway Python environment b
…[449 chars clipped]
- **MOT-03 — Planning scene from config (nominal until measured)** ([details](MOT-03.md)): Corrected the MOT-03 collision scene per the 2026-09-30 re-spec: removed the world-fixed camera_mount object/ACM entry (the D405 and mount are eye-in-hand robot links from GEOM-10, already in every pl
…[378 chars clipped]
- **MOT-05.1 — Decision + contracts: commissioning-gated execution (ADR-016 + INTERFACES §11)** ([details](MOT-05.1.md)): Finished the design-and-contracts card for MOT-05's safety-gated robot-execution module: ADR-016 plus a new section 11 of INTERFACES.md now fully specify the modes, config schemas, the full real-motio
…[538 chars clipped]
- **MOT-05.2 — Joint-trajectory file library: validation, hashing, limit checks, time scaling, densification (pure Python)** ([details](MOT-05.2.md)): Built the joint-trajectory validation library for the robot motion-execution safety gate: it checks a planned trajectory file against the real B601-DM joint limits (position, velocity, acceleration) a
…[367 chars clipped]
- **MOT-10.1 — Decision + contract: workcell survey procedure and survey record (INTERFACES §12, crackvision.workcell_survey/1)** ([details](MOT-10.1.md)): Defined the step-by-step procedure and file format for physically surveying the workcell (table, specimen, nearby obstacles) with a tape measure, calipers and a level, with no robot motion. The base's
…[282 chars clipped]
- **MOT-05.3 — Offline execution gate: commissioning/approval/execution-config loaders + full gate report (pure Python)** ([details](MOT-05.3.md)): Built the offline safety gate that must pass before the robot arm's trajectory executor is allowed to run in real mode: it checks trajectory validity, joint-limit compliance, speed caps, scene/end-eff
…[521 chars clipped]
- **MOT-10.2 — Workcell survey core: validate a crackvision.workcell_survey/1 record and derive measured scene objects (pure Python)** ([details](MOT-10.2.md)): Added the workcell-survey validator/derivation module (survey_core.py) that turns a raw tape-measure/caliper survey file into MoveIt-ready collision objects (table, specimen, obstacles) with propagate
…[367 chars clipped]
- **MOT-04.5 — Run the full reachability sweep, recommend + independently verify the specimen placement, document** ([details](MOT-04.5.md)): Ran the full-resolution MoveIt reachability sweep (3,780 poses, ~27 minutes) for the B601-DM using the corrected tool-tip/specimen-block model and found a feasible specimen placement at x=0.26 m, y=0.
…[526 chars clipped]
- **MOT-05.4 — execute_trajectory ROS node/CLI: online read-only checks, mock/dry/real execution, typed confirmation, e-stop + tracking monitor, execution record** ([details](MOT-05.4.md)): Implemented execute_trajectory, the ADR-016/§11 commissioning-gated-execution ROS node/CLI for crackvision_motion. It runs the MOT-05.3 offline gate before any rclpy.init() call, then online read-only
…[571 chars clipped]
- **MOT-10.3 — survey_to_scene CLI: survey → scene.yaml (+ verify configs at the measured pose), drift check; decouple scene tests from the production scene's nominal state** ([details](MOT-10.3.md)): Built the survey_to_scene tool that turns a physically-measured workcell survey into the robot's collision-safety scene file, replacing engineering guesses with real tape-measure/caliper numbers. It c
…[390 chars clipped]
- **LLM-03 — Download + verify shortlisted models (bounded, sequential)** ([details](LLM-03.md)): All three shortlisted local coding-model GGUF files remain downloaded and verified; re-running the verification script still exits 0 with no issues.
- **LLM-04 — Project benchmark harness for local coding models** ([details](LLM-04.md)): Built a benchmark harness with 13 realistic coding tasks (writing scripts, writing tests, fixing/editing existing code) drawn from this project's own conventions, each with an automated pass/fail chec
…[483 chars clipped]
- **LLM-05 — Run benchmark, select model, qualify task classes** ([details](LLM-05.md)): Ran the LLM-04 benchmark harness against all 3 LLM-01/03 candidates with identical settings (13 tasks, 3 classes each), selected Laguna-XS-2.1 (Q3_K_M GGUF, pass@1=13/13, lowest VRAM at 16966 MB) for 
…[234 chars clipped]
- **HOST-05 — Storage decision for the unmounted 1.9 TB NVMe (K1_Storage)** ([details](HOST-05.md)): Documented the storage decision for the unmounted 1.9 TB second NVMe drive (labelled K1_Storage, likely belonging to another project): per the operator's decision, it stays completely untouched - no m
…[221 chars clipped]
- **MOT-05.7 — execute_trajectory: subscribe to joint_states with sensor-data (BEST_EFFORT) QoS and separate the DDS-discovery wait from the staleness window** ([details](MOT-05.7.md)): Fixed execute_trajectory's joint_states subscriptions to use BEST_EFFORT (qos_profile_sensor_data) instead of RELIABLE depth-10, matching the real reBotArmController and rebot_motion's mock_driver pub
…[553 chars clipped]

## Quality loop

- Implementation/review sessions run: 50 / 180
- Audits that rejected work with evidence-backed findings: 13 (repairs created: 10)
- Re-verified after an upstream change: CAM-02, CAM-05.1, CAM-05.2, GEOM-01, GEOM-01.R1, GEOM-08.1, GEOM-08.2, GEOM-08.2.R1, GEOM-08.5, GEOM-08.5.R1, GEOM-08.6, GEOM-08.7, GEOM-08.8, GEOM-08.8.R1, GEOM-10, MOT-02, MOT-03, MOT-04.1, MOT-04.1.R1, MOT-04.2, MOT-04.3, MOT-04.3.R1, MOT-04.4, MOT-04.4.R1, MOT-04.6, MOT-05.1, MOT-05.1.R1, MOT-05.2, MOT-05.4, MOT-05.4.R1, MOT-10.1, MOT-10.1.R1, MOT-10.2, MOT-10.3, PERC-03, PERC-03.R1, TC-011, TC-016

## Planning changes

- decomposed: HOST-06 → HOST-06.1
- decomposed: MOT-05 → MOT-05.1, MOT-05.2, MOT-05.3, MOT-05.4, MOT-05.5, MOT-05.6
- decomposed: MOT-10 → MOT-10.1, MOT-10.2, MOT-10.3, MOT-10.4, MOT-10.5
- decomposed: LLM-06 → LLM-06.1, LLM-06.2

## Progress by work package

| Work package | Accepted | Total |
|---|---|---|
| Host platform & reproducibility (L-HOST) | 8 | 10 |
| Local coding model (L-LLM) | 5 | 8 |
| D405 capture (L-CAM) | 7 | 9 |
| Perception (L-PERC) | 13 | 16 |
| Geometry & calibration (L-GEOM) | 15 | 24 |
| Motion planning (L-MOTION) | 14 | 29 |
| Integration & commissioning (L-INT) | 1 | 6 |
| Operator workflow & handoff (L-OPS) | 1 | 4 |
| Imitation learning (later phase) (L-IL) | 0 | 1 |

## Waiting on the operator / hardware (snapshot)

- GEOM-04.1: privilege_denied — Denied. This request fails criteria 1, 2, 3 and 5.

(1) Root isn't needed. The target is /home/boosterk1/miniconda3/envs
…[2288 chars clipped]
- GEOM-09: escalation — lead decomposition failed twice: card GEOM-09.1: missing outcome; card GEOM-09.2: missing outcome; card GEOM-09.3: missing outcome
- HOST-03: escalation — lead decomposition failed twice: card HOST-03.1: missing outcome
- LLM-06.1: other — This implementer's sandbox session mounts plan/ (and task_cards/, .claude-auto/) read-only per its own environment instr
…[181 chars clipped]
- MOT-04.6.R1: operator_decision — The fix needs a file outside this card's scope.write. Either (a) add `sys.path.insert(0, str(Path(__file__).resolve().pa
…[181 chars clipped]
- MOT-10.4: hardware — operator must declare: workcell_measured

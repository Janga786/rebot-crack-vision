# Progress — week of Mon Oct 05 – Sun Oct 11, 2026

**40 task(s) completed and independently audited this week** · project total **71/125** accepted

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
- **HOST-04 — Arm USB adapter access via least-privilege udev rule** ([details](HOST-04.md)): The B601-DM arm's USB-to-CAN adapter now has its own least-privilege udev rule (matched by its specific USB vendor/product ID) installed on the workstation, replacing the operator's manual chmod 666 w
…[211 chars clipped]
- **HOST-03 — RealSense udev rules via a reviewed privileged action** ([details](HOST-03.md)): marked implemented by debugger D0002: Deliverables complete; only the owner-name check was sandbox-unprovable (replaced by an equivalent content+mode+uid-match check). Review the existing commits bc5b
…[33 chars clipped]
- **MOT-04.6 — Reachability on task frames (tool_tip / camera_link) with a workcell-faithful specimen proxy + camera view check** ([details](MOT-04.6-v2.md)): Implemented interactively by the technical-lead recovery session (Claude Opus 5.5, operator-authorised, 2026-09-30); commit 54e522e. Fixes MOT-04.5's frame error (standoffs meant as tool-tip clearance
…[217 chars clipped]
- **MOT-11 — Vendor-driver readiness check: verify reBotArmController's port, motor IDs, joint order, direction and gripper mapping against the 2026-10-10 bring-up evidence (read-only ~/rebot_ws)** ([details](MOT-11.md)): Implemented scripts/motion/check_vendor_driver.py, a pure read-only static checker verifying reBotArmController's port, motor IDs, joint order, direction, zero, gripper mapping, safe_park, SDK API, Py
…[170 chars clipped]
- **MOT-06.1 — Probe MoveIt Cartesian/transit services for task frames on the mock stack** ([details](MOT-06.11.md)): Built and committed a repeatable test that pokes MoveIt's path-planning services (used for tracing cracks) on the simulated robot and records exactly how they behave. The most important result: the pl
…[452 chars clipped]
- **MOT-06.2 — Decision + contract: Cartesian crack-path planning (ADR-017, INTERFACES §13)** ([details](MOT-06.2.md)): Wrote ADR-017 (Cartesian crack-path planning: candidate search over direction x uniform roll offset, selection rule, partial-trace FK-matching/splitting, independent re-check with jump_threshold disab
…[577 chars clipped]
- **MOT-12 — Vendor checker: evaluate the SDK layout its own V-SDK-API remedy produces (SDK pinned at 8427281, actuator/arm.py)** ([details](MOT-12.md)): The vendor-driver readiness checker (check_vendor_driver.py) now correctly evaluates the exact SDK layout its own fix recommends (pinning to commit 8427281), instead of erroring out before it could re
…[364 chars clipped]
- **MOT-06.3 — Cartesian planning core: config loader, paths3d segment tasks, roll/direction candidates, surface-scene check (pure Python)** ([details](MOT-06.3.md)): Built the first pure-Python building blocks for turning a measured 3D crack path into a robot-ready Cartesian plan: a config loader, a crack-path-file reader, and the roll/direction/ordering/collision
…[196 chars clipped]
- **MOT-06.4 — Cartesian plan document: assembly, coverage metrics, strict validator and loader (pure Python)** ([details](MOT-06.4.md)): Built and unit-tested the module that assembles, validates, writes and reads the robot's Cartesian crack-path plan files (and the camera view-plan files) -- the format the next motion-planning stage w
…[374 chars clipped]
- **MOT-06.5 — MoveIt client layer for Cartesian/transit/IK/FK/validity requests (cartesian_moveit)** ([details](MOT-06.5.md)): Built the MoveIt client layer that turns Cartesian crack-path plans into actual MoveIt service calls: it constructs the IK/Cartesian-path/joint-plan/FK/collision-check requests with collision-avoidanc
…[363 chars clipped]
- **GEOM-09.1 — Decision + contract: pixel-to-tool-pose uncertainty budget (ADR-018, INTERFACES §14)** ([details](GEOM-09.1.md)): Added ADR-018 and a new INTERFACES.md §14 that define how we'll audit the existing pixel-to-robot-arm position uncertainty calculation: breaking the total error estimate down into named causes (camera
…[351 chars clipped]
- **MOT-06.6 — plan_crack_path node/CLI: per-segment candidates, transit + approach/trace/retract, split fallback, independent re-check, plan file** ([details](MOT-06.6.md)): Implemented crackvision_motion.plan_crack_path: a MoveIt-calling ROS 2 node/CLI that plans every paths3d segment per docs/INTERFACES.md §13 — §13.3 segment order, §13.4 direction×roll candidates, §13.
…[556 chars clipped]
- **MOT-06.8 — View-phase core: camera_link view-pose candidates and D405 FOV coverage check (pure Python)** ([details](MOT-06.8.md)): Added the pure-Python geometry core for the robot's camera "view phase": given the specimen's position (from the scene file or a manual center), it generates candidate camera poses at 0° and 15° tilt 
…[353 chars clipped]
- **MOT-05.5 — Headless mock verification of the execution gate: dry/mock execution, refusals, collision, e-stop injection, vendor-interface mock** ([details](MOT-05.5.md)): scripts/ros/test_execution.sh now checks the commissioning-gated trajectory executor end to end with no hardware, using both the MoveIt mock stack and the vendor-interface mock_driver. All 9 cases pas
…[387 chars clipped]
- **MOT-06.11 — plan_crack_path: fraction==1.0 traces are full (no pointer-walk stall on resampled trajectories) + decouple IK solver budget from service-call waits** ([details](MOT-06.11.md)): Fixed the two defects debugger D0010 identified in plan_crack_path's Cartesian trace planning: (1) step-4 full-ness now follows §13.5 literally (fraction == 1.0 from /compute_cartesian_path itself), w
…[956 chars clipped]

## Quality loop

- Implementation/review sessions run: 84 / 340
- Audits that rejected work with evidence-backed findings: 26 (repairs created: 21)
- Re-verified after an upstream change: CAM-02, CAM-05.1, CAM-05.2, GEOM-01, GEOM-01.R1, GEOM-08.1, GEOM-08.2, GEOM-08.2.R1, GEOM-08.5, GEOM-08.5.R1, GEOM-08.6, GEOM-08.7, GEOM-08.8, GEOM-08.8.R1, GEOM-09.1.R1, GEOM-10, MOT-02, MOT-03, MOT-04.1, MOT-04.1.R1, MOT-04.2, MOT-04.3, MOT-04.3.R1, MOT-04.4, MOT-04.4.R1, MOT-04.6, MOT-04.6.R1, MOT-05.1, MOT-05.1.R1, MOT-05.2, MOT-05.4, MOT-05.4.R1, MOT-06.2, MOT-06.2.R1, MOT-06.3, MOT-06.3.R1, MOT-06.4, MOT-06.4.R1, MOT-06.5, MOT-06.5.R1, MOT-06.6.R1, MOT-10.1, MOT-10.1.R1, MOT-10.1.R2, MOT-10.2, MOT-10.3, PERC-03, PERC-03.R1, TC-011, TC-016

## Planning changes

- decomposed: HOST-06 → HOST-06.1
- decomposed: MOT-05 → MOT-05.1, MOT-05.2, MOT-05.3, MOT-05.4, MOT-05.5, MOT-05.6
- decomposed: MOT-10 → MOT-10.1, MOT-10.2, MOT-10.3, MOT-10.4, MOT-10.5
- decomposed: LLM-06 → LLM-06.1, LLM-06.2
- decomposed: MOT-06 → MOT-06.1, MOT-06.10, MOT-06.2, MOT-06.3, MOT-06.4, MOT-06.5, MOT-06.6, MOT-06.7, MOT-06.8, MOT-06.9
- decomposed: GEOM-09 → GEOM-09.1, GEOM-09.2, GEOM-09.3, GEOM-09.4

## Progress by work package

| Work package | Accepted | Total |
|---|---|---|
| Host platform & reproducibility (L-HOST) | 10 | 10 |
| Local coding model (L-LLM) | 5 | 8 |
| D405 capture (L-CAM) | 7 | 9 |
| Perception (L-PERC) | 13 | 16 |
| Geometry & calibration (L-GEOM) | 14 | 29 |
| Motion planning (L-MOTION) | 19 | 41 |
| Integration & commissioning (L-INT) | 1 | 6 |
| Operator workflow & handoff (L-OPS) | 1 | 4 |
| Imitation learning (later phase) (L-IL) | 0 | 1 |

## Waiting on the operator / hardware (snapshot)

- CAM-01: hardware — operator must declare: d405_connected
- GEOM-04.1: privilege_denied — Denied. This request fails criteria 1, 2, 3 and 5.

(1) Root isn't needed. The target is /home/boosterk1/miniconda3/envs
…[2288 chars clipped]
- LLM-06.1: dependency — The backlog currently has zero cards that are simultaneously unaccepted/ready, non-hardware, motion!=true, routing.conse
…[181 chars clipped]

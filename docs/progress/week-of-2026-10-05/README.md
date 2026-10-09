# Progress — week of Mon Oct 05 – Sun Oct 11, 2026

**9 task(s) completed and independently audited this week** · project total **52/92** accepted

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

## Quality loop

- Implementation/review sessions run: 12 / 18
- Audits that rejected work with evidence-backed findings: 2 (repairs created: 2)
- Re-verified after an upstream change: CAM-05.1, GEOM-08.1, MOT-02

## Progress by work package

| Work package | Accepted | Total |
|---|---|---|
| Host platform & reproducibility (L-HOST) | 5 | 9 |
| Local coding model (L-LLM) | 2 | 6 |
| D405 capture (L-CAM) | 7 | 9 |
| Perception (L-PERC) | 13 | 16 |
| Geometry & calibration (L-GEOM) | 15 | 24 |
| Motion planning (L-MOTION) | 7 | 16 |
| Integration & commissioning (L-INT) | 1 | 6 |
| Operator workflow & handoff (L-OPS) | 1 | 4 |
| Imitation learning (later phase) (L-IL) | 0 | 1 |

## Waiting on the operator / hardware (snapshot)

- GEOM-04.1: privilege_denied — Denied. This request fails criteria 1, 2, 3 and 5.

(1) Root isn't needed. The target is /home/boosterk1/miniconda3/envs
…[2288 chars clipped]
- GEOM-09: escalation — lead decomposition failed twice: card GEOM-09.1: missing outcome; card GEOM-09.2: missing outcome; card GEOM-09.3: missing outcome
- HOST-04: hardware — operator must declare: arm_usb_connected

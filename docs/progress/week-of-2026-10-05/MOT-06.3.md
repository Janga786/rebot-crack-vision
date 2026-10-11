# MOT-06.3 — Cartesian planning core: config loader, paths3d segment tasks, roll/direction candidates, surface-scene check (pure Python)

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`4742758`](https://github.com/Janga786/rebot-crack-vision/commit/4742758a9ea1962bd88bd25a3f57c42ecb925bce)

## What was done

Built the first pure-Python building blocks for turning a measured 3D crack path into a robot-ready Cartesian plan: a config loader, a crack-path-file reader, and the roll/direction/ordering/collision-check logic the full planner will use next. All 28 new tests plus the existing 308 motion-stack tests pass (336 total), and the new config file loads successfully with no ROS dependency required.

After the first audit, a repair round (MOT-06.3.R1) fixed the reported issues: Fixed two bugs found in review of the Cartesian crack-path planner's pre-planning safety check: it was flagging every geometrically correct crack trace as unsafe because it didn't know the specimen is supposed to sit exactly at the planned standoff distance, and it couldn't load capture files for cracks that lost all their depth data. Both are now fixed and verified against a scene matching production (table + specimen), with all 341 motion-package unit tests passing.

Files changed:
- [config/motion/cartesian_planning.yaml](https://github.com/Janga786/rebot-crack-vision/blob/4742758a9ea1962bd88bd25a3f57c42ecb925bce/config/motion/cartesian_planning.yaml) (+55 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py](https://github.com/Janga786/rebot-crack-vision/blob/4742758a9ea1962bd88bd25a3f57c42ecb925bce/ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py) (+689 / −13)
- [ros2_ws/src/crackvision_motion/test/fixtures/cartesian/paths3d_multi_component.json](https://github.com/Janga786/rebot-crack-vision/blob/4742758a9ea1962bd88bd25a3f57c42ecb925bce/ros2_ws/src/crackvision_motion/test/fixtures/cartesian/paths3d_multi_component.json) (+153 / −0)
- [ros2_ws/src/crackvision_motion/test/test_cartesian_core.py](https://github.com/Janga786/rebot-crack-vision/blob/4742758a9ea1962bd88bd25a3f57c42ecb925bce/ros2_ws/src/crackvision_motion/test/test_cartesian_core.py) (+531 / −6)

Commits: [`a0d8bca`](https://github.com/Janga786/rebot-crack-vision/commit/a0d8bca1573afd085e99a67f6a106988567649ae), [`4742758`](https://github.com/Janga786/rebot-crack-vision/commit/4742758a9ea1962bd88bd25a3f57c42ecb925bce)

## Why it was done

crackvision_motion.cartesian_core loads the §13 planning config and a §10 paths3d file into ordered segment tasks (tool_tip approach/trace/retract poses), generates the deterministic direction×roll candidate set, applies uniform roll offsets, and flags surface/scene mismatches — ROS-free and unit-tested; config/motion/cartesian_planning.yaml ships with sourced nominal values.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **22/40** tasks accepted; whole project: **75/123**.
- REQ-MOT-2: 10/21 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, regression ✔, config-loads ✔); independent audit accepted it (criteria: 9 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Both findings are fixed. surface_scene_mismatch now takes surface_object_id and surface_object_tolerance_m. It reads each trace waypoint's clearance_m from paths3d through the new SegmentTask.clearances_m field, and that field is carried through segment_tasks, apply_candidate (reversed when direction is reversed) and split_remainder. The surface object is checked against the relaxed bound clearance_m − tolerance and every other object against the full clearance, as §13.9 requires. load_paths3d accepts `segments: []` and still rejects a non-list value. New tests use the prod
…[744 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task MOT-06.3.R1): surface_scene_mismatch omits §13.9 surface_object_id relaxation and per-waypoint clearance, so it flags every correct trace in the production scene; load_paths3d rejects valid §10 files with a polyline whose segments[] is empty.

## What it unlocks next

- **Ready to start:** MOT-06.4 — Cartesian plan document: assembly, coverage metrics, strict validator and loader (pure Python)
- **Ready to start:** MOT-06.8 — View-phase core: camera_link view-pose candidates and D405 FOV coverage check (pure Python)
- Closer: MOT-06.6 — plan_crack_path node/CLI: per-segment candidates, transit + approach/trace/retract, split fallback, independent re-check, plan file (still needs MOT-06.4, MOT-06.5)

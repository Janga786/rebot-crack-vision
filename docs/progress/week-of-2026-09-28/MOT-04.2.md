# MOT-04.2 — Reachability map + placement contract (INTERFACES §7), map I/O, ROS-side CLI common layer

**Accepted:** 2026-09-29 · **Work package:** Motion planning (L-MOTION) · **Track:** software · **Verified commit:** [`d8032f8`](https://github.com/Janga786/rebot-crack-vision/commit/d8032f8a5d7776292be8710594a00ce67e2cf91b)

## What was done

Defined and implemented the reachability-map and specimen-placement data contracts for the robot motion-planning reachability sweep: a new INTERFACES.md section pins down the exact JSON/YAML schemas (units, frames, status codes), a Python module builds/writes/validates these files atomically, and a shared CLI helper gives the upcoming ROS command-line tools consistent exit codes and log output. All 49 new tests and the 26 pre-existing related tests pass (75/75).

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/d8032f8a5d7776292be8710594a00ce67e2cf91b/docs/INTERFACES.md) (+167 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py](https://github.com/Janga786/rebot-crack-vision/blob/d8032f8a5d7776292be8710594a00ce67e2cf91b/ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py) (+216 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/reachability_map.py](https://github.com/Janga786/rebot-crack-vision/blob/d8032f8a5d7776292be8710594a00ce67e2cf91b/ros2_ws/src/crackvision_motion/crackvision_motion/reachability_map.py) (+352 / −0)
- [ros2_ws/src/crackvision_motion/test/test_cli_common.py](https://github.com/Janga786/rebot-crack-vision/blob/d8032f8a5d7776292be8710594a00ce67e2cf91b/ros2_ws/src/crackvision_motion/test/test_cli_common.py) (+229 / −0)
- [ros2_ws/src/crackvision_motion/test/test_reachability_map.py](https://github.com/Janga786/rebot-crack-vision/blob/d8032f8a5d7776292be8710594a00ce67e2cf91b/ros2_ws/src/crackvision_motion/test/test_reachability_map.py) (+389 / −0)

Commits: [`d8032f8`](https://github.com/Janga786/rebot-crack-vision/commit/d8032f8a5d7776292be8710594a00ce67e2cf91b)

## Why it was done

A new numbered INTERFACES section fixes the reachability-map JSON and specimen-placement YAML schemas. reachability_map.py builds, writes and validates maps. cli_common.py gives the ROS-side (system python3) CLIs the §0.2 exit codes, §0.3 common flags and §0.4 log artefacts.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-2**: Reproducible install/config/tests/evaluation results, handoff docs, requirements→evidence map, honest readiness report

## How it moves the project forward

- Motion planning: **2/15** tasks accepted; whole project: **22/74**.
- REQ-MOT-2: 1/9 contributing tasks done
- REQ-OPS-2: 2/5 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, contract-section ✔, append-only ✔); independent audit accepted it (criteria: 11 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “MOT-04.2 appends INTERFACES.md §7 defining the reachability-map JSON and specimen-placement YAML schemas, implements reachability_map.py (build/write/load/validate, atomic writes, joint-margin math) and cli_common.py (§0.2/§0.3/§0.4 mirroring config.py/logging_setup.py). All three scheduler checks reproduce locally (49/49 unit tests pass, contract-section grep matches, append-only diff verified), scope is exactly the five allowed files, and all listed acceptance criteria are met by direct code inspection.”

## What it unlocks next

- **Ready to start:** MOT-04.3 — Specimen placement scorer + recommend_placement CLI (+ verification-grid emitter)
- **Ready to start:** MOT-04.4 — MoveIt reachability sweep node (compute_ik + independent FK re-check) + headless run/test scripts

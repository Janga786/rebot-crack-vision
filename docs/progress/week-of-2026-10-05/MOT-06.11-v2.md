# MOT-06.11 — plan_crack_path: fraction==1.0 traces are full (no pointer-walk stall on resampled trajectories) + decouple IK solver budget from service-call waits

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`69121d9`](https://github.com/Janga786/rebot-crack-vision/commit/69121d9561d145dc541ad5cbf966e3856cb1a595)

## What was done

Fixed the two defects debugger D0010 identified in plan_crack_path's Cartesian trace planning: (1) step-4 full-ness now follows §13.5 literally (fraction == 1.0 from /compute_cartesian_path itself), with a new per-configuration FK-agreement rule (nearest-point-on-polyline position check + bracket-interpolated orientation check + explicit final-waypoint check) replacing the §13.7 pointer walk for that case only (the walk still runs unchanged for fraction<1); (2) the /compute_ik solver budget is now a separate config key (ik_solver_timeout_s, default 0.05s matching rebot_ws's kinematics.yaml) decoupled from service_timeout_s (raised to 15.0s), which now exclusively bounds the slower Cartesian/motion-plan/fk/validity calls. Measured mock-stack facts motivating both fixes and a validation of the new rule are recorded in an ADR-017 addendum and an appended docs/INTERFACES.md §15. Added a committed test-only fixture (0.06 m / 31-waypoint boresight-down trace at the recommended placement) and scripts/ros/test_plan_trace_full.sh, which plans it against the mock stack and asserts full coverage, passing recheck on every leg, and a sub-300s runtime.

After the first audit, a repair round (MOT-06.11.R1) fixed the reported issues: The interface contract for Cartesian crack-path planning is consistent again. The amendment section (§15) now names every original section it changes, including the config schema in §13.2, and the original §13 text is untouched. The planner build, all 496 motion-package tests and the full-trace mock-stack run (5 legs, all rechecks passed, 179 s) still pass.

Files changed:
- [config/motion/cartesian_planning.yaml](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/config/motion/cartesian_planning.yaml) (+4 / −2)
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/docs/INTERFACES.md) (+88 / −12)
- [docs/adr/017-cartesian-crack-path-planning.md](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/docs/adr/017-cartesian-crack-path-planning.md) (+83 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py) (+9 / −2)
- [ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py) (+19 / −13)
- [ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py) (+115 / −9)
- [ros2_ws/src/crackvision_motion/test/fixtures/cartesian/trace_full/capture_record.json](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/ros2_ws/src/crackvision_motion/test/fixtures/cartesian/trace_full/capture_record.json) (+23 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/cartesian/trace_full/paths3d.json](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/ros2_ws/src/crackvision_motion/test/fixtures/cartesian/trace_full/paths3d.json) (+862 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/cartesian/trace_full/scene.yaml](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/ros2_ws/src/crackvision_motion/test/fixtures/cartesian/trace_full/scene.yaml) (+41 / −0)
- [ros2_ws/src/crackvision_motion/test/test_cartesian_core.py](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/ros2_ws/src/crackvision_motion/test/test_cartesian_core.py) (+27 / −0)
- [ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py) (+2 / −1)
- [ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py) (+230 / −2)
- [scripts/ros/test_plan_trace_full.sh](https://github.com/Janga786/rebot-crack-vision/blob/69121d9561d145dc541ad5cbf966e3856cb1a595/scripts/ros/test_plan_trace_full.sh) (+114 / −0)

Commits: [`389e52b`](https://github.com/Janga786/rebot-crack-vision/commit/389e52b6c260b0d06c8690bf02fca010b6be5ad2), [`817c31c`](https://github.com/Janga786/rebot-crack-vision/commit/817c31c829d8ba85a31c2868a8bd563069f7b0db), [`69121d9`](https://github.com/Janga786/rebot-crack-vision/commit/69121d9561d145dc541ad5cbf966e3856cb1a595)

## Why it was done

plan_crack_path plans a body-sized (0.06 m, 31-waypoint) boresight-down straight trace at the recommended placement as `full` on the mock stack with the committed config/motion/cartesian_planning.yaml, in bounded time: step-4 full-ness follows §13.5 (fraction == 1.0), every trace configuration is still FK-checked against the requested polyline, and the /compute_ik solver budget is no longer the same number as the Cartesian/motion-plan call wait.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **17/41** tasks accepted; whole project: **64/125**.
- REQ-MOT-1: 16/34 contributing tasks done
- REQ-MOT-2: 5/22 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, regression ✔, live-trace ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The §15 intro disclaimer in docs/INTERFACES.md now correctly lists §13.2 (via §15.3) and §13.11.2 (for fraction==1.0 trace legs, via §15.2) as amended, removing the prior self-contradiction. §13 itself remains byte-identical to commit 0f783b4 (confirmed via diff), build passes, and all 496 motion-package tests pass.”
- Quality loop: the audit found 3 issue(s) that were fixed before acceptance (repair task MOT-06.11.R1): §15's 'nothing changes §13.9–§13.14' disclaimer contradicts its own override of §13.11.2's waypoint_index_map rule; Repair edits §13.11.2 in place, which MOT-06.11 forbids and which contradicts §15's own 'not edited in place' statement; Rewritten §15 disclaimer still says §13.2 is unchanged, but §15.3 amends the §13.2 config schema.

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

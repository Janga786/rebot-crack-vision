# MOT-06.5 — MoveIt client layer for Cartesian/transit/IK/FK/validity requests (cartesian_moveit)

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`453d1ab`](https://github.com/Janga786/rebot-crack-vision/commit/453d1ab25818f745f0a75acd183275148ec29089)

## What was done

Built the MoveIt client layer that turns Cartesian crack-path plans into actual MoveIt service calls: it constructs the IK/Cartesian-path/joint-plan/FK/collision-check requests with collision-avoidance always on, and turns the responses back into plain Python results, including the FK-based method for figuring out how far along a crack trace a partial path actually got. Verified with 26 new unit tests (all passing, no running robot needed) plus the full 433-test suite for this package still passing after the change, and the ROS package still builds cleanly.

After the first audit, a repair round (MOT-06.5.R1) fixed the reported issues: Fixed a timing bug in the robot motion-planning code where an unreachable pose could crash the program instead of being reported as a normal "can't reach that point" result. Added a small safety margin (2 seconds) between the inverse-kinematics solver's own timeout and how long the system waits for its answer, matching a pattern already used elsewhere in the codebase. Verified with a new automated test, and reran the full test suite (434 tests) plus the project build, all passing.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py](https://github.com/Janga786/rebot-crack-vision/blob/453d1ab25818f745f0a75acd183275148ec29089/ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py) (+386 / −2)
- [ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py](https://github.com/Janga786/rebot-crack-vision/blob/453d1ab25818f745f0a75acd183275148ec29089/ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py) (+438 / −0)

Commits: [`f44fb58`](https://github.com/Janga786/rebot-crack-vision/commit/f44fb5898d589f464b078cb862088162e4a52a72), [`453d1ab`](https://github.com/Janga786/rebot-crack-vision/commit/453d1ab25818f745f0a75acd183275148ec29089)

## Why it was done

crackvision_motion.cartesian_moveit builds GetCartesianPath (link_name tool_tip, avoid_collisions true), GetMotionPlan (joint goal, explicit start_state), IK, FK and state-validity requests and parses their responses into plain Python results (configs, last reached waypoint by FK match, error codes), with message-level unit tests and no collision relaxation possible through its API.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **24/40** tasks accepted; whole project: **77/123**.
- REQ-MOT-1: 20/33 contributing tasks done
- REQ-MOT-2: 12/21 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, regression ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this. The repair adds `_IK_CALL_TIMEOUT_MARGIN_S = 2.0` and an `ik_call_timeout_s(cfg)` helper that returns `service_timeout_s + 2.0`. That is the same margin pattern `reachability_sweep` uses for its own IK calls. The docstrings now tell callers to pass this value as the wait timeout for `/compute_ik` calls. A new test checks that the solver timeout inside the built request is strictly smaller than the helper's call timeout. The diff only touches the two in-scope files. All 434 regression tests pass when I re-run them, and none of the original MOT-06.5 code p
…[240 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task MOT-06.5.R1): IK solver timeout equals the service-call wait timeout: infeasible IK can surface as a runtime timeout (exit 1) instead of ik_failed.

## What it unlocks next

- **Ready to start:** MOT-06.6 — plan_crack_path node/CLI: per-segment candidates, transit + approach/trace/retract, split fallback, independent re-check, plan file
- Closer: MOT-06.9 — plan_view node/CLI + headless smoke: reachable, collision-free camera view pose with the specimen in FOV (still needs MOT-06.8, MOT-06.6)

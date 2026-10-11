# MOT-06.6 — plan_crack_path node/CLI: per-segment candidates, transit + approach/trace/retract, split fallback, independent re-check, plan file

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`056e9d7`](https://github.com/Janga786/rebot-crack-vision/commit/056e9d77abf9da0c51a8216068aae4259634879c)

## What was done

Implemented crackvision_motion.plan_crack_path: a MoveIt-calling ROS 2 node/CLI that plans every paths3d segment per docs/INTERFACES.md §13 — §13.3 segment order, §13.4 direction×roll candidates, §13.5 IK/transit/Cartesian approach-trace-retract per candidate, §13.6 margin-based selection with first-in-order tie-break, §13.7 partial-trace split/re-entry (bounded by max_splits_per_segment, with too_short/exhausted-splits handling and a synthesised retract leg), §13.8 independent re-check (FK, /check_state_validity, inclusive joint limits, joint-step) on every committed configuration, and §13.9 pre-planning surface/scene mismatch (segments flagged this way are reported and never planned). Registered the `plan_crack_path` console script in setup.py.

After the first audit, a repair round (MOT-06.6.R1) fixed the reported issues: After a partial trace, the crack-path planner now retries the unreached part of the crack in the original paths3d order and with the original tool orientations, even when the first attempt ran the crack in reverse with a tool roll. A new unit test checks this pose by pose. All 468 crackvision_motion unit tests pass.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py](https://github.com/Janga786/rebot-crack-vision/blob/056e9d77abf9da0c51a8216068aae4259634879c/ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py) (+981 / −14)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/056e9d77abf9da0c51a8216068aae4259634879c/ros2_ws/src/crackvision_motion/setup.py) (+1 / −0)
- [ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py](https://github.com/Janga786/rebot-crack-vision/blob/056e9d77abf9da0c51a8216068aae4259634879c/ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py) (+649 / −12)

Commits: [`9483967`](https://github.com/Janga786/rebot-crack-vision/commit/94839672d54602ce5ba4de73e2bad234ec50ffaa), [`dd672e6`](https://github.com/Janga786/rebot-crack-vision/commit/dd672e65da68e5f7765125fe8bdc649d0543f017), [`056e9d7`](https://github.com/Janga786/rebot-crack-vision/commit/056e9d77abf9da0c51a8216068aae4259634879c)

## Why it was done

`ros2 run crackvision_motion plan_crack_path --paths3d PATH` applies the scene, plans every paths3d segment per §13 (direction×roll candidates, transit via OMPL, Cartesian approach/trace/retract for tool_tip, bounded split re-entry), re-checks every configuration independently, and writes a validated crackvision.cartesian_plan/1 with coverage, following §0/§13 exit codes and a ROS-free --dry-run.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **25/40** tasks accepted; whole project: **79/124**.
- REQ-MOT-1: 21/33 contributing tasks done
- REQ-MOT-2: 13/21 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, regression ✔, entry-point ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair correctly fixes finding c6af65a5745b06c4: for a reverse-direction partial, run_segment_splits now splits the un-rolled reversed task at the reversed-traversal last-reached index, then reverses the remainder back to paths3d order, which I verified algebraically reproduces exactly trace[0:N-1-k] in paths3d order (double-reversal cancels). All four scheduler acceptance checks reproduce locally (build, unit 34 passed, regression 468 passed, entry-point --help), and the four prior findings (recheck FK scoping, rolled/reversed retry base, §13.8 shortfall classification
…[136 chars clipped]”
- Quality loop: the audit found 5 issue(s) that were fixed before acceptance (repair task MOT-06.6.R1): Split retries reuse the previous candidate's rolled/reversed poses, compounding roll and mislabelling chosen; splitting.min_waypoints_per_remainder config value is ignored; Committed legs record recheck FK errors that violate §13.11.2's pass condition; §13.8 fraction<1 shortfall classification (collision/joint_jump) not implemented; A split retry after a reverse-direction partial keeps the reversal and reports direction relative to the reversed traversal, not paths3d.

## What it unlocks next

- **Ready to start:** MOT-06.7 — Headless mock-stack smoke test of plan_crack_path with consistent scene + paths3d fixtures
- Closer: MOT-06.9 — plan_view node/CLI + headless smoke: reachable, collision-free camera view pose with the specimen in FOV (still needs MOT-06.8)

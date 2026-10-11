# MOT-06.1 — Probe MoveIt Cartesian/transit services for task frames on the mock stack

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`343f4c7`](https://github.com/Janga786/rebot-crack-vision/commit/343f4c7886e44f7cebdde1a8664e41f2f3b39f4d)

## What was done

Built and committed a repeatable test that pokes MoveIt's path-planning services (used for tracing cracks) on the simulated robot and records exactly how they behave. The most important result: the planner cannot find a solution for the exact "straight down" tool orientation the plan assumed -- it sits on a mathematical blind spot of this arm's wrist -- but works fine a couple of degrees off, so the real planner will need to avoid that exact angle. The probe also pinned down precisely how to tell how far along a path the robot actually got when it stops early (e.g. because of an obstacle), which the next motion-planning card needs to get right.

After the first audit, a repair round (MOT-06.1.R1) fixed the reported issues: The repair task was already finished and verified in the previous turn; this turn just confirmed nothing further was pending.

Files changed:
- [docs/motion/CARTESIAN_PROBE.md](https://github.com/Janga786/rebot-crack-vision/blob/343f4c7886e44f7cebdde1a8664e41f2f3b39f4d/docs/motion/CARTESIAN_PROBE.md) (+69 / −2)
- [scripts/ros/probe_cartesian_path.py](https://github.com/Janga786/rebot-crack-vision/blob/343f4c7886e44f7cebdde1a8664e41f2f3b39f4d/scripts/ros/probe_cartesian_path.py) (+631 / −6)
- [scripts/ros/probe_cartesian_path.sh](https://github.com/Janga786/rebot-crack-vision/blob/343f4c7886e44f7cebdde1a8664e41f2f3b39f4d/scripts/ros/probe_cartesian_path.sh) (+52 / −0)

Commits: [`48784b6`](https://github.com/Janga786/rebot-crack-vision/commit/48784b6060906ea96bb3bc61ec459092af9f637e), [`343f4c7`](https://github.com/Janga786/rebot-crack-vision/commit/343f4c7886e44f7cebdde1a8664e41f2f3b39f4d)

## Why it was done

A committed, re-runnable live probe plus a note establishing how Humble's /compute_cartesian_path and /plan_kinematic_path behave for link_name tool_tip/camera_link on the MOT-02 mock stack (fields, fraction semantics, timing, collision stop, jump thresholds).

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **19/39** tasks accepted; whole project: **72/122**.
- REQ-MOT-1: 17/32 contributing tasks done
- REQ-MOT-2: 8/21 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, probe ✔, note ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “All three review findings from MOT-06.1's earlier review are genuinely fixed in commit 343f4c7, and the scheduler's acceptance checks (build, probe, note) pass at this revision with a clean working tree matching the reviewed commit.”
- Quality loop: the audit found 3 issue(s) that were fixed before acceptance (repair task MOT-06.1.R1): Doc's FK error figure contradicts its own cited JSON value by ~300x; Singularity 'seed/collision-independent' claim is not backed by the committed, re-runnable artifact; Probe can exit 0 on an incomplete run (fallback-IK-fails branch).

## What it unlocks next

- **Ready to start:** MOT-06.2 — Decision + contract: Cartesian crack-path planning (ADR-017, INTERFACES §13)
- Closer: MOT-06.5 — MoveIt client layer for Cartesian/transit/IK/FK/validity requests (cartesian_moveit) (still needs MOT-06.2)

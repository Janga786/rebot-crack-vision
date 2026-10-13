# MOT-05.6 — Operator execution runbook (mock → dry → real) + ROS_WORKSPACE.md entry

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`92e9fa9`](https://github.com/Janga786/rebot-crack-vision/commit/92e9fa9d4db0647975b9b5e52e5bcabce766002f)

## What was done

Added the operator runbook (docs/motion/EXECUTION.md) that tells anyone running the robot executor exactly how to go from a safe mock test to a real-hardware run, including why real mode is currently refused, how to use the emergency stop, and what to do if something goes wrong. Verified both automated checks pass: the doc contains every required term and every safety-gate name the code actually defines. Also added a short pointer to this new doc from the existing workspace documentation, without changing any of its prior content.

After the first audit, a repair round (MOT-05.6.R1) fixed the reported issues: Fixed a stale documentation row: the robot-arm execution gate docs now describe the view_move trajectory eligibility check separately from the crack-task case, so an operator debugging a failed check is pointed at the right cause and fix instead of an unrelated one. Verified both automated checks pass.

Files changed:
- [docs/motion/EXECUTION.md](https://github.com/Janga786/rebot-crack-vision/blob/92e9fa9d4db0647975b9b5e52e5bcabce766002f/docs/motion/EXECUTION.md) (+247 / −1)
- [docs/motion/ROS_WORKSPACE.md](https://github.com/Janga786/rebot-crack-vision/blob/92e9fa9d4db0647975b9b5e52e5bcabce766002f/docs/motion/ROS_WORKSPACE.md) (+11 / −0)

Commits: [`7749239`](https://github.com/Janga786/rebot-crack-vision/commit/77492395d0a0c443c2f2ab11657a428989f4e0da), [`92e9fa9`](https://github.com/Janga786/rebot-crack-vision/commit/92e9fa9d4db0647975b9b5e52e5bcabce766002f)

## Why it was done

docs/motion/EXECUTION.md is the operator-facing procedure for the commissioning-gated executor. It covers prerequisites per gate id, how to produce each record (commissioning, approval), the mock→dry→real progression, the typed confirmation, e-stop use (hardware first), reading the execution record and failure recovery. docs/motion/ROS_WORKSPACE.md points to it and to test_execution.sh. MOT-09 and OPS-01 can use it without reading code.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **43/57** tasks accepted; whole project: **105/144**.
- REQ-MOT-1: 33/41 contributing tasks done
- REQ-INT-2: 9/16 contributing tasks done
- Verification: automated checks run by the pipeline itself (doc-present ✔, gate-ids-documented ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair splits the G-ELIGIBLE row into two rows: one for crack_task/test and one for view_move. I checked the view_move row against `_evaluate_eligible_view_move` (execution_gate.py:719-773) and INTERFACES.md §17.2, and it matches both. It lists the integrity checks in order: the file exists, the sha matches, it loads via `load_view_plan`, `chosen`/`transit_leg` are non-null, and both endpoints match within 1e-9 rad. All of these refuse in every mode. It then gives the three sha-binding eligibility checks (limits, end_effector, scene), which warn in mock/dry and refuse i
…[234 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task MOT-05.6.R1): G-ELIGIBLE row in EXECUTION.md is stale for purpose:"view_move" trajectories.

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

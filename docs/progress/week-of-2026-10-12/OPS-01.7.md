# OPS-01.7 — plan stage: path3d -> plan_crack_path -> retime_plan -> trajectory_check for one case

**Accepted:** 2026-10-14 · **Work package:** Operator workflow & handoff (L-OPS) · **Track:** software · **Verified commit:** [`aa9dbb6`](https://github.com/Janga786/rebot-crack-vision/commit/aa9dbb66481f34e775a0788022b662274c36c2b3)

## What was done

Added the `plan` stage of the operator CLI workflow: a single script now runs path3d, the ROS motion planner, the trajectory retimer, and the independent trajectory checker in order for one crack case, skipping work already done and refusing to run if capture/paths haven't completed first. Verified with a full headless simulation run against a synthetic crack specimen and the mock robot-planning stack: the whole four-step pipeline completed and the final safety check report showed every one of its 9 checks passing.

Files changed:
- [scripts/operator/plan.sh](https://github.com/Janga786/rebot-crack-vision/blob/aa9dbb66481f34e775a0788022b662274c36c2b3/scripts/operator/plan.sh) (+159 / −0)
- [scripts/ros/test_operator_plan.sh](https://github.com/Janga786/rebot-crack-vision/blob/aa9dbb66481f34e775a0788022b662274c36c2b3/scripts/ros/test_operator_plan.sh) (+232 / −0)

Commits: [`aa9dbb6`](https://github.com/Janga786/rebot-crack-vision/commit/aa9dbb66481f34e775a0788022b662274c36c2b3)

## Why it was done

A case-scoped, resumable `plan` command that lifts a case's ordered pixel paths + capture record to a checked, retimed crack-trace trajectory, ready for preview -- with no execution step.

- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Operator workflow & handoff: **6/12** tasks accepted; whole project: **137/170**.
- REQ-OPS-1: 25/33 contributing tasks done
- Verification: automated checks run by the pipeline itself (e2e ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “plan.sh correctly sequences path3d -> plan_crack_path -> retime_plan -> trajectory_check at each tool's own default output paths, enforces the capture/paths precondition (exit 3) and freshness/--force skip logic via crackvision.operator_state, records all four outputs sha-bound on success, resets state on failure, and never touches preview/approval/execution. I independently reran scripts/ros/test_operator_plan.sh end-to-end (~8 min) and reproduced exit 0 with operator_state reporting the plan stage done and trajectory_check reporting all 9 checks pass, matching the schedul
…[92 chars clipped]”

## What it unlocks next

- Closer: OPS-01.8 — operator_cli.sh dispatcher: doctor/capture/segment/paths/plan/preview/execute, resumable 'run', end-to-end headless test + OPERATOR_WORKFLOW.md (still needs OPS-01.5, OPS-01.6)

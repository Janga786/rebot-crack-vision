# MOT-07.10 — Retime the view-plan transit leg and settle the §11.2 purpose for view moves

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`24b676e`](https://github.com/Janga786/rebot-crack-vision/commit/24b676ebb051ad523552483fcefa84d13a62e0f1)

## What was done

All sub-tasks of this work package were completed and audited together: MOT-07.10.1, MOT-07.10.1.R1, MOT-07.10.2, MOT-07.10.2.R1, MOT-07.10.2.R1.R1, MOT-07.10.2.R2, MOT-07.10.2.R2.R1, MOT-07.10.3, MOT-07.10.4, MOT-07.10.5.

## Why it was done

The view_plan transit leg (home → camera view pose) can be retimed into a gate-acceptable joint_trajectory per the resolution ADR-019 §16.8 records, with any §11.2/§11.7 purpose amendment implemented consistently in joint_trajectory, execution_gate, retime_plan and the checker.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **50/66** tasks accepted; whole project: **113/156**.
- REQ-MOT-2: 37/42 contributing tasks done
- REQ-OPS-1: 11/19 contributing tasks done
- Verification: independent audit accepted it (criteria: 2 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The MOT-07.10 branch outcome holds end to end. ADR-019's MOT-07.10 addendum settles §16.8 with a new purpose value, "view_move", rather than deferring to OPS-01. INTERFACES §17 is appended with explicit G-TRAJ and G-ELIGIBLE rules. joint_trajectory, execution_gate, retime_plan --view-plan and trajectory_check implement it consistently. Evidence I ran: - scripts/ros/test_view_retime_execute.sh passes. It chains view_plan → retime_plan --view-plan → trajectory_check (all C-* pass) → execute_trajectory: dry mode passes with G-TRAJ and G-ELIGIBLE both pass; mock mode completes 
…[920 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

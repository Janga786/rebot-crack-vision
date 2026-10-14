# MOT-07.10 — Retime the view-plan transit leg and settle the §11.2 purpose for view moves

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`25f9df1`](https://github.com/Janga786/rebot-crack-vision/commit/25f9df1e8699d36ace7bc9053c83f31358046dda)

## What was done

All sub-tasks of this work package were completed and audited together: MOT-07.10.1, MOT-07.10.1.R1, MOT-07.10.2, MOT-07.10.2.R1, MOT-07.10.2.R1.R1, MOT-07.10.2.R2, MOT-07.10.3, MOT-07.10.4, MOT-07.10.5.

## Why it was done

The view_plan transit leg (home → camera view pose) can be retimed into a gate-acceptable joint_trajectory per the resolution ADR-019 §16.8 records, with any §11.2/§11.7 purpose amendment implemented consistently in joint_trajectory, execution_gate, retime_plan and the checker.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **53/66** tasks accepted; whole project: **116/156**.
- REQ-MOT-2: 38/42 contributing tasks done
- REQ-OPS-1: 12/19 contributing tasks done
- Verification: independent audit accepted it (criteria: 2 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The MOT-07.10 branch does what it set out to do. ADR-019's addendum records the §16.8 decision: add a new purpose value, "view_move", rather than defer to OPS-01, with the alternatives and their rejection reasons written down. INTERFACES §17 is appended as the contract change, with explicit G-TRAJ and G-ELIGIBLE rules. The change is implemented consistently in joint_trajectory.load_trajectory, the execution_gate (G-TRAJ, plus a G-ELIGIBLE check split into an integrity step and an eligibility step), `retime_plan --view-plan` (one transit leg through the unchanged retime algo
…[920 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

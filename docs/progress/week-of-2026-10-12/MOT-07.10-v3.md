# MOT-07.10 — Retime the view-plan transit leg and settle the §11.2 purpose for view moves

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`e87d3b2`](https://github.com/Janga786/rebot-crack-vision/commit/e87d3b2f6d8f8585765176d11b1a864f3491ae86)

## What was done

All sub-tasks of this work package were completed and audited together: MOT-07.10.1, MOT-07.10.1.R1, MOT-07.10.2, MOT-07.10.2.R1, MOT-07.10.2.R2, MOT-07.10.3, MOT-07.10.4, MOT-07.10.5.

## Why it was done

The view_plan transit leg (home → camera view pose) can be retimed into a gate-acceptable joint_trajectory per the resolution ADR-019 §16.8 records, with any §11.2/§11.7 purpose amendment implemented consistently in joint_trajectory, execution_gate, retime_plan and the checker.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **50/66** tasks accepted; whole project: **112/156**.
- REQ-MOT-2: 35/42 contributing tasks done
- REQ-OPS-1: 8/19 contributing tasks done
- Verification: independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Integration audit of MOT-07.10 at e87d3b2 across its children (07.10.1–07.10.5 plus their repairs). ADR-019 §16.8 is settled by adding a new purpose value, not by deferring: the ADR-019 MOT-07.10 addendum and the appended INTERFACES §17 add purpose "view_move" with explicit G-TRAJ/G-ELIGIBLE rules (§17.2). The amendment is applied consistently in joint_trajectory.load_trajectory (§17.1 source shape), execution_gate (G-ELIGIBLE integrity then eligibility), retime_plan/retime (--view-plan runs the transit leg as a one-leg plan through the shared _retime_legs_doc; leg_index is
…[525 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

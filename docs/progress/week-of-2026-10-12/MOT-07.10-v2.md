# MOT-07.10 — Retime the view-plan transit leg and settle the §11.2 purpose for view moves

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`069b69f`](https://github.com/Janga786/rebot-crack-vision/commit/069b69f0ddc9d71e7ac4d776667b0a2003f61840)

## What was done

All sub-tasks of this work package were completed and audited together: MOT-07.10.1, MOT-07.10.1.R1, MOT-07.10.2, MOT-07.10.2.R1, MOT-07.10.2.R2, MOT-07.10.3, MOT-07.10.4, MOT-07.10.5.

## Why it was done

The view_plan transit leg (home → camera view pose) can be retimed into a gate-acceptable joint_trajectory per the resolution ADR-019 §16.8 records, with any §11.2/§11.7 purpose amendment implemented consistently in joint_trajectory, execution_gate, retime_plan and the checker.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **50/66** tasks accepted; whole project: **112/156**.
- REQ-MOT-2: 34/42 contributing tasks done
- REQ-OPS-1: 7/19 contributing tasks done
- Verification: independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Integration audit of MOT-07.10 at 069b69f. ADR-019 §16.8 was decided in favour of a new purpose value rather than deferral. The ADR-019 MOT-07.10 addendum records that decision, and the appended INTERFACES §17 adds `purpose: "view_move"` with explicit G-TRAJ and G-ELIGIBLE rules (§17.2). The amendment is implemented consistently in all four places: joint_trajectory.load_trajectory, execution_gate (G-TRAJ passes when the schema is valid; G-ELIGIBLE runs integrity checks, then sha-binding checks), retime_plan --view-plan (a one-leg plan through the shared retime algorithm) an
…[835 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

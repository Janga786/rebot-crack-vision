# MOT-07.10 — Retime the view-plan transit leg and settle the §11.2 purpose for view moves

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`8253078`](https://github.com/Janga786/rebot-crack-vision/commit/8253078079f77bc2a495ce4a53ffb4e8762ae92b)

## What was done

All sub-tasks of this work package were completed and audited together: MOT-07.10.1, MOT-07.10.1.R1, MOT-07.10.2, MOT-07.10.2.R1, MOT-07.10.2.R1.R1, MOT-07.10.2.R2, MOT-07.10.2.R2.R1, MOT-07.10.3, MOT-07.10.4, MOT-07.10.5.

## Why it was done

The view_plan transit leg (home → camera view pose) can be retimed into a gate-acceptable joint_trajectory per the resolution ADR-019 §16.8 records, with any §11.2/§11.7 purpose amendment implemented consistently in joint_trajectory, execution_gate, retime_plan and the checker.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **51/67** tasks accepted; whole project: **118/170**.
- REQ-MOT-2: 36/42 contributing tasks done
- REQ-OPS-1: 10/33 contributing tasks done
- Verification: independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The branch delivers what the card asks for, end to end. ADR-019's MOT-07.10 addendum and INTERFACES §17 settle §16.8 by adding purpose "view_move", with explicit G-TRAJ and G-ELIGIBLE rules. §11.2, §11.7 and §16.8 each point forward to §17, so the amendment is appended rather than rewritten. The purpose is implemented consistently in four places: joint_trajectory.load_trajectory, execution_gate (G-TRAJ, plus the view_move integrity and eligibility branch of G-ELIGIBLE), retime_plan --view-plan (which reuses the shared §16.2 body through _retime_legs_doc) and trajectory_chec
…[446 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

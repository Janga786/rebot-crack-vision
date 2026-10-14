# INT-01.3 — Document the one-command synthetic-to-limits-checked-mock-execution integration test

**Accepted:** 2026-10-14 · **Work package:** Integration & commissioning (L-INT) · **Track:** simulation · **Verified commit:** [`72a92c2`](https://github.com/Janga786/rebot-crack-vision/commit/72a92c2a027c495f8ee2d68eab0e24d494b2c648)

## What was done

Added docs/motion/INTEGRATION_E2E.md, which documents and records a real run of the one-command script that chains synthetic crack perception through MoveIt-mock planning, retiming, limits checking and a dry/mock execution. The verified run produced a fully-covered 36/36 waypoint crack path, a retimed 13.9 s / 12,361-point trajectory with zero self-check violations, and a clean mock execution that arrived at the final trajectory point exactly (zero error), all with the expected "not yet eligible for real execution" and nominal-geometry caveats stated honestly.</presentation_summary>
</invoke>

Files changed:
- [docs/motion/INTEGRATION_E2E.md](https://github.com/Janga786/rebot-crack-vision/blob/72a92c2a027c495f8ee2d68eab0e24d494b2c648/docs/motion/INTEGRATION_E2E.md) (+95 / −0)
- [docs/motion/ROS_WORKSPACE.md](https://github.com/Janga786/rebot-crack-vision/blob/72a92c2a027c495f8ee2d68eab0e24d494b2c648/docs/motion/ROS_WORKSPACE.md) (+8 / −0)

Commits: [`72a92c2`](https://github.com/Janga786/rebot-crack-vision/commit/72a92c2a027c495f8ee2d68eab0e24d494b2c648)

## Why it was done

docs/motion/INTEGRATION_E2E.md documents scripts/ros/run_integration_e2e.sh's method, the measured results of one real run, and its honest caveats, in the style of CARTESIAN_PLANNING.md/TIME_PARAMETERIZATION.md.

- Requirement **REQ-INT-1**: Simulation and recorded-data integration tests of the full pipeline

## How it moves the project forward

- Integration & commissioning: **4/9** tasks accepted; whole project: **112/157**.
- REQ-INT-1: 6/11 contributing tasks done
- Verification: automated checks run by the pipeline itself (docs ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “docs/motion/INTEGRATION_E2E.md documents the one-command integration test with a real, verifiable run; headline numbers in its table match the actual scratch logs (skeleton_pixels=105, candidates_tried=24, coverage 36/36 fraction 1.0, retime duration_s=13.872/n_points=12361/peak_acceleration_fraction up to 0.950, trajectory_check 9/9 pass, dry-mode gate outcomes and would_refuse_in_real list, mock-mode exact arrival). Caveats are stated honestly (nominal geometry, execution_eligible=false by design, mock/dry only, single scenario, view-plan leg out of scope). ROS_WORKSPACE.
…[87 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

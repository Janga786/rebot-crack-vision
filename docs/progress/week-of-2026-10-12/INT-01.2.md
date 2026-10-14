# INT-01.2 — One command: synthetic perception through MoveIt-mock crack planning, retiming and a limits-checked mock execution

**Accepted:** 2026-10-14 · **Work package:** Integration & commissioning (L-INT) · **Track:** simulation · **Verified commit:** [`e6e6e3d`](https://github.com/Janga786/rebot-crack-vision/commit/e6e6e3dab75b5a38e363fb638c43c823e08d1f90)

## What was done

One command (scripts/ros/run_integration_e2e.sh) now runs the full simulated pipeline: a synthetic crack image goes through segmentation, skeleton, paths and 3D, then MoveIt-mock crack planning with full coverage and retiming, passes all 9 trajectory checks, and finishes with a dry run and a mock execution. In three local runs it ended at the trajectory's final joint positions (max error 0.0 rad) and left no stray ROS processes. An intermittent ROS service timeout found during testing is now detected and retried once, with a logged warning.

Files changed:
- [scripts/ros/run_integration_e2e.sh](https://github.com/Janga786/rebot-crack-vision/blob/e6e6e3dab75b5a38e363fb638c43c823e08d1f90/scripts/ros/run_integration_e2e.sh) (+563 / −12)

Commits: [`cbc6ffb`](https://github.com/Janga786/rebot-crack-vision/commit/cbc6ffbc534f8ea408e20a7fd4de3b01c34ea2bc), [`e6e6e3d`](https://github.com/Janga786/rebot-crack-vision/commit/e6e6e3dab75b5a38e363fb638c43c823e08d1f90)

## Why it was done

scripts/ros/run_integration_e2e.sh chains INT-01.1's script into plan_view -> plan_crack_path -> retime_plan -> crackvision.trajectory_check -> execute_trajectory --mode dry and --mode mock on the headless moveit_mock stack, as one command, with full coverage, all limits/retime checks passing and arrival verified.

- Requirement **REQ-INT-1**: Simulation and recorded-data integration tests of the full pipeline

## How it moves the project forward

- Integration & commissioning: **3/9** tasks accepted; whole project: **122/156**.
- REQ-INT-1: 7/11 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, e2e ✔); independent audit accepted it (criteria: 10 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “run_integration_e2e.sh correctly chains the real INT-01.1 perception script into MoveIt-mock crack planning, retiming, trajectory_check and a limits-checked dry+mock execution. I independently re-ran both acceptance checks (build_ws.sh and run_integration_e2e.sh) and reproduced exit 0 with coverage.complete=true, all 9 trajectory_check outcomes pass, dry would_refuse_in_real containing G-ELIGIBLE/G-APPROVAL, and mock outcome=completed with G-LIMITS/G-STALE-CONFIG/G-COLLISION all pass and arrival max|dq|=0.0. No stray mock-stack processes remained afterward and only scripts/
…[39 chars clipped]”

## What it unlocks next

- **Ready to start:** INT-01.3 — Document the one-command synthetic-to-limits-checked-mock-execution integration test

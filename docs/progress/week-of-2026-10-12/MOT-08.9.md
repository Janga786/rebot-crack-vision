# MOT-08.9 — Document the preview → approval → execution workflow (PREVIEW_APPROVAL.md, EXECUTION.md, ROS_WORKSPACE.md)

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`935247e`](https://github.com/Janga786/rebot-crack-vision/commit/935247e3c5bb607029bdd6089c24d7abefe18f51)

## What was done

Added an operator runbook (docs/motion/PREVIEW_APPROVAL.md) walking the preview-and-approve workflow for trajectories before any dry/real execution, and cross-linked it from the existing EXECUTION.md and ROS_WORKSPACE.md docs. Verified the scheduler's required grep-based acceptance check passes against the new/edited files.

Files changed:
- [docs/motion/EXECUTION.md](https://github.com/Janga786/rebot-crack-vision/blob/935247e3c5bb607029bdd6089c24d7abefe18f51/docs/motion/EXECUTION.md) (+3 / −2)
- [docs/motion/PREVIEW_APPROVAL.md](https://github.com/Janga786/rebot-crack-vision/blob/935247e3c5bb607029bdd6089c24d7abefe18f51/docs/motion/PREVIEW_APPROVAL.md) (+163 / −0)
- [docs/motion/ROS_WORKSPACE.md](https://github.com/Janga786/rebot-crack-vision/blob/935247e3c5bb607029bdd6089c24d7abefe18f51/docs/motion/ROS_WORKSPACE.md) (+10 / −0)

Commits: [`935247e`](https://github.com/Janga786/rebot-crack-vision/commit/935247e3c5bb607029bdd6089c24d7abefe18f51)

## Why it was done

docs/motion/PREVIEW_APPROVAL.md gives the operator the exact commands to preview, inspect (PNGs and optional RViz), approve and then dry-run a trajectory. EXECUTION.md's G-APPROVAL row and approval bullet reflect §18, and ROS_WORKSPACE.md lists the new node and launch file.

- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery
- Requirement **REQ-OPS-2**: Reproducible install/config/tests/evaluation results, handoff docs, requirements→evidence map, honest readiness report

## How it moves the project forward

- Motion planning: **50/67** tasks accepted; whole project: **113/157**.
- REQ-OPS-1: 15/20 contributing tasks done
- REQ-OPS-2: 6/10 contributing tasks done
- Verification: automated checks run by the pipeline itself (docs ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “PREVIEW_APPROVAL.md accurately documents the preview→approve→execute workflow: every CLI flag listed for trajectory_check, trajectory_preview, approve_trajectory and execute_trajectory matches the actual argparse definitions (verified against src/crackvision/trajectory_preview.py, src/crackvision/approve_trajectory.py, config.py's add_common_args, and ros2_ws execute_trajectory.py), and the sequence matches scripts/ros/test_preview_approve.sh and test_preview_rviz.sh. The PNG inspection checklist, approval certification scope/expiry/invalidation language, and RViz no-Plan/E
…[336 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

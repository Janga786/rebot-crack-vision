# MOT-07.10.4 — trajectory_check: accept a view_plan and check view_move trajectories independently

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`c7c5476`](https://github.com/Janga786/rebot-crack-vision/commit/c7c5476c73e9e28255e4d2e178ab616c63939ae6)

## What was done

trajectory_check now also validates view-move trajectories against a view_plan file, running the same path/limits/report checks used for crack-task trajectories but over the single transit-leg polyline. Added 24 new automated tests covering a clean view-move trajectory (passes every check) plus deliberately broken cases (wrong plan hash, off-path point, overspeed sample, wrong report range), all verified passing (86/86 across the full checker test suite) with the checker still importing nothing from the ROS motion package.

Files changed:
- [src/crackvision/trajectory_check.py](https://github.com/Janga786/rebot-crack-vision/blob/c7c5476c73e9e28255e4d2e178ab616c63939ae6/src/crackvision/trajectory_check.py) (+240 / −18)
- [tests/test_trajectory_check_view.py](https://github.com/Janga786/rebot-crack-vision/blob/c7c5476c73e9e28255e4d2e178ab616c63939ae6/tests/test_trajectory_check_view.py) (+473 / −0)

Commits: [`c7c5476`](https://github.com/Janga786/rebot-crack-vision/commit/c7c5476c73e9e28255e4d2e178ab616c63939ae6)

## Why it was done

`./env.sh python -m crackvision.trajectory_check --trajectory T --plan view_plan.json` runs every §16.5 check on a view_move trajectory as amended by §17.4. It stays independent of crackvision_motion.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **45/57** tasks accepted; whole project: **107/144**.
- REQ-MOT-2: 29/38 contributing tasks done
- REQ-OPS-1: 5/11 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, independent ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The change does what §17.4 asks. Both scheduler acceptance checks pass when I re-ran them: 86 tests passed, and no line imports crackvision_motion or rclpy. I ran the CLI end to end on hand-built view plans and trajectories. A good view_move trajectory exits 0, and C-TOOLSPEED passes with numbers {trace_legs: []}. Each required failure exits 3 with the right check failing: wrong report point_range → C-REPORT, view_move against a cartesian_plan → C-PROVENANCE, crack_task against a view_plan → C-PROVENANCE, off-path point → C-PATH, overspeed sample → C-LIMITS. An execution_el
…[316 chars clipped]”

## What it unlocks next

- **Ready to start:** MOT-07.10.5 — Headless mock run: view_plan → retime_plan --view-plan → trajectory_check → execute_trajectory dry/mock

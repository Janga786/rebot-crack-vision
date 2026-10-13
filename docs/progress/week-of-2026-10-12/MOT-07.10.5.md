# MOT-07.10.5 — Headless mock run: view_plan → retime_plan --view-plan → trajectory_check → execute_trajectory dry/mock

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`7738a2f`](https://github.com/Janga786/rebot-crack-vision/commit/7738a2fb82a0e7e1f69e02ceba4b311b7253587a)

## What was done

Wrote scripts/ros/test_view_retime_execute.sh (modelled on test_retime_execute.sh) proving the headless mock chain view_plan -> retime_plan --view-plan -> trajectory_check -> execute_trajectory (dry, mock, real-refusal, and tampered-sha G-ELIGIBLE refusal); added a new "View move" section to docs/motion/TIME_PARAMETERIZATION.md documenting §17.

Files changed:
- [docs/motion/TIME_PARAMETERIZATION.md](https://github.com/Janga786/rebot-crack-vision/blob/7738a2fb82a0e7e1f69e02ceba4b311b7253587a/docs/motion/TIME_PARAMETERIZATION.md) (+46 / −0)
- [scripts/ros/test_view_retime_execute.sh](https://github.com/Janga786/rebot-crack-vision/blob/7738a2fb82a0e7e1f69e02ceba4b311b7253587a/scripts/ros/test_view_retime_execute.sh) (+337 / −0)

Commits: [`7738a2f`](https://github.com/Janga786/rebot-crack-vision/commit/7738a2fb82a0e7e1f69e02ceba4b311b7253587a)

## Why it was done

scripts/ros/test_view_retime_execute.sh proves, on the mock stack only, that the synthetic view plan's transit leg can be retimed, independently checked and executed. It also shows the real-only refusals the view move still gets, and TIME_PARAMETERIZATION.md documents the view-move workflow.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **48/57** tasks accepted; whole project: **110/144**.
- REQ-MOT-2: 33/38 contributing tasks done
- REQ-OPS-1: 6/11 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, view-mock-run ✔, crosscheck-regression ✔, docs ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this card. I re-ran scripts/ros/test_view_retime_execute.sh myself and it exits 0, matching the scheduler's evidence; the other three scheduler checks (build, crosscheck-regression, docs) also passed. The full chain works on the mock stack: - retime_plan --view-plan produces one transit leg (3.948 s, 4141 points). - trajectory_check passes all 9 checks; C-TOOLSPEED is the expected vacuous pass. - Dry mode exits 0 with G-TRAJ and G-ELIGIBLE both pass. G-START-STATE passes without any pre-positioning move. - Mock mode completes and the arm ends at the final poin
…[419 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

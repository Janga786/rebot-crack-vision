# MOT-07.10.3 — retime_plan --view-plan: retime the view_plan transit leg into a view_move trajectory

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`12dd7a8`](https://github.com/Janga786/rebot-crack-vision/commit/12dd7a89b5cd384795be3600738dd369a33110d5)

## What was done

Implemented retime_plan --view-plan (§17.3): factored retime.retime_plan_doc's shared §16.2-16.4 body into a private _retime_legs_doc helper, added retime_view_plan_doc on top of it, and updated retime_plan.py's CLI with a required mutually-exclusive --plan/--view-plan group, --purpose defaulting to None (usage error with --view-plan), load_view_plan-based loading, and the data/trajectories/view_trajectory.json default output. Added test_retime_view.py covering the happy path, dry-run, default --out, both usage-error cases, every exit-3 refusal (invalid view plan, null transit_leg, limits/end_effector sha mismatches), and execution_gate's offline G-TRAJ/G-LIMITS/G-ELIGIBLE acceptance in mock mode.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/retime.py](https://github.com/Janga786/rebot-crack-vision/blob/12dd7a89b5cd384795be3600738dd369a33110d5/ros2_ws/src/crackvision_motion/crackvision_motion/retime.py) (+77 / −10)
- [ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py](https://github.com/Janga786/rebot-crack-vision/blob/12dd7a89b5cd384795be3600738dd369a33110d5/ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py) (+70 / −25)
- [ros2_ws/src/crackvision_motion/test/test_retime_view.py](https://github.com/Janga786/rebot-crack-vision/blob/12dd7a89b5cd384795be3600738dd369a33110d5/ros2_ws/src/crackvision_motion/test/test_retime_view.py) (+269 / −0)

Commits: [`12dd7a8`](https://github.com/Janga786/rebot-crack-vision/commit/12dd7a89b5cd384795be3600738dd369a33110d5)

## Why it was done

`retime_plan --view-plan data/plans/view_plan.json` writes a §17.3 view_move joint_trajectory plus a .retime.json sidecar from the view plan's transit_leg, through the existing §16.2 algorithm, and the result loads with joint_trajectory and passes G-LIMITS offline.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **43/57** tasks accepted; whole project: **105/144**.
- REQ-MOT-2: 28/38 contributing tasks done
- REQ-OPS-1: 4/11 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, help ✔, ros-free ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this card. retime_view_plan_doc treats transit_leg as a one-leg plan and runs it through a shared private helper, _retime_legs_doc, which was factored out of retime_plan_doc rather than copied, so the algorithm is not duplicated. The §16.3 sha refusals now read from the view plan's sources, and the trajectory gets purpose view_move with a source shaped per §17.1. leg_index is carried through unchanged from transit_leg, and report.input_shas.plan is the view-plan file's sha. The CLI rules match §17.3. The tests cover the happy path (bit-exact endpoints, a valid
…[239 chars clipped]”

## What it unlocks next

- Closer: MOT-07.10.5 — Headless mock run: view_plan → retime_plan --view-plan → trajectory_check → execute_trajectory dry/mock (still needs MOT-07.10.4)

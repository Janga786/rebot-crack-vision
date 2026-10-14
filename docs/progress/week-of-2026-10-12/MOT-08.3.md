# MOT-08.3 — crackvision.trajectory_preview CLI: deterministic headless PNG render (joints, tool path + crack + scene, tool speed) and manifest

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`2ea7f1d`](https://github.com/Janga786/rebot-crack-vision/commit/2ea7f1df1bbf41f9d881a3b29fe412c4a6038aec)

## What was done

Built the trajectory_preview CLI that turns a planned robot trajectory into three PNG charts (joint motion, 3D tool path with the scene and crack overlaid, and tool speed) plus a JSON summary, so an operator can visually review a move before approving it. Verified it renders identical images on repeat runs with the same inputs, refuses cleanly when its inputs don't match, and still renders (with a red failure banner) when the trajectory's own safety checks failed — all 5 new tests plus the 19 existing dependency tests pass.

Files changed:
- [src/crackvision/trajectory_preview.py](https://github.com/Janga786/rebot-crack-vision/blob/2ea7f1df1bbf41f9d881a3b29fe412c4a6038aec/src/crackvision/trajectory_preview.py) (+554 / −0)
- [tests/test_trajectory_preview.py](https://github.com/Janga786/rebot-crack-vision/blob/2ea7f1df1bbf41f9d881a3b29fe412c4a6038aec/tests/test_trajectory_preview.py) (+279 / −0)

Commits: [`2ea7f1d`](https://github.com/Janga786/rebot-crack-vision/commit/2ea7f1df1bbf41f9d881a3b29fe412c4a6038aec)

## Why it was done

`./env.sh python -m crackvision.trajectory_preview` writes data/previews/{case}_{sha8}/ with joints.png, toolpath.png, toolspeed.png and a valid preview.json. It works headless and renders byte-identically when re-run.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **56/66** tasks accepted; whole project: **119/156**.
- REQ-MOT-2: 39/42 contributing tasks done
- REQ-OPS-1: 11/19 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, help ✔, independence ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “trajectory_preview.py implements the §18 CLI correctly: common flags plus the required §18 flags with correct defaults, correct exit-code mapping (3 for input refusal, 1 for internal manifest-validation/render failure, 2 for config errors, 0 otherwise), Agg forced before pyplot import, no plt.show, no crackvision_motion/rclpy imports. joints/toolpath/toolspeed PNGs match the card's richer content spec (q/v/a with limit lines and leg shading; XY/XZ/YZ tool path with scene boxes, crack polylines gated on sha match, start/end markers; unscaled+scaled speed with v_c bands), red
…[438 chars clipped]”

## What it unlocks next

- Closer: MOT-08.8 — Headless mock run: view plan → retime → check → preview → approve (pty) → execute_trajectory dry with G-APPROVAL pass, tamper refusals (still needs MOT-08.4, MOT-08.5, MOT-08.7)

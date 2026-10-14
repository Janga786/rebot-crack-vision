# MOT-08.8 — Headless mock run: view plan → retime → check → preview → approve (pty) → execute_trajectory dry with G-APPROVAL pass, tamper refusals

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`f48851b`](https://github.com/Janga786/rebot-crack-vision/commit/f48851b8bc321ce77f9cf9f4617d753a9cd5f1c7)

## What was done

The headless mock test for the view→preview→approve→execute workflow (test_preview_approve.sh) now runs end-to-end successfully: it builds, previews and gets a typed pseudo-terminal approval for a synthetic view-move trajectory, then verifies execute_trajectory reports the approval gate as passing in dry mode, and correctly flags tampered or mismatched approvals as warnings. This was previously blocked by a terminal-handling bug in the approval tool, which an earlier task (MOT-08.10) has since fixed, so this task required no further code changes — just re-verification, which succeeded (all three acceptance checks passed with exit code 0).

Files changed:
- [scripts/ros/test_preview_approve.sh](https://github.com/Janga786/rebot-crack-vision/blob/f48851b8bc321ce77f9cf9f4617d753a9cd5f1c7/scripts/ros/test_preview_approve.sh) (+363 / −0)

Commits: [`04308af`](https://github.com/Janga786/rebot-crack-vision/commit/04308af819a69683365ede27696c51641970a2d9)

## Why it was done

scripts/ros/test_preview_approve.sh proves on the mock stack only that a retimed view-move trajectory can be previewed and approved, and that execute_trajectory then reports G-APPROVAL pass. It also shows that tampering with the preview, approving a different trajectory, or running real without arming is refused or warned per §18.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **61/67** tasks accepted; whole project: **127/157**.
- REQ-MOT-2: 40/42 contributing tasks done
- REQ-OPS-1: 15/20 contributing tasks done
- REQ-INT-2: 16/21 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, preview-approve ✔, view-regression ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this card. `scripts/ros/test_preview_approve.sh` (commit 04308af) does what the card asks. I re-ran it at f48851b and it exited 0, writing everything under `logs/`. Nothing in `data/` or `config/` changed (checked with `find -newer` against a marker file), and the working tree is still clean. I opened the execution records to make sure each G-APPROVAL warn fires for the intended reason, not by accident. With the genuine approval, dry mode gives pass. The tampered copy warns at §18.3 check 2, naming `preview_tampered/toolpath.png` and its sha mismatch. Changing
…[501 chars clipped]”

## What it unlocks next

- **Ready to start:** MOT-08.9 — Document the preview → approval → execution workflow (PREVIEW_APPROVAL.md, EXECUTION.md, ROS_WORKSPACE.md)

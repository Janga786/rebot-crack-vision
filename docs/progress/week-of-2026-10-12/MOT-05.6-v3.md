# MOT-05.6 — Operator execution runbook (mock → dry → real) + ROS_WORKSPACE.md entry

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`7fcf949`](https://github.com/Janga786/rebot-crack-vision/commit/7fcf949ac721a12fb45a5f2126d798281d0a3be0)

## What was done

Added the operator runbook (docs/motion/EXECUTION.md) that tells anyone running the robot executor exactly how to go from a safe mock test to a real-hardware run, including why real mode is currently refused, how to use the emergency stop, and what to do if something goes wrong. Verified both automated checks pass: the doc contains every required term and every safety-gate name the code actually defines. Also added a short pointer to this new doc from the existing workspace documentation, without changing any of its prior content.

After the first audit, a repair round (MOT-05.6.R1) fixed the reported issues: Updated the operator-facing execution runbook so its gate-documentation table matches the code: the G-APPROVAL entry now explains the extra preview-artifact checks (exactly one verified, sha-bound, passing preview manifest) and gives operators a distinct fix-it step when that binding — rather than the approval's age or sha — is what's broken. Both automated documentation checks pass with exit code 0.

Files changed:
- [docs/motion/EXECUTION.md](https://github.com/Janga786/rebot-crack-vision/blob/7fcf949ac721a12fb45a5f2126d798281d0a3be0/docs/motion/EXECUTION.md) (+253 / −3)
- [docs/motion/ROS_WORKSPACE.md](https://github.com/Janga786/rebot-crack-vision/blob/7fcf949ac721a12fb45a5f2126d798281d0a3be0/docs/motion/ROS_WORKSPACE.md) (+11 / −0)

Commits: [`7749239`](https://github.com/Janga786/rebot-crack-vision/commit/77492395d0a0c443c2f2ab11657a428989f4e0da), [`92e9fa9`](https://github.com/Janga786/rebot-crack-vision/commit/92e9fa9d4db0647975b9b5e52e5bcabce766002f), [`7fcf949`](https://github.com/Janga786/rebot-crack-vision/commit/7fcf949ac721a12fb45a5f2126d798281d0a3be0)

## Why it was done

docs/motion/EXECUTION.md is the operator-facing procedure for the commissioning-gated executor. It covers prerequisites per gate id, how to produce each record (commissioning, approval), the mock→dry→real progression, the typed confirmation, e-stop use (hardware first), reading the execution record and failure recovery. docs/motion/ROS_WORKSPACE.md points to it and to test_execution.sh. MOT-09 and OPS-01 can use it without reading code.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **52/66** tasks accepted; whole project: **115/156**.
- REQ-MOT-1: 34/45 contributing tasks done
- REQ-INT-2: 10/20 contributing tasks done
- Verification: automated checks run by the pipeline itself (doc-present ✔, gate-ids-documented ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Commit 7fcf949 changes only docs/motion/EXECUTION.md, which is within scope. The rewritten G-APPROVAL row (line 172) and the §3 bullet (lines 40-45) match the code in execution_gate.py:780-876. The preview-binding checks run only after the sha and age checks pass. The row lists all five binding checks: non-empty preview_artifacts, per-entry sha256 on disk, exactly one trajectory_preview/1 manifest, matching trajectory.sha256, and check_report.all_pass is true. It states that these warn in mock/dry and refuse in real. The remedy for a broken preview binding is separate from 
…[288 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task MOT-05.6.R1): G-APPROVAL row in EXECUTION.md is stale for the §18.3 preview-artifact checks; G-ELIGIBLE row in EXECUTION.md is stale for purpose:"view_move" trajectories.

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

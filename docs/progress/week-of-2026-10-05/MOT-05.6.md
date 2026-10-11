# MOT-05.6 — Operator execution runbook (mock → dry → real) + ROS_WORKSPACE.md entry

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`7749239`](https://github.com/Janga786/rebot-crack-vision/commit/77492395d0a0c443c2f2ab11657a428989f4e0da)

## What was done

Added the operator runbook (docs/motion/EXECUTION.md) that tells anyone running the robot executor exactly how to go from a safe mock test to a real-hardware run, including why real mode is currently refused, how to use the emergency stop, and what to do if something goes wrong. Verified both automated checks pass: the doc contains every required term and every safety-gate name the code actually defines. Also added a short pointer to this new doc from the existing workspace documentation, without changing any of its prior content.

Files changed:
- [docs/motion/EXECUTION.md](https://github.com/Janga786/rebot-crack-vision/blob/77492395d0a0c443c2f2ab11657a428989f4e0da/docs/motion/EXECUTION.md) (+245 / −0)
- [docs/motion/ROS_WORKSPACE.md](https://github.com/Janga786/rebot-crack-vision/blob/77492395d0a0c443c2f2ab11657a428989f4e0da/docs/motion/ROS_WORKSPACE.md) (+11 / −0)

Commits: [`7749239`](https://github.com/Janga786/rebot-crack-vision/commit/77492395d0a0c443c2f2ab11657a428989f4e0da)

## Why it was done

docs/motion/EXECUTION.md is the operator-facing procedure for the commissioning-gated executor. It covers prerequisites per gate id, how to produce each record (commissioning, approval), the mock→dry→real progression, the typed confirmation, e-stop use (hardware first), reading the execution record and failure recovery. docs/motion/ROS_WORKSPACE.md points to it and to test_execution.sh. MOT-09 and OPS-01 can use it without reading code.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **27/41** tasks accepted; whole project: **88/128**.
- REQ-MOT-1: 22/34 contributing tasks done
- REQ-INT-2: 10/16 contributing tasks done
- Verification: automated checks run by the pipeline itself (doc-present ✔, gate-ids-documented ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this card. The runbook matches ADR-016, INTERFACES §11, the executor code and the shipped configs, and it covers every acceptance criterion. I re-checked all 12 gate ids defined in execution_gate.py and the 4 defined in execute_trajectory.py: each one has a full row in the gate table. These facts also match the code and configs: the default mode is dry, the e-stop topics, `approval_max_age_s`, where records are written, the `--dry-run` behaviour, the /dev/tty confirmation and the post-cancel "PRESS THE HARDWARE E-STOP" message. The vendor-driver facts match VE
…[240 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

# INT-03.4 — Stale-calibration-config and limit-violation safety evidence

**Accepted:** 2026-10-14 · **Work package:** Integration & commissioning (L-INT) · **Track:** simulation · **Verified commit:** [`f78ddb5`](https://github.com/Janga786/rebot-crack-vision/commit/f78ddb5658614bb6236b545011949ae7ecc300f7)

## What was done

Documented how the system already safely refuses robot motion when calibration configs go stale or a trajectory exceeds joint/velocity/acceleration limits, backed by a fresh end-to-end test run. Re-ran the existing mock execution test suite (scripts/ros/test_execution.sh) and confirmed exit code 0 overall, with the stale-calibration fixture correctly refusing (exit 3) in real mode and the overspeed fixture correctly refusing (exit 3) via the independent limits check alone. No new code was added; this card only produces the written safety evidence.

Files changed:
- [docs/integration/safety/STALE_CALIBRATION_AND_LIMITS.md](https://github.com/Janga786/rebot-crack-vision/blob/f78ddb5658614bb6236b545011949ae7ecc300f7/docs/integration/safety/STALE_CALIBRATION_AND_LIMITS.md) (+92 / −0)

Commits: [`f78ddb5`](https://github.com/Janga786/rebot-crack-vision/commit/f78ddb5658614bb6236b545011949ae7ecc300f7)

## Why it was done

docs/integration/safety/STALE_CALIBRATION_AND_LIMITS.md documents, with a fresh passing run as evidence, how G-STALE-CONFIG and G-LIMITS already give safe refusals end-to-end through execute_trajectory, citing the existing MOT-05.5/MOT-07.8 mechanisms rather than re-implementing them.

- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Integration & commissioning: **9/14** tasks accepted; whole project: **135/170**.
- REQ-INT-2: 20/26 contributing tasks done
- REQ-OPS-1: 22/33 contributing tasks done
- Verification: automated checks run by the pipeline itself (stale-and-limits ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The doc accurately documents G-STALE-CONFIG and G-LIMITS behavior with correct code/line citations and an accurate §11.7 table quote; a fresh re-run of scripts/ros/test_execution.sh reproduced the exact exit codes and refusal-id log lines quoted in the doc. No new test or library code was added, matching scope.”

## What it unlocks next

- **Ready to start:** INT-03.5 — Assemble the safety case + unified failure-injection runner

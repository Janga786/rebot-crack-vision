# INT-03.5 — Assemble the safety case + unified failure-injection runner

**Accepted:** 2026-10-14 · **Work package:** Integration & commissioning (L-INT) · **Track:** simulation · **Verified commit:** [`71da678`](https://github.com/Janga786/rebot-crack-vision/commit/71da678e45344be8805beceb6942f6314dd2566b)

## What was done

Built a single-command safety-case runner (scripts/safety/run_safety_case.sh) that exercises all five mock/dry fault classes — empty mask, invalid depth, unreachable path, stale calibration/config, and limit violations — and a SAFETY_CASE.md document tracing each to its refusal mechanism and evidence. Ran the full suite end-to-end: all four stages passed and the command exits 0, confirming the safety case reproduces cleanly from a single operator command.

Files changed:
- [docs/integration/SAFETY_CASE.md](https://github.com/Janga786/rebot-crack-vision/blob/71da678e45344be8805beceb6942f6314dd2566b/docs/integration/SAFETY_CASE.md) (+62 / −0)
- [scripts/safety/run_safety_case.sh](https://github.com/Janga786/rebot-crack-vision/blob/71da678e45344be8805beceb6942f6314dd2566b/scripts/safety/run_safety_case.sh) (+74 / −0)

Commits: [`71da678`](https://github.com/Janga786/rebot-crack-vision/commit/71da678e45344be8805beceb6942f6314dd2566b)

## Why it was done

docs/integration/SAFETY_CASE.md ties the five fault classes (invalid depth, empty mask, unreachable path, stale calibration/config, limit violations) to their refusal mechanisms and evidence, and scripts/safety/run_safety_case.sh runs the whole suite in one command.

- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Integration & commissioning: **10/14** tasks accepted; whole project: **139/170**.
- REQ-INT-2: 21/26 contributing tasks done
- REQ-OPS-1: 26/33 contributing tasks done
- Verification: automated checks run by the pipeline itself (safety-case ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “INT-03.5 delivers exactly what the card scoped: a thin orchestrator script (scripts/safety/run_safety_case.sh) running the four required fault-injection scripts in order with correct failure handling and PASS/FAIL summary, and a SAFETY_CASE.md doc with one accurate table row per fault class, correct gate-id and exit-code cross-references, explicit REQ-INT-2/REQ-OPS-1 traceability, and a clear mock/dry scope boundary naming the correct operator cards for hardware commissioning. Live re-run of `bash scripts/safety/run_safety_case.sh` reproduced exit 0 with all four stages pas
…[54 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

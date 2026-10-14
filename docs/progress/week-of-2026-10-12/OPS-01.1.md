# OPS-01.1 — Decision + contract: operator CLI stage model, run-state schema, resumability (ADR-021, INTERFACES §19)

**Accepted:** 2026-10-14 · **Work package:** Operator workflow & handoff (L-OPS) · **Track:** software · **Verified commit:** [`f854b7d`](https://github.com/Janga786/rebot-crack-vision/commit/f854b7d812d2cd833fe074e47f18658c573ba995)

## What was done

Wrote ADR-021 (operator CLI workflow decision) and appended docs/INTERFACES.md §19, fixing the doctor/capture/segment/paths/plan/preview/execute stage order, the crackvision.operator_run/1 run-state schema, the mechanical cascade-invalidation freshness rule, the no-gate-bypass rule (including the capture-time view move), and the ROS-vs-./env.sh breakdown per stage.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/f854b7d812d2cd833fe074e47f18658c573ba995/docs/INTERFACES.md) (+157 / −0)
- [docs/adr/021-operator-cli-workflow.md](https://github.com/Janga786/rebot-crack-vision/blob/f854b7d812d2cd833fe074e47f18658c573ba995/docs/adr/021-operator-cli-workflow.md) (+119 / −0)

Commits: [`f854b7d`](https://github.com/Janga786/rebot-crack-vision/commit/f854b7d812d2cd833fe074e47f18658c573ba995)

## Why it was done

A written, normative contract (new ADR + INTERFACES.md §19) that fixes the operator workflow's stage list/order, the per-case run-state file schema, the resumability/re-entry rule, and the rule that any trajectory execution (capture's view move or the final crack-trace execute) always goes through retime_plan -> trajectory_check -> trajectory_preview -> approve_trajectory -> execute_trajectory in that order with no OPS-01 code path ever skipping a gate.

- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Operator workflow & handoff: **2/12** tasks accepted; whole project: **96/170**.
- REQ-OPS-1: 6/33 contributing tasks done
- Verification: automated checks run by the pipeline itself (section-present ✔, adr-present ✔); independent audit accepted it (criteria: 8 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “ADR-021 and INTERFACES.md §19 are a well-formed, purely additive contract that meets every acceptance criterion: correct stage list/order with concrete delegation targets, the operator_run/1 schema with per-stage status/outputs/depends_on/notes, a mechanical cascade-invalidation freshness rule with the required worked example, an explicit no-gate-bypass rule covering both the final trajectory and the capture-time view move, a doctor stage restricted to {0,3} exit codes, and a ROS-vs-env.sh breakdown per stage. No contradictions with ADR-014/016/019/020 were found.”

## What it unlocks next

- **Ready to start:** OPS-01.2 — crackvision.operator_state: per-case run-state file library + status/reset CLI
- **Ready to start:** OPS-01.3 — doctor stage: aggregate check_env/check_host/check_realsense into one operator report
- **Ready to start:** OPS-01.5 — run_trajectory helper: retime -> trajectory_check -> trajectory_preview -> approve_trajectory -> execute_trajectory, one reusable sequence

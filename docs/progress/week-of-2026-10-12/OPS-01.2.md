# OPS-01.2 — crackvision.operator_state: per-case run-state file library + status/reset CLI

**Accepted:** 2026-10-14 · **Work package:** Operator workflow & handoff (L-OPS) · **Track:** software · **Verified commit:** [`e50cb33`](https://github.com/Janga786/rebot-crack-vision/commit/e50cb33640d2f63c763779a198d0b926a447f93b)

## What was done

Built the per-case run-state library and CLI that tracks progress through the operator workflow's six stages (capture, segment, paths, plan, preview, execute), detecting when upstream changes or tampered files should invalidate downstream work already marked done. All 10 unit tests pass, including one verifying that re-running an early stage correctly cascades staleness through every later stage without those stages' own files being touched.

Files changed:
- [src/crackvision/operator_state.py](https://github.com/Janga786/rebot-crack-vision/blob/e50cb33640d2f63c763779a198d0b926a447f93b/src/crackvision/operator_state.py) (+291 / −0)
- [tests/test_operator_state.py](https://github.com/Janga786/rebot-crack-vision/blob/e50cb33640d2f63c763779a198d0b926a447f93b/tests/test_operator_state.py) (+179 / −0)

Commits: [`e50cb33`](https://github.com/Janga786/rebot-crack-vision/commit/e50cb33640d2f63c763779a198d0b926a447f93b)

## Why it was done

A pure-Python library and CLI that creates, loads, updates and freshness-checks a case's crackvision.operator_run/1 file per INTERFACES §19, used by every later OPS-01 stage script.

- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Operator workflow & handoff: **3/12** tasks accepted; whole project: **131/170**.
- REQ-OPS-1: 18/33 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “operator_state.py implements §19's schema, cascade freshness rule, mark_stage/reset semantics, and a conventions-compliant CLI; all 10 tests pass and manual spot-checks (missing-file load, CLI status output) confirm the claimed behavior. The `--root` must precede the subcommand to be recognized, but this matches the existing argparse pattern in capture_record.py, so it is not a defect introduced by this card.”

## What it unlocks next

- **Ready to start:** OPS-01.4 — segment + paths stages: single-case wrapper over prepare_inputs/inference/skeleton/paths/visualize_paths
- **Ready to start:** OPS-01.7 — plan stage: path3d -> plan_crack_path -> retime_plan -> trajectory_check for one case
- Closer: OPS-01.6 — capture stage: D405 + joint-state/optical-tf + capture_record assembly, with optional move-to-view-pose (still needs OPS-01.5)

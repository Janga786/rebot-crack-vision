# OPS-01.4 — segment + paths stages: single-case wrapper over prepare_inputs/inference/skeleton/paths/visualize_paths

**Accepted:** 2026-10-14 · **Work package:** Operator workflow & handoff (L-OPS) · **Track:** software · **Verified commit:** [`dece2b5`](https://github.com/Janga786/rebot-crack-vision/commit/dece2b582e7ec317ceef2253600497121cc6dd8d)

## What was done

Built a command-line tool that runs crack-image segmentation and path-extraction for one case at a time by calling the existing, already-tested segmentation and skeleton/path modules directly, and records each step's success/failure in the project's per-case run-state file so reruns can skip work that's already done. Verified with 6 automated tests using a synthetic 100x100 test image and a stubbed-out segmentation step (no real AI model needed): confirms the tool correctly sequences the stages, marks failures when a sub-step errors, refuses to run the path-extraction step before segmentation has produced a result (exits with the documented error code), and skips re-running work that's already up to date unless forced.

Files changed:
- [scripts/operator_segment_paths.py](https://github.com/Janga786/rebot-crack-vision/blob/dece2b582e7ec317ceef2253600497121cc6dd8d/scripts/operator_segment_paths.py) (+196 / −0)
- [tests/test_operator_segment_paths.py](https://github.com/Janga786/rebot-crack-vision/blob/dece2b582e7ec317ceef2253600497121cc6dd8d/tests/test_operator_segment_paths.py) (+164 / −0)

Commits: [`dece2b5`](https://github.com/Janga786/rebot-crack-vision/commit/dece2b582e7ec317ceef2253600497121cc6dd8d)

## Why it was done

One command that takes one case's source image through segmentation (prepare_inputs+inference) and ordered pixel paths (skeleton+paths+visualize_paths), resumable via crackvision.operator_state, exactly reusing run_test.sh's existing per-stage logic rather than re-implementing it.

- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Operator workflow & handoff: **5/12** tasks accepted; whole project: **137/170**.
- REQ-OPS-1: 24/33 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “operator_segment_paths.py correctly wraps prepare_inputs/inference (segment) and skeleton/paths/visualize_paths (paths) via their main(argv) entry points, wires into operator_state per §19.2/§19.3 with correct exit codes, and enforces the precondition-exit-3 rule. Tests pass and scope is respected.”

## What it unlocks next

- Closer: OPS-01.8 — operator_cli.sh dispatcher: doctor/capture/segment/paths/plan/preview/execute, resumable 'run', end-to-end headless test + OPERATOR_WORKFLOW.md (still needs OPS-01.5, OPS-01.6, OPS-01.7)

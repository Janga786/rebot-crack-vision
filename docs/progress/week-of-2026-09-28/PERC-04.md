# PERC-04 — Path visualization + pipeline stage

**Accepted:** 2026-09-29 · **Work package:** Perception (L-PERC) · **Track:** software · **Verified commit:** [`c1839a5`](https://github.com/Janga786/rebot-crack-vision/commit/c1839a56c7bceb51e20c0485035c6b6b0821073e)

## What was done

Added a new pipeline stage that draws crack path overlays on top of the original photos — numbered labels show which crack is visited first/second/etc., arrows show the direction each crack is traced, and branch offshoots are drawn in distinct colors from the main path. It's wired into the one-command runner as an opt-in `--paths` flag, and all 11 new tests plus the full 252-test suite pass.

Files changed:
- [run_test.sh](https://github.com/Janga786/rebot-crack-vision/blob/c1839a56c7bceb51e20c0485035c6b6b0821073e/run_test.sh) (+22 / −2)
- [src/crackvision/visualize_paths.py](https://github.com/Janga786/rebot-crack-vision/blob/c1839a56c7bceb51e20c0485035c6b6b0821073e/src/crackvision/visualize_paths.py) (+350 / −0)
- [tests/test_visualize_paths.py](https://github.com/Janga786/rebot-crack-vision/blob/c1839a56c7bceb51e20c0485035c6b6b0821073e/tests/test_visualize_paths.py) (+215 / −0)

Commits: [`c1839a5`](https://github.com/Janga786/rebot-crack-vision/commit/c1839a56c7bceb51e20c0485035c6b6b0821073e)

## Why it was done

Operators see ordered paths (direction, order, branches) over the image, produced by the one-command pipeline.

- Requirement **REQ-PERC-2**: 1-px skeleton, branch handling and ordered crack paths in the original pixel frame
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Perception: **11/16** tasks accepted; whole project: **31/74**.
- REQ-PERC-2: 3/4 contributing tasks done
- REQ-OPS-1: 1/5 contributing tasks done
- Verification: automated checks run by the pipeline itself (pytest ✔, syntax ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “PERC-04 adds crackvision.visualize_paths, which renders order/direction/branch overlays from existing paths.json+original image without mutating any data artefact, and wires an opt-in --paths stage into run_test.sh gated against --skip-skeleton. Tests and full suite pass; diff matches claims.”

## What it unlocks next

- Closer: INT-01 — End-to-end simulation pipeline test (still needs GEOM-08, MOT-07, GEOM-08, MOT-07)
- Closer: OPS-01 — Operator CLI workflow (doctor → capture → segment → paths → plan → preview → execute) (still needs MOT-08, MOT-06)
- Closer: PERC-06 — Level-3 evaluation on real imagery (still needs PERC-05, PERC-05)

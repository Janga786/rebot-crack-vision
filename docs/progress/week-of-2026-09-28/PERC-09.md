# PERC-09 — Path accuracy metrics on synthetic ground truth

**Accepted:** 2026-09-29 · **Work package:** Perception (L-PERC) · **Track:** software · **Verified commit:** [`03f6292`](https://github.com/Janga786/rebot-crack-vision/commit/03f629229e55a210dd876da4919c4dbd43d7adec)

## What was done

Added a synthetic crack generator with known ground-truth centerlines (straight, curved, branched, and noisy variants) and a test suite that measures the crack-path-extraction pipeline's accuracy against that ground truth using Hausdorff distance, mean distance, coverage, and path-ordering metrics. All 8 accuracy tests pass with the extracted paths staying within 6px worst-case and 2.5px average distance of the true centerlines.

Files changed:
- [docs/perception/PATH_ACCURACY.md](https://github.com/Janga786/rebot-crack-vision/blob/03f629229e55a210dd876da4919c4dbd43d7adec/docs/perception/PATH_ACCURACY.md) (+76 / −0)
- [tests/test_path_accuracy.py](https://github.com/Janga786/rebot-crack-vision/blob/03f629229e55a210dd876da4919c4dbd43d7adec/tests/test_path_accuracy.py) (+177 / −0)
- [tools/synth_cracks.py](https://github.com/Janga786/rebot-crack-vision/blob/03f629229e55a210dd876da4919c4dbd43d7adec/tools/synth_cracks.py) (+208 / −0)

Commits: [`03f6292`](https://github.com/Janga786/rebot-crack-vision/commit/03f629229e55a210dd876da4919c4dbd43d7adec)

## Why it was done

Path extraction accuracy is measured against known centerlines, with regression thresholds.

- Requirement **REQ-PERC-2**: 1-px skeleton, branch handling and ordered crack paths in the original pixel frame

## How it moves the project forward

- Perception: **13/16** tasks accepted; whole project: **35/74**.
- REQ-PERC-2: 4/4 contributing tasks done — **requirement satisfied**
- Verification: automated checks run by the pipeline itself (pytest ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “PERC-09 adds tools/synth_cracks.py (synthetic straight/curved/branched/noisy crack rasters with exact ground-truth centerlines) and tests/test_path_accuracy.py, which runs the real skeleton→paths pipeline against that ground truth and enforces Hausdorff/mean-distance/coverage/ordering-error thresholds. Diff is scoped exactly to the allowed files. Tests pass (8/8), and I independently verified the metrics behave correctly (ordering_error_rate scores 0 for a reversed path but ~0.49 for a shuffled one; the badly-wrong-path regression test genuinely fails thresholds) and that t
…[200 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

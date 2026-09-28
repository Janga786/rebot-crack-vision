# PERC-03 — Ordered crack paths + path contract

**Accepted:** 2026-09-28 · **Work package:** Perception (L-PERC) · **Track:** software · **Verified commit:** [`7c219bb`](https://github.com/Janga786/rebot-crack-vision/commit/7c219bb92d89bcb03fb191a1f9f6d948cc539fdd)

## What was done

Added the crack-path extraction stage: given a 1-pixel skeleton, the pipeline now produces an ordered, simplified centerline path for the main crack plus any side branches, with multiple cracks in one image visited in a sensible (nearest-neighbor) order. All 21 new tests and the full 160-test suite pass, covering straight, curved, branched, looped, multi-crack and single-pixel synthetic cases.

After the first audit, a repair round (PERC-03.R1) fixed the reported issues: Fixed a bug in the crack-path extraction stage where paths crossing a junction (where two skeleton lines meet or fork) could jump over pixels using a straight-line shortcut that wasn't actually part of the crack. Now the code finds the real connecting route along the skeleton itself, and I added two tests using realistic crossing and fork shapes to catch this regression. The full test suite (164 tests) and the module's own tests (25 tests) both pass.

Files changed:
- [config/project.yaml](https://github.com/Janga786/rebot-crack-vision/blob/7c219bb92d89bcb03fb191a1f9f6d948cc539fdd/config/project.yaml) (+2 / −0)
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/7c219bb92d89bcb03fb191a1f9f6d948cc539fdd/docs/INTERFACES.md) (+72 / −2)
- [src/crackvision/paths.py](https://github.com/Janga786/rebot-crack-vision/blob/7c219bb92d89bcb03fb191a1f9f6d948cc539fdd/src/crackvision/paths.py) (+724 / −40)
- [tests/test_paths.py](https://github.com/Janga786/rebot-crack-vision/blob/7c219bb92d89bcb03fb191a1f9f6d948cc539fdd/tests/test_paths.py) (+548 / −0)

Commits: [`37324ea`](https://github.com/Janga786/rebot-crack-vision/commit/37324eab51c72b787b8880cddb22ec8c21103b68), [`55fbbbf`](https://github.com/Janga786/rebot-crack-vision/commit/55fbbbfd8d325bb935300fbc89f32557a9843434), [`7c219bb`](https://github.com/Janga786/rebot-crack-vision/commit/7c219bb92d89bcb03fb191a1f9f6d948cc539fdd)

## Why it was done

Each case yields ordered, simplified polylines (main path + branches) in pixel coordinates under a written contract.

- Requirement **REQ-PERC-2**: 1-px skeleton, branch handling and ordered crack paths in the original pixel frame

## How it moves the project forward

- Perception: **9/16** tasks accepted; whole project: **20/74**.
- REQ-PERC-2: 2/4 contributing tasks done
- Verification: automated checks run by the pipeline itself (pytest ✔, suite ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair works. Junction gaps are now bridged by a BFS over the real skeleton pixels. The original fork fixture now gives …(20,19),(19,20),(20,21)…, so the path runs through junction node (19,20) and no longer drops (20,21). I ran a stress test on 200 random dilated line masks through skeletonize, with spur settings 5 and 0: 1050 polylines, 0 with a gap, 0 points off the skeleton, and 0 cases that dropped an edge pixel. Every §3.5 reference now reads §3.13. Both acceptance commands pass (25 tests, and 164 passed with 2 skipped). The original PERC-03 criteria still hold, a
…[33 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task PERC-03.R1): Normative module docstring and CLI help point to the wrong INTERFACES section (§3.5 instead of §3.13); Dense paths break and drop a skeleton pixel wherever edges are joined through a junction.

## What it unlocks next

- **Ready to start:** PERC-09 — Path accuracy metrics on synthetic ground truth
- Closer: GEOM-08 — Pixel paths → robot-frame 3D paths with uncertainty (still needs GEOM-03, GEOM-03)
- Closer: PERC-04 — Path visualization + pipeline stage (still needs TC-011)

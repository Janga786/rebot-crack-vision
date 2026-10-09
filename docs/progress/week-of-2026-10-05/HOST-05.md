# HOST-05 — Storage decision for the unmounted 1.9 TB NVMe (K1_Storage)

**Accepted:** 2026-10-09 · **Work package:** Host platform & reproducibility (L-HOST) · **Track:** software · **Verified commit:** [`0aa27c1`](https://github.com/Janga786/rebot-crack-vision/commit/0aa27c19b14e990b30380eb2da120710152becbf)

## What was done

Documented the storage decision for the unmounted 1.9 TB second NVMe drive (labelled K1_Storage, likely belonging to another project): per the operator's decision, it stays completely untouched - no mounting, no fstab changes. All project data (rosbags, LLM model weights, caches) continues to live on the main root drive, which currently has 83 GB free, comfortably above the 40 GB minimum floor set for the LLM weights.

Files changed:
- [docs/host/STORAGE.md](https://github.com/Janga786/rebot-crack-vision/blob/0aa27c19b14e990b30380eb2da120710152becbf/docs/host/STORAGE.md) (+79 / −21)

Commits: [`5f503f5`](https://github.com/Janga786/rebot-crack-vision/commit/5f503f56340b5b4fd1f311e49abb6b1ae43cfd8a), [`0aa27c1`](https://github.com/Janga786/rebot-crack-vision/commit/0aa27c19b14e990b30380eb2da120710152becbf)

## Why it was done

An operator-approved decision on where datasets, rosbags and LLM weights live, applied safely.

- Requirement **REQ-HOST-1**: Reproducible host setup with pinned, recorded versions and an automated drift check

## How it moves the project forward

- Host platform & reproducibility: **8/10** tasks accepted; whole project: **66/106**.
- REQ-HOST-1: 5/6 contributing tasks done
- Verification: automated checks run by the pipeline itself (doc ✔); independent audit accepted it (criteria: 3 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “STORAGE.md accurately records read-only facts about the unmounted K1_Storage NVMe and the operator's decision to leave it untouched, with all data staying on root fs. Facts match live system state exactly; no privileged/mount action was taken; change is scoped to the single allowed file.”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

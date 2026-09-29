# CAM-03 — Recording validator with actionable diagnostics

**Accepted:** 2026-09-29 · **Work package:** D405 capture (L-CAM) · **Track:** camera · **Verified commit:** [`8914eb6`](https://github.com/Janga786/rebot-crack-vision/commit/8914eb6b7b9c3e400d887379ae033a188b1d5aff)

## What was done

Built a one-command recording validator (scripts/validate_recording.py) that checks a D405 recording for metadata completeness, plausible depth scale, timestamp sync/monotonicity, dropped frames, USB-2 fallback, and invalid-depth regions, printing a cause and fix for every problem found. All 16 automated tests pass, covering both clean recordings and each individual failure mode.

Files changed:
- [scripts/validate_recording.py](https://github.com/Janga786/rebot-crack-vision/blob/8914eb6b7b9c3e400d887379ae033a188b1d5aff/scripts/validate_recording.py) (+463 / −0)
- [tests/test_validate_recording.py](https://github.com/Janga786/rebot-crack-vision/blob/8914eb6b7b9c3e400d887379ae033a188b1d5aff/tests/test_validate_recording.py) (+260 / −0)

Commits: [`8914eb6`](https://github.com/Janga786/rebot-crack-vision/commit/8914eb6b7b9c3e400d887379ae033a188b1d5aff)

## Why it was done

One command tells whether a recording is usable and exactly what is wrong if not.

- Requirement **REQ-CAM-1**: D405 discovery with actionable diagnostics (USB 3, firmware, permissions)
- Requirement **REQ-CAM-2**: Aligned colour/depth capture with timestamps, intrinsics, depth scale, extrinsics and metadata

## How it moves the project forward

- D405 capture: **4/7** tasks accepted; whole project: **31/74**.
- REQ-CAM-1: 1/3 contributing tasks done
- REQ-CAM-2: 2/4 contributing tasks done
- Verification: automated checks run by the pipeline itself (pytest ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “CAM-03 recording validator implements all required checks (metadata completeness, depth scale plausibility, timestamp monotonicity, colour/depth delta, dropped frames, USB-2 fallback, invalid-depth fraction) with cause+fix hints per failure, correct INTERFACES §0.2 exit codes, and stays within scope. All 16 tests pass on re-run.”

## What it unlocks next

- Closer: CAM-01 — First D405 connection: discovery, firmware, USB link (still needs HOST-03)
- Closer: CAM-04 — Real recording session over a test surface (still needs CAM-01)

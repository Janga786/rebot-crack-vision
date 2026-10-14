# MOT-08.10 — approve_trajectory: open /dev/tty as separate read/write handles so the typed APPROVE prompt works on a real (pseudo-)terminal

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`d2d250b`](https://github.com/Janga786/rebot-crack-vision/commit/d2d250bb83b9c18a4956e7c8359630c14f7cda6f)

## What was done

Fixed a bug where the trajectory-approval tool could never actually accept a typed APPROVE confirmation on any real terminal, because it opened /dev/tty in a text mode that always errors on non-seekable tty devices. Replaced it with two separate read/write handles, verified with a new test that drives the prompt through an actual pseudo-terminal (not a mock) for both a correct and an incorrect confirmation phrase. All 29 tests for this module pass, including the 4 scheduler acceptance checks.</presentation_summary>

Files changed:
- [src/crackvision/approve_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/d2d250bb83b9c18a4956e7c8359630c14f7cda6f/src/crackvision/approve_trajectory.py) (+21 / −11)
- [tests/test_approve_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/d2d250bb83b9c18a4956e7c8359630c14f7cda6f/tests/test_approve_trajectory.py) (+63 / −0)

Commits: [`d2d250b`](https://github.com/Janga786/rebot-crack-vision/commit/d2d250bb83b9c18a4956e7c8359630c14f7cda6f)

## Why it was done

`crackvision.approve_trajectory` can read the typed APPROVE <sha8> phrase from a real terminal or pseudo-terminal. A real-pty regression test proves this, and every §18 refusal still holds.

- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **59/67** tasks accepted; whole project: **125/157**.
- REQ-OPS-1: 13/20 contributing tasks done
- REQ-INT-2: 14/21 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, no-bypass ✔, no-r-plus-tty ✔, no-tty-refuses ✔); independent audit accepted it (criteria: 3 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The fix is correct and minimal. `_open_tty` now opens /dev/tty twice, once with 'r' and once with 'w' (both utf-8). If the second open fails, the first handle is closed. The reader writes the same four prompt lines plus the prompt string to the write handle, flushes, reads one line from the read handle, and closes both handles in `finally`. Two new tests use `pty.fork()` to run `make_tty_reader` and `confirm` in a child whose controlling terminal is a real pty, with no monkeypatching. A mutation run showed the approve test catches the bug: with the old single 'r+' open rest
…[322 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

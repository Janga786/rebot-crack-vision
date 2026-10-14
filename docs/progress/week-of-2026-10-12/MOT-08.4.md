# MOT-08.4 — crackvision.approve_trajectory: re-verify preview + bindings, typed /dev/tty APPROVE <sha8>, write §11.5 approval

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`542ad3b`](https://github.com/Janga786/rebot-crack-vision/commit/542ad3b94ada94181606e769bb59f453010f8184)

## What was done

Built the approve_trajectory command that re-checks a rendered trajectory preview from scratch (trajectory file, its three preview images, the check report, and the robot's limits/scene/end-effector config files) directly against disk, refusing if any of them were tampered with, missing, or belong to a different file. Only when every check passes does it ask an operator to type an approval phrase on the terminal and then writes a signed approval record; 20 automated tests cover the happy path and every refusal case, all passing.

After the first audit, a repair round (MOT-08.4.R1) fixed the reported issues: Hardened the approve_trajectory CLI's check-report validation so corrupted or malformed robot-motion check reports (e.g. the literal JSON value null, or check lists with bad entries) are now cleanly refused with exit code 3 and no approval file written, instead of crashing. Added three new regression tests; the full suite of 27 tests for this CLI passes, along with the required no-bypass and no-controlling-terminal acceptance checks.

Files changed:
- [src/crackvision/approve_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/542ad3b94ada94181606e769bb59f453010f8184/src/crackvision/approve_trajectory.py) (+414 / −21)
- [tests/test_approve_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/542ad3b94ada94181606e769bb59f453010f8184/tests/test_approve_trajectory.py) (+525 / −0)

Commits: [`6f334fe`](https://github.com/Janga786/rebot-crack-vision/commit/6f334fe1a7cd2547fb95948d6791f4393063e30d), [`0e1b617`](https://github.com/Janga786/rebot-crack-vision/commit/0e1b617dab62af6ad333be641deb17d444feb0b6), [`542ad3b`](https://github.com/Janga786/rebot-crack-vision/commit/542ad3b94ada94181606e769bb59f453010f8184)

## Why it was done

`./env.sh python -m crackvision.approve_trajectory --preview PATH --operator ID` re-verifies the preview, trajectory, check report and config bindings, requires the typed /dev/tty phrase, and writes a five-key crackvision.execution_approval/1 file. Any other outcome is a refusal with exit 3 and no file written.

- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **57/66** tasks accepted; whole project: **120/156**.
- REQ-OPS-1: 12/19 contributing tasks done
- REQ-INT-2: 13/20 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, no-bypass ✔, no-tty-refuses ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this repair. Malformed check reports are now refused with exit 3 instead of crashing or being approved. This covers JSON null, arrays and other scalars, bytes that aren't UTF-8, empty files, 'checks' that is a string or dict, and failing entries whose 'check' is missing, null, a number or a dict. In every case no approval file is written and the run summary's errors name the reason. The 'null' bypass is closed by a `check_doc_ok` flag that only becomes true for a parsed dict. The repair touches only the two files it is allowed to, and it only adds tests; no ex
…[128 chars clipped]”
- Quality loop: the audit found 3 issue(s) that were fixed before acceptance (repair task MOT-08.4.R1): Malformed check report (JSON non-object or non-UTF-8) crashes with exit 1 instead of the §18.2 check-4 exit-3 refusal; Malformed check entries still crash with TypeError (exit 1) instead of an exit-3 refusal; A check report that is the JSON literal `null` passes verification and an approval is written.

## What it unlocks next

- Closer: MOT-08.8 — Headless mock run: view plan → retime → check → preview → approve (pty) → execute_trajectory dry with G-APPROVAL pass, tamper refusals (still needs MOT-08.5, MOT-08.7)

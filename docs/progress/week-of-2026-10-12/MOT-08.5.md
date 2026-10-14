# MOT-08.5 — execution_gate G-APPROVAL: verify preview_artifacts and the bound trajectory_preview/1 manifest (§18 amendment)

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`0ad574b`](https://github.com/Janga786/rebot-crack-vision/commit/0ad574bd807b21c3079e35c10e92f620024306a3)

## What was done

Amended _evaluate_approval in execution_gate.py to implement the §18.3 G-APPROVAL preview-binding checks (non-empty preview_artifacts, per-artifact sha verification, exactly-one trajectory_preview/1 manifest, manifest trajectory-sha match, check_report.all_pass) threaded via root from evaluate_offline, with warn-in-mock/dry / fail-in-real outcomes and no raising on malformed inputs. Added a new _evaluate_preview_binding helper and extensive new tests covering every violation case in both mock and real modes plus a full pass, and updated the existing approval-pass fixture and the four tests that mutate the trajectory to carry a dynamically-built valid preview manifest + PNG artifacts bound to the new trajectory sha.

After the first audit, a repair round (MOT-08.5.R3) fixed the reported issues: Re-checked a previously-flagged crash risk in the robot motion safety gate's approval-file validation and confirmed it's already fixed: feeding it a malformed preview artifact (wrong-type path, a path with an embedded null character, or a non-string checksum) correctly produces a clean rejection message instead of crashing the program. All required automated tests (177 unit tests plus a 57-test execution suite) pass, and the workspace builds cleanly, so no code changes were needed for this task.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py](https://github.com/Janga786/rebot-crack-vision/blob/0ad574bd807b21c3079e35c10e92f620024306a3/ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py) (+100 / −21)
- [ros2_ws/src/crackvision_motion/test/test_execution_gate.py](https://github.com/Janga786/rebot-crack-vision/blob/0ad574bd807b21c3079e35c10e92f620024306a3/ros2_ws/src/crackvision_motion/test/test_execution_gate.py) (+267 / −6)

Commits: [`857e554`](https://github.com/Janga786/rebot-crack-vision/commit/857e554c1287c2b67564492fa09adb5cf547c7cc), [`48928fb`](https://github.com/Janga786/rebot-crack-vision/commit/48928fbbfee8bb4b72de80fa593ca7662e112115), [`f28f805`](https://github.com/Janga786/rebot-crack-vision/commit/f28f8051b506cd1a459b80aa3d2f7de65a05892f)

## Why it was done

G-APPROVAL additionally verifies that every preview artefact exists with a matching sha and that exactly one is a trajectory_preview/1 manifest for the current trajectory with all_pass true. Violations are warn in mock/dry and fail in real, and malformed inputs never raise.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **56/66** tasks accepted; whole project: **119/156**.
- REQ-MOT-1: 38/45 contributing tasks done
- REQ-INT-2: 12/20 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, ros-free ✔, build ✔, execute-unit ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “This card adds no commits. Finding e5cc7474d30db83a was already fixed by MOT-08.5.R1 (commits f28f805 and 48928fb). At execution_gate.py:842-855, `_evaluate_preview_binding` checks the types of path and sha256 and rejects a path containing a NUL byte before it touches the filesystem. It also wraps resolve and read in `except Exception`, so any other error becomes a §18.3 check-2 violation. The parametrised test `test_malformed_preview_artifact_path_or_sha256_does_not_raise` (test_execution_gate.py:306-337) covers path=5, None, a list, a NUL-containing path and sha256=5 in m
…[432 chars clipped]”
- Quality loop: the audit found 6 issue(s) that were fixed before acceptance (repair task MOT-08.5.R3): A manifest JSON nested deeper than the recursion limit makes evaluate_offline raise RecursionError; A preview_artifacts path containing a lone surrogate still makes evaluate_offline raise UnicodeEncodeError; preview_artifacts entry with non-string or NUL-containing path makes evaluate_offline raise; Non-string or NUL-containing preview_artifacts path makes evaluate_offline raise instead of returning a G-APPROVAL violation; G-APPROVAL preview binding lets TypeError/ValueError escape evaluate_offline for malformed preview_artifacts entries.

## What it unlocks next

- Closer: MOT-08.8 — Headless mock run: view plan → retime → check → preview → approve (pty) → execute_trajectory dry with G-APPROVAL pass, tamper refusals (still needs MOT-08.7)

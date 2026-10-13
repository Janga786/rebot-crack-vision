# MOT-08.2 — Preview core: bind trajectory/plan/retime/check inputs, FK tool path, summary metrics and trajectory_preview/1 manifest (pure, no plotting)

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`00f73ca`](https://github.com/Janga786/rebot-crack-vision/commit/00f73ca61842041f4ea112c15b962f21aa7edd7b)

## What was done

Built the pure-Python preview core (crackvision/preview_core.py) that validates and sha-binds a trajectory, plan, retime report and check report together, computes the per-joint velocity/acceleration and tool-tip path summary metrics the operator preview will show, and builds/validates the trajectory_preview manifest format, all with no plotting or ROS dependencies. Added 19 unit tests covering every refusal case (bad trajectory, wrong plan schema, sha mismatches), a hand-verified numeric summary, and a manifest round trip; all pass, along with the existing 86 trajectory-check regression tests, confirming nothing else broke.

After the first audit, a repair round (MOT-08.2.R1) fixed the reported issues: Repaired a test gap in the motion-preview module's input-binding logic: the test that checks a retimed-trajectory's stored hash against the actual trajectory file now uses a valid, non-empty retime report and checks the specific mismatch error message. I confirmed the fix works by temporarily breaking the hash check in the code and verifying the test catches it, then restored the code unchanged. All 19 preview tests and the 86 related checker tests pass.

Files changed:
- [src/crackvision/preview_core.py](https://github.com/Janga786/rebot-crack-vision/blob/00f73ca61842041f4ea112c15b962f21aa7edd7b/src/crackvision/preview_core.py) (+682 / −0)
- [tests/test_preview_core.py](https://github.com/Janga786/rebot-crack-vision/blob/00f73ca61842041f4ea112c15b962f21aa7edd7b/tests/test_preview_core.py) (+511 / −2)

Commits: [`87c3463`](https://github.com/Janga786/rebot-crack-vision/commit/87c3463d853677e90a84809d4cd1abb5913be3e0), [`00f73ca`](https://github.com/Janga786/rebot-crack-vision/commit/00f73ca61842041f4ea112c15b962f21aa7edd7b)

## Why it was done

crackvision.preview_core validates and sha-binds the preview inputs, computes the §18 summary metrics and tool-tip path with crackvision.kinematics, and builds, validates and loads crackvision.trajectory_preview/1 manifests, with unit tests.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **54/66** tasks accepted; whole project: **116/156**.
- REQ-MOT-2: 38/42 contributing tasks done
- REQ-OPS-1: 9/19 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, independence ✔, checker-regression ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this repair. The retime sha-mismatch test now builds a retime report that passes schema validation (it has one leg) but carries the wrong trajectory_sha256. It checks for the sha-specific refusal message. To confirm the test really reaches the sha check, I disabled that comparison in a temporary copy of preview_core.py: the test then failed with "DID NOT RAISE PreviewRefusal", so it would catch the comparison being removed. With the code as committed, all three scheduler checks pass when I rerun them. The working tree is unchanged after my review, and only the
…[111 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task MOT-08.2.R1): Retime-report sha-mismatch test never reaches the sha-binding branch.

## What it unlocks next

- **Ready to start:** MOT-08.3 — crackvision.trajectory_preview CLI: deterministic headless PNG render (joints, tool path + crack + scene, tool speed) and manifest
- **Ready to start:** MOT-08.4 — crackvision.approve_trajectory: re-verify preview + bindings, typed /dev/tty APPROVE <sha8>, write §11.5 approval

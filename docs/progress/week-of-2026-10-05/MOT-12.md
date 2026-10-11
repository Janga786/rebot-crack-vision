# MOT-12 — Vendor checker: evaluate the SDK layout its own V-SDK-API remedy produces (SDK pinned at 8427281, actuator/arm.py)

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`c8fd245`](https://github.com/Janga786/rebot-crack-vision/commit/c8fd245b1c2b5f0db190754101cb19fce46652de)

## What was done

The vendor-driver readiness checker (check_vendor_driver.py) now correctly evaluates the exact SDK layout its own fix recommends (pinning to commit 8427281), instead of erroring out before it could report a verdict. Verified against both a synthetic test fixture and a real build of commit 8427281 from the operator's actual ~/rebot_ws, read-only: the checker now reports PASS for the SDK-API and baud checks at that pinned commit, with the two remaining real issues (an unsafe parking position and missing Python dependencies) still correctly flagged as failures.

After the first audit, a repair round (MOT-12.R1) fixed the reported issues: Re-checked the vendor-driver readiness tool's detached-HEAD test: it now generates its Git metadata on the fly instead of depending on a file Git can't track, so the test suite (19 tests) passes cleanly even from a brand-new checkout of the repo. All four required checks for this fix, including a live run against the pinned robot-SDK commit, passed with no code changes needed.

Files changed:
- [docs/motion/VENDOR_DRIVER.md](https://github.com/Janga786/rebot-crack-vision/blob/c8fd245b1c2b5f0db190754101cb19fce46652de/docs/motion/VENDOR_DRIVER.md) (+14 / −0)
- [scripts/motion/check_vendor_driver.py](https://github.com/Janga786/rebot-crack-vision/blob/c8fd245b1c2b5f0db190754101cb19fce46652de/scripts/motion/check_vendor_driver.py) (+13 / −3)
- [tests/fixtures/vendor_driver/pinned_sdk/reBotArm_control_py/actuator/__init__.py](https://github.com/Janga786/rebot-crack-vision/blob/c8fd245b1c2b5f0db190754101cb19fce46652de/tests/fixtures/vendor_driver/pinned_sdk/reBotArm_control_py/actuator/__init__.py) (+3 / −0)
- [tests/fixtures/vendor_driver/pinned_sdk/reBotArm_control_py/actuator/arm.py](https://github.com/Janga786/rebot-crack-vision/blob/c8fd245b1c2b5f0db190754101cb19fce46652de/tests/fixtures/vendor_driver/pinned_sdk/reBotArm_control_py/actuator/arm.py) (+12 / −0)
- [tests/fixtures/vendor_driver/pinned_sdk/reBotArm_control_py/actuator/gripper.py](https://github.com/Janga786/rebot-crack-vision/blob/c8fd245b1c2b5f0db190754101cb19fce46652de/tests/fixtures/vendor_driver/pinned_sdk/reBotArm_control_py/actuator/gripper.py) (+2 / −0)
- [tests/fixtures/vendor_driver/pinned_sdk/reBotArm_control_py/controllers/__init__.py](https://github.com/Janga786/rebot-crack-vision/blob/c8fd245b1c2b5f0db190754101cb19fce46652de/tests/fixtures/vendor_driver/pinned_sdk/reBotArm_control_py/controllers/__init__.py) (+3 / −0)
- [tests/test_check_vendor_driver.py](https://github.com/Janga786/rebot-crack-vision/blob/c8fd245b1c2b5f0db190754101cb19fce46652de/tests/test_check_vendor_driver.py) (+51 / −0)

Commits: [`edfbcc1`](https://github.com/Janga786/rebot-crack-vision/commit/edfbcc18baf2cc36e8ff1158458ed2fbc205123f)

## Why it was done

check_vendor_driver.py reaches a verdict (exit 0 or 3, never 2) against ~/rebot_ws with the SDK pinned at 8427281. The operator can then verify the vendor fixes with the same command before MOT-09's dry rehearsal.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **22/40** tasks accepted; whole project: **75/123**.
- REQ-MOT-1: 20/33 contributing tasks done
- REQ-INT-2: 8/16 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, live-report ✔, pinned-sdk-live ✔, read-only ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The detached-HEAD fix is committed at HEAD. It landed in commit 0841905 (tagged MOT-11.R1); MOT-12.R1 itself has no commits. `_make_pinned_sdk_tree` copies the pinned_sdk fixture with `.git` ignored, then writes `sdk_root/.git/HEAD` at runtime. A clean checkout built with `git archive HEAD` passes 19/19 tests. The live report, the read-only grep and the pinned-sdk-live check all hold. The working tree is unchanged.”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task MOT-12.R1): The detached-HEAD test depends on an untracked .git/HEAD fixture and fails in a clean checkout.

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

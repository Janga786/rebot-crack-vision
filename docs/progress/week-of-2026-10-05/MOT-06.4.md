# MOT-06.4 — Cartesian plan document: assembly, coverage metrics, strict validator and loader (pure Python)

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`df714b5`](https://github.com/Janga786/rebot-crack-vision/commit/df714b58240d02a7eda5f39756e225b8079e4b08)

## What was done

Built and unit-tested the module that assembles, validates, writes and reads the robot's Cartesian crack-path plan files (and the camera view-plan files) -- the format the next motion-planning stage will consume. All 56 new tests and the full 397-test motion package regression suite pass, covering every required rejection case (bad fields, wrong joint names, non-finite or out-of-limit joint positions, broken motion continuity between segments, and inconsistent coverage numbers) plus hand-checked coverage math for fully, partially, and failed/mismatched crack segments.

After the first audit, a repair round (MOT-06.4.R1) fixed the reported issues: Repaired the cartesian motion planner's plan-file builder (cartesian_plan.py) so it no longer trusts a possibly-stale eligibility flag: it now reads execution_eligible straight from the upstream 3D-path file and errors out if a caller's copy disagrees, and it now rejects internally-inconsistent segment reports (e.g. a 'failed' segment that still names a chosen candidate, or a 'full'/'partial' one with no candidate chosen). All 66 targeted tests and the full 407-test motion-planning suite pass.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py](https://github.com/Janga786/rebot-crack-vision/blob/df714b58240d02a7eda5f39756e225b8079e4b08/ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py) (+936 / −2)
- [ros2_ws/src/crackvision_motion/test/test_cartesian_plan.py](https://github.com/Janga786/rebot-crack-vision/blob/df714b58240d02a7eda5f39756e225b8079e4b08/ros2_ws/src/crackvision_motion/test/test_cartesian_plan.py) (+710 / −4)

Commits: [`bc6ea5c`](https://github.com/Janga786/rebot-crack-vision/commit/bc6ea5c188abf96253d0342dc2e8bb7766798481), [`df714b5`](https://github.com/Janga786/rebot-crack-vision/commit/df714b58240d02a7eda5f39756e225b8079e4b08)

## Why it was done

crackvision_motion.cartesian_plan builds, validates, writes and loads crackvision.cartesian_plan/1 and crackvision.view_plan/1 documents exactly per §13, computes per-segment status and coverage (by waypoints and by arc length, plus upstream paths3d counts), and verifies leg continuity — unit-tested without ROS.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **23/40** tasks accepted; whole project: **76/123**.
- REQ-MOT-2: 11/21 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, regression ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this card: both MOT-06.4 findings are fixed. `finalize` now takes `execution_eligible` from `paths3d_doc`. It raises `CartesianPlanError` if the value is not a bool or does not match a value already in sources. The caveat is built from the document's value. `add_segment_report` and `_validate_segment_report` both enforce the chosen/reasons rules for full, partial and failed segments. A probe confirmed each new test fails on the intended check, not on a side effect such as a coverage mismatch. Both acceptance checks pass when I re-run them: 66 unit tests and 40
…[95 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task MOT-06.4.R1): finalize does not copy execution_eligible from the paths3d file into sources.paths3d; Segment report status invariants for chosen/reasons from §13.11.3 are not enforced.

## What it unlocks next

- Closer: MOT-06.6 — plan_crack_path node/CLI: per-segment candidates, transit + approach/trace/retract, split fallback, independent re-check, plan file (still needs MOT-06.5)
- Closer: MOT-06.9 — plan_view node/CLI + headless smoke: reachable, collision-free camera view pose with the specimen in FOV (still needs MOT-06.8, MOT-06.5, MOT-06.6)

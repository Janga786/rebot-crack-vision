# MOT-07.8 — Offline cross-validation: retimer vs independent checker vs G-LIMITS on a committed synthetic plan, with fault injection

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`d6b7712`](https://github.com/Janga786/rebot-crack-vision/commit/d6b77127b83929d76f7d776dde4b87f96d76f1de)

## What was done

The offline cross-validation test for the motion retimer now runs cleanly end-to-end: it retimes a committed synthetic robot path, checks it independently, and confirms the safety limit gate agrees. It also proves the safety checks catch 7 different injected errors (like overspeed segments or corrupted timing data) every time, with zero false negatives.

After the first audit, a repair round (MOT-07.8.R1) fixed the reported issues: Fixed a test gap in the motion-planning fault-injection harness: the synthetic "tool-speed dip" fault used to trigger three unrelated checker failures at once, which would have hidden a real regression in the tool-speed check it's supposed to test. The fault is now a smooth, isolated slowdown that fails only the intended check, and the test script explicitly verifies each of the 7 injected faults trips its expected check. Re-ran the full cross-check script end-to-end and confirmed it passes (exit 0) with the new, stricter assertions.

Files changed:
- [ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/README.md](https://github.com/Janga786/rebot-crack-vision/blob/d6b77127b83929d76f7d776dde4b87f96d76f1de/ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/README.md) (+41 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/b601_dm_limits.yaml](https://github.com/Janga786/rebot-crack-vision/blob/d6b77127b83929d76f7d776dde4b87f96d76f1de/ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/b601_dm_limits.yaml) (+149 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/cartesian_plan.json](https://github.com/Janga786/rebot-crack-vision/blob/d6b77127b83929d76f7d776dde4b87f96d76f1de/ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/cartesian_plan.json) (+1042 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/end_effector.yaml](https://github.com/Janga786/rebot-crack-vision/blob/d6b77127b83929d76f7d776dde4b87f96d76f1de/ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/end_effector.yaml) (+94 / −0)
- [scripts/ros/retime_fault_inject.py](https://github.com/Janga786/rebot-crack-vision/blob/d6b77127b83929d76f7d776dde4b87f96d76f1de/scripts/ros/retime_fault_inject.py) (+403 / −12)
- [scripts/ros/test_retime_crosscheck.sh](https://github.com/Janga786/rebot-crack-vision/blob/d6b77127b83929d76f7d776dde4b87f96d76f1de/scripts/ros/test_retime_crosscheck.sh) (+287 / −0)

Commits: [`2ccf425`](https://github.com/Janga786/rebot-crack-vision/commit/2ccf425fd7676328df2e7350edaac315504c1b9b), [`d6b7712`](https://github.com/Janga786/rebot-crack-vision/commit/d6b77127b83929d76f7d776dde4b87f96d76f1de)

## Why it was done

scripts/ros/test_retime_crosscheck.sh retimes a committed MOT-06.10 synthetic plan (system python), passes it through the independent checker (conda) and execution_gate's offline gates, and shows that injected faults are caught by the checker and, for limit faults, also by G-LIMITS — all offline, with no ROS graph.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-INT-1**: Simulation and recorded-data integration tests of the full pipeline

## How it moves the project forward

- Motion planning: **43/52** tasks accepted; whole project: **105/139**.
- REQ-MOT-2: 28/33 contributing tasks done
- REQ-INT-1: 4/8 contributing tasks done
- Verification: automated checks run by the pipeline itself (crosscheck ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair works. On the committed fixture, the toolspeed_dip fault now fails only C-TOOLSPEED; C-LIMITS, C-SAMPLING and C-PATH all pass. test_retime_crosscheck.sh now asserts the expected failing check for each of the 7 faults, plus isolation for toolspeed_dip, and I confirmed that a mismatch makes it fail. I re-ran the cross-check: exit 0, working tree still clean. The dip falls outside the MOT-07.11 vertex-tied-dip allowance for two independent reasons. Its below-band run never touches a vertex segment for any v_c in the admissible range. And even if it were treated as v
…[324 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task MOT-07.8.R1): toolspeed_dip fault is not isolated: it also breaks C-SAMPLING and C-LIMITS, so it never shows that C-TOOLSPEED alone catches a dip.

## What it unlocks next

- **Ready to start:** MOT-07.9 — Headless mock execution of a retimed synthetic crack plan + TIME_PARAMETERIZATION.md

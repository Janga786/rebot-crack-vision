# MOT-05.5 — Headless mock verification of the execution gate: dry/mock execution, refusals, collision, e-stop injection, vendor-interface mock

**Accepted:** 2026-10-14 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`d765c43`](https://github.com/Janga786/rebot-crack-vision/commit/d765c4391e8da67d79f4bc257b9a554c6ee6586f)

## What was done

scripts/ros/test_execution.sh now checks the commissioning-gated trajectory executor end to end with no hardware, using both the MoveIt mock stack and the vendor-interface mock_driver. All 9 cases passed. Real mode and overspeed trajectories are refused offline. Dry mode leaves the joints unchanged (max change 0.0). Mock runs reach their goal. A start-state mismatch and a collision are refused with no motion. An injected e-stop stops the arm short of its goal (0.09 rad left). Real mode against the mock driver is refused by the driver-graph check, and no processes survive teardown.

After the first audit, a repair round (MOT-05.5.R1) fixed the reported issues: Repaired the motion-execution end-to-end mock test, which had broken after an earlier change tightened the operator-approval check to require a bound trajectory-preview record. The test fixture now generates that record correctly, and the full test suite (build, execution end-to-end, scene regression) passes again.

Files changed:
- [ros2_ws/src/crackvision_motion/test/fixtures/execution_test_real_refusal.yaml](https://github.com/Janga786/rebot-crack-vision/blob/d765c4391e8da67d79f4bc257b9a554c6ee6586f/ros2_ws/src/crackvision_motion/test/fixtures/execution_test_real_refusal.yaml) (+56 / −0)
- [scripts/ros/test_execution.sh](https://github.com/Janga786/rebot-crack-vision/blob/d765c4391e8da67d79f4bc257b9a554c6ee6586f/scripts/ros/test_execution.sh) (+575 / −22)

Commits: [`d32d1ee`](https://github.com/Janga786/rebot-crack-vision/commit/d32d1eec9a4ea52d4775619463b2d71d6dd52d64), [`ace1fd6`](https://github.com/Janga786/rebot-crack-vision/commit/ace1fd6de9442211e4ad46261bf9975238a422e1), [`51efed5`](https://github.com/Janga786/rebot-crack-vision/commit/51efed5bbba06fce4e82dbe15c2776213b8fcc5a), [`d765c43`](https://github.com/Janga786/rebot-crack-vision/commit/d765c4391e8da67d79f4bc257b9a554c6ee6586f)

## Why it was done

scripts/ros/test_execution.sh proves the executor end to end without hardware. Real mode is refused offline, and against a mock driver it is refused by the graph check. Dry mode leaves the arm state unchanged. Mock mode reaches the goal on the MoveIt mock stack and on rebot_motion's vendor-interface mock_driver. Start-state mismatch, overspeed and collision are refused with no motion, an injected e-stop stops mid-trajectory, and no stray processes survive.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **49/66** tasks accepted; whole project: **112/156**.
- REQ-MOT-1: 33/45 contributing tasks done
- REQ-INT-2: 10/20 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, execution-mock-e2e ✔, scene-regression ✔); independent audit accepted it (criteria: 3 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this repair. The commit changes one in-scope file, scripts/ros/test_execution.sh. In the vendor real-mode case, the approval used to carry an empty preview_artifacts list, which upstream MOT-08.5 now rejects at the offline approval check (G-APPROVAL). The fix writes a minimal preview manifest that is valid under §18.3 and bound to the trajectory's sha256. That lets the case get past the approval check and reach the online graph check (G-GRAPH) it is meant to test. The test assertions are as strict as before: exit 3, record outcome "refused", G-GRAPH recorded a
…[203 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task MOT-05.5.R1): check execution-mock-e2e fails after upstream change.

## What it unlocks next

- **Ready to start:** MOT-05.6 — Operator execution runbook (mock → dry → real) + ROS_WORKSPACE.md entry

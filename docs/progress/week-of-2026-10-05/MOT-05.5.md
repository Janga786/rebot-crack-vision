# MOT-05.5 — Headless mock verification of the execution gate: dry/mock execution, refusals, collision, e-stop injection, vendor-interface mock

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`51efed5`](https://github.com/Janga786/rebot-crack-vision/commit/51efed5bbba06fce4e82dbe15c2776213b8fcc5a)

## What was done

scripts/ros/test_execution.sh now checks the commissioning-gated trajectory executor end to end with no hardware, using both the MoveIt mock stack and the vendor-interface mock_driver. All 9 cases passed. Real mode and overspeed trajectories are refused offline. Dry mode leaves the joints unchanged (max change 0.0). Mock runs reach their goal. A start-state mismatch and a collision are refused with no motion. An injected e-stop stops the arm short of its goal (0.09 rad left). Real mode against the mock driver is refused by the driver-graph check, and no processes survive teardown.

Files changed:
- [ros2_ws/src/crackvision_motion/test/fixtures/execution_test_real_refusal.yaml](https://github.com/Janga786/rebot-crack-vision/blob/51efed5bbba06fce4e82dbe15c2776213b8fcc5a/ros2_ws/src/crackvision_motion/test/fixtures/execution_test_real_refusal.yaml) (+56 / −0)
- [scripts/ros/test_execution.sh](https://github.com/Janga786/rebot-crack-vision/blob/51efed5bbba06fce4e82dbe15c2776213b8fcc5a/scripts/ros/test_execution.sh) (+556 / −21)

Commits: [`d32d1ee`](https://github.com/Janga786/rebot-crack-vision/commit/d32d1eec9a4ea52d4775619463b2d71d6dd52d64), [`ace1fd6`](https://github.com/Janga786/rebot-crack-vision/commit/ace1fd6de9442211e4ad46261bf9975238a422e1), [`51efed5`](https://github.com/Janga786/rebot-crack-vision/commit/51efed5bbba06fce4e82dbe15c2776213b8fcc5a)

## Why it was done

scripts/ros/test_execution.sh proves the executor end to end without hardware. Real mode is refused offline, and against a mock driver it is refused by the graph check. Dry mode leaves the arm state unchanged. Mock mode reaches the goal on the MoveIt mock stack and on rebot_motion's vendor-interface mock_driver. Start-state mismatch, overspeed and collision are refused with no motion, an injected e-stop stops mid-trajectory, and no stray processes survive.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **27/40** tasks accepted; whole project: **81/124**.
- REQ-MOT-1: 22/33 contributing tasks done
- REQ-INT-2: 9/16 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, execution-mock-e2e ✔, scene-regression ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “This attempt's only change is a header-comment update in scripts/ros/test_execution.sh replacing a stale 'known limitation' note with a short note that MOT-05.7 fixed the RELIABLE/BEST_EFFORT QoS mismatch blocking the vendor_mock case. MOT-05.7 is confirmed accepted and the fix is present in execute_trajectory.py. I independently re-ran all three acceptance checks (build, test_execution.sh, test_scene.sh) and they reproduced exactly: all 9 sub-cases in the execution test pass with correct exit codes, gate ids, and outcomes, no stray processes remain, and the working tree wa
…[26 chars clipped]”

## What it unlocks next

- **Ready to start:** MOT-05.6 — Operator execution runbook (mock → dry → real) + ROS_WORKSPACE.md entry

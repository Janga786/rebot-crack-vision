# MOT-05.7 — execute_trajectory: subscribe to joint_states with sensor-data (BEST_EFFORT) QoS and separate the DDS-discovery wait from the staleness window

**Accepted:** 2026-10-10 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`e84a657`](https://github.com/Janga786/rebot-crack-vision/commit/e84a657ccd198905dded2e3d1659f2f798d4692f)

## What was done

Fixed execute_trajectory's joint_states subscriptions to use BEST_EFFORT (qos_profile_sensor_data) instead of RELIABLE depth-10, matching the real reBotArmController and rebot_motion's mock_driver publishers. Separated G-START-STATE's DDS-discovery wait (now bounded by --service-timeout-s, used as the OnlineChecks discovery bound for both the joint_states and latched arm_status waits) from the timeouts.joint_state_s staleness check applied only to the received message's own age. Added scripts/ros/test_joint_states_qos.sh, a live smoke test proving a BEST_EFFORT subscription receives mock_driver's joint_states while an explicit RELIABLE one does not, with real-driver-refusal (pgrep -x reBotArmControl -> exit 3) and clean process-group teardown.

After the first audit, a repair round (MOT-05.7.R1) fixed the reported issues: Added two targeted unit tests for the arm-execution safety node that close gaps a reviewer had flagged: one proves the pre-motion freshness check correctly refuses a stale sensor reading, and the other proves the in-motion safety monitor cancels the robot's goal within about 50ms if joint-position feedback stops arriving mid-motion, rather than only noticing after the whole motion times out. Both tests were manually verified to fail if the underlying safety logic they check is broken, confirming they actually test what they claim to. All 156 project unit tests and all 6 required build/integration checks pass.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/e84a657ccd198905dded2e3d1659f2f798d4692f/ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py) (+24 / −8)
- [ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/e84a657ccd198905dded2e3d1659f2f798d4692f/ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py) (+206 / −9)
- [scripts/ros/test_joint_states_qos.sh](https://github.com/Janga786/rebot-crack-vision/blob/e84a657ccd198905dded2e3d1659f2f798d4692f/scripts/ros/test_joint_states_qos.sh) (+131 / −0)

Commits: [`d2597b7`](https://github.com/Janga786/rebot-crack-vision/commit/d2597b765207384c7560863d31fa1bb1b6733d56), [`e84a657`](https://github.com/Janga786/rebot-crack-vision/commit/e84a657ccd198905dded2e3d1659f2f798d4692f)

## Why it was done

execute_trajectory receives /rebotarm/joint_states from the real reBotArmController and from rebot_motion's mock_driver, both of which publish BEST_EFFORT. A fresh node no longer refuses G-START-STATE because DDS discovery took longer than joint_state_s. This unblocks MOT-05.5 and MOT-09's `--mode dry --profile vendor` rehearsal.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **14/29** tasks accepted; whole project: **65/108**.
- REQ-MOT-1: 12/26 contributing tasks done
- REQ-INT-2: 5/15 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, qos-live ✔, real-refused-offline ✔, single-action-client ✔, no-vendor-motion-services ✔, mock-plan-regression ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this repair. The commit changes only test_execute_trajectory.py: it adds two tests, and the production file execute_trajectory.py is identical to the MOT-05.7 revision. I ran both required mutations myself on a git-archive copy in /tmp, and each one is caught by its new test. Mutating the age calculation at line 413 to `age_s = 0.0` fails test_start_state_fails_when_only_message_is_stale_relative_to_joint_state_timeout (1 failed, 56 passed). Mutating line 636 to `if False:` fails test_execute_joint_states_gap_mid_goal_trips_about_joint_state_s_after_last_messa
…[489 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task MOT-05.7.R1): No unit test drives start_state with a fake whose only message is older than joint_state_s; Mid-execution joint_states gap -> §11.8 trip is not pinned by any unit test.

## What it unlocks next

- Closer: MOT-05.5 — Headless mock verification of the execution gate: dry/mock execution, refusals, collision, e-stop injection, vendor-interface mock (still needs MOT-05.4)

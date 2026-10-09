# MOT-05.4 — execute_trajectory ROS node/CLI: online read-only checks, mock/dry/real execution, typed confirmation, e-stop + tracking monitor, execution record

**Accepted:** 2026-10-09 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`78c5afd`](https://github.com/Janga786/rebot-crack-vision/commit/78c5afd5711804edf777a3cba92423fa2c5a2507)

## What was done

Implemented execute_trajectory, the ADR-016/§11 commissioning-gated-execution ROS node/CLI for crackvision_motion. It runs the MOT-05.3 offline gate before any rclpy.init() call, then online read-only checks (G-GRAPH, G-START-STATE, G-COLLISION), then (real mode only) a /dev/tty typed confirmation, then executes via the single FollowJointTrajectory ActionClient under e-stop/staleness/tracking monitoring per ADR-016 §7-8 (cancel-and-hold, never disable). A crackvision.execution_record/1 is written in a finally block for every non-"--dry-run" invocation, including refusals; --dry-run runs only the offline gate in-process and writes nothing. Added the console_script entry point and exec_depends (control_msgs, trajectory_msgs, sensor_msgs, std_msgs, rebotarm_msgs).

After the first audit, a repair round (MOT-05.4.R1) fixed the reported issues: The trajectory executor now takes its allowed mode/profile pairs from a fixed table in the code, so an edited config can't make mock mode send goals to the real arm driver; the CLI exits with code 2 if the config differs from the table. All waits after an emergency stop now have time limits, and if a stop can't be confirmed the executor prints "PRESS THE HARDWARE E-STOP". A run counts as completed only after fresh joint readings show the arm within 0.06 rad of the final target, and long slow trajectories at the 10% commissioning speed are no longer cancelled after 5 s. The unit tests (148 passed), build, refusal checks and the MoveIt mock-planning smoke test all pass; nothing here was run on the physical arm.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/78c5afd5711804edf777a3cba92423fa2c5a2507/ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py) (+975 / −36)
- [ros2_ws/src/crackvision_motion/package.xml](https://github.com/Janga786/rebot-crack-vision/blob/78c5afd5711804edf777a3cba92423fa2c5a2507/ros2_ws/src/crackvision_motion/package.xml) (+7 / −0)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/78c5afd5711804edf777a3cba92423fa2c5a2507/ros2_ws/src/crackvision_motion/setup.py) (+1 / −0)
- [ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py](https://github.com/Janga786/rebot-crack-vision/blob/78c5afd5711804edf777a3cba92423fa2c5a2507/ros2_ws/src/crackvision_motion/test/test_execute_trajectory.py) (+808 / −19)

Commits: [`6f09e15`](https://github.com/Janga786/rebot-crack-vision/commit/6f09e154c7109d356bbb35ae1b14f43fe8c22f01), [`78c5afd`](https://github.com/Janga786/rebot-crack-vision/commit/78c5afd5711804edf777a3cba92423fa2c5a2507)

## Why it was done

`ros2 run crackvision_motion execute_trajectory` runs the MOT-05.3 offline gate first, so a refusal never initialises ROS. It then runs the online read-only checks (graph/profile discrimination, fresh start state, /check_state_validity on densified waypoints with the scene objects present). Dry mode stops there without constructing an ActionClient. Mock mode executes on a mock profile. Real mode additionally needs the /dev/tty typed confirmation and executes with e-stop/staleness/tracking monitoring. An execution record and §0.4 logs are always written.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **15/27** tasks accepted; whole project: **60/104**.
- REQ-MOT-1: 12/24 contributing tasks done
- REQ-INT-2: 5/13 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, real-refused-offline ✔, single-action-client ✔, no-vendor-motion-services ✔, mock-plan-regression ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “All five blocker/major findings from the MOT-05.4 review are fixed with a sound, hardcoded §11.1 profile table that the executor consults directly for every execution decision (action name, topic, expected node), making the config file purely a 'must-match-exactly' integrity check rather than an authority. I independently reproduced the original Finding 1 exploit against the patched code and confirmed it now fails with exit 2 before touching rclpy or execute(). I re-ran every scheduler acceptance command myself (build, 148 unit tests, real-mode offline refusal, single-Actio
…[335 chars clipped]”
- Quality loop: the audit found 5 issue(s) that were fixed before acceptance (repair task MOT-05.4.R1): Unacknowledged cancel blocks forever and never prints PRESS THE HARDWARE E-STOP; Trip while the send_goal request is pending returns without cancelling; the goal can then run unmonitored; No arrival verification after a successful result; timeouts.goal_s used as an absolute limit instead of scaled duration + goal_s; real-speed runs always abort; Mode/profile table taken from overridable --execution-config lets mock mode send goals to the real driver with every real-only gate skipped.

## What it unlocks next

- **Ready to start:** MOT-05.5 — Headless mock verification of the execution gate: dry/mock execution, refusals, collision, e-stop injection, vendor-interface mock

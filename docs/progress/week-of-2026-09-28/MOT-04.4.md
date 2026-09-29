# MOT-04.4 — MoveIt reachability sweep node (compute_ik + independent FK re-check) + headless run/test scripts

**Accepted:** 2026-09-29 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`965efea`](https://github.com/Janga786/rebot-crack-vision/commit/965efea385110af6e8267156d8cb07b4902ca8ec)

## What was done

Built the MoveIt reachability-sweep tool for the robot arm: it queries the real IK/FK solver (via move_group's compute_ik/compute_fk services, never an action client, so it can't move the arm) across a grid of inspection targets, double-checks every accepted solution with an independent forward-kinematics call, and writes out a reachability map. On a 6-point test fixture, run against the real headless MoveIt stack, it correctly found 3 near targets reachable and 3 far targets unreachable, with zero mismatches between the IK and FK checks, and cleanly tears down all ROS processes afterward. Along the way I discovered and worked around a real shell-scripting bug and confirmed a genuine physical finding: the gripper's own body is large enough that very close inspection standoffs (1-4 cm) can put it inside the surface being inspected once collision-checking is turned on, so a future production run may need a larger standoff than originally planned.

After the first audit, a repair round (MOT-04.4.R1) fixed the reported issues: Fixed a bug in the robot-arm reachability sweep tool where a misconfigured MoveIt request (e.g. a typo'd planning group name) was being silently recorded as \"target unreachable\" instead of being flagged as an error. Now such configuration/service failures correctly produce an \"error\" status naming the exact MoveIt error code and make the tool exit with a failure code, so downstream placement recommendations can no longer be built from a bad reachability map. Verified with a live repro (bad group name) confirming exit 1 and error status, plus the full build and smoke-test suite passing.

Files changed:
- [docs/motion/ROS_WORKSPACE.md](https://github.com/Janga786/rebot-crack-vision/blob/965efea385110af6e8267156d8cb07b4902ca8ec/docs/motion/ROS_WORKSPACE.md) (+103 / −1)
- [ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py](https://github.com/Janga786/rebot-crack-vision/blob/965efea385110af6e8267156d8cb07b4902ca8ec/ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py) (+646 / −1)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/965efea385110af6e8267156d8cb07b4902ca8ec/ros2_ws/src/crackvision_motion/setup.py) (+1 / −0)
- [ros2_ws/src/crackvision_motion/test/fixtures/reachability_smoke.yaml](https://github.com/Janga786/rebot-crack-vision/blob/965efea385110af6e8267156d8cb07b4902ca8ec/ros2_ws/src/crackvision_motion/test/fixtures/reachability_smoke.yaml) (+76 / −0)
- [scripts/ros/_mock_stack.sh](https://github.com/Janga786/rebot-crack-vision/blob/965efea385110af6e8267156d8cb07b4902ca8ec/scripts/ros/_mock_stack.sh) (+60 / −0)
- [scripts/ros/run_reachability.sh](https://github.com/Janga786/rebot-crack-vision/blob/965efea385110af6e8267156d8cb07b4902ca8ec/scripts/ros/run_reachability.sh) (+64 / −0)
- [scripts/ros/test_reachability.sh](https://github.com/Janga786/rebot-crack-vision/blob/965efea385110af6e8267156d8cb07b4902ca8ec/scripts/ros/test_reachability.sh) (+89 / −0)

Commits: [`c5b1a80`](https://github.com/Janga786/rebot-crack-vision/commit/c5b1a8095447386e38aeb35399c857291f6e428c), [`965efea`](https://github.com/Janga786/rebot-crack-vision/commit/965efea385110af6e8267156d8cb07b4902ca8ec)

## Why it was done

`reachability_sweep` queries move_group's /compute_ik for every target × orientation sample, with a surface collision slab and seed continuity. It re-checks every IK solution with /compute_fk and writes a §7 map. `scripts/ros/run_reachability.sh` wraps launch → sweep → teardown, and `scripts/ros/test_reachability.sh` proves it on a tiny fixture. The whole path is plan-free and execution-free.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **5/15** tasks accepted; whole project: **29/74**.
- REQ-MOT-1: 2/11 contributing tasks done
- REQ-MOT-2: 4/9 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, sweep-smoke ✔, no-execution-paths ✔, mock-plan-regression ✔); independent audit accepted it (criteria: 2 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair is narrow and correct. In _process_target, only NO_IK_SOLUTION and TIMED_OUT from /compute_ik now `continue` (they count toward unreachable). Every other non-SUCCESS code returns status 'error' with the MoveIt code name in 'error', and the existing counts.get('error') check turns that into exit 1. I re-ran the invalid-group repro: it now exits 1, all 6 targets have status error with 'compute_ik returned INVALID_GROUP_NAME …', and complete=True. The four scheduler checks pass when I re-run them. The working tree is unchanged and no stray processes were left behind
…[148 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task MOT-04.4.R1): Non-infeasibility /compute_ik error codes are silently classed as 'unreachable' (exit 0).

## What it unlocks next

- **Ready to start:** MOT-04.5 — Run the full reachability sweep, recommend + independently verify the specimen placement, document

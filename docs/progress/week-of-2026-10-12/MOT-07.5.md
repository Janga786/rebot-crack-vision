# MOT-07.5 — retime_plan CLI (ROS-free console script): plan → data/trajectories/{case}_trajectory.json + .retime.json

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`76f2c87`](https://github.com/Janga786/rebot-crack-vision/commit/76f2c8791ae6f314de2659f22e3f34f88c790160)

## What was done

Added the retime_plan command-line tool that turns a planned cartesian path into a timed robot trajectory file, following the project's §16.6 specification exactly. It is registered as both a ROS console command and a plain Python module, and a new 13-test suite (all passing) checks the success path, a safe dry-run preview mode, and every required error case, while the full existing 613-test suite for this ROS package still passes with no regressions.

After the first audit, a repair round (MOT-07.5.R1) fixed the reported issues: The retime_plan CLI's --dry-run preview now shows, for every leg, how many planned joint configurations it has and how many refined trajectory points it produced, not just timing — giving an operator a fuller preview before committing to a run. Also added an automated test proving that if the final safety check (full joint-limit verification) ever fails, the tool refuses to write any trajectory file and exits with the correct "precondition not met" code. All 14 unit tests, the workspace build, the CLI entry point, and the no-ROS-dependency check pass.

Files changed:
- [ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py](https://github.com/Janga786/rebot-crack-vision/blob/76f2c8791ae6f314de2659f22e3f34f88c790160/ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py) (+178 / −4)
- [ros2_ws/src/crackvision_motion/setup.py](https://github.com/Janga786/rebot-crack-vision/blob/76f2c8791ae6f314de2659f22e3f34f88c790160/ros2_ws/src/crackvision_motion/setup.py) (+1 / −0)
- [ros2_ws/src/crackvision_motion/test/test_retime_plan_cli.py](https://github.com/Janga786/rebot-crack-vision/blob/76f2c8791ae6f314de2659f22e3f34f88c790160/ros2_ws/src/crackvision_motion/test/test_retime_plan_cli.py) (+358 / −0)

Commits: [`a28479a`](https://github.com/Janga786/rebot-crack-vision/commit/a28479a0190bc6aec07b66318d6194cbf6126fe5), [`76f2c87`](https://github.com/Janga786/rebot-crack-vision/commit/76f2c8791ae6f314de2659f22e3f34f88c790160)

## Why it was done

`ros2 run crackvision_motion retime_plan` (and `python3 -m crackvision_motion.retime_plan`) retimes a cartesian plan per §16.6 with §0 flags, default paths under --root, --dry-run, atomic writes and exit codes 0/1/2/3, without importing rclpy.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **40/51** tasks accepted; whole project: **102/138**.
- REQ-MOT-2: 25/32 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, unit ✔, entry-point ✔, ros-free ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting this repair. Both findings are fixed. In `--dry-run` mode, every leg line now shows the configuration count (taken from the plan leg's positions), the refined point count (`last - first + 1`) and the duration. The refined count is correct because §16.4 says `point_range` includes both ends. The test checks these fields on all 4 leg lines. A new test monkeypatches `retime.check_limits`, which `retime.py:409` calls through the module global, so the patch does take effect. It then checks that `main` exits with code 3 and writes neither the trajectory nor the `.re
…[140 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task MOT-07.5.R1): No main(argv) test for the self-check-violation refusal (exit 3); --dry-run per-leg summary omits configurations and refined-point counts.

## What it unlocks next

- Closer: MOT-07.10 — Retime the view-plan transit leg and settle the §11.2 purpose for view moves (still needs MOT-07.7, MOT-07.9)
- Closer: MOT-07.8 — Offline cross-validation: retimer vs independent checker vs G-LIMITS on a committed synthetic plan, with fault injection (still needs MOT-07.7)

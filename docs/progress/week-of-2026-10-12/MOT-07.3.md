# MOT-07.3 — Retime core: retime config loader, plan polyline assembly, refinement and limit-constrained discrete time profile (pure Python)

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`a2209f4`](https://github.com/Janga786/rebot-crack-vision/commit/a2209f4725f958d090caedc85392a7fe67ed6d02)

## What was done

Built the core time-parameterization engine that turns an untimed robot-arm Cartesian plan into a timed joint trajectory respecting the arm's speed and acceleration limits, plus its config file and a 54-test suite. Verified it produces trajectories within about 1-11% of the theoretical fastest-possible timing on a straight move, and that it correctly slows the arm down at sharp corners, when approaching a joint's hard limit, and when given an external speed cap, with zero limit violations in every tested case. All 577 existing motion-package tests still pass, confirming no regressions.

Files changed:
- [config/motion/retime.yaml](https://github.com/Janga786/rebot-crack-vision/blob/a2209f4725f958d090caedc85392a7fe67ed6d02/config/motion/retime.yaml) (+31 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/retime_core.py](https://github.com/Janga786/rebot-crack-vision/blob/a2209f4725f958d090caedc85392a7fe67ed6d02/ros2_ws/src/crackvision_motion/crackvision_motion/retime_core.py) (+540 / −0)
- [ros2_ws/src/crackvision_motion/test/test_retime_core.py](https://github.com/Janga786/rebot-crack-vision/blob/a2209f4725f958d090caedc85392a7fe67ed6d02/ros2_ws/src/crackvision_motion/test/test_retime_core.py) (+518 / −0)

Commits: [`a2209f4`](https://github.com/Janga786/rebot-crack-vision/commit/a2209f4725f958d090caedc85392a7fe67ed6d02)

## Why it was done

crackvision_motion.retime_core loads config/motion/retime.yaml strictly, turns a validated cartesian plan into the exact refined joint polyline with leg spans, and computes segment durations satisfying §16's FD velocity/acceleration fractions, rest boundary, §11.6 margin rule and sample-period/step bounds — verified against joint_trajectory.check_limits on synthetic paths.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **37/51** tasks accepted; whole project: **99/138**.
- REQ-MOT-2: 22/32 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, regression ✔, config-loads ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “MOT-07.3 holds up under independent checking. I re-ran the unit and config-load checks: 54 passed, exit 0. The regression suite result (577 passed) is the scheduler's log; I did not re-run it. Beyond the shipped tests I ran adversarial probes through `joint_trajectory.check_limits` at full limits with margin 0.03, plus the step and period bounds. Every case had zero violations and τ ≤ 0.02 s: - a 180° reversal corner; - a 40-vertex zig-zag; - three random 20-vertex transits; - a path approaching and leaving joint2 = 0; - paths that start near the bound heading toward it, so
…[919 chars clipped]”

## What it unlocks next

- **Ready to start:** MOT-07.4 — Retimer: constant tool speed on trace legs + §11.2 trajectory and crackvision.retime_report/1 assembly with self-check

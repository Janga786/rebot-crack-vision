# MOT-07.11 — Retimer G-TOOLSPEED is infeasible on real plan_crack_path trace legs: tool-speed dips at joint-space vertices

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`bafe2cd`](https://github.com/Janga786/rebot-crack-vision/commit/bafe2cd58b77afd5a6ef6f1c950ac021165126bf)

## What was done

The retimer now handles a real crack-tracing plan that it used to refuse. It accepts short tool-speed dips where the joint path turns sharply (1–18° on the test plan), as long as each dip is at most 5 mm long and all dips together cover at most 5% of the trace. On the committed test plan this gives a constant tool speed of 11.2 mm/s (the floor is 5 mm/s) with zero joint-limit violations. The independent trajectory checker applies the same rule and agrees: all nine of its checks pass, and the cross-check catches all seven injected faults.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/bafe2cd58b77afd5a6ef6f1c950ac021165126bf/docs/INTERFACES.md) (+54 / −18)
- [docs/adr/019-time-parameterization.md](https://github.com/Janga786/rebot-crack-vision/blob/bafe2cd58b77afd5a6ef6f1c950ac021165126bf/docs/adr/019-time-parameterization.md) (+102 / −0)
- [ros2_ws/src/crackvision_motion/crackvision_motion/retime.py](https://github.com/Janga786/rebot-crack-vision/blob/bafe2cd58b77afd5a6ef6f1c950ac021165126bf/ros2_ws/src/crackvision_motion/crackvision_motion/retime.py) (+95 / −8)
- [ros2_ws/src/crackvision_motion/test/test_retime.py](https://github.com/Janga786/rebot-crack-vision/blob/bafe2cd58b77afd5a6ef6f1c950ac021165126bf/ros2_ws/src/crackvision_motion/test/test_retime.py) (+201 / −0)
- [src/crackvision/trajectory_check.py](https://github.com/Janga786/rebot-crack-vision/blob/bafe2cd58b77afd5a6ef6f1c950ac021165126bf/src/crackvision/trajectory_check.py) (+98 / −10)
- [tests/test_trajectory_check_path.py](https://github.com/Janga786/rebot-crack-vision/blob/bafe2cd58b77afd5a6ef6f1c950ac021165126bf/tests/test_trajectory_check_path.py) (+132 / −0)

Commits: [`bafe2cd`](https://github.com/Janga786/rebot-crack-vision/commit/bafe2cd58b77afd5a6ef6f1c950ac021165126bf)

## Why it was done

retime_plan, using the committed config/motion/retime.yaml, retimes the committed MOT-07.8 fixture (a verbatim run_cartesian_synthetic.sh plan) with exit 0 and some trace v_c ≥ trace.min_tool_speed_m_s. Joint limits, limit fractions and sampling stay as they are. Any change to G-TOOLSPEED semantics or §16.7 values is recorded in ADR-019 and INTERFACES §16 and mirrored in the independent checker's C-TOOLSPEED.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **25/52** tasks accepted; whole project: **79/139**.
- REQ-MOT-1: 20/41 contributing tasks done
- REQ-MOT-2: 13/33 contributing tasks done
- Verification: automated checks run by the pipeline itself (fixture-retimes ✔, motion-regression ✔, checker ✔); independent audit accepted it (criteria: 9 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “MOT-07.11 fixes G-TOOLSPEED's refusal of the real plan_crack_path trace leg via option (b): a documented rule amendment (vertex-tied dips bounded by max_ramp_m per-dip and speed_tolerance summed-share per leg). All three scheduler acceptance checks reproduce exactly as logged, the independent checker mirrors the retimer's rule without importing crackvision_motion, and scope/protected-config constraints are honored. I additionally ran the MOT-07.8 cross-check script (not a required check, but directly relevant evidence) and it reproduces the implementer's claimed v_c=0.01115
…[124 chars clipped]”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

# MOT-07.6 — Independent trajectory checker (conda side), part 1: schema, provenance, FD limits, §11.6 margin, rest and sampling + CLI

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`cf97e97`](https://github.com/Janga786/rebot-crack-vision/commit/cf97e97b6ac88f633d3edbdf1ce21d281e3517e5)

## What was done

Built the first half of the independent trajectory safety-checker for the robot arm's motion pipeline: a standalone tool (crackvision.trajectory_check) that re-derives, from the written specification alone and with zero shared code, whether a planned robot trajectory obeys the joint speed/acceleration limits, a safety margin near joint limits, smooth starts/stops from rest, and fine-enough time sampling. All 28 hand-built test cases pass, including the two worked examples from the project's own written specification (one that should pass and one that should fail), and the command-line tool correctly reports success/failure exit codes.

After the first audit, a repair round (MOT-07.6.R1) fixed the reported issues: The independent trajectory safety checker (crackvision.trajectory_check) now catches two previously-missed safety gaps: it rejects any planned trajectory whose start or end joint positions don't match the motion plan it's supposed to follow, and it rejects trajectories containing invalid NaN/Infinity joint values instead of silently passing them. All 32 tests pass, including new tests built specifically to prove these two cases are caught.

Files changed:
- [src/crackvision/trajectory_check.py](https://github.com/Janga786/rebot-crack-vision/blob/cf97e97b6ac88f633d3edbdf1ce21d281e3517e5/src/crackvision/trajectory_check.py) (+763 / −3)
- [tests/test_trajectory_check.py](https://github.com/Janga786/rebot-crack-vision/blob/cf97e97b6ac88f633d3edbdf1ce21d281e3517e5/tests/test_trajectory_check.py) (+554 / −0)

Commits: [`ad52b49`](https://github.com/Janga786/rebot-crack-vision/commit/ad52b490e5f8ebc27415c66e5ecde7220dc7171e), [`cf97e97`](https://github.com/Janga786/rebot-crack-vision/commit/cf97e97b6ac88f633d3edbdf1ce21d281e3517e5)

## Why it was done

`./env.sh python -m crackvision.trajectory_check` independently re-implements and reports C-SCHEMA, C-PROVENANCE, C-LIMITS, C-MARGIN, C-REST and C-SAMPLING from §16.5 for a trajectory + plan, writing a crackvision.trajectory_check/1 report with exit 0 pass / 3 fail / 2 usage, sharing no code with crackvision_motion.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits

## How it moves the project forward

- Motion planning: **40/51** tasks accepted; whole project: **102/138**.
- REQ-MOT-1: 33/40 contributing tasks done
- REQ-MOT-2: 24/32 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, independent ✔, cli-help ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Both review findings are fixed at cf97e97, and the repair changes nothing else. C-PROVENANCE now compares points[0] with the first configuration of the lowest-leg_index leg, and points[-1] with the last configuration of the highest-leg_index leg. The comparison is exact, which matches §16.5 C-PATH's bit-exact convention. On a mismatch it fails with a detail naming points[0] or points[-1]. In my probes this held for a two-leg plan listed out of order, and a matched end passed. _is_number now uses math.isfinite, so NaN or ±Infinity in t_s, positions, velocities, accelerations
…[543 chars clipped]”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task MOT-07.6.R1): C-PROVENANCE does not check trajectory start/end against the plan's first/last configuration; Non-finite positions (NaN/Infinity) pass C-SCHEMA and every other check; CLI exits 0.

## What it unlocks next

- **Ready to start:** MOT-07.7 — Independent trajectory checker, part 2: path fidelity, constant tool speed (crackvision.kinematics FK) and retime-report consistency

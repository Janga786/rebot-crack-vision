# Stale calibration and limit-violation safety evidence (INT-03.4)

## What "stale calibration" means here

A trajectory file (`crackvision.joint_trajectory/1`, `docs/INTERFACES.md` §11.2) carries two
declared fields, `scene_config_sha256` and `end_effector_config_sha256`, recorded at plan time.
These are not arbitrary hashes: they are the sha256 of `config/scene/scene.yaml` and
`config/robot/end_effector.yaml` *at the moment the trajectory was planned*, i.e. they encode the
committed hand-eye/TCP calibration outputs those files hold (produced upstream by GEOM-05/GEOM-07).
`G-STALE-CONFIG` (`ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py`,
`_evaluate_stale_config`, lines 895-924) re-hashes the *live* `config/scene/scene.yaml` and
`config/robot/end_effector.yaml` at execution time and compares them against the trajectory's
declared shas. If either declared sha is non-null and differs from the live file's sha, the
calibration the plan was built against is stale — the scene or end-effector config has moved on
since the plan was made, and the gate reports `fail`, which refuses in every mode (it does not wait
for `real`). If a declared sha is `null` instead, the gate still enforces mode-dependent behaviour.

This matches the exact per-mode outcome table in `docs/INTERFACES.md` §11.7's `G-STALE-CONFIG` row:

> `G-STALE-CONFIG` | offline | refuse | refuse | refuse | non-null `end_effector_config_sha256`/`scene_config_sha256` ≠ the current `config/robot/end_effector.yaml`/`config/scene/scene.yaml` sha256: **refuse in every mode** (replan). `null` either one: **warn** in mock/dry, **refuse** in real

So there are two distinct cases: an actual *mismatch* (sha present but stale) refuses `mock`, `dry`
and `real` alike and forces a replan; a *missing* declaration (`null`) is only a warning outside
`real` (`G-STALE-CONFIG` reports `warn`, which never refuses mock/dry per the general outcome
semantics in §11.7) but refuses in `real`. `_evaluate_stale_config`'s code mirrors this: the
`mismatches` branch returns `fail` unconditionally (lines 907-913), while the `nulls` branch returns
`fail` only when `mode == "real"` and `warn` otherwise (lines 915-923).

## What "limit violation" means here

`G-LIMITS` (§11.7 table) is the gate that catches a trajectory whose positions, velocities or
accelerations exceed `config/robot/b601_dm_limits.yaml`. Critically, it does not trust the
retimer's own self-reported compliance: it independently re-derives whether the file is within
limits by reading the limits file itself and recomputing bounds from the trajectory's own
`positions`/`velocities`/`accelerations`/`t_s` data (checked both from explicit
velocity/acceleration fields when present and always from finite differences, per §11.7's
`G-LIMITS` row and §11.6's margin policy). It checks the trajectory twice: unscaled (scale 1.0, as
written) and scaled (after retiming at the effective `speed_scale`), and refuses in every mode if
either check fails — "a file over the limits at scale 1.0 is refused whatever `s` is."

This is one layer of a multi-layer defence. The complementary evidence is MOT-07.8
(`docs/progress/week-of-2026-10-12/MOT-07.8.md`), which offline cross-validates three independent
implementations against each other on a committed synthetic plan: the motion retimer itself, an
independent checker (`scripts/ros/retime_fault_inject.py` / `scripts/ros/test_retime_crosscheck.sh`),
and `G-LIMITS`. MOT-07.8 injects 7 distinct faults (including overspeed/over-acceleration segments)
and asserts each is caught by its expected check with zero false negatives, and confirms the
checker and `G-LIMITS` agree on limit faults. That cross-check is why a limit violation surviving
one layer (e.g. a retimer bug that under-reports its own output) cannot silently reach execution:
`G-LIMITS` in `execute_trajectory` re-derives the answer from the limits file independently of
whatever the retimer claims, and MOT-07.8 is the offline evidence that this independent re-derivation
agrees with a third, separately-written checker under fault injection.

## This card's own run

Command run (the unmodified MOT-05.5 headless mock verification, `scripts/ros/test_execution.sh`):

```
bash scripts/ros/build_ws.sh && bash scripts/ros/test_execution.sh
```

Observed result: **exit 0** (overall script success; it builds, runs, and tears down the full
multi-phase mock suite).

Within phase 1 ("offline refusals, no ROS graph"), the two fixtures relevant to this card:

- **Stale-config fixture** — `--mode real` with `test/fixtures/trajectory_smoke.json` (which
  declares `end_effector_config_sha256: null` and `scene_config_sha256: null`): exit **3**, record
  outcome `refused`, and the refusal id set included `G-STALE-CONFIG` alongside `G-ARM`, `G-TRAJ`,
  `G-LIMITS-HASH`, `G-SCENE`, `G-EE`, `G-COMMISSIONING`, `G-ESTOP`, `G-APPROVAL`. This is the
  `null`-in-`real` branch of the §11.7 table (`fail`, because `real` requires non-null declared
  shas), observed directly in the captured log:

  ```
  2026-10-14 10:37:14,548 ERROR   crackvision_motion.execute_trajectory: offline gate refused: ['G-ARM', 'G-TRAJ', 'G-LIMITS-HASH', 'G-SCENE', 'G-EE', 'G-COMMISSIONING', 'G-ESTOP', 'G-APPROVAL', 'G-STALE-CONFIG']
  [ros2run]: Process exited with failure 3
     OK: exit 3, refusal ids match: ['G-APPROVAL', 'G-ARM', 'G-COMMISSIONING', 'G-EE', 'G-ESTOP', 'G-LIMITS-HASH', 'G-SCENE', 'G-STALE-CONFIG', 'G-TRAJ']
  ```

- **Overspeed fixture** — `--mode mock` with `test/fixtures/trajectory_overspeed.json`: exit **3**,
  record outcome `refused`, refusal id set `{G-LIMITS}` only, with no ROS contact attempted. Captured
  log:

  ```
  2026-10-14 10:37:15,293 ERROR   crackvision_motion.execute_trajectory: offline gate refused: ['G-LIMITS']
  [ros2run]: Process exited with failure 3
     OK: exit 3, G-LIMITS only
  ```

Both outcomes match the behaviour this document describes: a declared-but-missing calibration sha
is refused in `real` (and would only warn in `mock`/`dry`), and an overspeed trajectory is refused
by `G-LIMITS` alone, offline, before any ROS node is constructed — independent of, and in addition
to, the MOT-07.8 cross-validation of the same gate under fault injection.

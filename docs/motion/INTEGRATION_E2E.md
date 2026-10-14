# INTEGRATION_E2E.md — one-command synthetic-to-limits-checked-mock-execution run (INT-01.2)

**Date:** 2026-10-14. **Command:** `bash scripts/ros/run_integration_e2e.sh` (builds nothing
itself — run `bash scripts/ros/build_ws.sh` first). Example run's scratch roots:
`logs/integration_e2e/20261014T062848Z/` (this script's own work dir) plus
`logs/synthetic_perception/20261014T062848Z/` (the perception leg's scratch root, produced by
`scripts/perc/run_synthetic_perception.sh` and recovered from its stdout). Neither is committed;
`logs/` is git-ignored.

## 1. What this runs

One script chains the full pipeline, perception through a limits-checked mock execution, for the
first time:

```
tools/synth_scene3d.py                         (GEOM-08.8 generator, flat scene)
  -> crackvision.visualize                     (PERC-01-adjacent; stand-in prediction)
  -> crackvision.skeleton                      (TC-010/PERC-02)
  -> crackvision.paths                         (PERC-03)
  -> crackvision.visualize_paths                (PERC-04)
  -> crackvision.path3d                         (GEOM-08, docs/INTERFACES.md §10)
       [all six steps above via scripts/perc/run_synthetic_perception.sh (INT-01.1), unmodified]
  -> ros2 run crackvision_motion plan_view               (MOT-06, §13.14)
  -> ros2 run crackvision_motion plan_crack_path --require-full-coverage   (MOT-06, §13)
  -> ros2 run crackvision_motion retime_plan              (MOT-07, §16)
  -> ./env.sh python -m crackvision.trajectory_check      (MOT-07, §16.9, independent checker)
  -> ros2 run crackvision_motion execute_trajectory --mode dry  --profile moveit_mock   (MOT-05, §11)
  -> ros2 run crackvision_motion execute_trajectory --mode mock --profile moveit_mock   (MOT-05, §11)
```

The perception leg (`synth_scene3d.py` through `path3d`) runs through `./env.sh python`, exactly
as `scripts/perc/run_synthetic_perception.sh` runs it on its own (see `INT-01.1`'s own
documentation of that script). The planning/retime/execution leg runs under
`scripts/ros/env_ros.sh` plus the built `ros2_ws` overlay, same as
`docs/motion/CARTESIAN_PLANNING.md`/`docs/motion/TIME_PARAMETERIZATION.md`. The script builds its
own scratch `scene.yaml` from the generator's own ground truth (same pattern as
`run_cartesian_synthetic.sh`'s step 2 — `config/scene/scene.yaml` is never read), copies
`config/robot/end_effector.yaml`/`b601_dm_limits.yaml` into its scratch root (the planners always
resolve those two paths relative to `--root`), and passes the real
`config/motion/cartesian_planning.yaml`/`retime.yaml`/`execution.yaml` by absolute path unmodified.
Only `--mode dry`/`--mode mock`, profile `moveit_mock`, ever run; `CRACKVISION_ARM_REAL` is never
set.

## 2. Observed results (the run above)

| Step | Result |
|---|---|
| `crackvision.skeleton` | `mask_pixels=613`, **`skeleton_pixels=105`** (848x480 image, `skeleton_fraction≈0.000258`), 1 component before/after pruning |
| `crackvision.paths` | **1 component, 1 main path, 0 branches**, `main_path.length_px=104` |
| `crackvision.path3d` (paths3d) | **1 component, 1 segment, 105/105 points valid, 0 interpolated**, `execution_eligible=false`, reasons `wrist_camera_nominal, tool_nominal, optical_frames_nominal, synthetic_capture, uncertainty_exceeds_clearance` |
| `plan_view` | **1 candidate tried** (`tilt_deg=0.0, yaw_deg=0.0`), chosen on the first try |
| `plan_crack_path` | **1 segment**, **24 candidates tried**, chosen `direction=reverse, roll_offset_rad=1.047` (60°); **coverage 36/36 trace waypoints, fraction_by_length = 1.000000**, `--require-full-coverage` satisfied; `sources.paths3d.execution_eligible` carried through as `false` |
| `retime_plan` | `duration_s=13.872`, `n_points=12361`; `peak_velocity_fraction` (per joint) `[0.027, 0.376, 0.101, 0.195, 0.007, 0.663]`; `peak_acceleration_fraction` (per joint) `[0.947, 0.946, 0.950, 0.950, 0.009, 0.950]`; trace leg (leg_index 2) `tool_speed.constant=0.012646` m/s against a `0.05` m/s target (`min/max` `0.001538`/`0.012656` m/s), `ramp_in_m/ramp_out_m` `0.001513`/`0.000825` m; `self_check.violations=0` |
| `crackvision.trajectory_check` | all nine checks (`C-SCHEMA`, `C-PROVENANCE`, `C-LIMITS`, `C-MARGIN`, `C-REST`, `C-SAMPLING`, `C-PATH`, `C-TOOLSPEED`, `C-REPORT`) `pass`, exit 0 |
| `execute_trajectory --mode dry` | exit 0; `G-TRAJ`/`G-LIMITS-HASH`/`G-LIMITS`/`G-STALE-CONFIG`/`G-GRAPH`/`G-START-STATE`/`G-COLLISION` all `pass`; `G-SPEED`/`G-SCENE`/`G-EE`/`G-ELIGIBLE`/`G-APPROVAL` `warn` (nominal config, no approval — expected outside commissioning); `G-ARM`/`G-COMMISSIONING`/`G-ESTOP`/`G-CONFIRM` `skip` (real-only); `would_refuse_in_real` includes `G-ELIGIBLE` and `G-APPROVAL` |
| `execute_trajectory --mode mock` | exit 0, `outcome=completed`, arrival verified (`max|dq|=0.0` rad against the trajectory's final point) |
| Wall-clock (this run) | perception leg (`synth_scene3d.py` through `path3d`) ≈ 10 s; `plan_view` ≈ 1 s; `plan_crack_path` ≈ 160 s (24 IK/Cartesian/re-check candidate attempts); `retime_plan` ≈ 7 s; `trajectory_check` < 1 s; the two `execute_trajectory` calls + the home→start move ≈ a few seconds each on the mock stack |

Run `python3 -m json.tool ROOT/trajectory.json.retime.json` / the scratch root's `cartesian_plan.json`
/`view_plan.json` for the full documents; the script's own stdout prints the same headline numbers
on every run.

## 3. Caveats

- **Nominal camera/tool/scene geometry throughout.** `config/robot/end_effector.yaml`'s
  `wrist_camera`/`tool` blocks are still `value_status: nominal` (GEOM-05/GEOM-07 promote them),
  the capture is synthetic, and the scratch scene this script builds is an engineering construct
  from the generator's own ground truth, not a measured workcell (MOT-10). None of this run's
  outputs are a measurement of anything physical.
- **`execution_eligible` is `false` by design.** The synthetic `paths3d` document carries
  `wrist_camera_nominal, tool_nominal, optical_frames_nominal, synthetic_capture,
  uncertainty_exceeds_clearance` (same reasons `docs/motion/CARTESIAN_PLANNING.md`/
  `docs/motion/TIME_PARAMETERIZATION.md` already document for this chain); `plan_crack_path`
  carries that `false` through unchanged, and `execute_trajectory`'s `G-ELIGIBLE` warns outside
  `real` and would refuse in `real`.
- **Mock/dry only, no real-mode attempt.** Only `--mode dry`/`--mode mock`, profile
  `moveit_mock`, ever run; this script never sets `CRACKVISION_ARM_REAL` and never touches a real
  vendor driver (it refuses outright at start-up if one is already running).
- **Says nothing about the view-plan leg's own retime/execute.** `plan_view`'s `view_plan.json`
  is produced and validated here, but only `plan_crack_path`'s `cartesian_plan.json` is retimed
  and executed by this script; the view-plan transit leg's own retime/execute chain is covered
  separately by `scripts/ros/test_view_retime_execute.sh` (MOT-07.10.5,
  `docs/motion/TIME_PARAMETERIZATION.md` §7).
- **Single scenario.** Default flat scene (`--seed 0`, no `--tilt-deg`/`--view-tilt-deg`/
  `--holes`/`--depth-noise`): a continuous, unbroken crack with full coverage. This run says
  nothing about a tilted surface, partial/failed coverage, or the generator's hole-interpolation
  behaviour — those are covered directly by `docs/geometry/PATH3D_VERIFICATION.md` and
  `docs/motion/CARTESIAN_PLANNING.md`'s own caveats.

## 4. Operator commands

```bash
bash scripts/ros/build_ws.sh            # build the overlay once
bash scripts/ros/run_integration_e2e.sh # the whole chain above, headless, no hardware
```

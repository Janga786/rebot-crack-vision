# TIME_PARAMETERIZATION.md — MOT-07 retimer + headless mock execution (MOT-07.9)

**Date:** 2026-10-13. **Command:** `bash scripts/ros/test_retime_execute.sh` (builds nothing
itself — run `scripts/ros/build_ws.sh` first). Example run's scratch root:
`logs/cartesian_synthetic/20261013T111754Z/` (reused from `run_cartesian_synthetic.sh`, MOT-06.10);
script-level scratch output `logs/retime_execute/20261013T111754Z/` (neither is committed; `logs/`
is git-ignored).

## 1. Method, and why not TOTG/Ruckig

`docs/adr/019-time-parameterization.md` (ADR-019) is the normative source; this section only
restates the decision for an operator who has not read it.

MoveIt's Time-Optimal Trajectory Generation (TOTG) has no Python binding on this Humble install
(`moveit_py` is absent), and `ros-humble-ruckig`'s Debian package ships no importable Python
module under either interpreter this project uses. Beyond availability, both are a poor fit
anyway: TOTG fits `path_tolerance`-bounded splines through the input waypoints, deliberately
cutting corners to stay time-optimal — exactly the kind of unchecked excursion from an
already-collision-checked plan polyline (`plan_crack_path`'s own §13.8 independent re-check) this
project will not introduce. Ruckig's API is point-to-point, not multi-waypoint path-following, so
feeding it the plan's configurations one pair at a time would not guarantee the arm follows the
checked Cartesian polyline between them.

ADR-019 instead builds a pure-Python/numpy, ROS-free retimer
(`crackvision_motion.retime_plan`, §16 of `docs/INTERFACES.md`) that:

1. Concatenates the plan's leg polylines bit-exact (no corner blending — the geometric path is
   exactly the checked path, segment by segment, trading away TOTG's time-optimality for exact
   path fidelity).
2. Refines the polyline by linear subdivision only, bounding the per-joint step.
3. Runs a discrete forward–backward velocity-profiling pass over per-segment path speeds, at a
   configured fraction of `b601_dm_limits.yaml` (`limit_fraction.velocity`/`.acceleration`, 0.95
   each), enforcing a rest-to-rest start/end and `execution.yaml`'s position-margin rule.
4. Holds each trace leg at one near-constant tool-tip speed (bisected to the fastest value
   satisfying every other bound), using a new `tool_fk` module parity-tested against
   `crackvision.kinematics`.
5. Self-checks the result with `joint_trajectory.check_limits` at full (unscaled) limits before
   writing anything: any violation is a refusal (exit 3), never a silent clamp.

A separate, independent checker (`crackvision.trajectory_check`, conda environment, never imports
`crackvision_motion`) re-derives every one of §16's output guarantees from scratch, using FK from
`crackvision.kinematics` rather than the retimer's own `tool_fk`.

## 2. What this script runs

```
scripts/ros/run_cartesian_synthetic.sh                              (MOT-06.10, fresh scratch plan)
  -> ros2 run crackvision_motion retime_plan --plan ROOT/cartesian_plan.json --root ROOT
         --limits ROOT/config/robot/b601_dm_limits.yaml --end-effector-config ROOT/config/robot/end_effector.yaml
         --execution-config config/motion/execution.yaml --retime-config config/motion/retime.yaml
         --urdf presentation/sim/reBot_B601_DM_with_gripper.urdf --purpose crack_task  (§16.6)
  -> ./env.sh python -m crackvision.trajectory_check --trajectory ... --plan ... --report ...
         --limits ... --end-effector-config ... --execution-config ... --retime-config ...  (§16.9)
  -> [mock stack up] apply_scene --config ROOT/scene.yaml
  -> ros2 run crackvision_motion execute_trajectory --mode mock --profile moveit_mock
         --trajectory WORK/to_start.json ...                        (home(0) -> the plan's start_state)
  -> ros2 run crackvision_motion execute_trajectory --mode dry  --profile moveit_mock
         --trajectory ROOT/trajectory.json --root ROOT --scene-config ROOT/scene.yaml
         --execution-config config/motion/execution.yaml
  -> ros2 run crackvision_motion execute_trajectory --mode mock --profile moveit_mock
         --trajectory ROOT/trajectory.json --root ROOT --scene-config ROOT/scene.yaml
         --execution-config config/motion/execution.yaml
```

`--limits`/`--end-effector-config` are the scratch root's own copies (the ones the plan's
`sources.*` shas were recorded against); `--retime-config`/`--execution-config` are the real
production files at their repository paths, so this exercises the real policy, never a loosened
one. The `dry`/`mock` `execute_trajectory` calls pass `--root` (scratch, so `G-ELIGIBLE` can
resolve `source.paths3d` relative to it), `--scene-config` (the same scratch scene `apply_scene`
applied, so `G-COLLISION`'s expected object ids match what is actually in the live planning
scene) and `--execution-config` (the repo's real `execution.yaml`) explicitly — never a production
config edit.

**Getting the mock arm to the plan's start state first.** The plan's own `start_state` is a
synthetic `capture_record` joint state, not the mock stack's all-zero startup state, so
`G-START-STATE` would refuse the real trajectory immediately on a fresh stack. Per the card, the
script first runs one small, hand-written `purpose: "test"` trajectory
(home(all-zero) → the plan's start point, three ramped waypoints, the same pattern
`test_execution.sh`'s `trajectory_smoke.json` fixture uses) through `execute_trajectory --mode
mock --profile moveit_mock` — motion, but only ever against the mock stack, through the same
gated path everything else in this script uses, never a bypass of any gate.

Only `--mode dry`/`--mode mock`, profile `moveit_mock`, ever run. `CRACKVISION_ARM_REAL` is never
set, `--profile vendor`/`vendor_mock` are never used, and `scripts/ros/_mock_stack.sh`'s own `trap
stop_mock_stack EXIT INT TERM` tears the stack down on every exit path, including a failure
partway through.

## 3. Measured results (the run above)

`retime_plan` headline (`ROOT/trajectory.json.retime.json`):

| Field | Value |
|---|---|
| `duration_s` | 12.692 |
| `n_points` | 8845 |
| `peak_velocity_fraction` (max over joints) | 0.518 |
| `peak_acceleration_fraction` (max over joints) | 0.950 |
| trace leg 2: `v_c` (constant tool speed) | 0.011152 m/s |
| trace leg 2: `target` / `min` / `max` tool speed | 0.050000 / 0.001356 / 0.011162 m/s |
| trace leg 2: `ramp_in_m` / `ramp_out_m` | 0.001335 m / 0.000804 m |

`v_c` is well below the configured `trace.tool_speed_m_s` target (0.05 m/s): this synthetic
trace's joint-space vertex turns force the bisection down, per ADR-019's MOT-07.11 addendum
(vertex-tied tool-speed dips, G-TOOLSPEED rules 2–3). `trajectory_check` (conda, independent of
`crackvision_motion`) reports all nine checks (`C-SCHEMA`, `C-PROVENANCE`, `C-LIMITS`, `C-MARGIN`,
`C-REST`, `C-SAMPLING`, `C-PATH`, `C-TOOLSPEED`, `C-REPORT`) `pass`, exit 0.

`execute_trajectory --mode dry` (`ROOT/trajectory.json`, profile `moveit_mock`): exit 0,
`gate_report` — `G-TRAJ`/`G-LIMITS-HASH`/`G-LIMITS`/`G-STALE-CONFIG`/`G-GRAPH`/`G-START-STATE`/
`G-COLLISION` all `pass`; `G-SCENE`/`G-EE` `warn` (both configs load but carry `nominal`, not
`measured`, values — expected on a clean checkout); `G-ELIGIBLE` `warn` (`paths3d file reports
execution_eligible=false, reasons=['wrist_camera_nominal', 'tool_nominal',
'optical_frames_nominal', 'synthetic_capture', 'uncertainty_exceeds_clearance']`); `G-APPROVAL`
`warn` (no `--approval` given); `G-ARM`/`G-COMMISSIONING`/`G-ESTOP`/`G-CONFIRM` `skip`
(real-only, previewed into `real_preview[]`, not evaluated). `would_refuse_in_real` =
`["G-APPROVAL", "G-ARM", "G-COMMISSIONING", "G-EE", "G-ELIGIBLE", "G-ESTOP", "G-SCENE",
"G-SPEED"]` (§11.9's real-only preview ids plus every warn-outside-real gate) — `G-ELIGIBLE` and
`G-APPROVAL` are both present, as the card requires.

`execute_trajectory --mode mock` (same trajectory, same flags): exit 0, `outcome: "completed"`,
`G-LIMITS`/`G-STALE-CONFIG`/`G-COLLISION` all `pass`. Final `/joint_states` matched the
trajectory's last point within `0.0 rad` (well inside the `0.06 rad` arrival tolerance,
`ARRIVAL_TOL_RAD` in `execute_trajectory.py`) — arrival verified.

The execution record path for the real trajectory's mock run is
`logs/retime_execute/<stamp>/execution_records/<timestamp>.json` (the script's own `--record-dir`,
never committed).

## 4. Effective tool speed: `speed_scale × v_c`

`execute_trajectory`'s effective speed is always `speed_scale × (the trajectory's own commanded
FD velocities)`, which for a trace leg's constant-speed stretch is `speed_scale × v_c`.
`config/motion/execution.yaml`'s `speed_scale.default` (mock, used above) is `1.0`:
`--mode mock` ran this trajectory's trace leg at its full recorded `v_c = 0.011152` m/s.
`speed_scale.real_default` (the cautious first real-execution default, §11.7 `G-SPEED`) is
`0.10`: at the committed `config/motion/retime.yaml`'s `trace.tool_speed_m_s` target of
**0.05 m/s**, the effective real-mode tool speed would be `0.10 × 0.05 = 0.005 m/s` —
**5 mm/s**, per `retime.yaml`'s own `sources.trace` rationale and this script's own value above
confirming the formula against a real run (`0.10 × 0.011152 = 0.0011152` m/s for this particular
synthetic leg, whose vertex-tied dips pulled `v_c` well below the 0.05 m/s target).

## 5. Caveats

- **Nominal geometry.** `config/scene/scene.yaml`'s/`config/robot/end_effector.yaml`'s real
  entries are still `nominal`, not `measured` (GEOM-05/GEOM-07); this run uses the scratch scene
  `run_cartesian_synthetic.sh` builds from the synthetic generator's own ground truth, also
  `nominal`. `G-SCENE`/`G-EE` warn accordingly outside `real`, and would fail in `real` on an
  uncommissioned unit.
- **Ineligible input.** The synthetic `paths3d` document is `execution_eligible: false`
  (`wrist_camera_nominal`, `tool_nominal`, `optical_frames_nominal`, `synthetic_capture`,
  `uncertainty_exceeds_clearance`, carried through MOT-06.10's own validation) — `G-ELIGIBLE`
  warns outside `real` and would fail in `real`. This is the honest state of a wholly synthetic
  run; nothing here claims the retimed trajectory corresponds to a measured crack.
- **The view-plan gap (ADR-019 §16.8, named not solved).** `plan_view`'s `home → view pose`
  transit leg also needs retiming before it could ever execute, but `§11.2`'s `purpose` field only
  defines `crack_task` (requires `source.paths3d`) and `test` (refused unconditionally in `real`)
  — neither fits a view move today. This script retimes the `cartesian_plan` (crack-task) leg
  set only, exactly as `retime_plan` is scoped to do; it never attempts to retime or execute
  `view_plan.json`.
- **A hand-written "move to start" trajectory is still motion**, albeit mock-only and
  gate-checked the same as every other move in this script (see §2 above) — it is not a special
  bypass, but it is also not derived from any planned/collision-checked path of its own (its own
  `scene_config_sha256`/`end_effector_config_sha256`/`limits_file.sha256` are all `null`,
  `purpose: "test"`, so every gate it touches is `warn`-only outside `real` and it is refused
  unconditionally were it ever run in `real`).

## 6. Operator commands

```bash
bash scripts/ros/build_ws.sh              # build the overlay once
bash scripts/ros/test_retime_execute.sh   # the whole chain above, headless, no hardware
```

To inspect one run's artifacts afterward: `python3 -m json.tool
logs/retime_execute/<stamp>/execute_mock.log` for the full `execute_trajectory` console output,
`logs/retime_execute/<stamp>/headline.log` for the `retime_plan` headline numbers, and
`logs/cartesian_synthetic/<stamp>/trajectory.json.retime.json` for the full retime report.

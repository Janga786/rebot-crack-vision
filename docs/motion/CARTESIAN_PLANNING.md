# CARTESIAN_PLANNING.md — synthetic end-to-end Cartesian planning run (MOT-06.10)

**Date:** 2026-10-11. **Command:** `bash scripts/ros/run_cartesian_synthetic.sh` (builds nothing
itself — run `scripts/ros/build_ws.sh` first). Example run's scratch root:
`logs/cartesian_synthetic/20261011T160200Z/` (never committed; `logs/` is git-ignored).

## 1. What this runs

One script chains the real pipeline, end to end, for the first time:

```
tools/synth_scene3d.py --out ROOT --seed 0 --case-id synth_mot06   (GEOM-08.8 generator, flat scene)
  -> ./env.sh python -m crackvision.paths   --root ROOT --cases synth_mot06    (docs/INTERFACES.md §3.13)
  -> ./env.sh python -m crackvision.path3d  --root ROOT --cases synth_mot06    (docs/INTERFACES.md §10)
  -> ros2 run crackvision_motion plan_view        --scene-config ROOT/scene.yaml   (§13.14)
  -> ros2 run crackvision_motion plan_crack_path   --paths3d ROOT/data/paths3d/synth_mot06_paths3d.json
                                                     --scene-config ROOT/scene.yaml  --require-full-coverage (§13)
```

Everything happens under one scratch root `logs/cartesian_synthetic/<UTC stamp>/`, never under
`data/` in the repo tree. Perception/geometry steps (`synth_scene3d.py`, `crackvision.paths`,
`crackvision.path3d`) run through `./env.sh python`; ROS steps (`plan_view`, `plan_crack_path`,
the mock stack) run under `scripts/ros/env_ros.sh` + the built `ros2_ws` overlay, per project
convention (ROS Humble's py3.10 vs. this repo's conda py3.11, ADR-007).

The scratch scene (`ROOT/scene.yaml`, a `crackvision.scene_config/1` table + specimen box, both
`value_status: nominal`) is built *from the generator's own ground truth*
(`ROOT/data/synthetic/synth_mot06_gt.json`'s `center_base_m`/`normal_base`), not from
`config/scene/scene.yaml` or `config/motion/specimen_placement.yaml`: the specimen's top face is
placed at exactly `center_base_m[2]` and its centre at `center_base_m[:2]`, so the scene the
planners see is geometrically consistent with the synthetic crack surface by construction. The
production `config/motion/cartesian_planning.yaml` is used unmodified (passed by absolute path);
`config/robot/end_effector.yaml` and `config/robot/b601_dm_limits.yaml` are copied read-only into
the scratch root's `config/robot/` because `plan_view`/`plan_crack_path` always resolve those two
paths relative to `--root`.

## 2. Observed results (the run above)

| Step | Result |
|---|---|
| `synth_scene3d.py` | `ik_residual_norm=1.4e-17`, `roll_deg=270.0`, `chain_px=105` — camera-view IK converges, flat surface (`normal_base=[0,0,1]`), fallback specimen centre `(0.29, 0.0, -0.01)` (`config/motion/specimen_placement.yaml` is not copied into the scratch root, so `synth_scene3d.specimen_center` takes its ADR-014 §3 nominal fallback — see `tools/synth_scene3d.py`'s own `specimen_center()` docstring) |
| `crackvision.paths` → `crackvision.path3d` | 1 component, 1 main polyline, 1 segment, **105/105 points valid, 0 interpolated**, `paths3d.execution_eligible=False` with reasons `wrist_camera_nominal, tool_nominal, optical_frames_nominal, synthetic_capture, uncertainty_exceeds_clearance` |
| `plan_view` | **1 candidate tried** (`tilt_deg=0.0, yaw_deg=0.0`), chosen on the first try: FOV margins `[16.7°, 16.7°, 16.7°, 16.7°]` horizontal / `[2.2°, 2.2°, 2.2°, 2.2°]` vertical (all positive), reachable, recheck passed |
| `plan_crack_path` | **1 segment, status `full`**; **24 candidates tried**, chosen `direction=reverse, roll_offset_rad=1.047` (60°); **coverage 36/36 trace waypoints, fraction_by_length = 1.000000, `complete: true`**; `sources.paths3d.execution_eligible` carried through as `false` |
| Wall-clock (this run) | synth+paths+path3d ≈ 9 s; `plan_view` ≈ 1 s; `plan_crack_path` ≈ 160 s (24 IK/Cartesian/re-check candidate attempts over 36 trace waypoints + approach/retract/transit legs) |

Run `python3 -m json.tool ROOT/cartesian_plan.json` / `ROOT/view_plan.json` for the full documents;
the script's own stdout prints the same headline numbers on every run.

## 3. Caveats

- **Nominal camera/tool/scene geometry throughout.** `config/robot/end_effector.yaml`'s
  `wrist_camera`/`tool` blocks are still `value_status: nominal` (GEOM-05/GEOM-07 promote them),
  the capture is synthetic, and the scratch scene above is an engineering construct, not a
  measured workcell (MOT-10). None of this run's outputs are a measurement of anything physical.
- **Ineligible for real execution by design.** `paths3d.execution_eligible` is `false`
  (`wrist_camera_nominal`, `tool_nominal`, `optical_frames_nominal`, `synthetic_capture`,
  `uncertainty_exceeds_clearance`), and `cartesian_plan.sources.paths3d.execution_eligible` carries
  that `false` through unchanged — `plan_crack_path` never claims eligibility it wasn't given.
  `MOT-05`'s commissioning-gated executor refuses any plan built from an ineligible `paths3d` file.
- **The production `config/scene/scene.yaml` specimen disagrees with the recommended placement.**
  `config/motion/specimen_placement.yaml`'s feasible recommendation is centre `(0.26, 0.0)`,
  surface `z = 0.0`; the production `config/scene/scene.yaml` specimen box is centred at `(0.30,
  0.0)` with top face `z = 0.01` — both the 0.04 m lateral offset and the 0.01 m height offset are
  far above `cartesian_planning.yaml`'s `surface_object_tolerance_m` (0.002 m). This is why
  `run_cartesian_synthetic.sh` builds its own scratch scene instead of using
  `config/scene/scene.yaml`: planning this (or any other) synthetic case's `paths3d` against the
  production scene reports every segment `surface_scene_mismatch` by design, not a bug — until
  MOT-10 measures the real cell and reconciles `specimen_placement.yaml` with `scene.yaml`, that
  mismatch is the correct, honest answer for the production scene specifically.
- **Ordering is not optimised.** A single-component, single-segment case has nothing to order;
  this run says nothing about `crackvision.paths`' multi-component greedy nearest-endpoint tour
  (§3.13 point 2) under Cartesian planning.
- **Untimed output for MOT-07.** `cartesian_plan.json`'s `legs[]` is one continuous joint-space
  path (`transit → approach → trace → retract → ... → end_state`), but it carries no
  timestamps/velocities — never a valid `execute_trajectory --trajectory` input on its own. MOT-07
  owns turning these legs into a single time-parameterized trajectory (TOTG) within
  `config/robot/b601_dm_limits.yaml`'s vel/acc bounds and retiming it per docs/INTERFACES.md §11.2
  before `execute_trajectory` (MOT-05) can run it — even in mock, let alone real hardware.
- **Specific to this run's one scenario.** Default flat scene (`--seed 0`, no `--tilt-deg`/
  `--view-tilt-deg`/`--holes`/`--depth-noise`): a continuous, unbroken, non-cavity-biased crack
  with full coverage. It says nothing about the hole-interpolation/segment-split behaviour GEOM-08.8
  already verified directly against `crackvision.path3d` (`docs/geometry/PATH3D_VERIFICATION.md`),
  nor about a tilted surface or a partial-coverage/failed segment under Cartesian planning.

## 4. Known cross-system bug worked around by this script

The real `crackvision.path3d` writer emits `components[].component_id` as an **int**
(`docs/INTERFACES.md` §3.13's own normative example: `"component_id":0`, and confirmed by this
run's actual output), but `ros2_ws/.../crackvision_motion/cartesian_core.py`'s `_validate_paths3d`
requires it to be a non-empty **string** and raises `Paths3dError` otherwise. Every existing
`crackvision_motion` cartesian-plan test/fixture (`ros2_ws/src/crackvision_motion/test/fixtures/
cartesian/*/paths3d.json`, `scripts/ros/test_plan_trace_full.sh`'s fixture) hand-writes string
component ids, so nothing before this card ever fed the real writer's output through the real
loader — this is the first script that does. `run_cartesian_synthetic.sh` works around it by
rewriting `component_id` to `str(component_id)` in its own scratch copy of the generated
`paths3d.json` before planning (see the script's own top-of-file comment); it changes no contract,
no committed fixture and no file outside that run's scratch root. A real fix (either
`crackvision.path3d` should write a string, or `cartesian_core._validate_paths3d` should accept an
int, whichever `docs/INTERFACES.md` §10/§3.13 is meant to normatively require) is cross-team and
out of this card's scope — see discoveries.

## 5. MOT-07 handoff

`cartesian_plan.json`'s `legs[]` (this run: 5 legs — `transit, approach, trace, retract`, plus the
trailing `end_state` transit) is the contract MOT-07 consumes: one continuous joint-space path,
`positions` per leg already continuous leg-to-leg (`docs/INTERFACES.md` §13.12/§13.13), each
configuration already collision/limit-checked by `plan_crack_path`'s own independent re-check
(§13.8). MOT-07's job is exactly: concatenate `legs[].positions` into one trajectory, run
time-optimal trajectory generation (TOTG) against `config/robot/b601_dm_limits.yaml`'s velocity/
acceleration bounds (§11.2), and hand the result to `execute_trajectory` (MOT-05) — which will
still refuse real execution because `sources.paths3d.execution_eligible` is `false` here, same as
every other nominal/synthetic input today.

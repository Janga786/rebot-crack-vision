# Unreachable-path failure injection (INT-03.3)

## Fault class

A crack placed far outside the B601-DM's reachability envelope (`docs/motion/REACHABILITY.md`):
every Cartesian-trace/view candidate the planners try must fail IK/FK agreement for every segment.
`plan_view`/`plan_crack_path` (MOT-06) must refuse cleanly — the documented exit code, no plan file
that a downstream consumer could mistake for a valid, committable plan, no hang past their own
documented service/candidate budget, and no unhandled exception — instead of silently producing a
bogus plan.

## Injection method

`scripts/ros/test_safety_unreachable.sh`:

1. Generates one synthetic case (`tools/synth_scene3d.py` --seed 0, `case-id synth_unreachable`),
   then the real `crackvision.paths`/`crackvision.path3d` CLIs — the same front end
   `run_cartesian_synthetic.sh` (MOT-06.10) uses — producing `data/paths3d/synth_unreachable_paths3d.json`
   with the crack at its normal, reachable placement (centred near `(0.26, 0.00)` m, matching
   `docs/motion/specimen_placement.yaml`).
2. **Fault injection**: translates every base-frame 3D point this script's downstream planners
   actually consume by `dx = +5.0` m in `x`:
   - `polylines[].points[].p_base_m` (the raw per-point positions), and
   - `polylines[].segments[].waypoints[].position_m` / `surface_point_m` — the structure
     `cartesian_core.segment_tasks()` actually reads (a separately-derived structure from
     `points[]`; translating only the latter was tried first and left the planner targeting the
     original, reachable pose — see "What was found" below).
   Also applies the existing `component_id` int-&gt;string workaround
   (`run_cartesian_synthetic.sh`'s own documented cross-system mismatch) to this scratch copy.
   `docs/motion/REACHABILITY.md`'s own sweep only covers `x in [0.11, 0.50]` m and its own
   `reach_bound_from_urdf` IK prefilter bound is 0.962 m from `base_link` (§2/§8); `+5.0` m puts
   every translated point about 4.5x past even that generous prefilter bound.
3. Builds a scratch `scene_config/1` (table + specimen) from the generator's own ground truth,
   with the specimen/table centre shifted by the same `+5.0` m so the scene stays geometrically
   consistent with the translated crack (this test must fail on unreachability, never on an
   unrelated `surface_scene_mismatch`).
4. Brings up the headless `moveit_mock` stack (`scripts/ros/_mock_stack.sh`, the same pattern as
   `run_cartesian_synthetic.sh`/`run_integration_e2e.sh`), waits for all six MoveIt services.
5. Computes each tool's own documented time budget directly from
   `config/motion/cartesian_planning.yaml` (`service_timeout_s`, the view `tilt`/`yaw` candidate
   counts, the `(direction x roll_offset)` candidate count, `max_splits_per_segment`) — never an
   arbitrary wrapper timeout.
6. Runs `ros2 run crackvision_motion plan_view --center 5.26 0.0 ...` and then
   `ros2 run crackvision_motion plan_crack_path --paths3d <translated> --require-full-coverage ...`
   against the translated scene, asserting for each: the exit code, that no plan file was written,
   that the expected refusal reason appears in its log, that elapsed wall time is within its
   computed budget, and (after both runs) that neither log contains an unhandled-exception
   traceback.
7. Tears down the mock stack on exit via `_mock_stack.sh`'s own SIGINT-then-SIGKILL process-group
   cleanup (`trap ... EXIT`-equivalent teardown called explicitly at the end of this script, same
   as the other MOT-06/INT-01 scripts).

## What was found and fixed

The first attempt translated only `points[].p_base_m` and left `segments[].waypoints[].position_m`
untouched. `cartesian_core.segment_tasks()` (the function `plan_crack_path` actually calls to build
its per-segment trace) reads positions from `segments[].waypoints[]`, a structure derived from, but
distinct from, `polylines[].points[]`. With only `points[]` translated, `plan_crack_path` still
targeted the original, reachable pose near `(0.26, 0.00)` m and produced a normal, valid
`cartesian_plan.json` (`coverage.complete: true`, FK recheck errors around 1e-4-1e-3 m) in 160 s —
not a planner safety gap, but a gap in this test's own injection coverage of the paths3d schema.
Translating both point sets together fixed it: `plan_crack_path` now reports `ik_failed`/zero
planned waypoints for the translated case and refuses, as expected. No change to
`plan_crack_path.py`/`plan_view.py` was needed — both already implement the exact `CartesianConfigError`
/`PreconditionError`/exit-code idiom this card expected, confirmed by reading the source (see below)
before writing any assertion.

## Expected and observed exit codes

Read directly from source before writing the test (never assumed):

- `plan_view.py` `_run()`: if no `(tilt, yaw)` candidate passes FOV + IK/FK + transit re-check,
  `chosen` stays `None` and `_run` raises `PreconditionError("no view candidate selected (§13.14
  Selection): ...")` — strictly before `write_view_plan` is ever called.
- `plan_crack_path.py` `_run()`: after the whole paths3d file is attempted,
  `if plan["coverage"]["trace_waypoints_planned"] == 0: raise PreconditionError("zero trace
  waypoints planned across the whole paths3d file")` — strictly before `write_plan` is ever called.
- `cli_common.py` `run_cli()`: `except PreconditionError: status, exit_code = "precondition",
  EXIT_PRECONDITION` (`EXIT_PRECONDITION = 3`). Any other, unanticipated exception is caught by
  `run_cli`'s own catch-all (`except Exception`), logged, and mapped to `EXIT_RUNTIME` (1) — it
  never escapes as a raw traceback.

Observed (`bash scripts/ros/build_ws.sh && bash scripts/ros/test_safety_unreachable.sh`):

```
test_safety_unreachable.sh: == plan_view (expect refusal) ==
...ERROR   crackvision_motion.plan_view: no view candidate selected (§13.14 Selection): tilt=0.0 yaw=0.0: ik_failed; tilt=0.0 yaw=90.0: ik_failed; tilt=0.0 yaw=180.0: ik_failed; tilt=0.0 yaw=270.0: ik_failed; tilt=15.0 yaw=0.0: fov_failed; tilt=15.0 yaw=90.0: fov_failed; tilt=15.0 yaw=180.0: fov_failed; tilt=15.0 yaw=270.0: fov_failed
[ros2run]: Process exited with failure 3
test_safety_unreachable.sh: plan_view exit=3 elapsed=1s

test_safety_unreachable.sh: == plan_crack_path (expect refusal) ==
...INFO    crackvision_motion.plan_crack_path: segment {...}: planned 0/36 trace waypoints
...ERROR   crackvision_motion.plan_crack_path: zero trace waypoints planned across the whole paths3d file
[ros2run]: Process exited with failure 3
test_safety_unreachable.sh: plan_crack_path exit=3 elapsed=2s

test_safety_unreachable.sh: OK (logs/safety_unreachable/<stamp>)
```

Both tools exited **3** (`EXIT_PRECONDITION`), as documented. Neither `view_plan.json` nor
`cartesian_plan.json` was written (the script asserts their absence explicitly after each run).
Both runs completed in 1-2 s, far inside their computed documented budgets (`plan_view` budget
720 s = 2 tilt x 4 yaw x 6 calls/candidate x 15.0 s `service_timeout_s`; `plan_crack_path` budget
8640 s = 4 split-rounds x 24 `(direction, roll)` candidates x 6 calls/candidate x 15.0 s — both
deliberately generous upper bounds on the config's own per-call wait, since a failing IK/FOV
candidate normally returns in well under one `service_timeout_s`). Neither log contains an
unhandled-exception traceback.

## Safe-refusal evidence (for INT-03.5)

- Both planners detect "no feasible candidate for any segment/view" using their own existing,
  already-committed `PreconditionError` idiom — no new error path, no code change, was required.
- Neither planner ever calls `write_plan`/`write_view_plan` on the refusal path: confirmed both by
  reading the source (the raise happens strictly before the write call in both files) and by this
  script explicitly asserting the output files do not exist after each refusal.
- Refusal is fast (1-2 s observed here) and, even under the config's own worst-case per-call
  timeout assumption, provably bounded — never an unbounded hang.
- No traceback/unhandled exception reaches the operator; the failure is reported as a normal,
  logged `ERROR` line plus exit code 3, exactly like any other documented precondition failure
  (§0.2).

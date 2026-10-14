# Invalid-depth failure injection (INT-03.2)

## Fault class

The depth sensor's documented "no return" condition: the D405's `0` sentinel count in an aligned
`uint16` z16 depth image (`geometry.classify_depth`'s `REASON_ZERO`, ADR-012). This happens for real
on specular/absorbent surfaces, out-of-range geometry, or a sensor glitch, and is not a malformed
file — it is a value the depth stream can legitimately produce. The geometry-lifting chain
(`crackvision.path3d`, via `crackvision.lift3d`) must never fabricate a 3D point from it, must
record a populated reason on every point it cannot lift, and must correctly mark the resulting
case ineligible for execution, whether the corruption covers the whole frame or only part of it.

## Injection method

`scripts/perc/test_safety_invalid_depth.sh`:

1. Generates one synthetic case with a real crack via `tools/synth_scene3d.py` (GEOM-08.8,
   unmodified) and the same `case_map.json`/stand-in-prediction fixup
   `scripts/perc/run_synthetic_perception.sh` (INT-01.1) uses, under a `seed/` scratch root.
2. Runs `crackvision.skeleton` and `crackvision.paths` normally over that case, producing a real,
   non-empty skeleton and `data/paths/{case}_paths.json` — confirmed by asserting
   `skeleton_pixels > 0` before corrupting anything.
3. Copies the seeded case tree into two independent scratch roots, `full/` and `partial/`, so the
   two corruption runs differ only in their depth image, never in the mask or `paths.json`.
4. **Fault injection**: reads each root's §9 capture record to find `depth.file`, loads that PNG
   (`uint16`, same `(height, width)` as the colour/mask images), and overwrites it in place with
   the device's documented `0` "no-return" sentinel:
   - `full/`: every pixel set to `0` (100% invalid).
   - `partial/`: a 24-row band centred on the crack path's middle row set to `0` (~5% of the frame
     by area, but deliberately wide enough — more than the annulus outer radius of 8 px on each
     side, and more than `max_gap_px` (5) of path row-run — to invalidate a real, contiguous run of
     path points rather than being absorbed by annulus sampling's own `min_fraction_valid`
     tolerance (0.3) or bridged by the §10.3 gap-interpolation policy).
5. Runs, exactly as an operator would from the CLI, against each corrupted root independently:
   ```
   ./env.sh python -m crackvision.path3d --root "$FULL_ROOT"    --cases safety_invalid_depth -q
   ./env.sh python -m crackvision.path3d --root "$PARTIAL_ROOT" --cases safety_invalid_depth -q
   ```
   capturing stderr to its own log file per run.
6. Greps both captured stderr logs for `Traceback (most recent call last)` and fails loudly if
   found.
7. Before asserting an exit code, reads `src/crackvision/path3d.py`'s own `build_case_paths3d`
   source (rather than assuming) to confirm it never classifies a zero-valid-points case as
   `status: "failed"` — that status is reserved for a missing/malformed capture record,
   `paths.json`, or `case_map.json` entry, not for `no_valid_points`, which is instead recorded as
   one of `execution_eligible`'s `ineligible_reasons` in an otherwise-normal, fully-written output.
8. Parses both `data/paths3d/{case}_paths3d.json` files and asserts the output below.

## Expected and observed exit codes

Per `docs/INTERFACES.md` #0.2 and `path3d.py`'s own contract (confirmed from source per step 7
above, not assumed): a case whose depth is entirely or partially invalid is a normal per-case
outcome — the module still writes a complete, schema-valid `paths3d.json`, just with
`execution_eligible: false` — not a batch failure. `path3d.main()` only reaches `EXIT_PRECONDITION`
when every requested case fails outright (missing capture/`paths.json`/`case_map.json` entry) and
only reaches `EXIT_RUNTIME` when at least one requested case does fail outright; neither applies
here, so both runs are expected at `EXIT_OK` (0).

Observed on a run of `bash scripts/perc/test_safety_invalid_depth.sh`:

| Run | Expected exit | Observed exit |
|---|---|---|
| `crackvision.path3d` (100% invalid) | 0 (`EXIT_OK`) | 0 |
| `crackvision.path3d` (partial/banded) | 0 (`EXIT_OK`) | 0 |

No `Traceback` was present in either run's stderr.

## Observed output

### 100%-invalid run

All 105 dense points of the single polyline came back `valid: false`, each with a non-empty
`reason` (`insufficient_valid_fraction` / `no_valid_depth_in_annulus` — the annulus sampled around
every path pixel found zero usable depth, since the whole frame is the `0` sentinel) and every
geometry field (`depth_m`, `p_optical_m`, `p_base_m`, `n_base`, `normal_rms_m`,
`sigma_normal_rad`, `cov_base_m2`, `sigma_base_m`) `null` — never a point at the camera origin or
any other sentinel coordinate (`docs/INTERFACES.md` §10.2). `counts.valid == 0`,
`ineligible_reasons` contains `no_valid_points`, and `execution_eligible == false`:

```
ineligible_reasons=['wrist_camera_nominal', 'tool_nominal', 'optical_frames_nominal',
                     'synthetic_capture', 'no_valid_points']
```

### Partial (banded) run

105 points total; the 24-row band intersected the annulus of 19 path points, which came back
invalid (`insufficient_valid_fraction`: 11, `no_valid_depth_in_annulus`: 8 — `counts.valid == 86`).
Per `docs/geometry/PATH3D_VERIFICATION.md`'s §10.3 gap policy, a run of invalid points longer than
`max_gap_px` (5) is **not** bridged by interpolation — it splits the polyline into separate
`segments[]`. That is exactly what was observed: `counts.interpolated == 0` and the single
component's polyline came back with **2** `segments[]` (verified directly against the written
JSON, not inferred), confirming the oversized invalid run triggered a split rather than a silent
bridge. Every non-interpolated invalid point still carried a populated `reason` and
null `p_optical_m`/`p_base_m`/`n_base` — no fabricated coordinate for an invalid pixel. This run's
`execution_eligible` was also `false` (`uncertainty_exceeds_clearance` and the usual
nominal-calibration reasons for a synthetic capture), independent of the depth corruption, since
no real calibration/camera-TF data exists for this synthetic case.

## Why this is a safe refusal

- **No fabricated geometry.** In both runs, every point the annulus sampling or the plane fit
  could not support is `valid: false` with a populated, specific `reason`, and all eight geometry
  fields are `null` — confirmed directly against the on-disk JSON, not inferred from exit codes or
  log lines. `geometry.classify_depth` and `geometry.sample_surface_depth_annulus` never collapse
  an invalid depth to `0`/the camera origin; `np.nan` is carried through to JSON `null` by
  `path3d.py`'s own `_vec_or_none` helper.
- **Correct ineligibility.** The 100%-invalid case is marked `no_valid_points` and
  `execution_eligible: false`; `MOT-05` (`docs/INTERFACES.md` §10.6) must refuse real execution of
  any file reporting `execution_eligible: false`, so a fully-invalid-depth capture can never reach
  the arm.
- **No crash, no silent masking.** Neither run raised an unhandled exception (`path3d.py`'s own
  try/except boundaries around capture/mask/paths.json loading are the only places exceptions are
  caught, and none fired here because this is modelled as *data*, not a malformed file); the
  partial run's oversized invalid run correctly became two segments per the already-documented
  §10.3 policy, rather than being silently bridged across 24 missing rows.
- **Reused, not duplicated.** This exercises the exact `classify_depth` / `sample_surface_depth_annulus`
  / gap-policy code paths already verified against synthetic ground truth in
  `docs/geometry/PATH3D_VERIFICATION.md`; no change was needed in `path3d.py` or `lift3d.py` to
  pass this test — the existing invalid-depth handling already satisfies the card's acceptance
  criteria.

Citable by INT-03.5 as evidence that the perception+geometry chain safely refuses, rather than
fabricates, in the face of invalid depth from the real D405 sensor's own documented failure mode.

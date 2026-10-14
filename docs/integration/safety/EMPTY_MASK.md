# Empty-mask failure injection (INT-03.1)

## Fault class

A segmentation model that predicts **no crack** at all for an otherwise valid image: an all-zero
prediction mask. This is a legitimate real-world outcome (a clean surface), not malformed input,
and the perception chain (`crackvision.skeleton` -> `crackvision.paths` -> `crackvision.path3d`)
must carry it through to a clean, schema-valid "zero cracks found" result without crashing, without
mis-reporting it as a runtime/precondition error, and without fabricating a crack, a point, or a
false execution-eligibility claim.

## Injection method

`scripts/perc/test_safety_empty_mask.sh`:

1. Generates one synthetic case with `tools/synth_scene3d.py` (GEOM-08.8, unmodified) — this
   produces the §9 capture record, `data/synthetic/{case}_color.png`/`_depth.png`/`_gt.json`, and a
   minimal `case_map.json` entry, exactly as `scripts/perc/run_synthetic_perception.sh` (INT-01.1)
   does.
2. Completes the `case_map.json` entry to the full `docs/INTERFACES.md` §2 schema (same fixup as
   INT-01.1).
3. **Fault injection**: instead of copying the generator's dilated ground-truth mask to
   `naming.prediction_path(case_id)` (the real stand-in used by INT-01.1), writes an all-zero PNG of
   the exact same `(height, width)` as the source image to that path. This is a well-formed,
   correctly-shaped, all-background mask — the one deliberate substitution for a real OpenCrack
   inference that found nothing.
4. Runs, exactly as an operator would from the CLI:
   ```
   ./env.sh python -m crackvision.skeleton --root "$ROOT" --cases safety_empty_mask -q
   ./env.sh python -m crackvision.paths    --root "$ROOT" --cases safety_empty_mask -q
   ./env.sh python -m crackvision.path3d   --root "$ROOT" --cases safety_empty_mask -q
   ```
   under a fresh `logs/safety_empty_mask/<UTC stamp>/` scratch root, capturing each step's stderr to
   its own log file.
5. Greps every captured stderr for `Traceback (most recent call last)` and fails loudly if found.
6. Parses `data/skeletons/{case}_skeleton_stats.json`, `data/paths/{case}_paths.json` and
   `data/paths3d/{case}_paths3d.json` and asserts the exact output below.

## Expected and observed exit codes

Per `docs/INTERFACES.md` #0.2 and each module's own exit-code idiom (`EXIT_OK`/`EXIT_PRECONDITION`/
`EXIT_RUNTIME`/`EXIT_USAGE` in `src/crackvision/{skeleton,paths,path3d}.py`): a crack-free prediction
is a normal per-case outcome, not a batch failure, so every step's own `main()` reaches its
`summary.write(..., "ok", EXIT_OK)` / `return EXIT_OK` path. All three modules read the shared
`case_map.json` successfully, find the one requested case, process it without raising, and report
zero `failed` cases in their `RunSummary` — the condition that would otherwise push them to
`EXIT_PRECONDITION` (no `case_map.json`, or all requested cases unknown/failed) or `EXIT_RUNTIME`
(any requested case transitions to `status: failed`) never triggers for an all-zero mask.

Observed on a run of `bash scripts/perc/test_safety_empty_mask.sh`:

| Step | Expected exit | Observed exit |
|---|---|---|
| `crackvision.skeleton` | 0 (`EXIT_OK`) | 0 |
| `crackvision.paths` | 0 (`EXIT_OK`) | 0 |
| `crackvision.path3d` | 0 (`EXIT_OK`) | 0 |

No `Traceback` was present in any of the three steps' stderr.

## Observed output

- `data/skeletons/{case}_skeleton_stats.json`: `skeleton_pixels == 0`. `skeleton.py`'s
  `clean_and_skeletonize` runs `skimage.skeletonize` over an all-zero binary mask, which yields an
  all-zero skeleton — `binary.sum() == 0` propagates directly to `skeleton_pixels`.
- `data/paths/{case}_paths.json`: `component_count == 0`, `components == []`. `paths.build_graph`
  over an all-zero skeleton raster has no pixels to turn into a graph, so no connected component is
  ever found — zero components is the correct, intended shape of this file for a crack-free case,
  not an error state.
- `data/paths3d/{case}_paths3d.json`: validates against `docs/INTERFACES.md` #10:
  - `schema == "crackvision.paths3d/1"`, `components == []` (an empty `paths.json` components list
    lifts to an empty `paths3d.json` components list — `path3d.build_case_paths3d` iterates
    `paths_doc["components"]`, which is empty, so `components_out` stays `[]`).
  - `counts.points == 0`, `counts.valid == 0` (no points were ever lifted, so both the numerator and
    denominator of "fraction lifted" are honestly zero, not fabricated).
  - `execution_eligible == False`, with `ineligible_reasons` including `"no_valid_points"` (per §10.6,
    triggered by `counters["valid"] == 0`) alongside the synthetic-case reasons
    (`wrist_camera_nominal`, `tool_nominal`, `optical_frames_nominal`, `synthetic_capture`) that this
    generator always produces regardless of mask content. The file never claims a plan is safe to run
    on a case where no crack was lifted.

## Why this is a safe outcome

A crack-free prediction propagating to `skeleton_pixels=0`, `component_count=0` and an explicitly
`execution_eligible=false`, `no_valid_points`-flagged `paths3d.json` is the system correctly
reporting "nothing to do" rather than either (a) crashing or misreporting a legitimate zero-crack
case as a pipeline failure, which would force an unnecessary operator intervention, or (b)
fabricating a crack, a 3D point, or a false eligibility claim where none exists, which would be the
dangerous failure mode for a system whose downstream consumer (`MOT-05`) is required to refuse real
execution whenever `execution_eligible: false` (§10.6). No exception propagates out of any of the
three CLIs; each one terminates via its own designed `EXIT_OK` path with an honest, schema-valid,
empty result.

This run surfaced no defect in `skeleton.py`, `paths.py` or `path3d.py` — all three already handle
the empty-mask case correctly by construction, so none of those three files were modified for this
card.

## Citable facts for INT-03.5

- Script: `scripts/perc/test_safety_empty_mask.sh` (exit 0 on success).
- Exit codes observed for `skeleton`/`paths`/`path3d` against an all-zero mask: `0`/`0`/`0`.
- `skeleton_pixels=0`, `paths.json component_count=0`, `paths3d.json components=[]`,
  `execution_eligible=false`, `ineligible_reasons` includes `no_valid_points`.
- No traceback in any step's stderr.
- No code changes were required in `skeleton.py`, `paths.py` or `path3d.py`: the empty-mask path was
  already correct.

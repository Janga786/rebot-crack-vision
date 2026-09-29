# Path extraction accuracy on synthetic ground truth (PERC-09)

## Purpose

`crackvision.paths` (PERC-03) turns a skeleton graph into ordered polylines, but nothing until now
checked those polylines against a *known* centerline. This card adds:

- `tools/synth_cracks.py` — synthetic crack rasters (straight, curved, branched, noisy) generated
  from a parametric centerline the caller knows exactly, plus the corresponding ground-truth
  `(row, col)` polyline(s).
- `tests/test_path_accuracy.py` — runs the real pipeline (`crackvision.skeleton.
  clean_and_skeletonize` -> `crackvision.paths.build_case_paths`) on those rasters and scores the
  extracted path against ground truth, enforced as regression thresholds in CI.

## Generator (`tools/synth_cracks.py`)

- `generate_wavy_crack(...)` — a sinusoidal centerline (`amplitude_px`, `frequency`, `phase`)
  rasterized at a given `thickness_px` by stamping disks of `radius = thickness_px / 2` along a
  dense (1-px-step) walk of the centerline. `amplitude_px=0` degenerates to a straight line.
  Optional `salt_noise_prob` scatters random single-pixel noise across the canvas to exercise
  small-component pruning (`clean_and_skeletonize(..., min_component_size=...)`).
- `generate_branched_crack(...)` — the same wavy main crack plus one straight branch splitting off
  at `branch_frac` of the way along it, at `branch_angle_rad` from the local tangent.
- Both return a `SynthCrack(mask, centerline, branch_centerlines)`: `mask` is the bool raster,
  `centerline`/`branch_centerlines` are the dense, ordered ground-truth polylines used to draw it —
  not something separately measured, so there is no annotation noise in the ground truth itself.
- Determinism: every generator is a pure function of its arguments (`seed` only affects
  `salt_noise_prob` scattering); `test_generator_is_deterministic_given_seed` checks this directly.

## Metrics

All four operate on the dense ground-truth polyline vs. the pipeline's dense extracted polyline
(`crackvision.paths` keeps both dense and RDP-simplified points per docs/INTERFACES.md §3.13; dense
is what accuracy is scored against since the simplified variant is allowed to drop points anywhere
error stays within its own RDP tolerance, which would conflate simplification with extraction
error):

- **Symmetric Hausdorff distance (px)** — `max` of the two directed nearest-neighbor max distances
  (ground truth -> extracted, extracted -> ground truth), via `scipy.spatial.cKDTree`. Catches any
  single point that is badly off, in either direction (a spurious branch or a truncated path).
- **Mean distance (px)** — mean of the two directed nearest-neighbor *mean* distances. A softer,
  average-case companion to Hausdorff that is not dominated by one outlier pixel.
- **Coverage** — fraction of ground-truth points with some extracted point within
  `COVERAGE_THRESHOLD_PX` (3 px). Catches a path that matches well on average but drops a whole
  stretch of the true crack (e.g. a pruned spur that was actually real, or a branch the
  decomposition failed to visit).
- **Ordering error rate** — match each extracted point to the index of its nearest ground-truth
  point (ground truth is itself ordered along the true centerline), then count adjacent extracted
  steps whose matched index moves backwards, in whichever overall direction (forward or reverse)
  has fewer such steps — so a path recovered end-to-start is not penalized, but a path that
  zig-zags back and forth along its own centerline is. Reported as a fraction of steps.

`test_metrics_reject_a_badly_wrong_path` checks the metrics themselves are sensitive: a centerline
shifted 20 px sideways fails every threshold below, confirming they are not vacuously satisfied.

## Thresholds and why

| Metric | Threshold | Rationale |
|---|---|---|
| Symmetric Hausdorff | ≤ 6.0 px | Cracks are rasterized at `thickness_px=3` (radius 1.5 px); skeletonization of a stamped-disk raster can wander up to roughly that radius from the true centerline through ordinary raster/rounding jitter, plus a few px of slack at path endpoints/junction bridging. 6 px is ~2x that radius — enough headroom for expected jitter while still catching a real regression (wrong branch picked, spur mis-pruned, junction bridge cutting a corner). |
| Mean distance | ≤ 2.5 px | Same jitter source, but averaged over the whole path rather than worst-case — should sit close to or under the stamped radius (1.5 px) if extraction is correct; 2.5 px leaves margin without hiding a systematic bias. |
| Coverage (@ 3 px) | ≥ 0.95 | Allows a small fraction of endpoint pixels (junction bridging, path start/end) to sit slightly further than 3 px while still requiring the extracted path to track essentially the whole ground-truth centerline, not just a majority of it. |
| Ordering error rate | ≤ 0.02 | The repeated-diameter decomposition (PERC-03) is deterministic and should produce a monotonic traversal of the centerline; a small allowance (2% of steps) absorbs isolated nearest-neighbor ties near self-crossings of the sinusoidal test curves without masking an actual out-of-order/zig-zag bug. |

The branched-crack test uses looser thresholds (Hausdorff ≤ 10 px, mean ≤ 4 px) because the
diameter-based decomposition may legitimately route the *main* path through part of what was drawn
as the branch (whichever end-to-end route is longest wins, by design — see `crackvision/paths.py`
module docstring, policy 1); the test matches each extracted polyline to whichever ground-truth
centerline it is closer to before scoring, rather than assuming a fixed main<->main, branch<->branch
pairing.

## Running

```
./env.sh pytest tests/test_path_accuracy.py -q
```

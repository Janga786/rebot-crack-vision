# TC-010 — Skeletonization utility

**Task ID:** TC-010 · **Complexity:** SMALL–MEDIUM · **Prerequisites:** TC-002 (TC-007 for real data)

## Objective
Build `src/crackvision/skeleton.py`: binary mask → optional small-component cleanup → one-pixel
skeleton raster + statistics.

## Why this matters
The one-pixel centerline is this phase's perception endpoint (`adr/008`). Its statistics are the
evidence base for designing the deferred path-ordering stage.

## Prerequisites
TC-002 COMPLETE (scikit-image installed). Can be developed in parallel with TC-004–TC-009 using
synthetic masks; needs TC-007's `case_map.json` for a real end-to-end run.

## Scope
`src/crackvision/skeleton.py` per `docs/INTERFACES.md` §3.4, plus `tests/test_skeleton.py`.

## Out of scope — read this twice

**Do NOT implement, even though it may feel natural:**
- branch-point or endpoint detection
- graph construction (`networkx`, `skan`, or hand-rolled)
- path ordering / traversal / "walk the crack"
- spur pruning, branch selection, longest-path extraction
- polyline fitting, spline smoothing, Douglas-Peucker
- crack width estimation, length in mm, severity classification
- depth lookup or any 3D anything

`adr/008` explains why: these choices cannot be made sensibly before we have seen real D405 masks.
The stats file you emit is what that later design session will use. Adding traversal here would
guarantee it gets redesigned.

Also out of scope: modifying predictions, visualization (TC-009), the orchestrator (TC-011).

## Files to create
```
src/crackvision/skeleton.py
tests/test_skeleton.py
```

## Files allowed to modify
`docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`. Writes `data/skeletons/`.

## Files that must NOT be modified
`data/predictions/**` (read-only) · `src/crackvision/*.py` other than the new file · `scripts/**`.

## Implementation requirements

```
./env.sh python -m crackvision.skeleton [--cases A B ...] [--min-component-size N]
                                        [--dilate N] [--no-overlay] [common flags]
```

### The algorithm — exactly this, nothing more

```python
from skimage.morphology import remove_small_objects, skeletonize

binary  = np.asarray(pred) > 0                                   # defensive, RISKS.md R-03
cleaned = remove_small_objects(binary, min_size=N) if N > 0 else binary
skel    = skeletonize(cleaned)                                   # default method
```

`min_size` default **64** from `config/project.yaml` (`skeleton.min_component_size`); `0` disables
cleanup.

> `skimage.morphology.skeletonize` handles 2D natively. Do **not** pass `method=` unless the
> installed scikit-image requires it; if the signature has changed, adapt and note it under `ISSUES:`.
> `remove_small_objects` requires a boolean or labelled array — pass the boolean.

### Outputs

1. `data/skeletons/{case}_skeleton.png` — single-channel uint8, `skel * 255`, **same H×W as the
   original**, **1 px wide, never dilated**. This is the machine-readable artefact; a later stage
   indexes aligned depth with these exact pixel coordinates (`adr/002`).
2. `data/skeletons/{case}_skeleton_overlay.png` — RGB, the skeleton drawn opaque over the original in
   `visualization.skeleton_color` (default lime `(0,255,0)`). `--dilate N` may thicken the drawing
   **in this overlay only**, for human viewing — it must not affect output 1.
   `--no-overlay` skips this file.
3. `data/skeletons/{case}_skeleton_stats.json` — the schema in `INTERFACES.md` §3.4:
   `case_id, schema_version, image_height, image_width, mask_pixels, mask_fraction,
   components_before, components_after, min_component_size, skeleton_pixels, skeleton_fraction`.
   Component counts via `scipy.ndimage.label` or `skimage.measure.label` (8-connectivity), before and
   after cleanup.

### Behaviour

- Missing `case_map.json` → exit 3 with the actionable message.
- Missing prediction for a case → count failed, continue, exit 1 at the end.
- **Empty mask → all-zero skeleton, stats with zeros, status ok.** Not an error.
- `--skip-existing`, `--dry-run` as per the common flags.
- Public API: `skeletonize_case(cfg, case_entry, ...) -> dict`, `main() -> int`.

### Tests (`tests/test_skeleton.py`) — synthetic masks only, no model needed

| Test | Assertion |
|---|---|
| 5 px wide horizontal bar, 200 px long | skeleton is 1 px tall along its length; `skeleton_pixels` ≈ 200 (±5) |
| filled 50×50 square | `skeleton_pixels` ≪ `mask_pixels` |
| two blobs, one 10 px and one 5000 px, `min_size=64` | `components_before == 2`, `components_after == 1` |
| same with `min_size=0` | `components_after == 2` |
| all-zero mask | all-zero skeleton, stats all zero, exit 0 |
| any mask | `skeleton_pixels <= mask_pixels` |
| any mask | `skeleton.shape == mask.shape` |
| prediction with `{0,255}` values | same result as `{0,1}` (defensive binarisation) |
| stats JSON | validates against the schema; `mask_fraction == mask_pixels/(H*W)` |
| `--dilate 2` | `{case}_skeleton.png` is **unchanged**; only the overlay differs |

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh pytest tests/test_skeleton.py -v
./env.sh python -m crackvision.skeleton --help
./env.sh python -m crackvision.skeleton ; echo "exit=$?"
ls data/skeletons/
python3 -m json.tool data/skeletons/*_skeleton_stats.json | head -20
./env.sh python -c "
from PIL import Image; import numpy as np, glob
for f in sorted(glob.glob('data/skeletons/*_skeleton.png')):
    a=np.array(Image.open(f)); print(f, a.shape, a.dtype, np.unique(a), 'px', int((a>0).sum()))"
# prove no traversal code crept in:
grep -niE "networkx|skan|traverse|ordered_path|prune|spur|longest_path|polyline|spline" src/crackvision/skeleton.py || echo "clean: no traversal code"
```

## Acceptance criteria
- [ ] `{case}_skeleton.png` is single-channel uint8 containing only `{0,255}`, same H×W as the original.
- [ ] The skeleton is genuinely 1 px wide (verified by the horizontal-bar test).
- [ ] `skeleton_pixels <= mask_pixels` for every case.
- [ ] `remove_small_objects` with `min_size=64` drops the small blob; `min_size=0` keeps it.
- [ ] `_skeleton_stats.json` matches the schema and `components_before`/`components_after` are correct.
- [ ] An all-zero mask produces valid zero outputs and exit 0.
- [ ] `{0,255}`-valued and `{0,1}`-valued predictions give identical results.
- [ ] `--dilate` does not alter `{case}_skeleton.png`.
- [ ] **The grep above finds no traversal/graph/pruning code.**
- [ ] Missing `case_map.json` → exit 3.
- [ ] `./env.sh pytest tests/ -v` is green.
- [ ] No hard-coded `/home/` path.

## Failure handling
- **`skeletonize` signature changed** in the installed scikit-image → adapt, note under `ISSUES:`,
  do not pin or downgrade the package.
- **`remove_small_objects` deprecation warning about `min_size`/connectivity** → follow the current
  API; the behaviour (drop components below `min_size` pixels) must stay the same.
- **The skeleton looks noisy/spurred** → **do not prune it.** That is exactly the deferred design
  question (`adr/008`). Record the observation under `ISSUES:` — it is useful evidence.
- **Memory on a very large mask** → process one case at a time; do not batch into one array.

## Documentation update
Append the report with the stats for each processed case — these numbers feed the deferred
path-ordering design. Update `TASK_INDEX.md`: TC-010 → `COMPLETE`.

## Commit guidance
`feat: add mask cleanup and skeletonization utility`

## Completion report format
```
TASK: TC-010
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-011
```

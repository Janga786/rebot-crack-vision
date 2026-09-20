# TC-009 — Visualization pipeline

**Task ID:** TC-009 · **Complexity:** MEDIUM · **Prerequisites:** TC-007

## Objective
Build `src/crackvision/visualize.py`: turn `{0,1}`-valued prediction PNGs into human-viewable masks,
alpha-blended overlays, and three-panel comparison strips.

## Why this matters
The predictions are literally black to the human eye. Without this card there is no way to look at a
result, and the whole point of this phase is *looking at* whether OpenCrack works on D405 imagery.

## Prerequisites
TC-007 COMPLETE (`case_map.json` and the naming contract exist). Predictions from TC-006 or TC-008
are useful but not required — synthetic masks are fine for development and tests.

## Scope
`src/crackvision/visualize.py` per `docs/INTERFACES.md` §3.3, plus `tests/test_visualize.py`.

## Out of scope
- Skeletonization (TC-010) — do not draw a skeleton here.
- Any modification of prediction files.
- Contour extraction, bounding boxes, connected-component labelling, crack-width measurement,
  severity scoring, colour maps beyond the single overlay colour.
- Interactive viewers, HTML reports, montages across cases.

## Files to create
```
src/crackvision/visualize.py
tests/test_visualize.py
```

## Files allowed to modify
`docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`. Writes `data/overlays/`, `data/comparisons/`.

## Files that must NOT be modified
`data/predictions/**` (read-only) · `src/crackvision/{config,logging_setup,naming,prepare_inputs,inference}.py` ·
`scripts/**`.

## Implementation requirements

```
./env.sh python -m crackvision.visualize [--cases A B ...] [--alpha F] [--color R,G,B] [common flags]
```

1. Read `data/case_map.json`; missing → exit **3**: `"run: ./env.sh python -m crackvision.prepare_inputs first"`.
   `--cases` restricts to named case ids; default is all.
2. For each case, load the original via `source_path` and the prediction via `naming.prediction_path`.
   Missing prediction → count failed, continue (exit 1 at the end).
3. **Binarise defensively: `binary = np.asarray(pred) > 0`.** Never assume 255, never assume 1.
   This is the `RISKS.md` R-03 guard.
4. **Shape check:** if `prediction.shape[:2] != original.shape[:2]`, that is a hard per-case error —
   log ERROR, count failed, continue. It means the frame invariant was violated upstream
   (`RISKS.md` R-14). Do **not** resize to paper over it.
5. Outputs:
   - `data/overlays/{case}_mask.png` — single-channel uint8, `binary * 255`.
   - `data/overlays/{case}_overlay.png` — RGB. Where `binary`:
     `out = (1-α)·orig + α·colour`, else `out = orig`. Defaults α=0.5, colour `(255,0,0)` from
     `config/project.yaml`, overridable by `--alpha` / `--color`.
     Do the blend in float then cast back to uint8 — do not blend in uint8.
   - `data/comparisons/{case}_comparison.png` — `ORIGINAL | MASK | OVERLAY` concatenated
     horizontally with an 8 px white gutter (`visualization.panel_gutter_px`), each panel captioned
     in its top-left. The MASK caption also carries the crack-pixel percentage, e.g.
     `MASK  0.412%`.
6. Captions: `PIL.ImageDraw` with `ImageFont.load_default()`. **Do not require a TTF file** — none is
   guaranteed present. Draw the text on a small dark rectangle so it is legible over any background.
7. An **empty mask is not an error.** Emit all three files and log
   `INFO  case X: 0 crack pixels (0.000%)`.
8. `--skip-existing` skips cases whose three outputs already exist.
9. Provide `visualize_case(cfg, case_entry, ...) -> dict` and `main() -> int`.

### Tests (`tests/test_visualize.py`)
Use the `tmp_project` fixture from `tests/conftest.py`. Build synthetic originals and predictions in
the test — no real model needed.

- prediction with values `{0,1}` → mask contains `{0,255}`
- prediction with values `{0,255}` (defensive path) → mask still `{0,255}`, same crack pixels
- **all-zero prediction** → all three files exist, mask is all zero, exit 0, not an error
- all-ones prediction → overlay equals the blend everywhere
- mask/overlay/comparison dimensions: mask and overlay match the original exactly; comparison width
  is `3*W + 2*gutter`
- **shape mismatch** between prediction and original → that case is counted failed, others still
  processed, final exit 1
- grayscale original → overlay is still 3-channel RGB
- `--dry-run` writes nothing
- missing `case_map.json` → exit 3

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh pytest tests/test_visualize.py -v
./env.sh python -m crackvision.visualize --help
./env.sh python -m crackvision.visualize ; echo "exit=$?"
ls -la data/overlays/ data/comparisons/
./env.sh python -c "
from PIL import Image; import numpy as np, glob, json
cm=json.load(open('data/case_map.json'))
for c in cm['cases']:
    cid=c['case_id']
    o=Image.open(c['source_path']); m=Image.open(f'data/overlays/{cid}_mask.png')
    ov=Image.open(f'data/overlays/{cid}_overlay.png'); cp=Image.open(f'data/comparisons/{cid}_comparison.png')
    print(cid,'orig',o.size,'mask',m.size,'overlay',ov.size,'cmp',cp.size)
    assert m.size==o.size==ov.size, 'frame invariant violated'
    print('  mask unique', np.unique(np.array(m)))
print('OK')"
```

## Acceptance criteria
- [ ] All three output files are produced per case with the exact names from `INTERFACES.md` §1.2.
- [ ] `{case}_mask.png` is single-channel uint8 containing only `{0, 255}`.
- [ ] Mask and overlay dimensions **exactly** match the original (frame invariant).
- [ ] Comparison width == `3*W + 2*gutter`, height == `H`.
- [ ] Binarisation is `pred > 0` — verify by testing both a `{0,1}` and a `{0,255}` prediction.
- [ ] An all-zero prediction produces valid outputs and exit 0.
- [ ] A shape mismatch is reported as a per-case error without resizing, and the other cases still
      complete.
- [ ] Captions render without any TTF font file present.
- [ ] `--alpha` and `--color` are honoured.
- [ ] `--dry-run` writes zero files.
- [ ] Missing `case_map.json` → exit 3 with the actionable message.
- [ ] `./env.sh pytest tests/ -v` is green.
- [ ] No hard-coded `/home/` path.

## Failure handling
- **Original file listed in `case_map.json` no longer exists** → count failed, continue, exit 1.
- **Original is RGBA or grayscale** → convert to RGB for compositing; the *output* is always RGB.
- **`ImageFont.load_default()` renders unreadably small** → acceptable; do not add a font dependency.
  If the label is genuinely illegible, scale the default font via `font_size=` if the installed
  Pillow supports it, else leave it and note under `ISSUES:`.
- **Comparison image very large** (a 4000 px original → 12000 px strip) → acceptable; do not
  downscale panels. Note it if it becomes a problem.

## Documentation update
Append the report and list the generated files with their dimensions. Update `TASK_INDEX.md`:
TC-009 → `COMPLETE`.

## Commit guidance
`feat: add mask, overlay and comparison visualization`

## Completion report format
```
TASK: TC-009
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-010
```

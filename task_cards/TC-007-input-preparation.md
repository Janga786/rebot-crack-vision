# TC-007 — Input preparation utility and the naming contract

**Task ID:** TC-007 · **Complexity:** MEDIUM · **Prerequisites:** TC-002

## Objective
Build `src/crackvision/naming.py` (the single source of truth for `case_id` and path mapping) and
`src/crackvision/prepare_inputs.py` (arbitrary imagery → nnU-Net-ready 3-channel RGB PNGs +
`case_map.json`), with unit tests.

## Why this matters
`NaturalImage2DIO` rejects JPEG outright and asserts a 3- or 4-channel array, while the plans declare
exactly 3 channels. Anything that is not forced to 3-channel 8-bit RGB PNG fails inference, often with
an opaque error deep inside nnU-Net (`RISKS.md` R-04). This card is the only place that guarantee lives.

## Prerequisites
TC-002 COMPLETE. Can run in parallel with TC-004/005/006 — it needs no model.

## Scope
1. `src/crackvision/naming.py` — pure functions, no I/O.
2. `src/crackvision/prepare_inputs.py` — the CLI, per `docs/INTERFACES.md` §3.1.
3. `tests/test_naming.py`, `tests/test_prepare_inputs.py`, `tests/conftest.py`.

## Out of scope
- Running inference (TC-008).
- Visualization (TC-009) or skeletonization (TC-010).
- Reading or modifying `data/input_originals/` contents — it is **read-only** to this tool.
- Recursing into subdirectories of `input_originals/` (flat, non-recursive, by design).
- Any image enhancement: no CLAHE, no denoise, no sharpen, no gamma. Conversion only.

## Files to create
```
src/crackvision/naming.py
src/crackvision/prepare_inputs.py
tests/conftest.py
tests/test_naming.py
tests/test_prepare_inputs.py
```

## Files allowed to modify
`docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`. Writes to `data/nnunet_input/` and
`data/case_map.json` at runtime.

## Files that must NOT be modified
`data/input_originals/**` · `src/crackvision/config.py`, `logging_setup.py` (TC-001 owns them) ·
`scripts/**` · `models/**`.

## Implementation requirements

### `naming.py` — implement exactly `docs/INTERFACES.md` §1

```python
def derive_case_id(source: Path) -> str: ...
def assign_case_ids(sources: Iterable[Path]) -> dict[Path, str]: ...   # handles collisions
# plus the eight path helpers in INTERFACES.md §1.2
```

Sanitisation (normative):
```python
s = re.sub(r"[^A-Za-z0-9-]", "_", Path(source).stem)
s = re.sub(r"_+", "_", s).strip("_")
if s.endswith("_0000"): s = s[:-5].rstrip("_")
if not s: s = "image"
```
Collisions: iterate `sorted(sources)`; first wins the bare id, later ones get `__2`, `__3`, …
This makes the mapping deterministic and re-runnable.

The worked-example table in `INTERFACES.md` §1.1 **is** the test fixture for `test_naming.py` —
implement every row.

### `prepare_inputs.py`

```
./env.sh python -m crackvision.prepare_inputs [--input-dir DIR] [--output-dir DIR]
                                              [--max-side N] [--clean] [common flags]
```

Per file, in order:
1. `img = PIL.Image.open(src)`
2. `img = PIL.ImageOps.exif_transpose(img)` — phone JPEGs carry rotation in EXIF
3. `img = img.convert("RGB")` — **mandatory, unconditional.** Handles L, LA, P, RGBA, CMYK, I;16.
   This is the line the whole card exists for.
4. optional `--max-side N` downscale with `Image.LANCZOS` (off by default; records `scale_factor`)
5. `img.save(dst, format="PNG", compress_level=6)` where `dst = data/nnunet_input/{case}_0000.png`

Accepted source extensions (case-insensitive): `.jpg .jpeg .png .bmp .tif .tiff`.
Output is **always** PNG regardless of input — `NaturalImage2DIO.supported_file_endings` excludes
`.jpg`/`.jpeg` deliberately ("we cannot allow lossy compression").

Use **Pillow, not OpenCV**, for the conversion: OpenCV silently gives BGR and mishandles palette/CMYK
and 16-bit inputs.

Write `data/case_map.json` in full (schema: `INTERFACES.md` §2), sorted by `case_id`, with
project-relative POSIX paths and sha256 of source and output.

Error policy: a corrupt file logs ERROR, increments `counts.failed`, processing continues; final
status `"partial"`, exit 1. Empty/missing input dir → exit **3** naming the directory and the
accepted extensions.

### Tests

`tests/conftest.py` — a `tmp_project` fixture building a throwaway project root with
`config/project.yaml` and the `data/` tree, so tests never touch the real `data/`.

`tests/test_naming.py` — every row of the `INTERFACES.md` §1.1 table, plus collision ordering.

`tests/test_prepare_inputs.py` — generate fixtures in-test with PIL and assert the output is
**always** `mode == "RGB"`, 3 channels, PNG:

| Fixture | Why |
|---|---|
| RGB JPEG | the common case; also proves JPEG→PNG |
| **RGBA PNG** | 4 channels → would break the 3-channel contract |
| **Grayscale (`L`) PNG** | 1 channel → would break it the other way |
| Palette (`P`) PNG | exercises the palette path |
| CMYK TIFF | exercises an exotic mode |
| **16-bit (`I;16`) PNG** | must come out 8-bit |
| **EXIF-rotated JPEG** (orientation 6) | output must be upright; assert width/height swap |
| BMP | coverage of another accepted extension |
| A zero-byte / truncated file | must be counted as failed, not crash the run |

Plus: `case_map.json` validates and round-trips; `--dry-run` writes nothing; `--clean` removes stale
outputs; re-running is idempotent (same hashes).

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh pytest tests/test_naming.py tests/test_prepare_inputs.py -v
./env.sh python -m crackvision.prepare_inputs --help
./env.sh python -m crackvision.prepare_inputs ; echo "empty-dir exit=$? (expect 3)"

# real end-to-end on the smoke fixtures from TC-006 (if present):
cp data/smoke_test/input/*.png data/input_originals/ 2>/dev/null || true
./env.sh python -m crackvision.prepare_inputs ; echo "exit=$?"
ls data/nnunet_input/
python3 -m json.tool data/case_map.json
./env.sh python -c "
from PIL import Image
import glob
for f in glob.glob('data/nnunet_input/*.png'):
    im=Image.open(f); print(f, im.mode, im.size, im.format)
    assert im.mode=='RGB' and im.format=='PNG'
print('all RGB PNG OK')"
```

## Acceptance criteria
- [ ] Every row of the `INTERFACES.md` §1.1 case-id table passes in `test_naming.py`.
- [ ] Collision handling produces `__2`, `__3` deterministically, in sorted-path order.
- [ ] **Every** fixture in the table above produces `mode == "RGB"`, PNG, 8-bit — including RGBA,
      grayscale and 16-bit.
- [ ] The EXIF-rotated fixture comes out upright (dimensions swapped as expected).
- [ ] Output filenames are exactly `{case}_0000.png`.
- [ ] `data/case_map.json` matches the §2 schema, is sorted by `case_id`, and uses project-relative
      POSIX paths.
- [ ] A truncated file is counted in `counts.failed`, the run continues, status is `"partial"`, exit 1.
- [ ] Empty input directory → exit **3** with a message naming the directory and extensions.
- [ ] `--dry-run` writes zero files.
- [ ] Re-running produces identical `sha256_nnunet_input` values (idempotent).
- [ ] `data/input_originals/` is unmodified (compare a checksum of the directory before/after).
- [ ] No hard-coded `/home/` path.
- [ ] `./env.sh pytest tests/ -v` is green.

## Failure handling
- **Pillow cannot open a file** → count failed, continue. Never crash the batch for one bad file.
- **A source has an unsupported extension** → skip silently at DEBUG level (it may be a stray `.txt`).
- **Two sources map to the same `case_id`** → that is the collision path; it must *work*, not error.
- **`ImageOps.exif_transpose` returns `None`** on some Pillow versions for images with no EXIF →
  guard with `img = ImageOps.exif_transpose(img) or img`.

## Documentation update
Append the report including the case-id table test output. Update `TASK_INDEX.md`: TC-007 →
`COMPLETE`; TC-008 → `READY` if TC-006 is also COMPLETE; TC-009 → `READY`.

## Commit guidance
`feat: add input preparation pipeline and naming contract`

## Completion report format
```
TASK: TC-007
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-008
```

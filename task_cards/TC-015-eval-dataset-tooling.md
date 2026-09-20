# TC-015 — Evaluation-dataset metadata tooling

**Task ID:** TC-015 · **Complexity:** SMALL–MEDIUM · **Prerequisites:** TC-001 (TC-013 for real fixtures)

## Objective
Build `tools/dataset_manifest.py`: create, populate, validate and summarise the D405 evaluation
manifest CSV described in `docs/D405_TEST_PLAN.md`.

## Why this matters
The Level-3 evaluation is only interpretable if every image carries its distance, angle, lighting,
surface and crack labels. Building the tooling *before* collection is what makes it realistic that
the metadata actually gets recorded (`docs/D405_TEST_PLAN.md` §6, step 6).

## Prerequisites
TC-001 COMPLETE. TC-013 gives `--synthetic` fixtures to test `scan` against; without it, write the
fixtures by hand in the test and note it.

## Scope
`tools/dataset_manifest.py` with four subcommands, plus `tests/test_dataset_manifest.py`.

## Out of scope — this is a tooling card, not a data card
- **Collecting, capturing or labelling any images.** Zero real data is produced by this card.
- Computing IoU/Dice or any accuracy metric (`docs/VALIDATION_PLAN.md` defers all of L3).
- Annotation tooling, a labelling GUI, mask drawing.
- Statistical analysis, plots, charts, or a report generator.
- Modifying the capture utility (TC-013) or any pipeline component.

## Files to create
```
tools/dataset_manifest.py
tests/test_dataset_manifest.py
```
Creates `data/d405/eval_manifest.csv` at runtime (header-only until data exists).

## Files allowed to modify
`docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`. Writes `data/d405/eval_manifest.csv`.

## Files that must NOT be modified
`docs/D405_TEST_PLAN.md` — it is the **schema's source of truth**; if the schema needs a change,
report it rather than editing. `src/crackvision/**`, `scripts/**`.

## Implementation requirements

### Subcommands

```
./env.sh python tools/dataset_manifest.py init     [--out data/d405/eval_manifest.csv] [--force]
./env.sh python tools/dataset_manifest.py scan     [--metadata-dir data/d405/metadata] [--merge]
./env.sh python tools/dataset_manifest.py validate [--in data/d405/eval_manifest.csv]
./env.sh python tools/dataset_manifest.py summary  [--in data/d405/eval_manifest.csv]
```

- **`init`** — write a header-only CSV with the exact column order of `D405_TEST_PLAN.md` §3.
  Refuse to overwrite an existing file without `--force` (exit 2). **This is what "no data collected"
  looks like: a header and nothing else.**
- **`scan`** — read every `data/d405/metadata/*.json` and emit one row per frame with the
  **tool-derived** columns filled (`image_id`, `timestamp_utc`, `camera`, `color_path`, `depth_path`,
  `metadata_path`, `exposure_us`, `depth_valid_fraction`) and the human columns blank.
  `depth_valid_fraction` = fraction of non-zero pixels in the depth PNG (the `RISKS.md` R-08
  early-warning metric). `--merge` preserves any human-entered values for `image_id`s already
  present — **never clobber human labels.**
- **`validate`** — enforce everything in `D405_TEST_PLAN.md` §3: required columns present; enum
  membership for `camera`, `surface_type`, `lighting_condition`, `crack_present`, `crack_type`,
  `negative_class`, `block`; `image_id` unique; referenced colour/depth/metadata files exist;
  `negative_class` set whenever `crack_present == "no"`; `0 < distance_cm <= 200`;
  `0 <= camera_angle_deg <= 90`; ratings, if present, are integers 0–3.
  Print one line per problem with the row number; exit 0 if clean, 1 if any problem.
  **An empty (header-only) manifest is valid** — exit 0.
- **`summary`** — counts per `block`, per `distance_cm`, per `surface_type`, per
  `lighting_condition`, `crack_present` yes/no/ambiguous, and per `negative_class`; plus the mean of
  each `rating_*` column where populated. Compare against the §2 target allocation and print the
  shortfall per block, e.g. `distance: 12/25 (need 13 more)`. Exit 0 always.

### Implementation notes
- Use the stdlib `csv` module with `DictReader`/`DictWriter`. **Do not add a pandas dependency.**
- Define the column order and the enum vocabularies as module-level constants with a comment pointing
  at `docs/D405_TEST_PLAN.md` §3 as the source of truth.
- Blank human cells are `""`, never `"None"` or `"NaN"`.
- Write CSVs atomically (temp + `os.replace`) so a crash never truncates a manifest holding hours of
  human labelling.
- `--dry-run` prints what would be written.

### Tests
- `init` creates a header-only CSV with exactly the §3 columns, in order.
- `init` on an existing file without `--force` → exit 2, file untouched.
- `scan` against `--synthetic` metadata (or hand-written fixtures) produces one row per frame with
  tool columns populated and human columns blank.
- `scan --merge` preserves a human-entered `distance_cm`/`crack_present` on re-scan.
- `validate` on a header-only manifest → exit 0.
- `validate` catches each of: missing required column, bad enum value, duplicate `image_id`,
  missing referenced file, `crack_present == no` with blank `negative_class`,
  `distance_cm` out of range, non-integer rating.
- `summary` runs on an empty manifest without dividing by zero.

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh python tools/dataset_manifest.py --help
./env.sh python tools/dataset_manifest.py init ; echo "exit=$?"
cat data/d405/eval_manifest.csv
./env.sh python tools/dataset_manifest.py validate ; echo "empty validate exit=$? (expect 0)"
./env.sh python tools/dataset_manifest.py init ; echo "no-force exit=$? (expect 2)"

# against synthetic captures from TC-013:
./env.sh python -m crackvision.realsense_capture --synthetic 3 --session synthtest
./env.sh python tools/dataset_manifest.py scan ; echo "exit=$?"
cat data/d405/eval_manifest.csv
./env.sh python tools/dataset_manifest.py validate ; echo "exit=$?"
./env.sh python tools/dataset_manifest.py summary
./env.sh pytest tests/test_dataset_manifest.py -v
wc -l data/d405/eval_manifest.csv   # confirm NO real evaluation data was created
```

## Acceptance criteria
- [ ] All four subcommands exist and respond to `--help` with exit 0.
- [ ] `init` writes a header-only CSV whose columns match `D405_TEST_PLAN.md` §3 **exactly, in order**.
- [ ] `init` without `--force` on an existing file exits 2 and leaves it untouched.
- [ ] `scan` fills the tool-derived columns from metadata JSONs and leaves human columns blank.
- [ ] `depth_valid_fraction` is computed from the actual depth PNG.
- [ ] `scan --merge` provably preserves human-entered values.
- [ ] `validate` detects every one of the seven error classes listed above (one test each).
- [ ] `validate` on a header-only manifest exits 0.
- [ ] `summary` runs on an empty manifest without error and shows the §2 shortfall per block.
- [ ] **No pandas import.** Stdlib `csv` only.
- [ ] CSV writes are atomic.
- [ ] **No real evaluation imagery was captured or labelled** — the only rows are from synthetic
      fixtures, and they are identifiable as such.
- [ ] `./env.sh pytest tests/ -v` green.
- [ ] No hard-coded `/home/` path.

## Failure handling
- **TC-013 not complete / no synthetic metadata** → hand-write minimal fixture JSONs in the test and
  note it under `ISSUES:`. Do not block on it.
- **A depth PNG referenced by metadata is missing** → `depth_valid_fraction` is blank, `validate`
  flags the missing file. Do not crash.
- **The §3 schema seems wrong or insufficient** → report it; do **not** edit `D405_TEST_PLAN.md`.
- **A CSV contains a comma or newline in `notes`** → the stdlib `csv` module handles quoting; add a
  test for it.

## Documentation update
Append the report with the generated CSV header line. Update `TASK_INDEX.md`: TC-015 → `COMPLETE`.

## Commit guidance
`feat: add D405 evaluation manifest tooling`
Commit `tools/dataset_manifest.py` and the test. **Do not commit `data/d405/eval_manifest.csv`** —
it is under the gitignored `data/` tree.

## Completion report format
```
TASK: TC-015
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-016
```

# TC-011 — One-command workflow (`run_test.sh`)

**Task ID:** TC-011 · **Complexity:** MEDIUM · **Prerequisites:** TC-003, TC-007, TC-008, TC-009, TC-010

## Objective
Build `run_test.sh`: the single command that takes images in `data/input_originals/` all the way to
comparison images, with a readable summary and no stack traces.

## Why this matters
This is the project's actual user interface. The whole experiment — "does OpenCrack work on D405
imagery?" — is run through this one script, repeatedly, by a human looking at pictures.

## Prerequisites
TC-003, TC-007, TC-008, TC-009, TC-010 all COMPLETE.

## Scope
1. `run_test.sh` (executable) orchestrating the five stages.
2. `scripts/summarize_run.py` — the summary table generator it calls.

## Out of scope
- Changing any stage's behaviour. This card **only orchestrates**; if a stage needs a fix, report it.
- D405 capture (TC-013) — `run_test.sh` never touches the camera.
- Parallelism, job queues, watch-mode/auto-rerun, a GUI, a web report.
- Any new analysis or metric beyond reading the stages' own JSON outputs.

## Files to create
```
run_test.sh                 (chmod +x)
scripts/summarize_run.py
```

## Files allowed to modify
`docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`, `README.md` (the quickstart section only).

## Files that must NOT be modified
`src/crackvision/**` · `scripts/check_env.py`, `fetch_model.py`, `verify_model.py`, `smoke_test.py` ·
`env.sh` (TC-002 owns it).

## Implementation requirements

### `run_test.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # no hard-coded /home path
```

Flags: `--device cuda|cpu` (passed to inference), `--skip-skeleton`, `--clean` (passed to
`prepare_inputs`), `--help`.

Stages, stopping at the first non-zero exit:

| # | Command | On exit 3 |
|---|---|---|
| 1 | `"$ROOT/env.sh" python "$ROOT/scripts/check_env.py"` | `Environment not ready — see the FAIL rows above.` |
| 2 | `"$ROOT/env.sh" python -m crackvision.prepare_inputs [--clean]` | `No input images. Put .jpg/.png files in data/input_originals/ and re-run.` |
| 3 | `"$ROOT/env.sh" python -m crackvision.inference --device "$DEVICE"` | (its own message) |
| 4 | `"$ROOT/env.sh" python -m crackvision.visualize` | |
| 5 | `"$ROOT/env.sh" python -m crackvision.skeleton` (unless `--skip-skeleton`) | |
| 6 | `"$ROOT/env.sh" python "$ROOT/scripts/summarize_run.py"` | |

**Exit-code 3 must be handled explicitly**, printing that stage's friendly message and exiting 3 —
**never** a bash stack trace or a raw `set -e` abort. Because `set -e` would abort before you can
react, capture the code:

```bash
run_stage() {
  local name="$1"; shift
  echo; echo "── $name ──"
  set +e; "$@"; local rc=$?; set -e
  if [ $rc -eq 3 ]; then echo; echo "STOPPED: $name — $(friendly_msg "$name")"; exit 3; fi
  if [ $rc -ne 0 ]; then echo; echo "FAILED: $name (exit $rc)"; exit $rc; fi
}
```

Print a clear banner per stage and the elapsed time for the whole run.

### `scripts/summarize_run.py`

Reads `data/case_map.json`, each `data/skeletons/{case}_skeleton_stats.json` (if present), and
`logs/inference_latest.json`. Prints an aligned table:

```
CASE                  SIZE        CRACK %   SKEL PX   COMPONENTS   COMPARISON
Deck_Crack_01         1280x720      0.412       1204            3   data/comparisons/Deck_Crack_01_comparison.png
deck_plain_02          848x480      0.000          0            0   data/comparisons/deck_plain_02_comparison.png

2 cases · inference 4.8 s (2.4 s/image, cuda)

Done. Open the results:
  /abs/path/data/comparisons/
  /abs/path/data/overlays/
```

Print **absolute** paths in that closing block (so they are clickable in a terminal), but keep
project-relative paths inside the table. Exit 0 even if some per-case stats are missing — just show
`-` in those cells. Also writes `logs/summarize_run_latest.json`.

### `README.md` quickstart section
Add (or update) exactly:

```markdown
## Quickstart
1. Put images in `data/input_originals/`
2. Run `./run_test.sh`
3. Open `data/comparisons/`
```

Leave the rest of `README.md` for TC-016.

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
chmod +x run_test.sh
./run_test.sh --help

# empty-input path must be friendly, not a traceback:
mkdir -p /tmp/io.bak && mv data/input_originals/* /tmp/io.bak/ 2>/dev/null || true
./run_test.sh ; echo "empty exit=$? (expect 3)"
mv /tmp/io.bak/* data/input_originals/ 2>/dev/null || true

# full run:
cp data/smoke_test/input/*.png data/input_originals/ 2>/dev/null || true
./run_test.sh ; echo "exit=$?"
./run_test.sh --device cpu ; echo "cpu exit=$?"
./run_test.sh --skip-skeleton ; echo "skip-skel exit=$?"
ls data/comparisons/
```

## Acceptance criteria
- [ ] `./run_test.sh` with images present runs all five stages and exits 0.
- [ ] The summary table prints one row per case with size, crack %, skeleton pixels, component count
      and the comparison path.
- [ ] The closing block prints **absolute** paths to `data/comparisons/` and `data/overlays/`.
- [ ] With `data/input_originals/` empty: exits **3**, prints
      `"Put .jpg/.png files in data/input_originals/"`, and shows **no bash stack trace and no Python
      traceback**.
- [ ] `--device cpu` works end to end.
- [ ] `--skip-skeleton` skips stage 5 and the summary shows `-` in the skeleton columns.
- [ ] `--help` exits 0 and documents every flag.
- [ ] `run_test.sh` contains **no** hard-coded `/home/` path and works when invoked from another
      directory (`cd /tmp && ~/Projects/rebot_crack_vision/run_test.sh --help`).
- [ ] `set -euo pipefail` is present and exit code 3 is still handled gracefully.
- [ ] The README quickstart is exactly the three steps above.
- [ ] No stage's behaviour was modified to make this work.

## Failure handling
- **A stage exits non-zero for a non-3 reason** → print the stage name and code, exit with that code.
  Do not continue to later stages with partial data.
- **`set -e` aborts before the friendly message** → use the `run_stage` pattern above.
- **`summarize_run.py` finds missing stats** → show `-`, exit 0. The summary must never be the thing
  that fails the run.
- **Invoked from a different working directory** → must still work; resolve everything from `$ROOT`.

## Documentation update
Append the report with the full output of one successful run. Update `TASK_INDEX.md`: TC-011 →
`COMPLETE`.

## Commit guidance
`feat: add one-command run_test.sh workflow and run summary`

## Completion report format
```
TASK: TC-011
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-012
```

# TC-016 — Documentation and reproducibility finalisation

**Task ID:** TC-016 · **Complexity:** SMALL–MEDIUM · **Prerequisites:** TC-011, TC-014, TC-015

## Objective
Write the real `README.md`, capture a frozen environment snapshot, add the licence/attribution block,
and leave the project reproducible from a clean checkout.

## Why this matters
This is a capstone deliverable. Someone else — an advisor, a teammate, or you in three months — must
be able to reconstruct the environment and rerun the experiment from the repo alone.

## Prerequisites
TC-011, TC-014, TC-015 COMPLETE. TC-013 COMPLETE (its deferred-hardware note goes in the README).

## Scope
1. `README.md` — the real one.
2. `docs/ENVIRONMENT_SNAPSHOT.md` — frozen versions from `logs/check_env_latest.json`.
3. `docs/RUNBOOK.md` — from-scratch reproduction steps.
4. Verify `requirements/requirements-core.txt` is pinned.
5. Add `LICENSE-NOTICES.md` with third-party attribution.
6. Final consistency pass over `TASK_INDEX.md` and `COMPLETION_LOG.md`.

## Out of scope
- Any code change. If a doc reveals a bug, report it — do not fix it here.
- Publishing, pushing to a remote, creating a GitHub repo, adding CI.
- Writing the capstone report itself.
- Collecting data or running the evaluation.
- Rewriting `docs/ARCHITECTURE.md`, `INTERFACES.md`, `RISKS.md`, `D405_TEST_PLAN.md`,
  `VALIDATION_PLAN.md` or any ADR — they are the planning record. You may **append** a dated
  "Implementation notes" section to `ARCHITECTURE.md` if reality diverged.

## Files to create
```
README.md                        (replacing the TC-001 placeholder)
docs/ENVIRONMENT_SNAPSHOT.md
docs/RUNBOOK.md
LICENSE-NOTICES.md
```

## Files allowed to modify
`requirements/requirements-core.txt` (pin only), `task_cards/TASK_INDEX.md`,
`docs/COMPLETION_LOG.md`, and an **appended** section to `docs/ARCHITECTURE.md`.

## Files that must NOT be modified
`src/crackvision/**`, `scripts/**`, `tools/**`, `tests/**`, `env.sh`, `run_test.sh`,
`config/project.yaml`, `docs/adr/**`, `docs/INTERFACES.md`, `docs/RISKS.md`,
`docs/D405_TEST_PLAN.md`, `docs/VALIDATION_PLAN.md`, `task_cards/TC-*.md`,
`task_cards/AGENT_INSTRUCTIONS.md`.

## Implementation requirements

### `README.md`
```markdown
# rebot_crack_vision
One-paragraph: what this is, what it tests, what it does not do yet.

## Quickstart
1. Put images in `data/input_originals/`
2. `./run_test.sh`
3. Open `data/comparisons/`

## What this does
ASCII pipeline diagram: RGB -> OpenCrack nnU-Net v2 -> binary mask -> overlay/comparison -> 1-px skeleton

## Current scope / Not yet implemented
Point at adr/008, adr/009, adr/010 for depth projection, path ordering, ROS and fine-tuning.

## Setup from scratch
Point at docs/RUNBOOK.md.

## Commands
A table of every CLI with a one-line description and its `./env.sh` invocation.

## Project layout
Condensed tree with one-line purposes.

## Results interpretation
- Predictions in `data/predictions/` are uint8 with values {0,1} and LOOK BLACK. This is correct.
  Use `data/overlays/` and `data/comparisons/` to look at results.
- The model over-fires on joints, mortar lines and crack-like texture (see its model card).

## Attribution
CC-BY-4.0 model; cite Fadeev 2026 and Isensee et al. 2021. Link LICENSE-NOTICES.md.

## Documentation index
Links to every file in docs/ and task_cards/.
```

**The `{0,1}` prediction note must be prominent** — it is the single most confusing property of the
system for a newcomer (`RISKS.md` R-03).

### `docs/ENVIRONMENT_SNAPSHOT.md`
Generated from `logs/check_env_latest.json` (re-run `check_env.py` first). A table of: OS, kernel,
CPU, GPU, driver, Python, torch (+CUDA variant), nnunetv2, numpy, scikit-image, Pillow, OpenCV,
pyrealsense2, huggingface_hub — plus the model repo id, revision and checkpoint sha256 from
`config/model_manifest.json`. Date-stamped, and stated to be a snapshot rather than a requirement.

### `docs/RUNBOOK.md`
Numbered, copy-pasteable, from a clean clone to first result:
1. clone / `cd`
2. `conda create -n crackvision python=3.11 -y`
3. torch from the cu126 index (with the recorded fallbacks)
4. `pip install nnunetv2` + the core requirements + `pip install -e .`
5. `chmod +x env.sh run_test.sh`
6. `./env.sh python scripts/check_env.py`
7. `./env.sh python scripts/fetch_model.py`
8. `./env.sh python scripts/verify_model.py`
9. `./env.sh python scripts/smoke_test.py`
10. put images in `data/input_originals/`; `./run_test.sh`
11. optional: `pip install -r requirements/requirements-realsense.txt`;
    `./env.sh python scripts/check_realsense.py`

Plus a **Troubleshooting** table covering, at minimum:
`ModuleNotFoundError` for a ROS package (you forgot `./env.sh`) · `torch.cuda.is_available()` False ·
`checkpoint_final.pth not found` (missing `-chk`) · fold errors (missing `-f 0`) ·
predictions look black (they are `{0,1}` — correct) · CUDA OOM (the §9 ladder) ·
`nnUNet_results is not defined` (not run via `env.sh`) · no RealSense device found.

### `LICENSE-NOTICES.md`
- **OpenCrack nnU-Net model** — CC-BY-4.0, `fadeevla/opencrack-nnunet`, attribution required, with
  the BibTeX from the model card.
- **nnU-Net** — Apache-2.0, Isensee et al., Nature Methods 18(2):203–211, 2021, DOI
  10.1038/s41592-020-01008-z.
- Brief notes for PyTorch (BSD-3), scikit-image (BSD-3), Pillow (MIT-CMU), OpenCV (Apache-2.0),
  pyrealsense2 (Apache-2.0).
- State the project's own licence, or "not yet chosen — capstone coursework" if the user has not said.

### Pin the requirements
```bash
./env.sh pip freeze | grep -viE '^(torch|torchvision|nvidia-|triton|pyrealsense2)' \
  > requirements/requirements-core.txt
```
Verify it contains only `==` pins and re-add the header comment explaining the exclusions.

### Final consistency pass
- Every card in `TASK_INDEX.md` is `COMPLETE`, `PARTIAL` or `SKIPPED`, with a reason for the latter two.
- Every card has a completion report in `docs/COMPLETION_LOG.md`.
- Every deviation recorded in a completion report is reflected in the README or an appended
  "Implementation notes" section in `ARCHITECTURE.md`.
- Every doc link in the README resolves.

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh python scripts/check_env.py                # refresh the snapshot source
./env.sh pytest tests/ -v
./run_test.sh                                       # confirm the documented path still works
grep -c "==" requirements/requirements-core.txt     # all pinned
# every README doc link resolves:
grep -oE '\]\(([^)]+\.md)\)' README.md | sed 's/](//;s/)//' | while read f; do
  test -f "$f" && echo "OK  $f" || echo "BROKEN  $f"; done
git status --short
git log --oneline | head -20
```

## Acceptance criteria
- [ ] `README.md` contains the 3-step quickstart, the pipeline diagram, the command table, the
      project layout, the attribution block and a documentation index.
- [ ] The `{0,1}`-valued-prediction note appears prominently in the README.
- [ ] `docs/ENVIRONMENT_SNAPSHOT.md` lists every version from `check_env_latest.json` plus the model
      revision and checkpoint sha256, date-stamped.
- [ ] `docs/RUNBOOK.md` reproduces the setup in numbered, copy-pasteable steps and includes the
      troubleshooting table with **all eight** entries above.
- [ ] `LICENSE-NOTICES.md` credits OpenCrack (CC-BY-4.0, with BibTeX) and nnU-Net (Apache-2.0).
- [ ] `requirements-core.txt` is fully `==`-pinned and excludes torch/torchvision/nvidia-*/pyrealsense2.
- [ ] Every markdown link in `README.md` resolves to an existing file.
- [ ] `TASK_INDEX.md` shows a terminal status for every card; `COMPLETION_LOG.md` has an entry for
      every card.
- [ ] `./env.sh pytest tests/ -v` still green; `./run_test.sh` still works.
- [ ] **No source file was modified** by this card.
- [ ] `git status` is clean after the final commit, with no large files staged.

## Failure handling
- **A documented step does not actually work when you follow it** → that is the point of writing the
  runbook. Fix the *documentation* to match reality, and if reality is wrong, report it under
  `ISSUES:` rather than changing code.
- **A card is `PARTIAL` or `BLOCKED`** → document its real state honestly in the README's
  "Not yet implemented" section. Do not paper over it.
- **`check_env.py` has FAIL rows** → record them in the snapshot as-is. An honest snapshot of a
  partially-working system beats a fictional clean one.
- **The user has not chosen a licence** → say "not yet chosen — capstone coursework". Do not invent one.

## Documentation update
Append the final completion report to `docs/COMPLETION_LOG.md` with an overall project status:
which cards are COMPLETE, which are PARTIAL, and what the honest next step is.

## Commit guidance
`docs: finalise README, runbook, environment snapshot and attribution`

## Completion report format
```
TASK: TC-016
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: none — implementation phase complete. Next is D405 data collection per docs/D405_TEST_PLAN.md.
```

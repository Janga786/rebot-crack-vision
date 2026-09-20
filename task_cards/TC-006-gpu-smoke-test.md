# TC-006 — Synthetic end-to-end smoke test (Level 2 core)

**Task ID:** TC-006 · **Complexity:** MEDIUM · **Prerequisites:** TC-003, TC-005

## Objective
Build `scripts/smoke_test.py`: generate a synthetic image, run the **real** OpenCrack model on it via
the real nnU-Net CLI, and assert the outputs are well-formed.

## Why this matters
This is the first time the model actually runs. It proves the whole chain — environment, model
folder, CLI flags, GPU — works together, and it pins the `{0,1}`-valued-prediction convention with a
test before any consumer is written.

## Prerequisites
TC-003 COMPLETE (`check_env.py` green except model rows), TC-005 COMPLETE (model validated).

> **Note on ordering:** this card runs inference *before* `crackvision.inference` (TC-008) exists.
> That is deliberate — `smoke_test.py` builds the `subprocess` command itself, from
> `config/project.yaml`, and TC-008 later factors that logic into a reusable module. TC-008's card
> tells it to refactor `smoke_test.py` to call the new module.

## Scope
Implement `scripts/smoke_test.py` per `docs/INTERFACES.md` §3.8.

## Out of scope
- **Any accuracy assertion.** See below — this is the most important constraint on this card.
- `crackvision.inference` / `visualize` / `skeleton` modules (TC-008/009/010). Do the minimum
  inline here: run the CLI, and render a simple mask + overlay for eyeballing.
- Writing into `data/predictions/`, `data/overlays/` or any non-`smoke_test` directory.
- Downloading anything.

## Files to create
`scripts/smoke_test.py`
Outputs go under `data/smoke_test/{input,nnunet_input,predictions,overlays,skeletons}/` (gitignored).

## Files allowed to modify
`data/smoke_test/**`, `docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`.

## Files that must NOT be modified
`models/**`, `data/input_originals/**` and every other `data/` subdirectory, `scripts/check_env.py`,
`scripts/verify_model.py`, `src/crackvision/**`.

## Implementation requirements

### 1. Synthetic fixture generation (deterministic, `numpy.random.default_rng(1234)`)

Two 512×512 RGB images into `data/smoke_test/input/`:

- `synthetic_crack.png` — mid-grey base (≈`128`) + Gaussian noise (σ≈12), then a dark
  (value ≈40), 2–4 px wide, slightly wandering polyline from roughly `(60,40)` to `(460,470)` with
  one branch, drawn with `PIL.ImageDraw.line(..., width=3)`; light Gaussian blur to soften edges.
- `synthetic_blank.png` — the same base and noise, **no** crack.

Pure numpy + PIL. No network, no external assets, no downloads.

### 2. Run the real chain

```
input/*.png
  → convert to <case>_0000.png in data/smoke_test/nnunet_input/   (RGB, PNG — same rules as TC-007)
  → nnUNetv2_predict_from_modelfolder
       -i data/smoke_test/nnunet_input
       -o data/smoke_test/predictions
       -m models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d
       -f 0
       -chk checkpoint_ep0500.pth
       -device <--device, default cuda>
       -npp 3 -nps 3
  → simple mask (×255) and red overlay into data/smoke_test/overlays/
  → skimage skeletonize into data/smoke_test/skeletons/
```

`-f 0` and `-chk checkpoint_ep0500.pth` are **mandatory** — the defaults are `(0,1,2,3,4)` and
`checkpoint_final.pth`, neither of which exists for this model (`RISKS.md` R-05).
Build the command as a **list**, `shell=False`. Stream stdout/stderr into the log.

### 3. Assertions (all must hold)

- [ ] the subprocess exits 0
- [ ] `data/smoke_test/predictions/synthetic_crack.png` and `synthetic_blank.png` exist, non-empty
- [ ] each prediction's dtype is `uint8`
- [ ] `set(np.unique(pred)) ⊆ {0, 1}` ← **pins the R-03 convention**
- [ ] `pred.shape == (512, 512)` — identical to the input ← **the frame invariant, R-14**
- [ ] mask, overlay and skeleton files exist and are 512×512
- [ ] `skeleton_pixels <= mask_pixels`

### 4. What this test must NOT assert

> The synthetic image is a drawn polyline on synthetic noise. It is **out of distribution** for a
> model trained on ~50 k real pavement images. It may well produce **zero** crack pixels.
>
> **That is not a failure.** Do not assert that the crack was detected. Do not assert any IoU, Dice,
> recall or crack-pixel count > 0. Do not tune the fixture until the model "finds" it.
>
> Log the crack-pixel percentage for each image at INFO for human interest, and move on.
> (`docs/VALIDATION_PLAN.md` §L2, `adr/001`.)

### 5. Output

Print per-stage wall-clock timings and a banner:
```
============================================================
  SMOKE TEST PASSED   (4 assertions x 2 images, 11.3 s)
  predictions: data/smoke_test/predictions/
  crack pixels: synthetic_crack 0.41%   synthetic_blank 0.02%
  NOTE: crack-pixel counts are informational only; this test
        verifies execution, not accuracy.
============================================================
```
Exit 0 on pass, 1 on assertion failure, 3 on precondition failure (model missing, CUDA requested but
unavailable). `--keep` retains fixtures (default: keep them anyway; `--clean` may remove).

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh python scripts/smoke_test.py ; echo "exit=$?"
ls -la data/smoke_test/predictions/
./env.sh python -c "
from PIL import Image; import numpy as np
a=np.array(Image.open('data/smoke_test/predictions/synthetic_crack.png'))
print('dtype',a.dtype,'shape',a.shape,'unique',np.unique(a))"
# CPU path must also work:
./env.sh python scripts/smoke_test.py --device cpu ; echo "cpu exit=$?"
nvidia-smi --query-gpu=memory.used --format=csv   # confirm VRAM released after the run
```

## Acceptance criteria
- [ ] `SMOKE TEST PASSED` on `--device cuda`, exit 0.
- [ ] `SMOKE TEST PASSED` on `--device cpu`, exit 0 (slower is fine).
- [ ] Prediction `dtype == uint8` and `np.unique(pred) ⊆ {0,1}` — **printed in the report**.
- [ ] `pred.shape == (512,512)` for both images.
- [ ] The command actually used `-f 0` and `-chk checkpoint_ep0500.pth` (paste the command line from
      the log into the report).
- [ ] Mask, overlay and skeleton images generated at 512×512.
- [ ] `skeleton_pixels <= mask_pixels` for both.
- [ ] **No assertion anywhere in the file references IoU, Dice, recall, or requires crack pixels > 0.**
- [ ] Nothing was written outside `data/smoke_test/` and `logs/`.
- [ ] Per-stage timings printed.

## Failure handling
- **CUDA OOM** → do not retry blindly. Print the ladder from `ARCHITECTURE.md` §9 and re-run with
  `--device cpu` to prove the chain works; record the OOM under `ISSUES:`. **Never kill a GPU
  process** to make room.
- **nnU-Net errors "checkpoint_final.pth not found"** → you forgot `-chk`. Fix your command.
- **nnU-Net errors about folds** → you forgot `-f 0`.
- **"Not all input images have the same shape"** → `NaturalImage2DIO` read a non-3-channel image;
  your conversion step is wrong (force `.convert("RGB")`).
- **Model missing** → exit 3 pointing at TC-004/TC-005. Do not download it here.
- **Predictions are all zero** → **this is an acceptable outcome.** Log it, assert nothing about it,
  pass the test.

## Documentation update
Append the report with the exact nnU-Net command line, the timings, and the observed unique values
and crack-pixel percentages. Update `TASK_INDEX.md`: TC-006 → `COMPLETE`.

## Commit guidance
`feat: add synthetic end-to-end smoke test (level 2)`

## Completion report format
```
TASK: TC-006
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-007
```

# TC-008 — Batch inference runner

**Task ID:** TC-008 · **Complexity:** MEDIUM · **Prerequisites:** TC-005, TC-006, TC-007

## Objective
Build `src/crackvision/inference.py`: a reusable module + CLI that checks preconditions, invokes
`nnUNetv2_predict_from_modelfolder` correctly, times it, verifies the outputs, and handles OOM
gracefully.

## Why this matters
This is the project's core operation. The two most likely bugs in the whole project live here:
forgetting `-f 0` and forgetting `-chk checkpoint_ep0500.pth` (`RISKS.md` R-05).

## Prerequisites
TC-005 (model validated), TC-006 (inference proven to work), TC-007 (`nnunet_input/` + `case_map.json`).

## Scope
1. `src/crackvision/inference.py` per `docs/INTERFACES.md` §3.2.
2. Refactor `scripts/smoke_test.py` to call `crackvision.inference.run_inference()` instead of
   building its own subprocess command. **This is the one place you may edit another card's file**,
   and only for that substitution — do not change its assertions.

## Out of scope
- Visualization (TC-009), skeletonization (TC-010), the orchestrator (TC-011).
- Any post-processing of the prediction: no morphology, no thresholding, no rescaling to 0–255.
  The prediction file is nnU-Net's output, byte-for-byte.
- Ensembling, multi-fold, `--save_probabilities` (may be exposed as a flag but is not used).
- Retraining anything.

## Files to create
`src/crackvision/inference.py`

## Files allowed to modify
`scripts/smoke_test.py` — **only** to swap its inline subprocess call for `run_inference()`.
`docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`.

## Files that must NOT be modified
`src/crackvision/naming.py`, `prepare_inputs.py`, `config.py`, `logging_setup.py` · `scripts/check_env.py`,
`fetch_model.py`, `verify_model.py` · `models/**` · `data/predictions/**` other than by running.

## Implementation requirements

### The canonical command — build it as a list, `shell=False`

```python
cmd = [
    "nnUNetv2_predict_from_modelfolder",
    "-i", str(cfg.nnunet_input),
    "-o", str(cfg.predictions),
    "-m", str(model_folder),          # models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d
    "-f", "0",                                  # MANDATORY — default is (0,1,2,3,4)
    "-chk", "checkpoint_ep0500.pth",            # MANDATORY — default is checkpoint_final.pth
    "-device", device,
    "-npp", str(npp), "-nps", str(nps),
    "-step_size", str(step_size),
]
if disable_tta:    cmd.append("--disable_tta")
if not_on_device:  cmd.append("--not_on_device")
if skip_existing:  cmd.append("--continue_prediction")
```

`-f 0` and `-chk` come from `config/project.yaml` (`model.fold`, `model.checkpoint`) — not
hard-coded strings — but they must **always** be present. Log the full command line at INFO before
running it.

`adr/006` explains why `_from_modelfolder` rather than `nnUNetv2_predict -d 501`. Also implement
`--use-dataset-id` as a documented fallback that builds the `-d 501 -c 2d -f 0` form (it works
because TC-005 created the `nnunet/results` symlink), but the default path is `-m`.

### Preconditions → exit 3, each with an actionable message

| Condition | Message |
|---|---|
| model folder missing | `run: ./env.sh python scripts/fetch_model.py` |
| `plans.json` / `dataset.json` / `fold_0/checkpoint_ep0500.pth` missing | `run: ./env.sh python scripts/verify_model.py` |
| `data/nnunet_input/` empty | `run: ./env.sh python -m crackvision.prepare_inputs` |
| `--device cuda` and `not torch.cuda.is_available()` | `use --device cpu, or check ./env.sh python scripts/check_env.py` |
| `nnUNetv2_predict_from_modelfolder` not on PATH | `run TC-002 / check ./env.sh` |

### Execution and reporting

- Stream the subprocess's stdout/stderr into the logger line by line — do **not** capture silently.
  nnU-Net's own progress output is valuable.
- Time it. Record `duration_s`, `images`, `seconds_per_image` in `logs/inference_latest.json`.
- Afterwards, verify exactly one `data/predictions/{case}.png` per `{case}_0000.png` input. Missing
  ones → status `"partial"`, exit 1, list them.
- Log once per run, at INFO:
  `"predictions are uint8 PNGs with values {0,1} (not {0,255}); see data/overlays/ for viewable renders"`
  (`RISKS.md` R-03).

### OOM handling

If the subprocess fails and its combined output matches
`(?i)(out of memory|outofmemoryerror|cuda error: out of memory)`, do **not** retry automatically.
Log this ladder as literal runnable commands and exit 1:

```
CUDA out of memory. Try, in order:
  1. ./env.sh python -m crackvision.inference --not-on-device
  2. ./env.sh python -m crackvision.inference --not-on-device --npp 1 --nps 1
  3. ./env.sh python -m crackvision.inference --not-on-device --disable-tta
  4. ./env.sh python -m crackvision.prepare_inputs --max-side 1024   (then re-run inference)
  5. ./env.sh python -m crackvision.inference --device cpu
This machine is shared — other projects may be holding VRAM. Do NOT kill their processes.
```

### Public API

```python
def run_inference(cfg, *, device="cuda", disable_tta=False, not_on_device=False,
                  step_size=0.5, npp=3, nps=3, skip_existing=False,
                  dry_run=False, logger=None) -> dict
```
Returns the summary dict. `--dry-run` logs the command and returns without executing.

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh python -m crackvision.inference --help
./env.sh python -m crackvision.inference --dry-run -v 2>&1 | grep -E "\-f 0|checkpoint_ep0500"

# precondition path:
mv data/nnunet_input /tmp/ni.bak && mkdir data/nnunet_input
./env.sh python -m crackvision.inference ; echo "empty exit=$? (expect 3)"
rm -rf data/nnunet_input && mv /tmp/ni.bak data/nnunet_input

# real run (after TC-007 populated nnunet_input):
./env.sh python -m crackvision.inference ; echo "exit=$?"
ls data/predictions/
./env.sh python -c "
from PIL import Image; import numpy as np, glob
for f in sorted(glob.glob('data/predictions/*.png')):
    a=np.array(Image.open(f)); print(f, a.dtype, a.shape, np.unique(a))"
python3 -m json.tool logs/inference_latest.json
./env.sh python -m crackvision.inference --device cpu ; echo "cpu exit=$?"
./env.sh python scripts/smoke_test.py ; echo "smoke still passes exit=$?"
```

## Acceptance criteria
- [ ] `--dry-run -v` output contains **both** `-f 0` and `-chk checkpoint_ep0500.pth`.
- [ ] A real run produces one `data/predictions/{case}.png` per `{case}_0000.png`.
- [ ] Predictions are `uint8` with `np.unique(pred) ⊆ {0,1}` and the same shape as the inputs.
- [ ] `logs/inference_latest.json` contains `duration_s`, `images`, `seconds_per_image`.
- [ ] The `{0,1}` advisory line appears in the log.
- [ ] Each of the five preconditions exits **3** with its actionable message (test at least the
      empty-input and missing-model cases for real).
- [ ] `--device cpu` works.
- [ ] `--use-dataset-id` also produces predictions (proves TC-005's symlink is correct).
- [ ] The OOM ladder text exists in the source and is reachable (inspect the regex + branch; you need
      not force a real OOM).
- [ ] `run_inference()` is importable and returns the summary dict.
- [ ] `scripts/smoke_test.py` now calls `run_inference()` and **still passes**, with its assertions
      unchanged.
- [ ] No post-processing of prediction files anywhere in the module.
- [ ] No hard-coded `/home/` path.

## Failure handling
- **OOM** → print the ladder, exit 1. Never kill another process; never auto-retry.
- **`checkpoint_final.pth not found`** → `-chk` is missing from your command.
- **Fold errors** → `-f 0` is missing.
- **`RuntimeError: nnUNet_results is not defined`** → you used `-d 501` without `env.sh`; the default
  `-m` path should not hit this at all.
- **Subprocess hangs > 30 min on a handful of small images** → likely `-npp`/`-nps` worker deadlock;
  retry with `--npp 1 --nps 1` and note it.
- **Some predictions missing** → status `"partial"`, exit 1, list them. Do not silently succeed.

## Documentation update
Append the report with the exact command line, timings and `seconds_per_image`. Update
`TASK_INDEX.md`: TC-008 → `COMPLETE`.

## Commit guidance
`feat: add batch inference runner with OOM handling`

## Completion report format
```
TASK: TC-008
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-009
```

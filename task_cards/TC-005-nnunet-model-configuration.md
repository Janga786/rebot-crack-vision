# TC-005 — nnU-Net model wiring and semantic validation

**Task ID:** TC-005 · **Complexity:** SMALL · **Prerequisites:** TC-004

## Objective
Build `scripts/verify_model.py`: validate that the downloaded tree is a *semantically correct* nnU-Net
v2 model for our pipeline, and create the relative `nnunet/results` symlink that makes the `-d 501`
fallback path work.

## Why this matters
TC-004 proved the bytes arrived. This card proves they mean what the architecture assumes — 3 RGB
channels, binary crack labels, `NaturalImage2DIO`, a 2d config with 256×256 patches. Every downstream
assumption in `INTERFACES.md` rests on these four facts.

## Prerequisites
TC-004 COMPLETE — `models/opencrack-nnunet/Dataset501_OpenCrack/...` populated, manifest written.

## Scope
Implement `scripts/verify_model.py` per `docs/INTERFACES.md` §3.7.

## Out of scope
- Running inference (TC-006).
- Downloading (TC-004).
- Any modification of the downloaded files. This script is **read-only** with respect to `models/`.
- Loading the full model into GPU memory.

## Files to create
`scripts/verify_model.py`
It creates the symlink `nnunet/results/Dataset501_OpenCrack` at runtime.

## Files allowed to modify
`nnunet/results/` (the symlink only), `docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`.

## Files that must NOT be modified
`models/**` — read-only. `config/model_manifest.json` (TC-004 owns it). `src/crackvision/**`.

## Implementation requirements

`./env.sh python scripts/verify_model.py [--check-hashes] [--repair-symlink]`

### 1. Structural check
```
<models>/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/plans.json
<models>/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/dataset.json
<models>/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/fold_0/checkpoint_ep0500.pth
```

### 2. Semantic assertions — these are the contract (verified upstream 2026-09-16)

`dataset.json` must have:
```python
channel_names == {"0": "R", "1": "G", "2": "B"}          # exactly 3 RGB channels
labels        == {"background": 0, "crack": 1}           # binary
file_ending   == ".png"
overwrite_image_reader_writer == "NaturalImage2DIO"
```

`plans.json` must have:
```python
"2d" in configurations
configurations["2d"]["patch_size"] == [256, 256]
len(configurations["2d"]["normalization_schemes"]) == 3   # matches the 3 channels
plans_name == "nnUNetPlans"                               # matches the folder name
dataset_name == "Dataset501_OpenCrack"
```

> **A mismatch here is a BLOCKER, not something to patch.** These four dataset.json facts are what
> `prepare_inputs` (3-channel RGB PNG), `inference` (folder name) and `visualize`/`skeleton`
> (`pred > 0` binarisation) are built on. If upstream changed, the architecture needs revisiting —
> report it (`RISKS.md` R-06).

### 3. Checkpoint sanity
```python
ckpt = torch.load(path, map_location="cpu", weights_only=True)
```
- **`weights_only=True` is mandatory.** Never `False` — that would execute arbitrary pickle code from
  a downloaded file.
- Assert the result is a dict; log its top-level keys and, if a `network_weights` / `state_dict`
  entry exists, its tensor count.
- If `weights_only=True` raises because the checkpoint stores non-tensor metadata (nnU-Net
  checkpoints carry `init_args`, `trainer_name`, `inference_allowed_mirroring_axes`), catch it,
  log a WARN, and fall back to verifying only the file's **magic bytes** (a torch `.pth` is a ZIP:
  starts with `PK\x03\x04`). **Do not** fall back to `weights_only=False`.

### 4. Symlink
Create, if absent:
```
nnunet/results/Dataset501_OpenCrack  ->  ../../models/opencrack-nnunet/Dataset501_OpenCrack
```
- **Relative**, so the project stays relocatable (`adr/004`).
- If a *file or real directory* already exists at that path, do **not** delete it — exit 3 and say so.
  Only `--repair-symlink` may replace an existing *broken symlink*.
- Verify afterwards that `nnunet/results/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/plans.json`
  resolves.

### 5. `--check-hashes`
Re-compute sha256 for every file in `config/model_manifest.json` and compare. Mismatch → exit 1
naming the file.

### 6. Output
A PASS/FAIL table like `check_env.py`, plus `logs/verify_model_latest.json`. Exit 0 / 3.

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh python scripts/verify_model.py ; echo "exit=$?"
./env.sh python scripts/verify_model.py --check-hashes ; echo "hashes exit=$?"
ls -la nnunet/results/
readlink nnunet/results/Dataset501_OpenCrack
test -f nnunet/results/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/plans.json && echo "symlink resolves"
# the -d 501 fallback path is now discoverable:
./env.sh python -c "
import os,glob
print(sorted(glob.glob(os.path.join(os.environ['nnUNet_results'],'Dataset501*'))))"
```

## Acceptance criteria
- [ ] All three required files are found and reported.
- [ ] All four `dataset.json` assertions PASS with the observed values printed.
- [ ] All five `plans.json` assertions PASS with the observed values printed.
- [ ] The checkpoint loads with `weights_only=True` (or the documented magic-byte fallback is used and
      logged as WARN). **`weights_only=False` appears nowhere in the file.**
- [ ] `nnunet/results/Dataset501_OpenCrack` is a **relative** symlink (`readlink` starts with `../`).
- [ ] `nnunet/results/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/plans.json` resolves.
- [ ] `--check-hashes` passes against TC-004's manifest.
- [ ] Re-running is idempotent (symlink not recreated, exit 0).
- [ ] Nothing under `models/` was modified (`find models -newer config/model_manifest.json` → empty).
- [ ] `logs/verify_model_latest.json` written.

## Failure handling
- **A semantic assertion fails** → BLOCKED. Paste the actual JSON content into the report. Do not
  edit `dataset.json` or `plans.json` to make it pass — that is falsifying the model.
- **`torch.load` raises even for the magic-byte fallback** → the file is corrupt: report and suggest
  `./env.sh python scripts/fetch_model.py --force`.
- **A real directory sits at the symlink path** → exit 3, report. Do not delete it.
- **Symlinks unsupported on the filesystem** (ext4 here, so unlikely) → note it; the `-m` path from
  `adr/006` does not need the symlink, so this is a WARN, not a blocker.

## Documentation update
Append the report with the **actual printed values** of `channel_names`, `labels`, `file_ending`,
`overwrite_image_reader_writer` and `patch_size` — later agents should be able to confirm the contract
without re-reading the model. Update `TASK_INDEX.md`: TC-005 → `COMPLETE`, TC-006 → `READY` (TC-003
must also be COMPLETE).

## Commit guidance
`feat: add nnU-Net model wiring and semantic validation`

## Completion report format
```
TASK: TC-005
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-006
```

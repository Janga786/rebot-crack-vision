# ADR-006: Invoke `nnUNetv2_predict_from_modelfolder`, not `nnUNetv2_predict`

**Status:** accepted · **Date:** 2026-09-16

## Context
nnU-Net v2 exposes two inference entry points (verified in `pyproject.toml` and
`nnunetv2/inference/predict_from_raw_data.py`, 2026-09-16):

- `nnUNetv2_predict -d 501 -c 2d -f 0` — resolves the dataset **by ID**, which requires the
  `nnUNet_results` environment variable to be set and the tree to be discoverable under it.
- `nnUNetv2_predict_from_modelfolder -m <folder> -f 0` — takes the trainer-config folder directly.
  Its own argparse description says it exists "when the nnunet environment variables
  (`nnUNet_results`) are not set."

The model card suggests the first form. The HF repo already ships a folder containing `plans.json`,
`dataset.json` and `fold_0/`, which is precisely what `initialize_from_trained_model_folder()` wants.

## Decision
**Primary invocation is `nnUNetv2_predict_from_modelfolder`**, pointed at
`models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d`, with the mandatory
overrides `-f 0` and `-chk checkpoint_ep0500.pth`. The `-d 501` form is kept working (via the
`nnunet/results` symlink from `adr/004`) as a documented fallback, but is not what the tooling uses.

## Consequences
+ Removes an entire class of failure ("nnU-Net can't find dataset 501"): the path is explicit and
  an existence check gives a precise error before anything is launched.
+ Makes the model location a plain argument, so a second checkpoint could be tested by changing one path.
+ Insulates us from any future change to nnU-Net's dataset-ID resolution.
− Slightly diverges from the model card's documented snippet. Noted in the README, and the `-d 501`
  path is kept alive and tested so we are not locked in.
− `-f` defaults to `(0,1,2,3,4)` and `-chk` to `checkpoint_final.pth`; **forgetting either override
  is the single most likely implementation bug.** Both are explicit acceptance criteria in TC-008.

## Revisit when
nnU-Net removes or renames the `_from_modelfolder` entry point. Then fall back to `-d 501 -c 2d -f 0`,
which the symlink already supports.

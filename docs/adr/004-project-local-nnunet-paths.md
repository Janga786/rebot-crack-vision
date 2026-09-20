# ADR-004: nnU-Net roots live inside the project, not in `$HOME`

**Status:** accepted · **Date:** 2026-09-16

## Context
nnU-Net v2 reads `nnUNet_raw`, `nnUNet_preprocessed` and `nnUNet_results` from the environment
(`nnunetv2/paths.py` wraps each in an `_EnvPath` that raises on first use if unset). The upstream
convention is to export them from `~/.bashrc` pointing at some global directory. This machine's
`.bashrc` is already crowded and is shared by several robotics projects.

## Decision
Keep all three inside the project and export them from `env.sh` only:

```
CRACKVISION_ROOT=~/Projects/rebot_crack_vision
nnUNet_raw=$CRACKVISION_ROOT/nnunet/raw
nnUNet_preprocessed=$CRACKVISION_ROOT/nnunet/preprocessed
nnUNet_results=$CRACKVISION_ROOT/nnunet/results
```

`nnunet/results/Dataset501_OpenCrack` is a **relative symlink** to
`../../models/opencrack-nnunet/Dataset501_OpenCrack`, so the weights are stored once and the project
directory stays relocatable. `nnUNet_compile=f` is exported too, matching how the model was trained.

## Consequences
+ `~/.bashrc` is never modified — a hard constraint of this project.
+ The project is self-contained, movable, and deletable in one `rm -rf`.
+ `raw/` and `preprocessed/` are unused this phase but exist so the env vars are always valid.
− Anyone who runs `nnUNetv2_*` without `env.sh` gets a confusing `RuntimeError` about undefined
  variables. Mitigated by `adr/006` (we normally use the `-m` entry point, which needs no env vars at
  all) and by `check_env.py` asserting the variables point inside the project.

## Revisit when
Never, for this project. A training phase would populate `raw/` and `preprocessed/` in place.

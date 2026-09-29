# ENVIRONMENT_SNAPSHOT.md — frozen environment record

**This is a snapshot of one run, not a requirement.** It records what `scripts/check_env.py`,
`config/model_manifest.json` and a few shell probes reported at the moment below. Re-run
`./env.sh python scripts/check_env.py` and compare against `logs/check_env_latest.json` before
trusting any of this on a different machine or after any upgrade.

**Captured:** 2026-09-29T02:53:31Z, from `logs/check_env_20260929T025331Z.json`
(`logs/check_env_latest.json` at capture time), host `booster`.

## Versions

| Component | Version |
|---|---|
| OS | Ubuntu 22.04.5 LTS (jammy) |
| Kernel | 6.8.0-124-generic (x86_64) |
| CPU | Intel(R) Xeon(R) Gold 6128 @ 3.40GHz (12 logical CPUs) |
| GPU | see "GPU note" below |
| Driver | see "GPU note" below |
| Python | 3.11.16 |
| torch | 2.14.0+cu126 |
| nnunetv2 | 2.8.1 |
| numpy | 2.4.6 |
| scikit-image | 0.26.0 |
| Pillow | 12.3.0 |
| OpenCV (`opencv-python-headless`) | 5.0.0 |
| pyrealsense2 | 2.58.4 |
| huggingface_hub | 1.31.0 |

## Model pin

From `config/model_manifest.json`:

| Field | Value |
|---|---|
| Repo id | `fadeevla/opencrack-nnunet` |
| Revision | `1198179e893f5f6eb0dd3eae2d8de5f1adf85afc` |
| Downloaded | 2026-09-19T01:26:14Z |
| Checkpoint | `Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/fold_0/checkpoint_ep0500.pth` |
| Checkpoint bytes | 268091674 |
| Checkpoint sha256 | `dcba82012874682c84387c68a2b69f64eddef07e84ae7d813844cdf6396a1a5f` |

## `check_env.py` result at capture time

`16 PASS · 4 WARN · 0 FAIL` — every PASS row above is reflected in the version table. The 4 WARN
rows were:

| Check | Value | Hint |
|---|---|---|
| `cuda_available` | `False` | no CUDA device visible; CPU inference is still supported |
| `gpu_name` | `N/A (no CUDA device)` | — |
| `gpu_memory` | `N/A (no CUDA device)` | — |
| `nvidia_driver` | `nvidia-smi` returned exit status 9 ("couldn't communicate with the NVIDIA driver") | `nvidia-smi` errored |

## GPU note — this snapshot vs. the host's own record

The above WARN rows are **honest for the process that captured this snapshot** (an agent session
without a visible NVIDIA device). They do **not** mean this project has never run on a GPU:

- `docs/MACHINE_STATE.md` (captured 2026-09-16, a direct shell probe of the same host) records an
  **NVIDIA GeForce RTX 3090, 24576 MiB, driver 580.173.02**, healthy and idle.
- `docs/COMPLETION_LOG.md` documents multiple real `nnUNetv2_predict_from_modelfolder` runs on that
  same CUDA device for TC-006, TC-008 and TC-009 (e.g. "the real OpenCrack checkpoint ran end-to-end
  ... on both CUDA and CPU"), with GPU memory checked before/after each run (`178 MiB` idle baseline,
  no leaks).

So: the GPU exists on this host and has been used for real inference in other sessions, but was not
visible to the process that generated the `check_env_20260929T025331Z.json` this file is built from.
`torch_import`/`torch_cuda_build` still confirm a CUDA-capable torch wheel is installed either way.
CPU inference remains fully supported and is what `run_test.sh` falls back to automatically via
nnU-Net's own device selection when no CUDA device is visible.

## Regenerating this file

```bash
./env.sh python scripts/check_env.py
cat logs/check_env_latest.json
cat config/model_manifest.json
```

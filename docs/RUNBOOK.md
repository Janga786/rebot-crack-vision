# RUNBOOK.md — from a clean clone to a first result

Numbered, copy-pasteable steps. Every command is run from the repository root unless stated
otherwise. See `docs/ENVIRONMENT_SNAPSHOT.md` for the exact versions this was last verified against,
and `docs/MACHINE_STATE.md` for the target machine's own inventory.

## Setup

1. **Clone and enter the repository.**
   ```bash
   git clone <this repository's URL>
   cd rebot_crack_vision
   ```

2. **Create the conda environment.**
   ```bash
   conda create -n crackvision python=3.11 -y
   ```

3. **Install PyTorch from the cu126 index**, with `set +u` first — the conda activation hook
   references unset variables (see `env.sh`'s own comment on this).
   ```bash
   set +u
   source "$HOME/miniconda3/etc/profile.d/conda.sh"
   conda activate crackvision
   set -u
   pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126
   ```
   If that index is unreachable or the machine has no CUDA-capable GPU, fall back to the CPU wheel
   (`pip install torch torchvision`) — every stage in this project still works, just slower
   (`docs/ARCHITECTURE.md` §9).

4. **Install nnU-Net and the pinned core requirements, then the project itself.**
   ```bash
   pip install nnunetv2
   pip install -r requirements/requirements-core.txt
   pip install -e .
   ```

5. **Make the launcher scripts executable.**
   ```bash
   chmod +x env.sh run_test.sh
   ```

6. **Verify the environment (Level 1).**
   ```bash
   ./env.sh python scripts/check_env.py
   ```
   Expect mostly PASS. `WARN` on the CUDA/GPU rows is fine on a CPU-only machine — see the
   troubleshooting table below for what each row means.

7. **Fetch the model weights** (not committed to git — 268 MB, and git-lfs is not installed here).
   ```bash
   ./env.sh python scripts/fetch_model.py
   ```
   This downloads `fadeevla/opencrack-nnunet` from Hugging Face and writes
   `config/model_manifest.json`, which pins every file by size, sha256 and upstream revision.

8. **Verify the model tree.**
   ```bash
   ./env.sh python scripts/verify_model.py --check-hashes
   ```
   Expect all checks PASS (0 FAIL). This confirms the on-disk model matches the pinned manifest
   byte-for-byte, not just "a file exists at that path".

9. **Run the synthetic end-to-end smoke test (Level 2).**
   ```bash
   ./env.sh python scripts/smoke_test.py
   ```
   This runs the real checkpoint against synthetic fixtures and asserts the predictions are
   well-formed `{0,1}`-valued masks. It does not need real crack imagery. `--device` defaults to
   `cuda` and, unlike `check_env.py`, does **not** fall back on its own: on a machine with no
   visible CUDA device this exits `3` (precondition not met) with `--device cuda requested but
   torch.cuda.is_available() is False`. On a CPU-only/no-GPU machine, run
   `./env.sh python scripts/smoke_test.py --device cpu` instead.

10. **Run the unit test suite.**
    ```bash
    ./env.sh pytest tests/ -v
    ```

## First real result

11. **Drop real images into `data/input_originals/`**, then run the one-command pipeline:
    ```bash
    ./run_test.sh
    ```
    This orchestrates `check_env → prepare_inputs → inference → visualize → skeleton →
    summarize_run` and prints a per-case summary table. Open `data/comparisons/` to look at the
    output (see the README's "Results interpretation" section before you do — the raw prediction
    files in `data/predictions/` look solid black and that is correct).

## Optional: RealSense D405 hardware

12. **Install the RealSense extra and verify the software stack** (this does not need a camera
    attached):
    ```bash
    pip install -r requirements/requirements-realsense.txt
    ./env.sh python scripts/check_realsense.py
    ```
    With no D405 attached, expect PASS on the software-stack checks (`pyrealsense2_import`,
    `pyrealsense2_version`, `librealsense_runtime`) and WARN on the device-presence checks
    (`devices_found`, `d405_present`, `stream_profiles`, `depth_scale`) — that is the expected,
    hardware-optional state this project has run in throughout Phase 1.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError` for a package that is clearly installed | You ran `python` directly instead of `./env.sh python`. ROS Humble's `PYTHONPATH` (sourced from `~/.bashrc`) leaks Python 3.10 paths into any interpreter, including this project's 3.11 env (`docs/adr/007`). | Always invoke through `./env.sh` (e.g. `./env.sh python scripts/check_env.py`), never bare `python`. |
| `torch.cuda.is_available()` returns `False` | No CUDA device is visible to the process — no GPU on the machine, wrong driver, or (in a sandboxed/CI environment) the device node simply isn't passed through. | Confirmed on-machine GPU status separately with `nvidia-smi`. If truly no GPU, use `--device cpu` (`./run_test.sh --device cpu` or `./env.sh python -m crackvision.inference --device cpu`) — CPU inference is fully supported, just slower. |
| `checkpoint_final.pth not found` (or similar) | The released OpenCrack checkpoint is named `checkpoint_ep0500.pth`, not nnU-Net's usual `checkpoint_final.pth`, and the command was run without `-chk`. | Always pass `-chk checkpoint_ep0500.pth` — `crackvision.inference` does this for you automatically; only matters if you invoke `nnUNetv2_predict` by hand. |
| Fold errors (e.g. "fold X not found") | The released model has only `fold_0` trained; a command defaulted to all-folds or the wrong fold. | Always pass `-f 0` explicitly — again, `crackvision.inference` does this for you; only matters for manual `nnUNetv2_predict` invocations. |
| Predictions in `data/predictions/` look solid black | This is **correct**, not a bug. `NaturalImage2DIO.write_seg` writes raw `{0,1}` label values, which are indistinguishable from black to the human eye at any normal display gamma (`docs/RISKS.md` R-03). | Don't rescale or "fix" the prediction files. Look at `data/overlays/` and `data/comparisons/` instead — those are the human-viewable renders. |
| `CUDA out of memory` during inference | The GPU is shared with other processes and doesn't have enough free VRAM for this batch. | Work down `docs/ARCHITECTURE.md` §9's ladder in order: (1) `./env.sh python -m crackvision.inference --not-on-device`, (2) add `--npp 1 --nps 1`, (3) add `--disable-tta`, (4) an explicit, logged downscale of oversized inputs (`prepare_inputs --max-side`), (5) `--device cpu`. `crackvision.inference` detects the OOM and prints this exact ladder itself; it never retries automatically and never kills another process's GPU usage. |
| `nnUNet_results is not defined` (or `nnUNet_raw`/`nnUNet_preprocessed`) | These environment variables are set by `env.sh`, not by a plain `conda activate crackvision`. | Run through `./env.sh` (see the first row above) — it exports project-local `nnUNet_raw`/`nnUNet_preprocessed`/`nnUNet_results` under this repo (`docs/adr/004`). |
| `./env.sh python scripts/check_realsense.py` reports "no RealSense device found" (`devices_found: WARN`) | No D405 is physically attached to a USB-3 port, or it's on a USB-2 port. | This is the expected state for a development machine with no camera — the software stack is still verified independently of device presence. Attach a D405 to a USB-3 port to clear the WARN, or use `crackvision.realsense_capture --synthetic` to exercise the code path without hardware (`TC-013`). |

## Regenerating the environment snapshot

`docs/ENVIRONMENT_SNAPSHOT.md` is a point-in-time capture, not something this runbook keeps in
sync automatically. After any dependency upgrade, regenerate it from a fresh
`./env.sh python scripts/check_env.py` run plus `config/model_manifest.json`.

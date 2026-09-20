# TC-002 — Python environment, dependencies and the `env.sh` launcher

**Task ID:** TC-002 · **Complexity:** MEDIUM · **Prerequisites:** TC-001

## Objective
Create the isolated `crackvision` conda environment (Python 3.11), install CUDA PyTorch + nnU-Net v2
+ the imaging stack, write the `env.sh` scrubbing launcher, and pin the result.

## Why this matters
This is the card that makes every subsequent card runnable. It also defuses the single most dangerous
property of this machine: ROS Humble exports a Python **3.10** `PYTHONPATH` that leaks into a Python
**3.11** interpreter (verified — `docs/MACHINE_STATE.md` §4, `adr/007`).

## Prerequisites
- TC-001 COMPLETE (tree, `config/project.yaml`, `pyproject.toml` exist)
- Network access (verified working: `huggingface.co` → 200)
- `~/miniconda3` present, `conda` 26.3.2

## Scope
1. `conda create -n crackvision python=3.11 -y`
2. Install CUDA PyTorch **first**, then `nnunetv2`, then the rest.
3. Write `requirements/environment.yml`, `requirements-core.txt`, `requirements-torch.txt`,
   `requirements-realsense.txt`.
4. Write `env.sh` (executable).
5. `pip install -e .` into the env.
6. Pin `requirements-core.txt` from `pip freeze` once everything is green.

## Out of scope
- Downloading the model (TC-004).
- Writing `scripts/check_env.py` (TC-003) — but you must verify manually here.
- Any pipeline module.
- Installing into, upgrading, or removing **any other conda env**.
- `~/.bashrc`, system CUDA, NVIDIA drivers, ROS, `sudo`.

## Files to create
```
env.sh                                   (chmod +x)
requirements/environment.yml
requirements/requirements-core.txt
requirements/requirements-torch.txt
requirements/requirements-realsense.txt
```

## Files allowed to modify
`docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`. The conda env `crackvision` **only**.

## Files that must NOT be modified
`~/.bashrc` · `/usr/local/cuda*` · `/opt/ros/**` · conda envs `base`, `navila`, `navila-vila`,
`lerobot`, `isaaclab`, `vlnce-isaac` · `~/.local/**` · anything outside the project.

## Implementation requirements

### Install order is load-bearing (`RISKS.md` R-16)

```bash
conda create -n crackvision python=3.11 -y

# 1. CUDA torch FIRST, from the pytorch index — never from plain PyPI
~/miniconda3/envs/crackvision/bin/pip install torch torchvision \
    --index-url https://download.pytorch.org/whl/cu126

# 2. verify BEFORE continuing
~/miniconda3/envs/crackvision/bin/python -c \
 "import torch;print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
# expect something like: 2.14.0+cu126 12.6 True

# 3. nnU-Net (must NOT replace the CUDA torch)
~/miniconda3/envs/crackvision/bin/pip install nnunetv2

# 4. re-verify torch was not swapped for a CPU wheel
~/miniconda3/envs/crackvision/bin/python -c \
 "import torch;assert '+cu' in torch.__version__, torch.__version__; print('torch still CUDA:', torch.__version__)"

# 5. remaining direct deps
~/miniconda3/envs/crackvision/bin/pip install pyyaml pillow scikit-image opencv-python-headless pytest huggingface_hub

# 6. the project itself
cd ~/Projects/rebot_crack_vision && ~/miniconda3/envs/crackvision/bin/pip install -e .
```

**Notes, researched — do not re-derive:**
- `cu126` is chosen because the PyTorch index carries the widest recent coverage there and RTX 3090
  (sm_86) is supported. Driver 580.173.02 (CUDA 13.0) satisfies it. Documented fallbacks in order:
  `cu130`, then `cu129`.
- `nnunetv2` (2.8.1) pins `torch>=2.1.2,!=2.9.*`. Any torch ≥2.10 is fine. If pip tries to install
  torch 2.9.x, pin explicitly: `pip install "torch>=2.10" --index-url .../cu126`.
- `opencv-python-headless` (not `opencv-python`) — this machine has no reliable GUI path for
  containers/headless runs, and the preview window is optional.
- **A system CUDA toolkit is NOT needed** (`adr/005`). Do not install `cudatoolkit` via conda; it
  would shadow the wheel's bundled runtime.

### `env.sh`

Must work **both** ways:
```bash
source env.sh          # activates in the current shell
./env.sh python -m crackvision.foo    # runs one command in a scrubbed env
```

Required behaviour, in order:
1. Resolve `CRACKVISION_ROOT` to the directory containing `env.sh` (via `${BASH_SOURCE[0]}`), so the
   project is relocatable — **no hard-coded `/home/...`**.
2. `unset PYTHONPATH` — the ROS 3.10 leak (`adr/007`).
3. Rebuild `LD_LIBRARY_PATH` dropping every entry containing `/opt/ros` or `/usr/local/cuda-11.8`.
4. `unset PYTHONHOME`; `export PYTHONNOUSERSITE=1`.
5. Source `~/miniconda3/etc/profile.d/conda.sh` and `conda activate crackvision`.
   Guard with `set +u` around the conda hook (it references unset vars under `set -u`).
6. Export:
   ```
   CRACKVISION_ROOT, PYTHONNOUSERSITE=1
   nnUNet_raw=$CRACKVISION_ROOT/nnunet/raw
   nnUNet_preprocessed=$CRACKVISION_ROOT/nnunet/preprocessed
   nnUNet_results=$CRACKVISION_ROOT/nnunet/results
   nnUNet_compile=f
   ```
7. If arguments were given, `exec "$@"`; otherwise (sourced) just return.
8. Fail loudly with a clear message if the `crackvision` env does not exist.

Do **not** unset `LD_LIBRARY_PATH` wholesale — other entries may matter; filter it.

### `requirements/requirements-torch.txt`
Not pip-installable (it needs `--index-url`); a documented command file:
```
# Install FIRST, before nnunetv2. See adr/005.
#   pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126
# Fallbacks, in order: cu130, cu129
# Driver 580.173.02 / RTX 3090 sm_86 supports all of them.
# Constraint from nnunetv2: torch>=2.1.2, !=2.9.*
```

### `requirements/requirements-realsense.txt`
```
# Optional hardware extra — install separately, never with the core stack (RISKS.md R-12)
#   ./env.sh pip install -r requirements/requirements-realsense.txt
pyrealsense2>=2.55,<3
```
Install it in this card **and** verify it imports; if it fails, that is a WARN, not a blocker —
record it under `ISSUES:` and continue (TC-012 handles it properly).

### `requirements/environment.yml`
```yaml
name: crackvision
channels: [conda-forge]
dependencies:
  - python=3.11
  - pip
  - pip:
    # torch must be installed separately first — see requirements-torch.txt
    - -r requirements-core.txt
```

### `requirements/requirements-core.txt`
Start with the unpinned direct dependencies; **once everything is green, regenerate it pinned**:
```bash
./env.sh pip freeze | grep -viE '^(torch|torchvision|nvidia-|triton|pyrealsense2)' \
  > requirements/requirements-core.txt
```
Add a header comment explaining that torch and pyrealsense2 are deliberately excluded.

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
chmod +x env.sh

# 1. the scrub actually works — THE critical check
./env.sh python -c "import sys; r=[p for p in sys.path if 'ros/humble' in p]; \
  print('python', sys.version.split()[0]); print('ROS leaks:', r); assert not r, 'SCRUB FAILED'"

# 2. torch + CUDA
./env.sh python -c "import torch; print(torch.__version__, torch.version.cuda, \
  torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else '-')"

# 3. nnU-Net
./env.sh python -c "import nnunetv2; print('nnunetv2', nnunetv2.__file__)"
./env.sh which nnUNetv2_predict_from_modelfolder
./env.sh nnUNetv2_predict_from_modelfolder --help | head -5

# 4. imaging stack
./env.sh python -c "import numpy,skimage,PIL,cv2,yaml; \
  print(numpy.__version__, skimage.__version__, PIL.__version__, cv2.__version__)"

# 5. project importable + env vars
./env.sh python -c "import crackvision, os; \
  print(crackvision.__version__); \
  print({k:os.environ[k] for k in ['CRACKVISION_ROOT','nnUNet_raw','nnUNet_results','nnUNet_compile']})"

# 6. other envs untouched
conda env list
~/miniconda3/envs/isaaclab/bin/python -c "import torch; print('isaaclab torch still', torch.__version__)"
```

## Acceptance criteria
- [ ] `conda env list` shows `crackvision`, and all six pre-existing envs are still listed.
- [ ] `./env.sh python -c "import sys; ..."` reports Python **3.11.x** and **zero** `ros/humble`
      entries on `sys.path`. *(If this fails, nothing else matters — fix it first.)*
- [ ] `torch.__version__` contains `+cu`, `torch.cuda.is_available()` is `True`, device name contains
      `3090`.
- [ ] `torch.__version__` is **not** `2.9.*`.
- [ ] `import nnunetv2` succeeds and `nnUNetv2_predict_from_modelfolder --help` exits 0.
- [ ] numpy, scikit-image, Pillow, cv2, yaml all import.
- [ ] `import crackvision` succeeds (editable install worked).
- [ ] `nnUNet_raw`, `nnUNet_preprocessed`, `nnUNet_results` are set and all start with
      `$CRACKVISION_ROOT`; `nnUNet_compile == "f"`.
- [ ] `env.sh` contains no literal `/home/` path.
- [ ] `env.sh` works both sourced and as `./env.sh <cmd>`.
- [ ] `requirements-core.txt` is pinned (`==`) and excludes torch/torchvision/nvidia-*/pyrealsense2.
- [ ] `isaaclab` env's torch is unchanged (still `2.7.0+cu128`).
- [ ] `~/.bashrc` is unmodified (`git`-free check: compare mtime / `grep -c crackvision ~/.bashrc` → 0).

## Failure handling
- **cu126 wheel not found** → try `cu130`, then `cu129`. Record which worked under `ISSUES:`.
- **pip replaces CUDA torch with a CPU wheel** (check 4 in the install order fails) → reinstall torch
  from the index URL and pin it: `pip install "torch==<version>+cu126" --index-url ...`.
  Then re-run `pip install nnunetv2` and re-verify.
- **`torch.cuda.is_available()` is False** → check `nvidia-smi` works, then check `LD_LIBRARY_PATH`
  scrubbing in `env.sh` (stale `cuda-11.8` shadowing is the known cause, `MACHINE_STATE.md` F-2).
  Do **not** touch the driver or system CUDA. If unresolved → BLOCKED.
- **`pyrealsense2` install fails** → WARN only, continue, note under `ISSUES:`.
- **Disk full** → report; do not delete anything outside the project.
- **conda solve takes >15 min** → it is fine to wait; do not switch to mamba (not installed) or
  add channels beyond `conda-forge`.

## Documentation update
Append the completion report to `docs/COMPLETION_LOG.md`, including the **exact** torch version,
CUDA variant, nnunetv2 version and Python version installed — later cards rely on these.
Update `TASK_INDEX.md`: TC-002 → `COMPLETE`; TC-003, TC-004, TC-007, TC-010, TC-012, TC-015 → `READY`.

## Commit guidance
`chore: crackvision python 3.11 environment, pinned deps and env.sh launcher`
Commit only: `env.sh`, `requirements/*`. (The conda env itself is outside the repo.)

## Completion report format
```
TASK: TC-002
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-003
```

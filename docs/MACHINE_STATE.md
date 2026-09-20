# MACHINE_STATE.md — Target machine inventory

**Captured:** 2026-09-16 (read-only inspection, Opus planning session)
**Host:** `booster`
**Inspection method:** shell probes only. Nothing was installed, downloaded, or modified.

> Re-run `scripts/check_env.py` (TC-003) before trusting this file; it is a point-in-time snapshot.

---

## 1. Operating system / CPU / memory

| Item | Value |
|---|---|
| Distro | Ubuntu 22.04.5 LTS (jammy) |
| Kernel | 6.8.0-124-generic |
| Arch | x86_64 |
| CPU | Intel Xeon Gold 6128 @ 3.40 GHz |
| Cores / threads | 6 cores × 2 SMT = 12 logical CPUs |
| RAM | 62 GiB total, ~55 GiB available |
| Swap | 2.0 GiB |

**Planning consequence:** only 12 logical CPUs, and nnU-Net spawns worker processes for preprocessing
(`-npp`) and segmentation export (`-nps`). Defaults of 3/3 are appropriate; do **not** raise them.

---

## 2. GPU / NVIDIA / CUDA

| Item | Value |
|---|---|
| GPU | NVIDIA GeForce RTX 3090 (confirmed), 24576 MiB |
| Compute capability | sm_86 (Ampere) |
| Driver | 580.173.02 |
| Driver-reported CUDA | 13.0 |
| MIG | not enabled |
| `nvidia-smi` | healthy, GPU 0 present, 0% util |

### VRAM — freed during this planning session

At inspection time a long-running NaVILA VLM server (PID 10831, 43 days elapsed, unrelated
`k1_research` project) held **15992 MiB**, leaving only ~8.2 GiB free. The user authorised stopping it
during this session. It was terminated and the GPU is now clear:

```
$ nvidia-smi --query-gpu=memory.used,memory.total --format=csv
memory.used [MiB], memory.total [MiB]
178 MiB, 24576 MiB          → ≈ 23.8 GiB FREE, no compute apps
```

**Planning consequence:** the full 24 GB is available *today*, but this machine is shared with active
robotics projects that routinely park multi-GB servers on GPU 0. Implementation therefore still
assumes VRAM is **not** guaranteed: `inference.py` must expose the OOM degradation ladder in
`ARCHITECTURE.md` §9 and must never assume an empty GPU. OpenCrack's model card states the model runs
on an 8 GB consumer GPU, so even a heavily-shared card is workable.

**Rule for every task card: never kill a GPU process.** If VRAM is short, degrade (see §9 ladder) and
report. Stopping PID 10831 was an explicit one-time user decision made in the planning session.

### System CUDA toolkits installed (do not modify)

```
/usr/local/cuda -> cuda-11.8      (nvcc 11.8.89 is what is on PATH)
/usr/local/cuda-11, /usr/local/cuda-11.8
/usr/local/cuda-12, /usr/local/cuda-12.6, /usr/local/cuda-12.8
/usr/local/cuda-13, /usr/local/cuda-13.2
```

The system default `nvcc` is **11.8**, which is far older than the driver. This is irrelevant to this
project: PyTorch pip wheels bundle their own CUDA runtime and need only a sufficiently new **driver**
(580.x satisfies cu126 / cu128 / cu129 / cu130). **No system CUDA change is required or permitted.**
See `adr/005-no-system-cuda-modification.md`.

---

## 3. Python / Conda

| Item | Value |
|---|---|
| Conda | miniconda3 26.3.2 at `~/miniconda3`, `conda init` present in `.bashrc` |
| Mamba / micromamba | not installed |
| System pythons | `/usr/bin/python3.10` (ROS default), `/usr/bin/python3.11` |
| Shell default `python3` | 3.13.13 (conda `base`) |

### Existing conda environments (all belong to other projects — DO NOT TOUCH)

| Env | Python | torch | CUDA | nnunetv2 | pyrealsense2 |
|---|---|---|---|---|---|
| `base` | 3.13.13 | 2.12.0+cu130 | 13.0 ✅ avail | ✗ | ✗ |
| `navila` | 3.10.20 | 2.3.0+cu121 | 12.1 ✅ avail | ✗ | ✗ |
| `navila-vila` | ? | (not probed) | | | |
| `lerobot` | 3.12.13 | 2.11.0+cu130 | 13.0 ✅ avail | ✗ | ✗ |
| `isaaclab` | 3.11.15 | 2.7.0+cu128 | 12.8 ✅ avail | ✗ | ✗ |
| `vlnce-isaac` | ? | (not probed) | | | |

**`nnunetv2` is not installed anywhere. `pyrealsense2` is not installed anywhere.**
This project gets its own new env (`crackvision`) — see `adr/003-isolated-python-environment.md`.

---

## 4. ⚠️ VERIFIED ENVIRONMENT FOOTGUNS

These were empirically confirmed on this machine and drive several architecture decisions.

### F-1 — ROS Humble leaks Python 3.10 paths into *every* conda env

`~/.bashrc:125` runs `source /opt/ros/humble/setup.bash`, which exports:

```
PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages
```

`PYTHONPATH` is honoured by *any* interpreter regardless of version. Confirmed experimentally:

```
$ ~/miniconda3/envs/isaaclab/bin/python -c "import sys; print([p for p in sys.path if 'ros/humble' in p])"
python 3.11.15
ROS paths on sys.path: ['/opt/ros/humble/lib/python3.10/site-packages',
                        '/opt/ros/humble/local/lib/python3.10/dist-packages']

$ env -u PYTHONPATH ~/miniconda3/envs/isaaclab/bin/python -c "..."
ROS paths: []
```

**Consequence:** a Python 3.11 `crackvision` env will silently import pure-Python packages from ROS's
3.10 tree and hit ABI errors on compiled ones. Every entry point in this project MUST run through the
scrubbing launcher `env.sh` (TC-002). See `adr/007-env-scrubbing-launcher.md`.

### F-2 — Stale CUDA 11.8 on the global `LD_LIBRARY_PATH`

```
LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:\
/opt/ros/humble/lib:/usr/local/cuda-11.8/lib64:
```

A cu126/cu130 torch wheel ships its own `libcudart`/`libcublas`; a CUDA 11.8 directory ahead of them
on `LD_LIBRARY_PATH` can shadow them and produce confusing loader errors. `env.sh` scrubs this too.

### F-3 — `~/.local` user-site shadows Python 3.10 environments

`~/.local/lib/python3.10/site-packages` holds **272 entries** (pre-existing, from other projects).
Any Python 3.10 interpreter on this box — including a 3.10 conda env — picks these up.
`~/.local/lib/python3.11` and `python3.12` are **empty**, which is one more reason to choose Python
**3.11** for this project. (See the pre-existing `env_user_site_footgun` note for history.)

---

## 5. ROS 2

| Item | Value |
|---|---|
| Distro | Humble (`/opt/ros/humble`), sourced in `.bashrc` |
| `ROS_DISTRO` | `humble` |
| Python | 3.10 |

**ROS is explicitly out of scope for this implementation phase** (`adr/009-robot-ros-integration-deferred.md`).
It matters here only as a source of environment pollution (F-1, F-2).

---

## 6. RealSense / D405

| Item | Value | Notes |
|---|---|---|
| `ros-humble-librealsense2` | 2.57.7 (apt) | ROS-scoped build |
| `ros-humble-realsense2-camera` | 4.57.7 | ROS node — not used this phase |
| `realsense-viewer` | `/opt/ros/humble/bin/realsense-viewer` | available |
| `rs-enumerate-devices` | `/opt/ros/humble/bin/rs-enumerate-devices` | available |
| **`pyrealsense2` (pip)** | **NOT INSTALLED in any env** | must be pip-installed into `crackvision` |
| D405 physically attached | **NO** — `lsusb` shows no Intel RealSense device | only an AX210 BT radio |

librealsense 2.57.7 ≫ 2.50, so D405 is supported by the system tooling.
PyPI `pyrealsense2` latest is **2.58.4.10922** with `cp310/cp311/cp312/cp313` manylinux x86_64 wheels —
a cp311 wheel exists, so Python 3.11 is safe.

**No camera is attached right now.** Every D405 task card must therefore be fully executable and
testable *without* hardware, degrading gracefully. See TC-012 / TC-013.

---

## 7. Disk

At inspection: **42 G free of 468 G (91 % used)** — tight. Cleanup was authorised and performed during
this planning session:

| Action | Freed |
|---|---|
| `rm -rf ~/.cache/pip` (pure HTTP/wheel cache) | 20 G |
| `conda clean --all` (package tarballs + index) | ~0.4 G |
| `rm -rf ~/nvidia/nvidia_sdk` (JetPack 6.2.2 + 7.2 Orin Nano trees, user-owned portion) | 31 G |
| `rm -rf ~/Downloads/nvidia/sdkm_downloads` (SDK Manager arm64 .deb payloads) | 31 G |
| Trash | 7 M |

| Filesystem | Size | Used | Avail | Use% | Mount |
|---|---|---|---|---|---|
| `/dev/nvme0n1p2` | 468 G | 321 G | **124 G** | **73 %** | `/` (and `/home`) |

### ⚠️ Outstanding: ~20 G of root-owned Jetson rootfs

SDK Manager extracted parts of the JetPack trees as **root**, so `rm` as `boosterk1` could not remove
them. There is **no passwordless sudo** on this box. Remaining:

```
~/nvidia/nvidia_sdk/JetPack_6.2.2_Linux_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/rootfs
~/nvidia/nvidia_sdk/JetPack_7.2_Linux_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/rootfs
```

To reclaim it the user must run, interactively:

```bash
sudo rm -rf ~/nvidia/nvidia_sdk
```

This is **not** a task card and no agent should attempt it — see `AGENT_INSTRUCTIONS.md` rule 14.

**Planning consequence:** 124 G free is comfortable. This project needs ≈10 G
(conda env with torch ≈6–9 G, model ≈275 MB, imagery small). Still download the model with
`local_dir` **inside the project** rather than into the 17 G `~/.cache/huggingface` hub cache, so the
weights are not stored twice — see TC-004.

## 8. Project / Git state

| Item | Value |
|---|---|
| `~/Projects` | exists; contains only `k1_research` |
| `~/Projects/rebot_crack_vision` | **did not exist** before this session; this planning session created only `docs/` and `task_cards/` |
| `~/Projects` is a git repo | no |
| `git` | 2.34.1 |
| `git-lfs` | **NOT installed** |
| `git config user.name` | `Gavinw575` |
| `git config user.email` | `gavinw575@gmail.com` |
| `init.defaultBranch` | unset (git will default to `master` and print a hint) |

Related existing workspace: `~/rebot_ws` — a git repo for the **same physical arm**
(reBot-DevArm B601-DM, clone of `reBotArmController_ROS2`), with URDF/meshes, MoveIt2 config,
Gazebo sandbox and a MuJoCo demo suite. It is **not** modified by this project. It is the eventual
consumer of the 3D crack trajectory, which is why `ARCHITECTURE.md` keeps the perception output in a
clean, robot-agnostic form.

**git-lfs is absent ⇒ never commit the 268 MB checkpoint.** `.gitignore` must exclude `models/`,
`nnunet/` and `data/` (TC-001, `RISKS.md` R-10).

---

## 9. Tooling availability

| Tool | Status |
|---|---|
| `huggingface-cli` / `hf` | ✅ `~/.local/bin/` (huggingface_hub **0.36.2**, installed under py3.10 user-site) |
| HF auth token | none present — fine, `fadeevla/opencrack-nnunet` is public and ungated |
| Network → huggingface.co | ✅ HTTP 200 |
| `ffmpeg` | ✅ |
| `rsync`, `unzip` | ✅ |
| `tree`, `jq` | ❌ MISSING — do not use them in scripts |
| `sudo` | assume unavailable (consistent with the sibling `rebot_ws` project) |

Note: the `hf` CLI on `PATH` runs under the polluted py3.10 user-site. Task cards install
`huggingface_hub` into the project env and call it via the project's own Python instead.

---

## 10. Summary of constraints that shaped the architecture

1. **Isolated Python 3.11 conda env** — nothing else on this box has nnU-Net or pyrealsense2, and
   3.11 dodges both the ROS 3.10 `PYTHONPATH` collision *and* the `~/.local` py3.10 user-site.
2. **A scrubbing launcher (`env.sh`) is mandatory**, not cosmetic — F-1 and F-2 are verified live.
3. **No system CUDA / driver / ROS changes** — the driver is already new enough for any modern wheel.
4. **Do not assume a free GPU.** 24 GB is free right now (a 16 GB squatter was stopped with user
   approval), but this box is shared — keep the OOM degradation ladder and never kill a GPU process.
5. **No D405 attached** — all camera code must be writable and testable headlessly.
6. **No git-lfs** — model weights and data stay out of git. 124 GB free after cleanup; ~20 GB of
   root-owned Jetson rootfs still reclaimable only by the user with `sudo`.

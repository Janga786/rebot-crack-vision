# ADR-005: No changes to the system CUDA toolkit, NVIDIA driver, or ROS install

**Status:** accepted · **Date:** 2026-09-16

## Context
The box has seven CUDA toolkits (`/usr/local/cuda-11.8` … `cuda-13.2`), with the default `nvcc`
symlink on the *oldest*, 11.8, while the driver is 580.173.02 (CUDA 13.0 capable). ROS Humble is
sourced globally. All of it is load-bearing for other active projects (NaVILA sim2real, Isaac Lab
5.0, the K1 deploy stack, the reBot arm ROS 2 workspace). There is also no passwordless sudo.

Crucially, **PyTorch pip wheels bundle their own CUDA runtime**. A GPU torch build needs only a
sufficiently new *kernel driver*; the system toolkit and `nvcc` are irrelevant unless compiling CUDA
extensions from source — which this project never does. Driver 580.x satisfies cu126, cu128, cu129
and cu130 alike, and RTX 3090 is sm_86, supported by all of them.

## Decision
Install GPU PyTorch from `https://download.pytorch.org/whl/cu126` into the project env.
**Do not** install, remove, reorder or re-symlink any system CUDA toolkit; do not touch the NVIDIA
driver; do not modify `/opt/ros/**` or `~/.bashrc`. No `sudo` in any task card.

## Consequences
+ Zero blast radius on other projects. No reboot, no driver risk, no apt.
+ Reproducible from a single pip command.
− The stale `/usr/local/cuda-11.8/lib64` on the global `LD_LIBRARY_PATH` can shadow the wheel's own
  CUDA libraries, so `env.sh` must scrub it (`adr/007`).
− If a future dependency genuinely needs `nvcc`, it must ship a wheel or the task is BLOCKED and
  escalated to the user — not solved with sudo.

## Revisit when
A required dependency has no binary wheel for cp311/linux-x86_64. Even then, prefer the
`nvidia-cuda-nvcc` pip package over touching `/usr/local`.

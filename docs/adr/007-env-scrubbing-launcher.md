# ADR-007: All entry points run through `env.sh`, which scrubs the inherited environment

**Status:** accepted · **Date:** 2026-09-16

## Context
`~/.bashrc:125` unconditionally sources `/opt/ros/humble/setup.bash`, which exports:

```
PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages
LD_LIBRARY_PATH=/opt/ros/humble/...:/usr/local/cuda-11.8/lib64:
```

`PYTHONPATH` is version-agnostic, so those Python **3.10** directories land on the `sys.path` of a
Python **3.11** interpreter. Verified live on this machine:

```
$ ~/miniconda3/envs/isaaclab/bin/python -c "import sys;print([p for p in sys.path if 'ros/humble' in p])"
python 3.11.15
['/opt/ros/humble/lib/python3.10/site-packages', '/opt/ros/humble/local/lib/python3.10/dist-packages']
$ env -u PYTHONPATH … → []
```

The failure mode is nasty: pure-Python ROS packages import fine and shadow real ones, while compiled
extensions raise opaque ABI errors. The stale CUDA 11.8 entry on `LD_LIBRARY_PATH` can likewise
shadow the torch wheel's bundled CUDA libraries. The sibling `~/rebot_ws` project independently hit
this and hard-codes "strip conda / de-ROS" into every launcher.

## Decision
Ship `env.sh` at the project root. It must, before running anything:
1. `unset PYTHONPATH`
2. filter `/opt/ros/*` and `/usr/local/cuda-11.8/lib64` out of `LD_LIBRARY_PATH`
3. `unset PYTHONHOME`; `export PYTHONNOUSERSITE=1`
4. activate the `crackvision` conda env
5. export `CRACKVISION_ROOT`, `nnUNet_raw`, `nnUNet_preprocessed`, `nnUNet_results`, `nnUNet_compile=f`

It works both as `source env.sh` and as `./env.sh <command> …`. **Every documented command, every
acceptance test, and `run_test.sh` go through it.** `check_env.py` asserts the scrub actually worked
by failing if any `ros/humble` entry is on `sys.path` — so a regression is caught immediately.

## Consequences
+ Deterministic, reproducible behaviour regardless of the invoking shell's state.
+ The scrub is *verified*, not merely hoped for.
− One extra layer of indirection; a developer who runs bare `python` will get confusing failures.
  Mitigated by making `check_env.py`'s first check the `PYTHONPATH`/`sys.path` cleanliness test with
  an explicit "you probably forgot ./env.sh" message.
− Because ROS is scrubbed, `rclpy` is unavailable inside this env — intended (`adr/003`, `adr/009`).

## Revisit when
`~/.bashrc` stops sourcing ROS globally, or ROS integration becomes in-scope.

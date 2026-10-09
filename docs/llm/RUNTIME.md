# LLM runtime — pinned llama.cpp build

Built by `scripts/llm/build_llama_cpp.sh` (LLM-02). No sudo; entirely in `~/opt`.

## Pin

| | |
|---|---|
| Repo | https://github.com/ggml-org/llama.cpp |
| Tag | `v0.5.0` |
| Commit | `7fe450e19305b828c199d602c23a8337aaa1f03b` |
| Install dir | `/home/boosterk1/opt/llama.cpp` |
| Binary | `/home/boosterk1/opt/llama.cpp/build/bin/llama-server` |
| Built (UTC) | 2026-10-09T05:09:00Z |
| CUDA toolkit | 12.8 (`/usr/local/cuda-12.8`) |
| CMake CUDA arch | 86 (sm_86, RTX 3090) |

## CMake configuration

```
cmake -S /home/boosterk1/opt/llama.cpp -B /home/boosterk1/opt/llama.cpp/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DGGML_CUDA=ON \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc \
    -DCMAKE_CUDA_ARCHITECTURES=86 \
    -DLLAMA_CURL=ON
cmake --build /home/boosterk1/opt/llama.cpp/build --config Release --target llama-server -j <nproc>
```

## Rebuilding

```
scripts/llm/build_llama_cpp.sh            # no-op if the pinned commit is already built
scripts/llm/build_llama_cpp.sh --force    # force a clean rebuild at the pinned commit
```

The script re-checks out `7fe450e19305b828c199d602c23a8337aaa1f03b` on every run and refuses to proceed (exit 1) if the
working tree lands on a different commit, so the build is reproducible: same tag/commit in, same
binary out. This file and `config/llm_runtime.json` are regenerated from the just-built binary's
own `--version`/`--list-devices` output on every run, so they always describe the build that was
actually verified, not a stale record.

## `llama-server --version`

```
0.00.000.829 I srv  llama_server: initializing ...
version: 0.5.0-dev (build 11146, commit 7fe450e19)
built with GNU 11.4.0 for Linux x86_64
```

## `llama-server --list-devices`

Captured on this host (RTX 3090, driver/CUDA per `config/host_versions.json`):

```
0.00.000.646 I srv  llama_server: initializing ...
Available devices:
  CUDA0: NVIDIA GeForce RTX 3090 (24115 MiB, 23716 MiB free)
```

## Isolation (REQ-HOST-2)

This build is consumed only by the local-inference runtime (`~/claude-auto/claude_auto/localcoder.py`,
per `config/llm_candidates.json`), started on demand and never co-resident with nnU-Net or MoveIt
GPU use. It does not touch the `crackvision` conda env (py3.11) or the system ROS py3.10
interpreter — it is a standalone C++/CUDA binary invoked directly, not through `./env.sh`.

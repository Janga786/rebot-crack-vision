# LLM-02 — Build pinned llama.cpp (CUDA, sm_86) in user space

**Accepted:** 2026-10-09 · **Work package:** Local coding model (L-LLM) · **Track:** automation · **Verified commit:** [`6e386af`](https://github.com/Janga786/rebot-crack-vision/commit/6e386af1e7fabd12e7d4ee01d21ad4bd88b36655)

## What was done

The pinned llama.cpp build (CUDA, RTX 3090 target) is now fully set up at ~/opt/llama.cpp with no sudo involved, and the project's runtime docs/config were regenerated from that real build rather than a workaround path. Verified: llama-server reports version 0.5.0 at the exact pinned commit, and --list-devices confirms it sees the RTX 3090 GPU with CUDA.

Files changed:
- [config/llm_runtime.json](https://github.com/Janga786/rebot-crack-vision/blob/6e386af1e7fabd12e7d4ee01d21ad4bd88b36655/config/llm_runtime.json) (+25 / −5)
- [docs/llm/RUNTIME.md](https://github.com/Janga786/rebot-crack-vision/blob/6e386af1e7fabd12e7d4ee01d21ad4bd88b36655/docs/llm/RUNTIME.md) (+85 / −19)
- [scripts/llm/build_llama_cpp.sh](https://github.com/Janga786/rebot-crack-vision/blob/6e386af1e7fabd12e7d4ee01d21ad4bd88b36655/scripts/llm/build_llama_cpp.sh) (+205 / −0)

Commits: [`690ed21`](https://github.com/Janga786/rebot-crack-vision/commit/690ed211d93200cf450cfdf7de6609cb3145c4c7), [`6e386af`](https://github.com/Janga786/rebot-crack-vision/commit/6e386af1e7fabd12e7d4ee01d21ad4bd88b36655)

## Why it was done

A reproducible, pinned llama-server build exists at ~/opt/llama.cpp without sudo.

- Requirement **REQ-LLM-2**: Local model served by a pinned runtime; only benchmark-qualified task classes are routed to it
- Requirement **REQ-HOST-2**: Isolation between ROS (system py3.10), perception (conda crackvision py3.11) and local inference, verified automatically

## How it moves the project forward

- Local coding model: **2/6** tasks accepted; whole project: **52/92**.
- REQ-LLM-2: 1/4 contributing tasks done
- REQ-HOST-2: 2/2 contributing tasks done — **requirement satisfied**
- Verification: automated checks run by the pipeline itself (version ✔, json ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The rework pass correctly regenerates config/llm_runtime.json and docs/llm/RUNTIME.md from the real ~/opt/llama.cpp build now that the path is writable, removing stale scratch-path provenance notes. All acceptance checks pass and all criteria are independently verified.”

## What it unlocks next

- **Can now be planned in detail:** HOST-06 — Host reproducibility audit (fresh clone)
- Closer: LLM-04 — Project benchmark harness for local coding models (still needs LLM-03)

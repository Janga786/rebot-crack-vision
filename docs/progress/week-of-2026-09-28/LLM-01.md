# LLM-01 — Local coding model shortlist from primary sources (no downloads)

**Accepted:** 2026-09-28 · **Work package:** Local coding model (L-LLM) · **Track:** automation · **Verified commit:** [`28f9f06`](https://github.com/Janga786/rebot-crack-vision/commit/28f9f06277c0df833fdabd5be9a622ad9b70496a)

## What was done

For the local coding-model shortlist, I sourced exact GGUF files, sizes, cryptographic hashes and licenses directly from Hugging Face for three candidates that fit the 24 GB GPU with 32K+ token context: Qwen3.6-27B, Laguna-XS-2.1, and Qwen3-Coder-30B-A3B-Instruct, totaling about 48 GB of planned downloads out of a 60 GB budget. All three confirmed to run on llama.cpp with tool-calling support, and no model weights were actually downloaded for this research step. The next step (a separate task) will download and benchmark these three on the project's own workload to pick a winner.

Files changed:
- [config/llm_candidates.json](https://github.com/Janga786/rebot-crack-vision/blob/28f9f06277c0df833fdabd5be9a622ad9b70496a/config/llm_candidates.json) (+83 / −0)
- [docs/llm/SELECTION.md](https://github.com/Janga786/rebot-crack-vision/blob/28f9f06277c0df833fdabd5be9a622ad9b70496a/docs/llm/SELECTION.md) (+125 / −0)

Commits: [`28f9f06`](https://github.com/Janga786/rebot-crack-vision/commit/28f9f06277c0df833fdabd5be9a622ad9b70496a)

## Why it was done

A bounded, source-cited shortlist (≤3) with exact files, sizes, hashes, licenses and a VRAM-coexistence plan.

- Requirement **REQ-LLM-1**: Local coding model chosen from current primary sources + a project benchmark (quantization, context, tool reliability, latency, license, VRAM coexistence) with bounded downloads

## How it moves the project forward

- Local coding model: **1/6** tasks accepted; whole project: **16/69**.
- REQ-LLM-1: 1/4 contributing tasks done
- Verification: automated checks run by the pipeline itself (schema ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Independently re-verified all three candidates against live Hugging Face API data (file sizes, sha256 hashes) and the Laguna-XS-2.1 license text — everything in config/llm_candidates.json and docs/llm/SELECTION.md matches primary sources. Schema check passes, budget is under 60GB, deletion policy is stated, and scope is respected with no downloads and a clean working tree. One minor self-acknowledged approximation exists in the Laguna-XS-2.1 KV-cache math (real architecture mixes SWA/global attention and uses FP8 KV, not dense fp16 GQA as estimated), but it only makes the e
…[45 chars clipped]”

## What it unlocks next

- **Ready to start:** LLM-03 — Download + verify shortlisted models (bounded, sequential)

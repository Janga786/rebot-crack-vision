# LLM-01 — Local coding model shortlist (no downloads)

Checked 2026-09-28 against primary sources (Hugging Face API/model cards, license texts, the
`ggml-org` llama.cpp Hugging Face org, and the `docs/function-calling.md` page of the llama.cpp
repo). No model weights were downloaded for this card; every size and `sha256` below comes from
the Hugging Face `tree/main` API's `lfs.oid` field, not from a local hash computation.

This card only produces the shortlist. Selection among these candidates happens in **LLM-05**
via the project's own benchmark (quantization/context/tool-reliability/latency measured on this
machine) — vendor benchmark numbers quoted below are not comparable across scaffolds and are not
used to rank the candidates here.

## Serving route

`llama.cpp llama-server`, OpenAI-compatible API, started **on demand only**, and never co-resident
with nnU-Net or MoveIt GPU use (they compete for the same 24 GB card). Consumed by
`~/claude-auto/claude_auto/localcoder.py`, not by Claude Code itself.

## Candidates (2 of the allowed ≤3)

### 1. Qwen3.6-27B — Q4_K_M GGUF

- Model: [`Qwen/Qwen3.6-27B`](https://huggingface.co/Qwen/Qwen3.6-27B) (Apache-2.0, per repo tags
  and `license_link` in the GGUF README) — dense 27B causal LM with a hybrid layer stack: of 64
  layers, 48 are Gated DeltaNet (linear attention) and 16 are full Gated Attention (GQA, 4 KV
  heads x 256 head_dim), per the model's own
  [`config.json`](https://huggingface.co/Qwen/Qwen3.6-27B/raw/main/config.json) and README layer
  description ("16 x (3 x (Gated DeltaNet -> FFN) -> 1 x (Gated Attention -> FFN))").
- GGUF: [`unsloth/Qwen3.6-27B-GGUF`](https://huggingface.co/unsloth/Qwen3.6-27B-GGUF),
  file `Qwen3.6-27B-Q4_K_M.gguf`, **16,817,244,384 bytes** (16.82 GiB),
  sha256 `5ed60d0af4650a854b1755bd392f9aef4872643dc25a254bc68043fa638392a0` (HF API
  `lfs.oid` for that file).
- llama.cpp support: confirmed by the llama.cpp project's own
  [`ggml-org/Qwen3.6-27B-GGUF`](https://huggingface.co/ggml-org/Qwen3.6-27B-GGUF) repo
  ("automatically converted using https://github.com/ggml-org/convert", run via
  `llama serve -hf ggml-org/Qwen3.6-27B-GGUF`).
- Tool calling: `llama-server --jinja`. The stock Qwen3.6 chat template has documented bugs
  (thinking-block handling, premature stops) — see
  [`froggeric/Qwen-Fixed-Chat-Templates`](https://huggingface.co/froggeric/Qwen-Fixed-Chat-Templates)
  and [`spiritbuun/buun-Qwen3.6-chat_template`](https://huggingface.co/spiritbuun/buun-Qwen3.6-chat_template),
  both built on top of the official template and loadable via `--chat-template-file`. llama.cpp's
  own [function-calling docs](https://github.com/ggml-org/llama.cpp/blob/master/docs/function-calling.md)
  cover the generic OpenAI-style tool-call path these templates plug into.
- VRAM @32K context: only the 16 full-attention layers keep a growing KV cache
  (2 x 4 KV heads x 256 head_dim x 2 bytes = 4096 B/token/layer x 16 layers = 64 KiB/token ->
  ~2.1 GiB at 32K tokens); the 48 linear-attention layers hold a fixed-size recurrent state that
  does not grow with context (well under 1 GiB). Weights (16.82 GiB) + KV (~2.1 GiB) + linear
  state/compute buffers (~0.5-1 GiB) ~= **19.5-20.5 GiB**, fits the 24 GB card when the server has
  it exclusively.

### 2. Laguna-XS-2.1 — Q3_K_M GGUF (bartowski requant)

- Model: [`poolside/Laguna-XS-2.1`](https://huggingface.co/poolside/Laguna-XS-2.1), license
  **OpenMDW-1.1** (full text at
  [`poolside/Laguna-XS-2.1-GGUF/LICENSE.md`](https://huggingface.co/poolside/Laguna-XS-2.1-GGUF/raw/main/LICENSE.md)):
  a permissive, MIT-like grant ("permission is hereby granted, free of charge, to deal in the
  Model Materials without restriction... including under all copyright, patent, database, and
  trade secret rights") with no field-of-use or commercial restriction — permits this use.
  MoE, 40 full-attention GQA layers (48 Q heads, 8 KV heads, head_dim 128), 256 experts / 8
  active, per [`config.json`](https://huggingface.co/poolside/Laguna-XS-2.1/raw/main/config.json).
  BF16 total size is 66.93 GB (per the official `poolside/Laguna-XS-2.1-GGUF` tree listing),
  consistent with the ~33B-total/3B-active MoE description in the card leads.
- GGUF: the **official** `poolside/Laguna-XS-2.1-GGUF` repo only ships `Q4_K_M` (20.27 GiB) and
  `BF16`. At Q4_K_M, weights (20.27 GiB) + 32K KV (~5.2 GiB, see calc below) = ~25.5 GiB, which
  **exceeds the 24 GB budget**. Used instead:
  [`bartowski/Laguna-XS-2.1-GGUF`](https://huggingface.co/bartowski/Laguna-XS-2.1-GGUF)
  (tagged `base_model:quantized:poolside/Laguna-XS-2.1`, `license:openmdw-1.1`, imatrix quant),
  file `Laguna-XS-2.1-Q3_K_M.gguf`, **15,575,904,128 bytes** (14.51 GiB), sha256
  `00dda85ab74afccafa99b66e4d564a6ef76278a8c633d358cad630e7622cf618`.
- llama.cpp support: confirmed by the llama.cpp project's own
  [`ggml-org/Laguna-XS-2.1-GGUF`](https://huggingface.co/ggml-org/Laguna-XS-2.1-GGUF) repo.
- Tool calling: `llama-server --jinja` against the shipped `chat_template.jinja` (model card lists
  vLLM/llama.cpp-style serving as supported). Community reliability forks exist (e.g.
  `gghfez/Laguna-XS.2-jlens-GGUF`) if the stock template proves flaky in the LLM-05 benchmark.
- VRAM @32K context: dense GQA every layer (no linear-attention shortcut here) —
  2 x 8 KV heads x 128 head_dim x 2 bytes = 4096 B/token/layer x 40 layers = 160 KiB/token ->
  ~5.24 GiB at 32K tokens. Weights (14.51 GiB) + KV (~5.24 GiB) = **~19.7 GiB**, fits 24 GB with
  the model run exclusively.

### 3. Qwen3-Coder-30B-A3B-Instruct — IQ4_XS GGUF (fallback per card leads)

- Model: [`Qwen/Qwen3-Coder-30B-A3B-Instruct`](https://huggingface.co/Qwen/Qwen3-Coder-30B-A3B-Instruct),
  Apache-2.0. MoE, 48 full-attention GQA layers (32 Q heads, 4 KV heads, head_dim 128), 128
  experts, per its
  [`config.json`](https://huggingface.co/Qwen/Qwen3-Coder-30B-A3B-Instruct/raw/main/config.json).
- GGUF: [`unsloth/Qwen3-Coder-30B-A3B-Instruct-GGUF`](https://huggingface.co/unsloth/Qwen3-Coder-30B-A3B-Instruct-GGUF),
  file `Qwen3-Coder-30B-A3B-Instruct-IQ4_XS.gguf`, **16,378,076,320 bytes** (16.38 GiB), sha256
  `26cd4fef3ada9de84c6c59c4b93f9bc5df4d69a71e18cfe8a17369c2f8374ce7`.
- llama.cpp support and tool calling: this is one of the most widely deployed llama.cpp
  coding-agent targets; the repo's own
  [discussion #10](https://huggingface.co/unsloth/Qwen3-Coder-30B-A3B-Instruct-GGUF/discussions/10)
  documents chat-template and tool-calling fixes (2025-08-05) for `llama-server --jinja`.
- VRAM @32K context: 2 x 4 KV heads x 128 head_dim x 2 bytes = 2048 B/token/layer x 48 layers =
  96 KiB/token -> ~3.15 GiB at 32K tokens. Weights (16.38 GiB) + KV (~3.15 GiB) = **~19.5 GiB**,
  the largest free headroom (~4.5 GiB) of the three for llama.cpp compute buffers.

## Download budget

16.82 + 14.51 + 16.38 = **47.71 GiB / 48.77 GB**, under the 60 GB cap, computed from the exact
`size_bytes` above (no headroom assumed beyond that).

## Deletion policy

None of these candidates are downloaded by this card (LLM-01 is source-verification only). When
LLM-05 runs its project benchmark, it downloads all three, and after the benchmark selects a
winner, the non-selected candidates' GGUF files are deleted from local disk (`models/llm/` or
wherever LLM-05 stages them) — they remain available for re-download from Hugging Face if needed
later, so nothing is lost by deleting the local copies.

## Considered and not selected

- **Gemma 4 26B-A4B-it** — [`google/gemma-4-26B-A4B-it`](https://huggingface.co/google/gemma-4-26B-A4B-it)
  tags list `license:apache-2.0`, but Google's Gemma family has historically shipped under the
  separate Gemma Terms of Use (a usage-restriction license, not plain Apache-2.0), and the HF tag
  alone isn't sufficient primary-source confirmation — the actual `LICENSE`/Gemma-Terms text needs
  checking before this can be a bounded candidate. Deferred rather than spending part of the
  3-candidate cap on an unverified license; llama.cpp support looked plausible
  (`ggml-org/gemma-4-26B-A4B-it-GGUF` exists) if a future card wants to re-open this.

## Re-check note for future cards

The card's starting leads were dated 2026-09-28 and are re-verified as of this same date in this
document; before LLM-05 downloads anything, re-run the HF `tree/main` API calls for the three
`repo`/`file` pairs in `config/llm_candidates.json` to confirm the `sha256` and `size_bytes`
haven't changed (quantizer repos are occasionally re-uploaded).

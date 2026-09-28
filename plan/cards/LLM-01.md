# LLM-01 — Local coding model shortlist from primary sources (no downloads)

```json card
{
  "kind": "research",
  "requirements": [
    "REQ-LLM-1"
  ],
  "scope": {
    "write": [
      "docs/llm/SELECTION.md",
      "config/llm_candidates.json"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "schema",
        "cmd": "python3 -c \"import json;d=json.load(open('config/llm_candidates.json'));c=d['candidates'];assert 2<=len(c)<=3;assert all(x.get(k) for x in c for k in ('repo','file','size_bytes','sha256','license','quant','context','source_urls'));assert sum(x['size_bytes'] for x in c)<=60e9\""
      }
    ],
    "criteria": [
      "Every claim cites a primary source URL (model card, vendor blog/report, license text); secondary rankings only as leads",
      "Candidates fit 24 GB with ≥32K context KV at the chosen quant, with the server started on demand (never co-resident with nnU-Net/MoveIt GPU use)",
      "Tool-calling support in llama.cpp (--jinja chat template) confirmed per candidate",
      "Licenses permit this use; sha256 taken from the Hugging Face API (lfs.sha256), not computed after download",
      "Total download budget ≤ 60 GB; deletion policy for non-selected models stated"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 2
  },
  "track": "automation",
  "priority": 55,
  "id": "LLM-01",
  "title": "Local coding model shortlist from primary sources (no downloads)",
  "parent": "L-LLM",
  "outcome": "A bounded, source-cited shortlist (≤3) with exact files, sizes, hashes, licenses and a VRAM-coexistence plan."
}
```

Starting leads (verified 2026-09-28, re-check them): Qwen3.6-27B (HF Qwen/Qwen3.6-27B, Apache-2.0, dense 27B, vendor SWE-bench
Verified 77.2 with its own scaffold, thinking on by default); Laguna XS 2.1 (HF poolside/Laguna-XS-2.1, OpenMDW-1.1, MoE 33B/3B
active); Gemma 4 26B-A4B (MoE ~4B active; verify license); fallback Qwen3-Coder-30B-A3B-Instruct (Apache-2.0). Vendor benchmark numbers
are not comparable to our loop — selection happens in LLM-05 from the project benchmark. Use the HF API
(https://huggingface.co/api/models/<repo>/tree/main) for sizes and sha256 of GGUF files from the official org or a reputable quantizer.
Serving route: llama.cpp llama-server OpenAI-compatible API consumed by ~/claude-auto/claude_auto/localcoder.py (NOT Claude Code).

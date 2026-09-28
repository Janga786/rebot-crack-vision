# LLM-03 — Download + verify shortlisted models (bounded, sequential)

```json card
{
  "kind": "impl",
  "depends_on": [
    "LLM-01"
  ],
  "requirements": [
    "REQ-LLM-1"
  ],
  "scope": {
    "write": [
      "scripts/llm/download_models.py",
      "scripts/llm/verify_models.py",
      "config/llm_models.json",
      "docs/llm/DOWNLOADS.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "verify",
        "cmd": "python3 scripts/llm/verify_models.py",
        "timeout_s": 1800
      }
    ],
    "criteria": [
      "Downloads one file at a time; refuses to start if free disk would drop below 40 GB",
      "Each file's sha256 matches config/llm_candidates.json; partial files are resumed or deleted, never left as 'verified'",
      "config/llm_models.json lists local paths, sizes, hashes, verification time"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 1,
    "context": 2,
    "consequence": 2
  },
  "track": "automation",
  "timeout_min": {
    "impl": 240
  },
  "priority": 40,
  "extra_rw": [
    "~/models/llm"
  ],
  "id": "LLM-03",
  "title": "Download + verify shortlisted models (bounded, sequential)",
  "parent": "L-LLM",
  "outcome": "Shortlisted GGUF files are on disk with verified sha256, within the disk budget."
}
```



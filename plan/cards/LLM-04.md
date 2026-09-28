# LLM-04 — Project benchmark harness for local coding models

```json card
{
  "kind": "impl",
  "depends_on": [
    "LLM-02",
    "LLM-03"
  ],
  "requirements": [
    "REQ-LLM-1",
    "REQ-LLM-2"
  ],
  "scope": {
    "write": [
      "bench/llm/**",
      "docs/llm/BENCHMARK.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "selftest",
        "cmd": "python3 bench/llm/run_bench.py --self-test",
        "timeout_s": 600
      }
    ],
    "criteria": [
      "≥12 tasks in ≥3 task classes (e.g. 'script', 'test-writing', 'mechanical-edit'), each with an automated pass/fail checker; tasks drawn from this repo's real patterns",
      "Harness starts/stops llama-server per model, runs tasks via ~/claude-auto/claude_auto/localcoder.run_local (sys.path import), records pass@1, tool-call validity, wall time, tokens/s, peak VRAM (nvidia-smi sampling)",
      "--self-test validates checkers against reference solutions without any model"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 3
  },
  "resources": [
    "gpu"
  ],
  "track": "automation",
  "timeout_min": {
    "impl": 180
  },
  "priority": 35,
  "id": "LLM-04",
  "title": "Project benchmark harness for local coding models",
  "parent": "L-LLM",
  "outcome": "A repeatable benchmark of project-representative tasks drives the local route exactly as the scheduler would.",
  "gpu_min_free_mb": 22000
}
```



# LLM-05 — Run benchmark, select model, qualify task classes

```json card
{
  "kind": "impl",
  "depends_on": [
    "LLM-04"
  ],
  "requirements": [
    "REQ-LLM-1",
    "REQ-LLM-2"
  ],
  "scope": {
    "write": [
      "docs/llm/BENCHMARK_RESULTS.md",
      "evidence/llm/qualification.json",
      "evidence/llm/results/**"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "qual",
        "cmd": "python3 -c \"import json;q=json.load(open('evidence/llm/qualification.json'));assert isinstance(q['runner_ok'],bool);assert all(isinstance(c,str) for c in q['classes']);s=q['serving'];assert s['endpoint'].startswith('http://127.0.0.1') and s['start_argv'][0].startswith('/') and s['model']\""
      }
    ],
    "criteria": [
      "Each candidate run with identical settings; results tables committed",
      "qualification.json: runner_ok, model, classes (only those ≥80% pass on ≥5 tasks), thresholds, serving {endpoint on 127.0.0.1, model, start_argv with absolute paths, ctx, start_timeout_s}",
      "Non-selected model files deleted; disk usage reported",
      "If no candidate qualifies, runner_ok=false with the evidence — that is an acceptable outcome"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 3
  },
  "resources": [
    "gpu"
  ],
  "track": "automation",
  "timeout_min": {
    "impl": 240
  },
  "priority": 35,
  "extra_rw": [
    "~/models/llm"
  ],
  "id": "LLM-05",
  "title": "Run benchmark, select model, qualify task classes",
  "parent": "L-LLM",
  "outcome": "Evidence-based model choice; only task classes with ≥80% pass are qualified for local routing; losers deleted.",
  "gpu_min_free_mb": 22000,
  "publishes_local_qualification": "evidence/llm/qualification.json"
}
```



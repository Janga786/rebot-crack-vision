# PERC-03 — Ordered crack paths + path contract

```json card
{
  "kind": "impl",
  "depends_on": [
    "PERC-02"
  ],
  "requirements": [
    "REQ-PERC-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/paths.py",
      "tests/test_paths.py",
      "docs/INTERFACES.md",
      "config/project.yaml"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#3.4",
    "docs/INTERFACES.md#0.5",
    "presentation/demo/crack_to_path.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "pytest",
        "cmd": "./env.sh pytest tests/test_paths.py -q"
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q",
        "timeout_s": 1200
      }
    ],
    "criteria": [
      "Main path = longest simple path per component (graph diameter); branches emitted as separate polylines ordered by length; policy documented",
      "Multi-component ordering minimises travel (greedy nearest-endpoint tour with direction choice); deterministic",
      "RDP simplification tolerance in px from config; original dense path kept alongside the simplified one",
      "data/paths/{case}_paths.json schema appended as a new INTERFACES section: (row, col) ints in the original frame + (u, v) = (col, row) explicitly",
      "Tests cover straight, curved, branched, looped and multi-crack synthetic skeletons"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4
  },
  "id": "PERC-03",
  "title": "Ordered crack paths + path contract",
  "parent": "L-PERC",
  "outcome": "Each case yields ordered, simplified polylines (main path + branches) in pixel coordinates under a written contract."
}
```

presentation/demo/crack_to_path.py is prior art (two-BFS diameter + RDP) — reuse ideas, not code quality claims.

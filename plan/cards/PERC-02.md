# PERC-02 — Skeleton graph: junctions, endpoints, spur pruning

```json card
{
  "kind": "impl",
  "depends_on": [
    "TC-010",
    "ARCH-01"
  ],
  "requirements": [
    "REQ-PERC-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/skeleton_graph.py",
      "tests/test_skeleton_graph.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#3.4",
    "docs/INTERFACES.md#0.5"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "pytest",
        "cmd": "./env.sh pytest tests/test_skeleton_graph.py -q"
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q",
        "timeout_s": 1200
      }
    ],
    "criteria": [
      "8-connected pixel graph from the {0,255} skeleton PNG; nodes keep (row, col) exactly (no resize)",
      "Junction/endpoint classification correct on synthetic Y, X, T, loop, spur and multi-component skeletons",
      "Spur pruning by length in px (configurable, default from config/project.yaml new key), never removes a component's main path",
      "Graph serialisable to JSON (nodes, edges with pixel lists); deterministic output"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 2,
    "consequence": 3
  },
  "id": "PERC-02",
  "title": "Skeleton graph: junctions, endpoints, spur pruning",
  "parent": "L-PERC",
  "outcome": "A tested pixel-graph representation of the 1-px skeleton with configurable spur pruning, in the original frame."
}
```



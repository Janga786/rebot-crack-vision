# HOST-06 — Host reproducibility audit (fresh clone)

```json card
{
  "kind": "branch",
  "depends_on": [
    "HOST-01",
    "HOST-02",
    "LLM-02",
    "TC-016"
  ],
  "requirements": [
    "REQ-HOST-1",
    "REQ-OPS-2"
  ],
  "refine_after": [
    "HOST-01",
    "HOST-02",
    "LLM-02"
  ],
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 3
  },
  "id": "HOST-06",
  "title": "Host reproducibility audit (fresh clone)",
  "parent": "L-HOST",
  "outcome": "A fresh clone + documented steps reproduce the environment checks and test suite."
}
```

Draft: specify after the manifest, lock and llama.cpp build exist.

## Decomposed 2026-10-09T05:13:46Z by L-HOST (claude-sonnet-5, high)
Children: HOST-06.1

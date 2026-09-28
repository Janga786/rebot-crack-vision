# MOT-05 — Commissioning-gated execution interface (mock/dry/real)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-03",
    "MOT-01"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "spec_state": "draft",
  "refine_after": [
    "MOT-02"
  ],
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 5
  },
  "track": "simulation",
  "id": "MOT-05",
  "title": "Commissioning-gated execution interface (mock/dry/real)",
  "parent": "L-MOTION",
  "outcome": "Commissioning-gated execution interface (mock/dry/real)"
}
```

Draft: Safety-critical: typed confirmation, speed scaling ≤10% first runs, limit-file hash check, e-stop declaration, logs; tests in mock only; reuse ~/rebot_ws/deploy patterns.

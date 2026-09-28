# MOT-08 — Trajectory preview + operator approval artefact

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-07"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "spec_state": "draft",
  "refine_after": [
    "MOT-02"
  ],
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4
  },
  "track": "simulation",
  "id": "MOT-08",
  "title": "Trajectory preview + operator approval artefact",
  "parent": "L-MOTION",
  "outcome": "Trajectory preview + operator approval artefact"
}
```

Draft: Headless render + RViz; approval bound to trajectory file hash.

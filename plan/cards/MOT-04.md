# MOT-04 — Reachability map + recommended specimen placement

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-02",
    "GEOM-01"
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
  "id": "MOT-04",
  "title": "Reachability map + recommended specimen placement",
  "parent": "L-MOTION",
  "outcome": "Reachability map + recommended specimen placement"
}
```

Draft: Reuse the presentation §2.5 sweep idea with MoveIt IK.

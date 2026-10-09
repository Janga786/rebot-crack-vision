# MOT-05 — Commissioning-gated execution interface (mock/dry/real)

```json card
{
  "kind": "branch",
  "depends_on": [
    "MOT-03",
    "MOT-01",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
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

## ADR-014 guidance (technical-lead recovery, 2026-09-30)
- The real-execution gate must call crackvision_description.end_effector.assert_commissioning_ready (tool tip and
  camera extrinsic measured) in addition to scene_core.assert_commissioning_ready, and refuse otherwise.

## Decomposed 2026-10-09T05:59:34Z by L-MOTION (claude-opus-5-5, high)
Children: MOT-05.1, MOT-05.2, MOT-05.3, MOT-05.4, MOT-05.5, MOT-05.6

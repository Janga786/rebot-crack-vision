# MOT-06 — Cartesian path planning along crack paths

```json card
{
  "kind": "branch",
  "depends_on": [
    "MOT-03",
    "MOT-04",
    "GEOM-08",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "refine_after": [
    "MOT-02",
    "GEOM-08"
  ],
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4
  },
  "track": "simulation",
  "id": "MOT-06",
  "title": "Cartesian path planning along crack paths",
  "parent": "L-MOTION",
  "outcome": "Cartesian path planning along crack paths"
}
```

Draft: approach → inspection → retract, computeCartesianPath, fallbacks, coverage report.

## ADR-014 guidance (technical-lead recovery, 2026-09-30)
- Phases (ADR-014 §2): view (camera_link at ~0.25 m, optical axis within 15 deg of the anti-normal, whole specimen
  in the D405 FOV) -> approach (tool_tip at 0.04 m) -> trace (tool_tip at 0.01 m, computeCartesianPath with
  link_name tool_tip, roll free for continuity) -> retract. gripper_tcp is for grasping only.
- Collision scene = config/scene/scene.yaml (table + specimen) + the robot model's camera proxies.

## Decomposed 2026-10-11T00:46:04Z by L-MOTION (claude-opus-5-5, high)
Children: MOT-06.1, MOT-06.10, MOT-06.2, MOT-06.3, MOT-06.4, MOT-06.5, MOT-06.6, MOT-06.7, MOT-06.8, MOT-06.9

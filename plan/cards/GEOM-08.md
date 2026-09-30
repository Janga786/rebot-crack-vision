# GEOM-08 — Pixel paths → robot-frame 3D paths with uncertainty

```json card
{
  "kind": "branch",
  "depends_on": [
    "GEOM-02",
    "GEOM-03",
    "PERC-03",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-GEOM-1",
    "REQ-GEOM-2"
  ],
  "refine_after": [
    "GEOM-03",
    "PERC-03"
  ],
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 5
  },
  "track": "calibration",
  "id": "GEOM-08",
  "title": "Pixel paths → robot-frame 3D paths with uncertainty",
  "parent": "L-GEOM",
  "outcome": "Ordered 2D paths + depth + calibration → 3D tool waypoints (position + orientation) with per-point uncertainty."
}
```

Draft: needs the path contract and calibration plan.

## ADR-014 guidance (technical-lead recovery, 2026-09-30)
- 3D path: p_base = FK(q_capture) · T_gripper_link_camera_link · T_camera_link_optical · p_optical (INTERFACES §8.4):
  every capture must carry the arm joint state at the capture instant and the end_effector.yaml sha256; refuse
  captures without them. Uncertainty must include the camera extrinsic's status (nominal => not for execution).
- Tool waypoints are poses of tool_tip: boresight (+X) along the local surface anti-normal, free roll, positions at
  the trace clearance (nominal 0.01 m) above the surface; approach/retract at 0.04 m (ADR-014 §2).

## Decomposed 2026-09-30T09:28:16Z by L-GEOM (claude-opus-5-5, high)
Children: GEOM-08.1, GEOM-08.2, GEOM-08.3, GEOM-08.4, GEOM-08.5, GEOM-08.6, GEOM-08.7, GEOM-08.8

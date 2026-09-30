# GEOM-04 — Hand-eye calibration software (synthetic-verified)

```json card
{
  "kind": "branch",
  "depends_on": [
    "GEOM-02",
    "GEOM-03",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-GEOM-2"
  ],
  "refine_after": [
    "GEOM-03"
  ],
  "track": "calibration",
  "id": "GEOM-04",
  "title": "Hand-eye calibration software (synthetic-verified)",
  "parent": "L-GEOM",
  "outcome": "Hand-eye solver + capture tooling verified on synthetic data."
}
```

Draft: specify after the method decision.

## ADR-014 guidance (technical-lead recovery, 2026-09-30)
- Eye-in-hand (operator decision): solve AX = XB for X = T_gripper_link_camera_link (or the colour optical frame,
  converting with the driver's camera_link->optical TF), ChArUco fixed on the table, robot poses from FK.
- Initial guess / sanity prior: config/robot/end_effector.yaml wrist_camera (nominal CAD prior). Flag (do not average
  away) a solution > 10 mm / 5 deg from it: that indicates the wrong mount side, a seating problem or a bad solve.
- Output a measured wrist_camera block for end_effector.yaml (value_status measured + residuals + evidence path);
  GEOM-05 writes it. Synthetic tests must include the 15 deg stock pitch and the -Z mount side.

## Decomposed 2026-09-30T09:40:34Z by L-GEOM (claude-sonnet-5, high)
Children: GEOM-04.1, GEOM-04.2, GEOM-04.3, GEOM-04.4, GEOM-04.5

# MOT-10.1.R1 — Repair MOT-10.1: Rectangularity refusal can never fire: corners are tested against a mi

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "config/scene/workcell_survey.template.yaml",
      "docs/INTERFACES.md",
      "docs/motion/WORKCELL_SURVEY.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#7",
    "docs/INTERFACES.md#8",
    "docs/INTERFACES.md#11",
    "docs/motion/SCENE.md",
    "docs/motion/ROBOT_MODEL.md",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "config/scene/scene.yaml",
    "config/motion/reachability.yaml",
    "config/motion/specimen_placement.yaml",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "sections",
        "cmd": "bash -c 'test $(grep -c \"^## 11\\. \" docs/INTERFACES.md) -eq 1 && test $(grep -c \"^## 12\\. \" docs/INTERFACES.md) -eq 1 && grep -qF crackvision.workcell_survey/1 docs/INTERFACES.md'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "doc",
        "cmd": "bash -c 'test -s docs/motion/WORKCELL_SURVEY.md && grep -qF crackvision.workcell_survey/1 docs/motion/WORKCELL_SURVEY.md'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "template-yaml",
        "cmd": "./env.sh python -c \"import yaml,sys; d=yaml.safe_load(open('config/scene/workcell_survey.template.yaml')); sys.exit(0 if d.get('schema')=='crackvision.workcell_survey/1' else 1)\"",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "§12.7 defines a fit whose residual is non-trivial, with one exact metric. Examples: a least-squares rectangle fit with per-corner distance to the fitted rectangle's matching corner, or diagonal-length difference plus side-length differences. A worked numeric example shows a 5 mm-skewed corner being refused at the 3 mm default.",
      "§12.7 gives explicit formulas mapping every reading to base_link. It defines the sign conventions for the x/y offsets (including the near edge and the centreline y sign) and the obstacle z datum. It defines the table box's full pose and dimensions, including thickness, from either a measured field or a stated normative constant. The template comments state the same sign conventions.",
      "The template and §12 agree. Either the operator replaces the map with `null` when bolted directly (template comment says so), or rule 6 explicitly exempts adapter_plate_thickness when bolted_directly_to_table is true.",
      "All acceptance criteria of MOT-10.1 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4,
    "task_class": "decision+contract",
    "local_ok": false
  },
  "priority": 95,
  "repairs": "MOT-10.1",
  "findings": [
    "a6e8f3fe26e0f609",
    "e40bba643dbc95ec",
    "266663b1618db649"
  ],
  "id": "MOT-10.1.R1",
  "title": "Repair MOT-10.1: Rectangularity refusal can never fire: corners are tested against a mi",
  "parent": "MOT-10",
  "outcome": "Resolve the review findings on MOT-10.1 while every acceptance criterion of MOT-10.1 still holds."
}
```

Repair work generated deterministically from review attempt `00265-MOT-10.1-review` of `MOT-10.1`.
Original card: `plan/cards/MOT-10.1.md` — its acceptance criteria must still hold.

## Finding 1 [major] Rectangularity refusal can never fire: corners are tested against a minimum-area enclosing rectangle
- Evidence: docs/INTERFACES.md §12.7 rule 1: "fit the minimum-area rectangle through the 4 `corners` ... **Refuse** if any corner lies more than `rectangularity_tolerance_m` from the fitted rectangle." Every input point lies inside or on the boundary of its minimum-area bounding rectangle, so the distance from a corner to that rectangle is always 0. WORKCELL_SURVEY.md §5 repeats the same rule.
- Consequence: A badly mis-measured or trapezoidal set of corners passes. The specimen footprint and yaw become the inflated bounding box, which defeats the card's stated rectangularity-tolerance refusal. MOT-10.3 would have to invent its own fit and metric.
- Affected: docs/INTERFACES.md, docs/motion/WORKCELL_SURVEY.md
- Acceptance condition: §12.7 defines a fit whose residual is non-trivial, with one exact metric. Examples: a least-squares rectangle fit with per-corner distance to the fitted rectangle's matching corner, or diagonal-length difference plus side-length differences. A worked numeric example shows a 5 mm-skewed corner being refused at the 3 mm default.

## Finding 2 [major] Offset→base_link conversion, sign conventions and table box thickness are not specified, so derivation is not deterministic
- Evidence: §12.7 rule 1 says only "in the base-face coordinates of §12.1, i.e. `x = front_face_offset`, `y = centerline_offset`". It never states base_link x = 0.070 + x_from_front_face, or which y direction is positive. §12.4(b) `front_face_to_near_edge` is described as "behind the base's -x side (may be a small or negative offset...)": it is unclear which face it is measured from and what the sign is. Rule 3 gives the table top z but no box thickness or bottom face, although config/scene/scene.yaml's table box needs dimensions_m[2] (currently 0.05 nominal). Rule 3 also says "relative to `x = ±0.100` side faces" (should be y). Rule 4 does not say whether obstacle z_from_table_top is offset by -(adapter plate thickness).
- Consequence: MOT-10.3 implementers, and reviewers checking them, cannot derive a unique scene from a survey. Sign or reference-face mistakes could put the table, specimen or obstacles in the wrong place in a 'measured' collision scene used for commissioning.
- Affected: docs/INTERFACES.md, docs/motion/WORKCELL_SURVEY.md, config/scene/workcell_survey.template.yaml
- Acceptance condition: §12.7 gives explicit formulas mapping every reading to base_link. It defines the sign conventions for the x/y offsets (including the near edge and the centreline y sign) and the obstacle z datum. It defines the table box's full pose and dimensions, including thickness, from either a measured field or a stated normative constant. The template comments state the same sign conventions.

## Finding 3 [minor] Template's bolted-directly instruction conflicts with refusal rule 6 (null readings)
- Evidence: Template: `adapter_plate_thickness: # required (fill in) iff bolted_directly_to_table is false; else leave all null` with a 4-key map of nulls. §12.4(a) says the field is `null` when bolted directly. §12.7 rule 6 refuses "any reading anywhere in the file with a `null` `value_m`, `instrument`...". Parsing shows {'value_m': None, ...}, not None.
- Consequence: An operator who follows the template exactly for the normal bolted-directly case gets a survey that a literal implementation of the contract refuses. Or MOT-10.3 has to guess an exception.
- Affected: config/scene/workcell_survey.template.yaml, docs/INTERFACES.md
- Acceptance condition: The template and §12 agree. Either the operator replaces the map with `null` when bolted directly (template comment says so), or rule 6 explicitly exempts adapter_plate_thickness when bolted_directly_to_table is true.

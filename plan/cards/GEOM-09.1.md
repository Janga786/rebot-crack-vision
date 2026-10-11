# GEOM-09.1 — Decision + contract: pixel-to-tool-pose uncertainty budget (ADR-018, INTERFACES §14)

```json card
{
  "kind": "decision",
  "requirements": [
    "REQ-GEOM-1"
  ],
  "scope": {
    "write": [
      "docs/adr/018-uncertainty-budget.md",
      "docs/INTERFACES.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#9. Eye-in-hand capture record",
    "docs/INTERFACES.md#10. Robot-frame 3D crack paths and tool waypoints",
    "src/crackvision/kinematics.py",
    "src/crackvision/lift3d.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "adr-exists",
        "cmd": "test -f docs/adr/018-uncertainty-budget.md",
        "timeout_s": 10,
        "expect_exit": 0
      },
      {
        "id": "interfaces-section",
        "cmd": "grep -q '^## 14\\.' docs/INTERFACES.md",
        "timeout_s": 10,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "ADR-018 names every uncertainty source feeding the budget: pixel/skeleton localization noise, D405 stereo depth noise, camera intrinsics, hand-eye calibration sigma_calib (GEOM-04.3, rot+pos), TCP/pivot calibration sigma (GEOM-06), and FK joint uncertainty if modeled; it states which are already folded into GEOM-08.5/§10.4's per-point covariance and which this card must add.",
      "ADR-018 specifies the analytic method as a linearized (first-order Jacobian) decomposition that attributes the total pixel-to-tool-pose covariance to named additive per-term contributions, reusing §10.4's existing propagation rather than replacing it.",
      "ADR-018 specifies the Monte-Carlo method: per-term perturbation distributions, number of samples, which stages of the pixel->base_link->tool_tip chain are resampled, and the statistic used to compare empirical vs analytic covariance (e.g. ratio of eigenvalues/trace within a documented tolerance) plus how disagreement is flagged.",
      "INTERFACES.md §14 defines the `crackvision.uncertainty_budget/1` JSON report schema: per-point and per-path/per-waypoint total sigma, the named per-term contribution breakdown, the MC-vs-analytic agreement statistic and a pass/fail flag, units and frame for every quantity, and how it references the source paths3d/capture-record/calibration files it was computed from.",
      "The decision explicitly does not reopen or contradict any accepted GEOM-04/GEOM-06/GEOM-08 contract; it only adds a new aggregation layer on top of their already-accepted sigmas."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 3,
    "task_class": "design_decision",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-09.1",
  "title": "Decision + contract: pixel-to-tool-pose uncertainty budget (ADR-018, INTERFACES §14)",
  "parent": "GEOM-09",
  "outcome": "ADR-018 and a new INTERFACES.md §14 define the uncertainty-budget terms, the analytic propagation method, the Monte-Carlo cross-check method and the `crackvision.uncertainty_budget/1` report file contract."
}
```

Write `docs/adr/018-uncertainty-budget.md` and append `docs/INTERFACES.md` §14 (`crackvision.uncertainty_budget/1`).

Context: GEOM-08.5/§10.4 already propagate a first-order per-point covariance through pixel annulus sampling, plane-fit normals and the eye-in-hand chain (`camera_sigma_pos_m`/`camera_sigma_rot_rad` from `kinematics.EndEffector`, GEOM-04.3's hand-eye `sigma_calib`, GEOM-06's TCP/pivot sigma). GEOM-09 is scoped to *decompose and audit* that pipeline end-to-end from pixel to tool pose, not to re-derive it: produce a named contribution breakdown (how much of the final sigma comes from pixel noise vs depth noise vs hand-eye calibration vs TCP calibration) and cross-check the linearized result against a Monte-Carlo resample of the same chain.

Deliverables of this card are the ADR (why this decomposition, why linearization is adequate or where it is expected to break down, why MC is used as the cross-check rather than a second analytic derivation) and the §14 contract (exact JSON schema, units, frames, tolerance for MC-vs-analytic agreement). Keep it implementable by GEOM-09.2-.4 without further clarification: list every input file each later module will read (paths3d file §10, capture record §9, hand-eye/TCP calibration output files) and every field the budget report must carry.

Do not modify any accepted GEOM-04/06/08 file or contract — §14 is additive.

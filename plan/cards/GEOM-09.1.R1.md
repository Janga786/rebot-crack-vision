# GEOM-09.1.R1 — Repair GEOM-09.1: §14 schema is self-contradictory: adding the new FK term cannot both l

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-GEOM-1"
  ],
  "scope": {
    "write": [
      "docs/INTERFACES.md",
      "docs/adr/018-uncertainty-budget.md"
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
      "Revise §14.4/§14.5 (and the ADR §2 'sum exactly to the existing Σ_base' claim) so that sigma_total_m / sigma_along_normal_m in the report are defined as the NEW total including the fk term (necessarily ≥ the accepted paths3d sigma_base_m / sigma_along_normal_m), with contributions_m2 summing to that new total — not to the unchanged, FK-less value copied from the accepted paths3d file. The ADR and schema must agree on a single, non-contradictory definition of 'total' before this is implementable.",
      "All acceptance criteria of GEOM-09.1 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 3,
    "task_class": "design_decision",
    "local_ok": false
  },
  "track": "calibration",
  "priority": 99,
  "repairs": "GEOM-09.1",
  "findings": [
    "66405e3ae8710943"
  ],
  "id": "GEOM-09.1.R1",
  "title": "Repair GEOM-09.1: §14 schema is self-contradictory: adding the new FK term cannot both l",
  "parent": "GEOM-09",
  "outcome": "Resolve the review findings on GEOM-09.1 while every acceptance criterion of GEOM-09.1 still holds."
}
```

Repair work generated deterministically from review attempt `00526-GEOM-09.1-review` of `GEOM-09.1`.
Original card: `plan/cards/GEOM-09.1.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] §14 schema is self-contradictory: adding the new FK term cannot both leave totals unchanged and sum to them
- Evidence: docs/INTERFACES.md §14.4: "sigma_total_m ... identical to the source point's own sigma_base_m (§10.1)" and "contributions_m2 ... {pixel, depth, handeye, fk} ... sums exactly to the source point's cov_base_m2 under linearization (ADR-018 §2)". docs/INTERFACES.md §14.5 makes the parallel claim for sigma_along_normal_m. But ADR-018 §1/§3 defines Σ_base^(fk) as a new, nonzero additive covariance term (from σ_joint_rad=0.001 rad propagated through the FK Jacobian) that is explicitly *not* part of the existing (accepted, unchanged) cov_base_m2/sigma_base_m/sigma_along_normal_m — §10.4's own `unmodelled` list names `joint_encoder_and_fk`, and ADR-018's Consequences section states "cov_base_m2 and sigma_along_normal_m keep their existing meaning and values". A sum of pixel+depth+handeye+fk cannot equal the same total as pixel+depth+handeye alone unless fk's contribution is zero, which contradicts the entire premise of adding an FK term.
- Consequence: GEOM-09.2 (the analytic-decomposition implementer) cannot satisfy this schema literally: it must choose between (a) making contributions_m2 sum to a value larger than the accepted cov_base_m2 (violating the explicit 'sums exactly to cov_base_m2' and 'identical to sigma_base_m' schema text), or (b) reporting a zero/fabricated fk contribution to force the sums to match (defeating the stated purpose of the new term and silently misrepresenting FK uncertainty in a report meant to flag within_budget risk). Either path requires a judgment call the ADR/contract was supposed to pre-resolve, violating the card's own requirement to be 'implementable by GEOM-09.2-.4 without further clarification.'
- Affected: docs/INTERFACES.md, docs/adr/018-uncertainty-budget.md
- Acceptance condition: Revise §14.4/§14.5 (and the ADR §2 'sum exactly to the existing Σ_base' claim) so that sigma_total_m / sigma_along_normal_m in the report are defined as the NEW total including the fk term (necessarily ≥ the accepted paths3d sigma_base_m / sigma_along_normal_m), with contributions_m2 summing to that new total — not to the unchanged, FK-less value copied from the accepted paths3d file. The ADR and schema must agree on a single, non-contradictory definition of 'total' before this is implementable.

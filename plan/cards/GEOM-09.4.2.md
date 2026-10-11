# GEOM-09.4.2 — Write docs/geometry/UNCERTAINTY_BUDGET.md

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-09.4.1"
  ],
  "requirements": [
    "REQ-GEOM-1"
  ],
  "scope": {
    "write": [
      "docs/geometry/UNCERTAINTY_BUDGET.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#14. Pixel-to-tool-pose uncertainty budget",
    "docs/adr/018-uncertainty-budget.md",
    "docs/geometry/PATH3D_VERIFICATION.md",
    "src/crackvision/uncertainty_budget.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "doc-content",
        "cmd": "grep -q 'crackvision.uncertainty_budget/1' docs/geometry/UNCERTAINTY_BUDGET.md && grep -q 'python -m crackvision.uncertainty_budget' docs/geometry/UNCERTAINTY_BUDGET.md && grep -qi 'not.*physical\\|unmodeled\\|unmodelled' docs/geometry/UNCERTAINTY_BUDGET.md",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The doc explains, in plain terms, what the `crackvision.uncertainty_budget/1` report contains (per-point/per-waypoint named-term breakdown, the new `fk` term, the MC-vs-analytic agreement check, `summary.overall_pass`) and how an operator/reviewer would use it (e.g. to see which named source dominates a `within_budget: false` waypoint).",
      "The doc gives the exact CLI invocation (`./env.sh python -m crackvision.uncertainty_budget ...`), its key flags, and both possible outcomes (clean report, exit 0; the file is still written but the CLI exits non-zero when any point/waypoint is flagged) — matching `uncertainty_budget.py`'s actual documented exit-code convention, not a guessed one.",
      "The doc states explicitly, at the same honesty standard as PATH3D_VERIFICATION.md's own caveat, that this report budgets only the *modelled* uncertainty sources (pixel, depth, hand-eye, TCP, FK) and is not a measurement of physical accuracy or a substitute for the §10.4/ADR-018 `unmodelled` list (distortion Jacobian, depth bias, capture-time skew/motion, thermal drift) or for real hardware verification (GEOM-05/07, INT-04/05).",
      "The doc states the camera-intrinsics caveat verbatim (or paraphrased but faithfully) from §14.2/§14.3: intrinsics uncertainty is not a separate additive term, only indirectly exercised by the Monte-Carlo resample's nonlinear geometry.",
      "The doc accurately reflects `uncertainty_budget.py`'s actual behaviour as implemented in GEOM-09.4.1 (flag names, config file path/schema, default MC parameters, output file path) rather than restating the draft spec independent of what was actually built."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 1,
    "context": 2,
    "consequence": 2,
    "task_class": "docs",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-09.4.2",
  "title": "Write docs/geometry/UNCERTAINTY_BUDGET.md",
  "parent": "GEOM-09.4",
  "outcome": "docs/geometry/UNCERTAINTY_BUDGET.md documents the `crackvision.uncertainty_budget` report and CLI: what it means, how to run it, and its honest scope/limits, at the same honesty standard as docs/geometry/PATH3D_VERIFICATION.md."
}
```

Write `docs/geometry/UNCERTAINTY_BUDGET.md` once GEOM-09.4.1's `src/crackvision/uncertainty_budget.py` is accepted, so the doc describes what was actually built (flag names, defaults, output paths, exit-code convention) rather than the draft spec in isolation.

Follow `docs/geometry/PATH3D_VERIFICATION.md`'s structure and tone: a short purpose section, how to run the CLI (one worked example invocation with real flags), what the output file contains in plain language (not a restatement of the full §14 schema table — link to docs/INTERFACES.md §14 for the exact schema), and a clearly-labelled "Scope of what this verifies" / limits section stating this is a modelled-uncertainty budget, not a physical-accuracy measurement, and naming the unmodelled sources (distortion Jacobian, depth bias, capture-time skew/motion, thermal drift) and the intrinsics non-separability caveat. State the exit-code convention precisely: `overall_pass: true` -> exit 0; the file is written either way, but the CLI exits non-zero when `overall_pass` is `false`, exactly matching what `uncertainty_budget.py`'s `--help`/docstrings document. Do not invent behaviour not present in the accepted implementation.

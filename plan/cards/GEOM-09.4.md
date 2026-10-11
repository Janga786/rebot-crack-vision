# GEOM-09.4 — Uncertainty budget CLI + report (crackvision.uncertainty_budget)

```json card
{
  "kind": "branch",
  "depends_on": [
    "GEOM-09.1",
    "GEOM-09.2",
    "GEOM-09.3"
  ],
  "requirements": [
    "REQ-GEOM-1"
  ],
  "refine_after": [
    "GEOM-09.1"
  ],
  "scope": {
    "write": [
      "src/crackvision/uncertainty_budget.py",
      "tests/test_uncertainty_budget_cli.py",
      "docs/geometry/UNCERTAINTY_BUDGET.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#14. Uncertainty budget",
    "docs/INTERFACES.md#10. Robot-frame 3D crack paths and tool waypoints",
    "src/crackvision/uncertainty.py",
    "src/crackvision/uncertainty_mc.py",
    "src/crackvision/path3d.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_uncertainty_budget_cli.py -q",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The CLI follows docs/INTERFACES.md §0 (exit codes 0/1/2/3, common flags) and takes a paths3d file (§10) as its primary input, same as other GEOM-08 CLIs.",
      "The written report validates against the exact `crackvision.uncertainty_budget/1` schema from §14: per-point/per-waypoint contribution breakdown, totals, MC-vs-analytic agreement statistic and pass/fail flag, with correct units/frames and references back to the source files.",
      "The CLI refuses (non-zero documented exit code) rather than silently emitting a report when the MC cross-check disagreement exceeds the §14 tolerance, so a report is never marked fine when the analytic/MC methods actually disagree.",
      "docs/geometry/UNCERTAINTY_BUDGET.md documents what the report means, its scope/limits (same honesty standard as PATH3D_VERIFICATION.md: states this budgets the modeled uncertainty sources, not unmodeled physical error), and how to run the CLI.",
      "tests/test_uncertainty_budget_cli.py runs the CLI end-to-end on an existing synthetic/fixture paths3d case and checks the report file against the schema and a known-good budget."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 2,
    "task_class": "cli_tool",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-09.4",
  "title": "Uncertainty budget CLI + report (crackvision.uncertainty_budget)",
  "parent": "GEOM-09",
  "outcome": "A CLI reads a paths3d file (+ capture record) and writes a `crackvision.uncertainty_budget/1` report combining the analytic breakdown and Monte-Carlo cross-check for every point/waypoint."
}
```

Implement `crackvision/uncertainty_budget.py` as a CLI (module with `build_parser`/`main`, following the `path3d.py`/other GEOM-08 CLI shape and INTERFACES §0 flags/exit codes).

Input: a `crackvision.paths3d/1` file (§10) plus its referenced capture record (§9). For every lifted point and tool waypoint it contains, call `crackvision.uncertainty`'s analytic breakdown (GEOM-09.2) and, for a bounded subset (or all, if cheap enough) `crackvision.uncertainty_mc`'s cross-check (GEOM-09.3). Assemble and write a `crackvision.uncertainty_budget/1` report per §14 to `data/uncertainty_budget/{case}_budget.json`, and refuse (non-zero exit) if the MC-vs-analytic disagreement exceeds §14's tolerance anywhere.

Write `docs/geometry/UNCERTAINTY_BUDGET.md` describing the report, its command, and its honest scope (what uncertainty sources it does and does not model), matching the tone of `docs/geometry/PATH3D_VERIFICATION.md`. Add a CLI-smoke test on an existing fixture/synthetic paths3d case.

## Decomposed 2026-10-11T15:15:44Z by L-GEOM (claude-sonnet-5, high)
Children: GEOM-09.4.1, GEOM-09.4.2

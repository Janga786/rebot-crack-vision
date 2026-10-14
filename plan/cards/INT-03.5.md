# INT-03.5 — Assemble the safety case + unified failure-injection runner

```json card
{
  "kind": "integration",
  "depends_on": [
    "INT-03.1",
    "INT-03.2",
    "INT-03.3",
    "INT-03.4"
  ],
  "requirements": [
    "REQ-INT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "docs/integration/SAFETY_CASE.md",
      "scripts/safety/run_safety_case.sh"
    ]
  },
  "inputs": [
    "docs/integration/safety/EMPTY_MASK.md",
    "docs/integration/safety/INVALID_DEPTH.md",
    "docs/integration/safety/UNREACHABLE_PATH.md",
    "docs/integration/safety/STALE_CALIBRATION_AND_LIMITS.md",
    "docs/INTERFACES.md#0.2",
    "docs/INTERFACES.md#11",
    "scripts/perc/test_safety_empty_mask.sh",
    "scripts/perc/test_safety_invalid_depth.sh",
    "scripts/ros/test_safety_unreachable.sh",
    "scripts/ros/test_execution.sh"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "safety-case",
        "cmd": "bash scripts/ros/build_ws.sh && bash scripts/safety/run_safety_case.sh",
        "timeout_s": 1800,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "scripts/safety/run_safety_case.sh runs, in order, scripts/perc/test_safety_empty_mask.sh, scripts/perc/test_safety_invalid_depth.sh, scripts/ros/test_safety_unreachable.sh and scripts/ros/test_execution.sh, stops and reports clearly on the first failing stage, prints a one-line PASS/FAIL line per fault class, and exits non-zero if any stage failed; it never sets CRACKVISION_ARM_REAL and never touches real hardware.",
      "docs/integration/SAFETY_CASE.md has one table row per fault class (invalid depth, empty mask, unreachable path, stale calibration/config, limit violations): injection method, refusal mechanism (gate/check id), exit-code/outcome contract, and a link to the corresponding docs/integration/safety/*.md evidence file -- it links to those docs rather than duplicating their content.",
      "The doc explicitly traces each row to REQ-INT-2 and/or REQ-OPS-1, and explicitly states the scope boundary: this is a mock/dry, no-hardware safety case; REQ-INT-2's separately-gated hardware commissioning (verified limits, collision checks, e-stop under controlled conditions) is exercised by the operator cards (GEOM-05, GEOM-07, INT-04, INT-05), not here.",
      "The doc gives the single operator command (scripts/safety/run_safety_case.sh) to re-run the whole suite, consistent with REQ-OPS-1's clean-launch-to-execution workflow framing.",
      "Every cross-reference in the doc (gate id, evidence file, script) is checked by the reviewer to resolve to something real and accurate."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "integration-safety-assembly",
    "local_ok": false
  },
  "track": "simulation",
  "id": "INT-03.5",
  "title": "Assemble the safety case + unified failure-injection runner",
  "parent": "INT-03",
  "outcome": "docs/integration/SAFETY_CASE.md ties the five fault classes (invalid depth, empty mask, unreachable path, stale calibration/config, limit violations) to their refusal mechanisms and evidence, and scripts/safety/run_safety_case.sh runs the whole suite in one command."
}
```

Write scripts/safety/run_safety_case.sh as a thin orchestrator: run bash scripts/ros/build_ws.sh once if ros2_ws/install is missing, then run scripts/perc/test_safety_empty_mask.sh, scripts/perc/test_safety_invalid_depth.sh, scripts/ros/test_safety_unreachable.sh and scripts/ros/test_execution.sh in sequence, each under its own labelled echo block; stop immediately and report which stage failed if any script returns non-zero, otherwise print a final one-line PASS/FAIL summary table (one row per fault class) and exit 0. Never set CRACKVISION_ARM_REAL. Then write docs/integration/SAFETY_CASE.md: a table with one row per fault class (invalid depth, empty mask, unreachable path, stale calibration/config, limit violations) giving the injection method, the refusal mechanism (gate or check id), the exit-code/outcome contract, a link to the matching docs/integration/safety/*.md evidence file (do not duplicate their content, link to it), and the REQ-INT-2/REQ-OPS-1 traceability; an explicit statement that this safety case is mock/dry-only with no hardware, and that REQ-INT-2's hardware commissioning gate (verified limits, collision checks, e-stop, controlled conditions) is separately exercised by the operator cards (GEOM-05, GEOM-07, INT-04, INT-05); and the single command (bash scripts/safety/run_safety_case.sh) an operator runs to reproduce the whole suite.

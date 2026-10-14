# INT-03.4 — Stale-calibration-config and limit-violation safety evidence

```json card
{
  "kind": "impl",
  "requirements": [
    "REQ-INT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "docs/integration/safety/STALE_CALIBRATION_AND_LIMITS.md"
    ]
  },
  "inputs": [
    "scripts/ros/test_execution.sh",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py#_evaluate_stale_config",
    "docs/INTERFACES.md#11",
    "docs/motion/EXECUTION.md",
    "docs/motion/TIME_PARAMETERIZATION.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "stale-and-limits",
        "cmd": "bash scripts/ros/build_ws.sh && bash scripts/ros/test_execution.sh",
        "timeout_s": 1200,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "No new test code or library code is written by this card -- it re-runs the already-accepted scripts/ros/test_execution.sh (MOT-05.5) unmodified as its own evidence; this reuse, rather than new test code, is itself correct given that script already exercises G-STALE-CONFIG (phase 1, mismatched scene/end-effector config shas) and G-LIMITS (the overspeed fixture) through the real execute_trajectory CLI.",
      "docs/integration/safety/STALE_CALIBRATION_AND_LIMITS.md correctly explains what 'stale calibration' maps to in this system's contract (G-STALE-CONFIG: a trajectory's declared scene_config_sha256/end_effector_config_sha256, which encode the committed hand-eye/TCP calibration outputs, no longer matching the live config files) and quotes the exact per-mode outcome table from docs/INTERFACES.md #11.7 (warn in mock/dry, fail/refuse in real).",
      "The doc explains 'limit violation' as G-LIMITS' independent re-derivation from the limits file (not trusting the retimer's own self-check), and cites MOT-07.8's offline cross-validation (retimer vs independent checker vs G-LIMITS, with fault injection) as complementary multi-layer evidence.",
      "The doc records the exact command run for this card and its observed gate outcomes/exit code, distinct from merely restating MOT-05.6's runbook.",
      "Every citation (gate id, file, line/section) in the doc is checked by the reviewer to actually say what the doc claims."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "integration-safety-evidence",
    "local_ok": false
  },
  "track": "simulation",
  "id": "INT-03.4",
  "title": "Stale-calibration-config and limit-violation safety evidence",
  "parent": "INT-03",
  "outcome": "docs/integration/safety/STALE_CALIBRATION_AND_LIMITS.md documents, with a fresh passing run as evidence, how G-STALE-CONFIG and G-LIMITS already give safe refusals end-to-end through execute_trajectory, citing the existing MOT-05.5/MOT-07.8 mechanisms rather than re-implementing them."
}
```

Do not write any new test or library code. Run bash scripts/ros/build_ws.sh && bash scripts/ros/test_execution.sh (MOT-05.5's own headless mock verification, unmodified) and capture its output. Read ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py's _evaluate_stale_config and the G-LIMITS evaluation, plus docs/INTERFACES.md #11.7's gate outcome table, to ground the write-up precisely. Write docs/integration/safety/STALE_CALIBRATION_AND_LIMITS.md covering: (1) what 'stale calibration' means operationally here -- G-STALE-CONFIG compares a trajectory's declared scene_config_sha256/end_effector_config_sha256 (which encode the committed hand-eye/TCP calibration results produced by GEOM-05/GEOM-07) against the live config files at execution time, and quote the exact §11.7 per-mode outcome (warn in mock/dry, fail/refuse in real); (2) what 'limit violation' means -- G-LIMITS independently re-derives joint/velocity/acceleration limits from the limits file rather than trusting the retimer's own self-check, and cite MOT-07.8's offline fault-injection cross-check (retimer vs independent trajectory_check vs G-LIMITS) as the complementary multi-layer evidence that a limit violation cannot slip through a single layer; (3) this card's own fresh run: the exact command, and the observed gate outcomes/exit code from scripts/ros/test_execution.sh's phase 1 (stale-config fixture) and overspeed fixture. Keep every citation (file, section, gate id) accurate and checkable.

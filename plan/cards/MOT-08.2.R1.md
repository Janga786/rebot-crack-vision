# MOT-08.2.R1 — Repair MOT-08.2: Retime-report sha-mismatch test never reaches the sha-binding branch

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/preview_core.py",
      "tests/test_preview_core.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#18",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#16",
    "docs/INTERFACES.md#17",
    "src/crackvision/trajectory_check.py",
    "src/crackvision/kinematics.py",
    "config/robot/b601_dm_limits.yaml",
    "config/motion/execution.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_preview_core.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "independence",
        "cmd": "bash -c 'test -f src/crackvision/preview_core.py && ! grep -nE \"^\\s*(from|import)\\s+(crackvision_motion|rclpy|matplotlib)\" src/crackvision/preview_core.py'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "checker-regression",
        "cmd": "./env.sh pytest tests/test_trajectory_check.py tests/test_trajectory_check_path.py tests/test_trajectory_check_view.py -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The retime sha-mismatch test uses a schema-valid retime report (non-empty legs) whose trajectory_sha256 differs from the trajectory's sha. It asserts on the sha-specific message (e.g. match='trajectory_sha256 .* != current trajectory sha256'). The test fails if the retime sha comparison in bind_inputs is removed.",
      "All acceptance criteria of MOT-08.2 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "python-lib",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-08.2",
  "findings": [
    "d01fe44e64b7923e"
  ],
  "id": "MOT-08.2.R1",
  "title": "Repair MOT-08.2: Retime-report sha-mismatch test never reaches the sha-binding branch",
  "parent": "MOT-08",
  "outcome": "Resolve the review findings on MOT-08.2 while every acceptance criterion of MOT-08.2 still holds."
}
```

Repair work generated deterministically from review attempt `01181-MOT-08.2-review` of `MOT-08.2`.
Original card: `plan/cards/MOT-08.2.md` — its acceptance criteria must still hold.

## Finding 1 [major] Retime-report sha-mismatch test never reaches the sha-binding branch
- Evidence: tests/test_preview_core.py test_bind_inputs_refuses_retime_report_sha_mismatch: `retime_path = _write(tmp_path / "traj.json.retime.json", _retime_report("0" * 64, []))` with `pytest.raises(PreviewRefusal, match="retime report")`. _validate_retime_report_schema rejects empty legs first. Probe output: 'TEST-AS-WRITTEN: retime report does not parse as crackvision.retime_report/1: retime_report.legs must be a non-empty list'.
- Consequence: The acceptance criterion requires test coverage of a retime-report sha mismatch, and no test covers it. A regression that deleted or broke the retime trajectory_sha256 comparison in bind_inputs would still pass the whole suite.
- Affected: tests/test_preview_core.py
- Acceptance condition: The retime sha-mismatch test uses a schema-valid retime report (non-empty legs) whose trajectory_sha256 differs from the trajectory's sha. It asserts on the sha-specific message (e.g. match='trajectory_sha256 .* != current trajectory sha256'). The test fails if the retime sha comparison in bind_inputs is removed.

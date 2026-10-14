# MOT-08.4.R1 — Repair MOT-08.4: Malformed check report (JSON non-object or non-UTF-8) crashes with exi

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-OPS-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/approve_trajectory.py",
      "tests/test_approve_trajectory.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#18",
    "docs/INTERFACES.md#11",
    "docs/adr/016-commissioning-gated-execution.md",
    "src/crackvision/preview_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_approve_trajectory.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "no-bypass",
        "cmd": "bash -c '! grep -nE \"(--yes|--force|--non-interactive|sys\\.stdin|input\\()\" src/crackvision/approve_trajectory.py && grep -q \"/dev/tty\" src/crackvision/approve_trajectory.py'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "no-tty-refuses",
        "cmd": "bash -c 'setsid -w ./env.sh python -m crackvision.approve_trajectory --preview /nonexistent/preview.json --operator ci </dev/null; rc=$?; test $rc -eq 2 -o $rc -eq 3'",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "main() returns 3, with no file written and the refusal reason listed among the collected errors, when the check report (with the manifest sha re-synced) is (a) a JSON array, (b) bytes that are not valid UTF-8, or (c) a failing check entry with no 'check' key. A test covers at least (a) and (b).",
      "All acceptance criteria of MOT-08.4 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "python-cli-safety",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 91,
  "repairs": "MOT-08.4",
  "findings": [
    "2c70fa9564b51b70"
  ],
  "id": "MOT-08.4.R1",
  "title": "Repair MOT-08.4: Malformed check report (JSON non-object or non-UTF-8) crashes with exi",
  "parent": "MOT-08",
  "outcome": "Resolve the review findings on MOT-08.4 while every acceptance criterion of MOT-08.4 still holds."
}
```

Repair work generated deterministically from review attempt `01209-MOT-08.4-review` of `MOT-08.4`.
Original card: `plan/cards/MOT-08.4.md` — its acceptance criteria must still hold.

## Finding 1 [minor] Malformed check report (JSON non-object or non-UTF-8) crashes with exit 1 instead of the §18.2 check-4 exit-3 refusal
- Evidence: src/crackvision/approve_trajectory.py:113 `check_text = check_path.read_text(encoding="utf-8")` only catches OSError, and :129 `check_doc.get("trajectory_sha256")` assumes the report is a dict. In a probe built on the test's `case` fixture, writing the check report as `[]` (with the manifest's check_report.sha256 re-synced) gave `AttributeError: 'list' object has no attribute 'get'` at approve_trajectory.py:129. Writing it as bytes b"\xff\xfe" gave `UnicodeDecodeError` from read_text at :113. main() catches only PreviewRefusal and PreviewUsageError (:299), so the CLI exits 1 with a traceback. The same pattern applies at :135-140: a failing check entry with no 'check' key puts None into failed, and ', '.join(failed) raises TypeError.
- Consequence: The card's Outcome says any outcome other than success is a refusal with exit 3, and §18.2 check 4 lists an unreadable check report, or one that does not name the trajectory sha, as an exit-3 refusal. Instead the tool exits 1 with a traceback, and every other failing reason that would have been collected is lost. No approval file is written, so this does not open a bypass.
- Affected: src/crackvision/approve_trajectory.py, tests/test_approve_trajectory.py
- Acceptance condition: main() returns 3, with no file written and the refusal reason listed among the collected errors, when the check report (with the manifest sha re-synced) is (a) a JSON array, (b) bytes that are not valid UTF-8, or (c) a failing check entry with no 'check' key. A test covers at least (a) and (b).

# MOT-08.1.R1 — Repair MOT-08.1: ADR-020 contradicts §18.2 on whether approve_trajectory refuses a fail

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2",
    "REQ-OPS-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "docs/INTERFACES.md",
      "docs/adr/020-trajectory-preview-approval.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#16",
    "docs/INTERFACES.md#17",
    "docs/adr/016-commissioning-gated-execution.md",
    "docs/adr/019-time-parameterization.md",
    "docs/motion/EXECUTION.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
    "src/crackvision/trajectory_check.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "adr",
        "cmd": "test -s docs/adr/020-trajectory-preview-approval.md && grep -q 'Status:' docs/adr/020-trajectory-preview-approval.md",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "section",
        "cmd": "bash -c 'grep -qE \"^## 18\\. \" docs/INTERFACES.md && grep -q \"crackvision.trajectory_preview/1\" docs/INTERFACES.md && grep -q \"APPROVE <\" docs/INTERFACES.md && grep -nE \"^## (17|18)\\. \" docs/INTERFACES.md | awk -F: \"NR==1&&\\$2~/^## 17/{a=\\$1} NR==2&&\\$2~/^## 18/{b=\\$1} END{exit !(a&&b&&b>a)}\"'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The ADR's residual-risk paragraph no longer claims approve_trajectory accepts a manifest or check report with failing checks. It agrees with §18.2 check 4, which refuses any `fail` outcome, and it no longer mentions a non-existent `warn` check outcome.",
      "§18.2 specifies that the manifest entry's path is normalised by the §11.2 convention: the resolved --preview file, written repo-relative (POSIX) when it lies under --root and absolute otherwise. This makes it consistent with how §18.3 resolves the path.",
      "All acceptance criteria of MOT-08.1 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 4,
    "consequence": 5,
    "task_class": "motion-contract",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-08.1",
  "findings": [
    "67f4ac73eb7540af",
    "9283117e7ccf3d05"
  ],
  "id": "MOT-08.1.R1",
  "title": "Repair MOT-08.1: ADR-020 contradicts §18.2 on whether approve_trajectory refuses a fail",
  "parent": "MOT-08",
  "outcome": "Resolve the review findings on MOT-08.1 while every acceptance criterion of MOT-08.1 still holds."
}
```

Repair work generated deterministically from review attempt `01113-MOT-08.1-review` of `MOT-08.1`.
Original card: `plan/cards/MOT-08.1.md` — its acceptance criteria must still hold.

## Finding 1 [major] ADR-020 contradicts §18.2 on whether approve_trajectory refuses a failing check report
- Evidence: docs/adr/020-trajectory-preview-approval.md:119-125: "a determined operator can still type `APPROVE <sha8>` at a manifest whose `check_report.all_pass` is `false` as long as every individual `approve_trajectory` precondition (§18) happens to pass — this card does not add a check-report-must-pass precondition to `approve_trajectory` itself ... by design (an operator may have a documented reason to approve a `warn`-only failing run); only `G-APPROVAL`'s own `check_report.all_pass == true` requirement (§18) is the hard backstop, and it applies at execution time, not at approval time." But docs/INTERFACES.md:3197 (§18.2 check 4) refuses with exit 3 when 'any one of its checks has outcome `fail`', and the card's decision 4 requires this. Also, §16.5 check reports have no `warn` outcome.
- Consequence: §18 names ADR-020 as its normative source, and MOT-08.3 and later implement 'verbatim'. The two documents disagree on a safety-relevant refusal. An implementer or reviewer who follows the ADR could drop check 4 and allow approval of failing trajectories at approval time.
- Affected: docs/adr/020-trajectory-preview-approval.md
- Acceptance condition: The ADR's residual-risk paragraph no longer claims approve_trajectory accepts a manifest or check report with failing checks. It agrees with §18.2 check 4, which refuses any `fail` outcome, and it no longer mentions a non-existent `warn` check outcome.

## Finding 2 [major] Manifest entry in preview_artifacts uses the raw --preview string, not a repo-relative path
- Evidence: docs/INTERFACES.md:3223: "the manifest itself (`{path: --preview's own path, sha256: ...}`)". The card's decision 4 says: "`preview_artifacts` is the manifest itself followed by every manifest artefact, each as a repo-relative path with its sha256." §18.3 check 2 resolves each entry's path 'against `--root`'.
- Consequence: Suppose the operator runs approve_trajectory with --preview ./preview.json from inside the preview directory, or with a working directory that is not the repo root. The recorded path is then relative to that working directory. At execution time G-APPROVAL resolves it against --root, finds no file, and refuses in real. The result also differs from the §11.2/§18.1 path convention used for every other path field.
- Affected: docs/INTERFACES.md
- Acceptance condition: §18.2 specifies that the manifest entry's path is normalised by the §11.2 convention: the resolved --preview file, written repo-relative (POSIX) when it lies under --root and absolute otherwise. This makes it consistent with how §18.3 resolves the path.

# MOT-08.9 — Document the preview → approval → execution workflow (PREVIEW_APPROVAL.md, EXECUTION.md, ROS_WORKSPACE.md)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-08.8"
  ],
  "requirements": [
    "REQ-OPS-1",
    "REQ-OPS-2"
  ],
  "scope": {
    "write": [
      "docs/motion/PREVIEW_APPROVAL.md",
      "docs/motion/EXECUTION.md",
      "docs/motion/ROS_WORKSPACE.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#18",
    "docs/adr/020-trajectory-preview-approval.md",
    "scripts/ros/test_preview_approve.sh",
    "scripts/ros/test_preview_rviz.sh",
    "docs/motion/EXECUTION.md",
    "docs/motion/TIME_PARAMETERIZATION.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "docs",
        "cmd": "bash -c 'f=docs/motion/PREVIEW_APPROVAL.md; test -s $f && grep -q \"crackvision.trajectory_preview\" $f && grep -q \"approve_trajectory\" $f && grep -q \"APPROVE \" $f && grep -q \"preview_rviz.launch.py\" $f && grep -q \"PREVIEW_APPROVAL.md\" docs/motion/EXECUTION.md && grep -q \"preview_trajectory\" docs/motion/ROS_WORKSPACE.md'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Every command in PREVIEW_APPROVAL.md matches the actual CLIs (flags, defaults, exit codes) and the sequence exercised by test_preview_approve.sh.",
      "The doc says what the operator must look at in each PNG before approving (limit headroom, tool path against scene and crack, tool speed at the real speed scale, CHECK FAILED banner). It also says what an approval does and does not certify, and that it expires after approval_max_age_s and is invalidated by any change to the trajectory or preview artefacts.",
      "EXECUTION.md's G-APPROVAL row and approval bullet describe the §18 preview binding. No other gate row changes.",
      "The doc states plainly that real execution remains gated by commissioning (MOT-09), measured scene/end-effector configs and G-CONFIRM, and that approval alone never moves the arm."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 3,
    "task_class": "docs",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-08.9",
  "title": "Document the preview → approval → execution workflow (PREVIEW_APPROVAL.md, EXECUTION.md, ROS_WORKSPACE.md)",
  "parent": "MOT-08",
  "outcome": "docs/motion/PREVIEW_APPROVAL.md gives the operator the exact commands to preview, inspect (PNGs and optional RViz), approve and then dry-run a trajectory. EXECUTION.md's G-APPROVAL row and approval bullet reflect §18, and ROS_WORKSPACE.md lists the new node and launch file."
}
```

Write docs/motion/PREVIEW_APPROVAL.md with these sections:
- Purpose, with links to ADR-020 and §18.
- Inputs (trajectory, plan, .retime.json, .check.json).
- Step 1: trajectory_check.
- Step 2: trajectory_preview, with the output directory layout.
- Step 3: inspecting the PNGs, with a checklist.
- Step 3b (optional): RViz via preview_rviz.launch.py on a machine with a display. Note that it uses the mock planning stack and that the RViz config has no execute button.
- Step 4: approve_trajectory, covering the phrase, refusals and the default output path.
- Step 5: execute_trajectory --mode dry --approval, then the gated real path per EXECUTION.md.
- Troubleshooting: each refusal message mapped to its remedy.
- Verification: test_preview_approve.sh and test_preview_rviz.sh.

Update the G-APPROVAL row and the §3 approval bullet in docs/motion/EXECUTION.md, linking to the new doc. Add the preview_trajectory console script and preview_rviz.launch.py to docs/motion/ROS_WORKSPACE.md's package listing. Take every command from the scripts as they were accepted; do not invent flags.

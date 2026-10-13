# MOT-05.6.R1 — Repair MOT-05.6: G-ELIGIBLE row in EXECUTION.md is stale for purpose:"view_move" trajec

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "docs/motion/EXECUTION.md",
      "docs/motion/ROS_WORKSPACE.md"
    ]
  },
  "inputs": [
    "docs/adr/016-commissioning-gated-execution.md",
    "docs/INTERFACES.md#11",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execute_trajectory.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
    "config/motion/execution.yaml",
    "config/robot/commissioning.yaml",
    "scripts/ros/test_execution.sh",
    "docs/motion/ROS_WORKSPACE.md",
    "docs/motion/VENDOR_DRIVER.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "doc-present",
        "cmd": "bash -c 'f=docs/motion/EXECUTION.md; test -f $f && for s in \"--mode dry\" \"--mode real\" CRACKVISION_ARM_REAL commissioning.yaml execution_approval \"e-stop\" test_execution.sh; do grep -qF -- \"$s\" $f || { echo \"missing $s\"; exit 1; }; done; grep -qF EXECUTION.md docs/motion/ROS_WORKSPACE.md'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "gate-ids-documented",
        "cmd": "bash scripts/ros/env_ros.sh python3 -c \"import sys,re; sys.path.insert(0,'ros2_ws/src/crackvision_motion'); sys.path.insert(0,'ros2_ws/src/crackvision_description'); import crackvision_motion.execution_gate as g; ids=[v for k,v in vars(g).items() if k.startswith('G_') and isinstance(v,str)]; doc=open('docs/motion/EXECUTION.md').read(); miss=[i for i in ids if i not in doc]; print('missing', miss); sys.exit(1 if (miss or not ids) else 0)\"",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "EXECUTION.md's gate catalogue documents the view_move-specific G-ELIGIBLE checks, remedy and satisfier distinctly from the crack_task/paths3d case.",
      "All acceptance criteria of MOT-05.6 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "technical-docs",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-05.6",
  "findings": [
    "dac972af5c843d0a"
  ],
  "id": "MOT-05.6.R1",
  "title": "Repair MOT-05.6: G-ELIGIBLE row in EXECUTION.md is stale for purpose:\"view_move\" trajec",
  "parent": "MOT-05",
  "outcome": "Resolve the review findings on MOT-05.6 while every acceptance criterion of MOT-05.6 still holds."
}
```

Repair work generated deterministically from review attempt `01057-MOT-05.6-rereview` of `MOT-05.6`.
Original card: `plan/cards/MOT-05.6.md` — its acceptance criteria must still hold.

## Finding 1 [major] G-ELIGIBLE row in EXECUTION.md is stale for purpose:"view_move" trajectories
- Evidence: docs/motion/EXECUTION.md:166 vs. ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py:672-773 (added in cbdd286, MOT-07.10.2) and docs/INTERFACES.md §17.2.
- Consequence: An operator debugging a view_move G-ELIGIBLE failure is pointed at the wrong remedy and wrong owning card.
- Affected: docs/motion/EXECUTION.md
- Acceptance condition: EXECUTION.md's gate catalogue documents the view_move-specific G-ELIGIBLE checks, remedy and satisfier distinctly from the crack_task/paths3d case.

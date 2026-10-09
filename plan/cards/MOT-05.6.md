# MOT-05.6 — Operator execution runbook (mock → dry → real) + ROS_WORKSPACE.md entry

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-05.5"
  ],
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
    "docs/motion/ROS_WORKSPACE.md"
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
      "Every gate id has a row: what it checks, which mode it applies to, which artefact or card satisfies it (e.g. G-EE ← GEOM-05/GEOM-07, G-SCENE ← MOT-10, G-COMMISSIONING/G-ESTOP ← MOT-09, G-APPROVAL ← MOT-08, G-ELIGIBLE ← GEOM-08 §10.6) and the exact remedy.",
      "It states plainly that, as shipped, real mode is refused, and why (nominal scene, nominal end-effector, uncommissioned record). It does not claim any hardware readiness.",
      "The real-driver bring-up and enable steps are the operator's own documented commands, matching ADR-016. The executor's non-use of vendor motion services is explained.",
      "The e-stop section puts the hardware e-stop first and gives the software topics as secondary, with the exact ros2 topic pub commands and the post-cancel behaviour from ADR-016.",
      "The recovery section covers each exit code / outcome (refused, start mismatch, collision, e-stop, tracking abort, driver loss) and gives the next safe action.",
      "ROS_WORKSPACE.md gains a short section only; existing content is unchanged."
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
  "id": "MOT-05.6",
  "parent": "MOT-05",
  "title": "Operator execution runbook (mock → dry → real) + ROS_WORKSPACE.md entry",
  "outcome": "docs/motion/EXECUTION.md is the operator-facing procedure for the commissioning-gated executor. It covers prerequisites per gate id, how to produce each record (commissioning, approval), the mock→dry→real progression, the typed confirmation, e-stop use (hardware first), reading the execution record and failure recovery. docs/motion/ROS_WORKSPACE.md points to it and to test_execution.sh. MOT-09 and OPS-01 can use it without reading code."
}
```

Write for the operator who will run MOT-09 and later INT-04/INT-05. Derive every statement from ADR-016, §11 and the implemented code and config; do not invent behaviour. Cite exact commands (scripts/ros/env_ros.sh, sourcing the overlay, ros2 run crackvision_motion execute_trajectory …), file paths and record locations (logs/execution/). Include the GEOM-05/GEOM-07/MOT-09 ordering note from ADR-016 verbatim or by reference, so the operator knows that calibration motion does not go through this executor unless the technical lead approves an exception. Append to ROS_WORKSPACE.md; do not rewrite it.

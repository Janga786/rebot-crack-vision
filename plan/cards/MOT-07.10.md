# MOT-07.10 — Retime the view-plan transit leg and settle the §11.2 purpose for view moves

```json card
{
  "kind": "branch",
  "depends_on": [
    "MOT-07.5",
    "MOT-07.7"
  ],
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1"
  ],
  "refine_after": [
    "MOT-07.1",
    "MOT-07.9"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/retime_plan.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/retime.py",
      "ros2_ws/src/crackvision_motion/test/test_retime_view.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#16",
    "docs/adr/019-time-parameterization.md"
  ],
  "acceptance": {
    "checks": [],
    "criteria": [
      "Refined after MOT-07.1 decides §16.8: either a new purpose value with explicit G-TRAJ/G-ELIGIBLE rules appended as a contract amendment, or a documented deferral to OPS-01."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 4,
    "context": 3,
    "consequence": 5,
    "task_class": "motion-contract",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.10",
  "title": "Retime the view-plan transit leg and settle the §11.2 purpose for view moves",
  "parent": "MOT-07",
  "outcome": "The view_plan transit leg (home → camera view pose) can be retimed into a gate-acceptable joint_trajectory per the resolution ADR-019 §16.8 records, with any §11.2/§11.7 purpose amendment implemented consistently in joint_trajectory, execution_gate, retime_plan and the checker."
}
```

Draft. §13.14's `view_plan.transit_leg` must be executable for the operator workflow (capture pose). Under the current §11.2, a view move is neither `crack_task` (which requires `source.paths3d`) nor executable as `test` in real. Refine this card once ADR-019 §16.8 has recorded its decision.

The likely scope:
- `retime_plan --view-plan`, reusing `retime_core` (no trace legs);
- if amended, the purpose value in `joint_trajectory.load_trajectory` / `execution_gate` G-TRAJ/G-ELIGIBLE, and the checker;
- tests, and a mock run.

If the change touches execution_gate (MOT-05 accepted code), split it into its own card with the gate regression suite (`scripts/ros/test_execution.sh`) as acceptance.

## Decomposed 2026-10-13T12:15:23Z by L-MOTION (claude-opus-5-5, high)
Children: MOT-07.10.1, MOT-07.10.2, MOT-07.10.3, MOT-07.10.4, MOT-07.10.5

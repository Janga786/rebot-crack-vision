# GEOM-08.2.R1 — Repair GEOM-08.2: Measured uncertainty without `source` is accepted and a placeholder so

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-GEOM-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/kinematics.py",
      "tests/test_kinematics.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#6.2",
    "docs/INTERFACES.md#8",
    "docs/INTERFACES.md#9",
    "docs/INTERFACES.md#10",
    "docs/motion/ROBOT_MODEL.md",
    "presentation/sim/reBot_B601_DM_with_gripper.urdf",
    "config/robot/end_effector.yaml",
    "config/robot/b601_dm_limits.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_kinematics.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "no-ros-import",
        "cmd": "! grep -nE '^\\s*(import|from) (rclpy|moveit|tf2|geometry_msgs)' src/crackvision/kinematics.py",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "load_end_effector raises KinematicsError when a measured block's uncertainty lacks `source` (or it is empty). A test in tests/test_kinematics.py covers this for both the tool and wrist_camera blocks, and the existing tests still pass.",
      "All acceptance criteria of GEOM-08.2 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 2,
    "consequence": 4,
    "task_class": "impl-library",
    "local_ok": false
  },
  "track": "calibration",
  "priority": 95,
  "repairs": "GEOM-08.2",
  "findings": [
    "471771ec3294da5c"
  ],
  "id": "GEOM-08.2.R1",
  "title": "Repair GEOM-08.2: Measured uncertainty without `source` is accepted and a placeholder so",
  "parent": "GEOM-08",
  "outcome": "Resolve the review findings on GEOM-08.2 while every acceptance criterion of GEOM-08.2 still holds."
}
```

Repair work generated deterministically from review attempt `00176-GEOM-08.2-review` of `GEOM-08.2`.
Original card: `plan/cards/GEOM-08.2.md` — its acceptance criteria must still hold.

## Finding 1 [major] Measured uncertainty without `source` is accepted and a placeholder source is substituted
- Evidence: src/crackvision/kinematics.py _load_block_sigma: `source = uncertainty.get("source", "measured")`. Reproduced with a tmp yaml whose tool and wrist_camera blocks are both measured, with uncertainty {position_sigma_m[, rotation_sigma_rad]} and no source. load_end_effector returned sigma_source {'wrist_camera': 'measured', 'tool': 'measured'} and raised nothing. The card requires `uncertainty: {position_sigma_m, rotation_sigma_rad, source} ... else KinematicsError`. INTERFACES §10.4 says these values are sourced from PLAN.md residuals and are 'never a placeholder'.
- Consequence: A GEOM-05/GEOM-07 output that has no provenance for its sigma is accepted. Downstream §10 records would then carry a fabricated `source` string, which defeats the rule that a measured value must be traceable to its residual estimate.
- Affected: src/crackvision/kinematics.py, tests/test_kinematics.py
- Acceptance condition: load_end_effector raises KinematicsError when a measured block's uncertainty lacks `source` (or it is empty). A test in tests/test_kinematics.py covers this for both the tool and wrist_camera blocks, and the existing tests still pass.

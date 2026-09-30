# GEOM-11 — TCP/pivot calibration uses the tool_tip prior (ADR-014), not the grasp-centre gripper_tcp

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-06",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-GEOM-3"
  ],
  "scope": {
    "write": [
      "src/crackvision/calibration/tcp.py",
      "tests/test_tcp.py",
      "docs/calibration/TCP_BORESIGHT_PROCEDURE.md"
    ]
  },
  "inputs": [
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "docs/INTERFACES.md#8",
    "config/robot/end_effector.yaml",
    "docs/adr/012-frames-and-conventions.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "pytest",
        "cmd": "./env.sh pytest tests/test_tcp.py -q",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The prior tool-tip position is read from config/robot/end_effector.yaml (tool.xyz_m and its value_status/provenance) — no hard-coded grasp-centre prior remains as the default; gripper_tcp's -0.0443 m is documented only as the grasp centre.",
      "The comparison is a point comparison: position delta |measured - prior| (mm) is the primary number; the direction angle is reported only when both vectors are longer than a stated minimum (a zero-length prior never yields NaN-driven flags); thresholds live in the procedure, not the function.",
      "tests cover: closed-gripper calibration near (0,0,0) is consistent with the nominal prior; a held probe (e.g. +40 mm along +X) is reported as a positive delta, not an 'opposite direction' error; the old grasp-centre value would have been flagged 44 mm off (regression explanation); pivot solver tests unchanged.",
      "TCP_BORESIGHT_PROCEDURE.md names the tool actually calibrated (closed gripper tip or held probe/pen), says the solve measures T_gripper_link_tool_tip (tool0 == gripper_link), and says GEOM-07 writes the result into the end_effector.yaml tool block with value_status measured and its evidence; the §4 near/far parallax check targets tool_tip."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 3
  },
  "track": "calibration",
  "priority": 55,
  "id": "GEOM-11",
  "title": "TCP/pivot calibration uses the tool_tip prior (ADR-014), not the grasp-centre gripper_tcp",
  "parent": "L-GEOM",
  "outcome": "The pivot-calibration comparison and the TCP/boresight procedure refer to the physical tool tip (config/robot/end_effector.yaml tool block, nominal (0,0,0) = closed-finger tip on gripper_link) instead of gripper_tcp's -0.0443 m grasp-centre offset, so a correct GEOM-07 calibration is not flagged as a 44 mm disagreement."
}
```

Created by the technical-lead recovery session (2026-09-30). ADR-014 §5: GEOM-06's
PRIOR_TCP_OFFSET_M = (-0.0443, 0, 0) is the vendor MoveIt grasp centre (gripper_tcp), 44.3 mm proximal to the
fingertips, but a pivot calibration measures the physical tip that touches the fixture - for the closed gripper
that is gripper_link's origin (canonical URDF finger meshes reach x = 0.0000 at y = 0). check_boresight's angle
between offset directions is also undefined for the (0,0,0) prior. Fix the prior source and the comparison;
keep the pivot solver itself (accepted in GEOM-06) unchanged.

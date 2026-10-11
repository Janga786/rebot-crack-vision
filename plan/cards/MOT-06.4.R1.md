# MOT-06.4.R1 — Repair MOT-06.4: finalize does not copy execution_eligible from the paths3d file into s

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
      "ros2_ws/src/crackvision_motion/test/test_cartesian_plan.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#11",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
    "config/robot/b601_dm_limits.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_cartesian_plan.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "finalize sets plan['sources']['paths3d']['execution_eligible'] from paths3d_doc['execution_eligible'], or raises CartesianPlanError when they differ. The caveat is derived from that value. A test shows that a paths3d_doc with execution_eligible False yields sources False plus the ineligible caveat, or raises on a mismatch.",
      "Both add_segment_report and validate_plan raise CartesianPlanError for: failed with chosen != null, failed with empty reasons, and full/partial with chosen == null. Tests cover each case.",
      "All acceptance criteria of MOT-06.4 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "python-core",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-06.4",
  "findings": [
    "32b2daf791d7e22c",
    "3a2f921ba5f330ee"
  ],
  "id": "MOT-06.4.R1",
  "title": "Repair MOT-06.4: finalize does not copy execution_eligible from the paths3d file into s",
  "parent": "MOT-06",
  "outcome": "Resolve the review findings on MOT-06.4 while every acceptance criterion of MOT-06.4 still holds."
}
```

Repair work generated deterministically from review attempt `00514-MOT-06.4-review` of `MOT-06.4`.
Original card: `plan/cards/MOT-06.4.md` — its acceptance criteria must still hold.

## Finding 1 [major] finalize does not copy execution_eligible from the paths3d file into sources.paths3d
- Evidence: cartesian_plan.py finalize: `execution_eligible = plan.get("sources", {}).get("paths3d", {}).get("execution_eligible")` reads the caller-supplied value, and paths3d_doc is used only for `counts`. Repro: paths3d_doc={'counts':..., 'execution_eligible': False}, plan sources True → finalize succeeds and prints 'sources eligible: True ineligible caveat: False'. No test covers copying from the paths3d document.
- Consequence: A plan built from an ineligible paths3d file can claim it is eligible and leave out the §13.13 ineligible caveat. This breaks criterion 4 and misleads MOT-07 readers; G-ELIGIBLE re-reads the file itself, but the plan's own provenance is wrong.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py, ros2_ws/src/crackvision_motion/test/test_cartesian_plan.py
- Acceptance condition: finalize sets plan['sources']['paths3d']['execution_eligible'] from paths3d_doc['execution_eligible'], or raises CartesianPlanError when they differ. The caveat is derived from that value. A test shows that a paths3d_doc with execution_eligible False yields sources False plus the ineligible caveat, or raises on a mismatch.

## Finding 2 [major] Segment report status invariants for chosen/reasons from §13.11.3 are not enforced
- Evidence: add_segment_report(plan, _seg_ref(), [], 5, {'direction':'forward','roll_offset_rad':0.0}, 10, 0, 1.0, 0.0) returned status 'failed', reasons [], with chosen set, and finalize/validate_plan accepted it. add_segment_report(..., 1, None, 10, 10, 1.0, 1.0) (full with chosen=None) was also accepted. In _validate_segment_report, the 'failed' branch checks only tw_planned and candidates_tried, and the full/partial branches never check chosen.
- Consequence: Invalid documents pass the strict validator and reach MOT-07: a failed segment that names a chosen candidate, a failed segment with no reasons, or a planned segment with no chosen candidate. This violates §13.11.3 and the criterion's 'failed (with reasons)'.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py, ros2_ws/src/crackvision_motion/test/test_cartesian_plan.py
- Acceptance condition: Both add_segment_report and validate_plan raise CartesianPlanError for: failed with chosen != null, failed with empty reasons, and full/partial with chosen == null. Tests cover each case.

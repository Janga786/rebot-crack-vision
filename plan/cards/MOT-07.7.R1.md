# MOT-07.7.R1 — Repair MOT-07.7: Projection breaks ties at a revisited plan configuration by taking the

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/trajectory_check.py",
      "tests/test_trajectory_check_path.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#16",
    "docs/adr/019-time-parameterization.md",
    "src/crackvision/trajectory_check.py",
    "src/crackvision/kinematics.py",
    "config/robot/end_effector.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_trajectory_check_path.py tests/test_trajectory_check.py -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "independent",
        "cmd": "test -f src/crackvision/trajectory_check.py && ! grep -nE '^\\s*(from|import)\\s+(crackvision_motion|rclpy)' src/crackvision/trajectory_check.py",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "kinematics-regression",
        "cmd": "./env.sh pytest tests/test_kinematics.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "When distances tie (or are within tolerance), the projection prefers the segment that continues progress, e.g. the smallest progress ≥ the previous point's. The leg assignment cannot give one point to non-adjacent legs. A new test builds a plan whose last leg returns to an earlier vertex (e.g. transit HOME→A, approach A→B, trace B→C, retract C→A) with a correct trajectory and report: C-PATH, C-TOOLSPEED and C-REPORT pass, and the report point_ranges match exactly.",
      "A report leg with a missing or non-list point_range (or a non-dict tool_speed) gives C-REPORT outcome 'fail' with a detail message, and the CLI exits 3, not 1. Covered by a test.",
      "Each trace-leg entry in C-TOOLSPEED numbers contains L, v_0, v_max and F, asserted in a test.",
      "All acceptance criteria of MOT-07.7 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "verification",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 99,
  "repairs": "MOT-07.7",
  "findings": [
    "4c399faa89247366",
    "b8bb47b64f7d536f",
    "27cd9b537f7057ed"
  ],
  "id": "MOT-07.7.R1",
  "title": "Repair MOT-07.7: Projection breaks ties at a revisited plan configuration by taking the",
  "parent": "MOT-07",
  "outcome": "Resolve the review findings on MOT-07.7 while every acceptance criterion of MOT-07.7 still holds."
}
```

Repair work generated deterministically from review attempt `00890-MOT-07.7-review` of `MOT-07.7`.
Original card: `plan/cards/MOT-07.7.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] Projection breaks ties at a revisited plan configuration by taking the earliest segment, so correct retime_plan output fails C-PATH and C-REPORT
- Evidence: src/crackvision/trajectory_check.py project_polyline/eval_range: `if best is None or d < best[0]: best = (d, k, s)`. This keeps the first segment at minimal distance, with no preference for continuing forward from the cursor, and the window (cursor±4) covers earlier segments. Reproduced with the MOT-07.5 fixture plan (ros2_ws/src/crackvision_motion/test/test_retime_plan_cli.py::_build_plan; retract ends at Q_APPROACH_START = P[1]), retimed by the real retime_plan. `python -m crackvision.trajectory_check ... --report traj.json.retime.json` gives exit=3. Output: C-PATH fail 'trajectory point 25 polyline progress (1.161895004) regresses from point 24's (1.490686852)' and C-REPORT fail 'leg 0 report point_range [0, 18] != recomputed [0, 25] ... leg 3 report point_range [22, 25] != recomputed [22, 24]'. Probe: point 25 == P[-1] is True, but it projects to {'segment': 0, 's': 1.0, 'distance': 0.0}. _leg_point_ranges assigns legs by progress intervals, so the regressed point also lands in legs 0 and 1.
- Consequence: Correct trajectories from the production retimer are rejected whenever the plan polyline returns to or passes near an earlier configuration, which the upstream fixture's retract-to-approach-start leg does. Leg membership for C-TOOLSPEED and C-REPORT is then wrong as well. Downstream execution gating on this checker would block valid trajectories.
- Affected: src/crackvision/trajectory_check.py, tests/test_trajectory_check_path.py
- Acceptance condition: When distances tie (or are within tolerance), the projection prefers the segment that continues progress, e.g. the smallest progress ≥ the previous point's. The leg assignment cannot give one point to non-adjacent legs. A new test builds a plan whose last leg returns to an earlier vertex (e.g. transit HOME→A, approach A→B, trace B→C, retract C→A) with a correct trajectory and report: C-PATH, C-TOOLSPEED and C-REPORT pass, and the report point_ranges match exactly.

## Finding 2 [minor] Malformed retime report crashes the checker (exit 1) instead of failing C-REPORT
- Evidence: check_report: `rep_range = report_leg.get("point_range"); if list(rep_range) != [first, last]`. With a report whose legs[0] has no point_range, the CLI logs 'unexpected error while running checks / TypeError: NoneType object is not iterable' and exits 1.
- Consequence: A report that is missing or has wrong-typed declared fields is reported as a checker runtime error rather than a C-REPORT fail, and the other checks' results are not written.
- Affected: src/crackvision/trajectory_check.py
- Acceptance condition: A report leg with a missing or non-list point_range (or a non-dict tool_speed) gives C-REPORT outcome 'fail' with a detail message, and the CLI exits 3, not 1. Covered by a test.

## Finding 3 [minor] C-TOOLSPEED numbers omit L, v_0 and v_max per trace leg
- Evidence: §16.5: '`numbers` for C-TOOLSPEED records, per trace leg, `L`, `v_0`, `v_max` and `F`'. check_toolspeed's leg_numbers holds only point_range, speeds, min_speed, max_speed and F.
- Consequence: The report lacks evidence values the normative contract requires, so a reader cannot see why F is empty (rule-1 bound against v_max).
- Affected: src/crackvision/trajectory_check.py
- Acceptance condition: Each trace-leg entry in C-TOOLSPEED numbers contains L, v_0, v_max and F, asserted in a test.

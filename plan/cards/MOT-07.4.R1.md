# MOT-07.4.R1 — Repair MOT-07.4: validate_retime_report is not strict for parameters / created_utc / po

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/retime.py",
      "ros2_ws/src/crackvision_motion/test/test_retime.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#16",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/adr/019-time-parameterization.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/retime_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/tool_fk.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
    "config/motion/retime.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_retime.py -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "validate_retime_report rejects all of the following, each covered by a test: a parameters block that is not a valid crackvision.retime_config/1 echo (wrong schema, missing or extra nested key, out-of-range value); a non-ISO-8601 created_utc; and a point_range whose last index is ≥ totals.n_points.",
      "Tests assert that, for every leg, trajectory points[first] and points[last] equal that leg's first and last plan configuration (shared-boundary convention honoured). They also assert that peak_velocity_fraction and peak_acceleration_fraction equal values recomputed independently from the trajectory and limits, and that ramp_in_m/ramp_out_m equal the arc lengths of the leading/trailing below-band segments recomputed with tool_fk.",
      "All acceptance criteria of MOT-07.4 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 5,
    "task_class": "motion-algorithm",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-07.4",
  "findings": [
    "fa4ecd83f836fb00",
    "bffb7eadf63eb4ce"
  ],
  "id": "MOT-07.4.R1",
  "title": "Repair MOT-07.4: validate_retime_report is not strict for parameters / created_utc / po",
  "parent": "MOT-07",
  "outcome": "Resolve the review findings on MOT-07.4 while every acceptance criterion of MOT-07.4 still holds."
}
```

Repair work generated deterministically from review attempt `00859-MOT-07.4-review` of `MOT-07.4`.
Original card: `plan/cards/MOT-07.4.md` — its acceptance criteria must still hold.

## Finding 1 [major] validate_retime_report is not strict for parameters / created_utc / point_range bounds
- Evidence: retime.py validate_retime_report only checks `set(parameters) != _PARAMETER_KEYS` plus position_margin_rad; nested values are never validated. Probe on a valid report: parameters.trace='garbage' ACCEPTED; parameters.schema='wrong/9' ACCEPTED; parameters.limit_fraction=None ACCEPTED; legs[-1].point_range=[0,99999] with n_points=~27 ACCEPTED; created_utc='hello' ACCEPTED (§16.4 requires ISO-8601 UTC).
- Consequence: The criterion requires the report to validate against §16.4 with a strict loader. Under §16.4, parameters echo every retime.yaml field (§16.1 ranges). A malformed or tampered report passes this module's loader, and downstream consumers relying on it get no shape guarantee for parameters.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/retime.py, ros2_ws/src/crackvision_motion/test/test_retime.py
- Acceptance condition: validate_retime_report rejects all of the following, each covered by a test: a parameters block that is not a valid crackvision.retime_config/1 echo (wrong schema, missing or extra nested key, out-of-range value); a non-ISO-8601 created_utc; and a point_range whose last index is ≥ totals.n_points.

## Finding 2 [minor] Report point_range and totals peak fractions are not tested against the trajectory
- Evidence: test_retime.py happy path: `first, last = entry["point_range"]; expected_duration = traj[...][last] - traj[...][first]`. This only checks duration against the report's own point_range; nothing asserts that points[first]/points[last] are the leg's boundary configurations. No test touches totals.peak_velocity_fraction or peak_acceleration_fraction, and ramp_in_m/ramp_out_m are only bounded, never recomputed.
- Consequence: The criterion says these must match the trajectory '(tested)'. A regression in span remapping or the peak-fraction computation would go undetected. The current implementation is correct; my out-of-tree probe matched the point ranges to the plan leg endpoints.
- Affected: ros2_ws/src/crackvision_motion/test/test_retime.py
- Acceptance condition: Tests assert that, for every leg, trajectory points[first] and points[last] equal that leg's first and last plan configuration (shared-boundary convention honoured). They also assert that peak_velocity_fraction and peak_acceleration_fraction equal values recomputed independently from the trajectory and limits, and that ramp_in_m/ramp_out_m equal the arc lengths of the leading/trailing below-band segments recomputed with tool_fk.

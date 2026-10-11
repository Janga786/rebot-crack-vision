# MOT-06.6.R1 — Repair MOT-06.6: Committed legs record recheck FK errors that violate §13.11.2's pass c

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#0",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 1500,
        "expect_exit": 0
      },
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "entry-point",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion plan_crack_path --help'",
        "timeout_s": 120,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "recheck.max_pos_err_m/max_ang_err_rad are computed only over FK-checked configurations, i.e. those with a target (waypoint_index_map != null, or the transit final configuration). A unit test shows that an approach leg with interpolated intermediates records an error ≤ fk_tolerance_m.",
      "Retry sub-segments are built from un-rolled base poses, with the last-reached index mapped back to the original traversal. Retry legs and the chosen field report roll and direction relative to paths3d. A unit test asserts that a retry's roll-0 candidate matches the base quaternions.",
      "On a fraction<1 approach, trace or retract leg, the planner does the following from the last-reached (or leg-start) configuration: IK on the next unreached waypoint, then /check_state_validity, then a max_joint_step comparison, yielding collision, joint_jump or cartesian_incomplete per §13.8. This is covered by a unit test with fake service results.",
      "The too_short decision uses cfg['splitting']['min_waypoints_per_remainder'], for example by checking the remainder length in run_segment_splits before calling split_fn. A unit test with a non-default value covers it.",
      "All acceptance criteria of MOT-06.6 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 4,
    "consequence": 4,
    "task_class": "ros-node",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-06.6",
  "findings": [
    "8e9a7e822fe1b8a8",
    "11ab2dba7b6b1849",
    "b3c8792ea1680086",
    "2d382ffda7d1d567"
  ],
  "id": "MOT-06.6.R1",
  "title": "Repair MOT-06.6: Committed legs record recheck FK errors that violate §13.11.2's pass c",
  "parent": "MOT-06",
  "outcome": "Resolve the review findings on MOT-06.6 while every acceptance criterion of MOT-06.6 still holds."
}
```

Repair work generated deterministically from review attempt `00582-MOT-06.6-review` of `MOT-06.6`.
Original card: `plan/cards/MOT-06.6.md` — its acceptance criteria must still hold.

## Finding 1 [major] Committed legs record recheck FK errors that violate §13.11.2's pass condition
- Evidence: plan_crack_path.py _index_map_and_errors updates max_pos_err/max_ang_err for 'every comparison attempted (matched or not)', and the approach, trace and retract legs store those values in recheck (e.g. `legs.append(LegResult("approach", approach_configs, idx_map, _recheck_dict(pos_err, ang_err, ...)))`). Simulation: an approach leg interpolated in 1 mm steps from z=0.04 to the trace pose at z=0.01 gives `recorded max_pos_err_m 0.03`; a trace leg with 1 mm configurations and 2 mm waypoints gives 0.002. Both exceed fk_tolerance_m=0.001.
- Consequence: Under §13.11.2 a leg's recheck passes iff max_pos_err_m ≤ fk_tolerance_m, so every committed approach/retract leg (and typically every trace leg) in a real plan reports itself as failing the independent re-check. This contradicts the contract that only passing legs appear in legs[] and misleads MOT-07 and any G-* gate that reads recheck.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py
- Acceptance condition: recheck.max_pos_err_m/max_ang_err_rad are computed only over FK-checked configurations, i.e. those with a target (waypoint_index_map != null, or the transit final configuration). A unit test shows that an approach leg with interpolated intermediates records an error ≤ fk_tolerance_m.

## Finding 2 [major] Split retries reuse the previous candidate's rolled/reversed poses, compounding roll and mislabelling chosen
- Evidence: run_segment_splits: `remainder = split_fn(fallback.rolled_task, fallback.last_reached_index)`. split_remainder keeps remainder_trace quats as given (already rolled), and the next round applies apply_candidate again. Simulation: after rolling forward by 30° and splitting, the retry's 'forward, roll 0' trace[0] is `29.999999999999993` degrees from the paths3d base orientation.
- Consequence: §13.7 requires the remainder (paths3d trace waypoints) to be retried 'with the full §13.4–13.6 procedure from scratch'. Instead, the retry's candidate set is offset by the previous roll (and flipped by a previous reverse). The §13.4 tie-break order is relative to the wrong base, and segments[].chosen and legs[].roll_offset_rad/direction for sub_segments misreport the actual orientation and direction relative to the input poses.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py
- Acceptance condition: Retry sub-segments are built from un-rolled base poses, with the last-reached index mapped back to the original traversal. Retry legs and the chosen field report roll and direction relative to paths3d. A unit test asserts that a retry's roll-0 candidate matches the base quaternions.

## Finding 3 [major] §13.8 fraction<1 shortfall classification (collision/joint_jump) not implemented
- Evidence: No classification code exists in plan_crack_path.py (grep 'classif' finds nothing). Shortfalls are recorded as `reasons.add(cart_err or "cartesian_incomplete")` and `return fail(err or "cartesian_incomplete")`. cartesian_moveit.parse_cartesian's docstring says it 'does not attempt the §13.8/§13.7 collision/joint_jump re-classification, which needs extra IK/validity calls this layer deliberately leaves to its caller'.
- Consequence: §13.8 normatively requires an extra IK plus validity/step attribution of each Cartesian shortfall. Without it, segments[].reasons never carry the collision or joint_jump attribution the contract defines for these cases, so operators lose the diagnostic §13.10 promises.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py
- Acceptance condition: On a fraction<1 approach, trace or retract leg, the planner does the following from the last-reached (or leg-start) configuration: IK on the next unreached waypoint, then /check_state_validity, then a max_joint_step comparison, yielding collision, joint_jump or cartesian_incomplete per §13.8. This is covered by a unit test with fake service results.

## Finding 4 [minor] splitting.min_waypoints_per_remainder config value is ignored
- Evidence: plan_crack_path.py never reads cfg['splitting']['min_waypoints_per_remainder'] (grep finds nothing). The default split_fn=split_remainder uses the hard-coded `_MIN_REMAINDER_WAYPOINTS = 3` (cartesian_core.py:41/596), yet the loader accepts any value ≥1 and the plan echoes it in parameters.
- Consequence: A config with min_waypoints_per_remainder ≠ 3 is silently disobeyed (§13.7 'fewer than splitting.min_waypoints_per_remainder ... too_short'), and the plan's echoed parameters misstate the rule that was applied.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py
- Acceptance condition: The too_short decision uses cfg['splitting']['min_waypoints_per_remainder'], for example by checking the remainder length in run_segment_splits before calling split_fn. A unit test with a non-default value covers it.

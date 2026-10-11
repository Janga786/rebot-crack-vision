# MOT-06.11 — plan_crack_path: fraction==1.0 traces are full (no pointer-walk stall on resampled trajectories) + decouple IK solver budget from service-call waits

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-06.6"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py",
      "config/motion/cartesian_planning.yaml",
      "ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py",
      "ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py",
      "ros2_ws/src/crackvision_motion/test/test_cartesian_core.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/cartesian/trace_full/**",
      "scripts/ros/test_plan_trace_full.sh",
      "docs/adr/017-cartesian-crack-path-planning.md",
      "docs/INTERFACES.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "docs/motion/CARTESIAN_PROBE.md",
    "logs/cartesian_probe.json",
    "ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py",
    "config/motion/cartesian_planning.yaml",
    "scripts/ros/_mock_stack.sh",
    "config/motion/specimen_placement.yaml"
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
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_plan_crack_path.py ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py ros2_ws/src/crackvision_motion/test/test_cartesian_core.py -q -p no:cacheprovider",
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
        "id": "live-trace",
        "cmd": "bash scripts/ros/test_plan_trace_full.sh",
        "timeout_s": 1500,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Before changing code, the measured mock-stack facts are recorded in an appended ADR-017 amendment: per-call duration of /compute_cartesian_path for the 0.06 m trace and of /compute_ik, the returned trace point count, and the FK spacing between consecutive returned configurations, showing why the §13.7 pointer walk stalls at fraction 1.0.",
      "Per §13.5/§13.7 and ADR-017 §3, a step-4 trace that returns fraction == 1.0 reached every requested waypoint, and the FK pointer walk no longer decides whether it is full. The pointer walk is used only when fraction < 1, unchanged. A unit test with fake services shows a fraction-1.0 trace is `full` even when its configurations are resampled with spacing larger than 2·fk_tolerance_m, provided every configuration passes §13.8. Another unit test shows fraction < 1 behaviour is unchanged.",
      "§13.8 FK agreement is still applied to every trace configuration of a fraction-1.0 trace, not only the waypoint-matched ones. Each configuration's /compute_fk tool_tip position is within recheck.fk_tolerance_m of the requested (rolled) trace polyline. Its orientation is within recheck.fk_tolerance_deg of the orientation between the bracketing requested waypoints. The final configuration matches the last waypoint. Any violation fails the candidate with fk_mismatch, and a unit test with one deviating fake configuration proves it. recheck.max_pos_err_m and max_ang_err_rad report these errors. The rule is written as an appended amendment: an ADR-017 addendum plus a new appended numbered INTERFACES section. §13 is never edited in place.",
      "The /compute_ik solver budget is no longer the same number as the per-call wait for /compute_cartesian_path and /plan_kinematic_path. With TRAC-IK solve_type Distance spending its whole budget on every call, a body-sized trace call is not aborted by its wait, and an IK call does not take 10 s. Every new or changed key in config/motion/cartesian_planning.yaml carries value_status and a source, is validated by the loader (unit-tested, including rejection of invalid values), and is documented in the appended contract.",
      "scripts/ros/test_plan_trace_full.sh starts the mock stack via _mock_stack.sh and plans a committed test-only fixture. The fixture is a single straight boresight-down segment of 0.06 m with 31 trace waypoints at 0.002 m, centred at the recommended placement (0.26, 0.0) on a surface at z 0.010 m, with a matching test-only scene. Planning uses the committed config/motion/cartesian_planning.yaml with no override. The script asserts from the written plan file that the segment is `full` with trace_waypoints planned == total, that every leg's recheck passes, and that the planner finished within 300 s. It tears the stack down on every exit path. The fixture is never shortened, nor the tolerances relaxed, to make it pass.",
      "config/scene/scene.yaml is untouched. No code path gains any execution or motion capability. plan_view/MOT-06.9 behaviour is unchanged.",
      "All acceptance criteria of MOT-06.6 and MOT-06.6.R1 still hold."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 4,
    "task_class": "ros-node",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "id": "MOT-06.11",
  "title": "plan_crack_path: fraction==1.0 traces are full (no pointer-walk stall on resampled trajectories) + decouple IK solver budget from service-call waits",
  "parent": "MOT-06",
  "outcome": "plan_crack_path plans a body-sized (0.06 m, 31-waypoint) boresight-down straight trace at the recommended placement as `full` on the mock stack with the committed config/motion/cartesian_planning.yaml, in bounded time: step-4 full-ness follows §13.5 (fraction == 1.0), every trace configuration is still FK-checked against the requested polyline, and the /compute_ik solver budget is no longer the same number as the Cartesian/motion-plan call wait.",
  "source": "debugger D0010"
}
```

## Why this card exists (debugger D0010)

MOT-06.7 (headless smoke test of plan_crack_path) timed out three times. The cause is in the accepted planner, not in the smoke test.

1. **Full-trace check stalls on resampled trajectories.** `plan_crack_path.py:539-562` decides step-4 full-ness with the §13.7 FK pointer walk (`_index_map_and_errors`, :137-164) even when `/compute_cartesian_path` returns `fraction == 1.0`. §13.5 ("step 4 at `fraction == 1.0`"), §13.7 ("If no candidate's step 4 reaches `fraction == 1.0`") and ADR-017:95 ("`fraction == 1.0` skips it entirely") say otherwise. move_group time-parameterises and resamples the trajectory: CARTESIAN_PROBE.md / `logs/cartesian_probe.json` show 18 points for 32 waypoints on a 0.06 m line, FK error < 0.95 mm. The walk can advance only one waypoint per configuration and never recovers after a skipped waypoint, so a body-sized trace stalls (12/31 in attempt 00613). In 00614's `logs/test_cartesian_plan_launch.log`, all 155 Cartesian calls 'followed 100%', yet no retract is ever requested: all 24 candidates fail, then the splits retry.
2. **One timeout plays two roles.** `service_timeout_s` (2.0 s) is both the call wait and the TRAC-IK solver budget (`cartesian_moveit.py:202-215`). The rebot_ws kinematics.yaml uses `solve_type: Distance`, `kinematics_solver_timeout: 0.05`. A 0.06 m trace at `max_step 0.001` takes 3-4.6 s, so the default config aborts with exit 1 (00614 stream: "service call to '/compute_cartesian_path' did not complete within 2.0s"). Raising it to 10 s makes every `/compute_ik` take the full 10 s, about 20 s per candidate (exact 20.0 s gaps in the launch log) and about 20 min per run.

## What to do

Measure first (criterion 1), then fix both defects with unit tests using fake services, and add the small live check `scripts/ros/test_plan_trace_full.sh` with its fixture under `test/fixtures/cartesian/trace_full/`. Follow the pattern of `scripts/ros/test_reachability_task_frames.sh` / `_mock_stack.sh` (`set -m`, source env_ros + overlay, `start_mock_stack`, trap teardown). Record any contract clarification only as appended text (ADR-017 addendum + a new appended INTERFACES section).

Time budget: the session limit is 3600 s. Run long commands in the foreground with explicit timeouts, do not use ScheduleWakeup, and submit the structured result well before the limit.

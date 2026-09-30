# MOT-04.4 — MoveIt reachability sweep node (compute_ik + independent FK re-check) + headless run/test scripts

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-04.2"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py",
      "ros2_ws/src/crackvision_motion/setup.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/reachability_smoke.yaml",
      "scripts/ros/_mock_stack.sh",
      "scripts/ros/run_reachability.sh",
      "scripts/ros/test_reachability.sh",
      "docs/motion/ROS_WORKSPACE.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#7",
    "docs/motion/ROS_WORKSPACE.md",
    "docs/motion/ROBOT_MODEL.md#4",
    "scripts/ros/env_ros.sh",
    "scripts/ros/test_mock_plan.sh",
    "ros2_ws/src/crackvision_motion/launch/mock_planning.launch.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_map.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "config/motion/reachability.yaml",
    "config/robot/b601_dm_limits.yaml"
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
        "id": "sweep-smoke",
        "cmd": "bash scripts/ros/test_reachability.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "no-execution-paths",
        "cmd": "bash -c '! grep -nE \"ExecuteTrajectory|FollowJointTrajectory|execute_trajectory|MoveGroup\\b|plan_only *= *False|/move_action\" ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "mock-plan-regression",
        "cmd": "bash scripts/ros/test_mock_plan.sh",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The sweep uses only the /compute_ik, /compute_fk, /get_planning_scene and /apply_planning_scene services and the /robot_description topic. There is no action client, trajectory or controller interaction, so nothing can move even the mock arm.",
      "Every reachable target was FK-verified through /compute_fk: position error ≤ 1e-3 m and boresight axis error ≤ 0.5°. A disagreement yields status fk_mismatch and exit 1, never a silent pass.",
      "Joint solutions are checked against config/robot/b601_dm_limits.yaml. Margins use reachability_map.joint_limit_margin.",
      "The surface slab is added per surface_z layer, with ACM allowances fetched, extended and re-applied, and it is removed at the end and on error. The map records whether surface collision was on.",
      "The reach bound comes from the live /robot_description via reachability_core.reach_bound_from_urdf. Prefiltered targets are not IK-queried.",
      "The fixture's near targets are shown reachable and the far targets are unreachable or prefiltered. The fixture choice is justified by a recorded probe, not by loosening tolerances.",
      "run_reachability.sh leaves no stray move_group / ros2_control_node / robot_state_publisher / spawner processes from its own process group, and test_reachability.sh asserts this.",
      "Without the service stack: exit 3 within --service-timeout-s. With --require-all-reachable and any non-reachable target: exit 1. --max-duration-s produces complete:false and a §0.4 status of partial."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 4,
    "task_class": "ros2-moveit-integration",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-04.4",
  "parent": "MOT-04",
  "title": "MoveIt reachability sweep node (compute_ik + independent FK re-check) + headless run/test scripts",
  "outcome": "`reachability_sweep` queries move_group's /compute_ik for every target × orientation sample, with a surface collision slab and seed continuity. It re-checks every IK solution with /compute_fk and writes a §7 map. `scripts/ros/run_reachability.sh` wraps launch → sweep → teardown, and `scripts/ros/test_reachability.sh` proves it on a tiny fixture. The whole path is plan-free and execution-free."
}
```

This is the ROS side of MOT-04. It runs against the MOT-02 headless mock stack (mock_planning.launch.py). move_group's default capabilities already provide /compute_ik, /compute_fk, /get_planning_scene and /apply_planning_scene.

IK for the `arm` group is trac_ik. It needs libnlopt.so.0, which env_ros.sh already adds from $HOME/opt/nlopt/lib (ROS_WORKSPACE.md §5). Verify that the plugin loads by checking the launch log. If it does not load, the IK calls fail, which the smoke test must surface as an error, not as an 'unreachable' result.

### `reachability_sweep.py` (console script `reachability_sweep`, via cli_common.run_cli)
Flags:
- The §0.3 common flags.
- `--reachability-config` (default config/motion/reachability.yaml).
- `--limits` (default config/robot/b601_dm_limits.yaml).
- `--out` (default data/motion/reachability_map.json).
- `--service-timeout-s` (default 90).
- `--max-duration-s` (default 0 = unlimited).
- `--require-all-reachable`.

`--dry-run` loads the config, logs the target and orientation counts and the IK-call upper bound, and exits 0 without needing ROS services.

Procedure:
1. `rclpy.init`. Wait for the four services, or raise PreconditionError (exit 3).
2. Get the URDF from the `/robot_description` topic (std_msgs/String, QoS transient_local, reliable) with a timeout, otherwise exit 3. Check that `ik_link` exists in it. Compute `reach_bound_m`.
3. Enumerate targets with `reachability_core.enumerate_targets` and orientations with `reachability_core.orientations`.
4. For each surface_z layer, if surface_collision is enabled:
   - Apply a planning-scene diff adding CollisionObject `mot04_surface`: a BOX in `base_link`, with x/y extent = grid extent + 2·margin_m and height thickness_m, whose top face is at surface_z − 0.001.
   - Get the current ACM (/get_planning_scene with components ALLOWED_COLLISION_MATRIX), add `mot04_surface` with allowed=true against `allowed_links` and false against every other entry, and re-apply the ACM as part of a diff.
   - Remove the object (and restore the ACM) after the layer, and in a finally block.
5. For each target:
   - If prefilter is enabled and `prefiltered(position)`, set status prefiltered and make no IK call.
   - Otherwise iterate over the orientations. Send a `GetPositionIK` request with group, ik_link_name, a PoseStamped in base_link, avoid_collisions from config, timeout from config, and the seed robot_state. The seed is the last successful joint vector when `seed: neighbour` (the serpentine order keeps neighbours adjacent), otherwise the SRDF `home` state (read joint values from the SRDF group_state via the /robot_description_semantic topic, or fall back to all-zero if absent, and log which).
   - `roll_search: first` stops at the first success. `best` evaluates all orientations and keeps the maximum joint-limit margin.
   - FK re-check every chosen solution with `GetPositionFK` for ik_link. The position error is the norm of the difference. The axis error is the angle between R_fk·axis_local and d_world.
   - Also check the solution against the limits file.
   - Record the §7 fields. A service exception gives status error.
6. Write the map atomically (reachability_map.write_map). `complete` is false if --max-duration-s cut the run short.
7. Exit codes: 1 if any fk_mismatch or error, or if --require-all-reachable is set and anything is non-reachable. Otherwise 0.
8. Log progress every ~5% with counts and ETA.

Hard rule: no action clients, and no `/move_action`, ExecuteTrajectory or FollowJointTrajectory. The acceptance grep enforces this. The file must not even mention MoveGroup.

### Scripts
- `scripts/ros/_mock_stack.sh` (sourced helper) factors out the launch/pgid/cleanup pattern from test_mock_plan.sh: `start_mock_stack LOGFILE` and `stop_mock_stack`, with a trap on EXIT/INT/TERM, SIGINT to the group, a 10 s wait, then SIGKILL. Do NOT modify test_mock_plan.sh; it is accepted.
- `scripts/ros/run_reachability.sh [--reachability-config P] [--out P] [--require-all-reachable] [--max-duration-s N] [extra args → sweep]`:
  - Source env_ros.sh and the overlay (exit 3 with a hint if the overlay is not built).
  - Start the stack, logging to ros2_ws/log/reachability_launch.log.
  - Run `ros2 run crackvision_motion reachability_sweep …`, then tear down.
  - Exit with the sweep's exit code. On failure, print the tail of the launch log.
- `scripts/ros/test_reachability.sh`:
  - Run run_reachability.sh with `ros2_ws/src/crackvision_motion/test/fixtures/reachability_smoke.yaml` → `logs/test_reachability_map.json`.
  - Then, with system python3 and the map loader, assert: complete; ≥ 1 reachable; every target at x ≥ 0.9 m is prefiltered or unreachable; zero fk_mismatch/error; every reachable joint vector within limits.
  - Finally, assert no process from the launch pgid survives.

### Fixture
`reachability_smoke.yaml` is a valid reachability_config/1 with about 6 targets, e.g. x {min 0.20, max 0.95, step 0.75}, y {−0.02..0.02, step 0.02}, surface_z [0.12], standoffs [0.01], roll_samples 8, surface collision on.

First probe with a throwaway run that the near targets are reachable under MoveIt. The presentation's §2.5 found (0.22, 0) at z 0.12 comfortably reachable, but for gripper_link, which is 44.3 mm behind gripper_tcp. If they are not reachable, choose a near region that the probe shows is reachable and record the probe command and output in ROS_WORKSPACE.md. Never relax the 1 mm / 0.5° FK tolerances or the limits to make the test pass.

### Docs
Append to docs/motion/ROS_WORKSPACE.md a new section, 'Reachability sweep (MOT-04)', covering the commands, the services used, the no-execution guarantee, the fixture probe result, and the runtime of the smoke test.

### setup.py
Add the `reachability_sweep` console script. Keep the change additive; the file is shared with MOT-03 and MOT-04.3.

## Amendment 2026-09-30 (technical-lead recovery, ADR-014) — sanctioned changes, re-review against these
- The text above says the presentation's gripper_link is '44.3 mm behind gripper_tcp'. That was backwards:
  gripper_link's origin is the closed-finger tip, 44.3 mm DISTAL to gripper_tcp (the grasp centre).
- MOT-04.6 made the surface object a keep-out-aware specimen block when surface_collision.model is
  `specimen_block` (one CollisionObject, possibly several box primitives) and added optional environment objects
  from a scene config; the default `slab` geometry, the smoke fixture and this card's checks are unchanged.
- The mock stack (mock_planning.launch.py) now loads the GEOM-10 end-of-arm overlay (tool_tip, camera_link,
  camera proxies); the no-execution guarantee is unchanged.

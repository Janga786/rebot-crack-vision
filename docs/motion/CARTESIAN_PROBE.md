# CARTESIAN_PROBE.md — live `/compute_cartesian_path` and `/plan_kinematic_path` probe (MOT-06.1)

**Date:** 2026-10-10. **Command:** `bash scripts/ros/probe_cartesian_path.sh` (builds, launches the
headless mock stack via `scripts/ros/mock_planning.launch.py`, runs `scripts/ros/probe_cartesian_path.py`,
tears the stack down). Raw observations: `logs/cartesian_probe.json`.

This is a read-only research probe against the headless mock stack (ROS 2 Humble + MoveIt as built
into this repo's overlay; no driver, no real arm). It applies a probe-local collision scene (table
top at `z=0` + a 0.20×0.20 m specimen box, top face `z=0.010`, centred at the recommended placement
`(0.26, 0.0)` from `config/motion/specimen_placement.yaml`), never edits `config/scene/scene.yaml`,
and never sets `avoid_collisions: false`. No action client is ever created; nothing here can move
the mock (let alone real) arm.

## Facts

| Fact | Observed value | JSON key |
|---|---|---|
| `GetCartesianPath.Request` fields on this Humble install | `header`, `start_state`, `group_name`, `link_name`, `waypoints`, `max_step`, `jump_threshold`, `prismatic_jump_threshold`, `revolute_jump_threshold`, `avoid_collisions`, `path_constraints` — no separate `cartesian_jump_threshold`; the old single `jump_threshold` coexists with the newer split `prismatic_jump_threshold`/`revolute_jump_threshold` | `request_fields.GetCartesianPath.Request` |
| `GetMotionPlan.Request` fields | one field, `motion_plan_request` (`moveit_msgs/MotionPlanRequest`); the per-field request lives there (`group_name`, `start_state`, `goal_constraints`, `planner_id`, `num_planning_attempts`, `allowed_planning_time`, …) | `request_fields.GetMotionPlan.Request (via MotionPlanRequest)` |
| `tool_tip` accepted as `link_name`/`ik_link_name` | yes, for both `/compute_ik` and `/compute_cartesian_path` | `queries.cartesian_tool_tip_line_no_roll`, `queries.compute_ik_tool_tip_singularity_fallback` |
| `camera_link` accepted as `link_name`/`ik_link_name` | yes | `queries.compute_ik_camera_link`, `queries.cartesian_camera_link_line` |
| **Contradicts the MOT-06 card body's literal pose**: `tool_tip` 0.04 m above the specimen with `+X` pointing *exactly* straight down (roll = 0 about the boresight) | `/compute_ik` returns `NO_IK_SOLUTION` (`error_code = -31`) for every seed tried (home-zero seed used by the probe, plus 10 additional random seeds tried interactively while debugging this card — see Consequences). This is a genuine wrist singularity, not a seed or collision artifact: it reproduces with `avoid_collisions: false` too. Any roll a few degrees away from 0 (every 15° sample from 15° to 345° tried) succeeds immediately. | `queries.compute_ik_tool_tip` (`error_code: -31`), `queries.compute_ik_tool_tip_singularity_fallback` (`roll_deg: 2.0`, `success: true`) |
| FK round-trip error of the returned trajectory vs. its nearest (by position) requested waypoint | max position error ≈ 1e-4 m (after correcting for the resampling artifact below — see "last trajectory point" caveat), max orientation error ≈ 3e-5 deg | `queries.fk_check_line_no_roll` |
| Meaning of `fraction` | fraction of the **cartesian path length** achieved, not a fraction of the input waypoint *count*. The returned `RobotTrajectory`'s point count is **not** `len(waypoints)`: `/compute_cartesian_path` re-interpolates the whole polyline (start_state's own pose → waypoint[0] → waypoint[1] → …) at `max_step` resolution and then time-parameterizes it, so dense input waypoints (here, spaced 0.002 m, finer than `max_step = 0.005 m`) get consolidated. 32 requested waypoints produced an 18-point trajectory for the no-roll line and a 36-point trajectory for the roll-ramp line (rotation forces finer sampling even though the Cartesian positions are identical) | `queries.cartesian_tool_tip_line_no_roll` (`num_requested_waypoints: 32`, `trajectory.num_points: 18`), `queries.cartesian_tool_tip_line_roll_ramp` (`trajectory.num_points: 36`) |
| How to identify the last reached requested waypoint | `round(fraction * (len(waypoints) - 1))`, **not** the returned trajectory's point count or index (the resampling above makes those uncomparable; indexing the trajectory by position directly gave a spurious ~0.03 m "error" for the first point, which is actually the real initial ramp segment from `start_state`'s pose down to waypoint[0], not a tracking error) | `queries.fk_check_line_no_roll.last_reached_waypoint_index_from_fraction`, `.note` |
| Non-zero `time_from_start` | yes: the returned `RobotTrajectory.joint_trajectory` is time-parameterized (`first_time_from_start_s: 0.0`, `last_time_from_start_s` 1.63 s for the 0.06 m no-roll line, 3.45 s for the roll-ramp line, 1.17 s for the camera view line) | `queries.cartesian_tool_tip_line_no_roll.trajectory`, `queries.cartesian_tool_tip_line_roll_ramp.trajectory`, `queries.cartesian_camera_link_line.trajectory` |
| Behaviour with a waypoint 0.005 m inside the specimen, `avoid_collisions: true` | stops before the colliding waypoint: `fraction = 0.0404` of 33 requested waypoints → `round(0.0404 * 32) = 1`, i.e. it reached waypoint index 1 (the first line point, right before the colliding waypoint at index... the collision waypoint was itself placed at index 1 of this run, so the path correctly stops at the boundary) and returns `SUCCESS` (not an error code) with the partial trajectory | `queries.cartesian_tool_tip_collision_waypoint` |
| Effect of a roll change about `tool_tip` `+X` between waypoints (0°→90° ramp vs. none, same positions) | `fraction` stays `1.0` in both cases, but the returned trajectory roughly doubles in point count (18 → 36) and duration (1.63 s → 3.45 s): the interpolation/time-parameterization accounts for the added rotational motion even though the Cartesian path length (translation only) is unchanged | `queries.cartesian_tool_tip_line_no_roll` vs. `queries.cartesian_tool_tip_line_roll_ramp` |
| `/plan_kinematic_path` with an explicit `start_state` (home, all-zero) and a joint-constraint goal equal to the step-2 IK solution | accepted; `SUCCESS`, 43 trajectory points, planning time 0.27 s, no `planner_id` was set in the request (empty string — the pipeline's configured default planner is used) | `queries.plan_kinematic_path` |

## Consequences for MOT-06.2

1. **The card's literal step-2 pose (`tool_tip`, +X exactly straight down, roll = 0) has no IK
   solution on this arm — it sits exactly on a wrist singularity.** Any production
   approach/retract/trace motion that nominally wants "straight down" must plan at a roll a few
   degrees off that singularity (this probe used 2°, picked arbitrarily — not a recommendation for
   the real tolerance/margin, which MOT-06.2 should size properly), or must treat exact roll = 0 as
   a special case to avoid. This also means any reachability sweep or recommendation that samples
   orientation at roll = 0 only (not the case for `reachability_sweep.py`'s own `roll_samples: 8`
   search, which already avoids landing exactly on 0) would misreport this task frame/pose as
   unreachable.
2. **`fraction` is a path-length fraction, not a waypoint-count fraction, and the returned
   trajectory's point count has no fixed relationship to the number of input waypoints.** MOT-06.2
   must map `fraction` back to "how far along the crack did we get" via
   `round(fraction * (len(requested_waypoints) - 1))` against the *requested* waypoints, never by
   indexing or counting the returned `RobotTrajectory`.
3. **Dense waypoints spaced finer than `max_step` do not force dense output.** If MOT-06.2 needs a
   trajectory point at every crack-sample point (e.g. for per-point speed/dwell control), it must
   either shrink `max_step` below its own waypoint spacing or post-process/resample the returned
   trajectory — supplying closely-spaced waypoints alone does not guarantee a matching density of
   output points.
4. **A roll/orientation change inflates the output trajectory (more points, longer duration) even
   when the Cartesian translation is unchanged.** Rough budgeting for trace speed/time should not
   assume duration scales only with translated distance once the tool starts rotating along the
   path (e.g. to track crack-normal tilt).
5. **Collision stop semantics confirmed:** with `avoid_collisions: true` and a waypoint placed
   inside the specimen, `/compute_cartesian_path` returns `SUCCESS` (not an error) with
   `fraction < 1` and a trajectory that stops at the last collision-free waypoint — MOT-06.2 must
   check `fraction` (or the mapped last-waypoint index), not just the error code, to detect a
   collision-truncated trace.
6. **Both `tool_tip` and `camera_link` are valid `ik_link_name`/`link_name` values** for
   `/compute_ik` and `/compute_cartesian_path` on this install, consistent with ADR-014 §8.2 — no
   contradiction found there.
7. **`/plan_kinematic_path` works with an explicit `start_state` + joint-constraint goal**, which is
   the shape MOT-06.2 needs for "approach via a joint-space plan, then trace via Cartesian path."
   The request never set `planner_id`, so MOT-06.2 should decide explicitly whether to pin one or
   rely on the pipeline default.

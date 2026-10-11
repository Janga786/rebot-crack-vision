# ADR-019: Time parameterization and an independent trajectory checker

**Status:** accepted · **Date:** 2026-10-11 · **Supersedes:** — · **Superseded by:** —

## Context

MOT-06 (ADR-017, `docs/INTERFACES.md` §13/§15) produces an **untimed** `cartesian_plan` — a
continuous, collision-checked joint path with no velocities, accelerations or timestamps. MOT-07
has to turn that into a `crackvision.joint_trajectory/1` file (§11.2) that `execute_trajectory`
(ADR-016, §11) can gate and run: every point within `b601_dm_limits.yaml`'s position bound and the
§11.6 margin rule at scale 1.0, every segment's finite-difference (FD) velocity/acceleration within
limits, because that is exactly what `G-LIMITS` (§11.7) checks and exactly what the vendor driver
executes — `ros_actions.py:147-184` plans each commanded segment as a **time-ratio linear
interpolation** between consecutive trajectory points, so the FD values *are* the physically
commanded motion, not an approximation of some other continuous curve. REQ-MOT-2 ("time
parameterization within vel/acc limits") is this card's job; REQ-MOT-1's real B601-DM limits file
is the bound it must respect.

Two off-the-shelf retimers were the obvious first choice and both are ruled out by directly
verified facts (commands run against this repo's environment, 2026-10-11):

1. **MoveIt's Time-Optimal Trajectory Generation (TOTG) has no Python binding on this Humble
   install.** `ls /opt/ros/humble/lib/python3.10/site-packages | grep -i moveit` lists only
   `moveit_configs_utils` (and its egg-info) — there is no `moveit_py` package, so
   `moveit_core`'s TOTG implementation, which is C++-only, is not callable from any Python
   process this project runs (neither `/usr/bin/python3` nor `./env.sh python`). Calling it would
   require writing and maintaining a C++ ROS node, a categorically bigger commitment than this
   card's scope, for a project whose every other MOT-06/07 module (`cartesian_plan.py`,
   `joint_trajectory.py`) is deliberately pure Python for testability outside a ROS graph.
2. **`ros-humble-ruckig` is installed as a Debian package but has no importable Python module
   under either interpreter this project uses.** `dpkg -l ros-humble-ruckig` reports it installed
   (`0.9.2-1jammy.20260226.012139`), but `python3 -c 'import ruckig'` fails with
   `ModuleNotFoundError: No module named 'ruckig'` under both `/usr/bin/python3` and
   `./env.sh python` (checked 2026-10-11). The Debian package ships the C++ library and ROS
   message/service wrappers, not the `ruckig` Python wheel (which is a separate PyPI package this
   install does not have `pip`-installed). Beyond availability, Ruckig's own API is **point-to-point**
   (it generates a time-optimal trajectory between a small number of waypoints under velocity/
   acceleration/jerk limits), not multi-waypoint path-following: feeding it every plan configuration
   as a sequence of independent point-to-point problems would not guarantee the arm follows the
   checked Cartesian polyline between them — exactly the blending problem TOTG's own
   `path_tolerance` corner-rounding has (next paragraph), with no MoveIt-side mitigation available
   for a standalone library call.
3. **TOTG blends corners off the checked path, by design.** Even if a Python binding existed,
   `moveit_core`'s `TimeOptimalTrajectoryGeneration` fits `path_tolerance`-bounded cubic splines
   through the input waypoints, deliberately deviating from the input polyline at corners to stay
   time-optimal under the velocity/acceleration bound. §13's independent re-check and §13.9's
   surface/scene check certify the geometric path `plan_crack_path` actually produced — a corner
   blend not derived from that exact checked data is an unchecked position, specifically at trace
   corners near the specimen (`trace_clearance_m`, the tightest clearance in the whole plan). This
   card refuses to introduce an unchecked excursion from a path that was this project's whole reason
   for building an independent re-check (§13.8) in the first place.

Both facts and the corner-blending concern point the same way: a retimer that (a) runs as pure
Python/numpy, ROS-free, so it can be tested the same way as `joint_trajectory.py`/`cartesian_plan.py`
already are, and (b) follows the checked plan polyline **exactly**, with no corner blending, trading
away TOTG's time-optimality for geometric fidelity to an already-safety-checked path. §11.6/§11.7
also fix the target semantics: `G-LIMITS` checks **finite-difference** velocity/acceleration (never
explicit fields trusted on their own), and the vendor driver's own linear interpolation
(`ros_actions.py:147-184`) means FD values over the written `t_s` grid are exactly what gets
commanded — so a retimer that reasons in FD terms over a *known* sample grid, rather than producing
a continuous analytic curve it then samples, matches the system it feeds without a translation step
that could introduce daylight between "what was checked" and "what was commanded." §11.12 records a
20 ms vendor driver control tick, which bounds how fine a sample period needs to be before an FD
acceleration value reflects a real commanded velocity step rather than an artifact of oversampling a
coarser physical rate.

An example from the MOT-06.10 synthetic run (`logs/cartesian_synthetic/*/cartesian_plan.json`,
retained only as a non-committed log artifact but inspected for this ADR) confirms the input scale
this retimer actually sees: `transit` legs step up to 0.1 rad per joint between consecutive plan
configurations, while `trace` legs step roughly 0.02 rad — both coarse relative to a 20 ms control
tick at any sane angular speed, i.e. the untimed plan's own spacing cannot be executed directly and
must be both time-scaled and, for the coarser legs, resampled finer before FD values are meaningful.

## Decision

Build a new, pure-Python/numpy, ROS-free retiming algorithm (`crackvision_motion.retime_plan`,
system `python3`, no `rclpy`/`moveit`/`ruckig` import) that:

1. Concatenates the plan's `legs[].positions` (dropping each leg's duplicated boundary
   configuration, `cartesian_plan.CONTINUITY_TOL_RAD`) into one polyline, copied bit-exact from the
   plan — the plan's own §13.8 independent re-check already certified every one of these
   configurations; retiming must never perturb them.
2. Refines that polyline by linear subdivision only (no corner blending) so no per-joint step
   exceeds a configured bound — the geometric path the arm follows is *exactly* the checked
   polyline, segment by segment; slowing down at a corner (never cutting it) is the accepted
   trade-off against TOTG's smoother-but-off-path cornering.
3. Runs a discrete forward–backward velocity-profiling pass over per-segment path speeds, enforcing
   the FD velocity/acceleration bounds (at a configured fraction of the §13.8-independent
   `b601_dm_limits.yaml` numbers), a rest-to-rest start/end, and the §11.6 margin rule, then
   subdivides any segment whose resulting duration exceeds a configured max sample period and
   re-profiles, bounded in iteration count.
4. Additionally holds trace legs at one near-constant tool-tip speed (bisected to the fastest value
   that still respects every other bound), using FK from a new `tool_fk` module built the same way
   `crackvision.kinematics` builds `FK(q)` (URDF chain composed with `end_effector.yaml`'s `tool`
   block) so the retimer's notion of "tool tip" is the exact same frame GEOM-08/ADR-014 already
   define and `crackvision.kinematics` already implements — parity between the two is tested
   directly, not assumed.
5. Self-checks the result with `joint_trajectory.check_limits` at full (unscaled) limits before
   ever writing a file: any violation refuses the run (exit 3). The retimer never clamps a value
   that would otherwise violate a bound and never applies a hidden uniform slowdown to make a
   check pass silently — a design that violates its own target bounds is a bug in the profiling
   pass, not something to paper over at the last step.
6. Writes positions and `t_s` only — no explicit `velocities`/`accelerations` — because the vendor
   driver's own linear interpolation (`ros_actions.py:147-184`) commands the FD values regardless of
   what any explicit field said, and writing a second, unchecked set of numbers next to the ones
   that are actually checked and actually commanded would be exactly the kind of "measured-looking
   but not actually verified" data this project's `value_status` discipline exists to avoid
   elsewhere (ADR-014 §8.1's own reasoning, applied here to trajectory fields instead of config
   values).

The full normative schema, output guarantees, defaults and CLI contracts are appended as
`docs/INTERFACES.md` §16, in the same append-only style §15 already established for this file (§15
itself is the pattern: a new numbered section states and amends, never edits earlier sections in
place). §16 also specifies a **separate, independent trajectory checker**
(`crackvision.trajectory_check`) that re-derives every one of §16's output guarantees from scratch,
in the conda environment, without importing `crackvision_motion` and using FK from
`crackvision.kinematics` rather than the retimer's own `tool_fk` — the same "two independently
reasoned implementations agree" discipline this project already applies to IK/FK
(`/compute_fk` vs. `cartesian_plan`'s own re-check, §13.8) and to the planner vs. `G-LIMITS` (ADR-016
§5's "a sha match proves provenance, not correctness" reasoning). A single implementation checking
its own output can share a bug between the two; a geometrically-independent checker, deriving leg
membership and tool speed from the trajectory and plan data alone rather than trusting the
retimer's own bookkeeping, cannot share that retimer's bug by construction.

**Time-optimality is explicitly not claimed.** The profiling pass is a near-optimal, bounded-
iteration heuristic over discrete segment speeds, not an analytic time-optimal solution; §16 states
a single-axis, single-straight-move sanity bound (within 10% of the analytic trapezoid/triangle
profile at the configured velocity/acceleration fraction) as the only optimality claim made.

**§11.2's `purpose` gap is named, not solved, here.** §13.14's `view_plan` also needs retiming (a
`home → view pose` transit leg) before it could ever execute, but §11.2 only defines
`purpose: crack_task` (requires non-null `source.paths3d`) and `purpose: test` (refused
unconditionally in `real`, §11.7 `G-TRAJ`) — neither fits a view move today. §16.8 records this gap
and proposes, but does not implement, a `purpose: "view_move"` amendment for a future card
(MOT-07.10) to settle with its own `G-TRAJ`/`G-ELIGIBLE` rule changes. This card's retimer handles
`cartesian_plan` (crack-task) retiming only; `execution_gate`/`joint_trajectory` behaviour is
unchanged.

## Consequences

**Makes easy:**
- MOT-07.2+ implement one frozen algorithm and schema (§16) instead of re-deriving the FD-vs-explicit
  semantics, the rest-to-rest rule or the §11.6 margin interaction per sub-card.
- The retimer and `execute_trajectory`'s `G-LIMITS` agree by construction, because both reason in
  the same FD terms over the same vendor linear-interpolation model — there is no risk of a
  retimer that is "correct" by its own (different) definition of velocity/acceleration but fails
  `G-LIMITS` anyway, or passes `G-LIMITS` while silently violating what the arm actually does.
- A ROS-free, system-`python3` retimer is unit-testable without a colcon build or a ROS graph, the
  same way `joint_trajectory.py`/`cartesian_plan.py` already are.
- The independent checker gives a reviewer (human or automated) a second, structurally different
  opinion on every trajectory before it ever reaches `execute_trajectory`'s own gates.

**Costs:**
- No corner blending means every corner in a plan slows the arm down relative to what TOTG would
  produce on the same waypoints; crack paths with many sharp direction changes pay more than smooth
  ones. This is accepted explicitly (Decision, point 2) in exchange for exact path fidelity.
- Maintaining a hand-written profiling pass (forward–backward velocity limiting, period-bounded
  resampling, trace-leg constant-speed bisection) is more code than calling a vetted library would
  have been, had one been callable from Python here.
- Two independent implementations (retimer + checker) must be kept in sync with every amendment to
  §16's guarantees, exactly as MOT-05/MOT-06's existing independent re-checks already require of
  their own planner/gate pairs.

## Revisit when

- A `moveit_py` binding (or a `ruckig` Python wheel) becomes available on a future ROS Humble/Jazzy
  install this project actually runs, at which point TOTG/Ruckig's corner-blending and point-to-point
  trade-offs can be re-weighed against this card's exact-path, no-blend guarantee with a real
  available alternative on the table.
- Real crack geometry (post-commissioning) shows the no-corner-blending slowdown is operationally
  unacceptable for some class of path, prompting a bounded corner-rounding amendment that keeps the
  independent re-check's role intact.
- MOT-07.10 settles the `purpose: "view_move"` gap named in Decision/§16.8.

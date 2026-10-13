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

## Addendum (MOT-07.11, debugger D0012): vertex-tied tool-speed dips on real trace legs

**Status:** accepted (MOT-07.11). Amends INTERFACES §16.1 (field descriptions), §16.2 (G-TOOLSPEED
rules 2-3), §16.5 (C-TOOLSPEED `v_c` inference, leg-range slack) and §16.7 (table rows).

### Context

MOT-07.8 ran `retime_plan` on the committed MOT-06.10 fixture
(`ros2_ws/src/crackvision_motion/test/fixtures/retime/synth_plan/cartesian_plan.json`, a verbatim
`run_cartesian_synthetic.sh` plan) with the committed `config/motion/retime.yaml`. It refused with
`trace leg 2: no constant tool speed in [0.005, 0.05] m/s satisfies G-TOOLSPEED` (exit 3). Earlier
MOT-07 tests only used smooth synthetic trace legs.

### Measured evidence

Reproduced with the fixture, its `b601_dm_limits.yaml`/`end_effector.yaml` copies and the committed
`retime.yaml`/`execution.yaml`, profiling the trace leg at fixed `v_c` exactly as one bisection trial
does. Throwaway script, not committed.

- **Joint-space turns.** The trace leg has 21 plan points and 64.4 mm of tool travel (chords
  0.61-4.85 mm, mean 3.2 mm). Every one of its 19 interior vertices turns in joint space by
  1.35°-18.28° (median 6.07°). This is IK/Cartesian-interpolation jitter, not a planning defect;
  §13.8's re-check already certified this polyline.
- **The dips.** G-PATH forbids corner blending, so the joint velocity changes direction within one
  sample at each vertex, and G-ACC (1.0 rad/s² × 0.95) forces Step 3 to slow down there. Interior
  below-band runs (outside the leading/trailing ramps), by `v_c`:

  | `v_c` (m/s) | interior dips | deepest (`s/v_c`) | longest dip (mm) | share of leg arc | all vertex-tied |
  |---|---|---|---|---|---|
  | 0.002 | 0 | — | — | 0 | — |
  | 0.005 | 2 | 0.899 | 0.06 | 0.2% | yes |
  | 0.006 | 4 | 0.809 | 0.08 | 0.4% | yes |
  | 0.008 | 5 | 0.728 | 0.18 | 1.0% | yes |
  | 0.009 | 10 | 0.478 | 0.26 | 2.6% | yes |
  | 0.010 | 10 | 0.478 | 0.35 | 3.4% | yes |
  | 0.012 | 13 | 0.430 | 0.55 | 6.0% | yes |
  | 0.015 | 14 | 0.387 | 0.86 | 11.2% | yes |
  | 0.020 | 15 | 0.254 | 1.60 | 28.3% | yes |
  | 0.030 | 11 | 0.185 | 8.05 | 69.1% | yes |
  | 0.050 | whole leg below band (one 64 mm "ramp") | | | | |

  At 0.005 m/s the two dips are samples 7007-7008 and 7919-7920 of that trial's output (D0012
  quoted 479-480/1391-1392 relative to its own leg range). Every dip at every `v_c` contains a
  segment adjacent to a turning plan vertex. Rule 1 (above band) never binds: `max s_i / v_c =
  1.0009`.
- **Why the old rule refused.** Old rule 2 refused any interior below-band segment, so only
  `v_c ≤ ~0.002` passed, below `min_tool_speed_m_s = 0.005`. D0012 found that raising
  `speed_tolerance` to 0.09/0.10 still failed and 0.12 passed. Those dips are as deep as 0.48·v_c,
  so tolerance alone would have to widen the band to roughly ±50% to absorb them, and the claim
  would mean little.

### Decision: option (b), a G-TOOLSPEED amendment with explicit bounds

Option (a), profiling through a vertex without slowing down, is not available without leaving the
plan polyline: G-PATH forbids blending, and the discrete G-ACC check sees the direction change at
the vertex sample. Option (c), changing §16.7 values, was rejected: `speed_tolerance` would have to
roughly double, loosening the band everywhere rather than only at vertices.

G-TOOLSPEED rule 2 now also permits a **vertex-tied dip**: an interior maximal below-band run that
contains a segment ending or starting at the bit-exact copy of an interior plan vertex whose joint
direction changes (turn > 1e-9 rad, computed as `2·atan2(‖û−ŵ‖, ‖û+ŵ‖)`, which is accurate near zero
where `acos` of a rounded dot product is not). Two explicit bounds:

- **3a (per dip):** the dip's tool arc length is `≤ trace.max_ramp_m` (5 mm), the same bound as a
  ramp.
- **3b (per leg, new):** all vertex-tied dips of the leg together cover `≤ trace.speed_tolerance`
  (5%) of the leg's tool arc length. This reuses the existing number instead of adding a
  `retime.yaml` key. The claim becomes "within ±5% of `v_c`, and never above that band, over at
  least 95% of the leg (ramps aside); every slower stretch is ≤ 5 mm and sits at a plan vertex".
  Without 3b, rule 3a alone accepted the fixture at `v_c ≈ 0.027` m/s, where dips cover well over a
  quarter of the leg. Rule 3b keeps the bisection where dips stay local.

Not changed: `b601_dm_limits.yaml`, `limit_fraction.*`, `sampling.*`, every `trace.*` value,
G-LIMITS and G-SELFCHECK. There is no clamping and no uniform slowdown: `v_c` is still the largest
value the bisection finds. No plan configuration moves: G-PATH is untouched and every plan
configuration still appears bit-exact, in order. Dip depth is not bounded. A dip only ever makes
the tool slower than commanded, which is never a safety concern, and its extent is bounded by 3a/3b.
A below-band run that touches no turning vertex is still refused, as is any segment above the band.

### Result

With the committed configs, `retime_plan_doc` retimes the fixture with exit 0: trace leg 2 gets
`v_c = 0.011152` m/s (≥ 0.005), min segment speed 0.00136 m/s, max 0.011162 m/s, ramps 1.34 mm in
and 0.80 mm out. Peak velocity fraction is 0.663 and peak acceleration fraction 0.950. G-SELFCHECK
and `joint_trajectory.check_limits` at full limits with `execution.yaml`'s margin report zero
violations (`test_retime.py::test_retime_plan_doc_real_synthetic_fixture`).

### Independent checker

`src/crackvision/trajectory_check.py` C-TOOLSPEED (and C-REPORT's feasible-set test) applies the
same rules 2/3a/3b. It derives `V` from the plan's own polyline and the trajectory's bit-exact
points; it still does not import `crackvision_motion`. On the fixture's retimed trajectory it
computes `F = [0.010631, 0.011156]` m/s, which contains the retimer's `v_c`, and all nine checks
pass.

That run also exposed an unrelated C-REPORT disagreement. The checker's leg-range slack was
`1e-5` rad of polyline progress, and the fixture's trajectory has a point `9.4e-6` rad before the
retract→transit rest vertex. So the checker also gave that point to the transit leg (`[8491, …]`
vs the retimer's `[8492, …]`). The slack is now float noise (`1e-9`), matching §16.5's
"leg owning the segment the point lies on". Both fixes are covered by
`tests/test_trajectory_check_path.py`.

## Addendum (MOT-07.10): purpose "view_move" for view-plan transit legs

**Status:** accepted (MOT-07.10). Amends INTERFACES §11.2, §11.7, §16.3–§16.6; resolves §16.8's
named gap. Normative schema text lives in the newly appended `docs/INTERFACES.md` §17, in the same
append-only style this ADR's own body already used for §16 and the MOT-07.11 addendum above.

### Context

§16.8 named the gap directly: a `view_plan`'s `transit_leg` (home → the chosen camera pose, §13.14)
is exactly as untimed as a `cartesian_plan` leg, needs the same retiming algorithm (§16.2) to become
executable, but has no `paths3d` file to bind to. §11.2's `purpose` enum had only `"crack_task"`
(requires non-null `source.paths3d`) and `"test"` (refused unconditionally in `real` by `G-TRAJ`,
§11.7), so a retimed view move could not run in `real` under any existing `purpose` value. This card
settles that gap rather than deferring it to OPS-01, because OPS-01 ("clean launch → capture → plan
→ preview → controlled execution") needs an executable capture move: the wrist camera has to reach
its view pose for real, under the same commissioning/approval/confirmation discipline as a
crack-task move, or OPS-01 cannot exercise the real capture leg of its own workflow at all.

### Decision

Add `purpose: "view_move"` to §11.2's enum. A view_move trajectory carries `source.paths3d: null`
and a new `source.view_plan: {path, sha256}` binding to the `crackvision.view_plan/1` file (§13.14)
whose `transit_leg` it retimes. `G-TRAJ` treats `view_move` as schema-valid in every mode (never a
`warn`, since it is not `"test"`). `G-ELIGIBLE` gets a view_move-specific rule: an **integrity**
check (refuse in every mode) that the bound view-plan file exists, matches its recorded sha256,
parses via `load_view_plan`, has a non-null `chosen`/`transit_leg`, and that the trajectory's first/
last positions equal `transit_leg`'s endpoints to float tolerance; then an **eligibility** check
(warn in mock/dry, refuse in real) that the view plan's own recorded `limits`/`end_effector`/`scene`
shas match the trajectory's `limits_file.sha256`/`end_effector_config_sha256`/`scene_config_sha256`
— which `G-LIMITS-HASH`/`G-STALE-CONFIG` already bind to the live files, closing the provenance
chain the same way `G-ELIGIBLE` already closes it for a `crack_task` trajectory's `paths3d` binding.
Full field-level and gate-level text is §17 (`docs/INTERFACES.md`).

`retime_plan` gains a `--view-plan PATH` input, mutually exclusive with `--plan`, that always writes
`purpose: "view_move"` and retimes the view plan's single `transit_leg` as a one-leg plan through
the unchanged §16.2 algorithm (no trace legs, so G-TOOLSPEED is vacuous). `trajectory_check` learns
to read either a `cartesian_plan` or a `view_plan` as its `--plan` input, dispatching on the file's
own `schema` field, and runs the same checks against the view plan's one transit leg.

**Why a view-plan binding is enough provenance for a camera-positioning move.** `G-ELIGIBLE`'s job
for `crack_task` is to prove the trajectory traces a crack path that was actually planned,
collision-checked and marked eligible — the `paths3d` file is that proof, because it is the thing
`plan_crack_path`'s own re-check (§13.8) certified. A view move's analogous claim is narrower: it
only has to prove the trajectory ends at a camera pose that `plan_view`'s own FOV/reachability/
re-check selection (§13.14) certified, starting from the same `home` that selection assumed. The
`view_plan` file carries exactly that: a `chosen` candidate and a `transit_leg` that *is* the
certified `home → chosen pose` path, with the same `sources.{limits,end_effector,scene}` shas a
`cartesian_plan` carries. Binding to it and checking the trajectory's endpoints against
`transit_leg`'s own endpoints is therefore the same proof shape as `paths3d`, just pointed at a
different upstream artefact — not a weaker standard, and not a bypass of any commissioning,
e-stop, approval, start-state, collision or confirmation gate, every one of which still applies to
`view_move` unchanged (§17.2).

**Alternatives considered and rejected:**

- **Reuse `purpose: "test"`.** Rejected: `G-TRAJ` refuses `"test"` unconditionally in `real`
  (§11.7), by design, because `"test"` carries no provenance claim at all. A view move legitimately
  needs to run in `real` (it is how the camera gets into position for a real capture), so it cannot
  share a purpose value whose entire point is "never trust this in real".
- **A synthetic `paths3d` binding** (wrap the view plan's `transit_leg` in a fabricated
  `crackvision.paths3d/1` document just to satisfy `crack_task`'s existing shape). Rejected: a
  `paths3d` file's `execution_eligible`/`ineligible_reasons` and `components[]` encode a crack-trace
  planning claim (lifting policy, per-segment uncertainty, §10) that a view move never makes; writing
  one anyway would be a fabricated artefact claiming a review process (§13.8's crack-path re-check)
  that never ran over this data, exactly the kind of "measured-looking but not actually verified"
  record this project's `value_status`/provenance discipline exists to prevent (ADR-014 §8.1's
  reasoning, cited already in this ADR's main Decision).
- **Defer to OPS-01.** Rejected per the card body: OPS-01 needs an executable capture move as an
  input, not a decision to make itself, and OPS-01's own card scope is the operator workflow, not a
  gate-catalogue amendment. Settling the `purpose` gap inside a motion-stack card (this one) keeps
  gate changes in the same place §11/§16 already live, rather than splitting gate semantics across a
  workflow card.

### Consequences

**Makes easy:** a real-mode view move can be retimed, checked and executed through the exact same
gate catalogue, commissioning record and confirmation flow as a crack-task move, so OPS-01 composes
the capture step from existing, already-reviewed machinery instead of a parallel, weaker path.

**Costs:** `retime_plan`, `trajectory_check` and the gate catalogue each pick up one more purpose/
schema branch to maintain and keep in sync with any future §16/§17 amendment, the same bookkeeping
cost §16.8 already flagged when it deferred this decision.

### Revisit when

- A future view-plan shape adds more than one leg (e.g. a multi-pose scan), at which point §17.3's
  "one-leg plan" simplification needs revisiting.

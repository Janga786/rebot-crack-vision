# ADR-017: Cartesian crack-path planning — candidate search, partial-trace recovery and the view phase

**Status:** accepted · **Date:** 2026-10-10 · **Supersedes:** — · **Superseded by:** —

## Context

MOT-06.1 (`docs/motion/CARTESIAN_PROBE.md`) probed `/compute_cartesian_path` and `/plan_kinematic_path`
against the MOT-02 headless mock stack for the `tool_tip`/`camera_link` task frames ADR-014 defines. This
card turns those facts into the normative contract MOT-06.3–.6 implement: how a §10 `paths3d` file's
per-segment waypoints become an untimed, continuous, collision-checked joint path (REQ-MOT-2), on the real
B601-DM model, limits and collision scene (REQ-MOT-1). The output is deliberately **untimed** — MOT-07 owns
retiming into the ADR-016/§11 `joint_trajectory` contract. Nothing here produces anything `execute_trajectory`
can run directly.

Five MOT-06.1 facts shape the design below, each cited where it applies:

1. **Wrist singularity at exact "straight down, roll = 0."** `/compute_ik` returns `NO_IK_SOLUTION`
   (error `-31`) for `tool_tip` pointed exactly straight down with zero roll, for every seed tried (22
   trials), independent of collision checking. A roll a few degrees off succeeds immediately.
2. **`fraction` is a path-length fraction of the *requested* polyline, not a count of the *returned*
   trajectory's points**, and the returned point count has no fixed relationship to the input waypoint
   count (dense input gets resampled/consolidated at `max_step`; adding rotation inflates it further even
   at identical translation).
3. **Collision truncation is silent on the error code.** `/compute_cartesian_path` with
   `avoid_collisions: true` returns `SUCCESS` (not an error) with `fraction < 1` when a waypoint collides;
   the service gives no field saying *why* it stopped early.
4. **A roll change between waypoints (a ramp) measurably inflates trajectory size/duration** even when the
   Cartesian translation is unchanged (18→36 points, 1.63s→3.45s for one test line); a *uniform* per-segment
   roll offset does not introduce new waypoint-to-waypoint rotation beyond what §10.5's own orientation fill
   already carries.
5. **`/plan_kinematic_path` accepts an explicit `start_state` plus a joint-constraint goal** — exactly the
   "transit to an IK-solved approach pose" shape this card needs — and both `jump_threshold` (legacy
   scalar) and the split `prismatic_jump_threshold`/`revolute_jump_threshold` fields exist on this Humble
   install, but the probe never exercised their *numeric* effect, only their presence.

## Decision

Normative schemas live in `docs/INTERFACES.md` §13 (`crackvision.cartesian_planning_config/1`,
`crackvision.cartesian_plan/1`, `crackvision.view_plan/1`). This section records why they look the way
they do; §13 is implemented verbatim.

### 1. Candidate generation: direction × a *uniform* roll offset (fact 4)

The candidate set is `{forward, reverse} × roll_offsets_deg` (default 12 values: `0, ±30, ±60, ±90, ±120,
±150, 180`, config-overridable). Each offset is a single rotation about the waypoint's own local `+X`,
applied identically to the approach, every trace waypoint and the retract pose of a segment — never ramped
across the segment. Fact 4 is exactly why: a ramped roll would reintroduce the size/duration inflation the
probe measured even though §10.5's own orientation fill already gives a smooth, singularity-aware
per-waypoint frame. A uniform offset only ever needs to escape a *bad* orientation (a singularity, fact 1,
or a collision), not re-shape the path.

Candidates are enumerated, and ties are broken, in one fixed order: roll magnitude ascending (`0, 30, 60,
90, 120, 150, 180`), positive before negative at equal magnitude (skipped where there is only one sign:
`0`, `180`), forward before reverse. This resolves the card body's two selection sentences ("pick the
first candidate … that achieves the whole trace" vs. "maximise the minimum joint margin … break ties on
smallest `|roll offset|`, then forward") into one rule: **every** candidate in this set is attempted (the
margin comparison needs every full completion, so there is no early stop), and among those that fully
complete, the arg-max on minimum joint margin wins; the stated order is exactly the tie-break, so "first in
order" and "best margin, ties broken by order" agree whenever margins are not strictly separated.

Because the roll offset rotates about `tool_tip`'s local `+X` — the pointing axis — it never changes
`position_m`, only `quat_xyzw` (ADR-014 §2, §10.5 point 2). This matters for §4 below.

### 2. Fact 1 is handled by the candidate set itself, not a special case

Zero roll offset is in the default set and is tried, but the exact-singularity pose fact-1 describes is a
property of a *specific* orientation (boresight exactly world-down, roll exactly 0 about it), not of "roll
offset = 0" in general — §10.5's own orientation fill is almost never exactly that pose for a real crack
normal. No special-casing is added: if a particular segment's roll-offset-0 candidate does land on or near
a singularity, its IK step fails (`ik_failed`) and the search simply continues through the other eleven
offsets and the reverse direction, exactly as it would for a collision or any other infeasible candidate.
This is cheaper and more robust than detecting "is this pose near a singularity" directly, and fact 1 is
precisely the evidence that nearby orientations are feasible.

### 3. Last-reached waypoint: FK matching, not `fraction` (facts 2, 3)

CARTESIAN_PROBE.md's own consequence 2 recommends a fraction-based estimate for this —
`round(fraction * (len(requested_waypoints) - 1))` against the *requested* waypoints — and rejects only
indexing or counting the *returned* `RobotTrajectory` directly, whose point count has no fixed
relationship to the input waypoint count (consequence 3: resampling at `max_step` consolidates or
inflates it independent of which input waypoints were actually reached). This card does not adopt the
probe's `round(...)` estimate as its primary method; it uses **FK matching** instead. Rationale:
`round(fraction*(n-1))` assumes the server's resampling is spatially uniform along the requested
polyline, and consequence 3 (plus fact 4's roll-ramp point-count inflation) is exactly the evidence that
it is not — a candidate's own roll offset changes the returned trajectory's point count and timing
without changing which Cartesian positions were requested, so a linear-proportion index is not a safe
read of "how far did we get." FK matching instead verifies, pose by pose, which requested waypoints were
actually reached, reusing the same tolerance (`recheck.fk_tolerance_m`/`_deg`) the independent re-check
(§4 below) already requires: walk the returned trajectory's configurations in order, and walk the
requested trace waypoints with a pointer starting at 0; a configuration advances the pointer when its
`/compute_fk` pose at `tool_tip` is within that tolerance of the pointed-at waypoint. The last waypoint
index the pointer reached, minus one, is the last-reached index; `-1` if none matched. This is
well-defined because `/compute_cartesian_path`'s output only ever progresses forward along the requested
polyline (fact 2's own resampling never reorders it). `fraction` itself still gates whether this search
runs at all — `fraction == 1.0` skips it entirely (§13.5/§13.6).

Fact 3 (no cause field on a short `fraction`) means a short trace cannot be labelled `collision` for free.
§13's classification is a deliberately bounded, best-effort post-hoc check: IK the next unreached waypoint
from the last-reached configuration; if that fails, the cause stays the generic `cartesian_incomplete`; if
it succeeds, `/check_state_validity` on it decides `collision`; if that passes, a joint-step comparison
against `max_joint_step_rad` decides `joint_jump`; otherwise it stays `cartesian_incomplete`. This is one
extra IK call and one extra validity call per shortfall, not a new planning pass, and it is honest about
what it cannot attribute.

### 4. Independent re-check never trusts the server's jump heuristic (fact 5)

Fact 5 confirms the jump-threshold fields exist but the probe never calibrated their numeric effect — they
are a dimensionless, vendor-internal heuristic with no documented unit this project has verified. Shipping
an untuned guess for a safety-relevant discontinuity check would be exactly the kind of "measured-looking
but actually a guess" value this project's `value_status` convention exists to prevent (ADR-014 §8.1).
`jump_threshold`/`prismatic_jump_threshold`/`revolute_jump_threshold` therefore default to **disabled
(`0.0`)**, and jump detection is instead done entirely by §13's own `max_joint_step_rad`, applied to every
consecutive output-configuration pair by the independent re-check — a value in explicit radians this ADR
can reason about directly. Default **0.2 rad (~11.5°)**: comfortably above ordinary per-point joint motion
at the default `max_step_m` (0.001 m, itself chosen finer than `paths3d`'s own default waypoint spacing of
0.002 m, §10.4, so the Cartesian interpolation never coarsens an already-resampled trace) away from a
singularity, and comfortably below the kind of jump a true wrist-singularity discontinuity (fact 1) would
actually produce. The FK tolerance (1 mm / 1°) and the inclusive `b601_dm_limits.yaml` position bound
(never narrowed, exactly ADR-016 §6's rule) complete the independent re-check, together with
`/check_state_validity` against the production scene for every output configuration.

### 5. Surface/scene mismatch relaxes, rather than drops, the check against the surface the crack is on

The pre-planning pure check (card body) flags a trace position that lies inside or within clearance of a
scene collision object — but every trace waypoint is *by construction* exactly `trace_clearance_m` from the
specimen's surface (§10.5 point 2), which is the object the crack is on. Checking that object against the
*full* `clearance_m` bound would flag every trace waypoint, always, which is not what the card intends (it
exists to catch *other* objects: the table, a future obstacle, a mis-placed specimen) — but dropping the
surface object from the check entirely would also lose the one case a mis-placed specimen actually
produces: a measured crack surface that ends up inside, or barely clearing, the scene's specimen box
because that box's pose is wrong (`config/scene/scene.yaml`'s own specimen entry is explicitly a
placeholder today). §13 therefore keeps checking `surface_object_id` (default `"specimen"`), but against a
*relaxed* bound: `clearance_m − surface_object_tolerance_m` instead of the full `clearance_m`. Every other
scene object keeps the full `clearance_m` bound, unchanged.

`surface_object_tolerance_m` defaults to **0.002 m (2 mm)** — one order of magnitude above the kind of
noise this check must absorb for a geometrically-consistent scene (the standoff construction's own
floating-point arithmetic, §10.5 point 2, plus the axis-aligned box approximation of the specimen's exact
shape), so a correctly-placed trace waypoint (distance ≈ `clearance_m`, exactly, modulo that noise) never
false-positives; and at least one order of magnitude below the scale of a real specimen-placement error
this check exists to catch (the placement pipeline's own tolerances — `specimen_placement.yaml`'s
`tolerance_m: 0.02`, i.e. 20 mm — already treat sub-centimetre deviations as noise, so a specimen pose
wrong enough to matter moves the surface by centimetres, not millimetres). This is not forced by a
MOT-06.1 fact; it is a necessary clarification to keep the check internally consistent with §10.5's own
standoff definition while still catching a genuinely mis-placed surface, recorded here as the card
instructs.

### 6. View-phase geometry reuses existing project conventions, not new ones

The view pose's tilt/azimuth construction reuses `reachability.yaml`'s own tilt-about-a-horizontal-axis
convention (§7.1 `orientation.tilt_deg`/`tilt_azimuth_samples`) rather than inventing a new one: the optical
axis is the world-down direction rotated by `tilt_deg` about the horizontal axis at azimuth `yaw_deg`; the
resulting frame's `+Z`/`+Y` are completed with exactly §10.5's `+Y = +Z × +X` right-handed construction.
Position is always `view_distance_m` directly above the centre regardless of tilt — matching §8.3's own
description of `recommend_placement --emit-view-config`'s check, which this view phase subsumes for the
general (post-placement) case. Default `yaw_candidates_deg` (4 cardinal azimuths) matches
`reachability.yaml`'s own `tilt_azimuth_samples: 4`. Candidate selection is tilt ascending then yaw
ascending, first full success (reachable + FOV-clean) wins — preferring the simplest (smallest-tilt)
view when it already works, consistent with ADR-014 §8.2's 15° anti-normal bound being the tilt band's own
upper edge, not a target to reach for. FOV numbers (87° × 58° H×V) are the D405 public datasheet figures
(58° vertical already cited by ADR-014 §8.2/INTERFACES §8.2); §13 carries them with an explicit `source`
exactly like every other nominal, not-yet-independently-reverified value in this project's config files.

### 7. Safety invariants

- Collision checking is **never** relaxed: `avoid_collisions` is hardcoded `true` in code and is not a
  config key at all (an operator cannot even accidentally set it `false` — the strict loader would reject
  the unknown key).
- Clearances (`trace_clearance_m`, `approach_retract_offset_m` from the input `paths3d` file, §8.2/§10)
  are read, never recomputed smaller to "find" more coverage.
- Nothing is dropped silently: every point, segment, candidate and split outcome that does not plan is
  named in `reasons[]`/`ineligible`-style accounting, mirroring `paths3d`'s own `counts.invalid_by_reason`
  (§10.1).
- The plan copies the input `paths3d`'s `execution_eligible` in the exact §11.2 `source.paths3d` shape, so
  `MOT-07`/`G-ELIGIBLE` (§11.7) can keep refusing on it unchanged. Planning an ineligible `paths3d` file is
  explicitly allowed (sim/visualisation use), never silently upgraded to eligible.
- The plan is explicitly documented, in §13 and in its own `caveats[]`, as untimed geometric output: it is
  never executable directly, only via MOT-07's retiming into `joint_trajectory` (§11.2).

## Alternatives

- **Ramp the roll across a segment instead of a uniform offset.** Rejected (fact 4): measurably inflates
  trajectory size/duration for no benefit this project needs; §10.5's own fill already handles smooth
  per-waypoint orientation.
- **Index the returned trajectory by its own point count.** Rejected (fact 2/consequence 3): the point
  count has no fixed relationship to the requested waypoint count, so an index into it is not comparable
  to a requested-waypoint index at all.
- **Adopt the probe's own `round(fraction*(n-1))` estimate (consequence 2) as the last-reached-waypoint
  method, instead of FK matching.** Rejected (§3 above): the probe's own data (consequence 3, fact 4)
  shows the server's resampling is not spatially uniform along the requested polyline, so a linear
  proportion of `fraction` is not a reliable read of which waypoints were actually reached; FK matching
  verifies this directly, at the same tolerance the independent re-check already requires.
- **Trust MoveIt's `jump_threshold`/`revolute_jump_threshold` numeric defaults for discontinuity
  detection.** Rejected (fact 5): their numeric effect was never calibrated by the probe, which only
  confirmed the fields exist. An explicit, radian-denominated independent re-check is used instead.
- **Apply the full `clearance_m` surface/scene mismatch bound to the specimen, unmodified.** Rejected
  (§Decision 5): every trace waypoint is, by design, standoffed exactly `trace_clearance_m` from the
  specimen, so an unmodified check would always fire and catch nothing real.
- **Exclude the specimen from the surface/scene mismatch check entirely.** Rejected (§Decision 5): it
  loses the one case that matters most — a mis-placed specimen whose surface itself ends up inside or
  too close to the scene's specimen collision box — leaving that case to surface only later as a generic
  `cartesian_incomplete`/`collision` reason instead of the diagnostic `surface_scene_mismatch`.
- **Attribute every `fraction < 1` shortfall uniformly to `cartesian_incomplete`, skipping the
  IK/validity/joint-step classification.** Rejected: it would make every collision-truncated trace
  indistinguishable from every other failure mode in `reasons[]`, which the card's reason vocabulary
  explicitly treats as separate (`collision`, `joint_jump`, `cartesian_incomplete`).
- **Give `plan_view` a `--start-joints` flag like `plan_crack_path`.** Rejected: the card's own CLI flag
  list omits it; the view phase always transits from the SRDF all-zero `home`, independent of any
  in-progress crack-path plan.

## Consequences

**Makes easy:**
- MOT-06.3–.6 implement against one frozen schema (§13) instead of re-deriving candidate order, partial-trace
  recovery or the re-check from the card body each time.
- A plan file is fully self-describing (echoes its own config, every source sha) so MOT-07 and a human
  reviewer never have to re-load `cartesian_planning.yaml` to know what produced a given plan.
- Partial coverage is a first-class, inspectable outcome (`coverage`, per-segment `status`/`reasons[]`),
  not a silent truncation — useful for iterating on a placement or a crack mask before hardware exists.

**Costs:**
- The candidate search is combinatorial (up to 24 candidates per segment, more with splits): planning time
  scales with crack complexity. Segment ordering is explicitly not optimised (future work, stated in §13).
- The `cartesian_incomplete`/`collision`/`joint_jump` classification is a best-effort heuristic (one extra
  IK + validity call), not a ground-truth cause from the service; it can occasionally mislabel an edge case
  the probe didn't cover.
- `jump_threshold` being disabled by default means the server-side heuristic contributes nothing; all jump
  protection is this project's own `max_joint_step_rad`, which MOT-06.3+ may need to retune once real
  crack geometry is exercised.

## Revisit when

- MOT-06.1's probe, or a future one, actually calibrates `jump_threshold`/`revolute_jump_threshold`'s
  numeric effect on this install — at that point enabling it alongside (not instead of) the independent
  `max_joint_step_rad` check could be reconsidered.
- The D405 horizontal FOV figure (87°) gets an in-repo, device-verified source (e.g. a GEOM-05 artefact),
  superseding the datasheet-only provenance §13 ships with.
- Real crack geometry (post-GEOM-08 on real captures) shows the default roll-offset set or
  `max_splits_per_segment` under- or over-covers in practice.

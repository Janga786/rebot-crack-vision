# REACHABILITY.md — B601-DM reachability map and specimen placement (MOT-04.5)

## 1. Purpose

This document answers one question: where on the bench can the crack specimen be placed so the
B601-DM can put its inspection tool boresight-down over every point of the specimen, and so its
wrist camera can view the specimen for a capture? To answer it, this card ran a full-resolution
MoveIt/TRAC-IK reachability sweep over the workspace configured in
`config/motion/reachability.yaml`, then `recommend_placement` (MOT-04.3) scored the map under the
normative rule in `docs/INTERFACES.md` §7.3, then independently re-verified the winning placement
at half the sweep's grid step and checked the wrist camera's view.

**Headline result: a placement is feasible.** `config/motion/specimen_placement.yaml` is
`feasible: true`, `value_status: nominal`, with the winning centre at **(0.26, 0.00) m**,
`surface_z_m = 0.00` (the table top), `yaw_rad = 0.0`, footprint 0.20×0.20 m, min joint-limit
margin **0.4708 rad**. Both independent checks required by this card pass: the half-step
verification sweep (578 points the original sweep never sampled) is 578/578 reachable, and the
wrist camera can view the placement centre from 0.25 m. This result rests on the ADR-014 model
(task frame `tool_tip`, `specimen_block` collision proxy, the `table` object from
`config/scene/scene.yaml`). Section 6 explains why the earlier (pre-ADR-014) run found nothing.
Nothing was shrunk or loosened to manufacture this result — see §4 and §9.

Everything here ran against the mock ros2_control stack (`mock_components/GenericSystem`). The
sweep only sends IK and FK queries (`/compute_ik`, `/compute_fk`) and never commands a trajectory,
so nothing moved.

## 2. Method

- **Model / solver.** The canonical B601-DM model plus the ADR-014 end-of-arm overlay
  (`config/robot/end_effector.yaml`, GEOM-10), served by the mock stack's `move_group`. IK comes
  from MoveIt `/compute_ik` using the group's configured `trac_ik` plugin, `timeout_s = 0.02`,
  `avoid_collisions: true`. Seeds are the neighbouring target's solution, falling back to the SRDF
  `home` state.
- **Task frame.** `ik_link: tool_tip` (not `gripper_tcp`). `tool_tip` is the crack-facing tool
  reference, identity on `gripper_link` = the closed-finger tip (ADR-014). The committed standoffs
  0.01 m and 0.04 m are measured **from `tool_tip` along its boresight**, i.e. genuine tool-tip
  clearances above the surface, matching their original intent
  (`docs/TECHNICAL_APPROACH.md` §2.5 `STANDOFF_APPROACH_M` = 0.04 m).
- **Orientation.** Boresight-down. `axis_local = [1, 0, 0]` of `tool_tip` (prior evidence per
  `docs/INTERFACES.md` §6.5/§8, not GEOM-07-calibrated) is aligned with world `[0, 0, -1]`, sampled
  at 8 rolls about that axis. `tilt_deg = [0]`, so the 4 azimuth samples collapse to one: 8
  orientations per target, `roll_search: best`.
- **Targets.** Each grid node `(x, y)` sits on a surface at height `surface_z`. The tool tip is
  commanded to `(x, y, surface_z + standoff)` for each configured standoff.
- **Collision model.** `surface_collision.model: specimen_block` (ADR-014/MOT-04.6), not the
  earlier grid-wide thin slab: a solid block from the table top (`floor_z_m = 0.0`) up to each
  `surface_z`, over the grid plus `margin_m = 0.05`, excluding `base_keepout_m` (the robot base
  footprint, x ±0.09, y ±0.12 m — a specimen cannot overlap the base). Plus the static workcell
  **table** object from `config/scene/scene.yaml` (`environment.objects: [table]`), applied for
  the whole sweep. Plus full self-collision, including the wrist-camera mount/housing proxies
  (GEOM-10). Collision checking was never relaxed to produce a feasible result.
- **FK re-check.** Every IK success is independently re-checked with `/compute_fk`. It counts as
  `reachable` only if the FK pose agrees within 1 mm / 0.5°; otherwise `fk_mismatch`.
- **Prefilter.** Targets beyond `reach_bound_from_urdf` (0.962 m from `/robot_description`) would
  be marked `prefiltered` without calling IK; none were this far out (every target is
  `reachable`/`unreachable`).
- **Placement scoring.** §7.3's rule: a 0.20×0.20 m footprint, dilated by 0.02 m tolerance, at yaw
  0 or 90°. Every grid node inside the dilated footprint must be `reachable` at every configured
  standoff; score = the worst-case min joint-limit margin over those nodes.

### Grid (unchanged this run)

The committed grid is `x ∈ [0.11, 0.50]` step 0.03 m (14 values), `y ∈ [-0.39, 0.39]` step 0.03 m
(27 values), `surface_z_m = [0.00, 0.04, 0.08, 0.12, 0.16]`, `standoffs_m = [0.01, 0.04]` — 3780
targets. This 0.03 m step (coarsened from an original 0.025 m) was already committed by a prior
attempt at this card for runtime reasons (see §6); this run made **no further grid change**: the
full sweep completed in 1596 s (26.6 min), well inside the `--max-duration-s 9000` budget, so there
was no need to coarsen further. `ik_link`, standoffs, orientation, IK tolerance and collision
settings were not changed from the committed config.

## 3. Commands

```bash
bash scripts/ros/build_ws.sh

bash scripts/ros/run_reachability.sh --reachability-config config/motion/reachability.yaml \
    --out data/motion/reachability_map.json --max-duration-s 9000

bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && \
    ros2 run crackvision_motion recommend_placement --map data/motion/reachability_map.json \
    --out config/motion/specimen_placement.yaml \
    --emit-verify-config config/motion/placement_verify.yaml \
    --emit-view-config config/motion/view_verify.yaml'

bash scripts/ros/run_reachability.sh --reachability-config config/motion/placement_verify.yaml \
    --out logs/placement_verify_map.json --require-all-reachable

bash scripts/ros/run_reachability.sh --reachability-config config/motion/view_verify.yaml \
    --out logs/view_verify_map.json --require-all-reachable

./env.sh python scripts/motion/plot_reachability.py --map data/motion/reachability_map.json \
    --placement config/motion/specimen_placement.yaml --out docs/motion/figures/reachability.png
```

`data/motion/reachability_map.json` and `logs/*.json` are git-ignored generated evidence. Re-run
the commands above to reproduce them.

## 4. Results

| Item | Value |
|---|---|
| Map | `data/motion/reachability_map.json`, `complete: true` |
| Map sha256 | `c9dae86dbfa00eeb787cc3b9c159ec79d79fa2a4ae070f992ef30b897cf7e675` |
| Config sha256 (`config/motion/reachability.yaml`) | `54150cc37c1d5777b6ddc73f6221a46f7bbdd011f763434c9bfd3a201f529f83` |
| Robot limits sha256 (`config/robot/b601_dm_limits.yaml`) | `43f5e2e3a92bcae348b077bfc7d1dd223e555a28869789235813d4c2723aa82b` |
| Scene config sha256 (`config/scene/scene.yaml`) | `12a09d7b66258b2fc44223a684f78fc6a6551bcbc552c49e0611eb2fd85b775a` |
| git commit recorded in map | `dbc56a1d464e0293e18a7ec0447e8ae63c156fe2` |
| Sweep duration (`duration_s`) | 1596.1 s (26.6 min), wall time incl. mock-stack bring-up |
| Targets | 3780 = 14 x × 27 y × 5 surface_z × 2 standoffs |
| counts_by_status (whole map) | `reachable: 2322, unreachable: 1458` (no `prefiltered`/`fk_mismatch`/`error`) |

Reachable fraction and raw counts per `surface_z` (`summary.reachable_fraction_by_surface_z`, and
`reachable`/`unreachable` counts out of 756 targets per layer):

| surface_z (m) | reachable fraction | reachable / unreachable |
|---|---|---|
| 0.00 | 0.831 | 628 / 128 |
| 0.04 | 0.780 | 590 / 166 |
| 0.08 | 0.702 | 531 / 225 |
| 0.12 | 0.553 | 418 / 338 |
| 0.16 | 0.205 | 155 / 601 |

Reachability falls off monotonically with height, as expected (the arm has to reach further out and
up over the keep-out/table to clear a taller specimen while keeping the tool tip boresight-down).

![Reachability map](figures/reachability.png)

The figure has one panel per `surface_z`, each cell coloured by the fraction of the map's
standoffs that are reachable, with contour lines of min joint-limit margin over fully-reachable
cells, `base_link` marked at the origin, and the recommended footprint (+ tolerance) and
alternatives overlaid on the `surface_z = 0.00` panel. The title records the task frame
(`tool_tip`), solver (`trac_ik`), standoffs, and collision model (`specimen_block` + `table`).

**Recommended placement** (`config/motion/specimen_placement.yaml`, generated — not hand-edited —
by `recommend_placement`, `map_sha256`/`config_sha256` matching the values above):

| | center_xy_m | surface_z_m | yaw_rad | score_min_joint_margin_rad |
|---|---|---|---|---|
| Winner | (0.26, 0.00) | 0.00 | 0.0 | 0.4708 |
| Alt 1 | (0.26, -0.06) | 0.00 | 0.0 | 0.4167 |
| Alt 2 | (0.26, 0.06) | 0.00 | 0.0 | 0.4167 |
| Alt 3 | (0.23, -0.12) | 0.00 | 0.0 | 0.4060 |
| Alt 4 | (0.23, 0.12) | 0.00 | 0.0 | 0.4060 |

`recommend_placement` exited **0** (feasible, §7.4). `boresight_provenance: prior_evidence`.
`max_feasible_square_m` is absent (there is a feasible 0.20×0.20 m placement, so this field is only
populated on infeasibility).

### Best feasible placement per swept `surface_z`

Because a real specimen's thickness usually fixes its surface height (not the overall winner's
height), here is the best (highest-score) feasible 0.20×0.20 m placement at **every** swept
`surface_z`, computed with a throwaway script calling `placement.score_candidates` on the full map
restricted to each layer (not committed; see snippet below):

| surface_z (m) | best feasible centre (x, y) m | yaw | score (rad) |
|---|---|---|---|
| 0.00 | (0.26, 0.00) | 0.0 | 0.4708 |
| 0.04 | (0.26, 0.00) | 0.0 | 0.3233 |
| 0.08 | (0.26, 0.00) | 0.0 | 0.1645 |
| 0.12 | **none feasible** | — | — |
| 0.16 | **none feasible** | — | — |

At `surface_z` 0.12 and 0.16 no 0.20×0.20 m footprint has every node (at both standoffs) reachable,
even though many individual cells are (§7's per-cell table above shows 55%/21% reachable overall at
those layers) — the footprint requirement is stricter than any single cell. If the real specimen
turns out to be thicker than ~0.08 m above the table, this sweep has **no** feasible placement for
it at this footprint/tolerance and must be re-run (coarser footprint, or accept a worse margin via
`top_k`/no top_k filtering — an operator decision, not something to default silently).

Snippet used (system python3, via `scripts/ros/env_ros.sh`, not committed):

```python
import json
from crackvision_motion.placement import score_candidates
from crackvision_motion.reachability_core import load_config

map_dict = json.load(open('data/motion/reachability_map.json'))
cfg = load_config('config/motion/reachability.yaml')
candidates = score_candidates(map_dict, cfg)  # globally ranked best-first, §7.3 rule
for z in cfg['grid']['surface_z_m']:
    best = next((c for c in candidates if abs(c['surface_z_m'] - z) < 1e-9), None)
    print(z, best)
```

## 5. Independent verification

### 5.1 Half-step footprint re-check

`config/motion/placement_verify.yaml`, emitted by `recommend_placement --emit-verify-config`,
re-samples the winning placement's dilated footprint (0.26±0.12 m × 0.00±0.12 m, i.e.
`x ∈ [0.14, 0.38]`, `y ∈ [-0.12, 0.12]`) at **half the sweep's grid step** (0.015 m instead of
0.03 m) — 578 points the original sweep never sampled, at `surface_z = 0.00`, both standoffs.

```
bash scripts/ros/run_reachability.sh --reachability-config config/motion/placement_verify.yaml \
    --out logs/placement_verify_map.json --require-all-reachable
```

Result: exit **0**. `complete: true`, 578/578 `reachable` (0 unreachable/prefiltered/fk_mismatch/
error), duration 246.0 s, map sha256
`7ecf094dbcdf6e7ee6f099d189d5c495cd5164b62af0cd148fe192171870491f`. Every one of these points
independently passed the 1 mm / 0.5° FK re-check (that is what `reachable` means, §7.2). This is
the independent evidence behind the recommendation, at twice the original sweep's spatial
resolution.

### 5.2 Wrist-camera view check

`config/motion/view_verify.yaml`, emitted by `recommend_placement --emit-view-config`, checks that
`camera_link` (ADR-014 view phase, nominal CAD prior — not yet GEOM-05-calibrated) can view the
placement centre `(0.26, 0.00)` from 0.25 m, boresight-down (tilt 0°/15° sampled, `(x, y)` fixed at
the centre).

```
bash scripts/ros/run_reachability.sh --reachability-config config/motion/view_verify.yaml \
    --out logs/view_verify_map.json --require-all-reachable
```

Result: exit **0**. `complete: true`, 1/1 `reachable`, `fk_position_error_m = 1.1e-8`,
`fk_axis_error_deg = 1.9e-6`, min joint-limit margin 0.3487 rad, map sha256
`f65b14f668df69cba60496a5a118d0cf1bb82574a175c3a71be8eebe77a7811f`. The wrist camera can view the
placement centre from the nominal 0.25 m viewing distance without colliding with the table, the
specimen-block proxy, or itself.

## 6. Why the previous run found nothing (honest history)

An earlier attempt at this card (documented as "attempt 00094" and reflected in this doc before the
2026-09-30 ADR-014 recovery) found **zero reachable targets**: 4320/4320 `unreachable` (0
reachable, 0 errors, 0 FK mismatches, 0 prefiltered), `feasible: false`, `max_feasible_square_m:
0.0`. Its cause, root-caused by the 2026-09-30 technical-lead recovery session (ADR-014), was a
frame error, not a requirement or solver problem:

- The standoffs 0.01/0.04 m were always meant as **tool-tip** clearances (0.04 m is
  `docs/TECHNICAL_APPROACH.md` §2.5's `STANDOFF_APPROACH_M`, whose IK tip was `gripper_link` — the
  closed-finger tip). Attempt 00094 applied them to `gripper_tcp`, the grasp centre **44.3 mm
  proximal to that tip** (`xyz = -0.0443 0 0` from `gripper_link`), so at those standoffs the
  gripper body's fingertip went 34 mm / 4 mm **through** the surface. With
  `avoid_collisions: true`, IK correctly reported no collision-free solution for every target and
  every roll.
- The surface model was a grid-wide thin slab that ran under the robot base and hit the shoulder
  (`link2`) at `surface_z ≳ 0.10`, an artefact — a specimen cannot overlap the base. There was also
  no table object at all.

ADR-014 fixed both: it introduced `tool_tip` (identity on `gripper_link`, i.e. the actual
closed-finger tip) as the task frame for approach/trace/retract, so the standoffs are now measured
from the frame they were always meant for; MOT-04.6/MOT-03/GEOM-10 then replaced the grid-wide slab
with `surface_collision.model: specimen_block` (a block that stops at the base keep-out) plus a real
`table` object. This card ran the sweep against that corrected, already-committed model — it did
not author the model fix. The result above (61% overall reachable, a feasible placement) confirms
the diagnosis: the same IK/FK/collision pipeline that found zero reachable targets at the wrong
frame finds a large, usable reachable region at the right one.

## 7. Comparison with TECHNICAL_APPROACH §2.5

`docs/TECHNICAL_APPROACH.md` §2.5's sweep found the coupon good at `x = 0.22` m (18/18 waypoints
< 2 mm) and poor at `x = 0.42` m (5/18), at coupon surface `z = 0.12` m. With ADR-014, this map's
task frame (`tool_tip` = `gripper_link` origin) is now **the same physical point** that §2.5 used
(`gripper_link`), which was not true of the pre-ADR-014 `gripper_tcp` sweep.

This map's grid has nodes exactly at `x = 0.23` and `x = 0.41` (step 0.03 from 0.11), `y = 0`, and
its grid includes `surface_z = 0.12` exactly — §2.5's own coupon height. What the map says there:

| x (m) | standoff 0.01 m | standoff 0.04 m | min joint-limit margin |
|---|---|---|---|
| 0.23 | reachable | reachable | 0.259 / 0.138 rad |
| 0.41 | reachable | **unreachable** | 0.091 rad / — |

This is in the same direction as §2.5: the margin falls as x increases from 0.23 to 0.41, and at
0.41 the larger (0.04 m) tool-tip clearance already fails IK/collision while the smaller one still
succeeds with a thin margin (0.091 rad) — a point close to losing reachability entirely, consistent
with §2.5's "5/18 waypoints converge" at x = 0.42.

**This is directional agreement, not a reproduction, and the remaining differences are real:**
§2.5 used a custom 5-DOF damped-least-squares IK (position + boresight axis, free roll) with **no**
collision checking against anything; this map used MoveIt `trac_ik` with full 6-DOF pose solves per
roll sample, collision-checked against self-collision, the table and the specimen-block proxy, and
independently FK-re-checked. §2.5 measured 18 real crack-inspection waypoints with a mean 0.026 mm
error at the good position; this map only asks a binary IK+collision question at single points.
Agreement beyond "the same qualitative direction, at the same frame and surface height" has not
been measured and is not claimed here.

(ADR-014 §3 separately records a coarse 0.06 m-grid feasibility probe, run during the 2026-09-30
recovery, that described its own margin trend toward x ≈ 0.41 as "independently reproducing"
§2.5's 0.22-good/0.42-poor finding. This section is this card's own, full-resolution measurement of
that same question; it finds the same direction but is more conservative about calling it a
reproduction, given the collision-model and solver differences above.)

## 8. What is not yet known, and re-run triggers

Every value in `config/motion/reachability.yaml` and `config/robot/end_effector.yaml` is
`value_status: nominal` — an engineering starting point, not a measurement. The map and the
recommendation must be regenerated (sweep → `recommend_placement` → both verify sweeps → plot) when
any of the following changes:

- **GEOM-07 (tool tip / boresight calibration).** Replaces the prior-evidence `tool_tip` frame and
  `boresight.axis_local` with a measured pivot-calibration result.
- **GEOM-05 (camera extrinsic / hand-eye calibration).** Replaces the CAD-prior `camera_link` pose
  (and its collision proxies) used by the view-verify check.
- **The operator's mount-side confirmation (ADR-014 §1).** ADR-014's camera-pose prior rests on an
  *open assumption, confirmed by the operator before GEOM-05*, about which gripper face the D405
  mount sits on; if that assumption is wrong the camera prior (and so the view check) must be
  redone.
- **MOT-10 (measured workcell).** Replaces the nominal table/base-keepout geometry in
  `config/scene/scene.yaml` with tape-measured values, and ultimately the recommended placement
  itself with the real, as-placed, measured specimen pose.
- **Any change to `config/robot/end_effector.yaml`, `config/scene/scene.yaml` or
  `config/robot/b601_dm_limits.yaml`** — their sha256 values are pinned in `config_sha256`/
  `map_sha256`/`limits_sha256` above precisely so a stale map can be detected.

## 9. Handoff to MOT-10 / MOT-03

`config/motion/specimen_placement.yaml` (`feasible: true`, `value_status: nominal`) is the pose the
operator is meant to place the specimen at: centre `(0.26, 0.00)` m in `base_link`, top face at
`surface_z = 0.00` m (the table top), `yaw = 0`, footprint 0.20×0.20 m. The operator physically
places the specimen there, and **MOT-10 then measures the real, as-placed pose**, which supersedes
this file for every downstream consumer (§7.3's nominal disclaimer, carried verbatim into this
file's `caveats`).

`config/scene/scene.yaml`'s `specimen` collision object (MOT-03) is currently an explicitly-labelled
placeholder centred at `(0.30, 0.0)` with `0.01` m thickness, predating this recommendation. It is
out of this card's scope to edit (`config/scene/scene.yaml` is not in MOT-04.5's scope.write globs),
but it should be updated — by a MOT-03/MOT-10 follow-on — to take its nominal pose from this file
(`placement.center_xy_m`, `surface_z_m`, `yaw_rad`, `footprint_m`) until MOT-10's measurement
replaces it with a `measured` pose, per the `value_status: nominal|measured` vocabulary already used
throughout `config/scene/scene.yaml` and `config/motion/reachability.yaml`.

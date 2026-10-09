# WORKCELL_SURVEY.md — physical workcell survey procedure (MOT-10.1)

**Status:** NORMATIVE for how the real workcell (table, specimen, clearance obstacles) is measured and
recorded before `config/scene/scene.yaml` (MOT-03, `docs/motion/SCENE.md`) is flipped from `nominal` to
`measured`. Defines the procedure and the raw record schema (`crackvision.workcell_survey/1`,
`docs/INTERFACES.md` §12). No code, no robot motion. `motion: false` — the arm stays powered off or at
rest throughout; nothing is jogged.

## 0. Purpose

`scene_core.assert_commissioning_ready` (`docs/motion/SCENE.md` §2) refuses any real-hardware run while a
single scene value is `nominal`. Today every table/specimen/ACM entry in `config/scene/scene.yaml` is
nominal — an engineering guess, not a measurement. This document tells an operator, with hand tools and no
robot motion, exactly what to measure, in what frame, with what uncertainty, and how those raw readings
turn into `scene.yaml`'s `measured` values. MOT-10.3 (a later card, not implemented by this one) is the
`survey_to_scene` tool that reads a filled-in survey record and performs that conversion.

## 1. Datum: `base_link` and its physical reference faces

Every measurement in this procedure is an offset from one of three physical faces of the robot's base,
expressed directly in `base_link` (the planning frame everywhere else in this codebase — §6.1/§7.2 of
`docs/INTERFACES.md`). `base_link`'s own origin is, by the convention already used in
`docs/motion/SCENE.md` §4 and `config/scene/scene.yaml`, **the bottom face of `base_link.STL`** — the
point where the base casting meets the table when the robot is bolted directly down. That means the datum
is something an operator standing at the cell can physically touch: the underside of the base, where it
meets the tabletop.

The reference faces and their `base_link` offsets come from the canonical URDF's `base_link.STL`
axis-aligned bounding box (`docs/motion/ROBOT_MODEL.md` §1: canonical model
`rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf`, mesh
`meshes_b601_gripper/base_link.STL`). The URDF's `<visual>`/`<collision>` origin for `base_link` is
`xyz="0 0 0" rpy="0 0 0"` (identity), so STL vertex coordinates already are `base_link` frame coordinates
— no extra transform is needed.

Throwaway measurement snippet (binary STL, no external STL library required):

```python
import struct

def read_binary_stl_bbox(path):
    with open(path, "rb") as f:
        f.read(80)
        (n_tri,) = struct.unpack("<I", f.read(4))
        xs, ys, zs = [], [], []
        for _ in range(n_tri):
            f.read(12)
            for _ in range(3):
                x, y, z = struct.unpack("<fff", f.read(12))
                xs.append(x); ys.append(y); zs.append(z)
            f.read(2)
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs)), n_tri

path = "~/rebot_ws/install/rebotarm_bringup/share/rebotarm_bringup/description/meshes_b601_gripper/base_link.STL"
print(read_binary_stl_bbox(path))
```

Run (`~/rebot_ws/install` is the read-only underlay named by `docs/motion/ROBOT_MODEL.md`):

```
$ ./env.sh python -I /tmp/stl_bbox.py \
    /home/boosterk1/rebot_ws/install/rebotarm_bringup/share/rebotarm_bringup/description/meshes_b601_gripper/base_link.STL
triangles=77972
x: [-0.070000, 0.070000]  extent=0.140000
y: [-0.100000, 0.100000]  extent=0.200000
z: [-0.000000, 0.082650]  extent=0.082650
```

Giving the three fixed reference faces (also recorded in `docs/INTERFACES.md` §12.1):

| Reference face | `base_link` coordinate | Physical meaning |
|---|---|---|
| **bottom face** (table contact) | `z = 0` | the plane where the base casting meets the table/adapter plate |
| **front face** (`+x`, the direction the arm reaches: `config/motion/reachability.yaml`'s grid is `x ∈ [0.05, 0.50]`, all positive) | `x = +0.070 m` | the base casting/plate face nearest the specimen side of the cell |
| **side faces** (`±y`) | `y = ±0.100 m` | the two base faces either side of the arm's reach fan |

**The operator must confirm, before measuring anything else, that these are genuinely the flat faces on
the real casting/plate they can place a square or tape against** — the bounding box is computed from the
CAD mesh, not re-verified against the physical part by this procedure. If the real base's front/side
faces are rounded, chamfered, or otherwise not flush with the bounding-box planes above, note that in
`survey.notes` and pick the nearest flat reference feature, recording exactly which feature was used.

## 2. Tools and expected uncertainty

| Tool | Used for | Typical resolution | Typical 1σ uncertainty |
|---|---|---|---|
| Steel tape measure (≥2 m) | table extents, obstacle distances | 1 mm | ±2 mm |
| Steel rule (≤300 mm) | specimen corner offsets, short obstacle dimensions | 0.5 mm | ±1 mm |
| Engineer's square | confirming/aligning reference faces before measuring from them | — | — |
| Dial or digital calipers | specimen thickness | 0.01 mm | ±0.05 mm |
| Spirit level | table top flatness/level near the specimen | 0.5 mm/m (bubble) | ±1 mm over the checked span |

Every reading recorded in the survey file (`docs/INTERFACES.md` §12.2, the "reading" map) carries its
own `instrument`, `resolution_m` and `uncertainty_1sigma_m` — use the table above as defaults, but record
the uncertainty actually believed for that specific reading (e.g. a tape measurement across an obstructed
span deserves a larger stated uncertainty than a clean, square one).

## 3. Measurement checklist

Work through `config/scene/workcell_survey.template.yaml`, filling in each `value_m` (and its
`instrument`/`resolution_m`/`uncertainty_1sigma_m`) as you go. Do not compute anything yourself — every
field in the template is a raw reading or a direct observation, never a `base_link` coordinate you derive
by hand. `survey_to_scene` (MOT-10.3) does the arithmetic from §5 below.

1. **Base mounting.** Confirm (and note) whether the base is bolted directly to the table or sits on an
   adapter plate. If there is a plate, measure its thickness with calipers or a rule.
2. **Table.** From the front reference face (`x = +0.070`), tape-measure in `+x` to the far table edge
   and (if reachable), from that *same* front face but in `-x`, to the near table edge — there is no
   separate "`-x` face" to measure from. From each side reference face (`y = ±0.100`), tape-measure in
   the outward direction to the corresponding table edge. Measure the table top's thickness (top face to
   underside) with a rule or calipers. Lay the spirit level near where the specimen sits and record the
   bubble/gap deviation and where you checked it.
3. **Specimen.** Pick a consistent corner order (e.g. clockwise from the corner nearest the base) and
   record it in `survey.notes`. For each of the 4 top corners, measure its offset from the front
   reference face (x) and from the `y = 0` centerline (y) with the steel rule. Caliper the specimen
   thickness at 3 or more distinct points (edges and middle are a good spread) and confirm by eye that it
   is resting on the table, not propped or overhanging.
4. **Clearance obstacles.** Decide and record the survey radius around `base_link`'s origin (the card's
   "stated radius" — pick a value that comfortably covers everything the arm's reach (~0.9 m,
   `docs/motion/ROBOT_MODEL.md`) could approach: walls, fixtures, cable runs, and specifically the camera
   USB lead routing back to the host). For each obstacle, tape-measure an axis-aligned box (min/max in x
   from the front face, min/max in y from the centerline, min/max in z from the table top) that safely
   contains it — err generous, not tight.
5. **ACM observations.** Record, as plain observations (not re-derivations), whether the base is bolted
   to the table and whether the specimen rests on the table — these mirror step 1/3 but are the explicit
   facts the allowed-collision-matrix entries in `config/scene/scene.yaml` rest on.
6. **Photos and notes.** Save photographs under `data/workcell_survey/` (git-ignored — never commit
   imagery) and list their paths in `survey.photos`. Use `survey.notes` for anything that doesn't fit a
   field: which corner-numbering convention you used, which physical feature you used as a reference face
   if not the CAD-flat one, anything obstructed or awkward to measure.

## 4. Filling in the template and validating it

Copy `config/scene/workcell_survey.template.yaml` to a working file (e.g.
`data/workcell_survey/survey_<date>.yaml`, git-ignored) and replace every `null` with your reading. Every
`reading` block (`{value_m, instrument, resolution_m, uncertainty_1sigma_m}`) must be fully filled in —
`survey_to_scene` refuses a file with any field still `null` (`docs/INTERFACES.md` §12.7, refusal rule 6).

`survey_to_scene` (console script `survey_to_scene`, delivered by **MOT-10.3**, not this card) is an
`rclpy`-side tool run via `scripts/ros/env_ros.sh` — like `reachability_sweep`/`recommend_placement`
(`docs/INTERFACES.md` §7.4) — never `./env.sh`:

```bash
# validate-only: check the file against crackvision.workcell_survey/1 and exit (0 valid, 2 invalid)
ros2 run crackvision_motion survey_to_scene --validate-survey data/workcell_survey/survey_<date>.yaml

# convert: derive measured scene objects from the survey and write them to --out (there is no
# default --out — survey_to_scene never overwrites config/scene/scene.yaml unless given explicitly)
ros2 run crackvision_motion survey_to_scene \
    --survey data/workcell_survey/survey_<date>.yaml \
    --out config/scene/scene.yaml \
    --emit-verify-config data/motion/survey_placement_verify.yaml \
    --emit-view-config data/motion/survey_view_verify.yaml
```

See `docs/motion/SCENE.md` §8 for the full workflow (including the `--check-scene` drift check) and
the exact exit codes (0 ok; 2 invalid/refused survey or config error; 3 missing survey/scene file; 1
drift detected under `--check-scene`, per `docs/INTERFACES.md` §0.2).

## 5. Derived-value rules (normative; see `docs/INTERFACES.md` §12.7 for the authoritative text)

`survey_to_scene` (MOT-10.3), not the operator, computes:

- the specimen's centre, yaw and footprint, by fitting a rectangle to the 4 measured corners and checking
  a non-trivial residual (the larger of the diagonal-length difference and the two opposite-side-length
  differences — refused if that residual exceeds `derivation_params.rectangularity_tolerance_m`, default
  3 mm; a 5 mm-skewed corner on a nominal 0.2 m square is refused at the default, see §12.7's worked
  example);
- the specimen's top `z`, as the table-top datum `z_table_top = -(adapter plate thickness)` (exactly `0`
  if bolted directly) plus the mean of the caliper thickness readings — the specimen rests on the table,
  which sits *below* `base_link` `z = 0` by the plate thickness (e.g. a 0.010 m plate and a 0.040 m mean
  thickness give `-0.010 + 0.040 = 0.030` m);
- the table's collision box: footprint from the front/side reference faces (§12.7 rule 3's exact sign
  conventions), thickness from `table.thickness`, placed so its top face sits at
  `z = -(adapter plate thickness)` — exactly `0` when the base is bolted directly to the table;
- each obstacle's collision box, from the same front-face/centerline/table-top references and sign
  conventions.

Every object `survey_to_scene` writes into `config/scene/scene.yaml` gets `value_status: measured` and a
`source` string naming the survey file's path and sha256 — never a bare "measured", so a reviewer can
trace every number back to the exact survey record that produced it.

## 6. Refusal conditions

`survey_to_scene` refuses to write anything (no partial scene update) if:

- the specimen corners' rectangularity residual (§5 above, §12.7 rule 1) exceeds
  `derivation_params.rectangularity_tolerance_m`;
- `specimen.resting_on_table` is `false`, or disagrees with `acm_observations.specimen_resting_on_table`;
- `acm_observations.base_bolted_to_table` disagrees with `base_mounting.bolted_directly_to_table`;
- any derived obstacle box overlaps the robot's base keep-out (`base_keepout_m`, `docs/INTERFACES.md` §8.3);
- fewer than 3 specimen thickness readings are present;
- any single reading anywhere in the file has a `null` `value_m`, `instrument`, `resolution_m` or
  `uncertainty_1sigma_m` — **except** `base_mounting.adapter_plate_thickness` when
  `bolted_directly_to_table` is `true` (it is treated as thickness `0` and is not required).

## 7. Out of scope

- **Camera and mount poses are NOT tape-measured by this procedure.** They come from hand-eye calibration
  (GEOM-05, `docs/adr/013-*.md`/`docs/adr/014-end-effector-frames-and-task-phases.md`), which is a
  separate, instrumented procedure.
- **The eye-in-hand wrist camera mount is robot geometry** (GEOM-10, `config/robot/end_effector.yaml`),
  not a workcell object — this survey never touches it.
- **The tool centre point (TCP)** is calibrated by GEOM-07, not measured here.
- This procedure is `motion: false`. The arm stays powered off or at rest for its entire duration; nothing
  is commanded, jogged, or planned against during the survey itself.

## 8. Re-measure triggers

Re-run this survey (producing a new, dated record, never silently editing an old one) whenever:

- the specimen is moved, replaced, or re-clamped;
- the robot base is unbolted and re-bolted (even to the same spot — fasteners can shift the seating);
- the table itself is changed, moved, releveled, or resurfaced;
- a new or relocated obstacle appears inside the surveyed radius (new fixture, re-routed cable, etc.).

A stale survey is worse than an honest `nominal` scene: `scene_core.assert_commissioning_ready` will
happily pass a `measured` scene that no longer matches reality, because it only checks `value_status`,
never freshness. Re-measuring on every trigger above is the operator's responsibility; nothing in this
pipeline can detect a moved specimen on its own.

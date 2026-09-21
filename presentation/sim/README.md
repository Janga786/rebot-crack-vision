# Isaac Sim capstone scene: reBot B601-DM crack inspection

## What this is

A headless, RTX-rendered Isaac Sim scene of the reBot B601-DM 6-DOF arm
"inspecting" a concrete coupon whose top face carries the real crack image the
CV pipeline ran on, with the vision-derived inspection path drawn on top.
Everything is built and rendered by one script; nothing here is hand-edited
USD or a GUI capture.

## Environment / how to re-run

```
source ~/miniconda3/etc/profile.d/conda.sh && conda activate isaaclab
python -u /home/boosterk1/Projects/rebot_crack_vision/presentation/sim/build_scene.py
```

- Isaac Sim 5.0.0.0 (pip install), IsaacLab 0.44.9, Python 3.11, headless (no
  DISPLAY use, no GUI window).
- Runs the standard `isaacsim.asset.importer.urdf` importer and
  `omni.replicator.core` orchestrator for image capture (`Camera.get_rgb()`
  alone returns `None` until you drive a frame through
  `rep.orchestrator.step()` -- that's what `shoot()` does).
- **Runtime:** first-ever Isaac Sim launch on a machine compiles shaders and
  can take 5-15 minutes; every launch after that on this machine (shader
  cache warm in `~/.cache/ov`) takes ~10-12s to reach "app ready" and the
  whole script (scene build + 4 renders + USD export) finishes in
  **~100 seconds**.
- Everything is deterministic and idempotent -- re-running overwrites the
  same 3 PNGs, `coupon_crack_texture.png`, and `crack_inspection.usd`.

## What it builds

- A `reBot_B601_DM_with_gripper.urdf` copy (this directory) with the
  `package://rebotarm_bringup/...` mesh paths in the original
  `~/rebot_ws/src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf`
  rewritten to the absolute path of the read-only mesh directory (the
  original `~/rebot_ws` tree is never modified). The importer is run with
  `fix_base=True`.
- A robot plinth and a separate coupon pedestal (support column + 5cm slab),
  both concrete/metal `UsdPreviewSurface` materials, sitting on a large
  floor plane. All geometry is procedural (`UsdGeom.Cube`/`Mesh`), no
  external assets besides the robot meshes and the crack PNG.
- The coupon's top face is a **dedicated 0.30 x 0.30 m quad** (not a box's
  top face) centred at `COUPON_CENTER = (0.22, 0.0)`, top at `z=0.1204`,
  textured with `coupon_crack_texture.png` -- see "Texture orientation" below
  for why that file is a rotated copy of the source image, not the source
  image itself. That centre was originally `(0.42, 0.0)`; it moved after the
  boresight-axis fix below made position+orientation reachability measurably
  harder near full extension -- see "The gripper-boresight-axis bug" and
  `docs/TECHNICAL_APPROACH.md` sec 2.5 for the reachability sweep that picked
  0.22 m specifically (18/18 real inspection waypoints under 0.1 mm error,
  versus 5/18 under 2 mm at the original placement).
- Two copies of the 18-waypoint path from `presentation/demo/crack_path_3d.json`:
  - `/World/InspectionPath` -- the real path, spheres+tube at the JSON's own
    standoff height (surface + 45mm). Shown in `scene_overview.png` and
    `closeup_coupon.png`.
  - `/World/InspectionPathFlat` -- the same 18 waypoints with Z forced to
    `0.1215` (flush with the surface, zero standoff). Shown only in
    `topdown_path.png`, so that shot has zero perspective/parallax between
    path and crack regardless of camera angle -- it is the actual proof shot,
    not a nice-looking substitute.
  - A hidden-by-default `/World/CornerFiducials` group (4 distinct-colour
    markers at the source image's 4 corners via the exact
    `pixel_to_world` formula) used only for the `_fiducial_check.png`
    diagnostic (see below).
- A static "inspecting" arm pose (`STATIC_ARM_POSE_RAD`) loaded directly from
  `joint_trajectory.json`'s `crack_09` waypoint -- the actual IK solution
  `arm_kinematics.py` produces, not a separately hand-tuned pose. An earlier
  version of this constant *was* solved independently (closed-form FK +
  `scipy` `L-BFGS-B`) before the tool-boresight-axis bug described below was
  found, and inherited that bug: it visibly pointed the gripper sideways at
  the coupon. Loading the real trajectory instead means this file cannot
  drift out of sync with the kinematics again. A second "retracted" pose
  (`RETRACTED_ARM_POSE_RAD`) swings the arm clear of the coupon for the
  topdown shot (verified clear with the same FK) so the arm never occludes
  the crack in that shot. `apply_joint_pose(articulation, joint_angles_rad,
  gripper_opening_m)` sets these by **name**
  (`SingleArticulation.set_joint_positions`, a direct "teleport" of the
  physics DOF state) and is the same hook `render_trajectory_video.py` calls
  once per animation frame.

## The gripper-boresight-axis bug (found while adding the trajectory video)

The first version of the kinematics assumed the gripper's "pointing" direction
was the tool frame's local **+Z** axis, because it happened to equal world
`(0,0,-1)` at the all-zero joint configuration. That was a coincidence, not a
property of the gripper, and it meant every rendered pose here held the
*wrong* axis vertical -- invisible in a single still image, unmistakable in a
moving video, where the wrist visibly failed to track the surface below it.

The fix used the same "measure the real geometry" principle as the texture
orientation below: `_probe_gripper_axis.py` loads the URDF in Isaac Sim at
`q=0` and reads the world-space bounding-box center of `link6`, `gripper_link`
and both finger links, expressed in the gripper's own frame. Every local-Z
coordinate came back ~0 (the whole gripper body lies flat in the frame's own
XY plane) while local-X ran monotonically from the wrist toward the fingers --
the gripper physically points along local **+X**, not +Z.
`arm_kinematics.py` was corrected to hold local +X vertical instead, and the
whole trajectory was re-solved. Confirmed two ways after the fact: (1) an
independent, pre-existing MoveIt2 config for this exact arm
(`~/rebot_ws/src/rebotarm_moveit_config`) defines its own tool-centre-point
frame as a pure translation along the same local X axis, with no rotation --
built by someone else, for an unrelated purpose, and it agrees; (2)
`scene_overview.png`/`closeup_coupon.png` now visibly show the gripper hanging
straight down over the coupon, not reaching in sideways as they did before the
fix. Full derivation and the exact numbers: `docs/TECHNICAL_APPROACH.md` §2.3.

## Renders (all 1600x900, `presentation/renders/`)

- **`scene_overview.png`** -- 3/4 perspective view: arm on its plinth
  reaching over to the coupon on its own pedestal, floating path + crack
  both visible, dark floor for separation, warm key light / cool fill.
- **`topdown_path.png`** -- orthographic, camera centred exactly over the
  coupon (`(COUPON_CENTER, 1.15)` looking straight down), retracted arm pose,
  **flat** zero-standoff path only. This is the shot that has to prove the
  vision path lands on the painted crack.
- **`closeup_coupon.png`** -- perspective, arm still in the inspecting pose,
  camera close to the coupon surface so the gripper, several waypoints, the
  path tube and the crack are all legible together.
- **`_fiducial_check.png`** (in `sim/`, not `renders/` -- diagnostic, not a
  deliverable) -- same topdown camera, corner fiducials shown instead of the
  path. Confirms the 4 markers (white/black/cyan/magenta, one per source
  image corner) land exactly on the coupon's 4 geometric corners.
- **`trajectory.mp4`** -- built by `render_trajectory_video.py`, not
  `build_scene.py`. Same scene-construction helpers (duplicated, not
  imported, so a change to one script can't destabilize the other), plus a
  frame loop: at each output frame it linearly interpolates joint angles
  between the two solved keyframes bracketing that instant (an honest
  approximation between validated poses, not a re-solved Cartesian path),
  teleports the articulation there, and renders. Played back at 2x real time
  (24.3 s of motion -> ~12 s of video at 20 fps) with an on-frame HUD stating
  the real elapsed time and playback speed, so nothing about the speed-up is
  hidden. Frames are written to `_video_frames/` and encoded with `ffmpeg`
  (H.264/yuv420p); the PNG frames are deleted afterward, not committed.
  Rerun with `python -u render_trajectory_video.py [--fps N] [--speed X]
  [--max-frames N]` (the last one for a quick timing test before committing
  to a full render -- each frame costs about 2s at the default settle
  settings, so a full render is on the order of several minutes).

## Texture orientation -- what was wrong and how it was found

`crack_to_path.py` maps pixel `(row, col)` to world `(x, y)` with
`image +row -> robot -X`, `image +col -> robot -Y`. Whether the "obvious" UV
assignment on the coupon quad (`s=(x-xmin)/size`, `t=(y-ymin)/size`) lines the
texture file up with that convention depends on the renderer's own
texture-sampling convention, which isn't documented anywhere accessible here
-- so it was **measured, not assumed**, in two stages:

1. `_calib_probe.py` painted the coupon with a 4-quadrant colour texture and
   4 distinct markers at the image's 4 corners, rendered top-down, and read
   back which colour ended up under which marker. The result matched
   `np.rot90(image, k=1)` exactly (all 4 corners, verified with an explicit
   array-equality check against all 8 dihedral transforms, not eyeballed).
2. That rotation is applied once, in `bake_coupon_texture()`, to produce
   `coupon_crack_texture.png` from `presentation/assets/synthetic_crack_input.png`.
   The mesh's UV assignment is never touched -- only the baked texture file
   changes if this is ever re-derived.

**Independent verification (`_fiducial_check.png`):** after the fix, 4
distinctly-coloured markers were placed at the real coupon (not the
calibration texture) at the source image's 4 corners via the exact
`pixel_to_world` formula. All 4 land exactly on the coupon's geometric
corners. Since a 90-degree rotation is a rigid transform, exact agreement at
all 4 corners mathematically guarantees exact agreement everywhere in the
interior too -- there is no remaining degree of freedom for an interior
offset once the corners match.

**On the residual visual gap in `topdown_path.png`:** looking closely, the
path and the crack are close but not pixel-perfect at every point. This was
investigated rather than dismissed:
- Compared `synthetic_crack_input.png` (what's painted) against
  `synthetic_crack_skeleton.png` (what the path was derived from) directly:
  median pixel-level offset is **0 px** -- the two source images agree
  exactly, so this isn't a source-image mismatch.
- Compared the true 591-pixel skeleton centerline against the 18-waypoint
  RDP-simplified path used for the visualization: max deviation **0.87 mm**,
  mean **0.24 mm** -- the simplification itself is not the cause either.
- The corner-fiducial result above rules out a placement/orientation bug.
- What's left is the visible width of the rendered path elements themselves
  (4.5mm-radius spheres / 2mm-radius tube -> several mm of rendered width)
  and ordinary anti-aliasing/texture-filtering blur on a thin (1-2px-wide at
  source resolution) painted line. An attempt to quantify a "gap" from the
  rendered pixels directly (colour-threshold nearest-neighbour) produced
  inconsistent-direction, inconsistent-magnitude numbers (3-18mm depending on
  method) that don't behave like a systematic translation -- which is itself
  evidence against a real geometric offset, since a genuine texture/geometry
  bug would show up as a *consistent* shift, not a locally-varying one.
- Bottom line: no evidence of an actual placement bug survived investigation;
  what's visible is consistent with normal rendering/marker-size fuzziness
  on a coupon this size at this resolution, not a coordinate error. Waypoint
  math and JSON were never touched to make the picture agree, per the task's
  hard rule.

## Trajectory-playback hook (for the follow-up message)

`apply_joint_pose(articulation, joint_angles_rad, gripper_opening_m)` (in
`build_scene.py`) is the function to call once per frame of a future
`joint_trajectory.json`: it looks up each of `joint1..joint6` /
`gripper_joint1` / `gripper_joint2` by name in `articulation.dof_names` (so
it doesn't depend on DOF ordering), sets them with
`SingleArticulation.set_joint_positions` (an immediate kinematic "teleport",
already used here for both the static inspecting pose and the retracted
pose), then a few `world.step(render=False)` calls before capturing a frame
with `shoot()`. A trajectory-animation pass should loop:
`apply_joint_pose(art, frame_q) -> world.step(render=False) x N -> shoot(...)`
per frame, reusing the same `camera`, `art`, and `world` objects `main()`
already constructs.

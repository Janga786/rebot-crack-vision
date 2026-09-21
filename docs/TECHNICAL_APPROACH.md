# TECHNICAL_APPROACH.md — how this actually works, end to end

This is the "under the hood" document: the computer-vision model, and the kinematics and
simulation work that follows its output into a robot arm. `docs/ARCHITECTURE.md` is the
normative spec for the validated phase-1 pipeline (`src/crackvision/`) — read that first if
you haven't. This document explains *why* the kinematics and simulation code (all under
`presentation/demo/` and `presentation/sim/`, explicitly scaffolding, not phase-1) is built the
way it is, including two real mistakes that were found and fixed along the way (§2.3, §2.5),
because that's more useful to a reader than a clean story with the mistakes edited out.

---

## Part 1 — the computer-vision model

### 1.1 What it is

[**OpenCrack**](https://huggingface.co/fadeevla/opencrack-nnunet) is a 2D crack-segmentation
model built on [**nnU-Net v2**](https://github.com/MIC-DKFZ/nnUNet) (Isensee et al., *Nature
Methods* 2021) — not a custom architecture, but nnU-Net's standard self-configuring
`PlainConvUNet` with the hyperparameters nnU-Net's own fingerprinting picked for this dataset:

| | |
|---|---|
| Architecture | `PlainConvUNet`, 7 stages, **~92.5 M parameters** |
| Input | 3-channel RGB, 256×256 patches, `spacing=[1.0, 1.0]` |
| Normalization | z-score, computed per-channel from the training set |
| Training data | 49,531 images (`OpenCrack-cat1`) |
| Fold used | fold 0 of 5 (`checkpoint_ep0500.pth`, not the training-final checkpoint) |
| Output | binary segmentation, `{background: 0, crack: 1}` |
| Reported quality | 0.539 IoU / 0.641 clIoU (τ=4) on OpenCrack's own held-out test cohort |
| Reported throughput | ≈1.45 img/s at 2048², ≈21 img/s on 600-px tiles, on an 8 GB GPU |
| Licence | CC-BY-4.0 |

nnU-Net's own contribution isn't a novel network — it's the *self-configuration* pipeline
(dataset fingerprinting → plan generation → the U-Net variant, patch size, batch size and
normalization scheme above all fall out of that fingerprint automatically). OpenCrack is that
pipeline run once, checked in, and released as weights. This project does not retrain or
fine-tune it — the entire phase-1 question is whether those weights, as published, work
**zero-shot** on our own imagery.

### 1.2 Why this model, not a custom one

Building a crack detector from scratch means labelling a dataset first. OpenCrack's own model
card documents real limitations (it over-fires on crack-like texture and masonry mortar lines,
and collapses on wide road scenes at only 0.142 IoU) — but close-up concrete/bridge-deck
imagery, which is this project's actual regime, is squarely in-domain for what it was trained
on. Testing the free option before funding a labelling campaign is the entire point of phase 1;
see `docs/adr/001-opencrack-primary-model.md`.

### 1.3 The inference contract

`nnUNetv2_predict_from_modelfolder` is invoked directly against the downloaded model folder
(not through nnU-Net's dataset-ID resolution, which needs environment variables this project
avoids setting globally — see `docs/adr/006-predict-from-modelfolder.md`). Two defaults are
overridden, both because getting them wrong produces a confusing "file not found" several
layers deep in nnU-Net rather than a clear error:

- `-f 0` (nnU-Net defaults to ensembling folds `0,1,2,3,4`; only fold 0 was released)
- `-chk checkpoint_ep0500.pth` (nnU-Net defaults to `checkpoint_final.pth`, which this
  checkpoint doesn't have)

The single fact that shapes every consumer of the model's output: **predictions are PNGs with
pixel values `{0, 1}`, not `{0, 255}`** (`nnUNetv2`'s `NaturalImage2DIO.write_seg` writes raw
label values). Viewed directly, the mask looks solid black. Every piece of code that reads a
prediction — `visualize.py`, the skeleton extraction, the demo path-builder — binarizes as
`pred > 0`, and this is asserted by a test (`docs/RISKS.md` R-03) so it can never silently
regress.

### 1.4 The one rule that makes a downstream robot trajectory possible

**Nothing between the original image and the final skeleton resizes, crops, or pads.**
`prepare_inputs` forces every input to 3-channel RGB losslessly, at its original resolution;
the model's own patch-based inference reassembles a full-resolution mask; the skeleton is
extracted from that mask without touching its dimensions. The practical payoff: a pixel
`(row, col)` in the final skeleton is the *same* `(row, col)` in the original camera frame,
which is what makes it possible to later index an aligned depth frame with no re-registration
step. See `docs/INTERFACES.md` §0.5 for the normative statement of this rule.

---

## Part 2 — from a 2D mask to a 3D robot path

Everything below this line is presentation scaffolding (`presentation/demo/`,
`presentation/sim/`) — explicitly deferred by `docs/adr/002`, `adr/008` and `adr/009`, and
never imported by `src/crackvision/`. It exists to show the full intended pipeline working
end to end against a real segmentation result, not to claim the robot integration is done.

### 2.1 Mask → ordered centerline → 3D waypoints

`crack_to_path.py` takes the binary mask's skeleton (a `skimage`-style 1-pixel-wide centerline,
`presentation/assets/synthetic_crack_skeleton.png`), and:

1. **Orders it.** A crack's skeleton is a sparse pixel set, not a path — pixel order in the PNG
   is meaningless. Two breadth-first searches (find the pixel farthest from an arbitrary seed,
   then the pixel farthest from *that*) find the graph's diameter: the longest simple path
   through the skeleton, which is the dominant crack. A short branch/spur visible in the source
   image falls off the ordered path — an inspection pass follows the main crack, not every spur.
2. **Simplifies it.** Ramer-Douglas-Peucker reduces the ~590-pixel ordered centerline to 18
   waypoints (tolerance tuned so no corner of the crack's shape is lost), because a robot
   trajectory needs a *sparse* set of Cartesian targets, not one per pixel.
3. **Projects it into the robot's base frame.** The mapping is a plain affine transform because
   of the frame-invariant rule above: `image +row → robot −X`, `image +col → robot −Y`, scaled
   by the coupon's physical size in metres divided by the image's pixel size. This is the one
   place a physical assumption is made explicit rather than measured: *that* the coupon sits at
   a given pose (a 0.30 m square, surface at `z=0.12` in the robot base frame) is declared, not
   calibrated from a real camera extrinsic — there is no camera mounted on this arm yet. *Where*
   exactly, though, is not arbitrary: the centre is `(0.22, 0)`, chosen from a measured
   IK-reachability sweep (§2.5), not guessed. `presentation/demo/crack_path_3d.json` records the
   assumption explicitly in its own `assumptions` block rather than burying it in code.

### 2.2 Forward and inverse kinematics (`arm_kinematics.py`)

The reBot B601-DM is a 6-revolute-joint arm with a fixed-mount 2-finger gripper. Rather than
pull in a URDF-parsing/IK library, the whole chain is transcribed by hand from the URDF's own
`<origin>`/`<axis>`/`<limit>` tags into six `(xyz, rpy, axis, limits)` tuples plus one fixed
transform for the gripper, and forward kinematics is the direct product of per-joint
transforms:

```
T_i = Trans(xyz_i) · R(rpy_i) · Rot(axis_i, θ_i)          [URDF fixed-axis/extrinsic convention]
T_tool = T_1 · T_2 · T_3 · T_4 · T_5 · T_6 · T_gripper
```

The **geometric Jacobian** (how the tool's linear and angular velocity respond to each joint's
velocity) is computed analytically rather than by finite differences, using the standard
revolute-joint identity `J_v_i = z_i × (p_tool − p_i)`, `J_w_i = z_i`, where `z_i` and `p_i` are
read directly off the cumulative frame `T_i` computed during the same forward pass — a joint's
own rotation doesn't move its own axis or origin, so no extra computation is needed to get
these "before this joint's rotation" quantities.

**Inverse kinematics** is damped least-squares (Levenberg-Marquardt-style resolved-rate IK):
at each iteration, solve `Δq = Jᵀ(JJᵀ + λ²I)⁻¹e` for the joint step that reduces the current
error `e`, re-evaluate, repeat. Two engineering decisions matter more than the algorithm choice
itself:

- **Only 5 of the 6 possible pose constraints are pinned** (3 position + 2 pointing-direction
  for the tool's boresight axis), not a full 6-DOF orientation. The 6th degree of freedom
  (rotation *about* the boresight — "tool roll") is left to fall out of the minimum-norm
  damped-least-squares step. An earlier attempt to pin the full orientation left several
  waypoints unreachable (joints 4/5 saturated their limits trying to also fight for a specific
  roll angle nothing downstream needs); freeing that one redundant DOF fixed every waypoint.
- **Joint limits are enforced by clamping after every step**, not as a soft penalty, so a
  returned solution is always inside the URDF's limits even when the solve hasn't fully
  converged — an unreachable target degrades to "closest reachable pose," never an invalid one.

### 2.3 A real mistake, found and fixed: the boresight axis

The first version of this code assumed the gripper's "pointing" direction was the tool frame's
local **+Z** axis, because `fk([0,0,0,0,0,0])` happens to put that axis at world `(0, 0, −1)` —
straight down — at the all-zero joint configuration. That looked like confirmation. It wasn't:
it was a coincidence of one specific pose, and it meant every solved waypoint held the *wrong*
axis vertical. A still image can hide this (a camera angle can make a tilted gripper look
plausible); a video of the arm moving along the path cannot — the wrist visibly failed to track
the surface underneath it.

The fix followed the module's own stated principle — *measure the real geometry, don't assume
a coordinate convention* — the same way the coupon texture's orientation was determined in
§3.2 below. `presentation/sim/_probe_gripper_axis.py` loads the actual URDF in Isaac Sim at
`q=0`, takes the world-space bounding-box center of `link6`, `gripper_link`, and both finger
links (`gripper_left`/`gripper_right`), and expresses each in the gripper frame:

| link | local-frame center (x, y, z) |
|---|---|
| `link6` (upstream of the gripper) | (−0.1555, 0.0000, 0.0003) |
| `gripper_link` | (−0.1120, −0.0000, 0.0004) |
| `gripper_left` (finger) | (−0.0443, −0.0078, 0.0005) |
| `gripper_right` (finger) | (−0.0443, 0.0078, 0.0006) |

Every local-**Z** coordinate is ≈0 — the entire gripper body lies in a thin slab through the
frame's own XY plane — while local-**X** runs monotonically from −0.156 (the wrist, furthest
back) toward −0.044 (the fingers, furthest forward). The gripper physically extends along the
frame's local **+X** axis, not +Z. `arm_kinematics.py` was corrected to drive local +X to world
`(0, 0, −1)` instead, and re-verified: at `q=0` the boresight axis is now `(1, 0, 0)` (correctly
*not* vertical — the earlier "confirmation" is now understood as coincidence), and the IK
converges to point it straight down with axis error under 0.001°.

That correction changed which joint configurations are reachable (constraining a different
physical axis is a different, and for this arm harder, geometric problem — joints 4/5 have the
tightest limits in the chain, ±1.87/1.57 rad against ±2.8–3.14 rad on joints 1–3). A naive
seed-from-previous-waypoint chain walked into a joint-limit-locked branch partway along the path
and every later waypoint inherited it, growing to nearly 200 mm of position error by the final
waypoint. This is documented, not hidden: see `path_to_joint_trajectory.py`'s per-waypoint
**recovery** step, which tries the cheap continuity-preserving seed first and escalates to a
fresh multi-start search only on the waypoints where that's not enough — and the `recovery_log`
field in `joint_trajectory.json` records exactly which waypoints needed it. A first version of
recovery picked whichever candidate had the lowest position error with no regard for continuity,
which found a second real problem: one recovered waypoint landed 4.2 rad (241°) of joint6 away
from its neighbour — individually valid, but a wrist snap no real trajectory should ask for.
`find_good_seed`'s `q_prefer` pool fixes this: among every candidate within a small tolerance of
the best position error found, keep whichever is closest in joint space to the previous
waypoint's solution.

Two independent facts corroborate that local +X is the right choice, found after the fact:
- **Verified in the solved trajectory itself:** joint5 settles within a fraction of a degree of
  0 rad at all 20 poses, and joint6 holds one *exactly* constant value across all 18 real
  crack-inspection waypoints (it only departs from it at `approach`/`retract`, discussed below) —
  the arm's one redundant DOF landing on a clean, stable extremum is exactly the behaviour a
  correct axis choice should produce, not something forced.
- **An independent, pre-existing source:** `~/rebot_ws/src/rebotarm_moveit_config` — a MoveIt2
  configuration for this exact arm that already existed on this machine — defines an official
  tool-center-point frame, `gripper_tcp`, as a *pure translation* along `gripper_link`'s local
  **X** axis (`<origin xyz="-0.0443 0 0" rpy="0 0 0"/>`), at almost exactly the same coordinate
  the mesh probe found for the fingertips. Whoever built that config independently treated local
  X as the reach axis too, with no rotation applied — strong corroboration from a source that
  had nothing to do with this investigation.

### 2.4 Velocity along the path (`compute_velocities.py`)

Given the solved joint trajectory `q(t)` at 20 keyframes, three quantities are derived as
segment-average finite differences (explicitly *not* a finer resampling — an honest,
documented simplification):

- **Joint angular velocity** `q̇ᵢ = Δqᵢ/Δt` per joint, checked against each joint's URDF-rated
  velocity limit (50 rad/s on joints 1–3, 200 rad/s on 4–6).
- **Tool linear velocity** `v = Δp/Δt` from consecutive forward-kinematics tool positions.
- **Tool angular velocity**, via the axis-angle ("Rodrigues log map") of the relative rotation
  between consecutive tool orientations, `ω = axis(R_{i+1} R_i^T) · θ / Δt` — the same standard
  construction used to turn a rotation matrix back into a 3-vector rate.

Linear speed holds within 1% of the 2 cm/s design target throughout. Tool angular speed stays
under ~10°/s across the whole 18-point inspection pass itself — the wrist genuinely doesn't wobble
while joints 1–3 individually sweep tens of degrees — but spikes to ~34°/s in the single segment
ending at `retract`, because that is one of the two waypoints described in §2.5 that didn't fully
converge and exits on a different joint6 branch than the rest of the path. `fig07_velocity.png`
plots both, honestly, spike included.

### 2.5 A second real fix: where you put the part matters more than how hard the solver tries

Even after both corrections above, two of the 20 poses — `approach` and `retract`, pure vertical
hovers 45–145 mm above the crack path's first and last points — would not converge below 15–40 mm
of position error no matter how large the recovery search was made. All 18 real crack-inspection
waypoints, by contrast, converged to a mean of **0.026 mm**. That contrast (trivial on the surface,
hard immediately above it) is the signature of a *workspace placement* problem, not a solver
quality problem: the coupon was originally placed at `x = 0.42 m` in the robot base frame, and the
joints with the least range in this chain (4/5, ±107°/±90°) are exactly the ones asked to do more
work as the arm approaches full extension.

This was tested directly rather than assumed: a sweep re-solved the same 18 crack waypoints (single
seed, no recovery search) at coupon distances from 0.42 m down to 0.20 m.

| coupon x (m) | waypoints reaching <2 mm | mean error | max error |
|---|---|---|---|
| 0.42 (original) | 5 / 18 | 23.4 mm | 109.9 mm |
| 0.32 | 14 / 18 | 4.6 mm | 35.2 mm |
| 0.24 | 17 / 18 | 0.36 mm | 6.0 mm |
| **0.22 (chosen)** | **18 / 18** | **0.03 mm** | **0.10 mm** |
| 0.20 | 16 / 18 | 0.38 mm | 3.4 mm |

`crack_to_path.py`'s `COUPON_CENTER` was moved to `(0.22, 0.0)` on this evidence — a real,
standard robotics fix (place the part inside the arm's comfortable reach envelope) rather than a
solver tuning hack, and it is documented as such at the constant's definition, not silently
changed. The same logic was applied to `approach`/`retract`'s hover height: a second sweep showed
their own position/axis error falling steadily as the hover distance shrank from 0.10 m to 0.03 m,
so `STANDOFF_APPROACH_M` was reduced to 0.04 m — enough to keep a real physical clearance above the
surface while landing inside the arm's now-comfortable reach. This closed most, but not all, of the
gap: `approach` still finishes at 19.4 mm and `retract` at 5.7 mm even after the same recovery
search every other waypoint gets — the honest remainder is reported in `joint_trajectory.json`'s
`reachable` flags rather than hidden by a loosened tolerance.

**If this goes further than a demo:** `~/rebot_ws/src/rebotarm_moveit_config`'s kinematics plugin is
the default `KDLKinematicsPlugin`, not TRAC-IK — a drop-in config change that is known to be more
robust at exactly this kind of joint-limit-adjacent problem — and MoveIt's `computeCartesianPath()`
is the standard, purpose-built tool for "trace this sequence of Cartesian points," reporting the
fraction of a path it could complete rather than failing pointwise the way a hand-rolled per-waypoint
solver does. Wiring this demo's path into that existing ROS2/MoveIt stack instead of the standalone
Python IK here is real, valuable follow-up work, not a same-session swap.

---

## Part 3 — Isaac Sim

`presentation/sim/build_scene.py` (stills) and `render_trajectory_video.py` (animation) build
the same scene headlessly in Isaac Sim 5.0: the real B601-DM URDF (mesh paths rewritten to
absolute paths so Isaac's importer can resolve them — the original `~/rebot_ws` tree is never
modified), a concrete coupon whose top face is textured with the literal image the segmentation
model ran on, and the vision-derived path drawn as spheres and a tube in the scene.

### 3.1 Why headless, and the render pipeline

Isaac Sim's `Camera.get_rgb()` returns `None` until a frame has actually been pushed through
`omni.replicator.core`'s render-product graph — a bare `simulation_app.update()` loop is not
enough, which is a common early trap. `shoot()` (in `build_scene.py`) instead drives
`rep.orchestrator.step()` a fixed number of times per capture. First launch on this machine
compiles shaders (5–15 minutes); every launch after that reaches "app ready" in ~10 seconds.

### 3.2 The coupon texture orientation — also measured, not assumed

The same principle from §2.3 applies here: whether the "obvious" UV assignment on the coupon
quad lines the crack image up correctly depends on the renderer's own texture-sampling
convention, which isn't documented anywhere accessible. `_calib_probe.py` painted the coupon
with a 4-quadrant colour texture and dropped a distinctly-coloured marker at each of the source
image's 4 corners (via the exact `pixel_to_world` formula the real path uses), rendered
top-down, and read back which colour ended up under which marker. The result matched a 90°
rotation exactly (`np.rot90(image, k=1)`); that rotation is baked into the texture file once,
and the mesh's UV assignment is never touched. A second, independent check
(`_fiducial_check.png`) places 4 markers at the source image's corners on the *real* coupon and
confirms they land exactly on its geometric corners — since a rigid rotation that matches at
all 4 corners is mathematically exact everywhere, this rules out a placement bug rather than
merely suggesting one.

A small (few-millimetre) visual gap remains between the rendered path and the painted crack in
some of the top-down still. This was investigated the same way, not dismissed: a direct
pixel-space comparison of the drawn line against the model's own mask and skeleton found
essentially zero systematic bias (sub-pixel), which rules out the vision pipeline as the cause.
What's left is ordinary marker size and texture-filtering blur at this render resolution — see
`presentation/sim/README.md` for the full investigation.

### 3.3 The trajectory video

`render_trajectory_video.py` reuses the verified scene-construction helpers (copied, not
imported, from `build_scene.py`, so a change to one script can never destabilize the other) and
adds a frame loop: at each output frame, it linearly interpolates joint angles between the two
solved keyframes bracketing that instant, sets the articulation's joint positions directly
(`SingleArticulation.set_joint_positions` — an immediate kinematic "teleport," not a physics
simulation of the actuators), steps the renderer, and saves a frame. What's real: the joint
angles *at* every keyframe are the actual IK solution described in §2.2–2.3. What's a
presentation choice, stated on-screen via an on-frame HUD rather than hidden: motion *between*
keyframes is linear interpolation, not a re-solved Cartesian path, and playback runs at 2×
real time so a 24-second, 2 cm/s inspection pass is watchable. Frames are encoded to
`presentation/renders/trajectory.mp4` with `ffmpeg` (H.264, `yuv420p`) and the intermediate PNG
frames are deleted rather than committed.

---

## What's real and what's declared — summary

| Claim | Status |
|---|---|
| The segmentation model runs zero-shot, unmodified, on our hardware | **Measured** — real GPU inference, timed |
| The mask/skeleton faithfully represents the drawn crack | **Measured** — sub-pixel agreement, checked directly |
| The IK solution reaches all 18 real crack-inspection waypoints with the boresight vertical | **Measured** — mean 0.026 mm, max 0.099 mm position error |
| The `approach`/`retract` hover transitions reach the same target | **Measured, and honestly short** — 19.4 mm / 5.7 mm error even after recovery; see §2.5 |
| Tool speed is constant, wrist doesn't wobble across the inspection pass | **Measured** — from the same solved trajectory; see §2.4 for the one exception (the retract transition) |
| The coupon's pose relative to the robot | **Declared for camera calibration, chosen for IK reachability** — no camera-to-robot calibration exists yet, but the placement itself is backed by a measured sweep, not arbitrary (§2.5) |
| Motion *between* keyframes in the video | **Interpolated**, stated as such on-screen |
| Zero-shot quality on real D405 imagery | **Not yet measured** — the actual phase-1 headline question; see `docs/D405_TEST_PLAN.md` |

See `presentation/README.md` for the slide deck this document supports, and
`presentation/sim/README.md` / module docstrings in `presentation/demo/*.py` for the
file-by-file detail this document summarizes.

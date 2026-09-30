# ADR-014: Tool tip, grasp TCP and wrist camera are separate frames; viewing and tool motion are separate phases

**Status:** accepted · **Date:** 2026-09-30 · **Supersedes:** the `TCP`/boresight-prior parts of ADR-012
(frame table row `TCP`, §"Boresight / TCP") · **Superseded by:** —

## Context

Operator facts (authoritative, 2026-09-30):
- The Intel RealSense D405 is **eye-in-hand**, on Seeed's **stock** `D405_305_Mount.step`
  (`Seeed-Projects/reBot-DevArm@590ff16c`, `hardware/reBot_B601_DM/3D_Printed_Parts/`), fitted to the
  B601-DM gripper. The stock pose is a ≈15° downward pitch. The installed camera-to-robot extrinsic has
  **not** been measured; CAD is a prior only, and hand-eye calibration will supersede it.
- The arm's USB adapter is not connected. `K1_Storage` stays untouched.

Engineering facts established while unblocking MOT-04.5 (all re-derivable, see *Evidence*):
1. **`gripper_link`'s origin is the closed-finger tip.** In the canonical URDF (`~/rebot_ws@2ebc4b8c`), the
   finger meshes placed by `gripper_joint1/2` at q=0 reach max x = 0.0000 m in `gripper_link`, at a single
   point (y = 0, z = ±1.3 mm).
2. **`gripper_tcp` is the grasp centre, 44.3 mm proximal to that tip.** It is the vendor's MoveIt
   frame (`rebotarm.urdf.xacro`: `xyz="-0.0443 0 0"`), located at the finger mid-length. It suits grasping,
   not a tool that faces a surface.
3. MOT-04.1 intended its standoffs 0.01/0.04 m as **tool-tip clearances**: 0.04 m is the
   presentation's `STANDOFF_APPROACH_M`, whose IK tip was `gripper_link`. It then applied them to
   `gripper_tcp`, which put the fingertips 34 mm and 4 mm *inside* the surface. So "0 of 4320 targets
   reachable" was the physically correct answer to the wrong question, not a solver or collision bug. The
   MOT-04.4/04.5 card texts even state the 44.3 mm offset backwards.
4. The sweep's thin grid-wide surface slab ran under the robot base and hit the shoulder (link2) at
   surface_z ≥ ≈0.10 m. That is an artefact, since a specimen cannot overlap the base. There was also no
   table at all.
5. ADR-012, GEOM-06's `PRIOR_TCP_OFFSET_M = (-0.0443, 0, 0)` and the pivot procedure treat that grasp
   centre as the prior for the **physical tool tip** that a pivot calibration measures. A correct
   calibration of the closed gripper (≈ (0, 0, 0)) would therefore be flagged as a 44 mm disagreement.
6. Nothing in the repo modelled the camera. "Viewing distance", "grasp-centre clearance" and
   "tool-tip clearance" had been collapsed into one "standoff".

## Decision

### 1. Frames (all rigid on `gripper_link`, placed from one config file)

| Frame | Meaning | Value today | Measured by |
|---|---|---|---|
| `gripper_link` (= ADR-012 `tool0`) | last arm link; origin = closed-finger tip | URDF | — |
| **`tool_tip`** (new) | the crack-facing tool reference: approach, trace and retract are planned for it; boresight = its +X | identity on `gripper_link` (URDF-mesh prior, nominal) | GEOM-07 pivot calibration, with the tool actually used (closed gripper, probe or pen) |
| `gripper_tcp` | vendor grasp centre; stays the SRDF `arm` chain tip | −44.3 mm on x | not used for surface tasks |
| **`camera_link`** (new) | D405 depth/left-imager origin, realsense-ros convention (+x = optical axis, +y left, +z up) | CAD prior: xyz (−0.0783, −0.0090, −0.0640) m, rpy (π, −15°, 0) | GEOM-05 eye-in-hand hand-eye calibration |
| `camera_*_optical_frame` | driver frames | factory-calibrated, published by `realsense2_camera` at runtime; nominal copies only in mock/sim (`nominal_camera_frames:=true`) | factory |
| `camera_mount_link`, `camera_housing_link` | padded (+5 mm) collision boxes for the Seeed mount and the 42×42×23 mm housing | CAD prior | follow GEOM-05 / MOT-10 |

Single source of truth: `config/robot/end_effector.yaml` (schema `crackvision.end_effector/1`,
validated by `crackvision_description/end_effector.py`). Every block carries `value_status: nominal|measured`
plus its provenance. `crackvision_description` (`urdf/b601_dm_end_effector.urdf.xacro`,
`srdf/b601_dm_end_effector.srdf.xacro`) adds these frames to the vendor model without editing
`~/rebot_ws`, and the mock MoveIt stack loads it. Calibration updates the YAML and never the xacro.

The camera **pose prior** is derived as follows. Take the Seeed STEP camera datum, placed on the gripper
the way `github.com/bowenszhu/rebot-b601-rs-d405-wrist-mount@ca379b9a` does. Map it into this model by
Rx(π): Seeed states that the RS and DM grippers are physically identical, and its RS `gripper_end` and new
DM `end_link` frames are Rx(π) of `~/rebot_ws`'s `gripper_link`. Cross-check: the housing clears the
gripper mesh by 9.2 mm, matching that study's independent 9.17 mm, and the mount seats on the gripper body.
**Open assumption, confirmed by the operator before GEOM-05:** the camera sits on the gripper face that
points up at the all-zero joint pose (`gripper_link` −Z). The bowenszhu 30° redesign is *not* what is
installed; it is validated on the B601-RS only.

Derived: the optical axis is 15° off the tool boresight, tilted toward it. The optical centre is 101.5 mm
from `tool_tip`, and the axis crosses the boresight line ≈0.25 m from the camera.

### 2. Motion phases use different task frames and different distances

| Phase | Task frame | Constraint (nominal) | Why this number |
|---|---|---|---|
| **view** (capture) | `camera_link` | optical axis within 15° of the surface anti-normal; viewing distance 0.25 m (band 0.20–0.30) | inside the D405 ideal band (0.07–0.50 m) and the project's 0.10–0.40 m evaluation band; a 0.20 m specimen + tolerance fits the 58° vertical FOV from ≥ 0.235 m |
| **approach / retract** | `tool_tip` | boresight along −normal, clearance 0.04 m | presentation §2.5 hover, originally tip-referenced |
| **trace** (no-contact) | `tool_tip` | boresight along −normal, free roll, clearance 0.01 m | 3–5× the ADR-013 calibration + depth error budget (≈2–3 mm), so no contact under nominal error |

These are three separate configuration items. They are never one "standoff". Contact-following is not in
scope; if it is ever added, it gets its own clearance and force limits in a new ADR.

### 3. Reachability and placement

- IK targets are expressed for the task frame (`ik_link: tool_tip` or `camera_link`). MoveIt's
  `/compute_ik` resolves frames rigidly attached to the SRDF chain tip; this is verified live by
  `check_end_effector`, with 0.000 mm / 0.000° FK agreement.
- The collision model is self-collision including the camera proxies, the workcell **table** (from the
  MOT-03 scene config), and a **specimen block** from the table up to each surface height, excluding the
  robot base footprint plus 2 cm (`surface_collision.model: specimen_block`). Collision checking is never
  relaxed to manufacture feasibility.
- Placement is feasible only if the tool map is feasible (all footprint nodes at both tool clearances)
  **and** a camera view pose over the footprint centre exists (`recommend_placement --emit-view-config`).
- Feasibility probe (coarse, 0.06 m grid, scratch) run 2026-09-30:
  - 599/910 tool targets reachable at 0.01 and 0.04 m tip clearance, against 0 before;
  - a 0.20 m specimen at (0.29, 0) on the table has a ≥ 0.40 rad joint margin;
  - the half-step verification passed 200/200;
  - perpendicular camera views from 0.20–0.30 m are reachable over it;
  - the margin collapses toward x ≈ 0.41 m, which independently reproduces TECHNICAL_APPROACH §2.5's
    "0.22 good, 0.42 poor".

  MOT-04.5 produces the real, full-resolution result.

### 4. Eye-in-hand data contract

A 3D point from a capture is `p_base = FK(q_capture) · T_gripper_link_camera_link · T_camera_link_optical · p_optical`.
**Every capture used for 3D must record the arm's joint state (and the `end_effector.yaml` sha256) at the
capture instant.** Without them the capture cannot be placed in `base_link`. GEOM-08, CAM-05, INT-02 and
OPS-01 inherit this requirement.

### 5. Calibration supersedes priors, and the commissioning gate enforces it

- GEOM-04/05 solve AX = XB for X = `T_gripper_link_camera_link` (or the optical frame), starting from the
  CAD prior. GEOM-05 writes the result into `wrist_camera` with `value_status: measured`, flagging any
  disagreement with the prior above 10 mm or 5°. A disagreement that large points to a wrong mount side,
  seating problem or wrong model, not something to average away.
- GEOM-07 writes the measured tip into `tool`. The pivot prior is `tool_tip` = (0, 0, 0) for the closed
  gripper; a probe is measured, never assumed. GEOM-11 corrects GEOM-06's grasp-centre prior.
- MOT-05's real-execution gate calls `crackvision_description.end_effector.assert_commissioning_ready` (and
  MOT-03's `scene_core.assert_commissioning_ready`). It refuses while anything is still nominal.

## Consequences

**Makes easy:**
- MOT-04 can produce a feasible, independently verified placement instead of an operator-decision
  deadlock.
- Camera geometry is in the collision model before the first powered move.
- Calibrations replace clearly labelled priors in one file.
- Downstream cards (MOT-06 view/trace planning, GEOM-08 3D paths) have explicit frames to target.

**Costs:**
- Two extra frames and a description overlay to maintain alongside the vendor model.
- Every consumer must pick the right task frame: `gripper_tcp` is now only for grasping.
- The camera pose prior carries a discrete, operator-checked assumption (the mount side).
- Reachability results must be re-run after GEOM-05/07 and MOT-10.

## Evidence

- `config/robot/end_effector.yaml` (numbers and provenance).
- `ros2_ws/src/crackvision_description/test/test_end_effector.py` (21 tests).
- `scripts/ros/test_end_effector.sh` (live FK/IK/validity).
- `scripts/ros/test_reachability_task_frames.sh` (tool at 1 cm clearance plus camera view, live).
- Blocked MOT-04.5 attempt `00094` (the frame error, measured).

## Revisit when

- GEOM-05 or GEOM-07 disagree with a prior beyond the flags above.
- The operator reports the camera on the other gripper face.
- A different mount (e.g. the 30° redesign), tool or camera is fitted.
- Contact-following is brought into scope.

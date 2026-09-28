# ADR-012: One frame graph, one transform notation, one pixel convention

**Status:** accepted · **Date:** 2026-09-28 · **Supersedes:** — · **Superseded by:** —

## Context
Geometry now lifts (ADR-011, stage 2): depth→3D backprojection of the ordered crack path and
camera-to-robot calibration are in scope. Both a computer-vision engineer (row/col, u/v, pixel-centre
conventions) and a robotics engineer (frame names, `T_a_b` composition, quaternion order) will read
and write the same numbers, and every downstream card (GEOM-02…07) needs one place to point to instead
of re-deriving a convention per module. Getting this wrong is silent and expensive: a swapped
row/col or a mismatched quaternion order produces a plausible-looking but wrong 3D point, and nothing
in the pipeline raises an error — it just aims the arm at the wrong place. This ADR fixes the
convention once, before any code depends on a guess.

## Decision

### Frames
Named per REP-103 (units and axis orientation) and REP-105 (standard frame semantics). **Corrected
from the first revision of this ADR:** `~/rebot_ws/src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf`
does **not** define a `tool0` link — its kinematic chain ends `base_link → link1…link6 → gripper_link
→ gripper_left/gripper_right`. `~/rebot_ws/src/rebotarm_moveit_config/config/rebotarm.urdf.xacro:8-12`
adds one further fixed frame, `gripper_tcp`, as a child of `gripper_link` with a fixed offset
`xyz="-0.0443 0 0"` (i.e. -0.0443 m along `gripper_link`'s **+X**), and `rebotarm.srdf:4` sets
`<chain base_link="base_link" tip_link="gripper_tcp"/>`. So the prior claim that this frame table was
"consistent with the ROS 2 / MoveIt config already present" was false for `tool0`/`TCP` as originally
named — no code in `~/rebot_ws/src` mentions `tool0`, and `grep -c tool0` over that tree is `0`. This
revision fixes the table to match what actually exists in TF:

| Frame | Meaning |
|---|---|
| `base_link` | Robot base, fixed. All calibrated 3D crack-path points are expressed here before MoveIt planning. |
| `tool0` | **New fixed alias frame, not present in the URDF today.** Defined here as identical to `gripper_link` (zero offset) purely so this ADR's frame table can use the REP-103/MoveIt convention name for "last link of the arm's own kinematic chain, before the gripper's own tip frame." GEOM-06/07 (whichever of the two first brings up the calibration TF tree) is the card that publishes the static `gripper_link → tool0` identity transform; until it does, `tool0` does not resolve in `tf2` and no code may assume it does. |
| `TCP` (tool-centre point) | **Not a new frame.** `TCP` is this ADR's name for the frame that already exists in the MoveIt config as `gripper_tcp` (`rebotarm.urdf.xacro:8-12`, SRDF tip link `rebotarm.srdf:4`). Any code that needs the tool-centre point must look up `gripper_tcp` in `tf2`, or a card may publish a `TCP` alias of it — but the frames are the same physical point. The `-0.0443 m` fixed offset along `gripper_link`'s +X (URDF) is accepted here as **prior evidence**, corroborating (but not identical in origin to) the pointing-axis direction `docs/TECHNICAL_APPROACH.md` §2.2–2.3 measured in Isaac Sim; GEOM-06 defines the pivot-calibration solver and **GEOM-07 is the operator card that measures the real offset and records residuals against this URDF prior.** |
| `camera_link` | RealSense D405 body/mount frame, `+x` forward along the housing, per the vendor's REP-103-style mechanical convention. |
| `camera_color_frame` | Intermediate ROS-convention frame between `camera_link` and the optical frame below; same physical origin as `camera_link` but with axes already rotated (see below). Present in `realsense2_camera`'s published TF tree (`BaseRealSenseNode::calcAndAppendTransformMsgs`). |
| `camera_color_optical_frame` | The frame the intrinsics in `INTERFACES.md` §3.10 project through: **`+z` forward (into the scene), `+x` right, `+y` down** — the standard ROS *optical* frame convention (REP-103), not `camera_link`'s mechanical axes. All pixel projection/deprojection math (below) happens in this frame. |

**Corrected:** `camera_link` → `camera_color_optical_frame` is **not** a pure rotation and is **not**
device-independent. It factors as:

```
T_camera_link_camera_color_optical_frame
    = T_camera_link_camera_color_frame · T_camera_color_frame_camera_color_optical_frame
```

- `T_camera_link_camera_color_frame` carries the **per-device depth→colour extrinsic** — the same
  `rotation`/`translation` pair recorded per-frame in `INTERFACES.md` §3.10 item 7
  (`extrinsics_depth_to_color`, sourced from the device's own stream-profile extrinsics via
  `realsense2_camera`'s `BaseRealSenseNode::calcAndAppendTransformMsgs`). It has a real, generally
  non-zero translation and must be read from device metadata/driver, never hard-coded or assumed
  zero.
- `T_camera_color_frame_camera_color_optical_frame` **is** the fixed, translation-free
  mechanical→optical rotation every ROS camera driver applies, and is the only part of this chain
  that is a device-independent convention.

Only the second half is rotation-only; the first half is per-device and must come from the driver or
recorded metadata, not from this ADR.

### Transform naming
`T_a_b` transforms a point from frame `b` into frame `a`:

```
p_a = T_a_b · p_b
```

Composition follows the matching-subscript rule: `T_a_c = T_a_b · T_b_c`. So the camera-to-robot
calibration this ADR anticipates (REQ-GEOM-2) is named `T_base_link_camera_color_optical_frame`, and
the boresight/pivot calibration (GEOM-06/07) is `T_tool0_TCP` — both sides of that name resolve to
frames that exist in TF per the Frames section above (`tool0` once GEOM-06/07 publishes its static
alias of `gripper_link`; `TCP` as the existing `gripper_tcp` frame or an alias of it), not the
invented frames named in the first revision of this ADR.

- **Rotation representation:** quaternion, **ROS order `(x, y, z, w)`** — matches `geometry_msgs/Quaternion`
  and `tf2`. Any code using `scipy.spatial.transform.Rotation` (which is also `xyzw`) needs no
  reordering; anything touching a library that defaults to `wxyz` (e.g. raw Isaac Sim APIs) must
  convert explicitly at the boundary and say so in a comment.
- **Units:** metres and radians (SI, per REP-103) everywhere in frame/transform math. The one
  documented exception is raw depth storage, below, which is `uint16` device counts until scaled.

### Pixel convention
Two conventions coexist by necessity and must not be confused:

1. **On-disk / array convention:** every raster in this pipeline (mask, skeleton, aligned depth —
   `INTERFACES.md` §0.5) is indexed **`(row, col)`**, matching `numpy`/`PIL` array indexing
   (`arr[row, col]`, `arr.shape == (height, width)`).
2. **Projection convention:** `rs2_project_point_to_pixel` / `rs2_deproject_pixel_to_point`
   (`librealsense2/rsutil.h`, `RS2_API_VERSION` as shipped in `ros-humble-librealsense2` 2.57.7,
   installed at `/opt/ros/humble/include/librealsense2/rsutil.h`) take a `float pixel[2]` ordered
   **`(u, v) = (col, row)`** — i.e. `pixel[0]` is the horizontal/column coordinate, `pixel[1]` the
   vertical/row coordinate. So **`(u, v) = (col, row)`**, the transpose of the array convention above.
   Any code bridging the two must do so explicitly (e.g. `pixel = [col, row]` when calling into
   `pyrealsense2`, never `[row, col]`).

   **Pixel-centre convention:** confirmed from the librealsense reference implementation of
   `rs2_deproject_pixel_to_point` (`include/librealsense2/rsutil.h`, commit
   `5e73f7bb906a3cbec8ae43e888f182cc56c18692`):

   ```c
   float x = (pixel[0] - intrin->ppx) / intrin->fx;
   float y = (pixel[1] - intrin->ppy) / intrin->fy;
   ```

   There is **no `+0.5` pixel-corner offset** anywhere in this computation — an integer pixel
   coordinate addresses that pixel's **centre** directly (the same convention OpenCV uses). GEOM-02's
   deprojection/reprojection round-trip tests must reproduce exactly this formula (no half-pixel
   shift) to match `pyrealsense2`'s own intrinsics.

### Depth
Per-pixel depth is stored as raw `uint16` device counts (`INTERFACES.md` §3.10). To get metres:

```
depth_m = uint16_value * depth_scale_m_per_unit
```

where `depth_scale_m_per_unit` **must** be read from that frame's own metadata JSON (§3.10 item 7,
sourced from `depth_sensor.get_depth_scale()` — never hard-coded; the D405 default is `0.0001`,
other D4xx sensors commonly use `0.001`).

**Invalid depth** is:
- `uint16_value == 0` (the device's own "no return" sentinel), or
- outside the documented valid band for the D405 at the configured resolution — **≈ 0.07 m – 0.50 m**
  (`docs/ARCHITECTURE.md`, D405 section: "Ideal depth range ≈ 7 cm – 50 cm", matching the planned
  10–40 cm evaluation distances). This band is a property of the device/use-case, not a universal
  constant; a card that needs a different working distance must document its own band the same way,
  not silently reuse this one.

Any consumer of depth (GEOM-02 backprojection, per-point uncertainty per REQ-GEOM-1) must treat both
cases identically — mark the point invalid/absent, never silently project depth `0` to a 3D point at
the camera origin.

### Boresight / TCP — status of prior evidence
`docs/TECHNICAL_APPROACH.md` §2.2–2.3 already measured the reBot B601-DM gripper's physical pointing
axis in simulation (Isaac Sim URDF probe: local **+X**, not +Z, corroborated by the arm's own solved
IK behaviour and by `~/rebot_ws/src/rebotarm_moveit_config`). The MoveIt config's own fixed URDF
offset for `gripper_tcp` — `xyz="-0.0443 0 0"` relative to `gripper_link`, i.e. -0.0443 m along
`gripper_link`'s +X (`rebotarm.urdf.xacro:8-12`) — is a second, independent piece of prior evidence
pointing the same direction (local X, not Z). Both are accepted here as **prior evidence only** — they
ground the `T_tool0_TCP` convention defined above in real, previously-recorded facts rather than a
fresh guess, but neither substitutes for a physical calibration, and the `-0.0443 m` figure is a
design-time CAD/URDF value, not a measured one. GEOM-06 defines the pivot-calibration solver and
procedure; **GEOM-07 is the operator card that executes it against the real arm and records residuals
against this prior.** Nothing in this pipeline may treat the boresight axis or the `-0.0443 m` offset
as calibrated fact until GEOM-07 reports measured numbers.

## Consequences
+ Every later geometry card (GEOM-02…07) has one document to cite instead of re-deriving a
  convention, and disagreements between the CV side (row/col) and the robotics side (`T_a_b`,
  quaternion order) become a documentation bug, not a runtime one.
+ The `(row, col)` vs `(u, v) = (col, row)` distinction is now explicit and testable — GEOM-02 can
  and must write a unit test that projects a known 3D point and asserts pixel order and pixel-centre
  behaviour against this ADR's cited `rsutil.h` formula.
+ The boresight/TCP axis is documented as *prior evidence, not calibrated fact*, so no later card can
  accidentally treat a simulation measurement as a substitute for GEOM-07's physical verification.
- This ADR fixes notation only; it does not itself implement backprojection, uncertainty propagation,
  or the hand-eye solve (REQ-GEOM-1/2 remain open work for GEOM-02 and GEOM-06/07).

## Revisit when
GEOM-07's physical boresight/TCP measurement contradicts the simulation-derived axis in
`docs/TECHNICAL_APPROACH.md`, or a future camera/gripper swap changes any frame's physical
orientation.

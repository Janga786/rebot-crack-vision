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
Named per REP-103 (units and axis orientation) and REP-105 (standard frame semantics), consistent
with the ROS 2 / MoveIt config already present at `~/rebot_ws/src/rebotarm_moveit_config`:

| Frame | Meaning |
|---|---|
| `base_link` | Robot base, fixed. All calibrated 3D crack-path points are expressed here before MoveIt planning. |
| `tool0` | The kinematic chain's last link, per the URDF/MoveIt convention — not necessarily where the gripper physically points. |
| `TCP` (tool-centre point) | The gripper's *working* point/axis, offset from `tool0` by the pivot calibration (GEOM-06/07). Distinct from `tool0` on purpose: `docs/TECHNICAL_APPROACH.md` §2.2–2.3 already documents a real bug caused by conflating "the last URDF frame" with "the frame the gripper actually points along." |
| `camera_link` | RealSense D405 body/mount frame, `+x` forward along the housing, per the vendor's REP-103-style mechanical convention. |
| `camera_color_optical_frame` | The frame the intrinsics in `INTERFACES.md` §3.10 project through: **`+z` forward (into the scene), `+x` right, `+y` down** — the standard ROS *optical* frame convention (REP-103), not `camera_link`'s mechanical axes. All pixel projection/deprojection math (below) happens in this frame. |

`camera_link` → `camera_color_optical_frame` is a fixed rotation (no translation), the same fixed
rotation every ROS camera driver applies between a sensor's mechanical frame and its optical frame.
It is not calibrated per-device; it is a convention.

### Transform naming
`T_a_b` transforms a point from frame `b` into frame `a`:

```
p_a = T_a_b · p_b
```

Composition follows the matching-subscript rule: `T_a_c = T_a_b · T_b_c`. So the camera-to-robot
calibration this ADR anticipates (REQ-GEOM-2) is named `T_base_link_camera_color_optical_frame`, and
the boresight/pivot calibration (GEOM-06/07) is `T_tool0_TCP`.

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
IK behaviour and by `~/rebot_ws/src/rebotarm_moveit_config`). That measurement is accepted here as
**prior evidence only** — it grounds the `T_tool0_TCP` convention defined above in a real,
previously-verified fact rather than a fresh guess, but it does not substitute for a physical
calibration. GEOM-06 defines the pivot-calibration solver and procedure; **GEOM-07 is the operator
card that executes it against the real arm and records residuals.** Nothing in this pipeline may
treat the boresight axis as calibrated fact until GEOM-07 reports measured numbers.

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

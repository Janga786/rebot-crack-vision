# ADR-015: ROS 2 camera integration is two narrow touchpoints, not a running driver

**Status:** accepted · **Date:** 2026-10-08 · **Supersedes:** — · **Superseded by:** —

## Context

CAM-05 asks what scope, if any, ROS 2 camera integration needs. The live capture path
(`crackvision.realsense_capture`, `crackvision.recording`; CAM-01/CAM-02) already talks to the D405
directly via `pyrealsense2` and has no ROS dependency. Two facts from already-accepted work pull in
the opposite direction and must be reconciled, not re-decided:

- `docs/calibration/PLAN.md#0` confirms (2026-09-30, authoritative) that the mount is **eye-in-hand**:
  the D405 rides on the gripper, not on a fixed point in `base_link`. A fixed (eye-to-hand) mount would
  not need a per-capture joint-state reading at all — one static extrinsic would cover every capture.
  Eye-in-hand means the camera-to-`base_link` transform changes with every arm pose, so
  INTERFACES §8.4's eye-in-hand capture contract requires the joint state *at the capture instant*,
  not just once.
- `docs/INTERFACES.md#6.2` / ADR-012 §6.2 require `camera_link → camera_color_optical_frame` to be
  "read from `realsense2_camera`'s own published static TF, never hand-built from the metadata,"
  because that transform composes a real per-device translation (`camera_link → camera_color_frame`)
  with a fixed mechanical→optical rotation, and only the ROS driver's TF tree, not the
  `rs2_get_extrinsics` metadata captured per frame, carries that composed value.
- `docs/INTERFACES.md#8.4` separately says the same factor may be "taken from the driver's TF (§6.2)
  **or the device record**." Read on its own, "device record" could be misread as license to derive
  the transform from the per-frame extrinsics metadata that CAM-01/CAM-02 already capture — which
  would directly contradict §6.2's "never hand-built from the metadata."

Those two clauses are not in tension once "device record" is read correctly: it means a *previously
captured* `driver_tf` value (one earlier TF pull from `realsense2_camera`, recorded once per mount and
reused thereafter) carried forward into a capture's record, not a different, metadata-derived way to
produce the same number. §8.4 is naming where a record stores the §6.2 value for reuse; it is not
offering a second method for obtaining it. This ADR makes that reading explicit so CAM-05.2 does not
reopen it.

## Decision

**A continuously-running `realsense2_camera` driver/topic set is not needed for routine capture.**
`crackvision.realsense_capture` / `crackvision.recording` (direct `pyrealsense2`, non-ROS) remain the
sole live-capture path for REQ-CAM-2 (aligned colour/depth, timestamps, intrinsics, depth scale,
extrinsics, metadata). Nothing under `src/crackvision` depends on ROS.

ROS 2 is required at exactly two narrow points, both one-shot or bridge-only, never a standing node in
the default runtime path:

1. **One-time-per-mount static optical-frame TF capture.** §6.2 forbids hand-building
   `camera_link → camera_color_optical_frame` from per-frame extrinsics metadata, and only
   `realsense2_camera`'s published static TF carries the real composed value. This must be pulled from
   the ROS driver's TF tree once per physical mount (re-run only if the mount is disturbed), then
   reused as the `driver_tf` "device record" referenced by §8.4 for every subsequent capture — never
   re-derived from metadata.
2. **Joint-state-at-capture-instant bridge.** Because the mount is eye-in-hand
   (`docs/calibration/PLAN.md#0`), the arm pose at the moment of each capture is part of the
   `base_link` lift chain (§8.4: `FK(q) · T_gripper_link_camera_link · T_camera_link_camera_color_optical_frame`).
   A fixed eye-to-hand mount would not need this per capture; eye-in-hand does. This is a thin `rclpy`
   read of `/joint_states` taken alongside each `pyrealsense2` capture — it does not touch camera
   topics at all.

What CAM-05.2 builds:
- `capture_optical_tf`: a one-shot ROS 2 script that reads `realsense2_camera`'s static TF and writes
  `TF.json {xyz_m, quat_xyzw}` (ROS quaternion order, metres/radians per §6.2), matching the
  `--optical-tf` input already specified by GEOM-08.4's `crackvision.capture_record` CLI.
- `capture_joint_state`: a one-shot ROS 2 script that reads `/joint_states` and writes
  `JS.json {joint_names, positions_rad, stamp_ns, stamp_source}`, matching the `--joint-state` input
  already specified by that same CLI.

What CAM-05.2 explicitly does not build:
- No `realsense2_camera` launch file in this repo's default runtime path.
- No continuous `/camera/color/image_raw`-style topics or subscribers.
- No changes to `crackvision.realsense_capture` or `crackvision.recording`.
- No code under `src/crackvision` that imports `rclpy` or any ROS package.

Until an operator actually runs `capture_optical_tf` against hardware, `docs/INTERFACES.md`§9's
default optical source stays `nominal_d405` (CAD prior, flagged) and `execution_eligible` is `false`.
That is the intended, safe default — not a gap for CAM-05.2 to close.

This ADR elaborates ADR-012 §6.2 and `docs/INTERFACES.md`§8.4; it does not contradict or amend either.

## Consequences

This keeps the ROS surface minimal: two short-lived scripts invoked around a capture, not a
subscription-based pipeline to keep alive, version, or debug. It makes the TF/joint-state inputs to
`crackvision.capture_record` concrete and attributable to a named tool. It forecloses building any
camera-data path through ROS topics — if a future need (e.g. live visualization) wants that, it is a
new decision, not an extension of CAM-05.2's two scripts.

## Revisit when

The mount type changes from eye-in-hand to eye-to-hand (reopening whether per-capture joint state is
needed at all — see ADR-014 "Revisit when"), or a concrete requirement appears for continuous/streamed
ROS camera topics (e.g. live teleoperation or a ROS-based perception consumer) that the two one-shot
scripts cannot satisfy.

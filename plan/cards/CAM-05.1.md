# CAM-05.1 — Decision: scope of ROS 2 camera integration (ADR-015)

```json card
{
  "kind": "decision",
  "depends_on": [
    "GEOM-03",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-CAM-2"
  ],
  "scope": {
    "write": [
      "docs/adr/015-camera-ros-integration-scope.md",
      "docs/COMPLETION_LOG.md"
    ]
  },
  "inputs": [
    "docs/adr/012-frames-and-conventions.md",
    "docs/INTERFACES.md#6.2",
    "docs/INTERFACES.md#8.4",
    "docs/calibration/PLAN.md#0",
    "plan/cards/GEOM-08.4.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "exists",
        "cmd": "test -f docs/adr/015-camera-ros-integration-scope.md",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "content",
        "cmd": "grep -q 'nominal_d405' docs/adr/015-camera-ros-integration-scope.md && grep -q 'driver_tf' docs/adr/015-camera-ros-integration-scope.md && grep -q 'camera_color_optical_frame' docs/adr/015-camera-ros-integration-scope.md && grep -q 'CAM-05.2' docs/adr/015-camera-ros-integration-scope.md",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "completion-log",
        "cmd": "grep -q 'CAM-05.1' docs/COMPLETION_LOG.md",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Answers the CAM-05 title's question explicitly: a continuously-running realsense2_camera driver/topic set is NOT needed for routine capture — crackvision.realsense_capture / crackvision.recording (pyrealsense2, non-ROS, CAM-01/CAM-02) remain the sole live-capture path.",
      "Names the exactly two narrow points where ROS is required, and why, given the confirmed eye-in-hand mount (docs/calibration/PLAN.md §0): (a) a one-time-per-mount capture of the static T_camera_link_camera_color_optical_frame transform from realsense2_camera's published TF tree, because ADR-012 §6.2 / INTERFACES §6.2 forbids hand-building it from the per-frame rs2_get_extrinsics metadata; (b) a joint-state-at-capture-instant bridge (ADR-014 eye-in-hand consequence, INTERFACES §8.4) reading /joint_states via rclpy alongside each pyrealsense2 capture, since the camera moves with the arm.",
      "Explicitly reconciles the apparent tension between ADR-012 §6.2 ('must be read from realsense2_camera's own published static TF, never hand-built from the metadata') and INTERFACES §8.4's 'taken from the driver's TF ... or the device record': the 'device record' phrase means a previously-captured driver_tf value recorded into a capture, not license to derive the transform from raw per-frame extrinsics metadata.",
      "States that CAM-05.2 delivers two small ROS 2 scripts (capture_optical_tf, capture_joint_state) producing TF.json {xyz_m, quat_xyzw} and JS.json {joint_names, positions_rad, stamp_ns, stamp_source} matching the --optical-tf / --joint-state input formats already specified in GEOM-08.4's crackvision.capture_record CLI, so no code under src/crackvision needs to depend on ROS.",
      "States plainly that until an operator runs the driver_tf capture on real hardware, INTERFACES §9's default optical source stays nominal_d405 and execution_eligible is false — this is the intended, safe default, not a gap to fix here.",
      "Follows the docs/adr/000-template.md structure (Context/Decision/Consequences/Revisit when), sets Status: accepted, Supersedes: —, and notes it elaborates ADR-012 §6.2 and INTERFACES §8.4 without contradicting either.",
      "One entry appended to docs/COMPLETION_LOG.md for CAM-05.1; no other file touched beyond the scope list."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 3,
    "context": 3,
    "consequence": 3,
    "task_class": "decision-doc",
    "local_ok": false
  },
  "track": "camera",
  "id": "CAM-05.1",
  "parent": "CAM-05",
  "title": "Decision: scope of ROS 2 camera integration (ADR-015)",
  "outcome": "ADR-015 settles, in writing, that CAM-05 does not stand up a continuously-running realsense2_camera driver; it defines the exact two narrow ROS touchpoints (static optical-frame TF capture, joint-state-at-capture bridge) that CAM-05.2 implements, and reconciles wording in ADR-012 §6.2 and INTERFACES §8.4 that could otherwise be read as contradictory."
}
```

Write `docs/adr/015-camera-ros-integration-scope.md` following the `000-template.md` shape. Do not edit `docs/INTERFACES.md`, `docs/adr/012-frames-and-conventions.md` or any GEOM-08 card — this ADR elaborates them, it does not change them. Cite `docs/calibration/PLAN.md#0` (eye-in-hand confirmed 2026-09-30) as the reason the joint-state bridge is required at all: a fixed (eye-to-hand) mount would not need it.

Keep it short — this is a scope decision, not a new contract. A few paragraphs plus a short bullet list of "what CAM-05.2 builds" (a static optical-frame TF capture tool, a joint-state capture tool) and "what CAM-05.2 explicitly does not build" (no realsense2_camera launch file in this repo's default runtime path, no continuous /camera/color/image_raw-style topics, no changes to crackvision.realsense_capture or crackvision.recording) is enough.

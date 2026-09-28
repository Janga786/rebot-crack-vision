# GEOM-01.R1 — Repair GEOM-01: tool0/TCP are not mapped to the actual robot frames; the claim of cons

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-GEOM-1",
    "REQ-GEOM-2"
  ],
  "scope": {
    "write": [
      "docs/INTERFACES.md",
      "docs/adr/012-frames-and-conventions.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0.5",
    "docs/INTERFACES.md#3.10"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "adr",
        "cmd": "test -f docs/adr/012-frames-and-conventions.md"
      }
    ],
    "criteria": [
      "The ADR and INTERFACES §6.1 state which existing URDF link `tool0` refers to (e.g. `gripper_link` or `link6`), or say that `tool0` is a new fixed alias frame and which card publishes it and from which parent. They state how `TCP` relates to the existing MoveIt `gripper_tcp` frame (same frame or a renamed one, and the prior offset -0.0443 m along gripper_link +X as prior evidence). The false 'consistent with' claim is corrected, and `T_tool0_TCP` names frames that will exist in TF.",
      "The ADR lists the intermediate `camera_color_frame`, or otherwise states that camera_link → camera_color_optical_frame = (per-device depth→colour extrinsic from metadata/driver) · (fixed optical rotation). It no longer claims the whole transform is translation-free and device-independent.",
      "All acceptance criteria of GEOM-01 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 5
  },
  "track": "calibration",
  "priority": 95,
  "repairs": "GEOM-01",
  "findings": [
    "b74d31f9bdb7fa95",
    "5b0364b063e71dba"
  ],
  "id": "GEOM-01.R1",
  "title": "Repair GEOM-01: tool0/TCP are not mapped to the actual robot frames; the claim of cons",
  "parent": "L-GEOM",
  "outcome": "Resolve the review findings on GEOM-01 while every acceptance criterion of GEOM-01 still holds."
}
```

Repair work generated deterministically from review attempt `00004-GEOM-01-review` of `GEOM-01`.
Original card: `plan/cards/GEOM-01.md` — its acceptance criteria must still hold.

## Finding 1 [major] tool0/TCP are not mapped to the actual robot frames; the claim of consistency with the MoveIt config is false
- Evidence: ADR-012 Frames: 'Named per REP-103 ... consistent with the ROS 2 / MoveIt config already present at `~/rebot_ws/src/rebotarm_moveit_config`', and '`tool0` | The kinematic chain's last link, per the URDF/MoveIt convention'. In fact the links in ~/rebot_ws/src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf are base_link, link1..link6, gripper_link, gripper_left and gripper_right (`grep -c tool0` → 0, and no file under ~/rebot_ws/src mentions tool0). rebotarm.srdf:4 has `<chain base_link="base_link" tip_link="gripper_tcp"/>`, and rebotarm.urdf.xacro:8-12 defines `gripper_tcp` as a fixed translation `xyz="-0.0443 0 0"` from `gripper_link`. The ADR instead names a new frame `TCP` and a transform `T_tool0_TCP`, neither of which exists in TF.
- Consequence: A frames ADR exists so GEOM-02…07 can use frame names without guessing. As written, a tf2 lookup of `tool0` or `TCP` fails. Implementers must guess whether tool0 means link6 or gripper_link, and whether TCP means the existing gripper_tcp. That is the silent-convention ambiguity this card is supposed to eliminate. GEOM-06/07 could also calibrate against a different parent link than MoveIt plans with.
- Affected: docs/adr/012-frames-and-conventions.md, docs/INTERFACES.md
- Acceptance condition: The ADR and INTERFACES §6.1 state which existing URDF link `tool0` refers to (e.g. `gripper_link` or `link6`), or say that `tool0` is a new fixed alias frame and which card publishes it and from which parent. They state how `TCP` relates to the existing MoveIt `gripper_tcp` frame (same frame or a renamed one, and the prior offset -0.0443 m along gripper_link +X as prior evidence). The false 'consistent with' claim is corrected, and `T_tool0_TCP` names frames that will exist in TF.

## Finding 2 [minor] camera_link → camera_color_optical_frame wrongly described as rotation-only and not per-device
- Evidence: ADR-012: '`camera_link` → `camera_color_optical_frame` is a fixed rotation (no translation) ... It is not calibrated per-device; it is a convention.' This contradicts the repo's own contract INTERFACES §3.10 item 7, which records a per-device `"extrinsics_depth_to_color":{"rotation":[9 floats],"translation":[3 floats]}`. The realsense2_camera node installed here builds its static TFs from stream-profile extrinsics (symbol `BaseRealSenseNode::calcAndAppendTransformMsgs(stream_profile, stream_profile)` in /opt/ros/humble/lib/librealsense2_camera.so). In ROS convention, only camera_color_frame → camera_color_optical_frame is rotation-only; camera_link → camera_color_frame carries the factory depth→colour extrinsic.
- Consequence: A downstream card that composes a mount/CAD transform through camera_link with a pure rotation drops the depth→colour offset, which may be small for the D405. The result is a biased T_base_link_camera_color_optical_frame, and nothing raises an error.
- Affected: docs/adr/012-frames-and-conventions.md
- Acceptance condition: The ADR lists the intermediate `camera_color_frame`, or otherwise states that camera_link → camera_color_optical_frame = (per-device depth→colour extrinsic from metadata/driver) · (fixed optical rotation). It no longer claims the whole transform is translation-free and device-independent.

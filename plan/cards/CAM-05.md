# CAM-05 — ROS 2 camera integration (only if calibration/workflow needs it)

```json card
{
  "kind": "branch",
  "depends_on": [
    "GEOM-03"
  ],
  "requirements": [
    "REQ-CAM-2"
  ],
  "refine_after": [
    "GEOM-03"
  ],
  "track": "camera",
  "id": "CAM-05",
  "title": "ROS 2 camera integration (only if calibration/workflow needs it)",
  "parent": "L-CAM",
  "outcome": "Decide and, if needed, provide aligned camera topics for ROS consumers."
}
```

Draft: the core pipeline is pure Python; add realsense2_camera launch only if GEOM-03 or OPS-01 needs ROS topics.

## ADR-014 guidance (technical-lead recovery, 2026-09-30)
- Eye-in-hand consequence (INTERFACES §8.4): any capture that will be lifted to 3D must record the arm joint state
  (and end_effector.yaml sha256) at the capture instant. If ROS topics are used, capture /joint_states alongside
  the aligned images; the camera frames come from realsense2_camera TF (the URDF only places camera_link).

## Decomposed 2026-09-30T14:00:11Z by L-CAM (claude-sonnet-5, high)
Children: CAM-05.1, CAM-05.2

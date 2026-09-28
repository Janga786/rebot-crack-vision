# CAM-05 — ROS 2 camera integration (only if calibration/workflow needs it)

```json card
{
  "kind": "decision",
  "depends_on": [
    "GEOM-03"
  ],
  "requirements": [
    "REQ-CAM-2"
  ],
  "spec_state": "draft",
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

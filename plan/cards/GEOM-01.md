# GEOM-01 — Frames, units and pixel conventions (ADR-012 + geometry contract)

```json card
{
  "kind": "decision",
  "depends_on": [
    "ARCH-01"
  ],
  "requirements": [
    "REQ-GEOM-1",
    "REQ-GEOM-2"
  ],
  "scope": {
    "write": [
      "docs/adr/012-frames-and-conventions.md",
      "docs/INTERFACES.md"
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
      "Frames per REP-103/105: base_link, tool0/TCP, camera_link, camera_color_optical_frame (z forward, x right, y down)",
      "Transform naming T_a_b maps points from b to a; quaternion order stated (ROS xyzw); units m/rad",
      "Pixel convention: (row, col) in files; (u, v) = (col, row) for projection; pixel-centre convention taken from librealsense rsutil.h (cite the source line) and matched by GEOM-02 tests",
      "Depth: uint16 × depth_scale_m_per_unit from metadata; invalid = 0 or outside the documented valid band",
      "Boresight/TCP definitions reference the presentation's measured gripper axis as PRIOR evidence only, pending GEOM-07",
      "Appended as a new numbered INTERFACES section; no existing section edited"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 5
  },
  "track": "calibration",
  "priority": 65,
  "id": "GEOM-01",
  "title": "Frames, units and pixel conventions (ADR-012 + geometry contract)",
  "parent": "L-GEOM",
  "outcome": "One authoritative definition of every frame, transform name, unit and pixel convention used downstream."
}
```



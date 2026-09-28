# MOT-03 — Planning scene from config (nominal until measured)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-02"
  ],
  "requirements": [
    "REQ-MOT-1"
  ],
  "scope": {
    "write": [
      "config/scene/**",
      "ros2_ws/src/crackvision_motion/**",
      "docs/motion/SCENE.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 1500
      },
      {
        "id": "scene-test",
        "cmd": "bash scripts/ros/test_scene.sh",
        "timeout_s": 900
      }
    ],
    "criteria": [
      "Scene objects and ACM from YAML; each value tagged measured|nominal with source",
      "Test verifies collision objects appear in the planning scene and a colliding goal is rejected in mock",
      "Commissioning executor (MOT-05) will refuse scenes with nominal values — stated and tested here as a function"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4
  },
  "track": "simulation",
  "priority": 40,
  "id": "MOT-03",
  "title": "Planning scene from config (nominal until measured)",
  "parent": "L-MOTION",
  "outcome": "Collision scene (table, specimen, camera mount) loaded from config; UNMEASURED values block commissioning."
}
```



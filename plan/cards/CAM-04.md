# CAM-04 — Real recording session over a test surface

```json card
{
  "kind": "hardware",
  "depends_on": [
    "CAM-01",
    "CAM-02",
    "CAM-03"
  ],
  "requirements": [
    "REQ-CAM-2",
    "REQ-CAM-3"
  ],
  "scope": {
    "write": [
      "evidence/camera/recording_manifest.json",
      "docs/camera/RECORDING_SESSION_1.md",
      "data/recordings/**"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "manifest",
        "cmd": "python3 -m json.tool evidence/camera/recording_manifest.json >/dev/null"
      }
    ],
    "criteria": [
      "Sessions at ≥2 distances in the 10–40 cm band; validator passes; checksums in the manifest (data itself stays git-ignored)"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 3
  },
  "resources": [
    "usb"
  ],
  "hardware": [
    "d405_connected",
    "d405_test_scene_ready"
  ],
  "track": "camera",
  "priority": 40,
  "id": "CAM-04",
  "title": "Real recording session over a test surface",
  "parent": "L-CAM",
  "outcome": "A validated real RGB-D recording exists for replay-based integration tests."
}
```



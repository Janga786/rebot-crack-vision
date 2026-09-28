# CAM-02 — Lossless RGB-D recording + deterministic replay

```json card
{
  "kind": "impl",
  "depends_on": [
    "TC-013"
  ],
  "requirements": [
    "REQ-CAM-2",
    "REQ-CAM-3"
  ],
  "scope": {
    "write": [
      "src/crackvision/recording.py",
      "tests/test_recording.py",
      "docs/INTERFACES.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#3.10",
    "docs/INTERFACES.md#0.5"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "pytest",
        "cmd": "./env.sh pytest tests/test_recording.py -q"
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q",
        "timeout_s": 1200
      }
    ],
    "criteria": [
      "RecordingWriter/RecordingReader + ReplaySource yield the same frame type as crackvision.realsense_capture's live source",
      "session.json: schema_version, device {serial, firmware, usb_type}, stream configs, colour intrinsics (fx, fy, ppx, ppy, model, coeffs, width, height), depth_scale_m_per_unit read from the device, depth→colour extrinsics, alignment target, frame count",
      "Per-frame sidecar: frame index, device timestamps + timestamp domain for colour and depth, host monotonic ns, frame numbers",
      "Round trip is bit-exact (uint16 depth, uint8 RGB); no resize/crop (frame invariant)",
      "A new numbered section (recording contract) is APPENDED to docs/INTERFACES.md; existing sections untouched"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3
  },
  "track": "camera",
  "id": "CAM-02",
  "title": "Lossless RGB-D recording + deterministic replay",
  "parent": "L-CAM",
  "outcome": "Aligned RGB-D sessions can be recorded losslessly and replayed through the same interface as live capture."
}
```



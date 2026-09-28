# CAM-01 — First D405 connection: discovery, firmware, USB link

```json card
{
  "kind": "hardware",
  "depends_on": [
    "TC-013",
    "HOST-03",
    "CAM-03"
  ],
  "requirements": [
    "REQ-CAM-1"
  ],
  "scope": {
    "write": [
      "evidence/camera/d405_discovery.json",
      "docs/camera/D405_FIRST_CONNECTION.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "json",
        "cmd": "python3 -m json.tool evidence/camera/d405_discovery.json >/dev/null"
      }
    ],
    "criteria": [
      "Serial, firmware, product line and USB type recorded from the device; USB 3.x required",
      "Firmware compared against the librealsense release notes for the installed 2.57 (ROS) and 2.58 (pip) — any update is an operator decision, never done by the agent",
      "30-frame capture validated by scripts/validate_recording.py; failures explained"
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
    "d405_connected"
  ],
  "track": "camera",
  "id": "CAM-01",
  "title": "First D405 connection: discovery, firmware, USB link",
  "parent": "L-CAM",
  "outcome": "The real D405 is enumerated with recorded serial, firmware, USB 3 link and a validated short capture."
}
```



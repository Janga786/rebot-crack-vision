# HOST-03 — RealSense udev rules via a reviewed privileged action

```json card
{
  "kind": "impl",
  "depends_on": [
    "TC-012"
  ],
  "requirements": [
    "REQ-HOST-3",
    "REQ-CAM-1"
  ],
  "scope": {
    "write": [
      "host/udev/99-realsense-libusb.rules",
      "docs/host/DEVICE_ACCESS.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "installed",
        "cmd": "cmp host/udev/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules"
      }
    ],
    "criteria": [
      "Rule file content is the official librealsense rule for the installed version (provenance recorded in a header comment and DEVICE_ACCESS.md)",
      "Installed with exactly one privileged install command (then udevadm reload/trigger as separate requests)",
      "DEVICE_ACCESS.md documents rollback"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 1,
    "context": 2,
    "consequence": 4
  },
  "track": "camera",
  "priority": 35,
  "id": "HOST-03",
  "title": "RealSense udev rules via a reviewed privileged action",
  "parent": "L-HOST",
  "outcome": "Non-root access to the D405 is granted by the official librealsense udev rule, installed through the Opus-reviewed privileged path."
}
```

Find the canonical rule (e.g. under /opt/ros/humble/share/librealsense2/ or the librealsense GitHub tag matching the installed
2.57/2.58 versions — download unprivileged into the repo). Stage it at host/udev/99-realsense-libusb.rules. Then return status
"blocked" with a privileged_request, one at a time, in this order (each resumes you after execution):
1. argv ["/usr/bin/install","-m","0644","-o","root","-g","root","<ABSOLUTE repo path>/host/udev/99-realsense-libusb.rules","/etc/udev/rules.d/99-realsense-libusb.rules"], verify_argv ["/usr/bin/test","-f","/etc/udev/rules.d/99-realsense-libusb.rules"]
2. ["/usr/bin/udevadm","control","--reload-rules"]   3. ["/usr/bin/udevadm","trigger"]
Never request a shell, never write anywhere else under /etc.

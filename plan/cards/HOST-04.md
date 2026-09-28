# HOST-04 — Arm USB adapter access via least-privilege udev rule

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-01"
  ],
  "requirements": [
    "REQ-HOST-3"
  ],
  "scope": {
    "write": [
      "host/udev/99-rebot-arm.rules",
      "docs/host/DEVICE_ACCESS.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "installed",
        "cmd": "cmp host/udev/99-rebot-arm.rules /etc/udev/rules.d/99-rebot-arm.rules"
      }
    ],
    "criteria": [
      "VID:PID come from a cited source (vendor SDK/docs, ~/rebot_ws preflight, or lsusb with the adapter attached) — never guessed",
      "Rule is specific (ATTRS{idVendor}/{idProduct}), MODE 0660, GROUP plugdev, stable SYMLINK; installed via the privileged path",
      "No usermod/dialout change"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 3,
    "context": 2,
    "consequence": 4
  },
  "track": "physical",
  "priority": 30,
  "id": "HOST-04",
  "title": "Arm USB adapter access via least-privilege udev rule",
  "parent": "L-HOST",
  "outcome": "The B601-DM USB adapter is accessible to the user through a VID:PID-specific udev rule (group plugdev), not broad group changes."
}
```

Read ~/rebot_ws (deploy/preflight.sh, rebotarm_bringup, vendor SDK in third_party/) and ~/rebot_lerobot docs to establish the
adapter's VID:PID and device node. If no source states it, return status "blocked" with blocker type "hardware" asking the
operator to connect the adapter (prerequisite `arm_usb_connected`) so `lsusb` can be read. Then stage host/udev/99-rebot-arm.rules
and request the install + reload + trigger exactly as HOST-03 does.

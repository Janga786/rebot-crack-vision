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
  "hardware": [
    "arm_usb_connected"
  ],
  "track": "physical",
  "priority": 30,
  "id": "HOST-04",
  "title": "Arm USB adapter access via least-privilege udev rule",
  "parent": "L-HOST",
  "outcome": "The B601-DM USB adapter is accessible to the user through a VID:PID-specific udev rule (group plugdev), not broad group changes."
}
```

## Re-specified 2026-09-30 (technical-lead recovery)
Now gated on the operator prerequisite `arm_usb_connected` (declare with `claude-auto hw declare arm_usb_connected`
once the B601-DM USB/CAN adapter is plugged in). The adapter is NOT connected; its VID:PID must never be guessed
(attempt 00130 found only XXXX/YYYY placeholders in ~/rebot_ws docs and no cited value elsewhere).

When dispatched: read the adapter's idVendor/idProduct read-only from sysfs (e.g. /sys/bus/usb/devices/*/idVendor,
idProduct, product, serial) or the udev database (`udevadm info -q property -p /sys/class/tty/<node>`); the sandbox
never exposes arm serial/CAN device nodes, by design. Cite the exact output (or operator evidence added with
`claude-auto evidence add HOST-04 <lsusb-output>`). Stage host/udev/99-rebot-arm.rules (ATTRS idVendor/idProduct,
MODE 0660, GROUP plugdev, a stable SYMLINK such as rebot_arm), then request the install and the reload exactly as
HOST-03 does (install with a cmp verify_argv, then `udevadm control --reload-rules`; no blanket trigger, replug
instead). No usermod/dialout change.

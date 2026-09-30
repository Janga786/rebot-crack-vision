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
      "host/udev/99-crackvision-d405.rules",
      "host/udev/99-realsense-libusb.rules",
      "docs/host/DEVICE_ACCESS.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "installed",
        "cmd": "cmp host/udev/99-crackvision-d405.rules /etc/udev/rules.d/99-crackvision-d405.rules && test \"$(stat -c '%a %U %G' /etc/udev/rules.d/99-crackvision-d405.rules)\" = '644 root root'"
      },
      {
        "id": "least-privilege",
        "cmd": "bash -c 'f=host/udev/99-crackvision-d405.rules; r=$(grep -v -E \"^[[:space:]]*(#|$)\" $f); test \"$(echo \"$r\" | wc -l)\" -eq 1 && echo \"$r\" | grep -q \"ATTRS{idVendor}==\\\"8086\\\"\" && echo \"$r\" | grep -q \"ATTRS{idProduct}==\\\"0b5b\\\"\" && echo \"$r\" | grep -q \"MODE:=\\\"0660\\\"\" && echo \"$r\" | grep -q \"GROUP:=\\\"plugdev\\\"\" && ! echo \"$r\" | grep -q -E \"RUN|0666|0777|chmod\"'"
      },
      {
        "id": "broad-copy-removed",
        "cmd": "test ! -e host/udev/99-realsense-libusb.rules"
      }
    ],
    "criteria": [
      "The staged rule grants exactly what is needed: one active line, derived from the official librealsense v2.57.7 99-realsense-libusb.rules D405 line (8086:0b5b), with MODE 0660 and GROUP plugdev instead of 0666, no RUN hooks, no other device ids (no DFU/recovery ids, no other cameras). SUBSYSTEMS/ATTRS matching covers the USB node and its video4linux children (RSUSB and V4L2 backends). Provenance (upstream URL, tag, upstream sha256, the exact upstream line it derives from, why it is narrowed) is recorded in a header comment and in DEVICE_ACCESS.md. The broad official copy staged by attempt 00128 is removed from host/udev/.",
      "Installed with exactly one privileged install request, then one separate `udevadm control --reload-rules` request; no blanket `udevadm trigger` (no D405 is attached; the rule applies on the next plug-in, documented). verify_argv checks the installed content (cmp), not mere existence.",
      "DEVICE_ACCESS.md documents install, verification (content + owner/mode + replug + ./env.sh python scripts/check_realsense.py once a D405 is attached) and rollback, and states that no group membership, chmod or other system change is made (boosterk1 is already in plugdev)."
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

## Re-specified 2026-09-30 (technical-lead recovery)
The privileged reviewer (Opus xhigh) rightly denied PRIV-1790720360-HOST-03: the old criterion 1 demanded the
official rules file verbatim, which sets MODE 0666 on ~50 Intel ids (incl. DFU recovery ids) and runs root
`chmod -R 0777` hooks - far broader than one D405 for a user already in plugdev (REQ-HOST-3: least privilege).
The operator approves the narrow privileged udev workflow; the rule itself must be minimal.

1. Download the official rules file unprivileged (https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.57.7/config/99-realsense-libusb.rules,
   sha256 c610c3379d360006261b0fc26614316323551f5b11ffe81e9ebf48c4c0a43ce8 per attempt 00128) only to cite it.
   Its D405 line is `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5b", MODE:="0666", GROUP:="plugdev"`.
2. Stage host/udev/99-crackvision-d405.rules = provenance header comments + exactly:
   `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5b", MODE:="0660", GROUP:="plugdev"`
   `git rm` host/udev/99-realsense-libusb.rules (the broad copy must not stay where it could be installed).
3. Return status "blocked" with ONE privileged_request at a time (each executed approval resumes you):
   a. argv ["/usr/bin/install","-m","0644","-o","root","-g","root","<ABSOLUTE repo path>/host/udev/99-crackvision-d405.rules","/etc/udev/rules.d/99-crackvision-d405.rules"],
      verify_argv ["/usr/bin/cmp","<ABSOLUTE repo path>/host/udev/99-crackvision-d405.rules","/etc/udev/rules.d/99-crackvision-d405.rules"],
      rollback: remove that one file, then reload rules.
   b. argv ["/usr/bin/udevadm","control","--reload-rules"], verify_argv ["/usr/bin/test","-f","/etc/udev/rules.d/99-crackvision-d405.rules"].
   Never request a shell, never touch anything else under /etc, no usermod/chmod.
4. Update docs/host/DEVICE_ACCESS.md (install / verify / rollback / replug note / why narrowed).

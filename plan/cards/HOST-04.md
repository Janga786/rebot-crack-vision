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
  "inputs": [
    "evidence/operator/HOST-04/arm_adapter_usb_ids_2026-10-10.md",
    "host/udev/99-crackvision-d405.rules",
    "docs/host/DEVICE_ACCESS.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "installed",
        "cmd": "cmp host/udev/99-rebot-arm.rules /etc/udev/rules.d/99-rebot-arm.rules"
      },
      {
        "id": "least-privilege",
        "cmd": "bash -c 'f=host/udev/99-rebot-arm.rules; r=$(grep -v -E \"^[[:space:]]*(#|$)\" $f); test \"$(echo \"$r\" | wc -l)\" -eq 1 || { echo \"need exactly one active line\"; exit 1; }; for t in \"SUBSYSTEM==\\\"tty\\\"\" \"ATTRS{idVendor}==\\\"2e88\\\"\" \"ATTRS{idProduct}==\\\"4603\\\"\" \"MODE:=\\\"0660\\\"\" \"GROUP:=\\\"plugdev\\\"\" \"SYMLINK+=\\\"ttyREBOT_ARM\\\"\" \"ENV{ID_MM_DEVICE_IGNORE}=\\\"1\\\"\"; do echo \"$r\" | grep -qF \"$t\" || { echo \"missing $t\"; exit 1; }; done; ! echo \"$r\" | grep -q -E \"RUN|0666|0777|chmod|dialout|OWNER\"'"
      },
      {
        "id": "documented",
        "cmd": "bash -c 'f=docs/host/DEVICE_ACCESS.md; for s in 2e88 4603 99-rebot-arm.rules ttyREBOT_ARM ID_MM_DEVICE_IGNORE Rollback 8086; do grep -qF \"$s\" $f || { echo \"missing $s\"; exit 1; }; done'"
      }
    ],
    "criteria": [
      "VID:PID 2e88:4603 is cited from evidence/operator/HOST-04/arm_adapter_usb_ids_2026-10-10.md, which holds the journalctl -k excerpts of 2026-10-10 14:06:10 and 15:51:48 (HDSC 'CDC Device', SN 00000000050C) and the operator's lsusb statement. It is never guessed.",
      "host/udev/99-rebot-arm.rules has a provenance header and exactly one active line: SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"2e88\", ATTRS{idProduct}==\"4603\", MODE:=\"0660\", GROUP:=\"plugdev\", SYMLINK+=\"ttyREBOT_ARM\", ENV{ID_MM_DEVICE_IGNORE}=\"1\". It has no RUN hook, no 0666/0777, no OWNER and no dialout. The header explains why the symlink name must begin with tty and why ModemManager is told to ignore the port (body below).",
      "It is installed through the privileged path with exactly two requests, one at a time, as HOST-03 did. (a) /usr/bin/install -m 0644 -o root -g root <absolute repo path>/host/udev/99-rebot-arm.rules /etc/udev/rules.d/99-rebot-arm.rules, with verify_argv /usr/bin/cmp <same two paths>. (b) /usr/bin/udevadm control --reload-rules, with verify_argv /usr/bin/test -f /etc/udev/rules.d/99-rebot-arm.rules. There is no udevadm trigger: the adapter is unplugged and the rule applies at the next plug-in. No shell is requested, nothing else under /etc is touched, and there is no usermod, gpasswd or chmod.",
      "docs/host/DEVICE_ACCESS.md gains an 'Arm USB/CAN adapter (HOST-04)' section; the existing D405 content stays unchanged. The section covers provenance, why it is narrowed, the install commands, verification (content cmp; owner/mode from a host shell, since the sandbox shows root as nobody; the operator's replug check `ls -l /dev/ttyACM0 /dev/ttyREBOT_ARM` expecting crw-rw---- root plugdev), rollback (remove the file, then reload) and the dialout note below.",
      "No group membership is changed. The operator's own dialout membership is reported, not altered."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 4
  },
  "hardware": [
    "arm_usb_connected"
  ],
  "track": "physical",
  "priority": 88,
  "id": "HOST-04",
  "title": "Arm USB adapter access via least-privilege udev rule",
  "parent": "L-HOST",
  "outcome": "The B601-DM USB adapter is accessible to the user through a VID:PID-specific udev rule (group plugdev), not broad group changes."
}
```

## Re-specified 2026-09-30 (technical-lead recovery)
This card is gated on the operator prerequisite `arm_usb_connected`, declared with
`claude-auto hw declare arm_usb_connected` once the B601-DM USB/CAN adapter is plugged in. At the time the adapter
was NOT connected, and its VID:PID must never be guessed. Attempt 00130 found only XXXX/YYYY placeholders in the
~/rebot_ws docs and no cited value anywhere else.

## Re-specified 2026-10-10 (overnight session, on the operator's instruction): VID:PID now cited

The operator connected the adapter on 2026-10-10 and the kernel identified it. See
`evidence/operator/HOST-04/arm_adapter_usb_ids_2026-10-10.md`:

| Field | Value |
|---|---|
| Device | `idVendor=2e88`, `idProduct=4603`, `HDSC` / `CDC Device`, serial `00000000050C` |
| Driver and node | `cdc_acm`, `/dev/ttyACM0` |
| Default permissions | `root:dialout 0660` (the operator has been using `sudo chmod 666` as a stop-gap) |

The adapter is unplugged overnight. Cite the evidence file; do not try to read sysfs for it. By design, the
sandbox never exposes arm serial devices.

### Why the symlink must start with `tty`: never `/dev/rebot_arm`

The vendor SDK chooses its transport from the channel string:

```python
if channel.startswith("/dev/tty"): Controller.from_dm_serial(channel, 921600)
else:                               Controller(channel)   # SocketCAN
```

That logic is at `third_party/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py:485-487`, and in
`actuator/arm.py:119-121` at SDK commit `8427281`, the API the ROS driver uses.

The vendor deploy scripts prefer `/dev/rebot_arm` when it exists (`~/rebot_ws/deploy/_env.sh:59`) and pass it as
the driver's `channel` (`deploy/run_real_demo.sh:120`, `deploy/preflight.sh:114`). The vendor runbook also
suggests that name (`docs/WORKSTATION_DEPLOY.md` §1a). A `/dev/rebot_arm` symlink would therefore silently switch
the driver to the SocketCAN path. `ttyREBOT_ARM` keeps the dm-serial transport, and `_env.sh`, which does not look
for it, still falls back to `/dev/ttyACM0`.

### Why `ENV{ID_MM_DEVICE_IGNORE}="1"`

ModemManager is active and enabled on this host. It examined the adapter at every plug-in on 2026-10-10.
`journalctl -u ModemManager` shows, at 14:06:13, 14:32:17 and 15:51:51:

```
couldn't check support for device '/sys/devices/pci0000:00/0000:00:14.0/usb1/1-7': not supported by any plugin
```

Port 1-7 is the adapter. Marking the port ignored keeps ModemManager from ever opening it or sending AT probes
into the USB-to-CAN bridge. The property grants nothing.

### Note on dialout, for DEVICE_ACCESS.md

On 2026-10-10 at 15:54 MDT, `/etc/group` was changed to list `boosterk1` in `dialout`. That takes effect at the
next login. It was the operator's own change. dialout grants access to every serial device; this rule grants
plugdev access to one VID:PID's tty. The operator decides whether to keep dialout. This card changes no group
membership.

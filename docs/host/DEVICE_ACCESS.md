# Device Access — Intel RealSense D405

## Purpose

Grants the logged-in user (`boosterk1`, already a member of the `plugdev`
group) non-root access to the RealSense D405 via a project-specific udev
rule, so `pyrealsense2` / `realsense2_camera` can open the device without
`sudo`. No group membership, `chmod`, or other system change is made beyond
installing this one rule file.

## Rule provenance

- File: `host/udev/99-crackvision-d405.rules`
- Derived from upstream: `https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.57.7/config/99-realsense-libusb.rules`
- Upstream tag: `v2.57.7`
- Upstream file sha256: `c610c3379d360006261b0fc26614316323551f5b11ffe81e9ebf48c4c0a43ce8`
- Upstream line this rule derives from (D405, idProduct `0b5b`):
  `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5b", MODE:="0666", GROUP:="plugdev"`

### Why narrowed

A privileged reviewer (Opus, xhigh effort) denied an earlier attempt
(PRIV-1790720360-HOST-03) that staged the verbatim upstream file: it sets
`MODE 0666` (world read/write) on ~50 Intel USB ids — including DFU/recovery
ids this project never touches — and runs root `chmod -R 0777` `RUN` hooks
on the sysfs tree. That is far broader than REQ-HOST-3 (least privilege)
allows for a single camera whose operator is already in `plugdev`.

The staged rule keeps exactly one active line: the D405 id (`8086:0b5b`),
with `MODE:="0660"` and `GROUP:="plugdev"` instead of `0666`, no other
device ids, and no `RUN` hook. `SUBSYSTEMS=="usb"` and `ATTRS{...}` are
udev's *ancestor-matching* keys: udev evaluates the rule for every device
event and matches if the device itself or any parent satisfies them. So the
rule applies both to the USB device node (`/dev/bus/usb/BBB/DDD`, used by
librealsense's RSUSB/libusb backend) and to its `video4linux` children
(`/dev/videoN`, used by the V4L2/uvcvideo backend), whose USB ancestor
carries `idVendor=8086`/`idProduct=0b5b`. No separate video4linux line is
needed. (The header comment in the rule file words this as the backends
"walking up" to the USB ancestor; the matching is done by udev as described
here. The installed file is intentionally left byte-identical to the staged
one so the `cmp` verification keeps holding.)

The former broad copy, `host/udev/99-realsense-libusb.rules` (staged by an
earlier attempt), has been removed from this directory so it cannot be
installed by mistake.

## Installation (privileged — requires operator/reviewer-approved sudo)

Exactly two privileged steps, run once:

```
/usr/bin/install -m 0644 -o root -g root \
  host/udev/99-crackvision-d405.rules \
  /etc/udev/rules.d/99-crackvision-d405.rules

/usr/bin/udevadm control --reload-rules
```

No `udevadm trigger` is run: no D405 is attached at install time, and
`trigger` would re-run against unrelated already-attached USB devices. The
new rule takes effect the next time the D405 is plugged in (or any time
after a reload if it is already plugged in and replugged).

## Verify

Content and ownership/mode of the installed rule:

```
cmp host/udev/99-crackvision-d405.rules /etc/udev/rules.d/99-crackvision-d405.rules
stat -c '%a %U %G' /etc/udev/rules.d/99-crackvision-d405.rules   # expect: 644 root root
```

Note: inside the claude-auto bubblewrap sandbox (unprivileged user
namespace) `stat` reports root-owned files as `nobody nogroup` and `id` does
not list `plugdev`, because host uid/gid 0 and supplementary groups are not
mapped into the namespace. Run the owner/mode check from a normal host shell;
there it reports `644 root root`. `cmp` (content) is valid in both.

Functional check once a D405 is physically attached (unplug/replug first so
the rule applies to that device node):

```
./env.sh python scripts/check_realsense.py
```

## Rollback

```
/usr/bin/rm /etc/udev/rules.d/99-crackvision-d405.rules
/usr/bin/udevadm control --reload-rules
```

This removes the least-privilege grant; the D405 falls back to whatever
default USB permissions apply (typically root-only access). No other system
state (group membership, other udev rules, file modes elsewhere) is
touched by install or rollback.

## Arm USB/CAN adapter (HOST-04)

### Purpose

Grants the logged-in user (`boosterk1`, already a member of `plugdev`)
non-root access to the B601-DM arm's USB2CAN serial bridge via a
project-specific, VID:PID-scoped udev rule, so the arm driver can open the
port without `sudo` or the operator's `chmod 666` stop-gap. No group
membership or other system change is made beyond installing this one rule
file.

### Provenance

The adapter is the Damiao USB2CAN serial bridge (dm-serial, 921600 baud) to
the B601-DM CAN bus. Its identity is cited, never guessed, from
`evidence/operator/HOST-04/arm_adapter_usb_ids_2026-10-10.md`, which
excerpts `journalctl -k` at 2026-10-10 14:06:10 and 15:51:48:

```
idVendor=2e88, idProduct=4603, HDSC "CDC Device", SerialNumber 00000000050C
cdc_acm 1-7:1.0: ttyACM0: USB ACM device
```

and the operator's own `lsusb` statement, "HDSC VID:PID 2e88:4603". The
default node permissions observed were `root:dialout 0660` on
`/dev/ttyACM0`.

- File: `host/udev/99-rebot-arm.rules`
- Active line: `SUBSYSTEM=="tty", ATTRS{idVendor}=="2e88", ATTRS{idProduct}=="4603", MODE:="0660", GROUP:="plugdev", SYMLINK+="ttyREBOT_ARM", ENV{ID_MM_DEVICE_IGNORE}="1"`

### Why narrowed

Only this one VID:PID (`2e88:4603`) is matched, with `MODE:="0660"` and
`GROUP:="plugdev"` (not world-writable `0666`, not `root:dialout`), no `RUN`
hook, no `OWNER`, and no change to the `dialout` group. The operator
already belongs to `plugdev`, so this grants access to the one arm adapter
without widening any existing group's reach, matching the D405 rule above
and REQ-HOST-3.

`SYMLINK+="ttyREBOT_ARM"` must start with `tty`: the vendor SDK
(`third_party/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py:485-487`,
`actuator/arm.py:119-121` at SDK commit `8427281`) selects the dm-serial
transport only when the channel string starts with `/dev/tty`, otherwise it
falls back to SocketCAN. The vendor deploy scripts
(`~/rebot_ws/deploy/_env.sh:59`, `deploy/run_real_demo.sh:120`,
`deploy/preflight.sh:114`) and runbook prefer a `/dev/rebot_arm` symlink if
one exists, which would silently switch the driver to the wrong transport;
`ttyREBOT_ARM` avoids that name, and `_env.sh` falls back to
`/dev/ttyACM0` when it does not find the name it looks for.

`ENV{ID_MM_DEVICE_IGNORE}="1"`: ModemManager is active on this host and
probed the adapter's port (`1-7`) at every plug-in on 2026-10-10
(`journalctl -u ModemManager` at 14:06:13, 14:32:17, 15:51:51: "couldn't
check support for device ... not supported by any plugin"). Marking the
port ignored stops ModemManager from opening it or sending AT probes into
the USB-to-CAN bridge; the property grants no access by itself.

### Installation (privileged — requires operator/reviewer-approved sudo)

Exactly two privileged steps, run once, one at a time:

```
/usr/bin/install -m 0644 -o root -g root \
  host/udev/99-rebot-arm.rules \
  /etc/udev/rules.d/99-rebot-arm.rules

/usr/bin/udevadm control --reload-rules
```

No `udevadm trigger` is run: the adapter is unplugged at install time, and
`trigger` would re-run against unrelated already-attached USB devices. The
rule takes effect the next time the adapter is plugged in.

### Verify

Content of the installed rule:

```
cmp host/udev/99-rebot-arm.rules /etc/udev/rules.d/99-rebot-arm.rules
```

Owner/mode of the installed rule file, and the device node permissions
after a replug, must be checked from a normal host shell, not from inside
the claude-auto sandbox: the sandbox's unprivileged user namespace reports
root-owned files as `nobody nogroup`, so an owner/mode check run there can
never show the real host state.

```
stat -c '%a %U %G' /etc/udev/rules.d/99-rebot-arm.rules   # expect: 644 root root
```

Operator's replug check, after unplugging and replugging the adapter:

```
ls -l /dev/ttyACM0 /dev/ttyREBOT_ARM
# expect: crw-rw---- 1 root plugdev ... /dev/ttyACM0
#         crw-rw---- 1 root plugdev ... /dev/ttyREBOT_ARM -> ttyACM0 (or similar)
```

### Rollback

```
/usr/bin/rm /etc/udev/rules.d/99-rebot-arm.rules
/usr/bin/udevadm control --reload-rules
```

This removes the least-privilege grant; the adapter falls back to whatever
default permissions the `cdc_acm` driver assigns (observed as
`root:dialout 0660`). No other system state is touched by install or
rollback.

### Note on `dialout`

On 2026-10-10 at 15:54 MDT the operator changed `/etc/group` themselves to
add `boosterk1` to `dialout`; that takes effect at the operator's next
login. This card does not make, request, or alter that change. `dialout`
grants access to every serial device on the host; this udev rule grants
`plugdev` access to exactly one VID:PID's tty node. Whether to keep
`dialout` membership is the operator's decision, reported here and not
acted on.

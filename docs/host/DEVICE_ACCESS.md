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

# Device Access — Intel RealSense D405

## Purpose

Grants the logged-in user non-root access to the RealSense D405 (and other
Intel RealSense USB devices) via the official librealsense udev rule, so
`pyrealsense2` / `realsense2_camera` can open the device without `sudo`.

## Rule provenance

- File: `host/udev/99-realsense-libusb.rules`
- Source: `https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.57.7/config/99-realsense-libusb.rules`
- librealsense tag: `v2.57.7`, matching the installed apt package
  `ros-humble-librealsense2 2.57.7-1jammy.20260324.115117` (the pip
  `pyrealsense2` wheel is 2.58.4; no udev rule changes exist between these
  minor versions upstream).
- Upstream file sha256: `c610c3379d360006261b0fc26614316323551f5b11ffe81e9ebf48c4c0a43ce8`
- Fetched: 2026-09-29
- The staged file carries this provenance as a header comment; the header is
  additive (comment lines only) and does not alter any rule semantics.

## Installation (privileged, done outside this repo's automation)

```
sudo install -m 0644 -o root -g root \
  host/udev/99-realsense-libusb.rules \
  /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

After installing, unplug and replug the D405 (or re-trigger udev) so the new
rule applies to the already-connected device node.

## Verify

```
cmp host/udev/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules
ls -l /dev/bus/usb/*/* | grep plugdev   # device group should be plugdev, mode 0666 after replug
./env.sh python scripts/check_realsense.py
```

## Rollback

```
sudo rm /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

This removes the least-privilege grant; the device falls back to whatever
default USB permissions apply (typically root-only, or group access if
another rule already grants it).

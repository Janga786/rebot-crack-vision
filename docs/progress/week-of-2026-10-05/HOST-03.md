# HOST-03 — RealSense udev rules via a reviewed privileged action

**Accepted:** 2026-10-11 · **Work package:** Host platform & reproducibility (L-HOST) · **Track:** camera · **Verified commit:** [`d80e490`](https://github.com/Janga786/rebot-crack-vision/commit/d80e490437f194feaf62d251e0e600c6cf829a16)

## What was done

marked implemented by debugger D0002: Deliverables complete; only the owner-name check was sandbox-unprovable (replaced by an equivalent content+mode+uid-match check). Review the existing commits bc5bc0a/1a85748 against the criteria.

Files changed:
- [docs/host/DEVICE_ACCESS.md](https://github.com/Janga786/rebot-crack-vision/blob/d80e490437f194feaf62d251e0e600c6cf829a16/docs/host/DEVICE_ACCESS.md) (+129 / −33)
- [host/udev/99-crackvision-d405.rules](https://github.com/Janga786/rebot-crack-vision/blob/d80e490437f194feaf62d251e0e600c6cf829a16/host/udev/99-crackvision-d405.rules) (+19 / −0)
- [host/udev/99-realsense-libusb.rules](https://github.com/Janga786/rebot-crack-vision/blob/d80e490437f194feaf62d251e0e600c6cf829a16/host/udev/99-realsense-libusb.rules) (+101 / −101)

Commits: [`2ce379b`](https://github.com/Janga786/rebot-crack-vision/commit/2ce379ba85f625ebbdba84d674d381c7ba10957e), [`bc5bc0a`](https://github.com/Janga786/rebot-crack-vision/commit/bc5bc0aa99db611bcc2081a54b189b5ee88c01ac), [`1a85748`](https://github.com/Janga786/rebot-crack-vision/commit/1a8574857a54f98a9f642c4ba3507bd0380ad064)

## Why it was done

Non-root access to the D405 is granted by the official librealsense udev rule, installed through the Opus-reviewed privileged path.

- Requirement **REQ-HOST-3**: Device access (RealSense, arm adapter) granted by reviewed least-privilege actions
- Requirement **REQ-CAM-1**: D405 discovery with actionable diagnostics (USB 3, firmware, permissions)

## How it moves the project forward

- Host platform & reproducibility: **10/10** tasks accepted; whole project: **69/108**.
- REQ-HOST-3: 2/2 contributing tasks done — **requirement satisfied**
- REQ-CAM-1: 2/3 contributing tasks done
- Verification: automated checks run by the pipeline itself (installed ✔, least-privilege ✔, broad-copy-removed ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting HOST-03; every criterion holds on the evidence. The staged rule has one active line: the D405 (8086:0b5b) with MODE 0660, GROUP plugdev and no RUN hooks. Its header records where it came from: the upstream URL, tag v2.57.7, the upstream sha256, the exact upstream line, and why it was narrowed. The broad official copy has been removed. The privileged path made exactly two requests, both reviewed and executed: an install whose verify step compares content (cmp), then a separate `udevadm control --reload-rules`. No `udevadm trigger` was run. The installed file is
…[369 chars clipped]”

## What it unlocks next

- **Software prerequisites met, waiting for hardware/setup** (d405_connected): CAM-01 — First D405 connection: discovery, firmware, USB link

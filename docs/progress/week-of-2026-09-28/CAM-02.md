# CAM-02 — Lossless RGB-D recording + deterministic replay

**Accepted:** 2026-09-28 · **Work package:** D405 capture (L-CAM) · **Track:** camera · **Verified commit:** [`35560d1`](https://github.com/Janga786/rebot-crack-vision/commit/35560d1206ff30c09e80cfd467216a795ab16857)

## What was done

Added a lossless recording/replay layer for the D405 RGB-D pipeline: sessions record colour (lossless PNG) and raw depth (lossless 16-bit PNG) frames plus full device/intrinsics/extrinsics metadata, and can be replayed frame-for-frame with no camera attached. Verified the round trip is bit-exact for both image types and that replay is deterministic across repeated passes; all 10 new tests and the full 174-test project suite pass.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/35560d1206ff30c09e80cfd467216a795ab16857/docs/INTERFACES.md) (+64 / −0)
- [src/crackvision/recording.py](https://github.com/Janga786/rebot-crack-vision/blob/35560d1206ff30c09e80cfd467216a795ab16857/src/crackvision/recording.py) (+262 / −0)
- [tests/test_recording.py](https://github.com/Janga786/rebot-crack-vision/blob/35560d1206ff30c09e80cfd467216a795ab16857/tests/test_recording.py) (+219 / −0)

Commits: [`35560d1`](https://github.com/Janga786/rebot-crack-vision/commit/35560d1206ff30c09e80cfd467216a795ab16857)

## Why it was done

Aligned RGB-D sessions can be recorded losslessly and replayed through the same interface as live capture.

- Requirement **REQ-CAM-2**: Aligned colour/depth capture with timestamps, intrinsics, depth scale, extrinsics and metadata
- Requirement **REQ-CAM-3**: Lossless recording and deterministic replay usable without the camera

## How it moves the project forward

- D405 capture: **3/7** tasks accepted; whole project: **21/74**.
- REQ-CAM-2: 1/4 contributing tasks done
- REQ-CAM-3: 1/3 contributing tasks done
- Verification: automated checks run by the pipeline itself (pytest ✔, suite ✔); independent audit accepted it (criteria: 7 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “CAM-02 adds a self-contained recording.py (CaptureFrame, RecordingWriter, RecordingReader, ReplaySource) with lossless colour PNG + 16-bit depth PNG storage and JSON sidecars/session metadata. All required session.json and sidecar fields are present and tested; the bit-exact round trip claim was independently reverified against PIL's I;16 codec on a realistic-size array. The one debatable acceptance criterion (matching realsense_capture's live-source frame type) is honestly addressed in the implementer's notes: TC-013 has no reusable live-source object to match against, and
…[292 chars clipped]”

## What it unlocks next

- **Ready to start:** CAM-03 — Recording validator with actionable diagnostics
- Closer: CAM-04 — Real recording session over a test surface (still needs CAM-01, CAM-03)

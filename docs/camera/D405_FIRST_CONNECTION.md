# D405 first connection (CAM-01), 2026-10-22

The operator plugged the D405 into the workstation and asked for three things: make it ready, test it by recording
a video, then run the crack-vision pipeline live on it. Everything below was read from the device or measured on
it. The machine-readable versions are `evidence/camera/d405_discovery.json` (from `scripts/camera.py check`) and
`evidence/camera/d405_bringup_2026-10-22.json` (the whole session).

## Device and link

| | |
|---|---|
| Device | RealSense D405, serial **260522273107**, product line D400, USB PID 0B5B |
| Firmware | **5.15.1.55**. That predates 5.17.0.9, which fixed D405 auto-exposure; the newest D405 firmware is 5.17.0.10. pyrealsense2 2.58.4 bundles no firmware, so it reports no "recommended" version |
| USB | **3.2** (`Usb Type Descriptor`). `lsusb -t`: Bus 02 Port 4, `uvcvideo`, **5000M**, directly on the xHCI root hub with no hub in between |
| Access | The udev rule `99-crackvision-d405.rules` applied: the USB node is `root:plugdev 0660`. The operator is in `plugdev`, so no sudo was needed |
| SDK | pyrealsense2 2.58.4 (the `crackvision` env); ROS `librealsense2` 2.57.7 (the `rs-*` tools) |
| Kernel | 6.8.0-124, stock `uvcvideo` (no DKMS) |
| Depth scale | 1e-4 m per unit (read from the device) |
| Colour intrinsics, 848×480 | fx 436.23, fy 435.78, ppx 418.74, ppy 248.59, inverse Brown-Conrady |
| Colour intrinsics, 1280×720 | fx 654.35, fy 653.66, ppx 632.10, ppy 372.89 |
| Stream profiles | Colour and depth at 1280×720 @ 5/15/30; 848×480 and 640×480 up to 90 fps (full list in the discovery JSON) |

## Captures

| Test | Result |
|---|---|
| 90 aligned frames, 848×480 | **30.3 fps, 0 hardware frame gaps**, 94% of pixels with valid depth |
| 90 aligned frames, 1280×720 | **30.2 fps, 0 gaps**, 91% valid depth |
| Auto-exposure, room lighting | 28.2 ms, gain 0. Colour looks right: no overexposure or darkening seen despite the older firmware |
| `camera.py check`: 30 frames, validated | **validate_recording PASS** on every check; verdict **WARN** for the firmware only |
| **Video test**: `camera.py record --hd --seconds 10 --video` | 300 frames in 10.0 s (30.0 fps); lossless recording **PASS**; `video.mp4` H.264, 2560×720 (colour beside colourised depth), 10.0 s, 9.5 MB. A frame of it is `evidence/camera/d405_video_frame_2026-10-22.jpg` |

**First-attempt failure, fixed.** The first `check` wrote each frame's PNGs inside the capture loop (about 230 ms
per frame). 30 frames took 7 s and the validator correctly failed it on 172 hardware frame-number gaps.
Recordings now capture into memory at full rate and write afterwards: 21.5 s for the 300-frame 1280×720 session
on 8 threads, against 55.4 s serially.

## The crack model on the camera

| Test | Result |
|---|---|
| `camera.py snap` (848×480: burst of 10, sharpest frame, full pipeline) | Ran in 14.8 s. The lab room has **no cracks: 0 paths**, 1 stray pixel |
| `camera.py live` (848×480) | **4.7 fps** (model 215 ms per frame), 400 frames. Stream, stats and snapshot endpoints work. A frame is `evidence/camera/d405_live_view_2026-10-22.jpg` |
| `camera.py live --hd` (1280×720) | **2.1 fps** (model 421 ms), 60 frames |
| Live snapshot → `snap --image` | The full pipeline on the saved frame: 0 paths, 7 px |

The model's few predicted pixels in the room scene (0.07–0.08% in live view) are specks on the speckled floor,
about 3 m away. That is far outside the close-up range the model and this camera are for (7–50 cm).

## Same day: the live view's delay (operator: "it is slightly delayed")

| | Before | After |
|---|---|---|
| Video in the browser, 848×480 | 4.7 fps: only frames the model had finished | **30 fps**: every camera frame, with the newest mask |
| Model per frame, 848×480 | 204–215 ms | **54–57 ms** (17 passes a second); the mask trails the video by about 100 ms |
| Model per frame, 1280×720 | 404–421 ms | **111–120 ms** (8 a second); the mask trails by about 165 ms |

Where the time went:
- Frames arrived only 17–30 ms old, and no queue built up.
- The model was about 205 ms per frame. 68 ms of that was nnU-Net reloading the fold's weights on every call;
  the rest was running its 18 tiles through the network one at a time.
- The view could only change when the model finished.

The fixes, in `crackvision.live_predictor` and `camera.py live`:
- load the weights once;
- batch the tiles (nnU-Net's own tile order, Gaussian weighting and accumulation; identical output on every
  pixel of 10 real 848×480 frames, and on 99.9998% at 1280×720);
- run the model in its own thread on the newest frame while the view streams at the camera's rate.

`--sync` keeps the old behaviour.

## Not yet shown

- **A positive detection on real imagery.** No crack was in view. The next step is to point the wrist at a
  cracked surface (or a printed crack photo) at 10–40 cm and run `camera.py live`, then `camera.py snap`.
- **Firmware 5.17.0.10.** It is recommended because of the auto-exposure fixes, but colour looked fine in room
  light. Updating is the operator's decision ([D405_SETUP.md](D405_SETUP.md) §5).
- Hand-eye calibration (GEOM-04.x) and captures tied to joint states are separate steps.

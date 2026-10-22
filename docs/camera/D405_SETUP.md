# D405 camera: setup and running the crack model on it

**Status (2026-10-22): connected and working.** The D405 (serial 260522273107, firmware 5.15.1.55) is on a USB 3
port of this workstation. [D405_FIRST_CONNECTION.md](D405_FIRST_CONNECTION.md) has the full report.
- **Capture:** 30 fps with no dropped frames at 848×480 and 1280×720.
- **Video test:** 10 s at 1280×720, recorded losslessly and validated, plus an MP4.
- **The model on the camera:** `snap` (full pipeline) and `live` both run on the real camera. `live` shows the
  video at 30 fps with the crack mask on top. The model runs 17 times a second at 848×480 and 8 at 1280×720, so the
  mask trails the video by about 100–165 ms.
- **Open:** the firmware predates the D405 auto-exposure fixes (§5; updating is the operator's call), and no real
  crack has been in view yet.

Evidence: `evidence/camera/d405_discovery.json`, `evidence/camera/d405_bringup_2026-10-22.json`.

The hardware facts here come from the RealSense D400 datasheet (March 2026), the librealsense releases and
source, and the Linux 6.8 UVC driver. The sources are listed at the end.

## 1. The camera: DigiKey 99ACXD

| | |
|---|---|
| Product | RealSense **D405** depth camera. DigiKey 99ACXD, "camera only". Intel P/N 82635DSD405MP (the datasheet lists 99ACXD as the multi-pack MM#; the single-unit 99ACXC ships with a cable) |
| Connector | **USB 3 Micro-B** (not USB-C), with two M2×0.4 screw-lock holes 18 mm apart |
| Cable (needed for the camera-only SKU) | A certified **USB-A to USB 3.0 Micro-B** cable, ideally screw-locking (RealSense support commonly suggests Newnex). Keep a passive cable at **2 m or shorter**. Beyond that, use at most one active repeater, and never update firmware through it |
| Link | USB 3.1 Gen 1 is needed for every mode. On USB 2 the camera falls back to reduced modes (colour YUY2 1280×720 at 5–15 fps; depth 640×480 at 30 fps at most), and the colour can overexpose |
| Power | 1.48 W at most, from the USB bus |
| Size / mounting | 42×42×23 mm, 58 g. Back: 2× M3×0.5, 20 mm apart (4 mm deep). Bottom: 1/4-20 (5 mm). The arm uses Seeed's stock D405 mount |
| Imagers | Two global-shutter OV9782 sensors, 18 mm stereo baseline. **Colour comes from the left imager**, the same one depth is referenced to, so colour and depth share nearly one optical centre (`rs.align` still runs, for exact pixels) |
| Projector | **None.** Depth is passive stereo: it needs surface texture and light. Concrete has texture; shadows from the arm hurt |
| Field of view | 84° × 58° (92° diagonal) per the datasheet; retailer pages say 87°. This unit's colour intrinsics give fx = 436 px at 848×480 (about 88° horizontal) |
| Range | Ideal **7–50 cm**. Minimum Z is **10 cm at 1280×720** and 7 cm at 848×480. Z accuracy ±2% and RMS noise ≤ 1% within 0.5 m |
| Modes (USB 3) | 1280×720 at 5/15/30 fps; 848×480 and below up to 90 fps; colour and depth on the same grid |
| Depth units | 0.0001 m (0.1 mm) per unit by default. The tools read `get_depth_scale()` and never assume it |

## 2. Connecting it

1. **Port.** Use a **rear USB 3 Type-A port on the Z8 G4, with no hub in between.** `lsusb -t` on this machine
   shows a 5000M (USB 3) root hub; the camera must appear under it.
2. **Order.** Plug the cable into the camera first, then push the host end in **quickly and firmly.** A slow
   insertion into a Type-A port can enumerate as USB 2; this is RealSense erratum RSDSO-6804, marked "no fix".
   Tighten the screw locks if the cable has them.
3. **Check:**
   ```bash
   lsusb -d 8086:0b5b                      # Intel ... D405
   lsusb -t | grep -B1 -i video            # the camera's interfaces at 5000M, not 480M
   rs-enumerate-devices -s                 # Name, Serial, Firmware, Usb Type Descriptor (want 3.2)
   ```
4. **On the arm:** route the cable along the arm with slack at every joint, J4–J6 especially. J6 rolls ±3.14 rad
   and can wind a tight cable. Neither the planner nor arm_guard knows the cable exists. Clip it so it can't
   snag on the base or the table.

## 3. Software (already installed here)

| Piece | What is installed | Notes |
|---|---|---|
| Python SDK | `pyrealsense2` **2.58.4** in the `crackvision` env (pip; pinned `>=2.55,<3` in `requirements/requirements-realsense.txt`) | Bundles its own librealsense. Upstream's latest *official* release is v2.58.1; 2.58.2–2.58.4 and 2.59.1 are tagged beta on GitHub but are what pip installs by default. Fine for the D405 |
| Tools | `rs-enumerate-devices`, `realsense-viewer`, `rs-fw-update` from ROS Humble's `ros-humble-librealsense2` **2.57.7** (`/opt/ros/humble/bin`) | Used for diagnostics only. A version different from pip's is fine as long as only one program uses the camera at a time |
| udev | `/etc/udev/rules.d/99-crackvision-d405.rules` (HOST-03 / [DEVICE_ACCESS.md](../host/DEVICE_ACCESS.md)) | The D405 (`8086:0b5b`) only, mode 0660, group `plugdev`; the operator is in `plugdev`. See §5 about firmware recovery mode |
| Kernel | 6.8.0-124 | The stock `uvcvideo` driver lists the D405 (`0x0b5b`) with full metadata, so **no `librealsense2-dkms` kernel patch is needed** (DKMS doesn't list 6.8 anyway) |
| Model | OpenCrack nnU-Net, `models/opencrack-nnunet` (fold 0, `checkpoint_ep0500.pth`) | Runs on the RTX 3090 |

**On a fresh machine:**
1. `./env.sh pip install -r requirements/requirements-realsense.txt`.
2. Install the udev rule (`docs/host/DEVICE_ACCESS.md`, one `sudo install`).
3. Optionally install the RealSense apt repo's `librealsense2-utils` for the `rs-*` tools. Don't install
   `librealsense2-dkms`.

`./env.sh python scripts/check_realsense.py` verifies the software stack with or without a camera.

## 4. First connection (CAM-01)

```bash
./env.sh python scripts/camera.py check            # add --hd to also prove 1280x720
```

It runs these steps:
1. Lists the device: name, serial, firmware, the SDK's recommended firmware, product line, USB type.
2. Lists its stream profiles.
3. Streams with auto-exposure warm-up and records **30 frames losslessly** to `data/recordings/cam01_<UTC>/`.
4. Runs `scripts/validate_recording.py` on the recording: metadata, depth scale, timestamps, dropped frames,
   USB type, invalid depth.
5. Writes **`evidence/camera/d405_discovery.json`**, which is tracked in git.

The verdict is one of three:
- **FAIL:** a USB 2 link, no frames, or the validator fails.
- **WARN:** old firmware, an unusual depth scale, or validator warnings.
- **PASS.**

Without a camera, it explains what to plug in and where. If `lsusb` sees the D405 but the SDK can't open it, the
udev rule is missing or another program holds the camera. Close `realsense-viewer` and any ROS camera node.

## 5. Firmware

- The newest firmware for the D405 is **5.17.0.10** (July 2025). Firmware **5.17.0.9** fixed D405
  auto-exposure: dark colour when only colour streams, plus the auto-exposure region. Older 5.15/5.16 firmware has
  reports of overexposure and frozen auto-exposure (librealsense issue #12815). `check` warns below 5.17.0.9.
- **Don't flash 5.17.3.10.** It is for the D435/D455/D456/D457 only.
- **Updating is an operator decision.** The tools never flash anything. Procedure:
  1. Download `d400_series_production_fw_5_17_0_10` from dev.realsenseai.com (accept the EULA). The SDK stopped
     bundling firmware in v2.58.2.
  2. Run `rs-fw-update -l`, then `rs-fw-update -s <SERIAL> -f <the .bin from the zip>`.
  3. Use a rear port, no hub and a passive cable.
  4. Then re-run `scripts/camera.py check`.
- **Recovery mode.** If an update fails, the camera re-enumerates in recovery mode under a different USB product
  id, which the project's narrowed udev rule does not cover. Install upstream's full
  `99-realsense-libusb.rules` (needs sudo; operator) before `rs-fw-update -r -f <bin>`.

## 6. Running the crack model on the camera

| Command | What happens | Output |
|---|---|---|
| `./env.sh python scripts/camera.py snap` | Captures a burst of 10 frames after warm-up and keeps the **sharpest** (variance of the Laplacian; the arm may still be settling). Stores it as its own case and runs the accepted single-case pipeline: `prepare_inputs` → nnU-Net (the validated batch path, TTA on) → skeleton → crack paths → renders. About 15 s | `data/camera/<case>/snap.json` (the frame, its intrinsics, burst sharpness, results). Images: `data/overlays/<case>_overlay.png`, `data/comparisons/<case>_comparison.png`, `data/skeletons/<case>_skeleton_overlay.png`, `data/paths/<case>_paths_overlay.png`; the paths JSON in `data/paths/` |
| `./env.sh python scripts/camera.py live` | Keeps the network in memory (`crackvision.live_predictor`): the video runs at 30 fps and the model works on the newest frame in its own thread (`--sync` waits for the model on every frame instead). Then open **http://127.0.0.1:8090**. The mask is overlaid in red, with model time, crack area and distance on top. The **snapshot** button saves the current frame to `data/camera/live_<UTC>/` and returns the `snap --image` command for the full pipeline on it | Browser view. This env's OpenCV is headless, so the browser is the display |
| `./env.sh python scripts/camera.py record --seconds 10 --video` | Captures at the camera's full rate into memory first, so nothing slow runs in the capture loop: 30.0 fps, no dropped frames. Then writes a lossless RGB-D recording on 8 threads and validates it, plus `video.mp4` (H.264, colour beside colourised depth). `--frames N` instead of `--seconds`; `--no-frames` writes the video only; `--no-depth` gives a colour-only video. RAM is checked first. Replays bit-exactly with `--replay DIR` | `data/recordings/<session>/` (+ `video.mp4`) |

**Options:**
- `--hd` (1280×720) for more pixels on thin cracks. At that resolution the minimum distance is 10 cm.
- `--burst N`, `--name CASE`, `--no-model` (`snap`).
- `--tta` (`live`: mirroring as in the batch path, about 8× slower).
- `--replay DIR` (any command, no camera); `--image PNG` (`snap` on a saved frame).

**Speed** (real camera, RTX 3090):
- **Live (without TTA):** the view runs at the camera's 30 fps. The model runs on the newest frame in its own
  thread: 57 ms at 848×480 (17 passes a second; the mask trails the video by about 100 ms) and 120 ms at 1280×720
  (8 a second, about 165 ms behind). `--sync` instead shows only frames the model has finished: the mask is then
  exactly on its frame, but the whole view lags by a model pass.
  - The model is about 3.8× faster than stock nnU-Net's in-process predictor (204 → 54 ms at 848×480), with no
    change in output: identical masks on 100% of pixels over 10 real frames, 99.9998% at 1280×720. Two changes do
    it: the sliding-window tiles go through the network as one batch, and the single fold's weights load once
    instead of on every frame.
  - What remains is the camera itself. A frame is 17–30 ms old on arrival (28 ms auto-exposure in room light, one
    33 ms frame period), plus about 10 ms to encode and stream. More light shortens the exposure; `--fps 60` halves
    the frame period at 848×480.
- Snapshot: 15–19 s including capture, mostly nnU-Net start-up.
- Recording: 30.0 fps capture. Writing takes about 2 s per second of 1280×720 recording, after capture.

**Live versus batch.** With TTA, the in-memory network gives the batch path's mask exactly: 100% of pixels on the
sample crack, and a test checks this. Without TTA it agreed on 99.8% of pixels there.

**Scale.** `snap` takes the median depth over the crack pixels and divides it by the focal length to report
**millimetres per pixel**, so crack path lengths are also given in mm. Below 50 crack pixels with depth, it uses
the whole frame instead. Beyond 50 cm it flags the scale as rough. Values from this unit's intrinsics:

| Distance | 848×480 (fx 436) | 1280×720 (fx 654) |
|---|---|---|
| 10 cm | 0.23 mm/px | 0.15 mm/px |
| 20 cm | 0.46 mm/px | 0.31 mm/px |
| 30 cm | 0.69 mm/px | 0.46 mm/px |
| 40 cm | 0.92 mm/px | 0.61 mm/px |

## 7. Getting good crack images

- **Distance:** 10–40 cm (the D405_TEST_PLAN band).
- **Light:** the camera has no projector and the wrist shadows the surface at close range. Use diffuse light,
  ideally mounted near the wrist. Depth gets holes on dark or shadowed patches; `snap.json` reports
  `valid_depth_fraction`.
- **Motion:** stop the arm before `snap`. Global shutter avoids skew but not blur, and the burst's sharpest frame
  covers the settling.
- **Exposure:** auto-exposure (the default) is right for bring-up. For a consistent dataset (PERC-05), fix exposure
  and white balance on the colour sensor. The tool doesn't expose those options yet. Don't re-enable
  auto-exposure while it is already on; this is reported to freeze the D405.
- **Presets:** the D405 has its own Default, plus Hand, High Accuracy, High Density and Medium Density. The tools
  use the device default. Use High Accuracy if crack depth is ever measured.

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `check`: "No D405 on the USB bus" | Not plugged in, a power-only cable, or a dead port | Use a data-rated USB 3 Micro-B cable on a rear port; check `lsusb -d 8086:0b5b` |
| `check`: "the USB bus sees a D405 ... cannot open it" | The udev rule is missing, the user isn't in `plugdev`, or another process (viewer, ROS node) holds it | `cmp host/udev/99-crackvision-d405.rules /etc/udev/rules.d/99-crackvision-d405.rules`; `groups`; close the other program; replug |
| `check`: FAIL, USB link 2.x | Slow insertion, USB 2 port or hub, or a USB 2 cable | Rear port, no hub, USB 3 cable; replug the camera end first, then the host end firmly |
| "refused 1280x720@60" | No such mode | 1280×720 runs at 5/15/30 fps; 848×480 up to 90 |
| `Frame didn't arrive within 5000` / D405Error mentioning USB | Bandwidth or cable | Shorter or better cable; a port on another controller (`lsusb -t`) |
| Dark or overexposed colour, frozen auto-exposure | Old firmware (< 5.17.0.9), or USB 2 | §5; USB 3 |
| `live` page is blank | Wrong host or port, or the network is still loading (about 5 s) | Use the URL it prints; `--port` |
| validate_recording: dropped hardware frames | Something slow ran between frames. The first `check` on 2026-10-22 wrote PNGs in the capture loop and got 172 gaps in 30 frames | Fixed: recordings capture into memory first. Also make sure no other program streams from the camera |
| "Device or resource busy" / `live` or `snap` fails while another runs | One process at a time owns the D405 | Stop the other one (`live` holds the camera until Ctrl-C) |

## 9. Not covered yet

- **Hand-eye calibration** (camera ↔ wrist, GEOM-04.x). It needs `cv2.calibrateHandEye`, which this env's
  OpenCV 5 lacks (GEOM-04.1 is an operator step). Until it's done, crack positions are image and depth only, not
  robot coordinates.
- **Captures tied to a joint state** (`capture_record`, docs/INTERFACES.md §9). That is the arm-integration
  step; `snap` doesn't record joint states.
- **The real dataset and the zero-shot fitness decision** (PERC-05, PERC-06, PERC-08). `record` and `snap`
  produce the inputs for both.

## Sources

- RealSense D400 Series Datasheet (Mar 2026): https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf
- D400 Spec Update 043 (errata, incl. RSDSO-6804): https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Spec-Update-043.pdf
- librealsense releases (v2.58.1 official; v2.58.2–2.59.1 beta): https://github.com/realsenseai/librealsense/releases
- Linux install and apt repo: https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md
- Firmware releases and update tool: https://dev.realsenseai.com/docs/firmware-releases-d400 , https://dev.realsenseai.com/docs/firmware-update-tool
- Visual presets: https://dev.realsenseai.com/docs/d400-series-visual-presets
- udev rules: https://raw.githubusercontent.com/realsenseai/librealsense/master/config/99-realsense-libusb.rules
- D405 depth units default: https://raw.githubusercontent.com/realsenseai/librealsense/master/src/ds/d400/d400-device.cpp
- Linux 6.8 UVC device table (D405 0x0b5b): https://raw.githubusercontent.com/torvalds/linux/v6.8/drivers/media/usb/uvc/uvc_driver.c
- D405 auto-exposure issue: https://github.com/realsenseai/librealsense/issues/12815
- DigiKey listing: https://www.digikey.com/en/products/detail/realsense/99ACXD/16265565

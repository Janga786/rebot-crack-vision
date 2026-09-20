# TC-013 — D405 capture utility

**Task ID:** TC-013 · **Complexity:** COMPLEX · **Prerequisites:** TC-012

## Objective
Build `src/crackvision/realsense_capture.py`: synchronised, depth-aligned RGB + raw-depth capture from
an Intel RealSense D405, with full intrinsics/metadata sidecars — fully implementable and testable
with **no camera attached**.

## Why this matters
These files are the inputs to the whole Level-3 evaluation *and* the raw material for the deferred
3D-projection stage. Getting the depth scale or the alignment wrong here silently corrupts everything
downstream, months later (`RISKS.md` R-08, R-09).

## Prerequisites
TC-012 COMPLETE (`pyrealsense2` installed, `write_depth_png`/`read_depth_png` proven).

## Scope
`src/crackvision/realsense_capture.py` per `docs/INTERFACES.md` §3.10, plus tests extending
`tests/test_realsense_import.py`.

## Out of scope
- Running the crack pipeline on captured frames (that is `run_test.sh` after copying files in).
- Depth→3D point-cloud projection (`adr/002`).
- Camera↔robot calibration, hand-eye, any extrinsic to the arm (`adr/009`).
- Point-cloud export (`.ply`), RGB-D SLAM, ROS publishing.
- Collecting the actual evaluation dataset (`docs/D405_TEST_PLAN.md` — a later human activity).
- Post-processing filters (spatial/temporal/hole-filling). Raw depth only; note that as a future option.
- Any auto-exposure/gain tuning beyond the warm-up discard.

## Files to create
`src/crackvision/realsense_capture.py`

## Files allowed to modify
`tests/test_realsense_import.py` (add tests), `docs/COMPLETION_LOG.md`, `task_cards/TASK_INDEX.md`.
Writes `data/d405/**`.

## Files that must NOT be modified
`scripts/check_realsense.py` (TC-012 owns it — import its helpers, don't edit it) ·
`src/crackvision/{config,logging_setup,naming,prepare_inputs,inference,visualize,skeleton}.py` ·
`/opt/ros/**` · `env.sh`.

## Implementation requirements

```
./env.sh python -m crackvision.realsense_capture [--session NAME] [--frames N] [--interval S]
     [--color-width W --color-height H --depth-width W --depth-height H --fps F]
     [--no-align] [--preview] [--warmup N] [--list-devices] [--synthetic] [common flags]
```

### 1. Lazy import — mandatory

```python
def _import_rs():
    try:
        import pyrealsense2 as rs
    except ImportError as e:
        raise PreconditionError(
            "pyrealsense2 is not installed. Run: "
            "./env.sh pip install -r requirements/requirements-realsense.txt"
        ) from e
    return rs
```
**No module-level `import pyrealsense2`.** `import crackvision.realsense_capture` must succeed on a
machine with no RealSense stack, so the tests run anywhere.

### 2. Pipeline

```python
rs = _import_rs()
pipeline, config = rs.pipeline(), rs.config()
config.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, fps)
config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16,  fps)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)          # unless --no-align
```

Defaults from `config/project.yaml` `realsense:` — colour `848×480 @ 30 bgr8`, depth
`848×480 @ 30 z16`. These sit in the D405's comfortable range; expose overrides.

If `enable_stream`/`start` raises, catch it, enumerate and print the profiles the device **does**
support (`sensor.get_stream_profiles()`), then exit 3. A user should never have to guess.

### 3. Alignment — default ON

`rs.align(rs.stream.color)` makes `depth[r,c]` correspond to `color[r,c]`, and therefore to
`mask[r,c]` and `skeleton[r,c]`. That correspondence is the frame invariant this whole project is
built on (`adr/002`, `INTERFACES.md` §0.5). `--no-align` exists for diagnostics only and must log a
prominent WARN that the saved depth is **not** pixel-aligned to colour.

### 4. Warm-up
Discard the first `warmup_frames` (default **30**, from config) so auto-exposure settles.
Do not save them. Log how many were discarded.

### 5. Saving

- **Colour** → `data/d405/color/{session}_{NNNNNN}_color.png`, 8-bit **RGB**. The stream is `bgr8`;
  convert BGR→RGB exactly once, with an explicit comment saying so. (Double-conversion is the classic
  bug here.)
- **Depth** → `data/d405/depth/{session}_{NNNNNN}_depth.png`, **16-bit single channel, the raw `z16`
  buffer**. Use `write_depth_png` from TC-012. Never colorise, never scale to 8-bit, never JPEG.
  An optional `--save-depth-preview` may write a *separate* `_depthviz.png` via
  `rs.colorizer()` — clearly distinct, never a substitute.

### 6. Metadata — the part that matters most

Per frame, `data/d405/metadata/{session}_{NNNNNN}.json`, schema in `INTERFACES.md` §3.10.

> **`depth_scale_m_per_unit` MUST come from
> `profile.get_device().first_depth_sensor().get_depth_scale()`.**
> The D405 typically reports **0.0001** m/unit while other D4xx report 0.001. A hard-coded value
> silently makes every future 3D point 10× wrong. This is `RISKS.md` R-09 and a named acceptance
> criterion. **Do not hard-code it. Do not default it. If the call fails, exit 3.**

Also record, from the **aligned** profiles (not the raw ones):
`get_profile().as_video_stream_profile().get_intrinsics()` → `fx, fy, ppx, ppy, model, coeffs`
for both colour and depth, plus `get_extrinsics_to()` depth→colour, the stream configuration,
`warmup_frames`, `aligned_to`, and `exposure_us`/`gain` if
`frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure)` is supported (guard it — not all
firmware exposes it; `None` is fine).

Write one `{session}_session.json` with device info, the resolved configuration, the tool version and
start/end timestamps.

### 7. Modes with no hardware
- `--list-devices` — print name/serial/firmware/USB for each device; works with zero devices, exit 0.
- `--synthetic N` — write plausible fake colour (noise + a drawn dark line), depth (a smooth ramp
  with a few zero "holes"), and metadata with **`"synthetic": true`** and filenames prefixed
  `synth`. This exercises the file format and TC-015's tooling with no camera. Synthetic frames must
  be impossible to mistake for real ones.
- `--dry-run` — log the configuration and exit 0 without touching the device.

### 8. No device, not synthetic → exit 3
```
No RealSense device found. Attach a D405 (USB-3), or use --synthetic to exercise the file format.
```

### 9. `--preview`
OpenCV window showing colour and colorised depth side by side. Must be a **no-op with a WARN** when
`$DISPLAY` is unset, never a crash. `cv2.waitKey` for `q` to stop early.

### 10. Cleanup
`pipeline.stop()` in a `finally:` block, always. A KeyboardInterrupt must stop the pipeline cleanly
and still write the session JSON for the frames already captured.

### Tests (added to `tests/test_realsense_import.py`)
- `import crackvision.realsense_capture` succeeds with `pyrealsense2` absent (monkeypatched).
- `--synthetic 3` produces 3 colour + 3 depth + 3 metadata files + 1 session file.
- Every synthetic metadata JSON validates against the §3.10 schema and has `"synthetic": true`.
- The synthetic depth PNG round-trips as `uint16` (reuses the TC-012 helper).
- With no device and no `--synthetic`, `main()` returns **3**.
- `--dry-run` writes zero files.
- `@pytest.mark.camera` tests for real capture — skipped by default.

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh python -m crackvision.realsense_capture --help
./env.sh python -m crackvision.realsense_capture --list-devices ; echo "exit=$?"
./env.sh python -m crackvision.realsense_capture ; echo "no-camera exit=$? (expect 3)"
./env.sh python -m crackvision.realsense_capture --synthetic 3 --session synthtest ; echo "exit=$?"
ls -la data/d405/color/ data/d405/depth/ data/d405/metadata/
python3 -m json.tool data/d405/metadata/synthtest_000000.json
./env.sh python -c "
from PIL import Image; import numpy as np
a=np.array(Image.open('data/d405/depth/synthtest_000000_depth.png'))
print('depth dtype',a.dtype,'shape',a.shape,'max',a.max())
assert a.dtype==np.uint16, 'depth must be 16-bit'"
./env.sh pytest tests/test_realsense_import.py -v
grep -n "^import pyrealsense2\|^from pyrealsense2" src/crackvision/realsense_capture.py \
  && echo "FAIL: module-level import" || echo "OK: import is lazy"
grep -nE "0\.001|0\.0001" src/crackvision/realsense_capture.py   # must not be a hard-coded depth scale
```

## Acceptance criteria
- [ ] `import crackvision.realsense_capture` succeeds with `pyrealsense2` unavailable (test proves it).
- [ ] **No module-level `import pyrealsense2`** (grep above).
- [ ] **`depth_scale_m_per_unit` is read from `get_depth_scale()` and is never hard-coded** (grep
      above finds no literal depth-scale constant).
- [ ] `--list-devices` exits 0 with no camera.
- [ ] No camera and no `--synthetic` → exit **3** with the exact actionable message.
- [ ] `--synthetic 3` produces 3 colour + 3 depth + 3 per-frame metadata + 1 session JSON.
- [ ] Depth PNGs are **16-bit single-channel** and round-trip byte-identically as `uint16`.
- [ ] Colour PNGs are 8-bit RGB (not BGR — verify a known synthetic colour comes out correct).
- [ ] Every metadata JSON validates against `INTERFACES.md` §3.10, including `aligned_to`,
      both intrinsics blocks, and the depth→colour extrinsics.
- [ ] Synthetic frames carry `"synthetic": true` **and** the `synth` filename prefix.
- [ ] `--no-align` logs a prominent WARN.
- [ ] `--dry-run` writes zero files.
- [ ] `pipeline.stop()` is in a `finally:` block.
- [ ] `--preview` with `$DISPLAY` unset warns and continues rather than crashing.
- [ ] `./env.sh pytest tests/ -v` green (camera tests skipped).
- [ ] No hard-coded `/home/` path.
- [ ] **Hardware verification is explicitly marked DEFERRED** in the completion report — no D405 is
      attached to this machine, and that is expected.

## Failure handling
- **No camera** → the normal case here. Everything must be exercised with `--synthetic`. Mark the
  hardware-only acceptance items as deferred; do **not** fake a device to tick them.
- **`enable_stream` fails** (unsupported resolution/fps) → print the supported profiles, exit 3.
- **`get_depth_scale()` raises** → exit 3. Never substitute a guess. This is a hard stop.
- **Frames time out** (`wait_for_frames` raises) → retry up to 3 times, then exit 1 with the USB
  advice from TC-012 (USB-2 link, cable quality).
- **USB-2 link detected** → WARN loudly; resolutions may be silently reduced.
- **`pyrealsense2` unavailable** (TC-012 was PARTIAL) → the module must still import, `--synthetic`
  must still work, and the hardware paths report the install hint. Mark PARTIAL and continue.

## Documentation update
Append the report, including one full synthetic metadata JSON, and an explicit
**"HARDWARE VERIFICATION DEFERRED — no D405 attached"** line listing which acceptance items await a
camera. Update `TASK_INDEX.md`: TC-013 → `COMPLETE` (software) with the deferred note.

## Commit guidance
`feat: add D405 aligned RGB-D capture with intrinsics metadata`

## Completion report format
```
TASK: TC-013
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-014
```

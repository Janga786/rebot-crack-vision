# TC-012 — RealSense/D405 software validation (no camera required)

**Task ID:** TC-012 · **Complexity:** SMALL · **Prerequisites:** TC-002

## Objective
Install `pyrealsense2` into the project env and build `scripts/check_realsense.py`, which reports the
RealSense software and device state and **completes successfully with no camera attached**.

## Why this matters
No D405 is connected to this machine (`lsusb` confirms). This card makes the camera stack verifiable
now, so TC-013 can be written and tested without waiting for hardware.

## Prerequisites
TC-002 COMPLETE. Runs in parallel with TC-004–TC-011.

## Scope
1. Ensure `pyrealsense2` is installed from `requirements/requirements-realsense.txt`.
2. Implement `scripts/check_realsense.py` per `docs/INTERFACES.md` §3.9.
3. `tests/test_realsense_import.py` — the import-guard and 16-bit-PNG round-trip tests.

## Out of scope
- Capturing frames (TC-013).
- Any `apt`/`sudo` install of librealsense. The pip wheel is self-contained; the ROS-scoped
  `librealsense2 2.57.7` already on the box is **not** to be modified.
- Adding a udev rule, plugging in hardware, or anything requiring physical access.
- ROS `realsense2_camera` (`adr/009`).

## Files to create
```
scripts/check_realsense.py
tests/test_realsense_import.py
```

## Files allowed to modify
`requirements/requirements-realsense.txt` (only if the pin must change), `docs/COMPLETION_LOG.md`,
`task_cards/TASK_INDEX.md`. The `crackvision` conda env only.

## Files that must NOT be modified
`/opt/ros/**` · any apt package · `env.sh` · `src/crackvision/**` · any other conda env.

## Implementation requirements

### Install
```bash
./env.sh pip install -r requirements/requirements-realsense.txt
```
Verified upstream (2026-09-16): PyPI `pyrealsense2` **2.58.4.10922**, `requires_python >=3.9`, with
manylinux x86_64 wheels for **cp310/cp311/cp312/cp313** — a cp311 wheel exists, so this env is fine.

### `scripts/check_realsense.py`

`./env.sh python scripts/check_realsense.py [--json]`

Checks, in order, each exception-safe:

| # | Name | PASS / WARN / FAIL |
|---|---|---|
| 1 | `pyrealsense2_import` | FAIL if unimportable, with the install command as the hint |
| 2 | `pyrealsense2_version` | report `rs.__version__` (fall back to `importlib.metadata.version`) |
| 3 | `librealsense_runtime` | report the bundled runtime version if exposed |
| 4 | `devices_found` | `len(rs.context().query_devices())`; **0 → WARN, exit 0** |
| 5 | `device_details` | per device: name, serial, firmware, physical port, USB descriptor — only if present |
| 6 | `d405_present` | a device whose name contains `D405` → PASS; none → WARN |
| 7 | `usb_descriptor` | WARN if a device reports USB 2.x: `"USB-2 link — resolution/fps will be limited; use a USB-3 port and cable"` |
| 8 | `stream_profiles` | enumerate colour + depth profiles for a found D405; skipped with no device |
| 9 | `depth_scale` | `depth_sensor.get_depth_scale()` if a device is present — report it, noting D405 is typically 0.0001 m/unit |

Exit **0** when the software stack is healthy, even with zero devices. Exit **3** only if
`pyrealsense2` cannot be imported. Always write `logs/check_realsense_latest.json`.

The no-device output must be encouraging, not alarming:
```
devices_found  WARN   0
   hint: no RealSense device attached. The software stack is fine. Attach a D405 to a
         USB-3 port, or use `--synthetic` with crackvision.realsense_capture (TC-013).
```

### `tests/test_realsense_import.py`

1. **Import guard:** `import crackvision.realsense_capture` succeeds even when `pyrealsense2` is
   absent. Simulate absence with `monkeypatch.setitem(sys.modules, "pyrealsense2", None)` or by
   patching the import hook — the point is to prove the module-level import is lazy.
   *If TC-013 has not run yet, write this test with `pytest.importorskip` on
   `crackvision.realsense_capture` so it skips cleanly, and record that under `ISSUES:`.*
2. **16-bit depth PNG round-trip** — this is the important one and needs no camera:
```python
arr = rng.integers(0, 65535, size=(64, 96), dtype=np.uint16)
write_depth_png(arr, tmp_path / "d.png")
back = read_depth_png(tmp_path / "d.png")
assert back.dtype == np.uint16 and np.array_equal(back, arr)
```
   Implement `write_depth_png` / `read_depth_png` **here**, in `scripts/check_realsense.py` or a tiny
   helper, and have TC-013 import them — depth fidelity is the one thing that must be provably
   correct before hardware arrives (`RISKS.md` R-08/R-09).
3. `@pytest.mark.camera` tests that enumerate a real device, skipped by default.

## Commands/tests to run
```bash
cd ~/Projects/rebot_crack_vision
./env.sh pip install -r requirements/requirements-realsense.txt
./env.sh python -c "import pyrealsense2 as rs; print('pyrealsense2', getattr(rs,'__version__','?'))"
./env.sh python scripts/check_realsense.py ; echo "exit=$? (expect 0 with no camera)"
./env.sh python scripts/check_realsense.py --json | python3 -m json.tool
./env.sh pytest tests/test_realsense_import.py -v
lsusb | grep -i realsense || echo "confirmed: no RealSense device attached"
./env.sh python scripts/check_env.py ; echo "check_env exit=$? (pyrealsense2 row should now PASS)"
```

## Acceptance criteria
- [ ] `import pyrealsense2` succeeds in the `crackvision` env; the version is recorded in the report.
- [ ] `check_realsense.py` exits **0** with **zero devices attached** (WARN, not FAIL).
- [ ] The no-device hint text is present and actionable.
- [ ] `--json` produces valid JSON; `logs/check_realsense_latest.json` is written.
- [ ] The 16-bit depth PNG round-trip test passes: `uint16` in, byte-identical `uint16` out.
- [ ] `check_env.py`'s `pyrealsense2` row is now PASS.
- [ ] No `apt`, no `sudo`, no change under `/opt/ros`.
- [ ] `./env.sh pytest tests/ -v` is green (camera-marked tests skipped).
- [ ] No hard-coded `/home/` path.

## Failure handling
- **No cp311 wheel / install fails** → try `pyrealsense2>=2.55,<3` unpinned, then a specific older
  version. If none installs, mark **PARTIAL**: `check_realsense.py` must still exist and report the
  FAIL cleanly, and TC-013 remains writable (its code paths are all lazy). Do **not** try to build
  librealsense from source, and do **not** use `sudo`.
- **`rs.__version__` missing** → fall back to `importlib.metadata.version("pyrealsense2")`, else
  `"unknown"`; still PASS.
- **`rs.context()` raises** (no udev rules / no permission) → WARN with the message, exit 0. Do not
  install udev rules (needs sudo).
- **A device *is* attached** → great: fill in every device check and record the real `depth_scale` in
  the completion report. That value is valuable for TC-013.

## Documentation update
Append the report including the `pyrealsense2` version and full check output. Update `TASK_INDEX.md`:
TC-012 → `COMPLETE`, TC-013 → `READY`.

## Commit guidance
`feat: add RealSense software validation and depth PNG round-trip test`

## Completion report format
```
TASK: TC-012
STATUS: COMPLETE / BLOCKED / PARTIAL
CHANGES:
TESTS:
ISSUES:
NEXT CARD: TC-013
```

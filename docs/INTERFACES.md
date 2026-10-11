# INTERFACES.md — normative I/O contracts

**Status:** NORMATIVE. A task card that contradicts this file is wrong; report it, do not improvise.
Every path below is relative to `$CRACKVISION_ROOT` = `~/Projects/rebot_crack_vision`.

---

## 0. Universal conventions

### 0.1 Invocation

Every entry point is invoked through the scrubbing launcher (`ARCHITECTURE.md` §8.2):

```bash
./env.sh python -m crackvision.<module> [args]     # library CLIs
./env.sh python scripts/<name>.py [args]           # operational scripts
```

### 0.2 Exit codes (all CLIs)

| Code | Meaning | Example |
|---|---|---|
| 0 | success | |
| 1 | runtime failure | nnU-Net subprocess returned non-zero |
| 2 | usage / config error | `--device banana`; malformed `project.yaml` |
| 3 | precondition not met | model folder missing; `input_originals/` empty; no camera |

### 0.3 Common flags — every CLI accepts all of these

| Flag | Default | Meaning |
|---|---|---|
| `--root PATH` | `$CRACKVISION_ROOT` or auto-detected | project root override |
| `--config PATH` | `config/project.yaml` | config file override |
| `--verbose` / `-v` | off | DEBUG logging |
| `--quiet` / `-q` | off | WARNING+ only |
| `--dry-run` | off | log what would be done, write nothing, exit 0 |
| `--skip-existing` | off | do not regenerate outputs that already exist |

Default behaviour without `--skip-existing` is **overwrite**. Generating CLIs are idempotent.

### 0.4 Logging artefacts

Every CLI writes:
- `logs/<tool>_<YYYYmmddTHHMMSSZ>.log` — full human log (also mirrored to stderr)
- `logs/<tool>_<YYYYmmddTHHMMSSZ>.json` and `logs/<tool>_latest.json` — machine summary:

```json
{
  "tool": "prepare_inputs",
  "schema_version": 1,
  "started_utc": "2026-09-16T18:04:11Z",
  "finished_utc": "2026-09-16T18:04:13Z",
  "duration_s": 1.94,
  "status": "ok",
  "exit_code": 0,
  "counts": {"found": 12, "converted": 12, "skipped": 0, "failed": 0},
  "errors": []
}
```

`status` ∈ `{"ok", "partial", "failed", "precondition"}`. `<tool>` is the module/script basename.

### 0.5 The frame invariant

> **Every raster this pipeline generates has exactly the same `(height, width)` as the original
> input image.** No resize, crop, pad, or rotate — anywhere, ever, unless the user passes the explicit
> `--max-side` downscale flag, in which case the factor is recorded in `case_map.json`.

This is what allows a future stage to index the aligned D405 depth frame with skeleton `(row, col)`.

---

## 1. `case_id` — the join key

### 1.1 Derivation (implement in `crackvision/naming.py::derive_case_id`)

```
stem            = Path(source).stem
s               = re.sub(r"[^A-Za-z0-9-]", "_", stem)
s               = re.sub(r"_+", "_", s).strip("_")
if s.endswith("_0000"): s = s[:-5].rstrip("_")
if not s:               s = "image"
```

Collisions: process sources in `sorted()` order by full path; the first keeps `s`, subsequent get
`s + "__2"`, `s + "__3"`, … This makes the whole mapping deterministic and re-runnable.

**Worked examples** (these are the unit-test fixtures for TC-007):

| Source filename | `case_id` |
|---|---|
| `Deck Crack 01.JPG` | `Deck_Crack_01` |
| `IMG_2043.png` | `IMG_2043` |
| `bridge-pier.7.jpeg` | `bridge-pier_7` |
| `crack (copy).png` | `crack_copy` |
| `  spaced  .bmp` | `spaced` |
| `sample_0000.png` | `sample` |
| `2026-09-16_run1.tif` | `2026-09-16_run1` |
| `....png` | `image` |
| `A.png` then `a!.png` | `A`, then `a__2` (distinct stems → no collision: `A` vs `a`) |
| `x y.png` then `x-y.png`→`x-y`, then `x_y.png`→`x_y` | no collision |
| `a b.png` then `a_b.png` | `a_b`, then `a_b__2` |

### 1.2 Path mapping (implement as pure functions in `naming.py`)

| Function | Returns |
|---|---|
| `nnunet_input_path(case)` | `data/nnunet_input/{case}_0000.png` |
| `prediction_path(case)` | `data/predictions/{case}.png` |
| `mask_path(case)` | `data/overlays/{case}_mask.png` |
| `overlay_path(case)` | `data/overlays/{case}_overlay.png` |
| `comparison_path(case)` | `data/comparisons/{case}_comparison.png` |
| `skeleton_path(case)` | `data/skeletons/{case}_skeleton.png` |
| `skeleton_overlay_path(case)` | `data/skeletons/{case}_skeleton_overlay.png` |
| `skeleton_stats_path(case)` | `data/skeletons/{case}_skeleton_stats.json` |

> **Design note (deviation from the initial sketch, deliberate):** the request sketched
> `overlays/<original_name>_overlay.png`. We key every generated file on `case_id`, not the raw
> original filename, because original names may contain spaces, unicode, `/`-hostile characters and
> duplicate stems across extensions. The original name is preserved verbatim in `case_map.json`, so
> nothing is lost and the reverse lookup is exact. **Do not "fix" this back.**

---

## 2. `data/case_map.json`

Written **in full** by `prepare_inputs` on every run (never merged). Read by `visualize`,
`skeleton`, and `run_test.sh`.

```json
{
  "schema_version": 1,
  "generated_utc": "2026-09-16T18:04:13Z",
  "root": "/home/boosterk1/Projects/rebot_crack_vision",
  "cases": [
    {
      "case_id": "Deck_Crack_01",
      "source_path": "data/input_originals/Deck Crack 01.JPG",
      "source_name": "Deck Crack 01.JPG",
      "nnunet_input": "data/nnunet_input/Deck_Crack_01_0000.png",
      "width": 1280,
      "height": 720,
      "source_mode": "RGB",
      "source_format": "JPEG",
      "converted": true,
      "downscaled": false,
      "scale_factor": 1.0,
      "sha256_source": "…64 hex…",
      "sha256_nnunet_input": "…64 hex…"
    }
  ]
}
```

All paths are **project-relative, POSIX-style**. `cases` is sorted by `case_id`.
A consumer that cannot find `case_map.json` exits **3** with
`"run: ./env.sh python -m crackvision.prepare_inputs first"`.

---

## 3. Component contracts

### 3.1 `crackvision.prepare_inputs` (TC-007)

```
./env.sh python -m crackvision.prepare_inputs [--input-dir DIR] [--output-dir DIR]
                                              [--max-side N] [--clean] [common flags]
```

| | |
|---|---|
| **Reads** | `data/input_originals/**` (non-recursive by default), extensions `.jpg .jpeg .png .bmp .tif .tiff` (case-insensitive) |
| **Writes** | `data/nnunet_input/{case}_0000.png`, `data/case_map.json` |

**Required behaviour**

1. Use **Pillow** (`PIL.Image`) for all conversion. (Pillow, not OpenCV: it handles EXIF and
   palette/CMYK modes cleanly and never silently reorders to BGR.)
2. Apply `PIL.ImageOps.exif_transpose()` so a phone-rotated JPEG is written upright.
3. **Convert every image to exactly 3-channel 8-bit RGB** via `img.convert("RGB")`.
   This is mandatory and non-negotiable — `NaturalImage2DIO` asserts `shape[-1] in (3,4)` and the
   plans declare exactly 3 channels, so RGBA→4ch and grayscale→1ch both break inference
   (`ARCHITECTURE.md` §4.3).
4. Save as **PNG** (`compress_level=6`). PNG is required: `NaturalImage2DIO.supported_file_endings`
   deliberately excludes `.jpg`/`.jpeg`.
5. Never modify or move anything in `data/input_originals/` — it is read-only to this tool.
6. `--max-side N`: if `max(h, w) > N`, downscale with `Image.LANCZOS` preserving aspect, and record
   `downscaled: true` + `scale_factor`. **Off by default** (breaks the frame invariant).
7. `--clean`: delete existing `data/nnunet_input/*.png` before writing (removes stale cases).
8. Exit **3** if the input dir is missing or contains zero supported images; the message must name
   the directory and the accepted extensions.
9. A single unreadable/corrupt file logs an ERROR, increments `counts.failed`, and processing
   continues; final status becomes `"partial"` and exit code `1` if any failed.

### 3.2 `crackvision.inference` (TC-008)

```
./env.sh python -m crackvision.inference [--input-dir DIR] [--output-dir DIR]
                                         [--device cuda|cpu] [--disable-tta] [--not-on-device]
                                         [--step-size F] [--npp N] [--nps N] [common flags]
```

| | |
|---|---|
| **Reads** | `data/nnunet_input/*_0000.png`, `models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/` |
| **Writes** | `data/predictions/{case}.png`, `logs/inference_latest.json` |

**Canonical command it builds** (`subprocess.run`, list form, never `shell=True`):

```bash
nnUNetv2_predict_from_modelfolder \
  -i  <root>/data/nnunet_input \
  -o  <root>/data/predictions \
  -m  <root>/models/opencrack-nnunet/Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d \
  -f  0 \
  -chk checkpoint_ep0500.pth \
  -device cuda \
  -npp 3 -nps 3
```

`-f 0` and `-chk checkpoint_ep0500.pth` are **mandatory overrides** — the defaults are
`(0,1,2,3,4)` and `checkpoint_final.pth`, both of which fail against this model.

**Required behaviour**

1. Preconditions → exit **3** with an actionable message: model folder missing (`run fetch_model.py`),
   `plans.json`/`dataset.json`/`fold_0/checkpoint_ep0500.pth` missing (`run verify_model.py`),
   input dir empty (`run prepare_inputs`), `--device cuda` with `torch.cuda.is_available()` false.
2. Time the subprocess; record `duration_s`, `images`, and `seconds_per_image` in the JSON summary.
3. Stream the subprocess's stdout/stderr into the log (do not swallow it).
4. **OOM handling:** if the subprocess fails and its output matches `out of memory` /
   `CUDA out of memory` / `OutOfMemoryError`, do **not** retry automatically. Log the ladder from
   `ARCHITECTURE.md` §9 as concrete next commands and exit **1**.
5. Verify afterwards that one `data/predictions/{case}.png` exists per input; any missing → status
   `"partial"`, exit **1**.
6. Also expose `run_inference(...) -> dict` as an importable function returning the summary dict.

> **⚠️ Prediction pixel values are `0` and `1`, not `0` and `255`** (`NaturalImage2DIO.write_seg`).
> They look solid black in an image viewer. **This is correct.** Never "fix" it by rescaling the
> prediction file — `visualize` renders the viewable copy. `inference` must log, once per run:
> `"predictions are uint8 with values {0,1}; see data/overlays/ for viewable renders"`.

### 3.3 `crackvision.visualize` (TC-009)

```
./env.sh python -m crackvision.visualize [--cases A B ...] [--alpha F] [--color R,G,B] [common flags]
```

| | |
|---|---|
| **Reads** | `data/case_map.json`, `data/predictions/{case}.png`, the original via `source_path` |
| **Writes** | `data/overlays/{case}_mask.png`, `data/overlays/{case}_overlay.png`, `data/comparisons/{case}_comparison.png` |

1. Binarise defensively: `binary = (pred > 0)`. Never assume 255; never assume 1.
2. `{case}_mask.png` — single-channel uint8, `binary * 255`, same H×W as the original.
3. `{case}_overlay.png` — RGB. `out = original` where `~binary`; where `binary`,
   `out = (1-α)·original + α·colour`. Defaults `α = 0.5`, `colour = (255, 0, 0)` (from
   `config/project.yaml`, overridable by flag).
4. `{case}_comparison.png` — the three panels **original | mask(3-ch) | overlay** concatenated
   horizontally, separated by an 8 px white gutter, each panel labelled in the top-left with a
   readable caption (`ORIGINAL`, `MASK`, `OVERLAY`) plus the crack-pixel percentage on the mask panel.
   Use `PIL.ImageDraw` with `ImageFont.load_default()` — **do not** depend on a TTF file being present.
5. If the original has different dimensions from the prediction, that is a hard error for that case:
   log ERROR, count as failed, continue. (It means the frame invariant was violated upstream.)
6. An empty mask (zero crack pixels) is **not** an error — emit all three files and log INFO
   `"case X: 0 crack pixels (0.000%)"`.

### 3.4 `crackvision.skeleton` (TC-010)

```
./env.sh python -m crackvision.skeleton [--cases A B ...] [--min-component-size N]
                                        [--no-overlay] [common flags]
```

| | |
|---|---|
| **Reads** | `data/case_map.json`, `data/predictions/{case}.png`, original for the overlay |
| **Writes** | `data/skeletons/{case}_skeleton.png`, `{case}_skeleton_overlay.png`, `{case}_skeleton_stats.json` |

Algorithm — **exactly this, nothing more** (`adr/008`):

```
binary  = prediction > 0
cleaned = skimage.morphology.remove_small_objects(binary, min_size=N)   # N default 64, 0 disables
skel    = skimage.morphology.skeletonize(cleaned)                       # default method
```

1. Output `{case}_skeleton.png` — single-channel uint8, `skel * 255`, original H×W.
2. `{case}_skeleton_overlay.png` — skeleton drawn over the original in
   `config.skeleton_color` (default lime `(0,255,0)`), fully opaque, **1 px wide, not dilated**.
   (A `--dilate N` flag may exist for human viewing but must not affect `{case}_skeleton.png`.)
3. `{case}_skeleton_stats.json`:

```json
{"case_id":"Deck_Crack_01","schema_version":1,
 "image_height":720,"image_width":1280,
 "mask_pixels":18432,"mask_fraction":0.0200,
 "components_before":7,"components_after":3,"min_component_size":64,
 "skeleton_pixels":1204,"skeleton_fraction":0.00131}
```

4. **OUT OF SCOPE — do not implement:** branch/endpoint detection, graph construction, path ordering,
   pruning, spur removal, polyline fitting, spline smoothing, `skan`. The endpoint of this phase is a
   one-pixel raster. (`adr/008`)
5. Empty mask → all-zero skeleton, stats with zeros, status ok.

### 3.5 `scripts/check_env.py` (TC-003) — Level 1

```
./env.sh python scripts/check_env.py [--json] [--strict]
```

Reports, each as `PASS` / `WARN` / `FAIL` with the observed value:

| Check | Pass condition |
|---|---|
| Python version | `3.11.x` |
| `PYTHONPATH` clean | contains no `/opt/ros` entry — **proves `env.sh` worked** |
| `sys.path` clean | no `ros/humble` entry |
| torch import + version | `>=2.1.2`, not `2.9.*` |
| `torch.version.cuda` | not None |
| `torch.cuda.is_available()` | True (WARN not FAIL — CPU is a valid mode) |
| GPU name / total VRAM / free VRAM | RTX 3090 reported; free VRAM logged |
| NVIDIA driver | parsed from `nvidia-smi`; WARN if absent |
| `nnunetv2` import + version | importable |
| `nnUNetv2_predict_from_modelfolder` on PATH | `shutil.which` non-None |
| numpy / scikit-image / Pillow versions | importable |
| `pyrealsense2` | WARN-only if absent (optional extra) |
| Project paths | `data/`, `models/`, `nnunet/`, `logs/` exist and are writable |
| nnU-Net env vars | `nnUNet_raw`, `nnUNet_preprocessed`, `nnUNet_results` set and inside the project |
| Model checkpoint discoverable | `models/.../fold_0/checkpoint_ep0500.pth` exists, size ≈ 268 MB |

Exit 0 if no FAIL; exit 3 if any FAIL. `--strict` promotes WARN to FAIL.
Always writes `logs/check_env_latest.json` with every version string — this is the reproducibility
snapshot TC-016 consumes.

### 3.6 `scripts/fetch_model.py` (TC-004)

```
./env.sh python scripts/fetch_model.py [--repo-id ID] [--dest DIR] [--force] [--verify-only]
```

1. `huggingface_hub.snapshot_download(repo_id="fadeevla/opencrack-nnunet", local_dir="models/opencrack-nnunet")`.
   Use `local_dir` (**not** the default hub cache) so the 268 MB lives once, inside the project
   (`~/.cache/huggingface` is already 17 GB).
2. `allow_patterns=["Dataset501_OpenCrack/**", "README.md", "reproducibility/**"]`.
3. Idempotent: re-running with everything present re-verifies and exits 0 without re-downloading.
4. After download, compute sha256 of each file and write `config/model_manifest.json`:

```json
{"schema_version":1,"repo_id":"fadeevla/opencrack-nnunet",
 "downloaded_utc":"…","revision":"<commit sha if available>",
 "files":[{"path":"Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/fold_0/checkpoint_ep0500.pth",
           "bytes":281121234,"sha256":"…"}]}
```

5. Required files (exit 3 if any missing after download):
   `…/_2d/plans.json`, `…/_2d/dataset.json`, `…/_2d/fold_0/checkpoint_ep0500.pth`.
6. Network failure → exit **1** with the underlying error; never leave a half-written manifest.
7. Log the licence: **CC-BY-4.0, attribution required** (`adr/001`).

### 3.7 `scripts/verify_model.py` (TC-005)

```
./env.sh python scripts/verify_model.py [--check-hashes] [--repair-symlink]
```

1. Assert the tree and that `dataset.json` has `channel_names == {"0":"R","1":"G","2":"B"}`,
   `labels == {"background":0,"crack":1}`, `file_ending == ".png"`,
   `overwrite_image_reader_writer == "NaturalImage2DIO"`.
   Assert `plans.json` has a `"2d"` configuration with `patch_size == [256,256]`.
   **A mismatch is a BLOCKER, not something to patch** — upstream changed; report it.
2. Ensure the symlink `nnunet/results/Dataset501_OpenCrack → ../../models/opencrack-nnunet/Dataset501_OpenCrack`
   exists (relative, so the project stays movable). This makes the `-d 501` fallback path work too.
3. `--check-hashes` re-verifies sha256 against `config/model_manifest.json`.
4. Load the checkpoint header safely to confirm it is a real torch archive:
   `torch.load(path, map_location="cpu", weights_only=True)` → assert it is a dict and log its keys
   and `len(state_dict)`. **Never** run this with `weights_only=False`.
5. Exit 0 / 3.

### 3.8 `scripts/smoke_test.py` (TC-006) — Level 2 core

```
./env.sh python scripts/smoke_test.py [--device cuda|cpu] [--keep]
```

1. Generate a deterministic synthetic fixture into `data/smoke_test/input/` (seed fixed):
   a 512×512 mid-grey RGB image with Gaussian noise plus a dark, hand-drawn 2–4 px wide branching
   polyline (the "crack"), and a second **crack-free** noise-only image.
   Plain numpy + PIL — no network, no external assets.
2. Run the real chain on it: prepare → inference → visualize → skeleton, all writing under
   `data/smoke_test/` (**never** into the user's `data/predictions/` etc.).
3. Assertions (this is an execution test, **not** an accuracy test — `adr/001`, VALIDATION_PLAN §L2):
   - every expected output file exists and is non-empty
   - prediction dtype is `uint8` and `set(unique) ⊆ {0,1}`
   - prediction shape == input shape
   - skeleton pixel count ≤ mask pixel count
   - the whole chain exits 0
   - **No assertion on IoU, crack recall, or "did it find the crack".** Log the crack-pixel % for
     human interest and move on.
4. Print a clear `SMOKE TEST PASSED / FAILED` banner and the wall-clock per stage.

### 3.9 `scripts/check_realsense.py` (TC-012)

```
./env.sh python scripts/check_realsense.py [--json]
```

Must run to completion with **no camera attached** (that is the normal case on this box):

| Check | Behaviour with no hardware |
|---|---|
| `import pyrealsense2` | FAIL + install hint if missing |
| library version (`rs.__version__` / `pyrealsense2.pyrealsense2`) | reported |
| `rs.context().query_devices()` | 0 devices → **WARN, not FAIL**; exit 0 |
| per-device: name, serial, firmware, USB descriptor | only if present |
| D405 detection | matches product line / name containing `D405` |
| available stream profiles | enumerated only if a device is present |

Exit 0 when the software stack is fine, even with no device. Exit 3 only if `pyrealsense2` is
unimportable. Always writes `logs/check_realsense_latest.json`.

### 3.10 `crackvision.realsense_capture` (TC-013)

```
./env.sh python -m crackvision.realsense_capture [--session NAME] [--frames N] [--interval S]
        [--color-width W --color-height H --depth-width W --depth-height H --fps F]
        [--no-align] [--preview] [--list-devices] [--synthetic] [common flags]
```

| | |
|---|---|
| **Writes** | `data/d405/color/{session}_{NNNNNN}_color.png`, `data/d405/depth/{session}_{NNNNNN}_depth.png`, `data/d405/metadata/{session}_{NNNNNN}.json`, `data/d405/metadata/{session}_session.json` |

`{session}` defaults to `UTC YYYYmmddTHHMMSSZ`; `{NNNNNN}` is a zero-padded 6-digit frame counter.

1. `import pyrealsense2` is **lazy**, inside the function — the module must import cleanly without it,
   so `tests/` pass on a machine with no RealSense stack.
2. Defaults: colour `848×480 @ 30` `rs.format.bgr8`, depth `848×480 @ 30` `rs.format.z16`.
   Expose overrides; if `enable_stream` raises, catch it and print the profiles the device *does*
   support, then exit 3.
3. **`rs.align(rs.stream.color)` by default** (`--no-align` to disable). Alignment is what makes
   `depth[r,c]` correspond to `color[r,c]` and therefore to `mask[r,c]` — the whole point of this
   phase's frame invariant.
4. Discard the first `warmup_frames` (default **30**) so auto-exposure settles.
5. **Colour** → 8-bit RGB PNG. Convert BGR→RGB once, explicitly, and say so in a comment.
6. **Depth** → 16-bit single-channel PNG of the **raw `uint16` z16 buffer**. Never colorise, never
   scale to 8-bit, never JPEG. Write it with `PIL.Image.fromarray(arr, mode="I;16")` or
   `cv2.imwrite` — whichever the agent verifies round-trips byte-identically, and it must add a unit
   test proving `imread(write(arr)) == arr` for a random `uint16` array.
   A colorised preview, if wanted, is a *separate* `_depthviz.png`.
7. Per-frame metadata JSON:

```json
{"schema_version":1,"session":"20260916T180411Z","frame_index":0,
 "timestamp_utc":"…","device":{"name":"Intel RealSense D405","serial":"…","firmware":"…"},
 "aligned_to":"color","warmup_frames":30,
 "depth_scale_m_per_unit":0.0001,
 "color":{"width":848,"height":480,"fps":30,"format":"bgr8",
          "intrinsics":{"fx":..., "fy":..., "ppx":..., "ppy":..., "model":"…","coeffs":[...]}},
 "depth":{"width":848,"height":480,"fps":30,"format":"z16",
          "intrinsics":{...}},
 "extrinsics_depth_to_color":{"rotation":[9 floats],"translation":[3 floats]},
 "exposure_us":null,"gain":null,
 "files":{"color":"data/d405/color/…","depth":"data/d405/depth/…"}}
```

   **`depth_scale_m_per_unit` MUST come from `depth_sensor.get_depth_scale()`** — the D405 typically
   uses 0.0001 m/unit while other D4xx use 0.001. Hard-coding it silently corrupts every future 3D
   projection. This is a named acceptance criterion in TC-013.
8. `--list-devices` prints devices and exits 0 (works with none attached).
9. `--synthetic` writes plausible fake colour/depth/metadata with **`"synthetic": true`** in the JSON,
   so the file format and the downstream tooling can be exercised with no camera. Synthetic frames
   must never be mistaken for real: the flag is mandatory in the payload and in the filename prefix
   (`synthN_…`).
10. No camera + not `--synthetic` → exit **3** with:
    `"No RealSense device found. Attach a D405 (USB-3), or use --synthetic to exercise the file format."`
11. `--preview` opens an OpenCV window; it must be optional and must degrade to a no-op with a WARN
    when `$DISPLAY` is unset.

### 3.11 `run_test.sh` (TC-011)

```
./run_test.sh [--device cuda|cpu] [--skip-skeleton] [--clean]
```

Orchestrates, stopping at the first non-zero exit:

```
1. ./env.sh python scripts/check_env.py            # exit 3 → "environment not ready"
2. ./env.sh python -m crackvision.prepare_inputs   # exit 3 → "put images in data/input_originals/"
3. ./env.sh python -m crackvision.inference
4. ./env.sh python -m crackvision.visualize
5. ./env.sh python -m crackvision.skeleton
```

Then prints a summary table — per case: dimensions, crack-pixel %, skeleton-pixel count, and the
absolute path of the comparison image — followed by:

```
Done. Open the results:
  <root>/data/comparisons/
  <root>/data/overlays/
```

`set -euo pipefail`. A stage exiting **3** prints that stage's actionable message and exits 3 —
it must **not** print a bash stack trace. The intended UX is exactly:

> 1. put images in `data/input_originals/` → 2. `./run_test.sh` → 3. open `data/comparisons/`.

### 3.12 `tools/dataset_manifest.py` (TC-015)

```
./env.sh python tools/dataset_manifest.py init      --out data/d405/eval_manifest.csv
./env.sh python tools/dataset_manifest.py scan      --metadata-dir data/d405/metadata [--merge]
./env.sh python tools/dataset_manifest.py validate  --in data/d405/eval_manifest.csv
./env.sh python tools/dataset_manifest.py summary   --in data/d405/eval_manifest.csv
```

CSV schema and controlled vocabularies are defined in `docs/D405_TEST_PLAN.md` §3 and are the single
source of truth. `validate` checks required columns, enum membership, `image_id` uniqueness, and that
each referenced colour/depth file exists. **No image data is collected or labelled in this phase** —
this card delivers the tooling and an empty, header-only manifest only.

### 3.13 `crackvision.paths` (PERC-03)

```
./env.sh python -m crackvision.paths [--cases A B ...] [--rdp-tolerance-px PX]
                                     [--min-spur-length-px N] [common flags]
```

| | |
|---|---|
| **Reads** | `data/case_map.json`, `data/skeletons/{case}_skeleton.png` |
| **Writes** | `data/paths/{case}_paths.json` |

Builds the PERC-02 8-connected pixel graph over the `{0,255}` skeleton raster, prunes spurs shorter
than `--min-spur-length-px` (`skeleton_graph.prune_spurs`, default from `skeleton.min_spur_length_px`
in `config/project.yaml`, else 5), then decomposes each connected component into ordered polylines.

**Path-decomposition policy** (`crackvision.paths`, module docstring is normative if this summary and
the code ever disagree):

1. Per component, repeatedly extract the current **graph diameter** — the longest shortest-path
   between any two nodes, found by the standard two-sweep Dijkstra (edge weight = pixel-step count).
   This is exact for tree-shaped components (the large majority of real cracks) and the conventional
   heuristic when a component contains a cycle. The first extraction is the component's **main path**;
   each subsequent extraction, over whatever edges remain, is a **branch**. This repeats until every
   edge is claimed. Branches are then sorted by pixel length, longest first, in the output.
2. **Multi-component ordering** is a deterministic greedy nearest-endpoint tour: components are
   visited in `skeleton_graph` discovery order for the first pick, then each next component is
   whichever remaining one has an endpoint (start or end of its main path — a direction choice)
   nearest the current position; ties break on the lower `component_id`. Not a globally optimal tour —
   cheap and reproducible only.
3. Both the dense path (one point per skeleton pixel) and an RDP-simplified path (tolerance
   `--rdp-tolerance-px`, default `path_extraction.rdp_tolerance_px` in `config/project.yaml`, else
   `1.5`) are kept for every main path and branch, dense first.
4. Where two consecutive edges of a decomposed path don't meet on the same pixel (a PERC-02 edge
   ended on a raw junction pixel next to the merged graph node instead of on it), the gap is
   bridged by the shortest 8-connected route through actual skeleton pixels (a breadth-first search
   over the same raster PERC-02 built its graph from), not a straight line — the dense path stays
   8-connected across every junction/crossing, every inserted pixel is itself on the skeleton, and
   no traversed edge pixel is dropped.

`{case}_paths.json` schema:

```json
{"case_id":"Deck_Crack_01","schema_version":1,
 "image_height":720,"image_width":1280,
 "min_spur_length_px":5,"rdp_tolerance_px":1.5,
 "component_count":1,
 "components":[
   {"order_index":0,"component_id":0,
    "main_path":{"kind":"main","length_px":812,
      "dense":[{"row":10,"col":20,"u":20,"v":10}, "... one entry per skeleton pixel ..."],
      "simplified":[{"row":10,"col":20,"u":20,"v":10}, "... RDP-simplified corners only ..."]},
    "branches":[
      {"kind":"branch","length_px":34,"dense":["..."],"simplified":["..."]}
    ]}
 ]}
```

- Every point is `{"row": int, "col": int, "u": int, "v": int}`: `(row, col)` is the array-convention
  pixel coordinate in the **original** image frame (docs/INTERFACES.md §0.5 — never resized, cropped
  or padded relative to the source); `(u, v) = (col, row)` is the same point in the `pyrealsense2`/
  projection convention (§6.3), spelled out per-point so no downstream consumer has to re-derive it.
- `branches` is `[]` for a component with no branch points; `components` is `[]` for an empty skeleton.
- A component whose entire skeleton is a single isolated pixel emits a one-point `main_path` (`dense
  == simplified == [that pixel]`, `length_px: 0`) and no branches.
- **OUT OF SCOPE — do not implement here:** anything past pixel space — no 3D deprojection, no MoveIt
  waypoints, no robot motion. That is GEOM-0x's job, downstream of this file.

### 3.14 `crackvision.recording` (CAM-02)

Lossless RGB-D recording and deterministic, camera-free replay. This module never imports
`pyrealsense2` — it is pure file I/O over the frame type defined here, so any code written against
it also works with no camera attached.

**`CaptureFrame`** — the frame type every crackvision D405 source (live or recorded) yields:
`frame_index`, `color_rgb` (`uint8`, `(H, W, 3)`), `depth_u16` (`uint16`, `(H, W)`, raw z16 device
units — `depth_u16[r, c]` corresponds to `color_rgb[r, c]`, the frame invariant, §0.5),
`color_timestamp` / `color_timestamp_domain`, `depth_timestamp` / `depth_timestamp_domain`,
`color_frame_number`, `depth_frame_number`, `host_monotonic_ns` (`time.monotonic_ns()` at capture),
and an open `extra` dict for anything else (e.g. `exposure_us`, `gain`). Constructing one with the
wrong dtype/ndim for either array raises `ValueError`.

**`RecordingWriter(root, *, session, device, color, depth, color_intrinsics,
depth_scale_m_per_unit, extrinsics_depth_to_color, aligned_to="color")`** writes a session under
`root`:

```
root/session.json
root/color/{NNNNNN}_color.png   -- uint8 RGB, lossless PNG
root/depth/{NNNNNN}_depth.png   -- uint16 z16 units, lossless 16-bit PNG (PIL "I;16"), never scaled
root/frames/{NNNNNN}.json       -- per-frame sidecar
```

`.write_frame(frame)` rejects (raises `ValueError`, writes nothing) any frame whose `color_rgb` /
`depth_u16` shape doesn't match the session's declared `color`/`depth` stream config — the frame
invariant applies to recordings too: no resize/crop, ever. `.close()` (also called by `__exit__`)
writes `session.json`:

```json
{"schema_version":1,"session":"sess01",
 "device":{"serial":"…","firmware":"…","usb_type":"3.2"},
 "color":{"width":848,"height":480,"fps":30,"format":"rgb8"},
 "depth":{"width":848,"height":480,"fps":30,"format":"z16"},
 "color_intrinsics":{"fx":...,"fy":...,"ppx":...,"ppy":...,"model":"…","coeffs":[...],
                      "width":848,"height":480},
 "depth_scale_m_per_unit":0.0001,
 "extrinsics_depth_to_color":{"rotation":[9 floats],"translation":[3 floats]},
 "aligned_to":"color","frame_count":123}
```

`device` must carry `serial`, `firmware`, `usb_type`; `color_intrinsics` must carry `fx`, `fy`,
`ppx`, `ppy`, `model`, `coeffs`, `width`, `height`; `extrinsics_depth_to_color` must carry
`rotation`, `translation` — `RecordingWriter.__init__` raises `ValueError` if any are missing.
**`depth_scale_m_per_unit` must be the value the caller read from `depth_sensor.get_depth_scale()`**
(§3.10 item 7) — this module has no default and never guesses one.

Per-frame sidecar JSON: `schema_version`, `frame_index`, `color_timestamp`,
`color_timestamp_domain`, `depth_timestamp`, `depth_timestamp_domain`, `color_frame_number`,
`depth_frame_number`, `host_monotonic_ns`, `extra`, `files: {color, depth}` (paths relative to
`root`).

**`RecordingReader(root)`** reads a session back: `.session_info` is the parsed `session.json`;
`len(reader)` is the frame count; `.read_frame(i)` / iteration yield bit-exact `CaptureFrame`s in
recorded order — `color_rgb`/`depth_u16` compare equal (`np.array_equal`) to what was written,
because PNG (RGB8 and 16-bit grayscale) is a lossless codec and both arrays round-trip through it
unchanged.

**`ReplaySource(root)`** wraps a `RecordingReader` as a deterministic, no-camera stand-in for a live
source: `.session_info`, `len(...)`, and iteration yielding the same `CaptureFrame`s, in the same
order, on every pass — so code written against a live D405 stream can consume a recorded session
with no branch.

---

## 4. `config/project.yaml`

```yaml
schema_version: 1
paths:
  input_originals: data/input_originals
  nnunet_input:    data/nnunet_input
  predictions:     data/predictions
  overlays:        data/overlays
  comparisons:     data/comparisons
  skeletons:       data/skeletons
  smoke_test:      data/smoke_test
  d405:            data/d405
  models:          models/opencrack-nnunet
  logs:            logs
model:
  repo_id:        fadeevla/opencrack-nnunet
  dataset_name:   Dataset501_OpenCrack
  trainer_config: nnUNetTrainer__nnUNetPlans__2d
  fold:           0
  checkpoint:     checkpoint_ep0500.pth
  configuration:  2d
  dataset_id:     501
inference:
  device:         cuda
  step_size:      0.5
  disable_tta:    false
  not_on_device:  false
  npp:            3
  nps:            3
visualization:
  overlay_alpha:   0.5
  crack_color:     [255, 0, 0]
  skeleton_color:  [0, 255, 0]
  panel_gutter_px: 8
skeleton:
  min_component_size: 64
path_extraction:
  rdp_tolerance_px: 1.5
realsense:
  color: {width: 848, height: 480, fps: 30, format: bgr8}
  depth: {width: 848, height: 480, fps: 30, format: z16}
  align_to: color
  warmup_frames: 30
```

`crackvision.config` loads this, applies defaults for anything missing, resolves every path against
the project root, and raises a clear error (exit **2**) on a malformed file. **CLI flags always win
over the YAML.**

---

## 5. File ownership

Exactly one card creates any given file. No card edits another card's file except where the table in
`task_cards/TASK_INDEX.md` §File ownership explicitly permits an append (e.g.
`docs/COMPLETION_LOG.md`, which every card appends to, and `README.md`, which TC-016 finalises).

---

## 6. Frames, units and pixel conventions

**Normative source:** `docs/adr/012-frames-and-conventions.md`. This section is a lookup summary;
the ADR is authoritative if the two ever disagree.

### 6.1 Frames

| Frame | Meaning |
|---|---|
| `base_link` | Robot base, fixed. Calibrated 3D crack-path points are expressed here before MoveIt planning. |
| `tool0` | **Not present in the URDF today.** New fixed alias frame, identical to `gripper_link` (the actual last link of the arm's kinematic chain — `~/rebot_ws/src/rebotarm_bringup/.../reBot_B601_DM_with_gripper.urdf` has no `tool0`). GEOM-06/07 publishes the static `gripper_link → tool0` transform; nothing may assume it resolves in `tf2` before then. |
| `TCP` | Not a new frame — this project's name for the MoveIt config's existing `gripper_tcp` frame (`rebotarm.urdf.xacro`, SRDF tip link), a fixed child of `gripper_link` offset `-0.0443 m` along its `+X` (prior evidence; GEOM-07 measures the real offset). |
| `camera_link` | D405 body/mount frame, `+x` forward along the housing. |
| `camera_color_frame` | Intermediate ROS driver frame between `camera_link` and the optical frame; offset from `camera_link` by the per-device depth→colour extrinsic (real translation, still `camera_link`'s mechanical axes — see §6.2 note). |
| `camera_color_optical_frame` | `+z` forward, `+x` right, `+y` down (ROS optical-frame convention, REP-103). All pixel projection math (§6.3) happens here. |

### 6.2 Transform naming and units

`T_a_b` maps a point from frame `b` into frame `a`: `p_a = T_a_b · p_b`; composition is
`T_a_c = T_a_b · T_b_c`. Quaternion order is **ROS `(x, y, z, w)`**. Units are **metres and radians**
everywhere except raw depth storage (§6.4), which is `uint16` device counts until scaled.

`camera_link → camera_color_optical_frame` is **not** a single device-independent rotation: it
composes the per-device `camera_link → camera_color_frame` extrinsic (real translation, not zero —
related to but not directly usable as §3.10 item 7's `extrinsics_depth_to_color`, which is in the
opposite direction and mechanical/optical axis convention; read this transform from
`realsense2_camera`'s published static TF, never hand-built from the metadata) with the fixed,
translation-free `camera_color_frame → camera_color_optical_frame` mechanical→optical rotation
convention. See ADR-012 for the full derivation.

### 6.3 Pixel convention

- **On disk / in arrays:** `(row, col)` — `numpy`/`PIL` indexing, `arr.shape == (height, width)`.
- **`pyrealsense2` projection (`rs2_project_point_to_pixel` / `rs2_deproject_pixel_to_point`):**
  `pixel = (u, v) = (col, row)` — the transpose of the array convention. Never pass `(row, col)`
  into these calls.
- **Pixel-centre convention:** an integer pixel coordinate addresses that pixel's *centre*, with
  **no `+0.5` corner offset** — confirmed against the librealsense reference implementation
  (`rsutil.h`, `x = (pixel[0] - ppx) / fx`; see ADR-012 for the exact cited source). GEOM-02's
  projection/deprojection tests must match this exactly.

### 6.4 Depth

`depth_m = uint16_value * depth_scale_m_per_unit`, where `depth_scale_m_per_unit` comes from that
frame's own metadata JSON (§3.10 item 7) — never hard-coded. Invalid depth is `uint16_value == 0`,
or a value outside the documented valid band for the device/use-case (D405 default: **≈ 0.07 m –
0.50 m**, per `docs/ARCHITECTURE.md`). Both cases must be treated as absent, never projected as a 3D
point at the camera origin.

### 6.5 Boresight / TCP

The gripper's measured pointing axis in `docs/TECHNICAL_APPROACH.md` §2.2–2.3 (simulation-derived:
local `+X`) is **prior evidence only**, informing but not substituting for GEOM-07's physical
pivot/boresight calibration. No code may treat it as calibrated fact before GEOM-07 reports measured
residuals.

---

## 7. Reachability map and specimen placement (MOT-04)

Runs under the system python3 (ROS Humble), invoked via `scripts/ros/env_ros.sh`, not
`./env.sh` — these are `rclpy`-side tools, distinct from the `crackvision.*` CLIs in §3.

### 7.1 Reachability config

`config/motion/reachability.yaml` (schema `crackvision.reachability_config/1`) is the sweep's
single source of truth for the grid, orientation sampling, IK tuning, surface-collision geometry
and placement search parameters. It is loaded and validated by
`ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py::load_config`
(`ConfigError` on any structural or numeric violation) — see that module's docstring and
`config/motion/reachability.yaml`'s own comments for the field-by-field definitions; this section
does not repeat them.

### 7.2 Reachability map JSON

Default path `data/motion/reachability_map.json` (git-ignored — this is generated evidence, not a
committed input). Built, written and validated by
`ros2_ws/src/crackvision_motion/crackvision_motion/reachability_map.py`. Frame `base_link`, units
metres/radians, quaternion order **`(x, y, z, w)`** throughout, matching §6.2.

```json
{
  "schema": "crackvision.reachability_map/1",
  "created_utc": "2026-09-28T12:00:00Z",
  "git_commit": "d1638163...",
  "config_path": "config/motion/reachability.yaml",
  "config_sha256": "<64 hex>",
  "frame": "base_link",
  "units": {"length": "m", "angle": "rad"},
  "quaternion_order": "xyzw",
  "robot": {
    "group": "arm",
    "ik_link": "gripper_tcp",
    "ik_solver": "trac_ik",
    "reach_bound_m": 0.9,
    "limits_file": "config/robot/b601_dm_limits.yaml",
    "limits_sha256": "<64 hex>"
  },
  "boresight": {"axis_local": [1.0, 0.0, 0.0], "provenance": "prior_evidence"},
  "grid": {
    "grid": {"x_m": {"min": 0.05, "max": 0.50, "step": 0.025}, "y_m": {"...": "..."},
             "surface_z_m": [0.0, 0.04], "standoffs_m": [0.01, 0.04]},
    "orientation": {"roll_samples": 8, "tilt_deg": [0.0], "tilt_azimuth_samples": 4},
    "ik": {"timeout_s": 0.02, "avoid_collisions": true, "seed": "neighbour", "roll_search": "best"},
    "surface_collision": {"enabled": true, "thickness_m": 0.02, "margin_m": 0.05,
                           "allowed_links": ["base_link", "link1"]}
  },
  "complete": true,
  "duration_s": 842.1,
  "targets": [
    {
      "target_id": "z00_x000_y000_s0", "x": 0.05, "y": -0.40, "surface_z": 0.0, "standoff_m": 0.01,
      "position": [0.05, -0.40, 0.01], "status": "reachable",
      "ik_calls": 3, "ik_time_s": 0.006, "error": "",
      "tilt_deg": 0.0, "azimuth_rad": 0.0, "roll_rad": 0.0,
      "quat_xyzw": [0.0, 0.0, 0.0, 1.0],
      "joints": {"joint1": 0.1, "joint2": -0.2, "joint3": -0.3, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0},
      "min_joint_limit_margin_rad": 0.42, "fk_position_error_m": 0.0003, "fk_axis_error_deg": 0.1
    }
  ],
  "summary": {
    "counts_by_status": {"reachable": 1, "unreachable": 0, "prefiltered": 0, "fk_mismatch": 0, "error": 0},
    "reachable_fraction_by_surface_z": {"0": 1.0}
  }
}
```

`grid` echoes the *normalised* config `grid`, `orientation`, `ik` and `surface_collision` blocks
(post-`load_config`, tuples as JSON lists) verbatim, so a consumer never has to re-load and
re-validate the YAML just to know what was swept. `robot.ik_solver` is a free-text string naming
the MoveIt IK plugin actually used (e.g. `"trac_ik"` — see `config/robot/b601_dm_limits.yaml`'s
provenance note on the TRAC-IK switch). `robot.reach_bound_m` is the `reach_bound_from_urdf`
conservative bound used for prefiltering. `summary.reachable_fraction_by_surface_z` keys are the
`surface_z` values formatted with Python `format(z, ".6g")` (e.g. `"0"`, `"0.04"`).

`targets` has exactly one entry per `reachability_core.enumerate_targets()` result, in that order.
Every entry has `target_id, x, y, surface_z, standoff_m, position, status, ik_calls, ik_time_s,
error` (`error` is `""` unless `status` is `error` or `fk_mismatch`, in which case it names the
failure). A `reachable` entry additionally has `tilt_deg, azimuth_rad, roll_rad, quat_xyzw` (4
floats), `joints` (all of `joint1`..`joint6`, radians), `min_joint_limit_margin_rad` (from
`joint_limit_margin` against `config/robot/b601_dm_limits.yaml`), `fk_position_error_m` and
`fk_axis_error_deg`.

**Status semantics** (assigned by the MOT-04.4 sweep node, defined here so every consumer agrees):

| Status | Meaning |
|---|---|
| `reachable` | IK succeeded with collision checking, and an independent FK re-check agreed with the commanded pose within 1 mm / 0.5°. |
| `unreachable` | Every orientation sample at this target failed IK. |
| `prefiltered` | Beyond `robot.reach_bound_m`, provably unreachable by `reachability_core.prefiltered`; IK was never called. |
| `fk_mismatch` | IK reported success but the FK re-check disagreed by more than 1 mm / 0.5° — `error` names the discrepancy. |
| `error` | A service call failed (timeout, exception) rather than IK cleanly reporting infeasible. |

`complete: false` means the sweep was interrupted (crash, timeout, operator abort) before every
target got a final status. **Consumers MUST refuse a map with `complete: false`** — treat it the
same as a missing file, never as partial ground truth.

### 7.3 Specimen placement YAML

Default path `config/motion/specimen_placement.yaml` (schema `crackvision.specimen_placement/1`),
written by MOT-04.3's `recommend_placement` from a completed reachability map. **This is a nominal
recommendation, not a measurement**: the operator physically places the specimen at
`placement.center_xy_m`/`yaw_rad`, and MOT-10 measures the real, as-placed pose, which then
supersedes this file for every downstream consumer.

```yaml
schema: crackvision.specimen_placement/1
value_status: nominal
feasible: true
frame: base_link
placement:
  center_xy_m: [0.10, 0.00]
  surface_z_m: 0.04
  yaw_rad: 0.0
  footprint_m: [0.20, 0.20]
  tolerance_m: 0.02
  standoffs_m: [0.01, 0.04]
  score_min_joint_margin_rad: 0.42
alternatives:
  - {center_xy_m: [0.10, 0.05], surface_z_m: 0.04, yaw_rad: 0.0, footprint_m: [0.20, 0.20],
     tolerance_m: 0.02, standoffs_m: [0.01, 0.04], score_min_joint_margin_rad: 0.39}
max_feasible_square_m: null
map_sha256: "<64 hex>"
config_sha256: "<64 hex>"
boresight_provenance: prior_evidence
caveats:
  - "Nominal recommendation only; the operator places the specimen and MOT-10 measures and
     supersedes this file with the real, as-placed pose."
source: "recommend_placement (MOT-04.3)"
```

`placement` is `null` (with `feasible: false`) when no candidate satisfies the scoring rule
anywhere on the grid; `max_feasible_square_m` is then the side length of the largest fully-feasible
square found (for operator feedback on how much to shrink the footprint), and is otherwise absent
or `null`. `alternatives` holds up to `placement.top_k` (from `config/motion/reachability.yaml`)
further candidates, same shape as `placement`, ranked after the winner. `map_sha256`/
`config_sha256` pin the exact `reachability_map.json` and `reachability.yaml` this recommendation
was computed from; `boresight_provenance` copies the map's `boresight.provenance` so a consumer
can see at a glance that it is not yet GEOM-07-calibrated. `caveats` always includes the
nominal-recommendation disclaimer above.

**Scoring rule** (normative; implemented in `crackvision_motion/placement.py`). Grid geometry
(x/y/surface_z/standoffs) always comes from the *map* being scored; only `footprint_m`,
`tolerance_m`, `yaw_candidates_rad` and `top_k` come from the reachability config's `placement`
block. For each `surface_z` in the map's grid, each `yaw` in `placement.yaw_candidates_rad`, and
each candidate centre = every grid `(x, y)` node in the map:

> 1. The footprint rectangle is `footprint_m` = (w along the specimen's local x, d along its local
>    y), rotated by `yaw` about the centre, then dilated by `tolerance_m` on every side (this
>    dilation models operator placement error).
> 2. The candidate is `out_of_grid`, and therefore infeasible, if any corner of the dilated
>    rectangle lies outside `[x.min, x.max] x [y.min, y.max]` (the map's grid bounds).
> 3. Sample nodes are all map grid nodes inside the dilated rectangle, with an inclusive test using
>    a 1e-9 epsilon.
> 4. The candidate is feasible iff every (node, standoff) for all of the map's configured
>    standoffs has `status == reachable`.
> 5. Score = min over those (node, standoff) of `min_joint_limit_margin_rad`.
> 6. Ranking: score desc (compared after rounding to 1e-6), then smaller `|center_xy_m[1]|`
>    (closer to the arm's y=0 centreline), then smaller `yaw_rad`, then smaller `center_xy_m[0]`,
>    then smaller `surface_z_m`.
>
> The winner is rank 1. `alternatives` holds up to `placement.top_k - 1` further candidates, taken
> in rank order, skipping any candidate whose centre is within one grid step (in both x and y) of a
> better-ranked candidate already selected (the winner or an earlier alternative).
>
> If nothing is feasible: `feasible: false`, `placement: null`, and `max_feasible_square_m` is the
> largest square side `s` (searched downward from `min(footprint_m)` in steps of the map's grid
> step, at `yaw = 0` and the same `tolerance_m`) that has at least one feasible placement on the
> map, or `0` if none does.

### 7.4 CLIs

| CLI | Card | Exit codes |
|---|---|---|
| `scripts/ros/run_reachability.sh` | MOT-04.4 | 0 ok · 1 runtime/FK-mismatch/require-all failed · 2 config · 3 precondition (MoveIt services absent) |
| `reachability_sweep` | MOT-04.4 | 0 ok · 1 runtime/FK-mismatch/require-all failed · 2 config · 3 precondition (MoveIt services absent, input config invalid target) |
| `recommend_placement` | MOT-04.3 | 0 ok · 1 no feasible placement found · 2 config · 3 precondition (map missing or `complete: false`) |

All three are `rclpy`-side tools run via `scripts/ros/env_ros.sh` (never `./env.sh`) and follow the
§0 conventions (§0.2 exit codes, §0.3 common flags via
`ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py::add_common_args`, §0.4 log
artefacts via `cli_common.py::run_cli`).

---

## 8. End-of-arm frames, task frames and the eye-in-hand capture contract (ADR-014)

**Normative source:** `docs/adr/014-end-effector-frames-and-task-phases.md`. This section amends §6.1
(`TCP` row) and §6.5 for everything that faces a surface, and extends §7. Earlier sections are not edited.

### 8.1 `config/robot/end_effector.yaml` (schema `crackvision.end_effector/1`)

This file is the single source of truth for everything rigidly attached to `gripper_link`. It is validated
by `ros2_ws/src/crackvision_description/crackvision_description/end_effector.py::load_config`
(`EndEffectorError`). All transforms are `T_gripper_link_<child>`, in metres, with URDF fixed-axis rpy.

| Block | Frame(s) | Meaning |
|---|---|---|
| `tool` | `tool_tip` | crack-facing tool reference; `pointing_axis` is its boresight (+X) |
| `wrist_camera` | `camera_link` | D405 depth/left-imager origin, realsense-ros convention (+x optical axis, +y left, +z up); frame name fixed to `camera_link` |
| `collision[]` | one link per `id` | padded boxes (`collision_padding_m`) for the mount and the housing |
| `allowed_self_collisions` | — | rigid pairs only; every pair names at least one overlay link |

Every block carries `value_status: nominal|measured`, `provenance` and `source`. `assert_commissioning_ready(cfg)`
raises `EndEffectorNotCalibratedError` while any block is nominal. MOT-05 must call it before real motion.
Calibration cards (GEOM-05 camera, GEOM-07 tool) replace blocks with measured values. They never edit
the xacro.

### 8.2 Planning model

`crackvision_description/urdf/b601_dm_end_effector.urdf.xacro` (args `end_effector_config` = absolute
path, `nominal_camera_frames` default `false`) = the vendor `rebotarm.urdf.xacro`, unchanged, plus the
§8.1 frames and proxies. `srdf/b601_dm_end_effector.srdf.xacro` = the vendor SRDF plus the
`allowed_self_collisions` pairs.

`crackvision_motion/launch/mock_planning.launch.py` loads both, from `$CRACKVISION_ROOT/config/robot/end_effector.yaml`
(override: `$CRACKVISION_END_EFFECTOR_CONFIG`). It refuses to start on an invalid file. The SRDF `arm`
chain tip stays `gripper_tcp` (grasping).

Surface tasks target the task frame directly: `ik_link_name: tool_tip` or `camera_link`. MoveIt resolves
frames rigidly attached to the chain tip. `ros2 run crackvision_motion check_end_effector` verifies the
live model against the file; it is read-only and follows the §0 exit codes.

| Task | Task frame | Distance along the frame's +X to the surface point |
|---|---|---|
| view (capture) | `camera_link` | viewing distance, nominal 0.25 m (0.20–0.30) |
| approach / retract | `tool_tip` | tool clearance, nominal 0.04 m |
| no-contact trace | `tool_tip` | tool clearance, nominal 0.01 m |

### 8.3 Reachability additions to §7 (backward compatible)

- `ik_link` may be any task frame from §8.2. §7's `standoffs_m` / map `standoff_m` are distances of
  *that frame* from the surface point along its boresight.
- `surface_collision.model`: `slab` (default; §7 behaviour) or `specimen_block`. The latter adds
  `floor_z_m` and `base_keepout_m: [x_min, x_max, y_min, y_max]`. The proxy is a block from `floor_z_m` up
  to 1 mm under each `surface_z`, over grid + `margin_m`, minus the keep-out. Every grid node must lie
  outside the keep-out, and no `surface_z` may lie below `floor_z_m` (`ConfigError`). Geometry:
  `reachability_core.specimen_proxy_boxes`.
- Optional top-level `environment: {scene_config: <repo-relative crackvision.scene_config/1>, objects: [ids]}`.
  These static workcell objects, plus the scene's allowed-collision entries that touch them, are applied
  for the whole sweep. The map records them as
  `grid.environment = {scene_config, scene_config_sha256, objects}`.
- `recommend_placement --emit-view-config PATH` writes a camera view check: `camera_link` at 0.25 m above
  the placement centre, tilt ∈ {0°, 15°}, specimen proxy widened to the whole dilated footprint. A
  placement counts as usable only if its half-step verification **and** this view check pass with
  `--require-all-reachable`.
- `caveats` always starts with the §7.3 nominal disclaimer (verbatim). It then names the task frame and
  the collision model the recommendation rests on.

### 8.4 Eye-in-hand capture contract

A capture whose pixels are lifted to `base_link` must carry, taken at the capture instant:
- the arm joint state (`joint1..joint6`, rad) and its timestamp;
- the `end_effector.yaml` sha256 and its `wrist_camera.value_status`.

`p_base_link = FK(q) · T_gripper_link_camera_link · T_camera_link_camera_color_optical_frame · p_optical`,
with the last factor taken from the driver's TF (§6.2) or the device record. A capture without these
fields is valid for 2D perception only. Any card that records or consumes robot-mounted captures
(CAM-05, GEOM-08, INT-02, OPS-01) must implement this contract.

---

## 9. Eye-in-hand capture record (`crackvision.capture_3d/1`)

**Normative source:** implements §8.4. **File:** `data/captures/{case}_capture.json`, keyed by the
§1 `case_id`. Never committed (contains a live device/robot snapshot, not a build artefact).

### 9.1 Fields

| Field | Type | Meaning |
|---|---|---|
| `schema` | str | `"crackvision.capture_3d/1"` |
| `case_id` | str | §1 case id |
| `synthetic` | bool | mandatory; `true` for any generated/fake scene, never omitted |
| `image.height`, `image.width` | int | must equal the colour image, the aligned depth PNG, the mask and `paths.json`'s `image_height`/`image_width` (§0.5) |
| `color_intrinsics` | object | `{width, height, fx, fy, ppx, ppy, model, coeffs[5]}` — the §3.10 colour intrinsics, consumed via `geometry.Intrinsics` |
| `depth.file` | str | repo-relative path to the `uint16` z16 PNG, aligned to colour (§3.10 item 6) |
| `depth.depth_scale_m_per_unit` | float | from the device metadata (§3.10 item 7); never defaulted |
| `depth.aligned_to` | str | `"color"` |
| `source.kind` | str | `d405_metadata` \| `recording` \| `synthetic` |
| `source.ref` | str | repo-relative path to the originating file |
| `source.frame_index` | int \| null | frame index within `source.ref`, if applicable |
| `capture_stamp_ns` | int | image capture time |
| `clock` | str | `"utc_epoch_ns"` — the only clock defined in v1 |
| `robot.joint_names` | list[str] | exactly `["joint1", ..., "joint6"]`, in that order |
| `robot.positions_rad` | list[float] | 6 values, radians, same order as `joint_names` |
| `robot.stamp_ns` | int | joint-state timestamp, same `clock` |
| `robot.stamp_source` | str | where the stamp came from (e.g. `/joint_states`, `moveit_planning_scene`) |
| `robot.kinematic_model.path` | str | repo-relative URDF used for FK |
| `robot.kinematic_model.sha256` | str | sha256 of that URDF's bytes |
| `end_effector.path` | str | `"config/robot/end_effector.yaml"` |
| `end_effector.sha256` | str | sha256 of that file's bytes at capture time |
| `end_effector.wrist_camera_value_status` | str | `nominal` \| `measured`, copied from the file at capture time |
| `end_effector.tool_value_status` | str | `nominal` \| `measured`, copied from the file at capture time |
| `camera_optical.T_camera_link_camera_color_optical_frame` | object | `{xyz_m[3], quat_xyzw[4]}` |
| `camera_optical.source` | str | `driver_tf` \| `nominal_d405` |

`nominal_d405` (used when no live driver TF is available, e.g. `--synthetic` or offline replay) is
zero translation, rpy `(−π/2, 0, −π/2)`: optical `+z` = `camera_link` `+x`, optical `+x` = `−y`,
optical `+y` = `−z`. This is the realsense-ros nominal for the D405, whose colour stream is the left
imager.

### 9.2 Chain

```
p_base_link = FK(q) · T_gripper_link_camera_link · T_camera_link_camera_color_optical_frame · p_optical
```

(exactly as §8.4 / ADR-014 §4, in ADR-012 `T_a_b` notation, all quaternions `(x, y, z, w)`).

- `FK` uses the canonical gripper model (`docs/motion/ROBOT_MODEL.md`). The committed copy
  `presentation/sim/reBot_B601_DM_with_gripper.urdf` has kinematics identical to the vendor model.
- `T_gripper_link_camera_link` comes from `end_effector.yaml`'s `wrist_camera` block (§8.1).
- `T_camera_link_camera_color_optical_frame` is taken from the driver's published static TF (§6.2)
  when `camera_optical.source == "driver_tf"`, or from `nominal_d405` above otherwise. It is never
  hand-built from `color_intrinsics` or from `depth_scale_m_per_unit`.
- **Caveat (open item for MOT-09 / GEOM-09):** the real driver's `robot_description`
  (`reBot-DevArm_fixend.urdf`) places joint6's origin 4.3 mm differently from the canonical gripper
  model above. Joint *values* (`robot.positions_rad`) are shared between the two models; FK for this
  contract always follows the canonical/MoveIt gripper model, not the driver's URDF, so a consumer
  must not silently substitute the driver's kinematic tree here.

### 9.3 Example

```json
{"schema":"crackvision.capture_3d/1","case_id":"Deck_Crack_01","synthetic":false,
 "image":{"height":480,"width":848},
 "color_intrinsics":{"width":848,"height":480,"fx":425.3,"fy":425.3,"ppx":424.1,"ppy":239.6,
   "model":"Brown Conrady","coeffs":[0.0,0.0,0.0,0.0,0.0]},
 "depth":{"file":"data/d405/depth/20260930T101500Z_000012_depth.png",
   "depth_scale_m_per_unit":0.0001,"aligned_to":"color"},
 "source":{"kind":"d405_metadata","ref":"data/d405/metadata/20260930T101500Z_000012.json","frame_index":12},
 "capture_stamp_ns":1780300500000000000,"clock":"utc_epoch_ns",
 "robot":{"joint_names":["joint1","joint2","joint3","joint4","joint5","joint6"],
   "positions_rad":[0.12,-0.44,1.02,0.0,0.77,0.0],
   "stamp_ns":1780300500031000000,"stamp_source":"/joint_states",
   "kinematic_model":{"path":"presentation/sim/reBot_B601_DM_with_gripper.urdf",
     "sha256":"…64 hex…"}},
 "end_effector":{"path":"config/robot/end_effector.yaml","sha256":"…64 hex…",
   "wrist_camera_value_status":"nominal","tool_value_status":"nominal"},
 "camera_optical":{"T_camera_link_camera_color_optical_frame":
   {"xyz_m":[0.0,0.0,0.0],"quat_xyzw":[-0.5,0.5,-0.5,0.5]},"source":"nominal_d405"}}
```

### 9.4 Refusal rules

Consumers raise, and CLIs built on this contract exit **3**, for any of:

- the `robot` block is missing or incomplete, or a joint value is outside the URDF limits by more
  than `1e-3` rad;
- `end_effector.sha256` is missing;
- `|capture_stamp_ns − robot.stamp_ns| >` the max skew (default **0.1 s** — the arm must be
  stationary at capture);
- `image.height`/`image.width` do not match the colour image, aligned depth PNG, mask, or
  `paths.json`'s `image_height`/`image_width` (§0.5);
- the case's `case_map.json` entry has `downscaled: true` (a downscaled case is refused outright —
  §0.5's pixel-index guarantee no longer holds);
- `end_effector.sha256` differs from the current `config/robot/end_effector.yaml` file, **unless**
  the consumer is given an explicit override. With the override, the output records both hashes
  (`sources.end_effector.sha256_at_capture` and `sha256_now`, §10.1) and the resulting product is
  ineligible for execution (§10.6, `end_effector_changed_since_capture`).

A capture record without a `robot` block is valid for 2D perception only (§8.4) and must not be fed
into §10's lifting pipeline.

---

## 10. Robot-frame 3D crack paths and tool waypoints (`crackvision.paths3d/1`)

**File:** `data/paths3d/{case}_paths3d.json`. Consumes a §9 capture record plus the §3.13
`{case}_paths.json` for the same case.

### 10.1 Top-level fields

| Field | Type | Meaning |
|---|---|---|
| `schema` | str | `"crackvision.paths3d/1"` |
| `case_id` | str | §1 case id |
| `frame` | str | `"base_link"` |
| `image_height`, `image_width` | int | copied from the §9 capture record |
| `sources.paths_json` | str | repo-relative path |
| `sources.paths_json_sha256` | str | sha256 of that file |
| `sources.capture_record` | str | repo-relative path to the §9 capture used |
| `sources.capture_record_sha256` | str | sha256 of that file |
| `sources.mask` | str | repo-relative path to the crack mask used for surface sampling |
| `sources.mask_sha256` | str | sha256 of that file |
| `sources.end_effector.sha256_at_capture` | str | from the §9 record |
| `sources.end_effector.sha256_now` | str | sha256 of the current `config/robot/end_effector.yaml` at generation time |
| `sources.urdf_sha256` | str | sha256 of `robot.kinematic_model.path` (§9) |
| `parameters` | object | every tunable in §10.2–§10.4, see below |
| `calibration.wrist_camera` | object | `{value_status, position_sigma_m, rotation_sigma_rad, source}` |
| `calibration.tool` | object | `{value_status, position_sigma_m, source}` |
| `uncertainty_model.terms` | list[str] | modelled error terms (§10.4) |
| `uncertainty_model.unmodelled` | list[str] | named, explicitly not modelled (§10.4) |
| `execution_eligible` | bool | §10.6 |
| `ineligible_reasons` | list[str] | every §10.6 reason that applies; `[]` iff `execution_eligible` |
| `counts.points` | int | total dense points across all polylines |
| `counts.valid` | int | points with non-null geometry (includes `interpolated: true`) |
| `counts.interpolated` | int | points filled by the gap policy (§10.3) |
| `counts.invalid_by_reason` | object | `{reason: count}` for every point left invalid |

`parameters` must contain every value named in §10.2–§10.4 that is not already a `calibration.*`
field: `annulus_inner_px`, `annulus_outer_px`, `annulus_min_fraction_valid`, `valid_depth_range_m`,
`max_gap_px`, `sigma_px`, `waypoint_spacing_m`, `approach_retract_offset_m`, `trace_clearance_m`,
`roll_free` (bool), `max_clock_skew_s`.

### 10.2 `components[]`

Mirrors `paths.json` (§3.13): `order_index`, `component_id`, `polylines[]` with `kind: main|branch`
and `branch_index` (`null` for `main`, `0`-based index within the component's branch list for a
branch — matching the sorted-by-length order §3.13 already fixes).

Each polyline:

| Field | Type | Meaning |
|---|---|---|
| `points[]` | list | one entry per **dense** pixel of the source polyline, in `paths.json` order |
| `segments[]` | list | `{start_index, end_index}` (inclusive, into `points`) `+ waypoints[]` |

Each `points[]` entry:

| Field | Type | Meaning |
|---|---|---|
| `row`, `col`, `u`, `v` | int | copied from the source `paths.json` point (§3.13) |
| `valid` | bool | whether this point carries geometry |
| `reason` | str \| null | invalid reason (`depth_invalid`, `annulus_insufficient`, `normal_fit_failed`, …), `null` when valid |
| `interpolated` | bool | filled by the §10.3 gap policy rather than measured |
| `depth_m` | float \| null | sampled surface depth (annulus method, §10.3) |
| `fraction_valid` | float \| null | fraction of valid annulus samples backing `depth_m` (`geometry.AnnulusSample`) |
| `p_optical_m` | [float,float,float] \| null | 3D point in `camera_color_optical_frame` |
| `p_base_m` | [float,float,float] \| null | 3D point in `base_link` |
| `n_base` | [float,float,float] \| null | outward surface normal, unit vector, in `base_link` |
| `normal_rms_m` | float \| null | RMS plane-fit residual (`geometry.PlaneFit`) |
| `sigma_normal_rad` | float \| null | normal-direction uncertainty, §10.4 |
| `cov_base_m2` | [9 floats] \| null | row-major 3×3 `Σ_base`, m² |
| `sigma_base_m` | [float,float,float] \| null | `sqrt(diag(Σ_base))` per axis, m |

**Invalid points carry null geometry fields** (`depth_m`, `p_optical_m`, `p_base_m`, `n_base`,
`normal_rms_m`, `sigma_normal_rad`, `cov_base_m2`, `sigma_base_m` all `null`) — never a point placed
at the camera origin or any other sentinel coordinate.

Each `segments[]` waypoint:

| Field | Type | Meaning |
|---|---|---|
| `phase` | str | `approach` \| `trace` \| `retract` |
| `position_m` | [float,float,float] | `tool_tip` position in `base_link` |
| `quat_xyzw` | [4 floats] | `tool_tip` orientation in `base_link` |
| `surface_point_m` | [float,float,float] | the crack-surface point this waypoint stands off from |
| `normal_base` | [float,float,float] | outward normal at `surface_point_m`, unit vector, `base_link` |
| `clearance_m` | float | standoff distance actually used (`trace_clearance_m` or `approach_retract_offset_m`) |
| `sigma_along_normal_m` | float | §10.4 |
| `sigma_max_m` | float | `max(sigma_base_m)` of the nearest source point |
| `interpolated` | bool | waypoint's surface point came from an interpolated point |
| `within_budget` | bool | `3·sigma_along_normal_m ≤ clearance_m` |

### 10.3 Lifting policy

1. **Surface depth**, per dense pixel: `geometry.sample_surface_depth_annulus` on the crack mask
   (R-08 — cracks are cavities, so the reliable depth is the surrounding intact surface, not the
   crack pixel itself). Defaults: inner radius **3 px**, outer radius **8 px**, `min_fraction_valid`
   **0.3**; valid band from `geometry.DEFAULT_VALID_DEPTH_RANGE_M` (0.07–0.50 m). Below
   `min_fraction_valid` → `reason: annulus_insufficient`.
2. **Surface point**: the crack pixel `(u, v)` deprojected at that surface depth
   (`geometry.deproject_pixels`) — the pixel's ray, not the annulus samples' rays.
3. **Normal**: a plane fit (`geometry.fit_plane`) over the deprojected, valid, non-mask annulus
   pixels; fewer than 6 such points → `reason: normal_fit_failed`. The **outward** normal is chosen
   explicitly by `n · (−p_optical) > 0`.
   > **Pitfall:** `geometry.fit_plane` canonicalises its returned normal to optical `n_z ≥ 0`, which
   > points *away from* the camera despite what its own comment says. Callers must never rely on
   > `fit_plane`'s sign convention directly — always re-orient with the `n · (−p_optical) > 0` test
   > above before use.
4. **Gaps**: a run of at most `max_gap_px` (default **5**) invalid points strictly between two valid
   points is filled by linear interpolation in `base_link` (`p_base_m` lerped by pixel-run position;
   `n_base` filled by a normalised lerp of the neighbouring normals). Interpolated points get
   `interpolated: true` and inherit `cov_base_m2`/`sigma_base_m` from whichever of the two bounding
   neighbours has the larger trace (the more conservative of the two). A run longer than
   `max_gap_px` is **not** bridged — it splits the polyline into separate `segments[]`. Leading and
   trailing invalid runs (no valid point on one side) are trimmed, never extrapolated. A segment
   left with fewer than 3 valid points after trimming/splitting is dropped entirely, and the drop is
   reflected in `counts.invalid_by_reason` (its points keep their original `reason`, or
   `segment_too_short` if they had none).

### 10.4 Uncertainty (first order; GEOM-09 owns the full budget)

- `σ_px` default **1.0 px** (skeleton-centreline prior). `σ_z = geometry.depth_uncertainty_m(z, fx)`.
- `Σ_opt = J · diag(σ_px², σ_px², σ_z²) · Jᵀ`, with the pinhole-linearisation Jacobian
  `J = [[z/fx, 0, x/z], [0, z/fy, y/z], [0, 0, 1]]` (distortion Jacobian neglected — listed under
  `unmodelled`).
- `Σ_base = R · Σ_opt · Rᵀ + (σ_cam_pos² + (‖p_opt‖ · σ_cam_rot)²) · I₃`, where `R` is the rotation of
  `T_base_link_camera_color_optical_frame` (the §9.2 chain composed through `FK(q)`), and
  `σ_cam_pos`/`σ_cam_rot` are `calibration.wrist_camera.position_sigma_m`/`rotation_sigma_rad`.
- `σ_normal_rad = normal_rms_m / (outer_radius_px · z / fx)`.
- Per waypoint: `σ_along_normal = sqrt(nᵀ · Σ_base · n + σ_tool_pos²)`, where `n` is `normal_base` and
  `σ_tool_pos` is `calibration.tool.position_sigma_m`; `within_budget ⇔ 3 · σ_along_normal ≤ clearance_m`.
- **Calibration sigmas:** for a `measured` block, read `position_sigma_m`/`rotation_sigma_rad` (or,
  for `tool`, just `position_sigma_m`) from that block's own `uncertainty: {position_sigma_m,
  rotation_sigma_rad, source}` object in `end_effector.yaml` — sourced from the `docs/calibration/PLAN.md`
  §4 layer-3 held-out residual standard deviations, never a placeholder. **A `measured` block that
  lacks `uncertainty` is refused** (exit 3), because a measured value without a residual estimate is
  indistinguishable from an unverified guess. **GEOM-05 and GEOM-07 must write this `uncertainty`
  object** on the `wrist_camera` and `tool` blocks respectively when they promote them to
  `value_status: measured` — this is a requirement on those cards, not optional metadata.
  Nominal priors (used while `value_status: nominal`): `wrist_camera` **0.010 m / 5°** (ADR-014 §5's
  own prior-disagreement flag thresholds — anything a real calibration would flag as wrong is exactly
  the uncertainty the nominal prior should already carry); `tool` **0.005 m** (`end_effector.yaml`'s
  own documented "a few mm" seating-error caveat on the CAD prior).
- `uncertainty_model.unmodelled` (always all five, verbatim): `joint_encoder_and_fk`, `depth_bias`,
  `distortion_jacobian`, `capture_time_skew_motion`, `thermal_drift`.

### 10.5 Waypoint policy (ADR-014 §2)

1. Resample each segment by 3D arc length at `waypoint_spacing_m` (default **0.002 m**), both
   segment endpoints included. Normals at resampled positions are the normalised lerp of the
   bracketing source points' `n_base`.
2. `tool_tip`'s `+X` axis is set to `−n_out` (the boresight points into the surface, opposite the
   outward normal). `position_m = surface_point_m + clearance_m · n_out`, with `clearance_m =
   trace_clearance_m` (default **0.01 m**) for `trace` waypoints.
3. **Roll is free** (`parameters.roll_free: true` — MOT-06 may re-roll at execution time). The
   deterministic default fill here is parallel transport of a reference `+Z`: the first waypoint's
   `+Z` is the capture-pose `tool_tip` `Z` axis (`FK(q) · T_gripper_link_tool_tip`, `q` from the §9
   capture record) projected onto the plane ⟂ the waypoint's `+X` and renormalised; if that
   projection's norm is `< 1e-6` (capture pose nearly parallel to this waypoint's boresight), fall
   back to projecting `base_link` `+Z`, and if that also degenerates, `base_link` `+X`. Each
   subsequent waypoint's `+Z` is the previous waypoint's (already-transported) `+Z` projected onto
   its own `+X`-orthogonal plane and renormalised — i.e. propagated along the path, not recomputed
   from the capture pose each time. `+Y = +Z × +X`, right-handed, gives `quat_xyzw`.
4. One `approach` waypoint immediately before and one `retract` waypoint immediately after each
   segment's resampled `trace` waypoints, each at `approach_retract_offset_m` (default **0.04 m**)
   along that end's `n_out` from that end's surface point, with that end's orientation.

### 10.6 Execution eligibility

`execution_eligible = false`, with every applicable reason listed in `ineligible_reasons`, when any
of the following hold:

| Reason | Condition |
|---|---|
| `wrist_camera_nominal` | `calibration.wrist_camera.value_status != "measured"` |
| `tool_nominal` | `calibration.tool.value_status != "measured"` |
| `optical_frames_nominal` | the §9 capture's `camera_optical.source == "nominal_d405"` |
| `synthetic_capture` | the §9 capture's `synthetic == true` |
| `end_effector_changed_since_capture` | `sources.end_effector.sha256_at_capture != sha256_now` |
| `uncertainty_exceeds_clearance` | any waypoint has `within_budget == false` |
| `no_valid_points` | `counts.valid == 0` |

The file is **always** written, including when `execution_eligible` is `false` — it remains valid
input for mock/sim planning and visualization. `MOT-05` must refuse real execution of any path whose
file reports `execution_eligible: false`.

---

## 11. Commissioning-gated execution (ADR-016)

**Normative source:** `docs/adr/016-commissioning-gated-execution.md`. MOT-05.2–.6 implement this section
verbatim. Nothing here edits §0–§10. Exception to §0.1: `execute_trajectory` is an `rclpy` node and is
invoked as a ROS entry point (§11.10), not through `./env.sh python`.

### 11.1 Modes, driver profiles and `config/motion/execution.yaml` (schema `crackvision.execution_config/1`)

Modes: `mock` (goals only ever reach a mock driver), `dry` (every offline gate plus every online read-only
gate; never constructs a `follow_joint_trajectory` `ActionClient`; usable against the live vendor driver as
a motion-free rehearsal; also reports what `real` would additionally refuse, §11.7) and `real`.
`execute_trajectory --mode` defaults to `dry`. `--dry-run` (§0.3) is a separate, stronger thing: offline
gates only, no ROS node, no file writes, exit 0 regardless of gate outcome. It is not the same as
`--mode dry` (§11.10).

Driver profiles:

| Profile | Action | `joint_states` topic | Expected action-server node | Valid `--mode` values |
|---|---|---|---|---|
| `moveit_mock` | `/rebotarm_controller/follow_joint_trajectory` | `/joint_states` | ros2_control mock hardware (MOT-02) | `mock`, `dry` |
| `vendor_mock` | `/rebotarm/follow_joint_trajectory` | `/rebotarm/joint_states` | `mock_rebotarm_driver` | `mock`, `dry` |
| `vendor` | `/rebotarm/follow_joint_trajectory` | `/rebotarm/joint_states` | `reBotArmController` | `dry`, `real` |

A `--mode`/`--profile` pair not in this table is a usage error (exit 2), raised before any gate runs.

`config/motion/execution.yaml`:

| Field | Meaning |
|---|---|
| `schema` | `"crackvision.execution_config/1"` |
| `default_mode` | `mock \| dry \| real`, CLI default when `--mode` is omitted (ships `dry`) |
| `driver_profiles.<name>` | `{action, joint_states_topic, expected_node, modes}` for each row above; `vendor` also has `arm_status_topic` (`/rebotarm/arm_status`) |
| `default_driver_profile` | one of the three profile names |
| `speed_scale.default` | `speed_scale` for `mock` when `--speed-scale` is omitted (§11.12) |
| `speed_scale.cap` | hard cap for `mock` and `dry` (§11.12) |
| `speed_scale.real_default` | `speed_scale` for `dry` and `real` when `--speed-scale` is omitted; ships `0.10`. It never raises the real cap: `real`'s cap is `commissioning.yaml.speed_scale_cap` (§11.4), never this file |
| `tolerances.start_state_rad`, `tolerances.tracking_rad` | §11.12 |
| `timeouts.joint_state_s`, `timeouts.goal_s`, `timeouts.cancel_settle_s` | §11.12; `goal_s` bounds how long a single `follow_joint_trajectory` goal may run before the executor treats it as stuck and cancels; `cancel_settle_s` is the §11.8 post-cancel stop window |
| `densify_step_m` | §11.12 |
| `position_margin_rad` | §11.6 below |
| `approval_max_age_s` | §11.5/§11.12 |
| `estop_topics` | list, default `["/crackvision/estop", "/rebot_motion/estop"]` (§11.8) |

These are the only `speed_scale` key names. `execution.yaml` is loaded strictly: an unknown key is a
config error (exit 2), and so is a `speed_scale` value outside `(0, 1]`.

### 11.2 `crackvision.joint_trajectory/1` (file, produced by MOT-07)

| Field | Type | Meaning |
|---|---|---|
| `schema` | str | `"crackvision.joint_trajectory/1"` |
| `created_utc` | str | ISO-8601 UTC |
| `producer` | str | tool that wrote the file |
| `purpose` | str | `"crack_task"` \| `"test"` — `real` mode refuses `"test"` unconditionally |
| `planning_frame` | str | `"base_link"` |
| `joint_names` | list[str] | exactly `["joint1", ..., "joint6"]`, this order |
| `points[]` | list | `{t_s, positions[6], velocities[6]?, accelerations[6]?}`; `t_s` strictly increasing, first point `t_s == 0`. The file holds the **unscaled** trajectory (scale 1.0); it must be within the limits as written (§11.7 `G-LIMITS`) |
| `limits_file.path`, `limits_file.sha256` | str, str\|null | repo-relative path and sha256 of the limits file this trajectory was planned/retimed against; `null` allowed only outside `real` (ADR-016 Decision §5) |
| `end_effector_config_sha256` | str\|null | sha256 of `config/robot/end_effector.yaml` at plan time; `null` allowed only outside `real` (`G-STALE-CONFIG`) |
| `scene_config_sha256` | str\|null | sha256 of `config/scene/scene.yaml` at plan time; `null` allowed only outside `real` (`G-STALE-CONFIG`) |
| `source.paths3d.path`, `.sha256`, `.execution_eligible` | str, str, bool \| null (whole `source.paths3d` object is `null` for a trajectory not derived from a §10 paths3d file) | provenance for `G-ELIGIBLE` (§11.7). `execution_eligible` is a copy, kept for the reader. The gate never trusts it and reads the file instead. `real` refuses a `crack_task` with `source.paths3d: null` |

### 11.3 (reserved — numbering continues at 11.4 to match the card's gate-id ordering)

### 11.4 Commissioning record `config/robot/commissioning.yaml` (schema `crackvision.commissioning/1`)

| Field | Type | Meaning |
|---|---|---|
| `schema` | str | `"crackvision.commissioning/1"` |
| `commissioned` | bool | overall gate; ships `false` |
| `limits_file_sha256` | str\|null | must equal the current `config/robot/b601_dm_limits.yaml` sha256 for `G-COMMISSIONING` to pass |
| `speed_scale_cap` | float | real-mode hard cap (ADR-016 §4), in `(0, 1]`; ships `0.10`. Raised only by an operator with evidence (MOT-09) |
| `estop.kind` | str | fixed `"hardware"` — the record attests the physical e-stop circuit, never a ROS topic |
| `estop.verified` | bool | ships `false` |
| `estop.verified_utc`, `estop.operator` | str\|null | when/who verified it |
| `estop.evidence` | list[str] | free-text/paths to logs demonstrating the test |
| `joint_ranges_verified` | bool | ships `false` |
| `evidence[]` | list[str] | general supporting evidence (logs, reports) |
| `source` | str | free-text provenance |

Ships uncommissioned (`commissioned: false`, both `verified` flags `false`). MOT-09 fills it in.

### 11.5 Execution approval `crackvision.execution_approval/1` (produced by MOT-08)

| Field | Type | Meaning |
|---|---|---|
| `schema` | str | `"crackvision.execution_approval/1"` |
| `trajectory_sha256` | str | sha256 of the exact trajectory file being approved |
| `approved_by` | str | operator identifier |
| `approved_utc` | str | ISO-8601 UTC |
| `preview_artifacts[]` | list[{path, sha256}] | the rendered preview(s) the operator reviewed |

`G-APPROVAL` fails unless an approval file names the current trajectory's sha256 and
`now - approved_utc <= execution.yaml.approval_max_age_s`. It refuses only in `real` (§11.7).

### 11.6 Position-margin policy

The hard pass/fail bound for every point's position, in every mode, is `[lower, upper]` inclusive from
`config/robot/b601_dm_limits.yaml`, and it is never narrowed. This is required because the SRDF's all-zero
`home` sits with `joint2`/`joint3` exactly on `upper = 0.0`; narrowing the bound would fail the vendor's own
canonical rest pose.

`execution.yaml.position_margin_rad` (`m` below) does not change this bound. Per joint and point, with
position `q`:

- `near_upper` ⇔ `upper − q ≤ m`; `near_lower` ⇔ `q − lower ≤ m`. Both flags are reported in the
  `G-LIMITS` detail (informational).
- **Toward-bound speed** `w`: `w = +v` with respect to the upper bound and `w = −v` with respect to the
  lower bound.
- **Margin violation** (`kind: position_margin`): a near-limit point `i` that is not the last point, with
  `w_i > 0` (moving toward the bound it is near) and `w_{i+1} > w_i` (speeding up toward it). The last
  point is a violation only if it carries an explicit velocity with `w > 0`; without one it ends at rest.
  Moving away (`w ≤ 0`) always passes. Arriving at an inclusive bound at constant or falling speed passes.
- The rule is evaluated twice: once on the explicit `velocities` when present, and always on forward finite
  differences, where `v_i = (q_{i+1} − q_i) / (t_{i+1} − t_i)` is the per-segment velocity the vendor
  driver's linear interpolation commands. For the finite-difference pass, the last point has no forward
  segment and is never a violation.
- Uniform retiming (§11.7 `G-SPEED`) multiplies every `w` by the same `scale > 0`, so the outcome is the
  same scaled or unscaled. It is evaluated on the unscaled file.

**Home-approach example** (`m = 0.03`). `joint2` points `…, −0.04, −0.02, −0.01, 0.00` at `t_s` spacing that
gives 0.10 rad/s on every segment. −0.02 and −0.01 are `near_upper` with `w = 0.10`. The following segment
speed is 0.10, which is not greater, so they pass. 0.00 is the last point, on the inclusive bound, and
passes with explicit velocity 0 or none. **The trajectory passes**, and so does any TOTG-style approach
whose speed falls to 0 at home. Passing through the bound also passes: in `−0.20, −0.01, 0.00, −0.01, −0.20` (equal Δt), the first
−0.01 has `w = +0.01/Δt` and the next segment's `w = −0.01/Δt` is not larger; from 0.00 on, `w < 0`. Counter-example: `−0.03 → −0.02` at
0.10 rad/s, then `−0.02 → 0.00` at 0.20 rad/s, is a `position_margin` violation at −0.02, though every
position is within bounds. The lower bound is the mirror image (`w = −v`).

### 11.7 Gate catalogue

Categories: **offline** (file/config only, no ROS), **online read-only** (ROS graph/topic/service queries
that never command motion), **confirmation** (human-in-the-loop). **Every gate listed for the active mode
is evaluated and reported, even if an earlier gate already failed.** The executor does not stop at the
first failure. A gate whose precondition gate failed (e.g. `G-LIMITS` when `G-TRAJ` failed) is reported
with outcome `error` (not evaluable) rather than being dropped from the report.

**Outcomes** (`gate_report[].outcome`, §11.9):

| Outcome | Meaning |
|---|---|
| `pass` | evaluated; this mode's rule holds |
| `fail` | evaluated; this mode's rule is violated, so this mode refuses |
| `warn` | evaluated in `mock`/`dry`; this mode's rule holds but the `real` rule would fail. Only reported, never refuses. Never produced in `real` |
| `skip` | not evaluated in this mode (real-only gates in `mock`/`dry`). Never `pass` |
| `error` | could not be evaluated (precondition gate failed, unreadable input). Refuses like `fail` |

**Refusal rule, every mode:** the run refuses (exit 3, record `outcome: refused`, nothing sent to any
driver) iff at least one gate reports `fail` or `error`. `real` therefore proceeds only if every gate in the
table reports `pass`. `mock` and `dry` proceed with `warn` entries present.

**`dry` rehearsal of `real`:** besides its own `gate_report`, `dry` evaluates the offline real-only gates
`G-ARM`, `G-COMMISSIONING` and `G-ESTOP` with `real`'s rules. It writes them to the record's
`real_preview[]`, and their outcomes are never copied into `gate_report`, where they stay `skip`. It also
sets `would_refuse_in_real` to the ids of every `warn` gate plus every failing `real_preview` gate.
`G-CONFIRM` is interactive and is never previewed. `dry` exits **0** when no gate reports `fail`/`error`,
however long `would_refuse_in_real` is. `mock` reports `warn` the same way, but has no `real_preview`.

In the table, **refuse** = a violation is `fail`; **warn** = a violation is `warn`; `skip` = not evaluated.
"Integrity" violations refuse in every mode. "Authorization/commissioning" violations refuse only in
`real`.

| Gate | Category | mock | dry | real | Checks |
|---|---|---|---|---|---|
| `G-ARM` | confirmation | skip | skip (previewed) | refuse | env `CRACKVISION_ARM_REAL=1` set (the literal variable, read from the process environment) |
| `G-TRAJ` | offline | refuse | refuse | refuse | file matches the `crackvision.joint_trajectory/1` schema (§11.2). `purpose: "test"`: **warn** in mock/dry, **refuse** in real |
| `G-LIMITS-HASH` | offline | refuse | refuse | refuse | non-null `limits_file.sha256` ≠ current `b601_dm_limits.yaml` sha256: refuse in every mode. `null`: **warn** in mock/dry, **refuse** in real |
| `G-LIMITS` | offline | refuse | refuse | refuse | **(1) unscaled**: the file's trajectory at scale 1.0 is within `b601_dm_limits.yaml` positions (inclusive), velocities and accelerations, plus the §11.6 margin rule. **(2) scaled**: after retiming at the effective `speed_scale` `s`, every velocity is `≤ s · velocity_limit` and every acceleration `≤ s² · acceleration_limit`. Both are checked from explicit `velocities`/`accelerations` when present and always from finite differences of `positions` over `t_s`. (2) follows from (1) and is a cross-check on retiming. A file over the limits at scale 1.0 is refused whatever `s` is |
| `G-SPEED` | offline | refuse | refuse | refuse | effective `s` (`--speed-scale`, else the mode default: `speed_scale.default` for mock, `speed_scale.real_default` for dry/real) must be in `(0, cap]`. mock/dry cap = `execution.yaml.speed_scale.cap`; real cap = `commissioning.yaml.speed_scale_cap`. **Refuse, never clamp.** In dry, `s > commissioning.yaml.speed_scale_cap` (or an unreadable record) is **warn** |
| `G-SCENE` | offline | refuse | refuse | refuse | `scene_core.load_config` succeeds (refuse in every mode). `scene_core.assert_commissioning_ready` raises: **warn** in mock/dry, **refuse** in real, with the exception message |
| `G-EE` | offline | refuse | refuse | refuse | `end_effector.load_config` succeeds (refuse in every mode). `end_effector.assert_commissioning_ready` raises: **warn** in mock/dry, **refuse** in real, with the exception message |
| `G-COMMISSIONING` | offline | skip | skip (previewed) | refuse | `commissioning.yaml` loads, `commissioned == true`, and `limits_file_sha256` matches the current `b601_dm_limits.yaml` |
| `G-ESTOP` | offline | skip | skip (previewed) | refuse | `commissioning.yaml.estop.kind == "hardware"` and `estop.verified == true` |
| `G-ELIGIBLE` | offline | refuse | refuse | refuse | **Integrity (refuse in every mode):** when `source.paths3d` is non-null, the file at `source.paths3d.path` (resolved against `--root`) exists, its sha256 equals `source.paths3d.sha256`, and it parses as `crackvision.paths3d/1`. **Eligibility (only after the sha check passes; read from the file, never from the trajectory's copy):** the file's `execution_eligible` must be `true`, and its `ineligible_reasons` go into the detail. A trajectory copy of `execution_eligible` that differs from the file also counts as an eligibility failure. `purpose: "crack_task"` with `source.paths3d: null` is also an eligibility failure. Eligibility failures: **warn** in mock/dry, **refuse** in real |
| `G-APPROVAL` | offline | warn | warn | refuse | §11.5: an approval names the trajectory sha256 and is within `approval_max_age_s`. Missing, mismatched or expired approval: **warn** in mock/dry, **refuse** in real |
| `G-STALE-CONFIG` | offline | refuse | refuse | refuse | non-null `end_effector_config_sha256`/`scene_config_sha256` ≠ the current `config/robot/end_effector.yaml`/`config/scene/scene.yaml` sha256: **refuse in every mode** (replan). `null` either one: **warn** in mock/dry, **refuse** in real |
| `G-GRAPH` | online read-only | refuse | refuse | refuse | driver-profile node-identity discrimination (§11.1). `moveit_mock`/`vendor_mock` refuse if `reBotArmController` is visible. `vendor` refuses unless `reBotArmController` hosts the action server and no `mock_rebotarm_driver` is visible. `vendor` also refuses if any publisher exists on `/rebotarm/joints/*/cmd/*` or `/rebotarm/gripper/cmd/*` |
| `G-START-STATE` | online read-only | refuse | refuse | refuse | live `joint_states` on the profile topic are fresher than `timeouts.joint_state_s`, and the current position is within `tolerances.start_state_rad` of the first point (refuse in every mode). `vendor` profile only: the latched `/rebotarm/arm_status` must be received, with `enabled == true`, `state_machine == "IDLE"` and empty `error_codes`. A violation is **warn** in dry and **refuse** in real. The executor never calls `enable` |
| `G-COLLISION` | online read-only | refuse | refuse | refuse | densified waypoints (§11.12) valid via the MOT-02 mock stack's `/check_state_validity`, production scene applied, explicit `RobotState` per query (§11.11) |
| `G-CONFIRM` | confirmation | skip | skip | refuse | typed phrase (§11.8); asked only after every other gate passed |

### 11.8 Confirmation, monitoring and e-stop

Confirmation phrase: `EXECUTE <first 8 hex chars of the trajectory's sha256>`, typed on a controlling
`/dev/tty` (opened directly, never `stdin`), exact match after `strip()`. The e-stop topic/location and the
effective `speed_scale` are printed immediately before the prompt. No flag, env var or non-tty input
satisfies `G-CONFIRM`; no controlling tty is a refusal (exit 3), not a fallback.

E-stop triggers, each treated identically: a `True` message on any topic in `execution.yaml.estop_topics`
(default `/crackvision/estop`, `/rebot_motion/estop`, `std_msgs/Bool`), `SIGINT`/`SIGTERM`, `joint_states`
staleness beyond `timeouts.joint_state_s`, tracking error beyond `tolerances.tracking_rad`, or a goal running
longer than `timeouts.goal_s`.

**Response: cancel and hold. No trigger disables** (ADR-016 Decision §7):
1. Cancel the active `follow_joint_trajectory` goal. The vendor driver then holds the measured position with
   torque on (`ros_actions.py:175-180` → `hardware_manager.py:237-241`).
2. Watch `joint_states` until the arm has stopped: max per-joint `|Δq|` < `tolerances.start_state_rad`
   across a window of `timeouts.joint_state_s`, reached within `timeouts.cancel_settle_s` of the cancel.
3. If the cancel is not acknowledged, the arm does not settle within `timeouts.cancel_settle_s`, or
   `joint_states` are stale, print **"PRESS THE HARDWARE E-STOP"**. Software cannot do more safely.
4. Write the record (`outcome: estopped`, or `aborted` for tracking/timeout/driver loss) and exit 1.

The executor **never** calls `/{ns}/disable` or any other vendor service, during an e-stop or at teardown.
On exit the arm is left energised and holding. Bringing it to rest is the operator's job, using the vendor
sequence (safe_home or park, *then* disable), or the hardware e-stop. That stays so until MOT-09 confirms
on the physical unit that `disable` brings the arm to a safe rest.

**The physical/hardware e-stop is the safety function this project relies on; `/crackvision/estop` and
`/rebot_motion/estop` are secondary, software-only conveniences** that depend on the ROS graph being alive.
`commissioning.yaml.estop.kind` is fixed to `"hardware"` and `G-ESTOP` checks that the physical circuit
was verified, not either topic.

### 11.9 Execution record `crackvision.execution_record/1`

Written in **every** mode, including on refusal, to `logs/execution/<stamp>_<mode>_<sha8>.json`
(`<sha8>` = first 8 hex chars of the trajectory sha256; directory overridable with `--record-dir`). The
§0.4 `logs/<tool>_*.log`/`.json` artefacts are written in addition, not instead. `--dry-run` writes
neither (§11.10).

| Field | Type | Meaning |
|---|---|---|
| `schema` | str | `"crackvision.execution_record/1"` |
| `mode`, `driver_profile` | str | as invoked |
| `trajectory_sha256` | str | |
| `gate_report[]` | list[{gate, category, outcome, detail}] | one entry per §11.7 row, `outcome` ∈ `pass\|fail\|warn\|skip\|error` |
| `real_preview[]` | list[{gate, category, outcome, detail}] | `dry` only (§11.7): `G-ARM`, `G-COMMISSIONING`, `G-ESTOP` evaluated with `real`'s rules; `[]` in other modes |
| `would_refuse_in_real` | list[str] | gate ids `real` would refuse (`warn` gates plus failing `real_preview` gates); `[]` in `real` |
| `input_shas` | object | every sha256 this run read: trajectory, limits file, end_effector config, scene config, commissioning record, approval file, paths3d file |
| `effective_speed_scale` | float | |
| `outcome` | str | `refused \| rehearsed \| aborted \| estopped \| completed`. `rehearsed` = a `dry` run with no refusal |
| `joint_trace[]` | list | sampled `{t_s, commanded[6], actual[6]}`, sampling rate from `execution.yaml` (not separately specified here — MOT-05.6 picks it; must be dense enough to reconstruct tracking-error decisions) |

### 11.10 `execute_trajectory` CLI

`execute_trajectory` is a console-script entry point of the `crackvision_motion` ROS package (MOT-05.4). It
runs under `/usr/bin/python3` with ROS Humble, never under the conda interpreter that `./env.sh` scrubs ROS
from (ADR-007):

```bash
source scripts/ros/env_ros.sh                  # scrubbed ROS Humble + ~/rebot_ws underlay
set +u; source ros2_ws/install/setup.bash      # crackvision overlay
ros2 run crackvision_motion execute_trajectory --trajectory PATH [--mode mock|dry|real]
    [--profile moveit_mock|vendor_mock|vendor] [--speed-scale S] [--approval PATH]
    [--execution-config PATH] [--commissioning PATH] [--limits PATH] [--scene-config PATH]
    [--end-effector-config PATH] [--service-timeout-s SEC] [--record-dir DIR] [§0.3 flags]
```

`--mode` default `execution.yaml.default_mode` (ships `dry`); `--profile` default
`execution.yaml.default_driver_profile`; `--speed-scale` default per mode (§11.7 `G-SPEED`; `real`/`dry`:
`speed_scale.real_default` = 0.10); config paths default to `config/motion/execution.yaml`,
`config/robot/commissioning.yaml`, `config/robot/b601_dm_limits.yaml`, `config/scene/scene.yaml` and
`config/robot/end_effector.yaml` under `--root`; `--record-dir` default `logs/execution`.

Order: the offline gates run first, before `rclpy.init()`. If any offline gate refuses, no ROS node is
created and nothing is sent. Online gates follow. `dry` stops after them. `mock` sends the goal. `real`
asks `G-CONFIRM` and then sends the goal under §11.8 monitoring.

Exit codes (§0.2), per mode:

| Code | mock | dry | real |
|---|---|---|---|
| `0` | goal completed and arrival verified | no gate `fail`/`error` (`warn`, `real_preview` failures and a non-empty `would_refuse_in_real` allowed) | goal completed and arrival verified |
| `1` | runtime: aborted, e-stopped, tracking violation, goal timeout, action failure | runtime failure while running online checks (node crash, unexpected exception) | as mock |
| `2` | usage/config error, incl. a mode/profile pair outside §11.1 | same | same |
| `3` | any gate `fail`/`error`, or a required service/topic missing | same | same, incl. `G-ARM`/`G-CONFIRM` (no tty) refusals |

`--dry-run` (§0.3) evaluates only the offline gates (`G-TRAJ` through `G-STALE-CONFIG`, with the active
mode's rules), in-process, before `rclpy.init()`. It constructs no node, contacts no ROS graph, writes no
record or log file, and exits 0 regardless of gate outcome, printing the report to stdout. It still uses
the ROS entry point above to resolve the console script. The offline gate itself lives in the ROS-free
module `crackvision_motion.execution_gate` (MOT-05.3), which runs under system `python3` without a ROS graph
or colcon build. `--mode dry` is different: it runs the full online gate set, requires ROS, and always
writes the §11.9 record.

### 11.11 Real-mode collision-validity topology

`G-COLLISION` in every mode runs the MOT-02 headless mock-planning stack
(`crackvision_motion/launch/mock_planning.launch.py`) purely as a validity **oracle**: production
`config/scene/scene.yaml` is applied to it, and every densified waypoint is checked via its
`/check_state_validity` with an **explicit `RobotState`** per query. In `real` mode the oracle's own
controller (`/rebotarm_controller/follow_joint_trajectory`) is never addressed — `G-GRAPH` only ever
resolves the `vendor` profile's `/rebotarm/follow_joint_trajectory`, and no `ActionClient` for the oracle's
controller name is ever constructed. The oracle's own monitored/implicit planning-scene state being its
mock robot's state does not matter, because every validity query supplies the real driver's live joint
state explicitly; the oracle contributes only its collision/self-collision geometry evaluation.

### 11.12 Default numeric values

| Parameter | Default | Rationale |
|---|---|---|
| `speed_scale.real_default` | 0.10 | Real and dry runs default to the shipped commissioning cap: a real run at the default moves at ≤ 10% of the velocity limits and ≤ 1% of the acceleration limits, which `G-LIMITS` (1)+(2) check. Raising it beyond `commissioning.yaml.speed_scale_cap` is refused, not clamped. |
| `speed_scale.default` | 1.0 | Mock only. The unscaled file is already within the limits (`G-LIMITS` (1)), and no torque is at stake. Full speed keeps mock smoke runs short. |
| `speed_scale.cap` | 1.0 | Mock/dry cap. Uniform retiming never speeds a trajectory up, so 1.0 is the natural upper bound. Dry also reports `warn` above the real cap. |
| `tolerances.start_state_rad` | 0.02 rad | Must be tight: this checks the robot is *already* at the trajectory's first point before any goal is sent, not that a move *completed* — a fraction of the vendor stack's own 0.06 rad arrival tolerance (`motion_runner.py` `tol_rad`), which is itself an arrival (not a starting) tolerance. Also the post-cancel "stopped" threshold (§11.8). |
| `tolerances.tracking_rad` | 0.15 rad | Must sit strictly between the vendor's own arrival tolerance (0.06 rad) and its hard-fail tolerance (0.20 rad, `motion_runner.py` `fail_tol_rad`) so this executor's independent tracking cancel fires before any vendor-side hard failure, while staying loose enough that ordinary planned-motion lag under `speed_scale` retiming doesn't spuriously trip it. |
| `timeouts.joint_state_s` | 0.5 s | `joint_states` publishes at 50 Hz (mock driver) to 100 Hz (`reBotArmController`'s `joint_state_rate` default); 0.5 s is 25–50 missed publishes — generous against network/scheduler jitter, far faster than human reaction time for a dead driver. |
| `timeouts.cancel_settle_s` | 2.0 s | At the 0.10 real cap the arm moves slowly, and the driver's hold (`ros_actions.py:175-176`) acts within one 20 ms control tick (`ros_actions.py:184`). 2 s leaves a wide margin for that and is still short enough that the operator is told to press the hardware e-stop promptly if the hold does not take. |
| `timeouts.goal_s` | scaled trajectory duration + 5 s | A goal still running 5 s after its last scaled `t_s` is stuck. Cancel and hold (§11.8). |
| `densify_step_m` | 0.01 m | Max tool-tip Cartesian displacement between consecutive `G-COLLISION`-checked configurations before an extra linearly-interpolated joint configuration is inserted; chosen below both the trace clearance (0.01 m, ADR-014 §2) and the end-effector collision padding (0.005 m, `end_effector.yaml`), so any swept collision this could still miss is already smaller than margins the system carries elsewhere. |
| `position_margin_rad` | 0.03 rad (≈1.7°) | Near-limit threshold for the §11.6 approach rule, not a bound narrowing. It equals the vendor's own `safe_park_tolerance_rad` default (`rebotarm_controller.py:39`), small enough not to flag ordinary mid-range motion, and wide enough to cover the last segments of an approach to home. |
| `approval_max_age_s` | 3600 s (1 h) | Long enough for an operator to review a MOT-08 preview and walk to the workstation without re-approving; short enough that a stale approval can't silently authorize a run on a physically-rearranged cell later the same day (sha-bindings catch *file* changes; this bounds the *time* window for unrecorded physical changes). |

---

## 12. Workcell survey record (`crackvision.workcell_survey/1`)

**Normative source:** `docs/motion/WORKCELL_SURVEY.md` (MOT-10.1). That document is the operator-facing
procedure (tools, checklist, re-measure triggers); this section is the normative schema for the file the
procedure produces — the **raw** instrument readings, never operator-computed `base_link` coordinates.
Nothing here edits §0–§11. MOT-10.3 (`survey_to_scene`, `docs/motion/SCENE.md` §8) is the only consumer that
turns a validated survey record into `config/scene/scene.yaml` entries (§3) with `value_status: measured`.

### 12.1 Datum (fixed, not a survey field)

Every offset in a survey record is along the same `base_link` reference faces MOT-10.1 establishes from
the canonical URDF's `base_link.STL` bounding box (`~/rebot_ws/install/rebotarm_bringup/share/rebotarm_bringup/description/meshes_b601_gripper/base_link.STL`,
mesh origin identity in the URDF, so STL vertex coordinates equal `base_link` frame coordinates directly):

| Reference face | `base_link` coordinate | STL bbox axis |
|---|---|---|
| bottom face (table contact) | `z = 0` | `z ∈ [0.000, 0.08265]` m |
| front face (arm reach / `+x`) | `x = +0.070` m | `x ∈ [-0.070, 0.070]` m |
| side faces (`±y`) | `y = ±0.100` m | `y ∈ [-0.100, 0.100]` m |

These three numbers are fixed by the robot model, not re-measured per survey; a survey record does not
repeat them, it only states offsets *from* them.

### 12.2 Top-level keys (exact set; unknown keys are a config error)

| Key | Type | Meaning |
|---|---|---|
| `schema` | str | `"crackvision.workcell_survey/1"` |
| `survey` | map | `{operator, date, photos[], notes}` — §12.3 |
| `derivation_params` | map | `{rectangularity_tolerance_m}` — §12.6 |
| `base_mounting` | map | §12.4(a) |
| `table` | map | §12.4(b) |
| `specimen` | map | §12.4(c) |
| `obstacles` | map | §12.4(d) |
| `acm_observations` | map | §12.4(e) |

A **reading** (used throughout) is always the 4-key map `{value_m, instrument, resolution_m,
uncertainty_1sigma_m}` — `value_m` a float (metres or radians per field name), `instrument` and
`resolution_m` naming the tool and its smallest graduation, `uncertainty_1sigma_m` the operator's stated
1-sigma uncertainty for that specific reading. A reading with any of the four `null` is incomplete and
blocks every derived value that depends on it (§12.7).

### 12.3 `survey` (file-level provenance)

| Field | Type | Meaning |
|---|---|---|
| `operator` | str | who took the measurements |
| `date` | str | ISO-8601 date |
| `photos` | list[str] | repo-relative paths under `data/workcell_survey/` (git-ignored, §12 never commits these) |
| `notes` | str | free text |

### 12.4 Measurement blocks (raw readings only)

**(a) `base_mounting`**

| Field | Type | Meaning |
|---|---|---|
| `bolted_directly_to_table` | bool | `true` if the base casting/plate sits directly on the table with no adapter |
| `adapter_plate_thickness` | reading\|null | required iff `bolted_directly_to_table` is `false`; else `null` |

**(b) `table`** — all distances measured from the §12.1 reference faces, along the base's own axes.
All four edge readings are positive distances measured *from* the named reference face, in the
direction stated; §12.7 rule 3 gives the exact sign each one gets when mapped to `base_link`.

| Field | Type | Meaning |
|---|---|---|
| `front_face_to_far_edge` | reading | distance from the front reference face (`x = +0.070`), in the `+x` direction, to the table's far edge (the specimen side) |
| `front_face_to_near_edge` | reading | distance from the *same* front reference face (`x = +0.070`), but in the `-x` direction, to the table's near edge (the edge behind the robot). A value smaller than the base's own depth (`0.140` m) means the near edge sits under/behind the base; this is normal and still a positive reading — there is no separate "`-x` face", the measurement is always taken from the one front reference face |
| `pos_y_side_face_to_edge` | reading | distance from the `+y` side face (`y = +0.100`), in the `+y` direction, to the table's edge on that side |
| `neg_y_side_face_to_edge` | reading | distance from the `-y` side face (`y = -0.100`), in the `-y` direction, to the table's edge on that side |
| `thickness` | reading | table top thickness (top face to underside), used for the table collision box's `z` extent (§12.7 rule 3) |
| `flatness_deviation` | reading | spirit-level bubble deviation / straightedge gap near the specimen location |
| `flatness_note` | str | where on the table this was checked |

**(c) `specimen`**

| Field | Type | Meaning |
|---|---|---|
| `corners` | list[4 maps] | each `{x_from_front_face, y_from_centerline}` (both readings), one per physical top corner, in a consistent winding order stated in `survey.notes`. `x_from_front_face` is positive in the `+x` direction from the front reference face (`x = +0.070`); `y_from_centerline` is positive in the `+y` direction from the `y = 0` centerline (the same direction as `pos_y_side_face_to_edge`) |
| `thickness_readings` | list[≥3 readings] | caliper thickness at ≥3 distinct points on the specimen |
| `resting_on_table` | bool | observed, not derived |

**(d) `obstacles`**

| Field | Type | Meaning |
|---|---|---|
| `radius_m` | float | the stated clearance radius around `base_link`'s origin that was surveyed |
| `items` | list[map] | each `{id, description, x_from_front_face: {min, max} readings, y_from_centerline: {min, max} readings, z_from_table_top: {min, max} readings}` — one axis-aligned box per obstacle (wall, fixture, cable run, camera USB lead routing, etc.) inside `radius_m`. `x_from_front_face`/`y_from_centerline` use the same sign conventions as `specimen.corners` (§12.4(c)). `z_from_table_top` is positive going *up* from the table's top face (the `z = -(adapter_plate_thickness or 0)` datum of §12.7 rule 3), so a reading of `0` sits exactly on the table top and a fixture above the table has a positive `min`/`max` |

**(e) `acm_observations`**

| Field | Type | Meaning |
|---|---|---|
| `base_bolted_to_table` | bool | mirrors `base_mounting.bolted_directly_to_table`, stated here as the explicit ACM-facing observation |
| `specimen_resting_on_table` | bool | mirrors `specimen.resting_on_table` |
| `notes` | str | free text |

### 12.5 Explicitly out of scope (not in this schema)

Camera and mount poses are **not** tape-measured here — they come from hand-eye calibration
(GEOM-05, ADR-013/014). The eye-in-hand wrist mount is robot geometry (GEOM-10,
`config/robot/end_effector.yaml`). This procedure is `motion: false`: the arm stays powered off or at
rest throughout and nothing is jogged.

### 12.6 `derivation_params`

| Field | Type | Meaning |
|---|---|---|
| `rectangularity_tolerance_m` | float | max allowed deviation (any corner, after fitting the best rectangle) before the specimen corners are refused as non-rectangular (§12.7); normative default `0.003` (3 mm) |

### 12.7 Derived-value rules (normative, deterministic — implemented by MOT-10.3's `survey_to_scene`)

First, every raw reading is mapped to `base_link` coordinates (§12.1 reference faces; §12.4 states each
field's sign convention):

- `x_from_front_face` (specimen corners, obstacle x): `x = 0.070 + value_m`.
- `y_from_centerline` (specimen corners, obstacle y): `y = value_m` (the centerline *is* `base_link`
  `y = 0`).
- `table.front_face_to_far_edge`: `x_far = 0.070 + value_m`.
- `table.front_face_to_near_edge`: `x_near = 0.070 - value_m` (same front face, `-x` direction — see
  §12.4(b)).
- `table.pos_y_side_face_to_edge`: `y_pos = 0.100 + value_m`.
- `table.neg_y_side_face_to_edge`: `y_neg = -0.100 - value_m`.
- `obstacles.items[i].z_from_table_top`: `z = z_table_top + value_m`, where `z_table_top` is rule 3's
  table-top datum below (positive `value_m` is above the table top).

1. **Specimen centre/yaw/footprint**: let `P0, P1, P2, P3` be the 4 `corners`, converted to `base_link`
   `(x, y)` above, in the winding order `survey.notes` states (consecutive, e.g. clockwise). Define:
   - side lengths `s0 = |P1-P0|`, `s1 = |P2-P1|`, `s2 = |P3-P2|`, `s3 = |P0-P3|`;
   - diagonal lengths `d0 = |P2-P0|`, `d1 = |P3-P1|`;
   - `residual_m = max(|d0 - d1|, |s0 - s2|, |s1 - s3|)` — this is `0` only for an exact rectangle (equal
     diagonals, equal opposite sides), unlike a bounding-rectangle fit, which every quadrilateral touches
     exactly and so can never refuse anything.

   **Refuse** if `residual_m > rectangularity_tolerance_m`.

   Otherwise: `centre = mean(P0, P1, P2, P3)` (centroid); let `u_a = (P1-P0)/s0` and `u_c = -(P3-P2)/s2`
   (both should point the same way for a rectangle) — `yaw_rad = atan2(u_a.y + u_c.y, u_a.x + u_c.x)`;
   `footprint_m = [(s0+s2)/2, (s1+s3)/2]` (mean length of each pair of opposite sides, the first along the
   `yaw_rad` axis, the second perpendicular to it).

   **Worked example** (default `rectangularity_tolerance_m = 0.003`): an intended 0.200 m x 0.200 m square
   `P0=(0,0), P1=(0.200,0), P2=(0.200,0.200), P3=(0,0.200)` with `P2` mis-measured 5 mm too far in `+x`,
   i.e. `P2=(0.205,0.200)`. Then `s0=0.200`, `s1=0.200063`, `s2=0.205`, `s3=0.200`, `d0=0.286399`,
   `d1=0.282843`: `|d0-d1|=0.00356` m, `|s0-s2|=0.00500` m, `|s1-s3|=0.00006` m, so
   `residual_m = 0.00500 m = 5 mm > 0.003 m` — **refused**.
2. **Specimen top z**: the specimen rests on the table top (rule 6 refuses otherwise), so
   `z_top = z_table_top + mean(thickness_readings[*].value_m)`, i.e.
   `z_top = -(adapter_plate_thickness.value_m or 0) + mean(thickness_readings[*].value_m)`, with
   `z_table_top` from rule 3 (the adapter plate, if any, lifts the *base* above the table, so the table
   top — and everything resting on it — sits *below* `base_link` `z = 0`). Example: a 0.010 m plate and a
   0.040 m mean thickness give `z_top = -0.010 + 0.040 = 0.030` m; bolted directly, the same specimen
   gives `0.040` m.
3. **Table box**: footprint `x` extent `[x_near, x_far]` and `y` extent `[y_neg, y_pos]` from the mapped
   values above (`y = ±0.100` side faces, not `x = ±0.100`); `dimensions_m = [x_far - x_near, y_pos -
   y_neg, table.thickness.value_m]`; top face (and this rule's `z_table_top` datum used throughout §12.7)
   at `z_table_top = -(adapter_plate_thickness.value_m or 0)` — exactly `0` when `bolted_directly_to_table`
   is `true`; box centre `position_m = [(x_near+x_far)/2, (y_neg+y_pos)/2, z_table_top -
   table.thickness.value_m/2]`.
4. **Obstacle boxes**: each `items[i]` becomes one axis-aligned box directly from its `min`/`max` readings,
   mapped to `base_link` with the same `x_from_front_face`/`y_from_centerline` formulas as the table and
   specimen, and `z_from_table_top` mapped via rule 3's `z_table_top` datum above.
5. **Provenance**: every derived scene object (table, specimen, each obstacle) written to
   `config/scene/scene.yaml` by MOT-10.3 gets `value_status: measured` and `source` naming this survey
   file's path and `sha256`.
6. **Refusal conditions** (any one refuses the whole conversion, exit 2/3 per §0.2, no partial scene
   write): a corner-rectangularity violation (rule 1); `specimen.resting_on_table` is `false`;
   `acm_observations.base_bolted_to_table` disagrees with `base_mounting.bolted_directly_to_table`; any
   derived obstacle box overlaps the robot's base keep-out (`base_keepout_m`, §8.3); fewer than 3
   `thickness_readings`; and any reading anywhere in the file with a `null` `value_m`, `instrument`,
   `resolution_m` or `uncertainty_1sigma_m` — **except** `base_mounting.adapter_plate_thickness`, which is
   exempt from this null-check (and from rules 2/3 above, where it is treated as `0`) when
   `bolted_directly_to_table` is `true`.

---

## 13. Cartesian crack-path plans (`crackvision.cartesian_plan/1`)

**Normative source:** `docs/adr/017-cartesian-crack-path-planning.md`. Nothing here edits §0–§12; any
amendment to earlier behaviour is stated here, the way §8 amends §6. The output of this section is
**untimed geometric joint-path data for MOT-07 and sim/visualisation. It is never executed directly.**
`MOT-05`/`execute_trajectory` only ever consumes a §11.2 `joint_trajectory` file, which MOT-07 produces
*from* a §13 plan by retiming; nothing in §13 is a valid `execute_trajectory --trajectory` input.

### 13.1 Inputs

| Input | Default path | Role |
|---|---|---|
| `paths3d` | `--paths3d PATH` (required) | §10 file: segments, waypoints, `execution_eligible` |
| scene | `config/scene/scene.yaml` (`--scene-config`) | collision scene, applied for every online check (§13.8) and the pre-planning surface check (§13.9) |
| end-effector | `config/robot/end_effector.yaml` | task frames (`tool_tip`), collision proxies |
| limits | `config/robot/b601_dm_limits.yaml` | inclusive position bound (§13.8); velocity/acceleration are out of scope here — untimed output has no timing to check (MOT-07/`G-LIMITS`, §11.7, checks those) |
| planning config | `config/motion/cartesian_planning.yaml` (`--planning-config`) | §13.2 |

Every one of these is read by path and sha256; both are recorded in the output file's `sources` (§13.11.1)
so a plan is fully traceable without re-resolving `--root`-relative defaults later.

**Start state.** The joint state from the `paths3d` file's `sources.capture_record` (§9.1
`robot.joint_names`/`robot.positions_rad`), unless `--start-joints J1 J2 J3 J4 J5 J6` (radians) overrides
it. **End state.** `cartesian_planning.yaml`'s `end_state`: `start` (default, return to the start state),
`home` (the SRDF all-zero configuration) or `none` (plan ends wherever the last segment left it).

### 13.2 `config/motion/cartesian_planning.yaml` (schema `crackvision.cartesian_planning_config/1`)

Loaded strictly (`ConfigError`/exit 2 on any unknown key, missing required key, or value outside the
stated range). `avoid_collisions` is **not** a config key — it is hardcoded `true` in code (ADR-017 §7),
so there is no key an operator could set to relax it. `frame` (`base_link`), `group` (`arm`) and `ik_link`
(`tool_tip` for every approach/trace/retract pose; `camera_link` for the §13.14 view phase) are likewise
fixed by this contract, not config keys.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `schema` | str | — | `"crackvision.cartesian_planning_config/1"` |
| `candidates.roll_offsets_deg` | list[float] | `[0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180]` | roll offsets about each pose's local `tool_tip` `+X` (ADR-017 §1); `directions` is always `{forward, reverse}` and is not configurable |
| `cartesian.max_step_m` | float | `0.001` | `/compute_cartesian_path` `max_step`; finer than `paths3d`'s own `waypoint_spacing_m` default (0.002 m, §10.4) so the Cartesian interpolation never coarsens the trace (CARTESIAN_PROBE.md consequence 3) |
| `cartesian.jump_threshold` | float | `0.0` | legacy scalar `jump_threshold`; `0.0` = disabled (ADR-017 §4) |
| `cartesian.prismatic_jump_threshold` | float | `0.0` | disabled (ADR-017 §4) |
| `cartesian.revolute_jump_threshold` | float | `0.0` | disabled (ADR-017 §4) — jump detection is done entirely by `recheck.max_joint_step_rad` below |
| `recheck.fk_tolerance_m` | float | `0.001` | §13.8, from the card body |
| `recheck.fk_tolerance_deg` | float | `1.0` | §13.8, from the card body |
| `recheck.max_joint_step_rad` | float | `0.2` | max per-joint step between consecutive output configurations (§13.8); ADR-017 §4 |
| `splitting.max_splits_per_segment` | int | `3` | §13.7 |
| `splitting.min_waypoints_per_remainder` | int | `3` | §13.7; matches `paths3d`'s own minimum kept-segment length (§10.3) |
| `end_state` | str | `"start"` | `start \| home \| none` (§13.1) |
| `surface_object_id` | str | `"specimen"` | the one scene object id checked, in §13.9's mismatch check, against the relaxed `surface_object_tolerance_m` bound instead of the full `clearance_m` bound (ADR-017 §5) |
| `surface_object_tolerance_m` | float | `0.002` | §13.9; how much closer than the as-designed standoff a `surface_object_id` trace position must be before it counts as a mismatch (ADR-017 §5) |
| `service_timeout_s` | float | `2.0` | per-service-call timeout for every ROS call this card makes |
| `view.view_distance_m` | float | `0.25` | must lie in `[0.20, 0.30]` (ADR-014 §8.2 band); loader refuses otherwise |
| `view.tilt_candidates_deg` | list[float] | `[0.0, 15.0]` | each value must lie in `[0, 15]` (ADR-014 §8.2 anti-normal bound); loader refuses otherwise |
| `view.yaw_candidates_deg` | list[float] | `[0.0, 90.0, 180.0, 270.0]` | azimuth candidates for the tilt axis (§13.14); 4 cardinal directions, matching `reachability.yaml`'s own `tilt_azimuth_samples: 4` |
| `view.fov_deg.horizontal`, `view.fov_deg.vertical` | float | `87.0`, `58.0` | nominal D405 colour-stream FOV (datasheet figures; 58° vertical already cited by ADR-014 §8.2/INTERFACES §8.2). Not yet independently re-verified in this repo — carried with the same `value_status`-style honesty as every other nominal config value here |
| `view.fov_margin_deg` | float | `5.0` | angular margin subtracted from each half-FOV before the corner check (§13.14) |
| `view.footprint_m` | `[float, float]` | `[0.20, 0.20]` | footprint (x, y) used whenever `--center` is given (§13.14 — takes precedence over any scene `specimen` object); matches `config/motion/specimen_placement.yaml`'s own `placement.footprint_m` |
| `view.surface_z_m` | float | `0.0` | specimen-top height used whenever `--center` is given (§13.14 — same precedence); matches `config/motion/specimen_placement.yaml`'s own `placement.surface_z_m` (table top, §12.1 datum) |
| `value_status` | str | `"nominal"` | file-wide, matching the `reachability.yaml`/`view_verify.yaml` convention |
| `sources` | map | — | one free-text rationale per block above, same convention as `reachability.yaml`'s `sources` |

### 13.3 Segment iteration order

Iterate `paths3d.components[]` in `order_index` order (§10.1/§10.2). Within a component: the `kind: main`
polyline first, then `kind: branch` polylines ordered by `branch_index` ascending. Within a polyline:
`segments[]` in file order. Each segment supplies, from its §10.5 waypoints: one `approach` pose, the
ordered `trace` poses, one `retract` pose, all `tool_tip` poses in `base_link` (`position_m`, `quat_xyzw`).

### 13.4 Candidate set and deterministic order

For a given segment (or sub-segment, §13.7) attempt, the candidate set is `direction ∈ {forward, reverse}
× roll_offset_deg ∈ candidates.roll_offsets_deg`. Enumeration and tie-break order (ADR-017 §1) is fixed:
roll magnitude ascending (`0, 30, 60, 90, 120, 150, 180`), positive sign before negative at equal magnitude
(skipped where only one sign exists, at `0` and `180`), `forward` before `reverse`. Every candidate in the
set is attempted (margin comparison, §13.6, needs every full completion); this order is both the
enumeration order and the tie-break rule.

Applying `roll_offset_deg`: for every pose in the segment (approach, every trace waypoint, retract),
`quat_candidate = quat_waypoint ⊗ [cos(θ/2), sin(θ/2), 0, 0]` (xyzw order, `θ = roll_offset_deg` in
radians) — an intrinsic rotation about the pose's own local `+X`. `position_m` is unchanged by this (a
rotation about `+X` never moves the origin), which is why §13.9's surface/scene check does not need to be
repeated per candidate. `reverse` reorders the pose sequence to `[retract, reverse(trace), approach]` and
swaps the `approach`/`retract` labels; every pose's `position_m`/base `quat_xyzw` (before the roll offset)
is unchanged by this — only traversal order and the two end labels flip.

### 13.5 Per-candidate procedure

Let "current config" be the joint configuration the plan is at when this segment's planning starts: the
start state (§13.1) for the first segment, or the last configuration of the previous segment's chosen
`retract` leg otherwise. All candidates for one segment (sub-)attempt are IK-seeded from the **same**
current config; the chosen candidate's final configuration becomes the current config for the next
segment.

For each candidate, in order (§13.4):

1. **IK** for the (candidate-rolled) approach pose, `ik_link: tool_tip`, seeded from the current config,
   `avoid_collisions: true`. Failure → candidate fails, reason `ik_failed`.
2. **Transit**: `/plan_kinematic_path` with `start_state` = the current config and a joint-constraint goal
   = step 1's IK solution (CARTESIAN_PROBE.md consequence 7 — this exact shape is confirmed to work).
   Failure → candidate fails, reason `transit_failed`.
3. **Cartesian approach**: `/compute_cartesian_path`, `link_name: tool_tip`, `waypoints = [first trace
   pose]`, starting from step 2's final configuration, `avoid_collisions: true`,
   `max_step`/`jump_threshold`* from `cartesian.*` (§13.2). `fraction < 1` → candidate fails, reason
   `cartesian_incomplete` (no partial-trace fallback for this leg — see ADR-017 §Alternatives).
4. **Cartesian trace**: same service, `waypoints` = every (candidate-rolled) trace pose in this
   (sub-)segment's order, starting from step 3's final configuration. `fraction < 1` → see §13.7 (partial
   trace); this is the only step the partial-trace fallback applies to.
5. **Cartesian retract**: same service, `waypoints = [retract pose]`, starting from step 4's final
   configuration. `fraction < 1` → candidate fails, reason `cartesian_incomplete`.

A candidate **fully completes the trace** iff steps 1–5 all succeed (step 4 at `fraction == 1.0`) and
every resulting configuration passes §13.8's independent re-check.

### 13.6 Selection rule

Among the candidates that fully complete (§13.5), select the one maximising the minimum joint margin to
`b601_dm_limits.yaml`'s inclusive bound over every configuration in its approach+trace+retract legs; ties
(compared after rounding to `1e-6` rad) are broken by the §13.4 order (smallest `|roll_offset_deg|`, then
`forward`). If no candidate fully completes, §13.7 applies.

### 13.7 Partial trace, splitting and re-entry

If no candidate's step 4 reaches `fraction == 1.0`: among all attempted candidates, take the one whose
step-4 Cartesian call reached the most trace waypoints, found by **FK matching** (ADR-017 §3) — never by
indexing or counting the returned trajectory directly (CARTESIAN_PROBE.md consequence 3: its point count
has no fixed relationship to the requested waypoint count), and not by the probe's own `fraction`-based
estimate either, for the reasons ADR-017 §3 gives:

> Walk the candidate's returned trace trajectory's configurations in order and the requested trace
> waypoints with a pointer `j` starting at 0. For each configuration (in order), if `/compute_fk` at
> `tool_tip` agrees with trace waypoint `j` within `recheck.fk_tolerance_m`/`_deg`, advance `j`. The
> last-reached index is `j - 1` (`-1` if no match ever advanced `j`).

Among candidates, prefer the largest last-reached index; ties broken as in §13.6, restricted to that
largest-index set.

- If the best last-reached index is `-1` (not even waypoint 0 reached by any candidate): the segment
  **fails** — reasons aggregate every candidate's own failure reason; no split is attempted (there is no
  reached point to retract from or re-enter from).
- Otherwise: build legs for that candidate up through the last-reached trace configuration, then add a
  synthesised **retract** leg: `/compute_cartesian_path` from the last-reached configuration to a pose
  offset `approach_retract_offset_m` (from the `paths3d` file's own `parameters`, §10.1) along the
  last-reached waypoint's own `normal_base`, same orientation as that waypoint — exactly §10.5 point 4's
  own approach/retract construction, applied to this new boundary. This retract leg is independently
  re-checked like any other.
- The **remainder** (every trace waypoint strictly after the last-reached one) becomes a new sub-segment.
  If it has fewer than `splitting.min_waypoints_per_remainder` waypoints, it is **not** attempted — it is
  reported uncovered (reason `too_short`) without spending one of the splits below. Otherwise it is
  retried with the full §13.4–13.6 procedure from scratch (its own synthesised approach, §10.5 point 4,
  offset from its first waypoint's `normal_base`), consuming one of `splitting.max_splits_per_segment`
  (default 3) retries for this *original* segment. If the remainder itself only partially completes, this
  same §13.7 procedure recurses (its own last-reached search, its own further remainder) until either full
  coverage, a `too_short` remainder, or `max_splits_per_segment` retries are exhausted — whichever comes
  first. A remainder still outstanding when retries are exhausted is left uncovered (no `too_short` reason
  in that case; it was long enough, planning for it just never fully succeeded — its own reasons stand).

Each attempt (the original segment and every retry) is one `segments[]` entry (§13.11.3), distinguished by
`segment_ref.sub_segment`: `null` for the original, `0, 1, 2, …` for successive retries in the order they
were spawned.

### 13.8 Independent re-check

Applied to **every** output configuration this card produces (every approach/trace/retract leg
configuration, and the final configuration of every transit leg whose joint goal was itself derived from
an IK-solved Cartesian target):

| Check | Rule |
|---|---|
| FK agreement | `/compute_fk` at `tool_tip` against the configuration's target pose, within `recheck.fk_tolerance_m` / `recheck.fk_tolerance_deg` (default 1 mm / 1°, from the card body) |
| Collision validity | `/check_state_validity`, production scene applied (§13.1 `scene`), explicit `RobotState` per query — mirrors §11.11's oracle topology |
| Joint limits | `b601_dm_limits.yaml`'s `[lower, upper]` bound, **inclusive**, never narrowed — exactly §11.6's hard-bound rule (this is a geometric plan; the §11.6 position-*margin* policy, which is about approach speed into the bound, does not apply — there is no timing here) |
| Joint step | max over consecutive configuration pairs in the leg of `max_i abs(q_i[t+1] - q_i[t])`, compared against `recheck.max_joint_step_rad` |

**Transit legs whose joint goal is a pure joint target with no corresponding Cartesian pose** — the
trailing `end_state: home`/`end_state: start` leg (§13.1), and the §13.14 view phase's transit from `home`
— are exempt from the FK-agreement check (there is no pose to compare against) but still subject to the
collision, joint-limit and joint-step checks.

A failed re-check fails the leg carrying the failing configuration, with the corresponding reason
(`fk_mismatch`, `collision`, `joint_limit` or `joint_jump`, §13.10 below); it is never ignored, and a
candidate carrying a failing leg is not eligible for selection (§13.6) — the search moves to the next
candidate exactly as it would for an `ik_failed`/`transit_failed` candidate.

**Classifying a `fraction < 1` shortfall** (approach/trace/retract step, CARTESIAN_PROBE.md consequence 5: the
service gives no cause). From the last-reached configuration (§13.7's FK match, or the leg's own start
configuration if none reached): (a) IK the next unreached waypoint, seeded from that configuration; IK
failure leaves the reason the generic `cartesian_incomplete`; (b) IK success → `/check_state_validity` on
that solution with the production scene; invalid → reason `collision`; (c) valid →  compare the per-joint
step between the last-reached configuration and this IK solution against `recheck.max_joint_step_rad`;
over → reason `joint_jump`; (d) otherwise the reason stays the generic `cartesian_incomplete`. This is a
bounded, best-effort attribution (one extra IK + validity call), not a cause MoveIt itself reports.

### 13.9 Surface/scene consistency check (pre-planning, pure)

Before any candidate is attempted for a segment (or sub-segment), and independent of direction/roll (§13.4
— roll never moves `position_m`): for every trace waypoint's `position_m`, compute the signed distance to
every scene object in `scene.yaml` — transform the point into the object's local frame via its inverse
pose, then the exact axis-aligned box distance there (negative inside, exact Euclidean distance outside).
For every scene object **except** the one named by `surface_object_id` (default `"specimen"`, ADR-017
§5), a mismatch is flagged if the distance is less than that waypoint's own `clearance_m` (§10.2; equal to
`trace_clearance_m`). For `surface_object_id` itself, the same comparison uses the relaxed bound
`clearance_m − surface_object_tolerance_m` (§13.2, default `0.002`) instead of the full `clearance_m` —
every trace waypoint sits, by construction, exactly `clearance_m` from this object (§10.5 point 2), so the
full bound would flag every correct waypoint, but a position that is more than `surface_object_tolerance_m`
closer than that still gets caught (ADR-017 §5). If any object flags a mismatch, the segment is reported
`surface_scene_mismatch` and is **not planned at all**: zero candidates are tried, `segments[]` records
`candidates_tried: 0`, `chosen: null`, `trace_waypoints: {total: N, planned: 0}`.

### 13.10 Reason vocabulary

| Reason | Meaning |
|---|---|
| `ik_failed` | §13.5 step 1: no IK solution for the (rolled) approach pose from the current seed |
| `transit_failed` | §13.5 step 2: `/plan_kinematic_path` found no joint-space path to the IK goal |
| `cartesian_incomplete` | a Cartesian call (§13.5 steps 3–5) returned `fraction < 1`, cause not attributable to `collision` or `joint_jump` by §13.8's classification |
| `collision` | §13.8 classification, or a direct `/check_state_validity` failure during the independent re-check |
| `joint_jump` | §13.8 classification, or the independent re-check's `max_joint_step_rad` exceeded between consecutive configurations |
| `joint_limit` | independent re-check: a configuration violates `b601_dm_limits.yaml`'s inclusive bound |
| `fk_mismatch` | independent re-check: `/compute_fk` disagrees with the target pose beyond tolerance |
| `surface_scene_mismatch` | §13.9: a trace position is closer than its clearance to a non-surface scene object, or closer than `clearance_m − surface_object_tolerance_m` to the surface object itself |
| `too_short` | §13.7: a split remainder has fewer than `splitting.min_waypoints_per_remainder` waypoints and was never attempted |

### 13.11 Plan file `data/plans/{case}_cartesian_plan.json`

Untimed. `MOT-07` retimes all `legs[].positions[]`, concatenated in `leg_index` order, into one
§11.2 `joint_trajectory` file; MoveIt's own per-call timestamps are discarded entirely — they belong to
the planning calls, not to the plan.

#### 13.11.1 Top level

| Field | Type | Meaning |
|---|---|---|
| `schema` | str | `"crackvision.cartesian_plan/1"` |
| `case_id` | str | §1 case id, copied from the input `paths3d` file |
| `created_utc` | str | ISO-8601 UTC |
| `producer` | str | `"plan_crack_path"` |
| `planning_frame` | str | `"base_link"` |
| `joint_names` | list[str] | `["joint1", ..., "joint6"]` |
| `sources.paths3d` | `{path, sha256, execution_eligible}` | exactly the §11.2 `source.paths3d` shape, so `MOT-07`/`G-ELIGIBLE` can read it unchanged |
| `sources.scene`, `sources.end_effector`, `sources.limits`, `sources.config` | `{path, sha256}` each | every other §13.1 input |
| `parameters` | object | every field of §13.2's loaded config (echoed, so the plan is reproducible without re-reading `cartesian_planning.yaml`) |
| `start_state` | `{joint_names, positions_rad, source: "capture_record" \| "start_joints_override"}` | §13.1 |
| `end_state` | `{mode: start\|home\|none, positions_rad: list[float]\|null}` | §13.1; `positions_rad` is `null` for `none` |
| `legs[]` | list | §13.11.2 |
| `segments[]` | list | §13.11.3 |
| `coverage` | object | §13.12 |
| `caveats[]` | list[str] | §13.13 |

#### 13.11.2 `legs[]`

| Field | Type | Meaning |
|---|---|---|
| `leg_index` | int | 0-based, final concatenated order |
| `kind` | str | `transit \| approach \| trace \| retract` |
| `segment_ref` | object\|null | `{component_id, polyline_kind, branch_index, segment_index, sub_segment}`; `null` only for the trailing `end_state` transit leg, which belongs to no segment |
| `direction` | str\|null | `forward \| reverse`; `null` for the trailing `end_state` transit leg |
| `roll_offset_rad` | float\|null | the owning candidate's roll offset; `null` for the trailing `end_state` transit leg |
| `positions[][6]` | list[list[float]] | joint configurations in travel order, `joint1..6` |
| `waypoint_index_map` | list[int\|null] | per `positions[]` entry: the index into this leg's own target-pose list (the single approach/retract target = `0`; the segment's trace waypoint index for a `trace` leg) that configuration FK-matches (§13.7's matching rule), or `null` for an unmatched intermediate/resampled configuration. All `null` for a transit leg whose goal has no Cartesian pose (§13.8) |
| `recheck.max_pos_err_m` | float | max FK position error over this leg's FK-checked configurations |
| `recheck.max_ang_err_rad` | float | max FK angular error, same set |
| `recheck.all_valid` | bool | every configuration passed `/check_state_validity` |
| `recheck.within_limits` | bool | every configuration's positions satisfy the inclusive `b601_dm_limits.yaml` bound |
| `recheck.max_joint_step_rad` | float | max per-joint step between consecutive configurations in this leg |

A leg's recheck **passes** iff `max_pos_err_m ≤ parameters.recheck.fk_tolerance_m`,
`max_ang_err_rad ≤ parameters.recheck.fk_tolerance_deg` (converted to radians), `all_valid`,
`within_limits`, and `max_joint_step_rad ≤ parameters.recheck.max_joint_step_rad`. Only legs belonging to
a selected (§13.6) or kept-partial (§13.7) candidate ever appear here — a candidate carrying a failing leg
is never committed to `legs[]`.

#### 13.11.3 `segments[]`

One entry per planning **attempt** — the original segment and every §13.7 retry each get their own entry,
distinguished by `segment_ref.sub_segment` (`null` for the original, `0, 1, 2, …` for retries, in spawn
order).

| Field | Type | Meaning |
|---|---|---|
| `segment_ref` | object | `{component_id, polyline_kind, branch_index, segment_index, sub_segment}` |
| `status` | str | `full` (`trace_waypoints.planned == .total`) \| `partial` (`0 < planned < total`) \| `failed` (`planned == 0`, candidates were tried) \| `surface_scene_mismatch` (§13.9, never attempted) |
| `reasons[]` | list[str] | every §13.10 reason that applies to this attempt, deduplicated |
| `candidates_tried` | int | count of (direction, roll) candidates attempted for this attempt (`0` for `surface_scene_mismatch`) |
| `chosen` | `{direction, roll_offset_rad}`\|null | the selected/kept-partial candidate; `null` if `status` is `failed` or `surface_scene_mismatch` |
| `trace_waypoints.total` | int | trace waypoints this attempt was responsible for (the full segment for `sub_segment: null`; the still-outstanding remainder for a retry) |
| `trace_waypoints.planned` | int | how many of those this attempt actually covered |
| `trace_length_m.total`, `.planned` | float | 3D arc length (m), same scope as the waypoint counts above |

### 13.12 Coverage metrics

| Field | Type | Meaning |
|---|---|---|
| `trace_waypoints_total` | int | sum, over distinct original segments (one term each — ignoring retries), of that segment's `sub_segment: null` entry's own `trace_waypoints.total` |
| `trace_waypoints_planned` | int | sum of `trace_waypoints.planned` over **every** `segments[]` entry (original + every retry); sound without double-counting because a retry only ever attempts its parent's still-unreached remainder (§13.7), so the covered sets are disjoint |
| `length_m_total`, `length_m_planned` | float | identical aggregation, using `trace_length_m` |
| `fraction_by_length` | float | `length_m_planned / length_m_total`, or `1.0` if `length_m_total == 0` |
| `complete` | bool | `trace_waypoints_planned == trace_waypoints_total` |
| `upstream` | `{points, valid, invalid_by_reason}` | copied verbatim from the input `paths3d` file's `counts` (§10.1) |

### 13.13 Caveats

`caveats[]` always starts, verbatim, with the §7.3 nominal disclaimer: *"Nominal recommendation only; the
operator places the specimen and MOT-10 measures and supersedes this file with the real, as-placed pose."*
It then states: the task frame (`tool_tip`) and collision model this plan rests on (mirroring §8.3's own
pattern); that this file is **untimed geometric output, never executable directly** (§13 preamble); and,
when `sources.paths3d.execution_eligible` is `false`, that the plan was built from an execution-ineligible
`paths3d` file (valid for sim/visualisation only, per §10.6).

### 13.14 View phase: `crackvision.view_plan/1`

Default path `data/plans/view_plan.json` (`--out` overrides). Workcell-level, not per-case (no `case_id`):
positions the wrist camera to view the specimen, independent of any particular crack trace.

**Geometry.** `--center X Y`, when given, always takes precedence over the scene's `specimen` object
(an explicit operator value overrides the scene default even if a `specimen` object is also present):
centre `(cx, cy) = (X, Y)`, footprint `= view.footprint_m` and specimen-top height `z_top =
view.surface_z_m` (§13.2) — both config-sourced, never invented from FOV defaults. Without `--center`,
centre `(cx, cy)` and footprint come from the scene's `specimen` object (`position_m[0:2]`,
`dimensions_m[0:2]`), with `z_top = position_m[2] + dimensions_m[2]/2`. If `--center` was not given and
the scene has no `specimen` object, `plan_view` refuses (exit 3, §13.15) rather than inventing a
footprint or surface height. For each `(tilt_deg, yaw_deg)` candidate, in `view.tilt_candidates_deg`
ascending × `view.yaw_candidates_deg` ascending order:

- `camera_link` position: `(cx, cy, z_top + view.view_distance_m)` — always directly above the centre;
  tilt/yaw change orientation only, matching §8.3's own `--emit-view-config` description.
- Let `d0 = [0, 0, -1]` (world straight-down) and `a = [cos(yaw), sin(yaw), 0]` (horizontal azimuth axis).
  Optical axis `n_opt` = `d0` rotated by `tilt_deg` about `a` (right-hand rule). `camera_link +X := n_opt`,
  `+Z := a` (already ⟂ `n_opt` since `a ⟂ d0` and rotation about `a` preserves that), `+Y := +Z × +X`
  (same right-handed construction as §10.5 point 3).
- **FOV check.** For each of the specimen footprint's 4 corners at `z_top`: let `v` = corner − camera
  position. Horizontal angle `= atan2(v·(+Y), v·(+X))`; vertical angle `= atan2(v·(+Z), v·(+X))`. Passes
  iff `|horizontal| ≤ view.fov_deg.horizontal/2 − view.fov_margin_deg` and `|vertical| ≤
  view.fov_deg.vertical/2 − view.fov_margin_deg`, for all 4 corners.
- **Anti-normal check.** The angle between `n_opt` and `[0,0,-1]` (the specimen top face's anti-normal) is
  exactly `tilt_deg`, which `view.tilt_candidates_deg`'s loader-enforced `≤ 15°` bound (ADR-014 §8.2)
  already guarantees.
- **Reachability.** `/compute_ik` for `camera_link` at this pose; if it succeeds, `/plan_kinematic_path`
  with `start_state` = the SRDF all-zero `home` and a joint-constraint goal = that IK solution
  (CARTESIAN_PROBE.md consequence 7). Both legs pass the independent re-check (§13.8), FK-exempt only for `home`'s
  own trivial start.

**Selection.** The first candidate, in the stated order, that passes the FOV check, the reachability step
and the independent re-check. None passing is a refusal (exit 3, §13.15).

#### 13.14.1 Fields

| Field | Type | Meaning |
|---|---|---|
| `schema` | str | `"crackvision.view_plan/1"` |
| `created_utc`, `producer` | str | `"plan_view"` |
| `planning_frame` | str | `"base_link"` |
| `sources.scene`, `.end_effector`, `.limits`, `.config` | `{path, sha256}` | §13.1 inputs (no `paths3d`) |
| `parameters` | object | the `view.*` block of the loaded config, echoed |
| `center_source` | str | `"scene.specimen"` \| `"--center"`; `"--center"` whenever the flag was given, even if a scene `specimen` also exists (§13.14 precedence) |
| `center_xy_m`, `surface_z_m`, `footprint_m` | — | as resolved above; on the `"--center"` path, `surface_z_m`/`footprint_m` are `view.surface_z_m`/`view.footprint_m` (§13.2) |
| `candidates[]` | list | one entry per `(tilt_deg, yaw_deg)` tried: `{tilt_deg, yaw_deg, camera_pose: {position_m, quat_xyzw}, fov_check: {passed, horizontal_margins_deg[4], vertical_margins_deg[4]}, reachable, recheck_passed}` — the FOV/tilt evidence the card requires |
| `chosen` | `{tilt_deg, yaw_deg}`\|null | the selected candidate; `null` on refusal |
| `transit_leg` | object\|null | one §13.11.2-shaped leg (`kind: transit`, `segment_ref: null`, `direction: null`, `roll_offset_rad: null`), home → the chosen pose's IK solution; `null` on refusal |
| `caveats[]` | list[str] | same §13.13 construction (nominal disclaimer first, then the view task frame `camera_link` and collision model) |

### 13.15 CLIs

Both are `ros2 run crackvision_motion` console-script entry points of the same package as
`execute_trajectory` (§11.10), invoked the same way (exception to §0.1):

```bash
source scripts/ros/env_ros.sh
set +u; source ros2_ws/install/setup.bash
ros2 run crackvision_motion plan_crack_path --paths3d PATH [--out PATH] [--start-joints J1..J6]
    [--scene-config PATH] [--planning-config PATH] [--require-full-coverage] [§0.3 flags]
ros2 run crackvision_motion plan_view [--out PATH] [--center X Y] [--scene-config PATH]
    [--planning-config PATH] [§0.3 flags]
```

`--out` defaults to `data/plans/{case}_cartesian_plan.json` / `data/plans/view_plan.json`.
`--planning-config` defaults to `config/motion/cartesian_planning.yaml`.

Exit codes (§0.2):

| Code | `plan_crack_path` | `plan_view` |
|---|---|---|
| `0` | plan written (partial coverage allowed and reported in `coverage`, unless `--require-full-coverage`) | plan written |
| `1` | runtime failure (ROS/service exception) | same |
| `2` | usage/config error (incl. an unknown `cartesian_planning.yaml` key, or a `view.*` value outside its stated range) | same |
| `3` | refusal: invalid/unreadable input, a sha mismatch against a resolved input, a required service missing, zero trace waypoints planned across the whole file, or (`--require-full-coverage` only) `coverage.complete == false` | refusal: invalid input, a required service missing, or no candidate selected (§13.14 Selection) |

`--dry-run` (§0.3) validates every input (paths/shas/schema) and prints the segment/candidate plan — which
segments, in what order, with how many candidates each would try — without calling any ROS service,
writing no file, exit 0 regardless.

### 13.16 Default numeric values

| Parameter | Default | Rationale |
|---|---|---|
| `candidates.roll_offsets_deg` | 12 values, `0`..`180` | the card body's own default set; a uniform (not ramped) offset, ADR-017 §1 |
| `cartesian.max_step_m` | 0.001 m | finer than `paths3d`'s own 0.002 m waypoint spacing (§10.4); CARTESIAN_PROBE.md consequence 3 |
| `cartesian.*jump_threshold*` | `0.0` (disabled) | the probe confirmed these fields exist but never calibrated their numeric effect (fact 5); jump detection is `recheck.max_joint_step_rad` instead (ADR-017 §4) |
| `recheck.fk_tolerance_m` / `_deg` | 0.001 m / 1.0° | from the card body |
| `recheck.max_joint_step_rad` | 0.2 rad (~11.5°) | above ordinary per-point motion at `max_step_m` away from a singularity, below the scale of a true wrist-singularity discontinuity (CARTESIAN_PROBE.md consequence 1) |
| `splitting.max_splits_per_segment` | 3 | the card body's own default |
| `splitting.min_waypoints_per_remainder` | 3 | matches `paths3d`'s own minimum kept-segment length (§10.3) |
| `view.view_distance_m` | 0.25 m | ADR-014 §8.2 (band 0.20–0.30 m) |
| `view.tilt_candidates_deg` | `[0, 15]` | ADR-014 §8.2's anti-normal band edges |
| `view.yaw_candidates_deg` | `[0, 90, 180, 270]` | matches `reachability.yaml`'s own `tilt_azimuth_samples: 4` |
| `view.fov_deg` | `87° x 58°` (H x V) | D405 public datasheet (58° vertical already cited, ADR-014 §8.2); not yet independently re-verified in this repo |
| `view.fov_margin_deg` | 5.0° | nominal engineering cushion, not measured |

### 13.17 Future work

Segment ordering (§13.3) is not optimised for travel distance or re-tries — it is simply the input file's
own `order_index`/`branch_index`/file order. A future card may re-sequence segments to minimise transit
distance between them; this section does not attempt that.

---

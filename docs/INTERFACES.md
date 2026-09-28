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

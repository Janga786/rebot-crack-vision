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
